#include "control_thread.h"
#include "io_adapter.h"
// #include "interface.h" // 引入以使用 CmdIdx (如果有的话)
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <thread> // 必须包含这个头文件
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>

// 辅助函数：计算 ENU 到 Body Frame 的旋转矩阵 R_N_to_B
// 假设飞控姿态遵循标准的 Z-Y-X (Yaw-Pitch-Roll) 欧拉角约定
Eigen::Matrix3d get_R_N_to_B(double roll, double pitch, double yaw) {
    // R_B_to_N (Body to Navigation) = Rz(yaw) * Ry(pitch) * Rx(roll)
    // R_N_to_B (Navigation to Body) = R_B_to_N.transpose()

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    // 计算 R_B_to_N 矩阵
    Eigen::Matrix3d R_B_to_N;
    R_B_to_N(0, 0) = cy*cp;
    R_B_to_N(0, 1) = cy*sp*sr - sy*cr;
    R_B_to_N(0, 2) = cy*sp*cr + sy*sr;
    R_B_to_N(1, 0) = sy*cp;
    R_B_to_N(1, 1) = sy*sp*sr + cy*cr;
    R_B_to_N(1, 2) = sy*sp*cr - cy*sr;
    R_B_to_N(2, 0) = -sp;
    R_B_to_N(2, 1) = cp*sr;
    R_B_to_N(2, 2) = cp*cr;

    // 返回其转置 R_N_to_B
    return R_B_to_N.transpose();
}

// 辅助函数：角度归一化到 [-PI, PI]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// 辅助函数：将状态转换为字符串用于打印
std::string StateToString(FlightState state) {
    switch(state) {
        case FlightState::IDLE: return "IDLE";
        case FlightState::PREPARING: return "PREPARING";
        case FlightState::TAKEOFF: return "TAKEOFF";
        case FlightState::EXECUTING: return "EXECUTING";
        case FlightState::LANDING: return "LANDING";
        case FlightState::FINISHED: return "FINISHED";
        default: return "UNKNOWN";
    }
}

void ControlLoop(std::shared_ptr<IOAdapter> io,
                 std::shared_ptr<SharedContext> ctx,
                 Eigen::Vector3d goal_pos) {

    // 状态机初始化
    FlightState current_state = FlightState::IDLE;

    // 轨迹相关变量
    std::vector<obj_planner::PathPoint> local_trajectory;
    auto trajectory_start_time = std::chrono::steady_clock::now();
    const double TRAJECTORY_TIME_STEP = 0.01;

    // === 新增：本体坐标系控制变量 ===
    bool use_body_control = false;  // 是否使用本体坐标系控制
    Eigen::Vector3d target_vel_body = Eigen::Vector3d::Zero();
    double target_yaw_error = 0.0;

    // 起飞降落相关参数
    const double LAND_CHECK_HEIGHT = 0.3; // 判定已落地的地面高度 (米)

    Eigen::Vector3d initial_takeoff_pos; // 记录起飞点

    // 自动解锁相关参数
    int auto_arm_counter = 0;
    const int AUTO_ARM_DELAY = 200; // 2秒延时 (10ms * 200)

    // [新增] CSV 日志记录器初始化
    std::ofstream log_file;

    // 创建日志目录
    std::string log_dir = "flight_logs";

    // 创建目录 (如果不存在)
    struct stat st = {0};
    if (stat(log_dir.c_str(), &st) == -1) {
        mkdir(log_dir.c_str(), 0755);
        std::cout << "📁 Created log directory: " << log_dir << std::endl;
    }

    auto now_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << log_dir << "/flight_cmd_log_" << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S") << ".csv";
    std::string log_file_path = ss.str();

    log_file.open(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "❌ Failed to open log file: " << log_file_path << std::endl;
    } else {
        // 写入 CSV 表头
        log_file << "timestamp,state,x_speed,y_speed,z_speed,yaw_rate,fly_status,fail_flag,pos_x,pos_y,pos_z,yaw_curr" << std::endl;
        std::cout << "📝 Logging flight commands to: " << log_file_path << std::endl;
    }
    std::cout << "🚀 Control Thread Started with FSM (Auto-Arm Mode)." << std::endl;

    while (ctx->program_running) {
        auto loop_start = std::chrono::steady_clock::now();

        // --- 1. 【新增】惯导超时熔断检查 ---
        bool ins_healthy = io->IsINSActive();
        if (!ins_healthy) {
            static int ins_fatal_warn_count = 0;
            if (ins_fatal_warn_count++ % 10 == 0) { // 每10次打印一次，避免刷屏
                std::cout << "🚨 [FATAL] INS TIMEOUT! CRITICAL FAILURE!" << std::endl;
                std::cout << "🛑 EMERGENCY STOP: Sending CMD_ENGINE_STOP!" << std::endl;
            }

            // 【紧急策略】立即发送停车指令，防止飞丢
            io->SendCommand(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 0.0, CMD_ENGINE_STOP);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // --- 2. 获取所有传感器数据 ---
        PlanningInertialData_t ins_data = io->GetInertialData();
        // RCData_t rc_data = io->GetRCStatus(); // 注释掉遥控器检查，实现自动解锁

        // 基础数据处理
        if (!ins_data.position_valid) {
            // 如果惯导无效，无论在哪个状态都应警报或等待（除了 IDLE）
            if (current_state != FlightState::IDLE) {
                std::cout << "⚠️ INS Invalid during flight! Emergency!" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        Eigen::Vector3d current_pos(ins_data.position_x, ins_data.position_y, ins_data.position_z);
        Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();
        if (ins_data.velocity_valid) current_vel << ins_data.velocity_x, ins_data.velocity_y, ins_data.velocity_z;

        // === 新增：获取当前姿态数据 ===
        double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;
        if (ins_data.attitude_valid) {
            current_roll = ins_data.roll;
            current_pitch = ins_data.pitch;
            current_yaw = ins_data.yaw;
        }

        // 更新共享数据给规划线程
        {
            std::lock_guard<std::mutex> lock(ctx->mtx);
            ctx->robot_pos = current_pos;
            ctx->robot_vel = current_vel;
            // === 新增：更新姿态数据 ===
            ctx->robot_roll = current_roll;
            ctx->robot_pitch = current_pitch;
            ctx->robot_yaw = current_yaw;
            // === 新增：同步飞行状态给规划线程 ===
            ctx->flight_state = current_state;
        }

        // --- 2. 状态机逻辑 ---

        // 默认发送的数据
        Eigen::Vector3d target_pos = current_pos;
        Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d target_acc = Eigen::Vector3d::Zero();
        double target_yaw = current_yaw;
        int command_id = CMD_PRE_CONTROL; // 默认预控指令

        switch (current_state) {
            // ----------------------------------------------------
            case FlightState::IDLE:
                command_id = CMD_PRE_CONTROL;
                // 【修改 1】：只检查惯导数据，忽略遥控器
                if (ins_data.position_valid) {
                    std::cout << "✅ INS Valid. Auto-switching to PREPARING." << std::endl;
                    current_state = FlightState::PREPARING;
                } else {
                    static int log_cnt = 0;
                    if (log_cnt++ % 100 == 0) std::cout << "⏳ Waiting for INS..." << std::endl;
                }
                break;

            // ----------------------------------------------------
            case FlightState::PREPARING:
                command_id = CMD_PRE_CONTROL;

                // 【修改 2】：跳过遥控器解锁检查，改为自动延时解锁
                if (auto_arm_counter++ > AUTO_ARM_DELAY) {
                    std::cout << "🔓 Auto Armed! Switching to TAKEOFF." << std::endl;

                    // 记录起飞点
                    initial_takeoff_pos = current_pos;
                    std::cout << "📍 Takeoff Home set at: " << initial_takeoff_pos.transpose() << std::endl;

                    current_state = FlightState::TAKEOFF;
                } else {
                    if (auto_arm_counter % 50 == 0)
                        std::cout << "⏳ Auto arming in " << (AUTO_ARM_DELAY - auto_arm_counter) * 0.01 << "s..." << std::endl;
                }
                break;

            // ----------------------------------------------------
            case FlightState::TAKEOFF: {
                // 1. --- 发送起飞指令 (Step Input) ---
                // 直接设定目标起飞位置
                // 飞控会自行处理从 地面 -> 目标高度 的平滑过程
                Eigen::Vector3d takeoff_target = initial_takeoff_pos;
                takeoff_target.z() += ctx->takeoff_height;

                // 速度设为 0，表示我们只给位置设定点，让飞控自己决定速度
                target_pos = takeoff_target;
                target_vel = Eigen::Vector3d::Zero();
                target_acc = Eigen::Vector3d::Zero();

                // 发送自动起飞指令
                command_id = CMD_AUTO_TAKEOFF;

                // 2. --- 判断是否完成起飞 ---
                // 获取惯导/里程计反馈的当前真实高度
                double height_error = std::abs(target_pos.z() - current_pos.z());

                // 设定一个判断阈值，例如 0.15 米
                if (height_error < 0.15) {
                    // 如果是仿真模式，可能瞬间就到了，加一个简单的日志
                    // 如果是真机，必须等到高度真的接近了
                    if (ctx->is_simulation) {
                        // 仿真可能需要稍微延时一下确保状态稳定，或者直接切换
                    }

                    double relative_height = current_pos.z() - initial_takeoff_pos.z();
                    std::cout << "[FSM] Takeoff altitude reached (Err: " << height_error
                              << "m, Rel: " << relative_height << "m). Switching to EXECUTING." << std::endl;

                    // 准备切入自主轨迹，重置时间
                    trajectory_start_time = std::chrono::steady_clock::now();
                    current_state = FlightState::EXECUTING;
                }

                // 【修改】移除遥控器安全检查，因为现在是自动解锁模式
                // 在调试模式下，我们信任系统状态，不需要检查遥控器
                break;
            }

            // ----------------------------------------------------
            case FlightState::EXECUTING:
                command_id = CMD_PROGRAM_CONTROL; // 发送程控指令

                // === 本体坐标系轨迹跟踪逻辑开始 ===
                {
                    // 1. 检查并切换新轨迹
                    bool has_new_traj = false;
                    {
                        std::lock_guard<std::mutex> lock(ctx->mtx);
                        if (ctx->has_new_trajectory) {
                            auto now = std::chrono::steady_clock::now(); // 获取当前时间

                            // 只有当当前时间超过或等于有效时间时才执行
                            if (now >= ctx->trajectory_valid_time) {
                                local_trajectory = ctx->new_trajectory;
                                ctx->has_new_trajectory = false;

                                // === 【修复代码开始】 ===
                                // 计算时间滞后量
                                double lag = std::chrono::duration_cast<std::chrono::duration<double>>(now - ctx->trajectory_valid_time).count();

                                // 如果滞后超过 30ms (3个控制周期)，且我们处于刚起飞或低速状态
                                // 强制将轨迹起始时间对齐到"现在"，防止跳过加速段
                                if (lag > 0.03) {
                                    trajectory_start_time = now;
                                    std::cout << "⚠️ Trajectory lag detected (" << lag << "s). Re-aligning start time to NOW." << std::endl;
                                } else {
                                    trajectory_start_time = ctx->trajectory_valid_time;
                                }
                                // === 【修复代码结束】 ===

                                has_new_traj = true;

                                // 🔴【修改点】：暂时禁用本体控制，强制使用 ENU 控制
                                // use_body_control = true;  // <--- 原代码
                                use_body_control = false;    // <--- 修改后：强制使用 ENU

                                // std::cout << "⚠️ DEBUG MODE: Body Control DISABLED. Using ENU." << std::endl;
                            }
                        }
                    }

                    // 2. 采样轨迹并执行坐标系转换
                    if (!local_trajectory.empty()) {
                        auto now = std::chrono::steady_clock::now();
                        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - trajectory_start_time).count();
                        double t = elapsed / TRAJECTORY_TIME_STEP;
                        int idx = (int)t;
                        double alpha = t - idx;

                        Eigen::Vector3d target_pos_enu, target_vel_enu, target_acc_enu;
                        double target_yaw_abs;

                        if (idx < static_cast<int>(local_trajectory.size()) - 1) {
                            const auto& p0 = local_trajectory[idx];
                            const auto& p1 = local_trajectory[idx + 1];

                            // 插值计算 ENU 坐标系下的轨迹数据
                            target_pos_enu = (1-alpha)*Eigen::Vector3d(p0.x,p0.y,p0.z) + alpha*Eigen::Vector3d(p1.x,p1.y,p1.z);
                            target_vel_enu = (1-alpha)*Eigen::Vector3d(p0.vx,p0.vy,p0.vz) + alpha*Eigen::Vector3d(p1.vx,p1.vy,p1.vz);
                            target_acc_enu = (1-alpha)*Eigen::Vector3d(p0.ax,p0.ay,p0.az) + alpha*Eigen::Vector3d(p1.ax,p1.ay,p1.az);
                            target_yaw_abs = (1-alpha)*p0.yaw + alpha*p1.yaw;
                        } else {
                            // 轨迹结束，悬停在终点
                            const auto& last = local_trajectory.back();
                            target_pos_enu = Eigen::Vector3d(last.x, last.y, last.z);
                            target_vel_enu.setZero(); target_acc_enu.setZero();
                            target_yaw_abs = last.yaw;
                        }

                        // === 📢 核心转换：ENU → Body Frame ===

                        // a. 速度指令：ENU → Body Frame 转换
                        // 📢 使用当前的实际姿态进行旋转，这是转换的关键
                        Eigen::Matrix3d R_N_to_B = get_R_N_to_B(current_roll, current_pitch, current_yaw);

                        // 执行旋转: V_Body = R_N_to_B * V_ENU
                        target_vel_body = R_N_to_B * target_vel_enu;

                        // b. 航向角误差指令：绝对航向 → 误差
                        // 📢 航向角误差 Δψ = ψ_target - ψ_current
                        target_yaw_error = target_yaw_abs - current_yaw;

                        // 归一化到 [-π, π]，防止出现 350° - 10° = 340° 这种错误
                        target_yaw_error = normalizeAngle(target_yaw_error);

                        // 保持原有的 ENU 指令作为备用
                        target_pos = target_pos_enu;
                        target_vel = target_vel_enu;
                        target_acc = target_acc_enu;
                        target_yaw = target_yaw_abs;

                    } else {
                        // 无轨迹，悬停在当前位置
                        static Eigen::Vector3d hover_pos = current_pos;
                        if (target_pos == current_pos) { // 第一次进入
                            hover_pos = current_pos;
                        }
                        target_pos = hover_pos;
                        target_vel_body.setZero();  // Body 坐标系速度归零
                        target_yaw_error = 0.0;      // 航向误差归零
                        use_body_control = false;    // 禁用本体坐标系控制
                    }
                }
                // === 本体坐标系轨迹跟踪逻辑结束 ===

                // 【修改】移除飞行过程中的遥控器检查
                // 在自动解锁模式下，我们通过其他方式确保安全

                // 转换条件：到达终点 (假设外部或距离判断)
                {
                    double dist_to_goal = (current_pos - goal_pos).norm();
                    if (dist_to_goal < 0.5) {
                        std::cout << "🏁 Goal Reached. Switching to LANDING." << std::endl;
                        current_state = FlightState::LANDING;
                    }
                }
                break;

            // ----------------------------------------------------
            case FlightState::LANDING:
                command_id = CMD_AUTO_LAND; // 发送自动降落指令
                // 降落时通常不需要发送具体的目标位置，飞控会处理

                // 【修改】只检查高度条件，移除遥控器检查
                if (current_pos.z() < LAND_CHECK_HEIGHT) {
                    std::cout << "🛬 Touchdown detected. Mission Finished." << std::endl;
                    current_state = FlightState::FINISHED;
                }
                break;

            // ----------------------------------------------------
            case FlightState::FINISHED:
                command_id = CMD_ENGINE_STOP; // 停机

                std::cout << "✅ Mission Complete. Shutting down..." << std::endl;

                // 【修改 3】：降落完成后，通知所有线程退出
                {
                    std::lock_guard<std::mutex> lock(ctx->mtx);
                    ctx->program_running = false; // 这会让 control_thread 和 planning_thread 的 while 循环都结束
                }

                // 给一点时间让指令发出去，然后跳出循环
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return; // 直接退出控制线程函数
                break;
        }

        // ---------------------------------------------------------
        // 核心修改区域：准备输出结构体和计算控制量
        // ---------------------------------------------------------

        // 准备输出结构体
        AutoFlightCmd output_cmd;
        output_cmd.timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        if (current_state == FlightState::EXECUTING) {
            // 1. 轨迹更新逻辑 (保留原有逻辑，添加状态同步)
            {
                std::lock_guard<std::mutex> lock(ctx->mtx);
                if (ctx->has_new_trajectory && std::chrono::steady_clock::now() >= ctx->trajectory_valid_time) {
                    local_trajectory = ctx->new_trajectory;
                    ctx->has_new_trajectory = false;
                    trajectory_start_time = ctx->trajectory_valid_time;
                }
                // 同步规划器状态
                output_cmd.fly_status = ctx->is_avoiding_obstacles ? 1 : 0;
                output_cmd.fail_flag = ctx->planning_failed ? 1 : 0;
            }

            // 2. 计算控制量
            if (!local_trajectory.empty()) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - trajectory_start_time).count();

                int idx = (int)(elapsed / TRAJECTORY_TIME_STEP);
                Eigen::Vector3d target_vel_enu = Eigen::Vector3d::Zero();
                double target_yaw_rate = 0.0;

                if (idx < static_cast<int>(local_trajectory.size()) - 1) {
                    const auto& p0 = local_trajectory[idx];
                    const auto& p1 = local_trajectory[idx + 1];

                    // A. 速度 (ENU)
                    target_vel_enu = Eigen::Vector3d(p0.vx, p0.vy, p0.vz);

                    // B. 计算 YawRate (前馈微分: dYaw/dt)
                    double yaw_diff = p1.yaw - p0.yaw;
                    yaw_diff = normalizeAngle(yaw_diff); // 处理 -PI 到 PI 的跳变

                    // 假设点与点之间时间间隔固定为 TRAJECTORY_TIME_STEP
                    target_yaw_rate = yaw_diff / TRAJECTORY_TIME_STEP;

                } else {
                    target_vel_enu.setZero();
                    target_yaw_rate = 0.0;
                }

                // C. 坐标系转换 (ENU -> Body Frame)
                // 你的需求：X纵向(前)，Y横向(右)，Z垂向(上)
                double cy = cos(current_yaw);
                double sy = sin(current_yaw);

                // Body_Forward =  cos * Vx + sin * Vy
                // Body_Right   = -sin * Vx + cos * Vy  (原本是Left，取反变成Right)
                output_cmd.x_speed =  cy * target_vel_enu.x() + sy * target_vel_enu.y();
                output_cmd.y_speed = -(-sy * target_vel_enu.x() + cy * target_vel_enu.y()); // 注意负号
                output_cmd.z_speed = target_vel_enu.z();

                // D. YawRate 方向调整
                // 你的需求：右偏航(顺时针)为正。
                // 数学/ROS标准：左偏航(逆时针)为正。
                output_cmd.yaw_rate = -target_yaw_rate;

            } else {
                // 无轨迹悬停
                output_cmd.x_speed = 0; output_cmd.y_speed = 0; output_cmd.z_speed = 0;
                output_cmd.yaw_rate = 0;
            }
        }
        else if (current_state == FlightState::TAKEOFF) {
            output_cmd.z_speed = 0.5; // 向上 0.5m/s
            output_cmd.x_speed = 0; output_cmd.y_speed = 0; output_cmd.yaw_rate = 0;
        }
        else if (current_state == FlightState::LANDING) {
            output_cmd.z_speed = -0.5; // 向下 0.5m/s
            output_cmd.x_speed = 0; output_cmd.y_speed = 0; output_cmd.yaw_rate = 0;
        }
        else {
            // 其他状态（IDLE, PREPARING等）
            output_cmd.x_speed = 0; output_cmd.y_speed = 0; output_cmd.z_speed = 0; output_cmd.yaw_rate = 0;
        }

        // ---------------------------------------------------------
        // 更新到共享内存 (供其他线程使用)
        // ---------------------------------------------------------
        {
            std::lock_guard<std::mutex> lock(ctx->mtx);
            ctx->final_cmd = output_cmd;
        }

        // ---------------------------------------------------------
        // 发送指令 (保留原有逻辑)
        // ---------------------------------------------------------
        if (use_body_control) {
            // 📢 发送 Body Frame 速度和 Yaw 误差指令
            io->SendBodyControlCommand(target_vel_body, target_yaw_error, command_id);
        } else {
            // 保留原有的 ENU 坐标系指令（用于起飞、降落等状态）
            io->SendCommand(target_pos, target_vel, target_acc, target_yaw, command_id);
        }

        // ---------------------------------------------------------
        // 📝 [新增] 记录数据到 CSV
        // ---------------------------------------------------------
        if (log_file.is_open()) {
            log_file << std::fixed << std::setprecision(4)
                     << output_cmd.timestamp << ","
                     << StateToString(current_state) << ","
                     << output_cmd.x_speed << ","
                     << output_cmd.y_speed << ","
                     << output_cmd.z_speed << ","
                     << output_cmd.yaw_rate << ","
                     << (int)output_cmd.fly_status << ","
                     << (int)output_cmd.fail_flag << ","
                     << current_pos.x() << "," << current_pos.y() << "," << current_pos.z() << ","
                     << current_yaw
                     << std::endl;
        }

        // --- 4. 精确频率控制 (100Hz = 10ms) ---
        auto loop_end = std::chrono::steady_clock::now();

        // 计算本次循环执行消耗的时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);

        // 目标周期 10ms
        int target_period_ms = 10;

        // 计算剩余需要休眠的时间： 目标周期 - 已消耗时间
        int sleep_ms = target_period_ms - duration.count();

        // 如果还有剩余时间才休眠，否则说明系统卡顿，立即进入下一次循环
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        } else {
            // 可选：打印警告，说明计算耗时超过了10ms，无法维持100Hz
            // std::cout << "⚠️ Control loop overrun!" << std::endl;
        }
    }

    if (log_file.is_open()) {
        log_file.close();
        std::cout << "📝 Flight log file closed." << std::endl;
    }

    std::cout << "🚀 Control Thread Stopped." << std::endl;
}