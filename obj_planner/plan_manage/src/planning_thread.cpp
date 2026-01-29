#include "planning_thread.h"
#include "io_adapter.h"
#include "utils.h" // 需要包含 utils.h 以使用 createPathFromStart
#include <iostream>
#include <chrono>
#include <thread> // 【关键修正】必须包含这个头文件才能使用 std::this_thread

/*
 * @Function:Obj Planner:Planning Thread
 * @Author:Fancy
 * @Date:2026 01
 */

void PlanningLoop(std::shared_ptr<obj_planner::RealTimePlanner> planner,
                  std::shared_ptr<SharedContext> ctx,
                  std::shared_ptr<IOAdapter> io,
                  std::vector<obj_planner::PathPoint> global_path) {

    // --- 1. 参数配置源头 ---
    // 在这里集中修改飞行参数
    double max_vel = 5.0;   // 最大速度 (m/s) - 真机建议先设小一点
    double max_acc = 3.0;   // 最大加速度 (m/s^2)
    double max_jerk = 2.0;  // 最大加加速度 (m/s^3)

    std::cout << "[Planning] Init params: Vel=" << max_vel
              << " Acc=" << max_acc << " Jerk=" << max_jerk << std::endl;

    // --- 2. 初始化规划器 ---
    // 调用修改后的 init 函数，传入这些参数
    planner->init(max_vel, max_acc, max_jerk);
    
      // 从共享内存获取初始位置
    Eigen::Vector3d initial_pos = Eigen::Vector3d::Zero();
    {
        std::lock_guard<std::mutex> lock(ctx->mtx);
        initial_pos = ctx->robot_pos;
    }

    // 暂时保持 200m 地图，避免规划失败（稍后分析原因）
    Eigen::Vector3d origin = initial_pos - Eigen::Vector3d(100.0, 100.0, 0.0);
    origin.x() = std::max(0.0, origin.x()); origin.y() = std::max(0.0, origin.y()); origin.z() = 0.0;

    // 这里的参数必须与 RealTimePlanner 内部逻辑匹配
    planner->initEsdfMap(200.0, 200.0, 100.0, 2.0, origin, 1.0);
    // planner->initEsdfMap(40.0, 40.0, 10.0, 0.2, origin, 1.0);

    planner->setGlobalPath(global_path);
    planner->updateRobotState(initial_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    
    std::cout << "🧠 Planning Thread Started (STITCHING MODE)." << std::endl;

    // 预测延迟 (100ms)
    const double PLANNING_DELAY_PREDICTION = 0.1; 

    // === 状态保持变量 (实现拼接的关键) ===
    std::vector<obj_planner::PathPoint> last_traj; // 上一条生成的轨迹
    auto last_traj_start_time = std::chrono::steady_clock::now(); // 上一条轨迹的生效时间
    bool has_traj = false;

    // === 状态切换检测变量 ===
    // 记录上一次的状态，用于检测状态切换（Rising Edge）
    FlightState last_flight_state = FlightState::IDLE;

    // 缓存最终目标点，以便在起飞完成后重新生成路径
    Eigen::Vector3d mission_goal = Eigen::Vector3d::Zero();
    bool has_mission = false; 

    while (ctx->program_running) {
        auto now = std::chrono::steady_clock::now();

        // ====================================================
        // 【第一步：处理输入】检查地面站指令 (仅实机模式处理)
        // ====================================================
        {
            std::lock_guard<std::mutex> lock(ctx->mtx);

            // 📢 核心逻辑：仅当 is_real_flight 为 TRUE 且收到有效指令时才执行
            if (ctx->is_real_flight && ctx->gcs_cmd.valid) {

                std::cout << "🚀 [REAL FLIGHT] Processing GCS Command: "
                          << ctx->gcs_cmd.target_pos.transpose() << std::endl;

                // 1. 获取并缓存指令数据
                mission_goal = ctx->gcs_cmd.target_pos; // 【记录目标】
                has_mission = true;

                double new_speed = ctx->gcs_cmd.cruise_speed;
                planner->updateCruiseSpeed(new_speed);

                // 2. 生成初始全局路径 (给地面站看着用，此时起点可能是地面)
                std::vector<obj_planner::PathPoint> new_global_path =
                    createPathFromStart(ctx->robot_pos, mission_goal, 500);

                // 3. 将生成的路径设置给规划器并回传
                planner->setGlobalPath(new_global_path);
                ctx->gcs_telem.global_path = new_global_path;
                ctx->gcs_telem.global_path_updated = true;

                // 4. 标记指令已处理
                ctx->gcs_cmd.valid = false;
            }
            // else: 仿真模式或未收到指令，PlanningLoop 使用预设/内部逻辑继续运行。
        }

        // 1. 【新增】感知健康检查
        bool sensor_healthy = io->IsSensorActive();
        if (!sensor_healthy) {
            static int health_warn_count = 0;
            if (health_warn_count++ % 10 == 0) { // 每10次打印一次，避免刷屏
                std::cout << "🚨 [Planning] SENSOR UNHEALTHY! Entering safety mode." << std::endl;
            }

            // 感知不健康时的安全策略：
            // - 清空障碍物列表（避免使用过期数据）
            // - 规划器会生成保守的悬停或减速轨迹
            std::vector<obj_planner::Object3d> empty_obstacles;
            planner->updateObstacles(empty_obstacles);
        } else {
            // 2. 从 IO Adapter 获取障碍物数据
            std::vector<obj_planner::Object3d> obstacles = io->GetObstacles();
            planner->updateObstacles(obstacles);
        }
        
        // 计算新轨迹的“生效时刻” = 当前时间 + 预测的计算耗时
        auto future_valid_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(PLANNING_DELAY_PREDICTION)
        );

        // 1. 获取真实传感器数据和飞行状态
        Eigen::Vector3d sensor_pos, sensor_vel, sensor_acc;
        FlightState current_fsm_state;
        {
            std::lock_guard<std::mutex> lock(ctx->mtx);
            sensor_pos = ctx->robot_pos;
            sensor_vel = ctx->robot_vel;
            sensor_acc = ctx->robot_acc;
            current_fsm_state = ctx->flight_state;
            if (ctx->goal_reached) break;
        }

        // 2. 始终更新机器人状态 (这很重要！即使不起飞，地图也需要跟随飞机移动)
        planner->updateRobotState(sensor_pos, sensor_vel, sensor_acc);

        // 3. 【核心修改】待机检查
        // 如果还没有进入 EXECUTING 状态（比如还在 IDLE, PREPARING, TAKEOFF）
        // 我们就不做规划，并且重置历史记录
        if (current_fsm_state != FlightState::EXECUTING) {

            if (has_traj) {
                std::cout << "💤 [Planner] Not in EXECUTING. Clearing history & Idling..." << std::endl;
            }

            has_traj = false; // ❌ 清除历史轨迹，确保下次是"全新启动"
            last_traj.clear();

            // 更新历史状态，并跳过本次循环
            last_flight_state = current_fsm_state;
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 稍微休眠，省CPU
            continue;
        }

        // 4. 【关键逻辑】检测到刚从"非执行"切换到"执行" (Rising Edge)
        if (last_flight_state != FlightState::EXECUTING) {
            std::cout << "🚀 [Planner] Transition to EXECUTING detected!" << std::endl;

            // 如果有任务目标，以"当前空中位置"为起点，重新生成全局路径
            if (has_mission) {
                std::cout << "🔄 Regenerating Global Path from AIR: "
                          << sensor_pos.transpose() << " -> " << mission_goal.transpose() << std::endl;

                std::vector<obj_planner::PathPoint> air_start_path =
                    createPathFromStart(sensor_pos, mission_goal, 500);

                planner->setGlobalPath(air_start_path);

                // 强制重置拼接状态（双重保险）
                has_traj = false;
            }
        }

        // 更新历史状态
        last_flight_state = current_fsm_state;

        // =========================================================
        // 下面是正常的规划逻辑 (performReplanning ...)
        // =========================================================

        // 4. 确定规划起点 (start_pos)
        Eigen::Vector3d start_pos, start_vel, start_acc;
        bool use_stitching = false;

        // 尝试使用“轨迹拼接”
        if (has_traj) {
            // 计算 future_valid_time 相对于上一条轨迹起点的时刻 t
            double t_rel = std::chrono::duration<double>(future_valid_time - last_traj_start_time).count();
            
            // 从旧轨迹采样理论状态
            if (getStateFromTrajectory(last_traj, t_rel, start_pos, start_vel, start_acc)) {
#ifdef USE_REAL_HARDWARE
                // 实机模式：安全检查，如果理论位置和真实位置偏差太大 (>1.5m)，说明失控，强制回退
                if ((start_pos - sensor_pos).norm() < 1.5) {
                    use_stitching = true;
                } else {
                    std::cout << "⚠️ Tracking error too large! Resetting to sensor." << std::endl;
                }
#else
                // 仿真模式：信任轨迹拼接，假设完美跟踪
                use_stitching = true;
#endif
            }
        }

        // 如果无法拼接（第一次运行，或偏差太大，或旧轨迹跑完了），使用线性预测兜底
        if (!use_stitching) {
            // 【修改开始】-------------------------------------------------

            // 既然无法拼接，说明我们要么是刚起飞，要么是断档了。
            // 为了安全起见，我们假设从当前位置"从零起步"。
            // 这样可以避免把传感器的噪音（比如 drift）当作初速度带入规划，导致起步猛冲。

            start_pos = sensor_pos; // 甚至建议不要加预测量，直接用当前位置，更稳

            // 强制归零：无论传感器读数是多少，规划都从静止开始
            start_vel = Eigen::Vector3d::Zero();
            start_acc = Eigen::Vector3d::Zero();

            std::cout << "✨ [Planner] Resetting start state to HOVER (Vel=0)." << std::endl;

            // 原有逻辑备份（已注释掉）：
            // start_pos = sensor_pos + sensor_vel * PLANNING_DELAY_PREDICTION;
            // start_vel = sensor_vel;
            // start_acc = sensor_acc;

            // 【修改结束】-------------------------------------------------
        }

        // 3. 更新规划器状态 (确保地图加载正确)
        planner->updateRobotState(start_pos, start_vel, start_acc);

        // 4. 执行规划
        // 注意：performReplanning 内部最好能直接接收 start_pos 参数，或者它读取 updateRobotState 的值
        bool plan_success = planner->performReplanning(start_pos, start_vel, start_acc);

        // 5. 处理结果
        std::vector<obj_planner::PathPoint> trajectory;
        if (plan_success && planner->getCurrentTrajectory(trajectory)) {
            {
                std::lock_guard<std::mutex> lock(ctx->mtx);
                ctx->new_trajectory = trajectory;
                ctx->has_new_trajectory = true;
                ctx->trajectory_valid_time = future_valid_time;

                // ====================================================
                // 【新增】同步规划器的状态到共享内存
                // ====================================================
                ctx->is_avoiding_obstacles = planner->isAvoidanceTriggered();
                ctx->planning_failed = false;

                // ====================================================
                // 【第二步：准备输出】填充遥测数据 (实机和仿真模式都应回传状态)
                // ====================================================

                // A. 局部轨迹：直接使用发送给飞控的同一份数据
                ctx->gcs_telem.local_traj = trajectory;
                ctx->gcs_telem.local_traj_updated = true;

                // B. 障碍物：从规划器获取当前帧的障碍物
                ctx->gcs_telem.obstacles = planner->getCurrentObstacles();
                ctx->gcs_telem.obstacles_updated = true;

                // C. 当前状态
                ctx->gcs_telem.current_pos = sensor_pos;
                ctx->gcs_telem.current_vel = sensor_vel.norm();
            }

            // === 更新本地缓存，供下一次循环拼接使用 ===
            last_traj = trajectory;
            last_traj_start_time = future_valid_time;
            has_traj = true;

            // 调试打印：确认当前是在 拼接 还是 预测
            // std::cout << "🧠 Plan OK. Mode: " << (use_stitching ? "STITCH" : "PREDICT") << std::endl;
        } else {
             std::cout << "⚠️ Planner failed!" << std::endl;

             // [新增] 标记规划失败状态
             {
                 std::lock_guard<std::mutex> lock(ctx->mtx);
                 ctx->planning_failed = true;
                 ctx->is_avoiding_obstacles = false; // 失败时重置避障状态
             }

             // 规划失败时不更新 last_traj，下一次循环会尝试在旧轨迹更靠后的位置继续拼接
             // 如果一直失败，t_rel 会超出范围，自动触发线性预测兜底
        }

        // 频率控制 (2Hz = 500ms)
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count();
        int sleep_ms = 500 - elapsed; 
        if (sleep_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    std::cout << "🧠 Planning Thread Stopped." << std::endl;
}

// 辅助函数：从轨迹中采样指定时间点的状态（线性插值）
// 假设轨迹点的时间间隔固定为 0.01s (由 PlannerInterface 决定)
// 这保证了新轨迹的起点严格等于旧轨迹在同一时刻的理论状态，实现 C0/C1 连续
bool getStateFromTrajectory(const std::vector<obj_planner::PathPoint>& trajectory, 
                            double t_rel, 
                            Eigen::Vector3d& pos, 
                            Eigen::Vector3d& vel, 
                            Eigen::Vector3d& acc) {
    if (trajectory.empty()) return false;
    
    // 注意：这里的时间步长 0.01 必须与 planner_interface.cpp 中的 time_step 一致
    double dt = 0.01; 
    double max_time = (trajectory.size() - 1) * dt;
    
    // 如果请求的时间超出了轨迹范围（说明规划算慢了，或者轨迹跑完了），返回失败
    if (t_rel < 0.0) t_rel = 0.0;
    if (t_rel > max_time) return false;

    int idx = static_cast<int>(t_rel / dt);
    if (idx >= trajectory.size() - 1) idx = trajectory.size() - 2;
    
    double alpha = (t_rel - idx * dt) / dt;
    
    const auto& p0 = trajectory[idx];
    const auto& p1 = trajectory[idx + 1];

    // 线性插值位置、速度、加速度
    pos = Eigen::Vector3d(p0.x, p0.y, p0.z) * (1 - alpha) + Eigen::Vector3d(p1.x, p1.y, p1.z) * alpha;
    vel = Eigen::Vector3d(p0.vx, p0.vy, p0.vz) * (1 - alpha) + Eigen::Vector3d(p1.vx, p1.vy, p1.vz) * alpha;
    acc = Eigen::Vector3d(p0.ax, p0.ay, p0.az) * (1 - alpha) + Eigen::Vector3d(p1.ax, p1.ay, p1.az) * alpha;
    
    return true;
}