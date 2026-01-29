#include "io_adapter.h"
#include "robot_simulator.h"
#include "utils.h"

class SimIOAdapter : public IOAdapter {
private:
    std::shared_ptr<RobotSimulator> simulator_;
    std::vector<obj_planner::Object3d> obstacles_;
    Eigen::Vector3d global_origin_;
    bool origin_set_;

public:
    SimIOAdapter(const Eigen::Vector3d& init_pos) {
        // 初始化模拟器
        simulator_ = std::make_shared<RobotSimulator>(init_pos);

        // 【关键修改】将 false 改为 true
        // true = 开启反馈，GetInertialData 将返回上一次 SendCommand 的位置
        // 这样状态机就能读到高度变化，从而通过起飞检查
        simulator_->enableFlightControlFeedback(true);

        // 生成虚拟障碍物
        obstacles_ = createRealTimeObstacles();

        // 设置原点
        global_origin_ = init_pos;
        origin_set_ = true;

        std::cout << "🎮 SimIOAdapter initialized with origin: " << global_origin_.transpose() << std::endl;
    }

    // 模拟惯导数据：直接从模拟器读取状态
    PlanningInertialData_t GetInertialData() override {
        PlanningInertialData_t data;

        Eigen::Vector3d pos, vel, acc;
        double yaw;

        // 从模拟器获取当前状态
        simulator_->receiveFromFlightControl(pos, vel, acc, yaw);

        // 【新增】设置时间戳 (当前系统时间)
        data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 仿真中数据总是有效
        data.position_valid = true;
        data.position_x = pos.x();
        data.position_y = pos.y();
        data.position_z = pos.z();

        data.velocity_valid = true;
        data.velocity_x = vel.x();
        data.velocity_y = vel.y();
        data.velocity_z = vel.z();

        data.attitude_valid = true;
        data.roll = 0.0;    // 简化仿真中roll为0
        data.pitch = 0.0;   // 简化仿真中pitch为0
        data.yaw = yaw;

        // 加速度数据（假设为0，可以根据需要扩展）
        data.acc_x = acc.x();
        data.acc_y = acc.y();
        data.acc_z = acc.z();

        return data;
    }

    // 模拟雷达数据：返回生成的虚拟障碍物
    std::vector<obj_planner::Object3d> GetObstacles() override {
        // 在仿真中，障碍物通常是静态的
        // 可以在这里添加动态障碍物逻辑
        return obstacles_;
    }

    // 【新增】模拟遥控器数据
    RCData_t GetRCStatus() override {
        RCData_t data;
        data.is_valid = true;
        data.is_armed = true; // 仿真中默认直接解锁，随时可以起飞
        return data;
    }

    // 【新增】检查感知传感器是否健康 (仿真中永远健康)
    bool IsSensorActive() const override {
        return true; // 仿真环境中感知传感器永远不会超时
    }

    // 【新增】检查惯导是否健康 (仿真中永远健康)
    bool IsINSActive() const override {
        return true; // 仿真环境中惯导永远不会超时
    }

    // 【修改】发送指令
    void SendCommand(const Eigen::Vector3d& pos,
                     const Eigen::Vector3d& vel,
                     const Eigen::Vector3d& acc,
                     double yaw,
                     int command_id) override { // 增加 command_id 参数

        // 仿真器主要响应位置控制，但在状态机中我们可以打印日志
        if (command_id == CMD_AUTO_TAKEOFF) {
             // 可以在这里让模拟器由模拟重力转为模拟上升，或者简单地只打印
             // std::cout << "[Sim] Taking off..." << std::endl;
        }

        // 传递给物理模拟器
        simulator_->sendToFlightControl(pos, vel, acc, yaw);
    }

    // 【新增】发送本体坐标系控制指令
    virtual void SendBodyControlCommand(const Eigen::Vector3d& target_vel_body,
                                        double target_yaw_error,
                                        int command_id) override {
        // 仿真模式下，本体坐标系指令需要转换为ENU坐标系传递给模拟器

        // 获取当前无人机的姿态（简化仿真中roll=pitch=0）
        Eigen::Vector3d current_pos, current_vel, current_acc;
        double current_yaw;
        simulator_->receiveFromFlightControl(current_pos, current_vel, current_acc, current_yaw);

        // 计算从Body到ENU的旋转矩阵（简化版，仅考虑yaw）
        double cos_yaw = cos(current_yaw);
        double sin_yaw = sin(current_yaw);

        // Body Frame速度转换到ENU
        Eigen::Vector3d target_vel_enu;
        target_vel_enu.x() = cos_yaw * target_vel_body.x() - sin_yaw * target_vel_body.y();
        target_vel_enu.y() = sin_yaw * target_vel_body.x() + cos_yaw * target_vel_body.y();
        target_vel_enu.z() = target_vel_body.z(); // Z轴不变

        // 计算目标绝对航向角 = 当前航向 + 航向误差
        double target_yaw_abs = current_yaw + target_yaw_error;
        // 归一化
        while (target_yaw_abs > M_PI) target_yaw_abs -= 2 * M_PI;
        while (target_yaw_abs < -M_PI) target_yaw_abs += 2 * M_PI;

        // 📢 调试输出（可选）
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) { // 每1秒打印一次（100Hz循环）
            std::cout << "🎮 Sim Body Control: vel_body=(" << target_vel_body.transpose()
                      << "), yaw_err=" << target_yaw_error
                      << " -> vel_enu=(" << target_vel_enu.transpose()
                      << "), yaw_abs=" << target_yaw_abs << std::endl;
        }

        // 转换后的指令发送给模拟器（使用速度控制模式）
        Eigen::Vector3d zero_acc = Eigen::Vector3d::Zero(); // 仿真中不关心加速度
        simulator_->sendToFlightControl(Eigen::Vector3d::Zero(), target_vel_enu, zero_acc, target_yaw_abs);
    }

    // 获取全局原点（用于坐标系转换）
    Eigen::Vector3d getGlobalOrigin() const {
        return global_origin_;
    }
};