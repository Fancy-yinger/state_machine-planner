#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <mutex>
#include <atomic>
#include <vector>
#include <Eigen/Core>
#include "io_adapter.h"        // 需要 FlightState 定义
#include "planner_interface.h" // 需要 PathPoint 定义
#include "obstacle_types.h"    // 需要 Object3d 定义

// 1. [输入] 地面站指令 (GCS -> Drone)
struct GCSCommand {
    bool valid = false;             // 标志位：是否有新的有效指令
    uint32_t seq_id = 0;            // 指令序号
    Eigen::Vector3d target_pos;     // 目标位置 (ENU 坐标, 米)
    double cruise_speed;            // 巡航速度 (m/s)
};

// [新增] 符合飞控接口定义的指令结构
struct AutoFlightCmd {
    // 1. 位置/速度信息
    // XYZPosition 暂时不用
    double x_speed = 0.0;    // 纵向速度，向前为正 (m/s)
    double y_speed = 0.0;    // 横向速度，向右为正 (m/s)
    double z_speed = 0.0;    // 垂向速度，向上为正 (m/s)

    // 2. 姿态信息
    // pitch, roll, heading 暂时不用
    double yaw_rate = 0.0;   // 偏航角速度，右偏航(顺时针)为正 (rad/s)

    // 3. 状态标志
    uint8_t fly_status = 0;  // 0: 正常飞行, 1: 避障飞行
    uint8_t fail_flag = 0;   // 0: 规划成功, 1: 规划失败

    // 辅助时间戳
    double timestamp = 0.0;
};

// 2. [输出] 遥测数据 (Drone -> GCS)
struct GCSTelemetry {
    // 状态量
    Eigen::Vector3d current_pos;
    double current_vel;

    // 三大核心数据
    std::vector<obj_planner::PathPoint> global_path;    // 全局路径 (基于代码已有功能生成)
    std::vector<obj_planner::PathPoint> local_traj;     // 局部轨迹 (直接取自发送给飞控的数据)
    std::vector<obj_planner::Object3d> obstacles;       // 障碍物信息

    // 更新标志位
    bool global_path_updated = false;
    bool local_traj_updated = false;
    bool obstacles_updated = false;
};

// 线程间共享的数据上下文
struct SharedContext {
    std::mutex mtx; // 互斥锁，保护数据读写

    // --- 控制线程写 -> 规划线程读 ---
    // 机器人当前的真实物理状态
    Eigen::Vector3d robot_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_acc = Eigen::Vector3d::Zero();

    // === 新增：无人机反馈姿态 (ENU) ===
    // 📢 必须从 IOAdapter/飞控 实时获取并更新，用于坐标系转换
    double robot_roll = 0.0;  // 横滚角 (rad) - 绕X轴旋转
    double robot_pitch = 0.0; // 俯仰角 (rad) - 绕Y轴旋转
    double robot_yaw = 0.0;   // 航向角 (ENU 绝对角度, rad) - 绕Z轴旋转

    // 任务状态
    bool goal_reached = false;

    // 飞行参数
    double takeoff_height = 1.0; // 默认起飞高度 (米)
    bool is_simulation = false;   // 是否为仿真模式

    // 当前飞行状态（供规划线程使用）
    FlightState flight_state = FlightState::IDLE;

    // --- 规划线程写 -> 控制线程读 ---
    // 规划好的新轨迹
    std::vector<obj_planner::PathPoint> new_trajectory;
    // 标记位：告诉控制线程"有新货了"
    bool has_new_trajectory = false; 

    // --- 全局控制 ---
    // 控制程序是否继续运行
    std::atomic<bool> program_running{true};

    std::chrono::steady_clock::time_point trajectory_valid_time;

    // [新增] 规划器传递的具体状态
    bool is_avoiding_obstacles = false; // 是否处于避障模式
    bool planning_failed = false;       // 最近一次规划是否失败

    // [新增] 最终计算出的指令（供通讯线程读取发送给飞控）
    AutoFlightCmd final_cmd;

    // === 新增：系统模式和 GCS 接口槽位 ===
    bool is_real_flight = false; // 📢 关键标志: TRUE为实机模式，FALSE为仿真模式

    GCSCommand gcs_cmd;      // [输入槽]
    GCSTelemetry gcs_telem;  // [输出槽]

    // === 外部调用接口 (API) ===

    // 1. 外部写入指令（通讯线程使用）
    void setGCSCommand(const Eigen::Vector3d& target, double speed) {
        std::lock_guard<std::mutex> lock(mtx);
        gcs_cmd.target_pos = target;
        gcs_cmd.cruise_speed = speed;
        gcs_cmd.seq_id++;
        gcs_cmd.valid = true;
    }

    // 2. 外部读取遥测（通讯线程使用）
    bool getGCSTelemetry(GCSTelemetry& out_data) {
        std::lock_guard<std::mutex> lock(mtx);
        out_data = gcs_telem;
        gcs_telem.global_path_updated = false; // 假设全局路径不常变，读完后重置
        return true;
    }
};

#endif