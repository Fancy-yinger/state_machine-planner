#ifndef IO_ADAPTER_H
#define IO_ADAPTER_H

#include <vector>
#include <Eigen/Core>
#include "obstacle_types.h"

// 【新增】简化的遥控器数据结构
struct RCData_t {
    bool is_valid = false;
    bool is_armed = false; // 是否解锁
    // 其他通道数据暂时不需要，只要解锁信号
};

// 【新增】飞行状态枚举
enum class FlightState {
    IDLE,
    PREPARING,
    TAKEOFF,
    EXECUTING,
    LANDING,
    FINISHED
};

// 【重要修改】此处枚举值必须严格对应 interface.h
enum CmdIdx {
    CMD_EXTERNAL_CONTROL = 1,    // 外控
    CMD_PROGRAM_CONTROL = 3,     // 程控 (自主飞行用这个)
    CMD_AUTO_TAKEOFF = 14,       // 自动起飞
    CMD_AUTO_LAND = 15,          // 自动着陆 (interface.h 是 15, 之前写成了 13)
    CMD_PRE_CONTROL = 18,        // 预控 (interface.h 是 18, 之前写成了 0)
    CMD_ENGINE_STOP = 13         // 停车 (interface.h 是 13)
};

// 统一的数据结构
struct PlanningInertialData_t {
    uint64_t timestamp = 0;  // 【新增】时间戳 (毫秒)
    bool position_valid = false;
    double position_x = 0; double position_y = 0; double position_z = 0;
    bool attitude_valid = false;
    double roll = 0; double pitch = 0; double yaw = 0;
    bool velocity_valid = false;
    double velocity_x = 0; double velocity_y = 0; double velocity_z = 0;
    double acc_x = 0; double acc_y = 0; double acc_z = 0;
};

// 【核心】抽象基类
class IOAdapter {
public:
    virtual ~IOAdapter() {}

    // 1. 获取惯导数据 (输入)
    virtual PlanningInertialData_t GetInertialData() = 0;

    // 2. 获取障碍物 (输入)
    virtual std::vector<obj_planner::Object3d> GetObstacles() = 0;

    // 3. 【新增】获取遥控器数据
    virtual RCData_t GetRCStatus() = 0;

    // 4. 【新增】检查感知传感器是否健康
    virtual bool IsSensorActive() const = 0;

    // 5. 【新增】检查惯导是否健康
    virtual bool IsINSActive() const = 0;

    // 6. 【修改】发送控制指令 (增加command_id参数)
    // command_id 对应 interface.h 中的 CmdIdx
    virtual void SendCommand(const Eigen::Vector3d& pos,
                             const Eigen::Vector3d& vel,
                             const Eigen::Vector3d& acc,
                             double yaw,
                             int command_id) = 0;

    // 【新增】基于本体坐标系的速度和航向角误差指令接口
    /**
     * @brief 发送基于本体坐标系的速度和航向角误差指令给飞控。
     * @param target_vel_body  目标速度 (Body Frame, X-前, Y-右, Z-上/下)
     * @param target_yaw_error 目标航向角误差 (rad)
     * @param command_id       指令ID/模式
     */
    virtual void SendBodyControlCommand(const Eigen::Vector3d& target_vel_body,
                                        double target_yaw_error,
                                        int command_id) = 0;
};

#endif