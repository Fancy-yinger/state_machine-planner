#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
#include "planner_interface.h"
#include "obstacle_types.h"

/*
 * @Function:Obj Planner:Real Time Planner
 * @Author:Fancy
 * @Date:2026 01
 */

namespace obj_planner {

enum class PlanningStatus {
    IDLE,
    PLANNING, 
    EXECUTING,
    FAILED,
    GOAL_REACHED
};

class RealTimePlanner {
private:
    std::shared_ptr<PlannerInterface> local_planner_;
    
    // 规划状态
    std::vector<PathPoint> global_path_;
    std::vector<PathPoint> current_trajectory_;
    std::vector<Object3d> current_obstacles_;
    
    // 机器人状态
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_acceleration_;
    
    // 地图管理
    Eigen::Vector3d map_origin_;
    double map_size_;
    double map_height_;
    bool map_initialized_;

    double max_vel_;  // 添加这一行来存储最大速度
    double max_acc_;
    double max_jerk_;
    
    // 规划控制
    int current_path_index_ = 0;
    double replan_rate_ = 2.0;
    double safety_margin_ = 0.5;
    // 【新增】用于存储地图参数的成员变量
    double map_resolution_; 
    double map_inflate_;
    
    // 线程控制
    std::atomic<bool> planning_active_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread planning_thread_;
    mutable std::mutex state_mutex_;
    mutable std::mutex trajectory_mutex_;

    Eigen::Vector3d last_planned_velocity_;
    Eigen::Vector3d last_planned_acceleration_;
    double last_planned_yaw_ = 0.0;
    double last_planned_pitch_ = 0.0;
    std::chrono::steady_clock::time_point last_plan_time_;

    Eigen::Vector3d filtered_target_;
    bool target_initialized_ = false;
    Eigen::Vector3d last_raw_target_; // 【新增】记录上一帧的原始目标

    // [新增] 规划器状态变量
    bool avoidance_triggered_ = false; // 是否处于避障模式
    bool last_plan_failed_ = false;    // 最近一次规划是否失败

public:
    RealTimePlanner();
    ~RealTimePlanner();

    void init(double max_vel, double max_acc, double max_jerk);
    void initEsdfMap(double x_size, double y_size, double z_size, 
                    double resolution, Eigen::Vector3d origin, double inflate_values);
    
    void setGlobalPath(const std::vector<PathPoint>& global_path);
    void updateObstacles(const std::vector<Object3d>& obstacles);
    void updateRobotState(const Eigen::Vector3d& position, 
                         const Eigen::Vector3d& velocity,
                         const Eigen::Vector3d& acceleration);
    
    void startPlanning(double replan_rate = 2.0);
    void stopPlanning();
    
    bool getCurrentTrajectory(std::vector<PathPoint>& trajectory);
    bool isGoalReached() const;
    PlanningStatus getPlanningStatus() const;
    
    void exportDataForPython(const std::vector<PathPoint>& global_traj,
                           const std::vector<PathPoint>& local_traj,
                           const std::vector<Object3d>& obstacles);

    Eigen::Vector3d selectLocalTargetWithContinuity(double dt);
    std::vector<PathPoint> generateContinuousWaypoints(double dt, const Eigen::Vector3d& local_target);
    void smoothTrajectoryTransition(std::vector<PathPoint>& trajectory, double dt);
    bool checkObstacleMapIntersection(const Object3d& obstacle, 
                                    const Eigen::Vector3d& map_origin,
                                    double map_size, double map_height);
    void updateObstaclesInView();

    bool performReplanning(const Eigen::Vector3d& start_pos, 
                       const Eigen::Vector3d& start_vel, 
                       const Eigen::Vector3d& start_acc);

    std::vector<Eigen::Vector3d> generateLocalWaypoints(const Eigen::Vector3d& target);

    // 1. 动态更新巡航速度
    void updateCruiseSpeed(double speed) {
        max_vel_ = speed;
        local_planner_->initParam(max_vel_, max_acc_, max_jerk_);
    }

    // 2. 获取当前障碍物
    std::vector<Object3d> getCurrentObstacles() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_obstacles_;
    }

    // [新增] 获取当前是否触发了避障逻辑
    bool isAvoidanceTriggered() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return avoidance_triggered_;
    }

    // [新增] 获取最近一次规划是否失败
    bool isLastPlanFailed() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return last_plan_failed_;
    }

private:
    // 新增的私有方法声明
    void planningLoop();
    Eigen::Vector3d selectLocalTarget();
    // std::vector<Eigen::Vector3d> generateLocalWaypoints();
    double calculateLookaheadDistance() const;
    bool checkTrajectorySafety(const std::vector<PathPoint>& traj);
    void handlePlanningFailure();
    
    // 地图管理方法
    bool needMapUpdate(const Eigen::Vector3d& robot_position);
    void updateMapPosition(const Eigen::Vector3d& robot_position);
    void reinitializeLocalMap();
    
    bool isInLocalMap(const Eigen::Vector3d& point) const;
    
    // 辅助方法
    bool isTargetReachable(const Eigen::Vector3d& target) const;
    bool isPointSafe(const Eigen::Vector3d& point) const;
    
    // 智能避障相关方法
    double calculateMinObstacleDistance(const Eigen::Vector3d& point) const;
    Eigen::Vector3d calculateBestAvoidanceDirection() const;
    double evaluateAvoidanceDirection(const Eigen::Vector3d& direction) const;
    double calculateObstacleDensity() const;
};

} // namespace obj_planner

#endif