/****************************************************************************
 *
 *   Copyright (c) 2026 ObjPlanner Integration
 *   Author: Fancy
 *   Date: 2026 01
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 ****************************************************************************/
/**
 * @file objplanner_wrapper.h
 *
 * @brief Wrapper class for integrating obj_planner into state_machine
 *
 * This wrapper simplifies the usage of obj_planner's PlannerInterface
 * and handles data conversion between ROS messages and obj_planner formats.
 */

#ifndef OBJPLANNER_WRAPPER_H
#define OBJPLANNER_WRAPPER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <controller_msgs/FlatTarget.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Include egoplanner headers
#include <plan_manage/planner_interface.h>
#include <plan_manage/obstacle_types.h>
#include <plan_env/sdf_map.h>

namespace statemachine
{

  class ObjPlannerWrapper
  {
  public:
    // --- 【关键修改】添加这行宏 ---
    // 这确保了 new/make_shared 分配内存时会自动进行 16 字节对齐
    // 必须添加这个宏，因为类中包含 Eigen::Vector3d 等固定尺寸的 Eigen 类型成员
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ---------------------------

    ObjPlannerWrapper();
    ~ObjPlannerWrapper();

    // Initialize the planner with parameters
    bool init(double max_vel, double max_acc, double max_jerk,
              double map_size, double map_height, double map_resolution, double map_inflate);

    // Set target waypoint
    void setTarget(const geometry_msgs::PoseStamped& target);

    // 【新增】设置全局路径（用于路径跟随）
    void setGlobalPath(const std::vector<geometry_msgs::PoseStamped>& path);

    // Update current drone state
    void updateDroneState(const geometry_msgs::PoseStamped& pose,
                         const geometry_msgs::TwistStamped& vel);

    // Update obstacles from ROS message format
    void updateObstacles(const std::vector<geometry_msgs::Vector3>& obstacle_positions,
                        const Eigen::Vector3d& drone_pos);

    // Update obstacles directly from planner's obstacle data (preferred method)
    void updateObstaclesFromPlanner(const std::vector<void*>& box_obstacles,
                                   const std::vector<void*>& sphere_obstacles,
                                   const Eigen::Vector3d& drone_pos);

    // Alternative: Update obstacles with explicit obstacle definitions
    struct BoxObstacle {
        double x, y, z;        // Center position
        double width, length, height;  // Size
    };

    struct SphereObstacle {
        double x, y, z;        // Center position
        double radius;         // Radius
    };

    void updateObstacles(const std::vector<BoxObstacle>& box_obstacles,
                        const std::vector<SphereObstacle>& sphere_obstacles,
                        const Eigen::Vector3d& drone_pos);

    // Main planning function - returns success/failure
    bool planTrajectory();

    // Get planned trajectory points for controller
    std::vector<controller_msgs::FlatTarget> getControllerTrajectory() const;

    // Check if goal is reached
    bool isGoalReached(double threshold = 0.3) const;

    // Get current planning status
    bool hasValidTrajectory() const { return has_valid_trajectory_; }
    int getTrajectoryPointCount() const { return trajectory_points_.size(); }

    // Clear trajectory
    void clearTrajectory();

    // 【新增】根据时间采样轨迹点
    bool getSetpointAtTime(double time_from_start, controller_msgs::FlatTarget& target);

    // 【新增】获取轨迹总时长
    double getTrajectoryDuration() const;

    // 【新增】发布轨迹可视化
    void publishTrajectoryVisualization();

  private:
    // ObjPlanner components
    std::shared_ptr<obj_planner::PlannerInterface> planner_;

    // Planner state
    bool initialized_;
    bool has_valid_trajectory_;

    // Drone state
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d current_acc_;

    // Target state
    Eigen::Vector3d target_pos_;
    Eigen::Vector3d target_vel_;
    bool target_set_;

    // 【新增】全局路径存储（用于复杂路径跟随，如L形、多航点）
    std::vector<Eigen::Vector3d> global_waypoints_;

    // Trajectory storage (in obj_planner format)
    std::vector<obj_planner::PathPoint> trajectory_points_;

    // Obstacle storage
    std::vector<obj_planner::ObstacleInfo> obstacles_;

    // Map parameters
    double map_size_;
    double map_height_;
    double map_resolution_;
    double map_inflate_;
    Eigen::Vector3d map_origin_;

    // Planning parameters
    double max_vel_;
    double max_acc_;
    double max_jerk_;

    // Internal helper functions
    void convertPoseToEigen(const geometry_msgs::PoseStamped& pose, Eigen::Vector3d& pos);
    void convertTwistToEigen(const geometry_msgs::TwistStamped& twist, Eigen::Vector3d& vel);
    obj_planner::PathPoint createPathPoint(const Eigen::Vector3d& pos,
                                          const Eigen::Vector3d& vel,
                                          const Eigen::Vector3d& acc,
                                          double time_from_start);

    // 【新增】路径跟随辅助函数
    bool getLocalTargetFromPath(double horizon, Eigen::Vector3d& local_target, Eigen::Vector3d& local_vel);
    double distToSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& seg_start, const Eigen::Vector3d& seg_end,
                        Eigen::Vector3d& projection, double& t) const;
    int findClosestSegment(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& path, double& min_dist) const;

    // === 移植自 RealTimePlanner 的智能避障辅助函数 ===
    double calculateMinObstacleDistance(const Eigen::Vector3d& point) const;
    Eigen::Vector3d calculateBestAvoidanceDirection() const;
    double evaluateAvoidanceDirection(const Eigen::Vector3d& direction, double relative_angle) const;
    bool isInLocalMap(const Eigen::Vector3d& point) const;

    // === 传感器感知辅助函数 ===
    bool isObstacleInFOV(double obs_x, double obs_y, double obs_z) const;

    // Visualization publishers
    ros::Publisher trajectory_vis_pub_;
    ros::Publisher obs_vis_pub_;  // Internal obstacles visualization
    ros::Publisher history_vis_pub_;  // 红色历史轨迹可视化

    // 红色历史轨迹 Marker（保留，用于完整记录飞行路径）
    visualization_msgs::Marker history_marker_;
    Eigen::Vector3d last_history_pos_;  // 用于过滤过密的点

    // 轨迹开始时间（用于绿线修剪）
    ros::Time trajectory_start_time_;

    // === 【新增】感知相关成员变量 ===
    // 感知参数 (来自旧代码)
    double sensor_range_;                      // 传感器检测范围 (默认50.0米)
    double sensor_fov_;                        // 水平视场角 (默认120.0度)
    double sensor_fov_rad_;                    // 水平视场角 (弧度)
    double sensor_height_;                     // 垂直检测范围 (默认10.0米, 即上下各5米)

    // 全局障碍物存储（从参数服务器加载）
    std::vector<BoxObstacle> global_box_obstacles_;
    std::vector<SphereObstacle> global_sphere_obstacles_;
    bool obstacles_loaded_;                    // 障碍物是否已加载标志

    // 当前偏航角（用于FOV计算）
    double current_yaw_;

    // =============================================

    // 【新增】发布内部障碍物可视化（改为public供state_machine调用）
  public:
    void publishObstacleVisualization();

    // 【新增】检查并更新地图位置（滚动地图机制）
    void checkAndUpdateMap();

    // 【新增】感知更新 - 执行FOV筛选+障碍物注入
    void updateSensor();

    // 【新增】从参数服务器加载障碍物
    void loadObstacles(ros::NodeHandle& nh);
  };

} // namespace statemachine

#endif // OBJPLANNER_WRAPPER_H
