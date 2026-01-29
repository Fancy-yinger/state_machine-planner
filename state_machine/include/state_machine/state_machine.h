/****************************************************************************
 *
 *   Copyright (c) 2022-2025 Batuhan Yumurtaci. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file state_machine.h
 *
 * @date 03 June 2022
 * 
 * @brief State Machine Mavros
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */

 //作用：防止头文件被重复包含，避免编译错误
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <ros/ros.h>
#include <stdlib.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>


#include "planner/GetTrajectory.h"
#include "controller_msgs/FlatTarget.h"

#include <thread>
#include <mutex>
#include <state_machine/objplanner_wrapper.h>
#include <state_machine/state_base.h>
#include <map>
#include <string>
#include <memory>

namespace statemachine
{  
  // Timer class to perform time based actions
  // Timer计时器类功能：用于任务时间管理（如轨迹执行计时、着陆过程计时）
  class Timer
  {
  public:
    Timer(){};
    ros::Time start_time_;

    void startTimer()
    {start_time_ = ros::Time::now();}

    double getTime()
    {return ros::Duration(ros::Time::now() - start_time_).toSec();}
  };
  
  // State machine class to manage the mission
  // StateMachine 主类
  class StateMachine
  {

  public:
    StateMachine(ros::NodeHandle nh);

    enum States
    {
      ARMED = 0,           //无人机解锁
      TAKEOFF = 1,         //起飞阶段
      WAYPOINT = 2,        //航点飞行模式
      TRAJECTORY = 3,      //轨迹跟踪模式
      LANDING = 4,         //着陆阶段
      PERCHING = 5,        //特种降落模式，实现精准停泊或抓取操作
      ATRAJECTORY = 6,     // A* 轨迹规划状态
      OBJ_PLANNER = 7       // ObjPlanner 实时轨迹规划状态
    };
    
    // Define vectors and matrices
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Matrix<double, 4, Eigen::Dynamic> WaypointMatrix;
    
  private:
    bool wp_completed_;                         // Flag used for waypoint mission
    bool request_input_ = false;                // Keyboard input trigger
    bool goal_received_;                         // 是否收到目标点


    int n_wp_;                                  // Total number of waypoints
    int state_;                                 // State of the state machine
    int input_;                                 // Keyboard input
    int current_wp_;                            // Current waypoint index
    int perching_type_;                         // Vertical or inclined perching 键盘输入触发器垂直或倾斜栖息

    double ttc_threshold_;                      // Time to contact threshold 接触时间阈值
    double ttc_threshold_ver_;                  // Time to contact threshold for vertical perching 垂直栖息的接触时间阈值
    double ttc_threshold_inc_;                  // Time to contact threshold for inclined perching 倾斜栖息的接触时间阈值
    double wp_converged_threshold_;             // Waypoint threshold for absolute distance in 3D space 三维空间绝对距离的航点阈值
    double wp_stopped_threshold_;               // Waypoint threshold for absolute velocity in 3D space 三维空间绝对速度的航点阈值

    ros::Rate rate_ = 20;                       // Loop rate
    
    ros::NodeHandle nh_;                        // Node handles
    
    ros::Publisher rpy_pub_;                    // Roll, pitch and yaw publisher
    ros::Publisher local_pos_pub_;              // Desired Position & Orientation
    ros::Publisher timeToContact_pub_;          // Time to contact publisher
    ros::Publisher perching_pose_pub_;          // Publish the perching target for data analysis
    ros::Publisher actuator_control_pub_;       // Actuator control for servos
    ros::Publisher controller_trigger_pub_;     // Trigger for geometric controller
    ros::Publisher flat_target_pub_;            // 【Phase 3】FlatTarget trajectory setpoint for ObjPlanner
    
    ros::Subscriber state_sub_;                 // FCU state subscriber
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation
    ros::Subscriber local_pos_vel_sub_;         // Linear & Angular velocity
    ros::Subscriber goal_pose_sub_;             // 目标点订阅者

    ros::ServiceClient land_client_;            // Landing ros service
    ros::ServiceClient arming_client_;          // Arming ros service
    ros::ServiceClient planner_client_;         // Getting trajectory ros service
    ros::ServiceClient set_mode_client_;        // Setting mode ros service
    ros::ServiceClient force_disarm_client_;    // Setting mode ros service
    
    std_msgs::Float64 timeToContact_msg_;       // Time to contact message
    std_msgs::Bool controller_activation_;      // Activation flag for controller
    std_msgs::Int32 actuator_control_msg_;      // Control arm actuator servos 
    // IDLE = 0, FLIGHT = 1, PREPARE = 2, CLOSE = 3, RELEASE = 4,
    
    mavros_msgs::State current_state_;          // FCU state
    mavros_msgs::CommandTOL land_cmd_;          // Land command for landing service
    mavros_msgs::CommandBool arm_cmd_;          // Arm command for arming service
    mavros_msgs::SetMode offb_mode_cmd_;        // Offboard command for mode service
    mavros_msgs::CommandLong force_disarm_cmd_; // Force disarm command for perching

    geometry_msgs::PoseStamped des_pose_;       // Target pose
    geometry_msgs::PoseStamped zero_pose_;      // Target pose for initial states
    geometry_msgs::PoseStamped takeoff_pose_;   // Take-off pose
    geometry_msgs::PoseStamped current_pose_;   // Current pose
    geometry_msgs::PoseStamped ver_perch_pose_; // Target pose for vertical perching
    geometry_msgs::PoseStamped inc_perch_pose_; // Target pose for inclined perching
    geometry_msgs::TwistStamped current_vel_;   // Current velocity
    geometry_msgs::Vector3 rpy_;                // Current roll, pitch and yaw
    geometry_msgs::PoseStamped goal_pose_;      // 目标点位置

    // ObjPlanner 相关
    std::shared_ptr<ObjPlannerWrapper> egoplanner_wrapper_;  // ObjPlanner 包装器
    bool egoplanner_initialized_;              // ObjPlanner 初始化标志
    bool egoplanner_goal_reached_;             // ObjPlanner 目标到达标志
    Timer egoplanner_timer_;                   // ObjPlanner 计时器
    // 【新增】轨迹执行时间追踪
    ros::Time egoplanner_traj_start_time_;     // 轨迹开始时间
    bool has_active_egoplanner_traj_;          // 是否有正在执行的轨迹
    bool replan_needed_;                       // 【新增】是否需要重新规划
    // 不再需要 obstacle_positions_，直接从参数服务器或planner获取障碍物

    // 【Simulated Sensor FOV】新增成员变量 - 扇形感知模型 (类LiDAR)
    double sensor_range_;                      // 传感器检测范围 (默认50.0米)
    double sensor_fov_;                        // 水平视场角 (默认120.0度)
    double sensor_fov_rad_;                    // 水平视场角 (弧度)
    double sensor_height_;                     // 垂直检测范围 (默认10.0米, 即上下各5米)
    std::vector<ObjPlannerWrapper::BoxObstacle> global_box_obstacles_;    // 全局盒状障碍物列表
    std::vector<ObjPlannerWrapper::SphereObstacle> global_sphere_obstacles_;  // 全局球状障碍物列表
    bool obstacles_loaded_;                    // 障碍物是否已加载标志
    ros::Publisher sensor_fov_vis_pub_;        // 传感器FOV可视化发布器 (扇形填充+轮廓)

    planner::GetTrajectory waypoint_cmd_;       // Trigger waypoint w. planner ros service
    planner::GetTrajectory trajectory_cmd_;     // Trigger minimum snap polynomial w. planner ros service
    planner::GetTrajectory perching_trj_cmd_;   // Trigger minimum snap polynomial f. perching w. planner ros service
    planner::GetTrajectory localtakeoff_cmd_;   // Trigger local takeoff w. planner ros service
    planner::GetTrajectory globaltakeoff_cmd_;  // Trigger global takeoff w. planner ros service
    planner::GetTrajectory astar_trajectory_cmd_; // A* trajectory command  // 添加这一行

    
    Vec3 current_RPY_;                          // Current Roll, Pitch and Yaw [rad]
    
    WaypointMatrix wp_matrix_;                  // Matrix to store Waypoints

    Timer traj_timer_;                          // Timer object for trajectory
    Timer perching_timer_;                      // Timer object for perching to release actuators
    
    std::mutex input_mutex_;                    // Non blocking state machine

    
    // Callback function to get the current fcu status
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    
    // Callback function to get the current pose
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Callback function to get the current linear and angular velocity
    void localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    
    // Transition conditions of the state machine
    void updateState();     
    
    // Commands in a particular state 根据当前状态发布控制指令
    void publishCommand();

    // Publish target body rates and thrust for geometric controller
    void pubRateCommands();

    // Set and send the next waypoint for trajectory generation  // 填充航点坐标到 waypoint_cmd_ // 通过 planner_client_ 调用服务
    void sendNextWaypoint(int index);
    
    // Read the waypoints from YAML file
    void readWaypoints();
    
    // Check if arrived at target waypoint - position  // 计算两个位置的三维欧氏距离  // 对比预设阈值 wp_converged_threshold_
    bool wpConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
    
    // Check if arrived at target waypoint - current velocity  // 检查当前速度是否低于 wp_stopped_threshold_
    bool wpStopped(); 

    // Time to target position based on current position and velocity
    double timeToContact(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
    
    // Initialize the FCU connection for Mavros and PX4 连接飞控
    void initializeFCU();
    
    // Initialize publishers, subscribers and servies
    void initializePublishers();         //创建消息发布器
    void initializeSubscribers();        //创建消息订阅器
    void initializeServices();           //创建ros服务
    void initializeParameters();         //初始化加载参数
    
    // Decouple threads of UpdateState and PublishCommand as waiting for user input
    // Required for uniterrupted advertisement of the desired pose 
    // Otherwise the offboard mode is deactivated
    int getInput();
    void getKeyboardInput();
    void setInput(int input);
    void requestKeyboardInput();

    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // ObjPlanner 相关方法
    void initializeObjPlanner();
    void updateObjPlannerState();
    void publishObjPlannerTrajectory();
    // 【Simulated Sensor FOV】新增方法
    void loadObstacles();                      // 从参数服务器加载所有障碍物（一次性）
    void publishSensorFOVVisualization();      // 发布传感器范围可视化

    // [论文架构重构] 状态机上下文与状态池
    StateContext ctx_;                                      // 上下文实例
    std::map<std::string, std::shared_ptr<StateBase>> state_pool_; // 状态池
    std::shared_ptr<StateBase> current_state_ptr_;          // 当前活跃状态指针

    // 初始化上下文的辅助函数
    void initializeContext();

    // [论文 3.4.1] 状态切换函数 (State Transition)
    void changeState(const std::string& new_state_id);

  };
}

#endif