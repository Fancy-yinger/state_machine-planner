/****************************************************************************
 *
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


#include <state_machine/state_machine.h>
#include <state_machine/states/state_obj_planner.h>
#include <state_machine/states/state_takeoff.h>
#include <state_machine/states/state_landing.h>
#include <state_machine/states/state_armed.h>
// 【新增】
#include <state_machine/states/state_emergency_hover.h>

#define FCU_TIME_BUFFER 3.0f

using namespace XmlRpc;

namespace statemachine
{
  StateMachine::StateMachine(ros::NodeHandle nh) : nh_(nh)
  {
    ROS_WARN("[INIT]: Starting State Machine ...");

    initializePublishers();
    initializeSubscribers();
    initializeServices();
    initializeParameters();
    readWaypoints();
    initializeFCU();

    // [Phase 3 Fix] 初始化 ObjPlanner BEFORE 绑定 Context
    // 确保 egoplanner_wrapper_ 对象已创建，否则 Context 会持有空指针
    initializeObjPlanner();

    // [Init] Initialize Context for State Pattern
    // 必须在所有成员变量初始化完成后才能调用
    initializeContext();

    ROS_WARN("[INIT]: State Machine is ready!");

    // Start state machine
    if (current_state_.armed){
      if(getInput() == 0)
          state_ = ARMED;
      requestKeyboardInput();
    } else {
      setInput(100);
      }  

    while (ros::ok())
    {
      ros::spinOnce();
      updateState();
      publishCommand();

      // 【Simulated Sensor FOV】持续发布传感器范围可视化（独立于飞行状态）
      // 这确保在启动时立即看到FOV，无论处于什么状态
      publishSensorFOVVisualization();

      rate_.sleep();
    }

  }

  // Define Callback functions
  void StateMachine::stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    current_state_ = *msg;
    }
  
    //该函数接收位姿消息，提取四元数转换为欧拉角（RPY），将弧度转为角度后发布。主要流程：保存当前位姿→提取四元数→计算RPY→弧度转角度→发布角度数据。
  void StateMachine::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    current_pose_ = *msg;
    // Get roll pitch and yaw
    tf::Quaternion q(current_pose_.pose.orientation.x, 
                    current_pose_.pose.orientation.y, 
                    current_pose_.pose.orientation.z, 
                    current_pose_.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_RPY_ << roll, pitch, yaw;
    
    // Conversion radians to degree
    double pi = 3.14159;
    rpy_.x = roll * (180 / pi);
    rpy_.y = pitch * (180 / pi);
    rpy_.z = yaw * (180 / pi);
    
    rpy_pub_.publish(rpy_);
    }

  void StateMachine::localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
  {
    current_vel_ = *msg;
    }

  void StateMachine::updateState()
  {
    // [Phase 3] 混合运行模式 (Hybrid Mode)
    // 对于已迁移的状态 (ARMED, TAKEOFF, OBJ_PLANNER, LANDING)，使用新架构
    // 对于未迁移的状态 (WAYPOINT, TRAJECTORY, PERCHING, ATRAJECTORY)，使用旧的 switch-case

    if (state_ == ARMED || state_ == TAKEOFF || state_ == OBJ_PLANNER ||
        state_ == LANDING || state_ == EMERGENCY_HOVER) // 【新增】
    {
      // 第一次进入这些状态时，初始化指针
      if (current_state_ptr_ == nullptr) {
        std::string target;
        if (state_ == ARMED) target = "ARMED";
        else if (state_ == TAKEOFF) target = "TAKEOFF";
        else if (state_ == OBJ_PLANNER) target = "OBJ_PLANNER";
        else if (state_ == LANDING) target = "LANDING";
        else if (state_ == EMERGENCY_HOVER) target = "EMERGENCY_HOVER"; // 【新增】
        changeState(target); // 这会调用 enter()
      }

      // 运行新架构
      if (current_state_ptr_) {
        std::string next = current_state_ptr_->execute(&ctx_);
        if (!next.empty()) changeState(next);
      }
    }
    else
    {
      // 从新架构切换到旧架构时的清理
      if (current_state_ptr_) {
        current_state_ptr_->exit(&ctx_);
        current_state_ptr_ = nullptr;
      }

      // 旧的 switch-case 逻辑（用于未迁移的状态）
      switch (state_)
      {

      // [Phase 3] 已迁移到新架构，旧代码保留作参考
      // case StateMachine::ARMED:
      // {
      //   // Arming sequence
      //   if (current_state_.armed){
      //     ROS_INFO("[ARMED] : Press ENTER for Take-off: ");
      //     if(getInput() == 0)
      //         state_ = TAKEOFF;
      //         controller_activation_.data = true;
      //     requestKeyboardInput();
      //   } else {
      //     ROS_INFO("[ARMED] : Arming ...");
      //     setInput(100);
      //     }
      //   }
      // break;

    // [Phase 3] 已迁移到新架构，旧代码保留作参考
    // case StateMachine::TAKEOFF:
    // {
    //   // 旧的 TAKEOFF 逻辑已被 StateTakeoff 类替代
    // }
    // break;

    case StateMachine::WAYPOINT:
    {
      // Follow the waypoints in the YAML file 
      // If the mission is completed ask the user to switch to next state
      
      bool arrivedWP = false;
      bool stoppedWP = false;
      arrivedWP = wpConvergence( current_pose_, des_pose_);
      stoppedWP = wpStopped();

      if(arrivedWP && stoppedWP && current_wp_ < n_wp_-1){
        // Arrived at waypoint, ready to send next waypoint
        ROS_INFO("[WAYPOINT] Arrived at WP %d!", current_wp_);
        if(getInput() == 0){
            // Increment index
            current_wp_++;
            sendNextWaypoint(current_wp_);
          }  
        requestKeyboardInput();
      } else if ( stoppedWP && (current_wp_ == 0)){ 
        // Initial waypoint, ready to start the mission
        ROS_INFO("[WAYPOINT] Press ENTER to start waypoint mission!");
        if(getInput() == 0){
          sendNextWaypoint(current_wp_);          
        }
        requestKeyboardInput();
      } else if (arrivedWP && stoppedWP && (current_wp_ == n_wp_-1)){
        // Final waypoint, ready to switch state
        ROS_INFO("[WAYPOINT] Mission completed!");
        if(getInput() == 0)
          switch (perching_type_)
          {
          case 1:
            state_ = TRAJECTORY;
            break;
          case 2:
            state_ = PERCHING;
            actuator_control_msg_.data = 2; // PREPARE
            break;
          case 3:
            state_ = PERCHING;
            actuator_control_msg_.data = 2; // PREPARE
            break;
          default:
            state_ = TRAJECTORY;
            break;
          }
        requestKeyboardInput();
      } else {
        // Flying to target waypoint
        ROS_INFO("[WAYPOINT] Flying to WP %d ...", current_wp_);
        setInput(100);
      }
      
      }
    break;

    case StateMachine::TRAJECTORY:
    {
      // Trigger the planner for the trajectory 
      if (!trajectory_cmd_.response.success){                                       //轨迹响应未成功
        planner_client_.call(trajectory_cmd_);
        traj_timer_.startTimer();
      } else if (trajectory_cmd_.response.success && traj_timer_.getTime()>2) {     //成功且超时2秒
        ROS_INFO("[TRAJECTORY] : Following trajectory press ENTER to abort!");
        if(getInput() == 0)
            state_ = LANDING;
        requestKeyboardInput(); 
      } else {
        ROS_INFO("[TRAJECTORY] : Waiting ...");
        setInput(100);
      }

      }
    break;
    
    case StateMachine::LANDING:
    {
      // Lands at the current position
      if (!land_cmd_.response.success){
        land_client_.call(land_cmd_);
        }
      
      // If the system status is MAV_STATE_ACTIVE
      if (current_state_.system_status == 4){
        ROS_INFO("[LANDING] : Going down ...");
      } else {
        ROS_INFO("[LANDING] : Landing completed!");
        actuator_control_msg_.data = 0; // IDLE 空闲
        } 

      }
    break;

    case StateMachine::PERCHING:
    {
      // Trigger the planner for the perching trajectory 
      if (!perching_trj_cmd_.response.success){                                      //若规划指令未成功，调用规划器并启动计时器
        planner_client_.call(perching_trj_cmd_);
        traj_timer_.startTimer();
      } else if (perching_trj_cmd_.response.success && traj_timer_.getTime()>1) {
        ROS_INFO("[PERCHING] : Following trajectory press ENTER to abort!");
        if(getInput() == 0)
            state_ = LANDING;
        requestKeyboardInput();
      } else {
        ROS_INFO("[PERCHING] : Waiting ...");
        setInput(100);
      }

      }
    break;

    case StateMachine::ATRAJECTORY:
    {
      // A* 轨径规划状态
      if (!astar_trajectory_cmd_.response.success){
        ROS_INFO("[ATRAJECTORY] Generating A* trajectory...");
        // 如果收到了目标点，将目标点信息传递给planner服务
        if (goal_received_) {
            astar_trajectory_cmd_.request.x = goal_pose_.pose.position.x;
            astar_trajectory_cmd_.request.y = goal_pose_.pose.position.y;
            astar_trajectory_cmd_.request.z = goal_pose_.pose.position.z;
            // 简化处理偏航角
            tf::Quaternion q(goal_pose_.pose.orientation.x,
                           goal_pose_.pose.orientation.y,
                           goal_pose_.pose.orientation.z,
                           goal_pose_.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            astar_trajectory_cmd_.request.h = yaw;
        }
        planner_client_.call(astar_trajectory_cmd_);
        traj_timer_.startTimer();
      } else if (astar_trajectory_cmd_.response.success && traj_timer_.getTime()>2) {
        // ROS_INFO("[ATRAJECTORY] Following trajectory, press ENTER to abort!");
        // if(getInput() == 0)
        //     state_ = LANDING;
        // requestKeyboardInput();
        //判断是否到达终点
        bool arrivedGoal = false;
        if (goal_received_){
          arrivedGoal = wpConvergence( current_pose_, goal_pose_);
        }

        if (arrivedGoal){
          ROS_INFO("[ATRAJECTORY] Reached goal, press ENTER to abort!");
          if(getInput() == 0)
            state_ = LANDING;
          requestKeyboardInput();
        } else {
          ROS_INFO("[ATRAJECTORY] Following trajectory, press ENTER to abort!");
          if(getInput() == 0)
            state_ = LANDING;
          requestKeyboardInput();
        }

      } else {
        ROS_INFO("[ATRAJECTORY] Waiting for trajectory generation...");
        setInput(100);
      }
      break;
    }

    // [Phase 3] 已迁移到新架构，旧代码保留作参考
    // case StateMachine::OBJ_PLANNER:
    // {
    //   // 旧的 OBJ_PLANNER 逻辑已被 StateObjPlanner 类替代
    // }
    // break;
    }
  }
  }

  void StateMachine::publishCommand()
  {
    switch (state_)
    {

    case StateMachine::ARMED:
    {
      // Parallel thread for continuous tasks  
      }
    break;

    case StateMachine::TAKEOFF:
    {
      // Parallel thread for continuous tasks
      actuator_control_msg_.data = 1; // FLIGHT
      actuator_control_pub_.publish(actuator_control_msg_);
      }
    break;

    case StateMachine::WAYPOINT:
    { 
      // Parallel thread for continuous tasks
      }
    break;

    case StateMachine::TRAJECTORY:
    {
      // Parallel thread for continuous tasks
      }
    break;

    case StateMachine::LANDING:
    {
      // Parallel thread for continuous tasks
      actuator_control_pub_.publish(actuator_control_msg_);
      }
    break;

    case StateMachine::PERCHING:
    {
      // 2: vertical perching, 3: inclined perching
      if (perching_type_ == 2){
        timeToContact_msg_.data = timeToContact(current_pose_, ver_perch_pose_);      //计算垂直接触时间
        ttc_threshold_ = ttc_threshold_ver_;
        perching_pose_pub_.publish(ver_perch_pose_);
      } else if (perching_type_ == 3){
        timeToContact_msg_.data = timeToContact(current_pose_, inc_perch_pose_);      //计算倾斜接触时间
        ttc_threshold_ = ttc_threshold_inc_;
        perching_pose_pub_.publish(inc_perch_pose_);
      }
      
      // For visualization
      if (timeToContact_msg_.data >= ttc_threshold_ && !force_disarm_cmd_.response.success){
        timeToContact_pub_.publish(timeToContact_msg_);
      }

      // && !force_disarm_cmd_.response.success
      if (timeToContact_msg_.data < ttc_threshold_ && !force_disarm_cmd_.response.success){
        force_disarm_client_.call(force_disarm_cmd_);
        actuator_control_msg_.data = 3; // CLOSE
        perching_timer_.startTimer();
        ROS_ERROR("[PERCHING]: Motors disarmed!");
      }

      if(perching_timer_.getTime() > 2 && force_disarm_cmd_.response.success){
        // Send release command to dynamixel
        ROS_WARN("[PERCHING]: Releasing actuators ...");
        actuator_control_msg_.data = 4; // RELEASE
      }
      
      actuator_control_pub_.publish(actuator_control_msg_);

      }
    break;

    // [Phase 3] 已迁移到新架构，发布逻辑在 StateObjPlanner::execute() 中处理
    // case StateMachine::OBJ_PLANNER:
    // {
    //   // 旧的发布逻辑已被 StateObjPlanner 类替代
    // }
    // break;

    // 【新增】EMERGENCY_HOVER 的 case
    // 由于控制逻辑（发布 setpoint）已经完全移交给 StateEmergencyHover::execute()，
    // 这里留空即可，或者打印调试信息。
    case StateMachine::EMERGENCY_HOVER:
    {
      // Logic handled in StateEmergencyHover::execute()
    }
    break;

    }
  }
  
  void StateMachine::sendNextWaypoint(int index)
  {
    waypoint_cmd_.request.type = 0;
    waypoint_cmd_.request.x = wp_matrix_(0, index);
    waypoint_cmd_.request.y = wp_matrix_(1, index);
    waypoint_cmd_.request.z = wp_matrix_(2, index);
    waypoint_cmd_.request.h = wp_matrix_(3, index);

    // Set desired pose for waypoint convergence check
    des_pose_.pose.position.x = waypoint_cmd_.request.x;
    des_pose_.pose.position.y = waypoint_cmd_.request.y;
    des_pose_.pose.position.z = waypoint_cmd_.request.z;

    planner_client_.call(waypoint_cmd_);                             //调用规划客户端发送请
  }

  void StateMachine::readWaypoints()
  {
    // Read the waypoints from YAML file
    double pos_x_wp;
    double pos_y_wp;
    double pos_z_wp;
    double pos_h_wp;
    bool no_fail = true;

    ROS_INFO("[INIT]: Reading waypoints ...");
    while(ros::ok() && no_fail){
      
      std::string idx = "/"+ std::to_string(n_wp_) + "/";
      std::string wp_str = "planner/waypoints" + idx;

      no_fail = nh_.getParam(wp_str + "pos/x", pos_x_wp);                //获取x坐标参数

      if (!no_fail){        
        //ROS_INFO("[INIT]: Number of waypoints: %d", n_wp_);            //失败，记录总路径点数
        break;
      }else{ 
        n_wp_++;                                                         //成功，路径点数加1
      }
    }

    wp_matrix_.resize(4, n_wp_);                                         //将所有点存入4行n列的矩阵wp_matrix_中
    
    for (int i = 0; i < n_wp_; i++){
      
      std::string idx = "/"+ std::to_string(i) + "/";
      //std::string wp_str = ros::this_node::getName() + "/waypoints" + idx;
      std::string wp_str = "planner/waypoints" + idx;

      no_fail = nh_.getParam(wp_str + "pos/x", pos_x_wp);
      no_fail = nh_.getParam(wp_str + "pos/y", pos_y_wp);
      no_fail = nh_.getParam(wp_str + "pos/z", pos_z_wp);
      no_fail = nh_.getParam(wp_str + "pos/h", pos_h_wp);
      
      if(no_fail){
        wp_matrix_.col(i) << pos_x_wp, pos_y_wp, pos_z_wp, pos_h_wp;      //成功，存入矩阵列
      }else{
        ROS_ERROR("[INIT]: Reading waypoints failed!");
          
      }
    }

    ROS_INFO("[INIT]: Waypoints ready");

  }

  //判断两个三维位置（pose1和pose2）的欧氏距离是否小于设定阈值，用于检测目标点是否收敛
  bool StateMachine::wpConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  {
    
    Vec3 v1;
    Vec3 v2;

    v1(0) = pose1.pose.position.x;
    v1(1) = pose1.pose.position.y;
    v1(2) = pose1.pose.position.z;

    v2(0) = pose2.pose.position.x;
    v2(1) = pose2.pose.position.y;
    v2(2) = pose2.pose.position.z;

    //wp_convergence_threshold
    bool converged = (v1 - v2).norm() < wp_converged_threshold_;           //计算两个坐标向量的差值的范数，并与阈值比较

    return converged;
  }

  //无人机是否在全局坐标系下速度低于阈值
  bool StateMachine::wpStopped()
  {
    Eigen::Vector3d quad_vel_body;                                         //无人机在局部坐标系下的速度
    quad_vel_body(0) = current_vel_.twist.linear.x;
    quad_vel_body(1) = current_vel_.twist.linear.y;
    quad_vel_body(2) = current_vel_.twist.linear.z;
    
    Eigen::Quaterniond quad_att(                                           //无人机的姿态四元数
      current_pose_.pose.orientation.w,
      current_pose_.pose.orientation.x,
      current_pose_.pose.orientation.y,
      current_pose_.pose.orientation.z);
    
    Eigen::Matrix3d quad_rot;
    quad_rot = quad_att.toRotationMatrix();                                //四元数转旋转矩阵
    
    Eigen::Vector3d quad_vel;
    quad_vel = quad_rot * quad_vel_body;                                   //旋转矩阵乘以速度向量，将速度转换到全局坐标系
    
    bool stopped = quad_vel.norm() < wp_converged_threshold_;              //计算速度模并与阈值比较，判断是否停止

    return stopped;
  }
  
  //计算无人机到达目标点的预计时间
  double StateMachine::timeToContact(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  { 
    Vec3 v1;
    v1(0) = pose1.pose.position.x;
    v1(1) = pose1.pose.position.y;
    v1(2) = pose1.pose.position.z;

    Vec3 target;
    target(0) = pose2.pose.position.x;
    target(1) = pose2.pose.position.y;
    target(2) = pose2.pose.position.z;

    Vec3 quad_vel_body;                                                    //获取当前机体速度
    quad_vel_body(0) = current_vel_.twist.linear.x;
    quad_vel_body(1) = current_vel_.twist.linear.y;
    quad_vel_body(2) = current_vel_.twist.linear.z;
    
    Eigen::Quaterniond quad_att(
    current_pose_.pose.orientation.w,
    current_pose_.pose.orientation.x,
    current_pose_.pose.orientation.y,
    current_pose_.pose.orientation.z);
    
    Eigen::Matrix3d quad_rot;
    quad_rot = quad_att.toRotationMatrix();
    
    Vec3 quad_vel;
    quad_vel = quad_rot * quad_vel_body;                                   //转换速度到世界坐标系

    double absVelocity = quad_vel.norm();                                  //计算速度模长、位置间距和高差
    double absDistance = (v1 - target).norm();
    double altitudeDifference = v1(2) - target(2);  

    // relevant only if the quadrotor is above the target point
    // and the altitude difference is less than 1 m    若无人机高于目标且高差在1米内返回1，否则返回距离/速度
    return (altitudeDifference < 0 && altitudeDifference > 1) ? 1 : (absDistance / absVelocity);
  }

  void StateMachine::initializeFCU()
  {
    // FCU Initialisation Sequence 
    ros::Time last_request = ros::Time::now();

    // Wait for FCU connection before publishing anything  等待FCU连接
    while(ros::ok() && !current_state_.connected && 
    (ros::Time::now() - last_request > ros::Duration(5.0))){                        //等待连接且未超时
      ros::spinOnce();
      rate_.sleep();
      ROS_INFO("[INIT]: Connecting to FCT ...");
    }
    
    // Send a few setpoints before starting,   发送初始化设定点
    // Otherwise cant switch to Offboard
    for(int i = 100; ros::ok() && i > 0; --i){                                       //发送100次零位设定点
      local_pos_pub_.publish(zero_pose_);
      ros::spinOnce();
      rate_.sleep();
    }

    // Timer used to prevent FCU overflowing
    last_request = ros::Time::now();
        
    local_pos_pub_.publish(zero_pose_);

    // Change to Offboard mode and arm w. time gaps
    while(ros::ok() && !current_state_.armed){                                      //设备未解锁？
      if( current_state_.mode != "OFFBOARD" &&                                      //处于Offboard模式？
      (ros::Time::now() - last_request > ros::Duration(FCU_TIME_BUFFER))){
          if( set_mode_client_.call(offb_mode_cmd_) && offb_mode_cmd_.response.mode_sent){
              ROS_INFO("[INIT]: Offboard enabled");
          }
          last_request = ros::Time::now();
      } else {
          if( current_state_.mode == "OFFBOARD" && !current_state_.armed &&          //尝试切换模式
              (ros::Time::now() - last_request > ros::Duration(FCU_TIME_BUFFER))){   //更新计时并发布设定点
              if( arming_client_.call(arm_cmd_) &&
                  arm_cmd_.response.success){
                  ROS_INFO("[INIT]: Vehicle armed");
              }
              last_request = ros::Time::now();
          }
      }
      local_pos_pub_.publish(zero_pose_);
      ros::spinOnce();
      rate_.sleep();
    }   
  }

  void StateMachine::initializePublishers()
  {
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    controller_trigger_pub_ = nh_.advertise<std_msgs::Bool>
            ("geometric_controller/start_trigger", 1);
    rpy_pub_ = nh_.advertise<geometry_msgs::Vector3>
            ("quadrotor/RPY", 1);
    timeToContact_pub_ = nh_.advertise<std_msgs::Float64>
            ("quadrotor/TTC", 1);
    actuator_control_pub_ = nh_.advertise<std_msgs::Int32>
            ("actuator_control/state", 1);
    perching_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("quadrotor/PerchingPose", 1);
    // 【Phase 3】FlatTarget 发布器，用于 ObjPlanner 轨迹跟踪
    flat_target_pub_ = nh_.advertise<controller_msgs::FlatTarget>
            ("reference/flatsetpoint", 1);
    // 【Simulated Sensor FOV】传感器FOV可视化发布器 (扇形填充+轮廓合并发布)
    sensor_fov_vis_pub_ = nh_.advertise<visualization_msgs::Marker>
            ("state_machine/sensor_fov", 1);


  }

  void StateMachine::initializeSubscribers()
  {
    state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &StateMachine::stateCallback, this);

    local_pos_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &StateMachine::localposeCallback, this);
    
    // 【修复】订阅 Local (ENU) 坐标系速度，而不是 Body 坐标系
    // velocity_body: 机体坐标系 (X=前, Y=左, Z=上)
    // velocity_local: 世界坐标系 (ENU: X=东, Y=北, Z=上) - EgoPlanner需要这个
    local_pos_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 10, &StateMachine::localvelCallback, this);
    // 添加目标点订阅
    goal_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("planner/goal_pose", 10, &StateMachine::goalPoseCallback, this);

    // 不再订阅点云，改为直接使用planner中的障碍物数据
    // obstacle_sub_ 已移除

  }

  void StateMachine::initializeServices()
  {
    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    planner_client_ = nh_.serviceClient<planner::GetTrajectory>
            ("planner/trigger");
    
    force_disarm_client_ = nh_.serviceClient<mavros_msgs::CommandLong>
            ("mavros/cmd/command");

  }

  void StateMachine::initializeParameters()
  {
    // Commands for mavros services
    offb_mode_cmd_.request.custom_mode = "OFFBOARD";

    arm_cmd_.request.value = true;

    land_cmd_.request.yaw = 0;
    land_cmd_.request.latitude = 0;
    land_cmd_.request.longitude = 0;
    land_cmd_.request.altitude = 0;

    force_disarm_cmd_.request.broadcast = false;
    force_disarm_cmd_.request.command = 400;
    force_disarm_cmd_.request.confirmation = 0;
    force_disarm_cmd_.request.param1 = 0.0;
    force_disarm_cmd_.request.param2 = 21196.0;
    force_disarm_cmd_.request.param3 = 0.0;
    force_disarm_cmd_.request.param4 = 0.0;
    force_disarm_cmd_.request.param5 = 0.0;
    force_disarm_cmd_.request.param6 = 0.0;
    force_disarm_cmd_.request.param7 = 0.0;

    // Initialize the variables
    state_ = ARMED;
    n_wp_ = 0;
    current_wp_ = 0;
        
    // ENU frame is used -> PX4 transforms to NED
    des_pose_.pose.position.x = 0;
    des_pose_.pose.position.y = 0;
    des_pose_.pose.position.z = 0;

    zero_pose_.pose.position.x = 0;
    zero_pose_.pose.position.y = 0;
    zero_pose_.pose.position.z = 0;

    // Read the take-off pose
    nh_.getParam(ros::this_node::getName() + "/takeoff/x", takeoff_pose_.pose.position.x);
    nh_.getParam(ros::this_node::getName() + "/takeoff/y", takeoff_pose_.pose.position.y);
    nh_.getParam(ros::this_node::getName() + "/takeoff/z", takeoff_pose_.pose.position.z);
    nh_.getParam(ros::this_node::getName() + "/takeoff/h", globaltakeoff_cmd_.request.h);

    // Pose arguments are not used if type == 1
    trajectory_cmd_.request.type = 1; 
    trajectory_cmd_.request.x = -1; 
    trajectory_cmd_.request.y = -1;
    trajectory_cmd_.request.z = -1;
    trajectory_cmd_.request.h = -1;

    // Threshold values, user configurable
    nh_.getParam(ros::this_node::getName() + "/wp_convergence_threshold", wp_converged_threshold_);
    nh_.getParam(ros::this_node::getName() + "/wp_stopped_threshold", wp_stopped_threshold_);
    
    // Perching configuration

    // Time to contact
    nh_.getParam(ros::this_node::getName() + "/ttc_threshold_ver", ttc_threshold_ver_);
    nh_.getParam(ros::this_node::getName() + "/ttc_threshold_inc", ttc_threshold_inc_);

    // Read vertical perching pose
    nh_.getParam("/planner/waypoints_vertical_perching/3/pos/x", ver_perch_pose_.pose.position.x);
    nh_.getParam("/planner/waypoints_vertical_perching/3/pos/y", ver_perch_pose_.pose.position.y);
    nh_.getParam("/planner/waypoints_vertical_perching/3/pos/z", ver_perch_pose_.pose.position.z);

    // Read vertical perching pose
    nh_.getParam("/planner/waypoints_inclined_perching/0/pos/x", inc_perch_pose_.pose.position.x);
    nh_.getParam("/planner/waypoints_inclined_perching/0/pos/y", inc_perch_pose_.pose.position.y);
    nh_.getParam("/planner/waypoints_inclined_perching/0/pos/z", inc_perch_pose_.pose.position.z);

    // Get the perching type
    nh_.getParam(ros::this_node::getName() + "/perching_type", perching_type_);

    perching_trj_cmd_.request.type = perching_type_; 
    perching_trj_cmd_.request.x = -1; 
    perching_trj_cmd_.request.y = -1;
    perching_trj_cmd_.request.z = -1;
    perching_trj_cmd_.request.h = -1;

    // Messages
    controller_activation_.data = false;

    // 初始化 A* 相关变量
    goal_received_ = false;
    // A* 轨迹命令初始化
    astar_trajectory_cmd_.request.type = 4;  // 对应 ATRAJECTORY 状态
    astar_trajectory_cmd_.request.x = -1;
    astar_trajectory_cmd_.request.y = -1;
    astar_trajectory_cmd_.request.z = -1;
    astar_trajectory_cmd_.request.h = -1;

    // 初始化 ObjPlanner 相关变量
    egoplanner_initialized_ = false;
    egoplanner_goal_reached_ = false;
    egoplanner_wrapper_ = nullptr;
    has_active_egoplanner_traj_ = false;
    replan_needed_ = true;  // 初始化时需要规划

    // 【Simulated Sensor FOV】初始化传感器参数 (扇形感知模型)
    nh_.param(ros::this_node::getName() + "/sensor/range", sensor_range_, 50.0);
    nh_.param(ros::this_node::getName() + "/sensor/fov", sensor_fov_, 120.0);
    nh_.param(ros::this_node::getName() + "/sensor/height", sensor_height_, 10.0);
    obstacles_loaded_ = false;

    // 转换为弧度
    sensor_fov_rad_ = sensor_fov_ * M_PI / 180.0;

    ROS_INFO("[SENSOR_FOV]: Sector perception model initialized:");
    ROS_INFO("[SENSOR_FOV]:   - Range: %.2f meters", sensor_range_);
    ROS_INFO("[SENSOR_FOV]:   - FOV: %.2f degrees (%.3f radians)", sensor_fov_, sensor_fov_rad_);
    ROS_INFO("[SENSOR_FOV]:   - Height: %.2f meters (+/-%.2f)", sensor_height_, sensor_height_ / 2.0);
  }

  int StateMachine::getInput()
  {
    int input;
    input_mutex_.lock();
    input = input_;
    input_mutex_.unlock();
    return input;
  }

  void StateMachine::requestKeyboardInput()
  {
    if (request_input_)
      return;
    setInput(100);
    std::thread keyboard(&StateMachine::getKeyboardInput, this);
    keyboard.detach();
  }

  void StateMachine::setInput(int input)
  {
    input_mutex_.lock();
    input_ = input;
    input_mutex_.unlock();
  }

  void StateMachine::getKeyboardInput()
  {
    request_input_ = true;
    int input;
    std::cin.ignore();
    setInput(0);
    request_input_ = false;
  }

  // 添加目标点回调函数
  void StateMachine::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
      goal_pose_ = *msg;
      goal_received_ = true;
      replan_needed_ = true;  // 【修复】标记需要重新规划
      ROS_INFO("[STATE] Goal received: x=%.2f, y=%.2f, z=%.2f",
              goal_pose_.pose.position.x,
              goal_pose_.pose.position.y,
              goal_pose_.pose.position.z);
  }

  // 障碍物点云回调函数 - 已移除，不再使用点云数据
  // 我们直接使用planner中已配置的障碍物数据

  // 【Simulated Sensor FOV】从参数服务器加载所有障碍物（一次性）
  void StateMachine::loadObstacles()
  {
    if (obstacles_loaded_)
    {
      ROS_WARN("[SENSOR_FOV]: Obstacles already loaded, skipping...");
      return;
    }

    ROS_INFO("[SENSOR_FOV]: Loading obstacles from parameter server...");

    // 清空现有障碍物列表
    global_box_obstacles_.clear();
    global_sphere_obstacles_.clear();

    // 从参数服务器读取障碍物配置
    XmlRpc::XmlRpcValue obstacle_list;

    // 1. 获取当前节点的名称 (例如 "/state_machine")
    std::string node_name = ros::this_node::getName();

    // 2. 拼接完整的参数路径 (例如 "/state_machine/obstacles")
    std::string param_path = node_name + "/obstacles";

    // 3. 尝试读取参数
    // 先试私有路径 /state_machine/obstacles，如果失败再试全局路径 /obstacles
    bool param_read_success = nh_.getParam(param_path, obstacle_list) ||
                               nh_.getParam("/obstacles", obstacle_list);

    if (!param_read_success)
    {
      ROS_ERROR("[SENSOR_FOV]: Failed to read obstacles from %s or /obstacles",
                param_path.c_str());
      return;
    }

    ROS_INFO("[SENSOR_FOV]: Successfully loaded obstacle list from %s",
             param_path.c_str());

    // 解析障碍物列表
    for (int i = 0; i < obstacle_list.size(); ++i)
    {
      XmlRpc::XmlRpcValue obstacle = obstacle_list[i];

      if (obstacle.hasMember("type"))
      {
        std::string type = obstacle["type"];

        if (type == "box")
        {
          ObjPlannerWrapper::BoxObstacle box;
          box.x = obstacle["x"];
          box.y = obstacle["y"];
          box.z = obstacle["z"];
          box.width = obstacle["width"];
          box.length = obstacle["length"];
          box.height = obstacle["height"];
          global_box_obstacles_.push_back(box);

          ROS_INFO("[SENSOR_FOV]: Loaded BOX obstacle: center=(%.2f, %.2f, %.2f), size=(%.2f, %.2f, %.2f)",
                   box.x, box.y, box.z, box.width, box.length, box.height);
        }
        else if (type == "sphere")
        {
          ObjPlannerWrapper::SphereObstacle sphere;
          sphere.x = obstacle["x"];
          sphere.y = obstacle["y"];
          sphere.z = obstacle["z"];
          sphere.radius = obstacle["radius"];
          global_sphere_obstacles_.push_back(sphere);

          ROS_INFO("[SENSOR_FOV]: Loaded SPHERE obstacle: center=(%.2f, %.2f, %.2f), radius=%.2f",
                   sphere.x, sphere.y, sphere.z, sphere.radius);
        }
      }
    }

    obstacles_loaded_ = true;
    ROS_INFO("[SENSOR_FOV]: Obstacle loading complete! Total: %zu box, %zu sphere",
             global_box_obstacles_.size(), global_sphere_obstacles_.size());
  }

  // 初始化 ObjPlanner
  void StateMachine::initializeObjPlanner()
  {
    if (egoplanner_initialized_)
    {
      ROS_WARN("[OBJPLANNER]: Already initialized");
      return;
    }

    ROS_INFO("[OBJPLANNER]: Initializing ObjPlanner wrapper...");

    // 创建 ObjPlanner 包装器实例
    egoplanner_wrapper_ = std::make_shared<ObjPlannerWrapper>();

    // 从参数服务器读取 ObjPlanner 配置
    double max_vel, max_acc, max_jerk;
    double map_size, map_height, map_resolution, map_inflate;

    // 确保请求的是 3.0 m/s (这是实际飞行的目标速度)
    nh_.param(ros::this_node::getName() + "/obj_planner/max_vel", max_vel, 3.0);
    // 加速度请求给 3.0，保证起步有力
    nh_.param(ros::this_node::getName() + "/obj_planner/max_acc", max_acc, 3.0);
    nh_.param(ros::this_node::getName() + "/obj_planner/max_jerk", max_jerk, 2.0);
    nh_.param(ros::this_node::getName() + "/obj_planner/map_size", map_size, 100.0);
    nh_.param(ros::this_node::getName() + "/obj_planner/map_height", map_height, 10.0);
    nh_.param(ros::this_node::getName() + "/obj_planner/map_resolution", map_resolution, 0.4);
    nh_.param(ros::this_node::getName() + "/obj_planner/map_inflate", map_inflate, 0.2);

    // 初始化 ObjPlanner
    if (egoplanner_wrapper_->init(max_vel, max_acc, max_jerk, map_size, map_height, map_resolution, map_inflate))
    {
      egoplanner_initialized_ = true;
      ROS_INFO("[OBJPLANNER]: Initialization successful!");

      // 【Simulated Sensor FOV】加载全局障碍物（一次性）
      egoplanner_wrapper_->loadObstacles(nh_);
    }
    else
    {
      ROS_ERROR("[OBJPLANNER]: Initialization failed!");
    }
  }

  // 更新 ObjPlanner 状态
  void StateMachine::updateObjPlannerState()
  {
    if (!egoplanner_initialized_ || egoplanner_wrapper_ == nullptr)
    {
      ROS_WARN_THROTTLE(2.0, "[OBJPLANNER]: Not initialized, cannot update state");
      return;
    }

    // 更新无人机状态
    egoplanner_wrapper_->updateDroneState(current_pose_, current_vel_);

    // 设置目标点
    if (goal_received_)
    {
      egoplanner_wrapper_->setTarget(goal_pose_);
    }

    // 【Simulated Sensor FOV】扇形感知模型 (类LiDAR)
    // 使用传感器范围、角度和高度过滤障碍物
    std::vector<ObjPlannerWrapper::BoxObstacle> local_box_obstacles;
    std::vector<ObjPlannerWrapper::SphereObstacle> local_sphere_obstacles;

    // 获取无人机当前位置和偏航角
    Eigen::Vector3d drone_pos;
    drone_pos(0) = current_pose_.pose.position.x;
    drone_pos(1) = current_pose_.pose.position.y;
    drone_pos(2) = current_pose_.pose.position.z;

    double drone_yaw = current_RPY_(2);  // 当前偏航角 (弧度)

    // 检查障碍物是否已加载
    if (!obstacles_loaded_)
    {
      ROS_WARN_THROTTLE(2.0, "[SENSOR_FOV]: Obstacles not loaded yet!");
      return;
    }

    // 遍历全局盒状障碍物，过滤出传感器范围内的障碍物
    int box_count_in_range = 0;
    for (const auto& box : global_box_obstacles_)
    {
      // 计算障碍物中心到无人机的2D水平距离
      double dx = box.x - drone_pos(0);
      double dy = box.y - drone_pos(1);
      double dz = box.z - drone_pos(2);
      double distance_2d = std::sqrt(dx * dx + dy * dy);

      // 【条件1】距离检查
      if (distance_2d >= sensor_range_)
      {
        continue;  // 超出传感器范围，跳过
      }

      // 【条件2】高度检查 (垂直范围 +/- sensor_height_/2)
      if (std::abs(dz) > sensor_height_ / 2.0)
      {
        continue;  // 超出垂直感知范围，跳过
      }

      // 【条件3】角度检查 (水平FOV)
      // 计算障碍物相对无人机的角度
      double obstacle_angle = std::atan2(dy, dx);
      double angle_diff = obstacle_angle - drone_yaw;

      // 归一化角度到 [-PI, PI]
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

      // 检查是否在FOV范围内
      if (std::abs(angle_diff) > sensor_fov_rad_ / 2.0)
      {
        continue;  // 超出水平FOV，跳过
      }

      // 所有条件满足，添加到局部障碍物列表
      local_box_obstacles.push_back(box);
      box_count_in_range++;
    }

    // 遍历全局球状障碍物，过滤出传感器范围内的障碍物
    int sphere_count_in_range = 0;
    for (const auto& sphere : global_sphere_obstacles_)
    {
      // 计算障碍物中心到无人机的2D水平距离
      double dx = sphere.x - drone_pos(0);
      double dy = sphere.y - drone_pos(1);
      double dz = sphere.z - drone_pos(2);
      double distance_2d = std::sqrt(dx * dx + dy * dy);

      // 【条件1】距离检查
      if (distance_2d >= sensor_range_)
      {
        continue;  // 超出传感器范围，跳过
      }

      // 【条件2】高度检查
      if (std::abs(dz) > sensor_height_ / 2.0)
      {
        continue;  // 超出垂直感知范围，跳过
      }

      // 【条件3】角度检查
      double obstacle_angle = std::atan2(dy, dx);
      double angle_diff = obstacle_angle - drone_yaw;

      // 归一化角度
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

      if (std::abs(angle_diff) > sensor_fov_rad_ / 2.0)
      {
        continue;  // 超出水平FOV，跳过
      }

      // 所有条件满足
      local_sphere_obstacles.push_back(sphere);
      sphere_count_in_range++;
    }

    ROS_DEBUG_THROTTLE(2.0, "[SENSOR_FOV]: Sector (yaw=%.1f°, fov=%.1f°, range=%.1fm, h=%.1fm): %d/%zu box, %d/%zu sphere",
                       drone_yaw * 180.0 / M_PI, sensor_fov_, sensor_range_, sensor_height_,
                       box_count_in_range, global_box_obstacles_.size(),
                       sphere_count_in_range, global_sphere_obstacles_.size());

    // 更新过滤后的障碍物到ObjPlanner
    egoplanner_wrapper_->updateObstacles(local_box_obstacles, local_sphere_obstacles, drone_pos);
  }

  // 发布 ObjPlanner 轨迹到控制器
  void StateMachine::publishObjPlannerTrajectory()
  {
    if (!egoplanner_initialized_ || egoplanner_wrapper_ == nullptr || !has_active_egoplanner_traj_)
    {
      return;
    }

    // 计算时间流逝
    ros::Time now = ros::Time::now();
    double time_elapsed = (now - egoplanner_traj_start_time_).toSec();

    // 获取采样点 (FlatTarget)
    controller_msgs::FlatTarget target_msg;
    bool success = egoplanner_wrapper_->getSetpointAtTime(time_elapsed, target_msg);

    if (success)
    {
      // 【修复】发布给 geometric_controller 订阅的正确话题
      // geometric_controller 订阅的是 "reference/flatsetpoint"
      static ros::Publisher flat_target_pub_ = nh_.advertise<controller_msgs::FlatTarget>(
          "reference/flatsetpoint", 1);

      flat_target_pub_.publish(target_msg);
    }
    else
    {
      // 如果采样失败（例如轨迹为空）
      ROS_WARN_THROTTLE(1.0, "[OBJPLANNER]: Trajectory execution finished or invalid");
    }
  }

  // 【Simulated Sensor FOV】发布传感器扇形感知范围可视化 (类LiDAR)
  void StateMachine::publishSensorFOVVisualization()
  {
    // 获取无人机当前位置和朝向 (Yaw)
    double drone_x = current_pose_.pose.position.x;
    double drone_y = current_pose_.pose.position.y;
    double drone_z = current_pose_.pose.position.z;
    double drone_yaw = current_RPY_(2); // 偏航角

    // ===== 1. 扇形填充 (TRIANGLE_LIST) ID=0 =====
    visualization_msgs::Marker sector_marker;
    sector_marker.header.frame_id = "map";
    sector_marker.header.stamp = ros::Time::now();
    sector_marker.ns = "sensor_fov"; // 命名空间
    sector_marker.id = 0;            // ID 0
    sector_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    sector_marker.action = visualization_msgs::Marker::ADD;

    // 颜色设置：半透明青色
    sector_marker.color.r = 0.0;
    sector_marker.color.g = 1.0;
    sector_marker.color.b = 1.0;
    sector_marker.color.a = 0.2; // 填充透明度

    sector_marker.pose.orientation.w = 1.0;
    sector_marker.scale.x = 1.0;
    sector_marker.scale.y = 1.0;
    sector_marker.scale.z = 1.0;

    // ===== 2. 扇形轮廓 (LINE_STRIP) ID=1 =====
    visualization_msgs::Marker outline_marker = sector_marker;
    outline_marker.id = 1;           // ID 1 (区分ID很重要)
    outline_marker.type = visualization_msgs::Marker::LINE_STRIP;
    outline_marker.color.a = 0.8;    // 轮廓不透明，更清晰
    outline_marker.scale.x = 0.05;   // 线条宽度

    // 几何计算通用参数
    geometry_msgs::Point center;
    center.x = drone_x;
    center.y = drone_y;
    center.z = drone_z;

    int segments = 30; // 扇形分段数，越多越圆滑
    double start_angle = drone_yaw - sensor_fov_rad_ / 2.0;
    double angle_step = sensor_fov_rad_ / segments;

    // --- 构建填充 (Triangle List) ---
    for (int i = 0; i < segments; ++i)
    {
        double ang1 = start_angle + i * angle_step;
        double ang2 = start_angle + (i + 1) * angle_step;

        geometry_msgs::Point p1;
        p1.x = drone_x + sensor_range_ * std::cos(ang1);
        p1.y = drone_y + sensor_range_ * std::sin(ang1);
        p1.z = drone_z;

        geometry_msgs::Point p2;
        p2.x = drone_x + sensor_range_ * std::cos(ang2);
        p2.y = drone_y + sensor_range_ * std::sin(ang2);
        p2.z = drone_z;

        // 一个三角形由3个点组成：圆心 -> 点1 -> 点2
        sector_marker.points.push_back(center);
        sector_marker.points.push_back(p1);
        sector_marker.points.push_back(p2);
    }

    // --- 构建轮廓 (Line Strip) ---
    // 关键修复：从圆心开始画
    outline_marker.points.push_back(center);

    // 画圆弧边缘 (从左到右)
    for (int i = 0; i <= segments; ++i)
    {
        double ang = start_angle + i * angle_step;
        geometry_msgs::Point p;
        p.x = drone_x + sensor_range_ * std::cos(ang);
        p.y = drone_y + sensor_range_ * std::sin(ang);
        p.z = drone_z;
        outline_marker.points.push_back(p);
    }

    // 关键修复：最后连回圆心，闭合图形
    outline_marker.points.push_back(center);

    // 发布 (全部发给 sensor_fov_vis_pub_)
    // 这样在 RViz 里订阅 "state_machine/sensor_fov" 就能同时看到两个了
    sensor_fov_vis_pub_.publish(sector_marker);
    sensor_fov_vis_pub_.publish(outline_marker);
  }

  void StateMachine::initializeContext()
  {
    // 绑定基础资源
    ctx_.nh = &nh_;

    // 绑定感知数据 (指针指向成员变量的地址)
    ctx_.current_pose = &current_pose_;
    ctx_.current_vel = &current_vel_;
    ctx_.current_state = &current_state_;

    // 绑定任务数据
    ctx_.goal_pose = &goal_pose_;
    ctx_.goal_received = &goal_received_;

    // 绑定控制接口
    ctx_.local_pos_pub = &local_pos_pub_;
    ctx_.actuator_control_pub = &actuator_control_pub_;
    ctx_.flat_target_pub = &flat_target_pub_;
    ctx_.controller_trigger_pub = &controller_trigger_pub_;  // 【Phase 3.5】绑定控制器触发器
    ctx_.planner_client = &planner_client_;
    ctx_.set_mode_client = &set_mode_client_;  // 【新增】绑定模式切换客户端

    // 绑定算法模块
    ctx_.egoplanner_wrapper = egoplanner_wrapper_;

    // 绑定键盘输入接口
    ctx_.get_input = [this]() { return this->getInput(); };
    ctx_.set_input = [this](int val) { this->setInput(val); };
    ctx_.request_input = [this]() { this->requestKeyboardInput(); };

    // [论文 3.3.2] 状态池构建 (State Pool Construction)
    // 利用多态特性，将具体状态实例存入 map
    state_pool_["ARMED"] = std::make_shared<StateArmed>();        // 【新增】注册准备状态
    state_pool_["TAKEOFF"] = std::make_shared<StateTakeoff>();
    state_pool_["OBJ_PLANNER"] = std::make_shared<StateObjPlanner>();
    state_pool_["LANDING"] = std::make_shared<StateLanding>();     // 【新增】注册降落状态

    // 【新增】注册紧急悬停状态
    state_pool_["EMERGENCY_HOVER"] = std::make_shared<StateEmergencyHover>();

    // 可以在这里设置初始状态指针，用于测试
    // current_state_ptr_ = state_pool_["OBJ_PLANNER"];

    ROS_INFO("[INIT]: State Context Initialized (Thesis Framework v1.0)");
    ROS_INFO("[INIT]: State Pool initialized with %lu states.", state_pool_.size());
  }

  void StateMachine::changeState(const std::string& new_state_id)
  {
    // 1. 检查目标状态是否存在
    if (state_pool_.find(new_state_id) == state_pool_.end()) {
      ROS_ERROR("[StateMachine] Transition failed: State '%s' not found!", new_state_id.c_str());
      return;
    }

    // 2. 退出当前状态
    if (current_state_ptr_) {
      current_state_ptr_->exit(&ctx_);
    }

    // 3. 切换指针
    current_state_ptr_ = state_pool_[new_state_id];

    // [论文 3.4.1] 状态集成模块：更新状态机内部记录的 ID (兼容旧代码)
    if (new_state_id == "ARMED") state_ = ARMED;
    else if (new_state_id == "TAKEOFF") state_ = TAKEOFF;
    else if (new_state_id == "OBJ_PLANNER") state_ = OBJ_PLANNER;
    else if (new_state_id == "LANDING") state_ = LANDING;
    else if (new_state_id == "WAYPOINT") state_ = WAYPOINT;
    else if (new_state_id == "TRAJECTORY") state_ = TRAJECTORY;
    else if (new_state_id == "PERCHING") state_ = PERCHING;
    else if (new_state_id == "ATRAJECTORY") state_ = ATRAJECTORY;
    // 【新增】正确映射到新的枚举值
    else if (new_state_id == "EMERGENCY_HOVER") state_ = EMERGENCY_HOVER;

    ROS_INFO("[StateMachine] State switched to: %s", new_state_id.c_str());

    // 4. 进入新状态
    if (current_state_ptr_) {
      current_state_ptr_->enter(&ctx_);
    }
  }

} // namespace statemachine

