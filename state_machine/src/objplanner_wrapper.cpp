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
 *
 ****************************************************************************/

#include <state_machine/objplanner_wrapper.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>  // 【新增】用于发布障碍物可视化数组
#include <algorithm>  // 【新增】用于 std::min
#include <cmath>  // 【新增】用于 std::atan2

namespace statemachine
{

  ObjPlannerWrapper::ObjPlannerWrapper()
    : initialized_(false),
      has_valid_trajectory_(false),
      target_set_(false),
      map_size_(100.0),
      map_height_(10.0),
      map_resolution_(0.4),
      map_inflate_(0.2),
      max_vel_(5.0),
      max_acc_(4.0),
      max_jerk_(2.0),
      trajectory_start_time_(ros::Time(0)),  // 【新增】初始化轨迹开始时间
      // === 【新增】感知相关初始化 ===
      sensor_range_(50.0),
      sensor_fov_(120.0),
      sensor_fov_rad_(120.0 * M_PI / 180.0),
      sensor_height_(10.0),
      obstacles_loaded_(false),
      current_yaw_(0.0)
      // ====================================
  {
    // Initialize map origin to center
    map_origin_ = Eigen::Vector3d(-map_size_/2, -map_size_/2, 0.0);

    // Initialize state vectors
    current_pos_ = Eigen::Vector3d::Zero();
    current_vel_ = Eigen::Vector3d::Zero();
    current_acc_ = Eigen::Vector3d::Zero();
    target_pos_ = Eigen::Vector3d::Zero();
    target_vel_ = Eigen::Vector3d::Zero();

    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Wrapper created");
  }

  ObjPlannerWrapper::~ObjPlannerWrapper()
  {
    if (planner_ != nullptr)
    {
      // Planner interface will be automatically cleaned up by shared_ptr
    }
  }

  bool ObjPlannerWrapper::init(double max_vel, double max_acc, double max_jerk,
                               double map_size, double map_height, double map_resolution, double map_inflate)
  {
    if (initialized_)
    {
      ROS_WARN("[OBJ_PLANNER_WRAPPER]: Already initialized");
      return true;
    }

    // ========== 【安全钳制】打印并强制限制参数 ==========
    ROS_WARN("[OBJ_PLANNER_WRAPPER] INPUT PARAMS: vel=%.2f, acc=%.2f, jerk=%.2f, map_size=%.2f, map_height=%.2f",
             max_vel, max_acc, max_jerk, map_size, map_height);

    // 安全钳制：无论传入什么值，强制限制在安全范围内
    // 【虚高设置】给规划器 6.0 的上限，这样 3.0 的轨迹就显得"非常安全"，绝对不会触发刹车
    const double SAFE_MAX_VEL = 6.0;
    // 加速度是锯齿的主要元凶，直接给到 6.0，防止转弯时因为加速度超标而减速
    const double SAFE_MAX_ACC = 6.0;

    max_vel_ = std::min(max_vel, SAFE_MAX_VEL);
    max_acc_ = std::min(max_acc, SAFE_MAX_ACC);
    max_jerk_ = max_jerk; // jerk 不限制
    map_size_ = map_size;
    map_height_ = map_height;
    map_resolution_ = map_resolution;
    map_inflate_ = map_inflate;

    // ========== 【CRITICAL FIX】重新计算地图原点 ==========
    // map_origin_ 必须在 map_size_ 更新后重新计算，否则会使用基于 100m 的旧原点
    // 导致无人机实际位于地图外部
    // Z轴设为-1.0，给无人机下方留出1m余量，避免起飞时被判定为碰撞
    map_origin_ = Eigen::Vector3d(-map_size_ / 2.0, -map_size_ / 2.0, -1.0);
    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Map origin recalculated: (%.2f, %.2f, %.2f)",
             map_origin_.x(), map_origin_.y(), map_origin_.z());
    // ========== 地图原点修复结束 ==========

    ROS_WARN("[OBJ_PLANNER_WRAPPER] CLAMPED PARAMS: vel=%.2f (max %.2f), acc=%.2f (max %.2f)",
             max_vel_, SAFE_MAX_VEL, max_acc_, SAFE_MAX_ACC);
    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Map dimensions: %.2fm x %.2fm x %.2fm (XYZ)",
            map_size_, map_size_, map_height_);
    // ========== 安全钳制结束 ==========

    // Create planner interface instance
    planner_ = std::make_shared<obj_planner::PlannerInterface>();

    // Initialize planner parameters
    planner_->initParam(max_vel_, max_acc_, max_jerk_);

    // Initialize ESDF map with configurable height
    // Map center will be updated dynamically based on drone position
    planner_->initEsdfMap(map_size_, map_size_, map_height_,  // x, y, z size (use map_height parameter)
                         map_resolution_, map_origin_, map_inflate_);

    // Initialize visualization publishers
    static ros::NodeHandle nh;
    trajectory_vis_pub_ = nh.advertise<visualization_msgs::Marker>(
        "obj_planner_trajectory_visualization", 10, true);  // latch=true
    obs_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "obj_planner_obstacles_vis", 10, true);  // latch=true

    // 【新增】初始化历史轨迹发布者
    history_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/planner/history_vis", 10, true);  // queue=10, latch=true

    // 【新增】初始化历史轨迹 Marker 属性
    history_marker_.header.frame_id = "map";  // 与蓝绿线保持一致
    history_marker_.id = 0;
    history_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    history_marker_.action = visualization_msgs::Marker::ADD;
    history_marker_.scale.x = 0.1; // 线宽（10cm，更醒目）
    history_marker_.color.a = 1.0;  // 不透明
    history_marker_.color.r = 1.0; // 红色历史轨迹
    history_marker_.color.g = 0.0;
    history_marker_.color.b = 0.0;
    history_marker_.pose.orientation.w = 1.0;

    last_history_pos_ << -999, -999, -999; // 初始化为一个无效值

    initialized_ = true;
    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Initialized with vel=%.1f, acc=%.1f, jerk=%.1f",
             max_vel_, max_acc_, max_jerk_);

    // 【新增】如果配置了传感器参数，读取它们
    // 注意：这里暂时使用默认值，后续可以通过参数服务器配置
    sensor_range_ = 50.0;
    sensor_fov_ = 120.0;
    sensor_fov_rad_ = sensor_fov_ * M_PI / 180.0;
    sensor_height_ = 10.0;

    return true;
  }

  // =========================================================
  // 【新增】感知相关方法
  // =========================================================

  // 【核心】从参数服务器加载障碍物
  void ObjPlannerWrapper::loadObstacles(ros::NodeHandle& nh)
  {
    if (obstacles_loaded_) return;

    XmlRpc::XmlRpcValue obstacle_list;

    // 尝试读取参数服务器中的障碍物配置
    if (nh.getParam("/planner/obstacles", obstacle_list) || nh.getParam("/obstacles", obstacle_list))
    {
      ROS_INFO("[OBJ_PLANNER_WRAPPER]: Loading obstacles from parameter server...");

      for (int i = 0; i < obstacle_list.size(); ++i)
      {
        XmlRpc::XmlRpcValue obstacle = obstacle_list[i];

        if (obstacle.hasMember("type"))
        {
          std::string type = static_cast<std::string>(obstacle["type"]);

          if (type == "box")
          {
            BoxObstacle box;
            box.x = static_cast<double>(obstacle["x"]);
            box.y = static_cast<double>(obstacle["y"]);
            box.z = static_cast<double>(obstacle["z"]);
            box.width = static_cast<double>(obstacle["width"]);
            box.length = static_cast<double>(obstacle["length"]);
            box.height = static_cast<double>(obstacle["height"]);
            global_box_obstacles_.push_back(box);
          }
          else if (type == "sphere")
          {
            SphereObstacle sphere;
            sphere.x = static_cast<double>(obstacle["x"]);
            sphere.y = static_cast<double>(obstacle["y"]);
            sphere.z = static_cast<double>(obstacle["z"]);
            sphere.radius = static_cast<double>(obstacle["radius"]);
            global_sphere_obstacles_.push_back(sphere);
          }
        }
      }

      obstacles_loaded_ = true;
      ROS_INFO("[OBJ_PLANNER_WRAPPER]: Loaded %zu box + %zu sphere obstacles from parameter server",
               global_box_obstacles_.size(), global_sphere_obstacles_.size());
    }
    else
    {
      ROS_WARN("[OBJ_PLANNER_WRAPPER]: No obstacles found in parameter server!");
    }
  }

  // 【核心】模拟传感器更新 - FOV过滤 + 障碍物注入
  void ObjPlannerWrapper::updateSensor()
  {
    if (!obstacles_loaded_)
    {
      ROS_DEBUG_THROTTLE(5.0, "[OBJ_PLANNER_WRAPPER] Obstacles not loaded yet, skipping sensor update");
      return;
    }

    // === Step 1: FOV 过滤 ===
    std::vector<BoxObstacle> local_boxes;
    std::vector<SphereObstacle> local_spheres;

    for (const auto& box : global_box_obstacles_)
    {
      // 使用统一的FOV判断函数
      if (isObstacleInFOV(box.x, box.y, box.z))
      {
        local_boxes.push_back(box);
      }
    }

    // Sphere obstacles 同样处理
    for (const auto& sphere : global_sphere_obstacles_)
    {
      if (isObstacleInFOV(sphere.x, sphere.y, sphere.z))
      {
        local_spheres.push_back(sphere);
      }
    }

    // === Step 2: 将筛选后的障碍物注入到底层规划器 ===
    updateObstacles(local_boxes, local_spheres, current_pos_);

    // === Step 3: 发布可视化（让RViz显示"当前看到的障碍物"）===
    // 如果需要显示FOV效果，可以只发布 local_boxes
    publishObstacleVisualization();

    ROS_DEBUG_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER] Sensor update: %zu boxes + %zu spheres in FOV",
                      local_boxes.size(), local_spheres.size());
  }

  void ObjPlannerWrapper::setTarget(const geometry_msgs::PoseStamped& target)
  {
    convertPoseToEigen(target, target_pos_);
    target_set_ = true;

    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Target set to (%.2f, %.2f, %.2f)",
             target_pos_.x(), target_pos_.y(), target_pos_.z());
  }

  // 【新增】设置全局路径
  void ObjPlannerWrapper::setGlobalPath(const std::vector<geometry_msgs::PoseStamped>& path)
  {
    if (path.empty())
    {
      ROS_WARN("[OBJ_PLANNER_WRAPPER]: Received empty path, ignoring");
      return;
    }

    // 清空旧路径
    global_waypoints_.clear();

    // 转换ROS路径到Eigen格式
    for (const auto& pose : path)
    {
      Eigen::Vector3d wp;
      convertPoseToEigen(pose, wp);
      global_waypoints_.push_back(wp);
    }

    // 更新目标点为路径终点
    target_pos_ = global_waypoints_.back();
    target_set_ = true;

    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Global path set with %zu waypoints", global_waypoints_.size());
    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Path start: (%.2f, %.2f, %.2f)",
             global_waypoints_.front().x(), global_waypoints_.front().y(), global_waypoints_.front().z());
    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Path end: (%.2f, %.2f, %.2f)",
             global_waypoints_.back().x(), global_waypoints_.back().y(), global_waypoints_.back().z());
  }

  void ObjPlannerWrapper::updateDroneState(const geometry_msgs::PoseStamped& pose,
                                          const geometry_msgs::TwistStamped& vel)
  {
    // ========== 【优化】加速度估算 ==========
    // 保存上一次的速度和时间，用于估算加速度
    // 如果不估算加速度，每次重规划起点加速度都是0，会导致轨迹"折断"
    static Eigen::Vector3d last_vel = Eigen::Vector3d::Zero();
    static ros::Time last_time = ros::Time(0);
    ros::Time now = ros::Time::now();

    convertPoseToEigen(pose, current_pos_);
    convertTwistToEigen(vel, current_vel_);

    // ========== 【新增】计算偏航角用于 FOV 感知 ==========
    // 从四元数提取偏航角 (yaw)
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
                         pose.pose.orientation.y, pose.pose.orientation.z);
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    // ================================================

    // 简单的加速度估算（差分法 + 低通滤波）
    if (!last_time.isZero()) {
        double dt = (now - last_time).toSec();
        if (dt > 0.01 && dt < 0.2) { // 避免时间跳变或过长间隔
            // 低通滤波系数 (0.3 = 新数据权重)
            double alpha = 0.3;
            Eigen::Vector3d raw_acc = (current_vel_ - last_vel) / dt;

            // 简单的低通滤波，平滑加速度估算
            current_acc_ = alpha * raw_acc + (1.0 - alpha) * current_acc_;

            // 限制最大加速度噪音（防止异常值）
            double acc_limit = 3.0;  // 3.0 m/s²
            current_acc_.x() = std::max(-acc_limit, std::min(acc_limit, current_acc_.x()));
            current_acc_.y() = std::max(-acc_limit, std::min(acc_limit, current_acc_.y()));
            current_acc_.z() = std::max(-acc_limit, std::min(acc_limit, current_acc_.z()));
        }
    } else {
        // 第一次运行，初始化为0
        current_acc_ = Eigen::Vector3d::Zero();
    }

    last_vel = current_vel_;
    last_time = now;
    // ========== 加速度估算结束 ==========

    // ========== 【新增】历史轨迹更新逻辑（红色轨迹） ==========
    // 只有当移动距离超过 0.05m 才记录一个点，避免悬停时点过于密集消耗资源
    if ((current_pos_ - last_history_pos_).norm() > 0.05)
    {
        geometry_msgs::Point pt;
        pt.x = current_pos_(0);
        pt.y = current_pos_(1);
        pt.z = current_pos_(2);

        // 添加到红色轨迹 Marker
        history_marker_.points.push_back(pt);
        last_history_pos_ = current_pos_;

        // 实时发布红色轨迹
        history_marker_.header.stamp = ros::Time::now();
        history_vis_pub_.publish(history_marker_);
    }
    // ========== 历史轨迹记录结束 ==========
  }

  void ObjPlannerWrapper::updateObstacles(const std::vector<geometry_msgs::Vector3>& obstacle_positions,
                                        const Eigen::Vector3d& drone_pos)
  {
    if (!initialized_)
    {
      ROS_WARN_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Not initialized, cannot update obstacles");
      return;
    }

    // Clear old obstacles
    obstacles_.clear();

    // Convert obstacle positions to ObstacleInfo format
    // Assuming fixed size obstacles (you can customize this)
    const double default_obstacle_size = 0.5; // 0.5m cube

    for (const auto& pos : obstacle_positions)
    {
      obj_planner::ObstacleInfo obs;
      obs.x = pos.x;
      obs.y = pos.y;
      obs.z = pos.z;
      obs.size_x = default_obstacle_size;
      obs.size_y = default_obstacle_size;
      obs.size_z = default_obstacle_size;
      obs.type = 0; // Box

      obstacles_.push_back(obs);
    }

    // 【修复】先检查是否需要移动地图中心
    static ros::Time last_map_update = ros::Time(0);  // 【FIX】初始化为0，确保首次调用立即更新
    bool map_reset = false;

    // 仅在需要时移动地图（例如每1秒，或无人机接近边界）
    if ((ros::Time::now() - last_map_update).toSec() > 1.0)
    {
      // 计算新中心，Z轴逻辑：地图应该覆盖飞行高度
      // 【FIX】动态计算 Z 轴偏移量：40% 的地图高度，避免硬编码
      double z_offset = map_height_ * 0.4;  // 对于 5m 地图，偏移 2.0m
      Eigen::Vector3d map_center = drone_pos - Eigen::Vector3d(map_size_/2, map_size_/2, z_offset);

      planner_->initEsdfMap(map_size_, map_size_, map_height_,
                           map_resolution_, map_center, map_inflate_);
      last_map_update = ros::Time::now();
      map_reset = true;

      ROS_DEBUG("[OBJ_PLANNER_WRAPPER]: Map center updated to (%.2f, %.2f, %.2f)",
                map_center.x(), map_center.y(), map_center.z());
    }

    // 【关键】在地图初始化/重置之后设置障碍物
    // 如果刚重置了地图，必须重新设置障碍物
    // 如果没有重置，setObstacles 可能是增量更新（取决于planner实现）
    planner_->setObstacles(obstacles_, drone_pos);

    // ========== 【修复：手动调用可视化发布】 ==========
    // 增加 5Hz (0.2s) 的节流控制，保证实时性的同时避免刷屏
    static ros::Time last_vis_time = ros::Time(0);
    if ((ros::Time::now() - last_vis_time).toSec() > 0.2)
    {
        publishObstacleVisualization();
        last_vis_time = ros::Time::now();
    }
    // ===============================================

    ROS_DEBUG_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Updated %zu obstacles", obstacles_.size());
  }

  // 新增：直接从 planner 的障碍物数据更新（推荐方法）
  void ObjPlannerWrapper::updateObstacles(const std::vector<BoxObstacle>& box_obstacles,
                                        const std::vector<SphereObstacle>& sphere_obstacles,
                                        const Eigen::Vector3d& drone_pos)
  {
    if (!initialized_)
    {
      ROS_WARN_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Not initialized, cannot update obstacles");
      return;
    }

    // Clear old obstacles
    obstacles_.clear();

    // Convert BoxObstacle to ObstacleInfo
    for (const auto& box : box_obstacles)
    {
      obj_planner::ObstacleInfo obs;
      obs.x = box.x;
      obs.y = box.y;
      obs.z = box.z;
      obs.size_x = box.width;
      obs.size_y = box.length;
      obs.size_z = box.height;
      obs.type = 0; // Box

      obstacles_.push_back(obs);

      ROS_DEBUG("[OBJ_PLANNER_WRAPPER]: Added box obstacle at (%.2f, %.2f, %.2f) size (%.2f, %.2f, %.2f)",
                box.x, box.y, box.z, box.width, box.length, box.height);
    }

    // Convert SphereObstacle to ObstacleInfo (approximate as box)
    for (const auto& sphere : sphere_obstacles)
    {
      obj_planner::ObstacleInfo obs;
      obs.x = sphere.x;
      obs.y = sphere.y;
      obs.z = sphere.z;
      obs.size_x = sphere.radius * 2.0;  // Approximate sphere as cube
      obs.size_y = sphere.radius * 2.0;
      obs.size_z = sphere.radius * 2.0;
      obs.type = 1; // Sphere (but treated as box in current implementation)

      obstacles_.push_back(obs);

      ROS_DEBUG("[OBJ_PLANNER_WRAPPER]: Added sphere obstacle at (%.2f, %.2f, %.2f) radius %.2f",
                sphere.x, sphere.y, sphere.z, sphere.radius);
    }

    // 【修复】先检查是否需要移动地图中心
    static ros::Time last_map_update = ros::Time(0);  // 【FIX】初始化为0，确保首次调用立即更新
    bool map_reset = false;

    // 仅在需要时移动地图（例如每1秒，或无人机接近边界）
    if ((ros::Time::now() - last_map_update).toSec() > 1.0)
    {
      // Z轴向下偏移 2.0 米，防止无人机掉出地图
      // 对于 5m 高的地图，覆盖范围是无人机相对高度 [-2m, +3m]
      Eigen::Vector3d map_center = drone_pos - Eigen::Vector3d(map_size_/2, map_size_/2, 2.0);
      planner_->initEsdfMap(map_size_, map_size_, map_height_,
                           map_resolution_, map_center, map_inflate_);
      last_map_update = ros::Time::now();
      map_reset = true;

      ROS_DEBUG("[OBJ_PLANNER_WRAPPER]: Map center updated to (%.2f, %.2f, %.2f)",
                map_center.x(), map_center.y(), map_center.z());
    }

    // 【关键】在地图初始化/重置之后设置障碍物
    planner_->setObstacles(obstacles_, drone_pos);

    // ========== 【修复：手动调用可视化发布】 ==========
    // 增加 5Hz (0.2s) 的节流控制，保证实时性的同时避免刷屏
    static ros::Time last_vis_time = ros::Time(0);
    if ((ros::Time::now() - last_vis_time).toSec() > 0.2)
    {
        publishObstacleVisualization();
        last_vis_time = ros::Time::now();
    }
    // ===============================================

    ROS_INFO_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Updated %zu box + %zu sphere obstacles",
                      box_obstacles.size(), sphere_obstacles.size());
  }

  // 【新增】检查并更新地图位置（滚动地图机制）
  void ObjPlannerWrapper::checkAndUpdateMap()
  {
    if (!initialized_) return;

    // 计算当前地图的中心点（注意 map_origin_ 是左下角，不是中心）
    Eigen::Vector3d current_map_center = map_origin_ + Eigen::Vector3d(map_size_/2, map_size_/2, map_height_/2);

    // 计算无人机到地图中心的水平距离
    Eigen::Vector2d dist_xy(current_pos_.x() - current_map_center.x(),
                           current_pos_.y() - current_map_center.y());

    // 如果无人机离中心超过 30% 地图大小，就移动地图
    // 例如 40m 地图，偏离 12m 就更新，确保始终在中心区域
    if (dist_xy.norm() > map_size_ * 0.30)
    {
        // 计算新的原点（以无人机为中心）
        double z_offset = map_height_ * 0.4; // 垂直方向偏移，给无人机下方留出空间

        // 新的原点 (Min X, Min Y, Min Z)
        Eigen::Vector3d new_origin = current_pos_ - Eigen::Vector3d(map_size_/2, map_size_/2, z_offset);

        // 调用底层接口重置地图
        planner_->initEsdfMap(map_size_, map_size_, map_height_,
                            map_resolution_, new_origin, map_inflate_);

        // 更新记录的原点
        map_origin_ = new_origin;

        // 【重要】重置地图后，必须重新注入当前已知的障碍物！
        // 否则地图是空的，之前的障碍物会丢失
        planner_->setObstacles(obstacles_, current_pos_);

        ROS_INFO_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Map Rolling... New origin: (%.1f, %.1f, %.1f)",
                 map_origin_.x(), map_origin_.y(), map_origin_.z());
    }
  }

  bool ObjPlannerWrapper::planTrajectory()
  {
    if (!initialized_)
    {
      ROS_ERROR("[OBJ_PLANNER_WRAPPER]: Cannot plan - not initialized");
      return false;
    }

    if (!target_set_)
    {
      ROS_ERROR("[OBJ_PLANNER_WRAPPER]: Cannot plan - no target set");
      return false;
    }

    // 【核心修复】规划前检查地图是否需要跟随
    checkAndUpdateMap();

    // 🔍 调试：打印当前速度向量（验证坐标系）
    // 如果向 北(Y+) 飞，Vel 应该接近 (0, speed, 0)
    // 如果打印出来是 (speed, 0, 0)，说明坐标系还是错的（Body frame）！
    double vel_angle = std::atan2(current_vel_.y(), current_vel_.x()) * 57.3; // 转为角度
    ROS_INFO_THROTTLE(1.0, "[Check] Pos(%.1f, %.1f) Vel(%.2f, %.2f) | Angle=%.1f°",
        current_pos_.x(), current_pos_.y(),
        current_vel_.x(), current_vel_.y(),
        vel_angle
    );

    // ==========================================================================================
    // 1. 【核心修复】确定规划起点 (Start State) - 实现轨迹拼接
    // ==========================================================================================
    Eigen::Vector3d start_pos, start_vel, start_acc;

    // 计算从上一条轨迹开始到现在的时间流逝
    ros::Time now = ros::Time::now();
    double time_elapsed = 0.0;
    if (!trajectory_start_time_.isZero()) {
        time_elapsed = (now - trajectory_start_time_).toSec();
    }

    // 【致命修复】Lookahead 必须设为 0.0！
    // 之前设为 0.05/0.1 导致了"时空穿越"，每一帧目标点都瞬移，引发震荡。
    // 设为 0.0 保证新轨迹无缝衔接在旧轨迹的"当前时刻"位置。
    double lookahead = 0.0;

    // 判断是否可以使用拼接 (Stitching)
    bool use_stitching = false;
    controller_msgs::FlatTarget stitch_target;

    if (has_valid_trajectory_ && !trajectory_points_.empty())
    {
        // 尝试从上一条轨迹采样 (time_elapsed) 时刻的状态
        double traj_duration = getTrajectoryDuration();

        // 允许超时 1.0s 内拼接 (2Hz 频率下，超时容忍度要大一点)
        if (time_elapsed < traj_duration + 1.0)
        {
            if (getSetpointAtTime(time_elapsed + lookahead, stitch_target))
            {
                 Eigen::Vector3d stitch_pos(stitch_target.position.x, stitch_target.position.y, stitch_target.position.z);

                 // 【临时放宽】在震荡调试期，将偏差阈值从 2.0m 放宽到 5.0m
                 // 强迫规划器尽量拼接，把飞机"拉"回平滑轨迹，而不是跟着飞机抖
                 if ((stitch_pos - current_pos_).norm() < 5.0)
                 {
                     use_stitching = true;
                     start_pos = stitch_pos;
                     start_vel = Eigen::Vector3d(stitch_target.velocity.x, stitch_target.velocity.y, stitch_target.velocity.z);
                     start_acc = Eigen::Vector3d(stitch_target.acceleration.x, stitch_target.acceleration.y, stitch_target.acceleration.z);
                 }
                 else
                 {
                     ROS_WARN_THROTTLE(1.0, "[OBJ_PLANNER]: Stitching rejected (Deviation > 5.0m)");
                 }
            }
        }
    }

    if (!use_stitching)
    {
        // 无法拼接（刚起飞、偏差大），使用传感器数据
        start_pos = current_pos_; // 不再加 lookahead 预测
        start_vel = current_vel_;

        // 【修改】保留当前加速度，而不是清零
        // 避免起飞时的顿挫和轨迹折断
        start_acc = current_acc_;
        // start_acc = Eigen::Vector3d::Zero(); // 删除这行

        if (start_vel.norm() < 0.1) start_vel = Eigen::Vector3d::Zero();
    }

    // ==========================================================================================
    // 2. 智能计算局部目标 (Local Target) - 移植自 RealTimePlanner
    // ==========================================================================================

    // 【优化】规划距离提升至 30m，或者根据速度动态调整
    // 地图大小允许的话，30m 能提供更丝滑的绕行体验
    double base_horizon = 30.0;
    // 动态调整：速度越快看得越远 (30m ~ 45m)
    double dynamic_horizon = std::max(base_horizon, start_vel.norm() * 6.0);
    // 【安全边界优化】使用 0.40 代替 0.45，为 40m 地图提供更多边界余量
    // 40m * 0.40 = 16m (vs 原来的 18m)，减少"terminal point in obstacle"风险
    const double planning_horizon = std::min(dynamic_horizon, map_size_ * 0.40);

    // 【声明】局部目标点及其速度
    Eigen::Vector3d local_target;
    Eigen::Vector3d local_vel;
    bool target_found = false;

    // 【新增】优先尝试沿着全局路径搜索 (如果已设置)
    if (!global_waypoints_.empty())
    {
      // getLocalTargetFromPath 内部已经处理了路径跟随逻辑
      if (getLocalTargetFromPath(planning_horizon, local_target, local_vel))
      {
        // 检查这个从路径上选出来的点是否被遮挡
        if (calculateMinObstacleDistance(local_target) > 0.5)
        {
          // 还要检查连线是否安全
          bool line_safe = true;
          Eigen::Vector3d check_vec = local_target - start_pos;
          double check_len = check_vec.norm();
          for(double d=1.0; d<check_len; d+=1.5) {
            if (calculateMinObstacleDistance(start_pos + check_vec.normalized()*d) < 0.5) {
              line_safe = false; break;
            }
          }
          if (line_safe) target_found = true;
        }
      }
    }

    // 如果没有全局路径或者路径被挡住，回退到基于方向的搜索
    if (!target_found)
    {
      // 步骤 A: 初始目标选择 (沿全局路径)
      Eigen::Vector3d raw_target;
      Eigen::Vector3d direction_to_target;

      double dist_to_global = (target_pos_ - start_pos).norm();
      if (dist_to_global > planning_horizon)
      {
        direction_to_target = (target_pos_ - start_pos).normalized();
        raw_target = start_pos + direction_to_target * planning_horizon;
        local_vel = direction_to_target * max_vel_;
      }
      else
      {
        raw_target = target_pos_;
        local_vel = Eigen::Vector3d::Zero();
      }

      // 步骤 B: 障碍物检测与修正
      local_target = raw_target; // 默认使用原始目标
      double safe_margin = 0.8; // 稍微加大安全边距

      // 检查直连路径是否有阻挡
      bool blocked = false;
      double dist_to_raw = (raw_target - start_pos).norm();
      Eigen::Vector3d dir_check = (raw_target - start_pos).normalized();

      // 建议检查步长设为 1.5m 左右
      for (double d = 1.0; d < dist_to_raw; d += 1.5)
      {
        if (calculateMinObstacleDistance(start_pos + dir_check * d) < safe_margin)
        {
          blocked = true;
          break;
        }
      }
      // 还要检查终点本身是否安全
      if (calculateMinObstacleDistance(raw_target) < safe_margin)
        blocked = true;

      if (!blocked)
      {
        // 路径安全，使用原始目标
        // local_target 已经设置为 raw_target
      }
      else
      {
        // 步骤 C: 触发避障
        ROS_WARN_THROTTLE(1.0, "[OBJ_PLANNER] Obstacle detected, avoiding...");

        Eigen::Vector3d best_dir = calculateBestAvoidanceDirection();
        bool found_safe = false;

        // 【优化】搜索策略：
        // 1. 优先找远处的点 (保持速度)
        // 2. 如果远处不行，不要立刻放弃，尝试稍微近一点的点 (比如 15m)，
        //    这样可以避免因为远处有墙就原地停住，而是先飞到中间空旷处
        for (double d = planning_horizon; d > 5.0; d -= 2.0)
        {
          Eigen::Vector3d candidate = start_pos + best_dir * d;

          if (isInLocalMap(candidate) && calculateMinObstacleDistance(candidate) > safe_margin)
          {
            local_target = candidate;
            local_vel = best_dir * max_vel_;
            found_safe = true;
            ROS_INFO("[OBJ_PLANNER] Found safe target at %.2fm offset", d);
            break;
          }
        }

        if (!found_safe)
        {
          // 实在没办法，只能悬停
          ROS_ERROR("[OBJ_PLANNER] Trap detected! Hovering.");
          local_target = start_pos; // 保持原地
          local_vel = Eigen::Vector3d::Zero();
        }
      }
    }

    // ==========================================================================================
    // 3. 【核心修复】构建初始路径 (Path Points) - 速度平滑插值
    // ==========================================================================================
    std::vector<obj_planner::PathPoint> path_points;

    // 3.1 添加起点 (使用计算好的拼接起点)
    obj_planner::PathPoint start_pt_msg;
    start_pt_msg.x = start_pos.x(); start_pt_msg.y = start_pos.y(); start_pt_msg.z = start_pos.z();
    start_pt_msg.vx = start_vel.x(); start_pt_msg.vy = start_vel.y(); start_pt_msg.vz = start_vel.z();
    start_pt_msg.ax = start_acc.x(); start_pt_msg.ay = start_acc.y(); start_pt_msg.az = start_acc.z();
    start_pt_msg.yaw = 0.0; start_pt_msg.pitch = 0.0; start_pt_msg.time_from_start = 0.0;
    path_points.push_back(start_pt_msg);

    // 3.2 【修改点】根据距离动态计算中间点数量，确保密度适配高速飞行
    double total_dist = (local_target - start_pos).norm();
    // 密度设为 0.6m (适配 3m/s 速度: 3m/s * 0.2s = 0.6m)，且至少有 2 个点
    int num_waypoints = std::max(2, (int)(total_dist / 0.6));

    // 打印调试信息，确认修复生效
    ROS_INFO("[OBJ_PLANNER_WRAPPER]: Generating path with %d waypoints for dist=%.2fm (density=0.6m/pt)",
             num_waypoints, total_dist);

    for (int i = 1; i <= num_waypoints; i++)
    {
      double ratio = (double)i / (num_waypoints + 1);
      obj_planner::PathPoint wp;

      // 位置线性插值
      wp.x = start_pos.x() + ratio * (local_target.x() - start_pos.x());
      wp.y = start_pos.y() + ratio * (local_target.y() - start_pos.y());
      wp.z = start_pos.z() + ratio * (local_target.z() - start_pos.z());

      // ========== 【核心修复】速度线性插值 ==========
      // 之前速度设为0导致轨迹在中间减速，产生"走走停停"的波浪
      // 现在使用线性插值保持运动连续性，让优化器生成平滑轨迹
      wp.vx = start_vel.x() + ratio * (local_vel.x() - start_vel.x());
      wp.vy = start_vel.y() + ratio * (local_vel.y() - start_vel.y());
      wp.vz = start_vel.z() + ratio * (local_vel.z() - start_vel.z());
      // ========== 【核心修复结束】 ==========

      wp.ax = 0.0; wp.ay = 0.0; wp.az = 0.0;
      wp.yaw = 0.0; wp.pitch = 0.0; wp.time_from_start = 0.0;
      path_points.push_back(wp);
    }

    // 3.3 添加终点 (Local Target)
    obj_planner::PathPoint goal_pt_msg;
    goal_pt_msg.x = local_target.x(); goal_pt_msg.y = local_target.y(); goal_pt_msg.z = local_target.z();
    goal_pt_msg.vx = local_vel.x(); goal_pt_msg.vy = local_vel.y(); goal_pt_msg.vz = local_vel.z();
    goal_pt_msg.ax = 0.0; goal_pt_msg.ay = 0.0; goal_pt_msg.az = 0.0;
    goal_pt_msg.yaw = 0.0; goal_pt_msg.pitch = 0.0; goal_pt_msg.time_from_start = 0.0;
    path_points.push_back(goal_pt_msg);

    // 将初始路径传给规划器 (用于生成 B-spline 的控制点初值)
    planner_->setPathPoint(path_points);

    // ==========================================================================================
    // 4. 执行规划
    // ==========================================================================================
    // 传递 start_vel 和 start_acc 确保边界约束
    bool success = planner_->makePlanWithState(
        start_pos,
        start_vel,
        start_acc,
        local_target,       // 局部目标
        local_vel,          // 局部速度
        Eigen::Vector3d::Zero()  // 终点加速度 (通常设为0)
    );

    if (success)
    {
      planner_->getLocalPlanTrajResults(trajectory_points_);
      has_valid_trajectory_ = !trajectory_points_.empty();

      // 【关键】重置轨迹开始时间为"现在"
      // 注意：虽然我们是往前 lookahead 了一点时间进行规划的，但对 controller 来说，
      // 这条轨迹就是从"现在"（或者说现在+传输延迟）开始执行的。
      trajectory_start_time_ = ros::Time::now();

      if (has_valid_trajectory_)
      {
        ROS_INFO_THROTTLE(2.0, "[OBJ_PLANNER]: Planning success. Pts=%zu. Stitching=%d",
                 trajectory_points_.size(), use_stitching);
      }
    }
    else
    {
      ROS_ERROR_THROTTLE(1.0, "[OBJ_PLANNER]: Planning failed");
      // 规划失败时，不要清除旧轨迹，让它继续飞完剩余部分，增加鲁棒性
      // has_valid_trajectory_ = false;
    }

    return success;
  }

  std::vector<controller_msgs::FlatTarget> ObjPlannerWrapper::getControllerTrajectory() const
  {
    std::vector<controller_msgs::FlatTarget> flat_targets;

    if (!has_valid_trajectory_ || trajectory_points_.empty())
    {
      ROS_WARN_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: No valid trajectory to convert");
      return flat_targets;
    }

    // Convert obj_planner PathPoint to controller_msgs::FlatTarget
    for (const auto& pt : trajectory_points_)
    {
      controller_msgs::FlatTarget flat_target;

      flat_target.type_mask = 0;  // Use position and velocity
      flat_target.position.x = pt.x;
      flat_target.position.y = pt.y;
      flat_target.position.z = pt.z;

      flat_target.velocity.x = pt.vx;
      flat_target.velocity.y = pt.vy;
      flat_target.velocity.z = pt.vz;

      flat_target.acceleration.x = pt.ax;
      flat_target.acceleration.y = pt.ay;
      flat_target.acceleration.z = pt.az;

      // Note: FlatTarget doesn't include yaw - yaw is handled separately
      // in the state machine through position/velocity control

      flat_targets.push_back(flat_target);
    }

    ROS_DEBUG("[OBJ_PLANNER_WRAPPER]: Converted %zu trajectory points to FlatTarget format",
              flat_targets.size());

    return flat_targets;
  }

  bool ObjPlannerWrapper::isGoalReached(double threshold) const
  {
    if (!target_set_)
      return false;

    double distance = (current_pos_ - target_pos_).norm();
    return distance < threshold;
  }

  void ObjPlannerWrapper::clearTrajectory()
  {
    trajectory_points_.clear();
    has_valid_trajectory_ = false;
    target_set_ = false;

    // 注意：红色历史轨迹 (history_marker_) 不会清除，记录完整飞行路径
  }

  // Private helper functions
  void ObjPlannerWrapper::convertPoseToEigen(const geometry_msgs::PoseStamped& pose, Eigen::Vector3d& pos)
  {
    pos(0) = pose.pose.position.x;
    pos(1) = pose.pose.position.y;
    pos(2) = pose.pose.position.z;
  }

  void ObjPlannerWrapper::convertTwistToEigen(const geometry_msgs::TwistStamped& twist, Eigen::Vector3d& vel)
  {
    vel(0) = twist.twist.linear.x;
    vel(1) = twist.twist.linear.y;
    vel(2) = twist.twist.linear.z;
  }

  obj_planner::PathPoint ObjPlannerWrapper::createPathPoint(const Eigen::Vector3d& pos,
                                                          const Eigen::Vector3d& vel,
                                                          const Eigen::Vector3d& acc,
                                                          double time_from_start)
  {
    obj_planner::PathPoint pt;
    pt.x = pos.x();
    pt.y = pos.y();
    pt.z = pos.z();
    pt.vx = vel.x();
    pt.vy = vel.y();
    pt.vz = vel.z();
    pt.ax = acc.x();
    pt.ay = acc.y();
    pt.az = acc.z();
    pt.time_from_start = time_from_start;
    pt.yaw = 0.0;
    pt.pitch = 0.0;
    return pt;
  }

  // ========== 【辅助函数】检查障碍物是否在传感器FOV内 ==========
  bool ObjPlannerWrapper::isObstacleInFOV(double obs_x, double obs_y, double obs_z) const
  {
    // 1. 距离检查
    double dx = obs_x - current_pos_.x();
    double dy = obs_y - current_pos_.y();
    double dz = obs_z - current_pos_.z();
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist >= sensor_range_) return false;

    // 2. 高度检查（垂直FOV）
    if (std::abs(dz) > sensor_height_ / 2.0) return false;

    // 3. 水平FOV角度检查
    double angle = std::atan2(dy, dx);
    double angle_diff = angle - current_yaw_;

    // 角度归一化到 [-π, π]
    while (angle_diff > M_PI) angle_diff -= 2*M_PI;
    while (angle_diff < -M_PI) angle_diff += 2*M_PI;

    return (std::abs(angle_diff) <= sensor_fov_rad_ / 2.0);
  }

  // ========== 【新增】发布内部障碍物可视化 ==========
  void ObjPlannerWrapper::publishObstacleVisualization()
  {
    visualization_msgs::MarkerArray marker_array;

    // 预估数量
    size_t total_obstacles = global_box_obstacles_.size() + global_sphere_obstacles_.size();
    marker_array.markers.reserve(total_obstacles);

    int marker_id = 0;

    // === 处理 Box 障碍物 ===
    for (const auto& box : global_box_obstacles_)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "obj_planner_obstacles";
      marker.id = marker_id++;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(0);  // 永不过期

      marker.type = visualization_msgs::Marker::CUBE;
      marker.pose.position.x = box.x;
      marker.pose.position.y = box.y;
      marker.pose.position.z = box.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = box.width;
      marker.scale.y = box.length;
      marker.scale.z = box.height;

      // 【关键】根据是否在FOV内设置颜色
      bool in_fov = isObstacleInFOV(box.x, box.y, box.z);

      if (in_fov)
      {
        // 在感知范围内 → 橙色（需要避障）
        marker.color.r = 1.0;  // R
        marker.color.g = 0.5;  // G
        marker.color.b = 0.0;  // B
        marker.color.a = 0.8;  // 更不透明
      }
      else
      {
        // 不在感知范围内 → 绿色（环境障碍物，不需要避障）
        marker.color.r = 0.0;  // R
        marker.color.g = 1.0;  // G
        marker.color.b = 0.0;  // B
        marker.color.a = 0.3;  // 更半透明
      }

      marker_array.markers.push_back(marker);
    }

    // === 处理 Sphere 障碍物 ===
    for (const auto& sphere : global_sphere_obstacles_)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "obj_planner_obstacles";
      marker.id = marker_id++;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(0);

      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position.x = sphere.x;
      marker.pose.position.y = sphere.y;
      marker.pose.position.z = sphere.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = sphere.radius * 2.0;
      marker.scale.y = sphere.radius * 2.0;
      marker.scale.z = sphere.radius * 2.0;

      // 【关键】根据是否在FOV内设置颜色
      bool in_fov = isObstacleInFOV(sphere.x, sphere.y, sphere.z);

      if (in_fov)
      {
        // 在感知范围内 → 橙色
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
      }
      else
      {
        // 不在感知范围内 → 绿色
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
      }

      marker_array.markers.push_back(marker);
    }

    // 发布MarkerArray
    obs_vis_pub_.publish(marker_array);

    // 统计信息
    int fov_count = 0;
    for (const auto& marker : marker_array.markers)
    {
      if (marker.color.r > 0.5) fov_count++;  // 橙色障碍物（R > 0.5）
    }

    ROS_DEBUG_THROTTLE(5.0, "[OBJ_PLANNER_WRAPPER]: Published %zu obstacles (%d in FOV/orange, %zu out of FOV/green)",
                      marker_array.markers.size(), fov_count, marker_array.markers.size() - fov_count);
  }

  void ObjPlannerWrapper::publishTrajectoryVisualization()
  {
    if (trajectory_points_.empty())
    {
      ROS_WARN_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: No trajectory to visualize");
      return;
    }

    // ========== 【关键修复】计算轨迹执行时间，修剪过去部分 ==========
    double time_elapsed = 0.0;
    if (!trajectory_start_time_.isZero())
    {
      time_elapsed = (ros::Time::now() - trajectory_start_time_).toSec();
    }
    else
    {
      // 首次调用，初始化开始时间
      trajectory_start_time_ = ros::Time::now();
      time_elapsed = 0.0;
    }

    // ========== 2. 显示剩余轨迹（绿色，将要飞行的路径）【修复Gap】==========
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "obj_planner_trajectory";
    traj_marker.id = 0;  // 固定 ID，会覆盖旧轨迹
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.lifetime = ros::Duration(0);  // 永不过期

    traj_marker.scale.x = 0.03;  // 【修复】更细的线宽，更加美观

    // 绿色，表示规划轨迹
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 1.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;  // 不透明

    // 【关键修复】第一步：添加当前无人机位置作为起点（无缝连接）
    geometry_msgs::Point current_point;
    current_point.x = current_pos_.x();
    current_point.y = current_pos_.y();
    current_point.z = current_pos_.z();
    traj_marker.points.push_back(current_point);

    // 【关键修复】第二步：只添加未来轨迹点（修剪过去部分）
    for (const auto& pt : trajectory_points_)
    {
      // 只添加时间戳大于已执行时间的点
      if (pt.time_from_start > time_elapsed)
      {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        traj_marker.points.push_back(p);
      }
    }

    // 发布轨迹
    trajectory_vis_pub_.publish(traj_marker);

    // ========== 3. 显示目标点（红色球体）==========
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "obj_planner_goal";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.lifetime = ros::Duration(0);

    // 目标点位置
    goal_marker.pose.position.x = target_pos_.x();
    goal_marker.pose.position.y = target_pos_.y();
    goal_marker.pose.position.z = target_pos_.z();
    goal_marker.pose.orientation.w = 1.0;

    // 球体大小
    goal_marker.scale.x = 0.4;
    goal_marker.scale.y = 0.4;
    goal_marker.scale.z = 0.4;

    // 红色，表示目标
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 0.9;

    trajectory_vis_pub_.publish(goal_marker);

    ROS_DEBUG_THROTTLE(5.0, "[OBJ_PLANNER_WRAPPER]: Trajectory vis - time_elapsed=%.2fs, points=%zu (from current pos)",
                      time_elapsed, traj_marker.points.size());
  }

  // 获取轨迹总时长
  double ObjPlannerWrapper::getTrajectoryDuration() const
  {
    if (trajectory_points_.empty() || trajectory_points_.size() < 2)
      return 0.0;

    // 使用最后一个点的时间
    return trajectory_points_.back().time_from_start;
  }

  // 核心：根据时间插值获取目标点
  bool ObjPlannerWrapper::getSetpointAtTime(double time_from_start, controller_msgs::FlatTarget& target)
  {
    // 【FIX】使用静态变量保持上一次有效的偏航角
    static double last_valid_yaw = 0.0;

    if (trajectory_points_.empty())
      return false;

    // 边界检查：如果时间超出轨迹末端，使用最后一个点
    if (time_from_start >= trajectory_points_.back().time_from_start)
    {
      const auto& pt = trajectory_points_.back();

      target.header.stamp = ros::Time::now();
      target.header.frame_id = "map";
      target.type_mask = 2; // IGNORE_SNAP_JERK: Position + Velocity + Acceleration

      target.position.x = pt.x;
      target.position.y = pt.y;
      target.position.z = pt.z;

      target.velocity.x = pt.vx;
      target.velocity.y = pt.vy;
      target.velocity.z = pt.vz;

      // 【强力限幅】限制加速度在安全范围内
      double acc_limit = 2.0; // 2.0 m/s^2
      target.acceleration.x = std::max(-acc_limit, std::min(acc_limit, pt.ax));
      target.acceleration.y = std::max(-acc_limit, std::min(acc_limit, pt.ay));
      target.acceleration.z = std::max(-acc_limit, std::min(acc_limit, pt.az));

      // jerk 和 snap 设为零
      target.jerk.x = 0.0; target.jerk.y = 0.0; target.jerk.z = 0.0;
      target.snap.x = 0.0; target.snap.y = 0.0; target.snap.z = 0.0;

      return true;
    }

    // 边界检查：如果时间小于起点，使用第一个点
    if (time_from_start <= trajectory_points_.front().time_from_start)
    {
      const auto& pt = trajectory_points_.front();

      target.header.stamp = ros::Time::now();
      target.header.frame_id = "map";
      target.type_mask = 2;

      target.position.x = pt.x;
      target.position.y = pt.y;
      target.position.z = pt.z;

      target.velocity.x = pt.vx;
      target.velocity.y = pt.vy;
      target.velocity.z = pt.vz;

      // 【强力限幅】限制加速度在安全范围内
      double acc_limit = 2.0; // 2.0 m/s^2
      target.acceleration.x = std::max(-acc_limit, std::min(acc_limit, pt.ax));
      target.acceleration.y = std::max(-acc_limit, std::min(acc_limit, pt.ay));
      target.acceleration.z = std::max(-acc_limit, std::min(acc_limit, pt.az));

      target.jerk.x = 0.0; target.jerk.y = 0.0; target.jerk.z = 0.0;
      target.snap.x = 0.0; target.snap.y = 0.0; target.snap.z = 0.0;

      return true;
    }

    // 查找时间对应的两个相邻点进行插值
    int idx = 0;
    for (int i = 0; i < trajectory_points_.size() - 1; ++i)
    {
      if (time_from_start >= trajectory_points_[i].time_from_start &&
          time_from_start < trajectory_points_[i + 1].time_from_start)
      {
        idx = i;
        break;
      }
    }

    const auto& p0 = trajectory_points_[idx];
    const auto& p1 = trajectory_points_[idx + 1];

    // 计算插值系数 alpha
    double dt = p1.time_from_start - p0.time_from_start;
    double alpha = 0.0;
    if (dt > 1e-6)
    {
      alpha = (time_from_start - p0.time_from_start) / dt;
    }
    alpha = std::max(0.0, std::min(1.0, alpha)); // 限制在 [0, 1] 范围内

    // 线性插值位置、速度、加速度
    double x = (1 - alpha) * p0.x + alpha * p1.x;
    double y = (1 - alpha) * p0.y + alpha * p1.y;
    double z = (1 - alpha) * p0.z + alpha * p1.z;

    double vx = (1 - alpha) * p0.vx + alpha * p1.vx;
    double vy = (1 - alpha) * p0.vy + alpha * p1.vy;
    double vz = (1 - alpha) * p0.vz + alpha * p1.vz;

    double ax = (1 - alpha) * p0.ax + alpha * p1.ax;
    double ay = (1 - alpha) * p0.ay + alpha * p1.ay;
    double az = (1 - alpha) * p0.az + alpha * p1.az;

    // 【FIX】Yaw 插值（改进：低速时保持上一次的朝向）
    double yaw = last_valid_yaw;  // 默认使用上一次的朝向
    if (std::abs(vx) > 0.1 || std::abs(vy) > 0.1)
    {
      // 速度足够时，根据速度方向计算新的朝向
      yaw = std::atan2(vy, vx);
      last_valid_yaw = yaw;  // 更新静态变量
    }
    // 否则保持 last_valid_yaw 不变（无人机停止时不会重置为0）

    // 填充消息
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map";
    target.type_mask = 2; // IGNORE_SNAP_JERK: Position + Velocity + Acceleration

    target.position.x = x;
    target.position.y = y;
    target.position.z = z;

    target.velocity.x = vx;
    target.velocity.y = vy;
    target.velocity.z = vz;

    // 【强力限幅】限制加速度在安全范围内
    double acc_limit = 2.0; // 2.0 m/s^2
    target.acceleration.x = std::max(-acc_limit, std::min(acc_limit, ax));
    target.acceleration.y = std::max(-acc_limit, std::min(acc_limit, ay));
    target.acceleration.z = std::max(-acc_limit, std::min(acc_limit, az));

    target.jerk.x = 0.0; target.jerk.y = 0.0; target.jerk.z = 0.0;
    target.snap.x = 0.0; target.snap.y = 0.0; target.snap.z = 0.0;

    return true;
  }

  // ========== 【新增】路径跟随辅助函数 ==========

  // 计算点到线段的距离，并返回投影点和参数t
  double ObjPlannerWrapper::distToSegment(const Eigen::Vector3d& point,
                                          const Eigen::Vector3d& seg_start,
                                          const Eigen::Vector3d& seg_end,
                                          Eigen::Vector3d& projection,
                                          double& t) const
  {
    Eigen::Vector3d seg_vec = seg_end - seg_start;
    double seg_len = seg_vec.norm();

    if (seg_len < 1e-6)
    {
      // 线段长度为0，直接返回起点
      projection = seg_start;
      t = 0.0;
      return (point - seg_start).norm();
    }

    // 计算投影参数 t = ((P - A) · (B - A)) / |B - A|^2
    t = (point - seg_start).dot(seg_vec) / (seg_len * seg_len);

    // 限制t在[0,1]范围内（线段端点之间）
    t = std::max(0.0, std::min(1.0, t));

    // 计算投影点
    projection = seg_start + t * seg_vec;

    // 返回距离
    return (point - projection).norm();
  }

  // 查找距离给定点最近的路径线段
  int ObjPlannerWrapper::findClosestSegment(const Eigen::Vector3d& point,
                                            const std::vector<Eigen::Vector3d>& path,
                                            double& min_dist) const
  {
    if (path.size() < 2)
    {
      min_dist = (point - path.front()).norm();
      return 0;
    }

    int closest_seg = 0;
    min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
      Eigen::Vector3d projection;
      double t;
      double dist = distToSegment(point, path[i], path[i+1], projection, t);

      if (dist < min_dist)
      {
        min_dist = dist;
        closest_seg = i;
      }
    }

    return closest_seg;
  }

  // 从全局路径计算局部目标点（核心逻辑）
  bool ObjPlannerWrapper::getLocalTargetFromPath(double horizon,
                                                  Eigen::Vector3d& local_target,
                                                  Eigen::Vector3d& local_vel)
  {
    // 回退条件1：没有全局路径或路径点太少
    if (global_waypoints_.empty() || global_waypoints_.size() < 2)
    {
      ROS_DEBUG_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: No global path available, using direct target");
      return false; // 触发回退到直线模式
    }

    // 回退条件2：无人机距离路径太远（可能偏离了路径）
    double dist_to_path;
    int closest_seg = findClosestSegment(current_pos_, global_waypoints_, dist_to_path);

    const double MAX_PATH_DEVIATION = 5.0; // 最大允许偏离距离
    if (dist_to_path > MAX_PATH_DEVIATION)
    {
      ROS_WARN_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Drone far from path (%.2fm), using direct target", dist_to_path);
      return false;
    }

    // Step 1: 找到无人机在最近线段上的投影点
    Eigen::Vector3d projection;
    double t;
    distToSegment(current_pos_, global_waypoints_[closest_seg], global_waypoints_[closest_seg + 1],
                  projection, t);

    // Step 2: 从投影点开始，沿路径向前查找horizon距离外的点
    double remaining_dist = horizon;
    Eigen::Vector3d current_point = projection;
    local_target = projection;

    // 从最近线段的投影点开始遍历
    for (size_t i = closest_seg; i < global_waypoints_.size() - 1; ++i)
    {
      const Eigen::Vector3d& wp_start = (i == closest_seg) ? current_point : global_waypoints_[i];
      const Eigen::Vector3d& wp_end = global_waypoints_[i + 1];

      Eigen::Vector3d seg_vec = wp_end - wp_start;
      double seg_len = seg_vec.norm();

      if (seg_len < 1e-6)
        continue;

      if (remaining_dist <= seg_len)
      {
        // 目标点在当前线段内
        local_target = wp_start + (remaining_dist / seg_len) * seg_vec;
        remaining_dist = 0.0;
        break;
      }
      else
      {
        // 跳过整个线段，继续下一段
        remaining_dist -= seg_len;
        local_target = wp_end;
      }
    }

    // 如果remaining_dist > 0，说明已经到达路径末端
    if (remaining_dist > 0.1)
    {
      local_target = global_waypoints_.back();
      ROS_DEBUG_THROTTLE(2.0, "[OBJ_PLANNER_WRAPPER]: Reached end of global path");
    }

    // Step 3: 计算局部速度方向（沿路径切线方向）
    // 找到local_target所在的线段
    int target_seg = findClosestSegment(local_target, global_waypoints_, dist_to_path);

    if (target_seg < static_cast<int>(global_waypoints_.size()) - 1)
    {
      // 计算该线段的方向向量
      Eigen::Vector3d path_direction = (global_waypoints_[target_seg + 1] - global_waypoints_[target_seg]).normalized();
      local_vel = path_direction * max_vel_;
    }
    else
    {
      // 已到达终点，速度为0
      local_vel = Eigen::Vector3d::Zero();
    }

    // 调试输出
    static ros::Time last_debug_time = ros::Time(0);
    if ((ros::Time::now() - last_debug_time).toSec() > 1.0)
    {
      ROS_DEBUG("[OBJ_PLANNER_WRAPPER]: Path following - closest_seg=%d, horizon=%.2fm, local_target=(%.2f, %.2f, %.2f)",
                closest_seg, horizon, local_target.x(), local_target.y(), local_target.z());
      last_debug_time = ros::Time::now();
    }

    return true;
  }

  // =========================================================
  // 移植自 RealTimePlanner 的智能避障逻辑
  // =========================================================

  bool ObjPlannerWrapper::isInLocalMap(const Eigen::Vector3d& point) const
  {
    // 简单的以无人机为中心的地图范围检查
    double limit_dist = map_size_ * 0.5;
    return (point - current_pos_).norm() < limit_dist;
  }

  double ObjPlannerWrapper::calculateMinObstacleDistance(const Eigen::Vector3d& point) const
  {
    if (obstacles_.empty())
      return std::numeric_limits<double>::max();

    double min_dist = std::numeric_limits<double>::max();

    for (const auto& obs : obstacles_)
    {
      // AABB 距离计算
      double dx = std::max(0.0, std::abs(point.x() - obs.x) - obs.size_x / 2.0);
      double dy = std::max(0.0, std::abs(point.y() - obs.y) - obs.size_y / 2.0);
      double dz = std::max(0.0, std::abs(point.z() - obs.z) - obs.size_z / 2.0);

      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist < min_dist)
        min_dist = dist;
    }
    return min_dist;
  }

  Eigen::Vector3d ObjPlannerWrapper::calculateBestAvoidanceDirection() const
  {
    if (obstacles_.empty())
    {
      // 如果没有障碍物，优先指向目标
      return (target_pos_ - current_pos_).normalized();
    }

    // 【优化】优先使用路径切线方向作为参考基准，而不是单纯的目标连线
    Eigen::Vector3d ref_dir;
    if (!global_waypoints_.empty())
    {
      // 尝试使用最近路径段的方向作为参考
      double min_dist;
      int closest_seg = findClosestSegment(current_pos_, global_waypoints_, min_dist);

      if (closest_seg >= 0 && closest_seg < (int)global_waypoints_.size() - 1)
      {
        // 使用最近路径段的切线方向
        ref_dir = (global_waypoints_[closest_seg + 1] - global_waypoints_[closest_seg]).normalized();
        ROS_DEBUG_THROTTLE(2.0, "[OBJ_PLANNER] Using path tangent as ref_dir");
      }
      else
      {
        // 回退到目标连线
        ref_dir = (target_pos_ - current_pos_).normalized();
      }
    }
    else
    {
      // 没有全局路径，使用目标连线
      ref_dir = (target_pos_ - current_pos_).normalized();
    }

    double goal_yaw = std::atan2(ref_dir.y(), ref_dir.x());  // 参考方向的角度

    // 构建相对于参考方向的候选方向
    std::vector<Eigen::Vector3d> candidates;
    std::vector<double> relative_angles;  // 用于记录相对角度

    // 定义相对于参考方向的角度偏移（度数）
    std::vector<double> angle_offsets = {
      0,    // 0度：正前方（优先）
      30,   // 右前30度
      -30,  // 左前30度
      60,   // 右前60度
      -60,  // 左前60度
      90,   // 右侧90度
      -90,  // 左侧90度
      15,   // 右前15度
      -15   // 左前15度
    };

    for (double angle_deg : angle_offsets)
    {
      double angle_rad = angle_deg * M_PI / 180.0;
      double absolute_angle = goal_yaw + angle_rad;

      Eigen::Vector3d dir(std::cos(absolute_angle), std::sin(absolute_angle), 0);
      candidates.push_back(dir);
      relative_angles.push_back(angle_deg);
    }

    // 垂直避障选项（仅当前方完全堵死时使用）
    candidates.push_back(Eigen::Vector3d(0, 0, 1));   // 上
    relative_angles.push_back(999);  // 标记为垂直方向

    Eigen::Vector3d best_dir = Eigen::Vector3d::Zero();
    double best_score = -std::numeric_limits<double>::max();
    int best_idx = -1;

    for (size_t i = 0; i < candidates.size(); ++i)
    {
      double score = evaluateAvoidanceDirection(candidates[i], relative_angles[i]);
      if (score > best_score)
      {
        best_score = score;
        best_dir = candidates[i];
        best_idx = i;
      }
    }

    // 调试输出
    if (best_idx >= 0)
    {
      ROS_DEBUG_THROTTLE(1.0, "[OBJ_PLANNER] Best avoidance: idx=%d, angle_offset=%.1f°, score=%.2f",
               best_idx, relative_angles[best_idx], best_score);
    }

    return best_dir;
  }

  double ObjPlannerWrapper::evaluateAvoidanceDirection(const Eigen::Vector3d& dir, double relative_angle) const
  {
    double score = 0.0;

    // 1. 【优化】障碍物距离得分 (增加饱和封顶)
    double test_dist = 10.0; // 探测距离稍微加大到 10m
    Eigen::Vector3d check_pt = current_pos_ + dir * test_dist;
    double obs_dist = calculateMinObstacleDistance(check_pt);

    // 【关键】距离得分封顶！
    // 只要有 8米 的安全空间就足够了，超过 8米 不再加分
    // 防止因为侧面无限空旷(20m+)而导致得分过高，压过了角度权重
    double effective_dist = std::min(obs_dist, 8.0);
    score += effective_dist * 2.0; // Max score = 16.0

    // 2. 角度偏好得分
    if (relative_angle < 900)
    {
      // 使用高斯分布奖励接近0度的方向
      double angle_rad = std::abs(relative_angle) * M_PI / 180.0;
      // 稍微放宽 sigma 到 40度，让微小的绕行更容易被接受
      double angle_reward = std::exp(-angle_rad * angle_rad / (2.0 * 0.7 * 0.7));

      // 【关键】权重 12.0 (相比 max dist score 16.0，这个比重很健康)
      // 如果前方堵死(dist=0)，侧方30度(dist=8, angle=0.8) -> 16 + 9.6 = 25.6
      // 侧方90度(dist=8, angle=0) -> 16 + 0 = 16
      // 结果：选30度，而不是90度。
      score += angle_reward * 12.0;

      // 额外惩罚大角度
      if (std::abs(relative_angle) > 80.0)
      {
        score -= 30.0;
      }
    }

    // 3. 垂直惩罚
    score -= std::abs(dir.z()) * 5.0; // 加大垂直惩罚，尽量走水平

    return score;
  }

} // namespace statemachine
