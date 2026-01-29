#include "real_time_planner.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <sys/stat.h>  // 用于目录检查
#include <unistd.h>    // 用于 getcwd
#include <iomanip>  // 用于 std::fixed 和 std::setprecision

/*
 * @Function:Obj Planner:Real Time Planner Implementation
 * @Author:Fancy
 * @Date:2026 01
 */

namespace obj_planner {

// 构造函数定义
RealTimePlanner::RealTimePlanner() 
    : map_size_(200.0),  // 200m滚动窗口
      map_height_(100.0),
      map_initialized_(false) 
{
    local_planner_ = std::make_shared<PlannerInterface>();

    // 初始位置
    current_position_ = Eigen::Vector3d(50.0, 50.0, 50.0);
    current_velocity_ = Eigen::Vector3d::Zero();
    current_acceleration_ = Eigen::Vector3d::Zero();
    
    // 初始地图以起始位置为中心
    map_origin_ = current_position_ - Eigen::Vector3d(map_size_/2, map_size_/2, 0);
    
    std::cout << "🗺️ RealTimePlanner with rolling map initialized" << std::endl;
    std::cout << "   Initial map center: (" << (map_origin_.x() + map_size_/2) 
              << ", " << (map_origin_.y() + map_size_/2) << ")" << std::endl;
}

RealTimePlanner::~RealTimePlanner() {
    stopPlanning();
}

void RealTimePlanner::init(double max_vel, double max_acc, double max_jerk) {
    max_vel_ = max_vel;   // 保存参数
    max_acc_ = max_acc;
    max_jerk_ = max_jerk;
    local_planner_->initParam(max_vel, max_acc, max_jerk);
}

void RealTimePlanner::initEsdfMap(double x_size, double y_size, double z_size, 
                                 double resolution, Eigen::Vector3d origin, double inflate_values) {
    // 【新增】保存传入的参数到成员变量
    map_size_ = x_size; 
    map_height_ = z_size;
    map_resolution_ = resolution;
    map_inflate_ = inflate_values;
    local_planner_->initEsdfMap(x_size, y_size, z_size, resolution, origin, inflate_values);
}

void RealTimePlanner::setGlobalPath(const std::vector<PathPoint>& global_path) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_path_ = global_path;
    current_path_index_ = 0;
}

void RealTimePlanner::updateObstacles(const std::vector<Object3d>& obstacles) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_obstacles_ = obstacles;
    
    // 更新局部规划器的障碍物
    // local_planner_->setObstacles(obstacles);
    // 说明：
    // 障碍物的实际添加，会由 planningLoop -> performReplanning -> updateObstaclesInView 
    // 在每次规划周期开始时，根据当前地图位置自动筛选并添加。
}

void RealTimePlanner::updateRobotState(const Eigen::Vector3d& position, 
                                      const Eigen::Vector3d& velocity,
                                      const Eigen::Vector3d& acceleration) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_position_ = position;
    current_velocity_ = velocity;
    current_acceleration_ = acceleration;
}

void RealTimePlanner::startPlanning(double replan_rate) {
    if (planning_active_) {
        std::cout << "Planning is already active!" << std::endl;
        return;
    }
    
    replan_rate_ = replan_rate;
    planning_active_ = true;
    stop_requested_ = false;
    
    planning_thread_ = std::thread(&RealTimePlanner::planningLoop, this);
    
    std::cout << "Real-time planning started at " << replan_rate_ << " Hz" << std::endl;
}

void RealTimePlanner::stopPlanning() {
    stop_requested_ = true;
    planning_active_ = false;
    
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
    
    std::cout << "Real-time planning stopped" << std::endl;
}

void RealTimePlanner::planningLoop() {
    auto last_plan_time = std::chrono::steady_clock::now();
    
    while (!stop_requested_) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_plan_time);
        
        double elapsed_seconds = elapsed.count() / 1000.0;
        double target_interval = 1.0 / replan_rate_;
        
        if (elapsed_seconds >= target_interval) {
            bool success = performReplanning(current_position_, current_velocity_, current_acceleration_);
            
            if (success) {
                std::cout << "Replanning successful" << std::endl;
            } else {
                std::cout << "Replanning failed" << std::endl;
                handlePlanningFailure();
            }
            
            last_plan_time = current_time;
        }
        
        // 避免忙等待
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool RealTimePlanner::performReplanning(const Eigen::Vector3d& start_pos,
                                        const Eigen::Vector3d& start_vel,
                                        const Eigen::Vector3d& start_acc) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);

    auto total_start = std::chrono::high_resolution_clock::now();

    // [新增] 默认重置失败标志
    last_plan_failed_ = false; 

    // 地图更新检查
    if (needMapUpdate(start_pos)) {
        updateMapPosition(start_pos);
        updateObstaclesInView();
    }
    
    // 1. 获取原始目标点（可能跳变）
    Eigen::Vector3d raw_target = selectLocalTarget();

    // // 2. 目标点低通滤波（核心平滑逻辑）
    // if (!target_initialized_) {
    //     filtered_target_ = raw_target;
    //     target_initialized_ = true;
    // } else {
    //     // alpha 越小越平滑，建议 0.05
    //     double alpha = 0.05; 
    //     filtered_target_ = filtered_target_ * (1.0 - alpha) + raw_target * alpha;
    // }
    
    // // 【关键】后续所有计算都基于这个平滑后的点
    // Eigen::Vector3d local_target = filtered_target_;

   // ==================== 【修改核心：基于原始目标突变的逻辑】 ====================
    double alpha = 0.4; // 默认：快速响应 (对应 5m/s 巡航)

    if (!target_initialized_) {
        filtered_target_ = raw_target;
        last_raw_target_ = raw_target;
        target_initialized_ = true;
    } else {
        // 核心逻辑：检查"原始目标"是否发生了"瞬移"
        // 我们比较的是：这一帧的 raw_target vs 上一帧的 raw_target
        // 正常飞行时，raw_target 也会随飞机移动，但变化量 = 飞机速度 * dt
        // max_vel_ * 0.5s = 正常变化量。
        // 如果发生了避障，raw_target 会突变较大距离。
        // 我们将阈值设为 max_vel_ * 1.0，足以区分"正常飞行"和"避障跳变"。

        double raw_jump = (raw_target - last_raw_target_).norm();
        double jump_threshold = max_vel_ * 1.0; // 动态阈值，根据速度调整

        if (raw_jump > jump_threshold) {
            // 只有当原始目标真的跳变了，我们才启用强力平滑
            alpha = 0.05; 
            std::cout << "🐌 Avoidance Jump Detected! (" << raw_jump << "m)" << std::endl;
        } 
        
        // 执行滤波
        filtered_target_ = filtered_target_ * (1.0 - alpha) + raw_target * alpha;
        
        // 更新历史值
        last_raw_target_ = raw_target;
    }
    
    Eigen::Vector3d local_target = filtered_target_;


    // 3. 设定期望速度
    // ✅ 使用成员变量
    double cruise_speed = max_vel_; // 使用传入的最大速度作为巡航速度

    // B-Spline 优化器初始化: (最大速度, 最大加速度, 最大加加速度)
    // 通常优化器的约束可以设置得比物理极限稍大一点点，或者严格相等
    local_planner_->initParam(max_vel_, max_acc_, max_jerk_);

    // 计算目标速度向量
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d diff = local_target - start_pos;
    double dist_to_local_target = diff.norm();

    if (dist_to_local_target > 0.1) {
        Eigen::Vector3d target_dir = diff.normalized();
        target_vel = target_dir * cruise_speed;
    } else {
        if (start_vel.norm() > 0.1) {
             target_vel = start_vel.normalized() * cruise_speed;
        }
    }

    // 4. 终点减速逻辑
    if (!global_path_.empty()) {
        Eigen::Vector3d final_goal(global_path_.back().x, global_path_.back().y, global_path_.back().z);
        double dist_to_final = (final_goal - start_pos).norm();
        // 距离终点小于减速距离时，期望速度设为 0
        // 减速距离基于最大速度动态调整
        double decel_distance = max_vel_ * 2.5; // 2.5秒的制动距离
        if (dist_to_final < decel_distance) {
            target_vel = Eigen::Vector3d::Zero();
        }
    }

    // 5. 边界检查
    if (!isInLocalMap(local_target)) {
         local_target.x() = std::max(map_origin_.x() + 10.0, std::min(map_origin_.x() + map_size_ - 10.0, local_target.x()));
         local_target.y() = std::max(map_origin_.y() + 10.0, std::min(map_origin_.y() + map_size_ - 10.0, local_target.y()));
         local_target.z() = std::max(10.0, std::min(map_height_ - 10.0, local_target.z()));
    }

    // 6. 生成路点（使用滤波后的目标）
    std::vector<Eigen::Vector3d> local_waypoints = generateLocalWaypoints(local_target);
    
    if (!local_waypoints.empty()) {
        local_waypoints[0] = start_pos; 
    }
    
    std::vector<PathPoint> path_points;
    for (const auto& wp : local_waypoints) {
        PathPoint point; point.x = wp(0); point.y = wp(1); point.z = wp(2);
        path_points.push_back(point);
    }
    local_planner_->setPathPoint(path_points);
    
    // 7. 执行规划
    bool plan_success = local_planner_->makePlanWithState(
        start_pos,   
        start_vel,   
        start_acc,   
        local_target, 
        target_vel, 
        Eigen::Vector3d::Zero()
    );

    if (!plan_success) {
        // std::cout << "❌ Planner backend failed." << std::endl;
        last_plan_failed_ = true; // [新增] 标记规划失败
        return false;
    }

    // 8. 获取结果
    std::vector<PathPoint> new_trajectory;
    local_planner_->getLocalPlanTrajResults(new_trajectory);
    
    if (new_trajectory.empty()) return false;

    // 9. 更新轨迹
    {
        std::lock_guard<std::mutex> traj_lock(trajectory_mutex_);
        current_trajectory_ = new_trajectory;
    }
    
    // 10. 导出数据
    exportDataForPython(global_path_, new_trajectory, current_obstacles_);

    return true;
}

Eigen::Vector3d RealTimePlanner::selectLocalTarget() {
    // [新增] 每次选择目标前重置避障状态
    avoidance_triggered_ = false;

    if (global_path_.empty()) {
        return current_position_;
    }
    
    // 1. 找到全局路径上距离当前位置最近的点
    int start_index = 0;
    double min_dist_to_path = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_path_.size(); ++i) {
        Eigen::Vector3d pt(global_path_[i].x, global_path_[i].y, global_path_[i].z);
        double dist = (pt - current_position_).norm();
        if (dist < min_dist_to_path) {
            min_dist_to_path = dist;
            start_index = i;
        }
    }

    // 2. 默认目标：沿全局路径向前看
    // double lookahead_dist = 30.0; 
    // 【修改后】根据当前速度动态调整，或者直接加大
    // 建议：至少是 当前速度 * 3.0秒，例如 10m/s 时看 30m，甚至 50m
    double lookahead_dist = std::max(30.0, current_velocity_.norm() * 4.0); 

    // 限制最大前瞻，防止超出地图范围太多
    lookahead_dist = std::min(lookahead_dist, 60.0);

    Eigen::Vector3d raw_target = current_position_;
    
    for (size_t i = start_index; i < global_path_.size(); ++i) {
        const PathPoint& path_point = global_path_[i];
        Eigen::Vector3d point(path_point.x, path_point.y, path_point.z);
        if ((point - current_position_).norm() > lookahead_dist) {
            raw_target = point;
            break;
        }
        raw_target = point; 
    }

    // 3. 【修改一】更合理的检测阈值，防止误报
    bool obstacle_blocking = false;
    double dist_to_target = (raw_target - current_position_).norm();
    Eigen::Vector3d dir_to_target = (raw_target - current_position_).normalized();

    // 建议将检测半径设为 1.0 米左右
    double check_radius = 1.0;

    for (double d = 2.0; d < dist_to_target; d += 2.0) {
        Eigen::Vector3d check_pt = current_position_ + dir_to_target * d;
        if (calculateMinObstacleDistance(check_pt) < check_radius) {
            obstacle_blocking = true;
            // std::cout << "⚠️ Path blocked..." << std::endl;
            break;
        }
    }

    if (obstacle_blocking) {
        // [新增] 检测到阻挡，标记为避障状态
        avoidance_triggered_ = true;
        std::cout << "🛡️ Obstacle detected, switching to AVOIDANCE mode." << std::endl;

        Eigen::Vector3d best_dir = calculateBestAvoidanceDirection();

        // 【修改二】动态搜索安全落点，防止把目标定在墙里
        // 从 8米 开始尝试，逐渐缩短距离
        for (double offset = 8.0; offset >= 2.0; offset -= 1.0) {
            Eigen::Vector3d candidate = raw_target + best_dir * offset;

            // 简单的垂直高度限制（保留原逻辑）
            double vertical_diff = candidate.z() - current_position_.z();
            if (std::abs(vertical_diff) > 3.0) {
                 candidate.z() = current_position_.z() + (vertical_diff > 0 ? 3.0 : -3.0);
            }

            // 关键验证：新目标必须在地图内 且 不在障碍物里
            if (isInLocalMap(candidate) && calculateMinObstacleDistance(candidate) > check_radius) {
                std::cout << "✅ Found safe avoidance target at offset " << offset << "m" << std::endl;
                return candidate;
            }
        }

        // 如果所有尝试都失败（死胡同），保持原位或悬停，不要强行穿墙
        std::cout << "⚠️ Cannot find safe avoidance target! Holding position." << std::endl;
        return current_position_;
    }

    return raw_target;
}

// 在 generateLocalWaypoints 函数中，添加状态继承
// std::vector<Eigen::Vector3d> RealTimePlanner::generateLocalWaypoints() {
//     std::vector<Eigen::Vector3d> waypoints;
//     Eigen::Vector3d local_target = selectLocalTarget();
//     double distance = (local_target - current_position_).norm();
    
//     // // 【修改】强制最少点数为 15，无论距离多近
//     // int min_points = 15; 
//     // int calculated_points = static_cast<int>(distance / 3.0);
//     // int num_points = std::max(min_points, calculated_points);

//     // ================== 【修改核心】 ==================
//     // 原逻辑：min_points = 15, spacing = 3.0 -> 导致点太密，速度被锁死在 3.3m/s
//     // 新逻辑：稀疏化点位，匹配 10m/s 的底层时间步长 (ts=0.6s)
//     // 目标间距 = max_vel * ts = 10 * 0.6 = 6.0m
    
//     int min_points = 10; // 只要能构成B样条(order=3)，最少4-5个点就够了
//     int calculated_points = static_cast<int>(distance / 3.0); // 间距拉大
//     int num_points = std::max(min_points, calculated_points);
    
//     // 防止点数过少导致B样条退化（至少保证有起、终和中间几个点）
//     if (num_points < 5) num_points = 5; 
//     // =================================================
    
//     // 即使目标就是当前位置（悬停），也要生成 15 个重叠的点
//     if (distance < 0.1) {
//         std::cout << "⚠️ Generating hover waypoints (stacking " << num_points << " points)" << std::endl;
//         for (int i = 0; i < num_points; ++i) {
//             waypoints.push_back(current_position_);
//         }
//         return waypoints;
//     }
    
//     // 正常路径生成
//     waypoints.push_back(current_position_);
//     for (int i = 1; i < num_points; ++i) {
//         double t = static_cast<double>(i) / (num_points - 1);
//         waypoints.push_back(current_position_ + (local_target - current_position_) * t);
//     }
    
//     return waypoints;
// }

// 【修改后】增加参数 target，不再并在内部调用 selectLocalTarget()
std::vector<Eigen::Vector3d> RealTimePlanner::generateLocalWaypoints(const Eigen::Vector3d& target) {
    std::vector<Eigen::Vector3d> waypoints;
    
    double distance = (target - current_position_).norm();
    
    // 密度控制逻辑
    int min_points = 10; 
    int calculated_points = static_cast<int>(distance / 3.0); // 间距 7米
    int num_points = std::max(min_points, calculated_points);
    if (num_points < 5) num_points = 5; 

    // 悬停逻辑
    if (distance < 0.1) {
        for (int i = 0; i < num_points; ++i) {
            waypoints.push_back(current_position_);
        }
        return waypoints;
    }
    
    // 路径生成
    Eigen::Vector3d segment_vec = target - current_position_;
    
    waypoints.push_back(current_position_);
    for (int i = 1; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        waypoints.push_back(current_position_ + segment_vec * t);
    }
    
    return waypoints;
}

double RealTimePlanner::calculateLookaheadDistance() const {
    // 基于当前速度计算前瞻距离
    double current_speed = current_velocity_.norm();
    // 前瞻距离基于最大速度动态调整
    double min_lookahead = max_vel_ * 20.0;    // 最小前瞻距离：20倍最大速度
    double max_lookahead = max_vel_ * 50.0;    // 最大前瞻距离：50倍最大速度
    double speed_factor = 15.0;                // 速度因子

    double lookahead = min_lookahead + speed_factor * current_speed;
    return std::min(lookahead, max_lookahead);
}

bool RealTimePlanner::checkTrajectorySafety(const std::vector<PathPoint>& traj) {
    if (traj.empty()) {
        return false;
    }
    
    // 检查轨迹起点是否与当前位置匹配
    const auto& traj_start = traj.front();
    Eigen::Vector3d traj_start_pos(traj_start.x, traj_start.y, traj_start.z);
    double start_error = (traj_start_pos - current_position_).norm();
    
    if (start_error > 10.0) {
        std::cout << "❌ Trajectory start error too large: " << start_error << "m" << std::endl;
        return false;
    }
    
    // 严格的轨迹安全检查
    int collision_count = 0;
    const int max_collisions = traj.size() / 50; // 只允许2%的碰撞点，大幅提高安全性
    
    for (const auto& point : traj) {
        Eigen::Vector3d pos(point.x, point.y, point.z);
        
        // 检查是否与障碍物碰撞
        for (const auto& obstacle : current_obstacles_) {
            Eigen::Vector3d obs_center(obstacle.rect3d.center.x, 
                                      obstacle.rect3d.center.y, 
                                      obstacle.rect3d.center.z);
            Eigen::Vector3d obs_size(obstacle.rect3d.size.x, 
                                    obstacle.rect3d.size.y, 
                                    obstacle.rect3d.size.z);
            
            // 简单的AABB碰撞检测
            bool collision = true;
            for (int i = 0; i < 3; ++i) {
                if (std::abs(pos(i) - obs_center(i)) > (obs_size(i) / 2.0 + safety_margin_)) {
                    collision = false;
                    break;
                }
            }
            
            if (collision) {
                collision_count++;
                if (collision_count <= 5) { // 只打印前5次碰撞
                    std::cout << "⚠️ Trajectory collision at (" << pos.x() << ", " 
                              << pos.y() << ", " << pos.z() << ") with obstacle at ("
                              << obs_center.x() << ", " << obs_center.y() << ", " 
                              << obs_center.z() << ")" << std::endl;
                }
                
                if (collision_count > max_collisions) {
                    std::cout << "❌ Too many collisions: " << collision_count 
                              << " > " << max_collisions << std::endl;
                    return false;
                }
            }
        }
    }
    
    if (collision_count > 0) {
        std::cout << "⚠️ Trajectory has " << collision_count << " collision points (allowed: " 
                  << max_collisions << ")" << std::endl;
    }
    
    return true;
}

void RealTimePlanner::handlePlanningFailure() {
    std::cout << "🚨 Planning failed! Executing Emergency Logic..." << std::endl;
    
    std::vector<PathPoint> emergency_trajectory;
    double hover_duration = 2.0; 
    int num_points = 20;
    
    // 【修改】设定最大天花板高度（例如 100米）
    // 你的障碍物最高也就 70-80米，飞到 100米 足够了，别飞到 200米去
    double max_ceiling = 100.0; 
    
    // 检查是否已经到达天花板
    bool reach_ceiling = current_position_.z() > max_ceiling;

    for (int i = 0; i < num_points; ++i) {
        PathPoint point;
        point.x = current_position_.x();
        point.y = current_position_.y();
        
        if (reach_ceiling) {
            // 【关键】如果已经很高了，保持高度，禁止继续上升！
            point.z = current_position_.z(); 
            point.vz = 0.0;
        } else {
            // 否则缓慢上升
            point.z = current_position_.z() + (i * 0.5 * (hover_duration / num_points));
            point.vz = 0.5;
        }
        
        point.vx = 0.0; point.vy = 0.0; 
        point.speed = point.vz;
        point.ax = 0.0; point.ay = 0.0; point.az = 0.0;
        point.yaw = 0.0; 
        if (current_velocity_.norm() > 0.1) {
            point.yaw = atan2(current_velocity_.y(), current_velocity_.x());
        }
        
        emergency_trajectory.push_back(point);
    }
    
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_trajectory_ = emergency_trajectory;
    }
    
    if (reach_ceiling) {
        std::cout << "✅ Emergency: Ceiling reached (" << current_position_.z() << "m). Holding altitude." << std::endl;
    } else {
        std::cout << "✅ Emergency: Climbing..." << std::endl;
    }
}



bool RealTimePlanner::getCurrentTrajectory(std::vector<PathPoint>& trajectory) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (current_trajectory_.empty()) {
        return false;
    }
    
    trajectory = current_trajectory_;
    return true;
}

bool RealTimePlanner::isGoalReached() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (global_path_.empty()) {
        return false;
    }
    
    const PathPoint& last_point = global_path_.back();
    Eigen::Vector3d goal(last_point.x, last_point.y, last_point.z);
    double distance_to_goal = (goal - current_position_).norm();
    
    // 到达距离基于最大速度动态调整
    double goal_threshold = max_vel_ * 2.0; // 2秒的飞行距离
    goal_threshold = std::max(1.0, goal_threshold); // 最小1米阈值

    return distance_to_goal < goal_threshold;
}

PlanningStatus RealTimePlanner::getPlanningStatus() const {
    if (isGoalReached()) {
        return PlanningStatus::GOAL_REACHED;
    }
    
    if (!planning_active_) {
        return PlanningStatus::IDLE;
    }
    
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (current_trajectory_.empty()) {
        return PlanningStatus::FAILED;
    }
    
    return PlanningStatus::EXECUTING;
}

void RealTimePlanner::exportDataForPython(const std::vector<PathPoint>& global_traj,
                                        const std::vector<PathPoint>& local_traj,
                                        const std::vector<Object3d>& obstacles) {
    
    // 创建数据目录
    std::string base_dir = "./planning_data/";
    std::string command = "mkdir -p " + base_dir;
    system(command.c_str());
    
    // 使用时间戳创建唯一文件名
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    std::string timestamp_str = std::to_string(timestamp);
    std::string data_dir = base_dir + "replan_" + timestamp_str + "/";
    command = "mkdir -p " + data_dir;
    system(command.c_str());
    
    std::cout << "💾 Saving planning data to: " << data_dir << std::endl;
    
    // 1. 导出全局轨迹（只在第一次导出）
    static bool global_exported = false;
    if (!global_exported) {
        std::string global_file = base_dir + "global_trajectory.txt";
        std::ofstream global_stream(global_file);
        for (const auto& point : global_traj) {
            global_stream << point.x << " " << point.y << " " << point.z << std::endl;
        }
        global_stream.close();
        global_exported = true;
        std::cout << "✅ Global trajectory saved" << std::endl;
    }
    
    // 2. 导出本次局部轨迹（包含完整运动信息）
    std::string local_file = data_dir + "local_trajectory.txt";
    std::ofstream local_stream(local_file);
    
    // 添加时间戳信息作为文件头
    local_stream << "# Replanning timestamp: " << timestamp << std::endl;
    local_stream << "# Format: x y z vx vy vz speed yaw pitch ax ay az" << std::endl;
    
    for (const auto& point : local_traj) {
        local_stream << point.x << " " << point.y << " " << point.z << " "
                    << point.vx << " " << point.vy << " " << point.vz << " "
                    << point.speed << " " 
                    << point.yaw << " " << point.pitch << " "
                    << point.ax << " " << point.ay << " " << point.az << std::endl;
    }
    local_stream.close();
    
    // 3. 导出本次障碍物信息
    std::string obstacle_file = data_dir + "obstacles.txt";
    std::ofstream obstacle_stream(obstacle_file);
    
    // 添加局部地图边界信息
    if (!local_traj.empty()) {
        const auto& current_pos = local_traj.front();
        obstacle_stream << "##LOCAL_MAP_BOUNDARY## " 
                     << current_pos.x << " " << current_pos.y << " " << current_pos.z << " "
                     << "120.0 120.0 80.0" << std::endl;
    }
    
    // 添加当前地图边界信息
    obstacle_stream << "##LOCAL_MAP_BOUNDARY## " 
                 << map_origin_.x() << " " << map_origin_.y() << " " << map_origin_.z() << " "
                 << map_size_ << " " << map_size_ << " " << map_height_ << std::endl;
    
    for (const auto& obj : obstacles) {
        obstacle_stream << obj.rect3d.center.x << " " 
                    << obj.rect3d.center.y << " " 
                    << obj.rect3d.center.z << " " 
                    << obj.rect3d.size.x << " " 
                    << obj.rect3d.size.y << " " 
                    << obj.rect3d.size.z << std::endl;
    }
    obstacle_stream.close();
    
    // 4. 导出规划状态信息
    std::string status_file = data_dir + "planning_status.txt";
    std::ofstream status_stream(status_file);
    
    if (!local_traj.empty()) {
        const auto& start = local_traj.front();
        const auto& end = local_traj.back();
        
        status_stream << "Replanning Timestamp: " << timestamp << std::endl;
        status_stream << "Local Trajectory Info:" << std::endl;
        status_stream << "Start: " << start.x << " " << start.y << " " << start.z << std::endl;
        status_stream << "End: " << end.x << " " << end.y << " " << end.z << std::endl;
        status_stream << "Length: " << local_traj.size() << " points" << std::endl;
        status_stream << "Distance: " << sqrt(pow(end.x-start.x,2) + pow(end.y-start.y,2) + pow(end.z-start.z,2)) << "m" << std::endl;
        
        if (local_traj.size() > 1) {
            double total_time = local_traj.size() * 0.1;
            status_stream << "Estimated duration: " << total_time << "s" << std::endl;
        }
        
        // 添加运动学信息
        if (local_traj.size() >= 7) {
            double max_speed = 0.0;
            double max_acc = 0.0;
            for (const auto& point : local_traj) {
                if (point.speed > max_speed) max_speed = point.speed;
                double acc = sqrt(point.ax*point.ax + point.ay*point.ay + point.az*point.az);
                if (acc > max_acc) max_acc = acc;
            }
            status_stream << "Max Speed: " << max_speed << " m/s" << std::endl;
            status_stream << "Max Acceleration: " << max_acc << " m/s²" << std::endl;
        }
    }
    
    status_stream.close();
    
    // 5. 更新最新的数据链接（用于实时可视化）
    // std::string latest_link = base_dir + "latest/";
    // command = "rm -f " + latest_link + " && ln -sf " + data_dir + " " + latest_link;
    // system(command.c_str());
    
    std::cout << "✅ Planning data saved to: " << data_dir << std::endl;
    std::cout << "   Trajectory points: " << local_traj.size() << std::endl;
    std::cout << "   Obstacles: " << obstacles.size() << std::endl;
}

bool RealTimePlanner::isTargetReachable(const Eigen::Vector3d& target) const {
    // 修正：使用地图范围检查
    if (!isInLocalMap(target)) {
        std::cout << "❌ Target is outside map boundaries: (" 
                  << target.x() << ", " << target.y() << ", " << target.z() << ")" << std::endl;
        return false;
    }
    
    // 检查目标是否在障碍物内
    for (const auto& obstacle : current_obstacles_) {
        Eigen::Vector3d obs_center(obstacle.rect3d.center.x, 
                                  obstacle.rect3d.center.y, 
                                  obstacle.rect3d.center.z);
        Eigen::Vector3d obs_size(obstacle.rect3d.size.x, 
                                obstacle.rect3d.size.y, 
                                obstacle.rect3d.size.z);
        
        // AABB碰撞检测
        bool collision = true;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(target(i) - obs_center(i)) > (obs_size(i) / 2.0 + safety_margin_)) {
                collision = false;
                break;
            }
        }
        
        if (collision) {
            std::cout << "❌ Target is inside obstacle at (" 
                      << obs_center.x() << ", " << obs_center.y() << ", " 
                      << obs_center.z() << ")" << std::endl;
            return false;
        }
    }
    
    return true;
}

bool RealTimePlanner::isPointSafe(const Eigen::Vector3d& point) const {
    // 修正：使用地图范围检查
    if (!isInLocalMap(point)) {
        return false;
    }
    
    // 检查是否与障碍物碰撞
    for (const auto& obstacle : current_obstacles_) {
        Eigen::Vector3d obs_center(obstacle.rect3d.center.x, 
                                  obstacle.rect3d.center.y, 
                                  obstacle.rect3d.center.z);
        Eigen::Vector3d obs_size(obstacle.rect3d.size.x, 
                                obstacle.rect3d.size.y, 
                                obstacle.rect3d.size.z);
        
        // AABB碰撞检测
        bool collision = true;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(point(i) - obs_center(i)) > (obs_size(i) / 2.0 + safety_margin_)) {
                collision = false;
                break;
            }
        }
        
        if (collision) {
            std::cout << "❌ Point (" << point.x() << ", " << point.y() << ", " << point.z() 
                      << ") collides with obstacle at (" << obs_center.x() << ", " 
                      << obs_center.y() << ", " << obs_center.z() << ")" << std::endl;
            return false;
        }
    }
    
    return true;
}


    // 新增：检查是否需要移动地图
    bool RealTimePlanner::needMapUpdate(const Eigen::Vector3d& robot_position) {
        if (!map_initialized_) return true;
        
        // 计算机器人当前位置到地图中心的距离
        Eigen::Vector3d map_center = map_origin_ + Eigen::Vector3d(map_size_/2, map_size_/2, map_height_/2);
        double distance_to_center = (robot_position - map_center).norm();
        
        // 如果机器人离地图中心太远（超过地图半径的40%），需要更新地图
        return distance_to_center > map_size_ * 0.4;
    }
    
    // 新增：更新地图位置
    void RealTimePlanner::updateMapPosition(const Eigen::Vector3d& robot_position) {
        // 计算新的地图原点，使机器人位于地图中心
        Eigen::Vector3d new_origin = robot_position - Eigen::Vector3d(map_size_/2, map_size_/2, 0);
        
        // 确保地图不会超出全局路径范围（0-2000m）
        new_origin.x() = std::max(0.0, std::min(2000.0 - map_size_, new_origin.x()));
        new_origin.y() = std::max(0.0, std::min(2000.0 - map_size_, new_origin.y()));
        new_origin.z() = 0.0;
        
        // 如果地图位置有显著变化，更新地图
        if ((new_origin - map_origin_).norm() > 10.0) {
            std::cout << "🔄 Rolling local map to new position" << std::endl;
            std::cout << "   Old map: [" << map_origin_.x() << " to " << map_origin_.x() + map_size_ 
                    << "] x [" << map_origin_.y() << " to " << map_origin_.y() + map_size_ << "]" << std::endl;
            
            map_origin_ = new_origin;
            
            std::cout << "   New map: [" << map_origin_.x() << " to " << map_origin_.x() + map_size_ 
                    << "] x [" << map_origin_.y() << " to " << map_origin_.y() + map_size_ << "]" << std::endl;
            std::cout << "   Robot at: (" << robot_position.x() << ", " << robot_position.y() << ")" << std::endl;
            
            // 重新初始化ESDF地图到新位置
            reinitializeLocalMap();
            
            // 重新添加新地图范围内的障碍物
            updateObstaclesInView();
            
            map_initialized_ = true;
        }
    }
    
    void RealTimePlanner::reinitializeLocalMap() {
        double resolution = map_resolution_;
        double x_size = map_size_;
        double y_size = map_size_;  
        double z_size = map_height_;
        double inflateValue = map_inflate_;
        
        std::cout << "   Reinitializing ESDF map at origin: (" 
                << map_origin_.x() << ", " << map_origin_.y() << ", " << map_origin_.z() << ")" << std::endl;
        
        local_planner_->initEsdfMap(x_size, y_size, z_size, resolution, map_origin_, inflateValue);
        // 【修改】使用保存的 map_resolution_ 和 map_inflate_
        // local_planner_->initEsdfMap(x_size, y_size, z_size, map_resolution_, map_origin_, map_inflate_);
    }
    
      void RealTimePlanner::updateObstaclesInView() {
        // 重新添加在新地图范围内的障碍物
        std::vector<Object3d> local_obstacles;
        int added_count = 0;

        for (const auto& obstacle : current_obstacles_) {
            Eigen::Vector3d obs_center(obstacle.rect3d.center.x,
                                    obstacle.rect3d.center.y,
                                    obstacle.rect3d.center.z);

            // 检查障碍物是否在新地图范围内
            if (isInLocalMap(obs_center)) {
                local_obstacles.push_back(obstacle);
                added_count++;

                // if (added_count <= 5) { // 只打印前5个障碍物信息
                //     std::cout << "   Adding obstacle at (" << obs_center.x() << ", "
                //             << obs_center.y() << ", " << obs_center.z() << ")" << std::endl;
                // }
            }
        }

        // 修改：传入当前无人机位置
        local_planner_->setObstacles(local_obstacles, current_position_);
        std::cout << "   Added " << local_obstacles.size() << " obstacles to local map with drone position ("
                  << current_position_.x() << ", " << current_position_.y() << ", " << current_position_.z() << ")" << std::endl;
    }
    
   bool RealTimePlanner::isInLocalMap(const Eigen::Vector3d& point) const {
        return point.x() >= map_origin_.x() && point.x() <= map_origin_.x() + map_size_ &&
            point.y() >= map_origin_.y() && point.y() <= map_origin_.y() + map_size_ &&
            point.z() >= map_origin_.z() && point.z() <= map_origin_.z() + map_height_;
    }

    double RealTimePlanner::calculateMinObstacleDistance(const Eigen::Vector3d& point) const {
        if (current_obstacles_.empty()) {
            return std::numeric_limits<double>::max();
        }
        
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& obstacle : current_obstacles_) {
            Eigen::Vector3d obs_center(obstacle.rect3d.center.x, 
                                    obstacle.rect3d.center.y, 
                                    obstacle.rect3d.center.z);
            Eigen::Vector3d obs_size(obstacle.rect3d.size.x, 
                                    obstacle.rect3d.size.y, 
                                    obstacle.rect3d.size.z);
            
            // 计算点到障碍物边界的最小距离
            double dx = std::max(0.0, std::abs(point.x() - obs_center.x()) - obs_size.x()/2.0);
            double dy = std::max(0.0, std::abs(point.y() - obs_center.y()) - obs_size.y()/2.0);
            double dz = std::max(0.0, std::abs(point.z() - obs_center.z()) - obs_size.z()/2.0);
            
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            min_distance = std::min(min_distance, distance);
        }
        
        return min_distance;
    }

    Eigen::Vector3d RealTimePlanner::calculateBestAvoidanceDirection() const {
        if (current_obstacles_.empty()) {
            // 如果没有障碍物，使用全局路径方向
            if (!global_path_.empty()) {
                Eigen::Vector3d goal(global_path_.back().x, global_path_.back().y, global_path_.back().z);
                return (goal - current_position_).normalized();
            }
            return Eigen::Vector3d(1.0, 0.0, 0.5).normalized(); // 默认向前向上
        }
        
        // 分析障碍物分布，选择最佳避障方向
        std::vector<Eigen::Vector3d> candidate_directions = {
            // === 新增：纯水平避障方向 (Z=0) ===
            Eigen::Vector3d(0.0, 1.0, 0.0),   // 纯左 (假设Y轴为左)
            Eigen::Vector3d(0.0, -1.0, 0.0),  // 纯右
            Eigen::Vector3d(1.0, 1.0, 0.0),   // 左前
            Eigen::Vector3d(1.0, -1.0, 0.0),  // 右前
            
            // === 原有：向上避障方向 (保留用于当左右都被堵死时) ===
            Eigen::Vector3d(1.0, 0.0, 0.5),   
            Eigen::Vector3d(0.0, 1.0, 0.5),   
            Eigen::Vector3d(-1.0, 0.0, 0.5),
            // ... 可以减少一些冗余的向上方向，保留几个关键的即可
            Eigen::Vector3d(0.0, 0.0, 1.0)    // 纯向上 (作为最后保底)
        };
        
        Eigen::Vector3d best_direction;
        double best_score = -std::numeric_limits<double>::max();
        
        for (const auto& dir : candidate_directions) {
            double score = evaluateAvoidanceDirection(dir);
            if (score > best_score) {
                best_score = score;
                best_direction = dir;
            }
        }
        
        // 如果所有方向都不好，使用向上避障
        if (best_score < 0) {
            return Eigen::Vector3d(0.0, 0.0, 1.0); // 直接向上
        }
        
        return best_direction.normalized();
    }
    
    double RealTimePlanner::evaluateAvoidanceDirection(const Eigen::Vector3d& direction) const {
        double score = 0.0;

        // 检查方向上的障碍物距离
        Eigen::Vector3d test_point = current_position_ + direction * 20.0; // 测试20米外的点
        double min_distance = calculateMinObstacleDistance(test_point);

        // 距离越远，分数越高
        score += min_distance * 2.0;

        // 考虑全局路径方向的一致性
        if (!global_path_.empty()) {
            Eigen::Vector3d goal(global_path_.back().x, global_path_.back().y, global_path_.back().z);
            Eigen::Vector3d to_goal = (goal - current_position_).normalized();
            double alignment = direction.dot(to_goal);
            score += alignment * 10.0; // 与目标方向一致加分
        }

        // 考虑当前速度方向
        if (current_velocity_.norm() > 0.1) {
            double velocity_alignment = direction.dot(current_velocity_.normalized());
            double velocity_bonus = velocity_alignment * max_vel_; // 与速度方向一致加分，基于最大速度
            score += velocity_bonus;
        }

        // 新增：惩罚垂直分量，避免无脑向上飞
        // 因为爬升通常比水平移动更耗能，应该作为最后的选择
        double vertical_penalty_weight = 50.0; // 垂直移动惩罚权重
        double vertical_component = std::abs(direction.z()); // Z轴分量的绝对值
        score -= vertical_component * vertical_penalty_weight;

        // 如果是向上移动（正Z方向），额外增加惩罚
        if (direction.z() > 0) {
            score -= 30.0; // 额外惩罚向上飞
        }

        return score;
    }
    
    double RealTimePlanner::calculateObstacleDensity() const {
        if (current_obstacles_.empty()) {
            return 0.0;
        }
        
        // 计算当前位置周围20米范围内的障碍物密度
        double search_radius = 20.0;
        int obstacle_count = 0;
        
        for (const auto& obstacle : current_obstacles_) {
            Eigen::Vector3d obs_center(obstacle.rect3d.center.x, 
                                    obstacle.rect3d.center.y, 
                                    obstacle.rect3d.center.z);
            double distance = (obs_center - current_position_).norm();
            
            if (distance <= search_radius) {
                obstacle_count++;
            }
        }
        
        // 归一化密度（0-1范围）
        double max_expected_obstacles = 10.0; // 20米半径内最多10个障碍物
        return std::min(1.0, obstacle_count / max_expected_obstacles);
    }

} // namespace obj_planner