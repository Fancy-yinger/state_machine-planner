#include "planner_interface.h"
#include "obstacle_types.h"

#include <fstream>
#include <iostream>
#include <chrono>  // 【新增】用于手动节流控制

/*
 * @Function:Obj Planner:Planner Interface
 * @Author:Fancy
 * @Date:2026 01
 */

namespace obj_planner
{

    PlannerInterface::PlannerInterface()
    {

    }

    PlannerInterface::~PlannerInterface()
    {

    }

    void PlannerInterface::initParam(double max_vel,double max_acc,double max_jerk)
    {
        // 安全验证模式参数
        pp_.max_vel_ = max_vel;
        pp_.max_acc_ = max_acc;
        pp_.max_jerk_ = max_jerk;

        pp_.feasibility_tolerance_ = 0.05;

        // ========== 安全验证配置 ==========
        // 使用中等控制点间距，既能处理0.5m小障碍物，又不会过于密集
        pp_.ctrl_pt_dist = 0.5;  // 【验证】暂时设为0.5m，验证小障碍物避障能力

        // 适度规划视野，平衡前瞻性和计算效率
        pp_.planning_horizen_ = 50.0;  // 扩大规划视野至50m，适应3m/s高速飞行
        // ========== 安全验证配置结束 ==========
    }
    
    void PlannerInterface::initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d origin,double inflate_values)
    {
        std::cout << "x_size =" << x_size << " y_size =" << y_size << " z_size" << z_size << std::endl;
        std::cout << "resolution =" << resolution << std::endl;
        std::cout << "origin =" << origin << std::endl;
        std::cout << "inflate_values =" << inflate_values << std::endl;
        //初始化ESDF地图
        grid_map_.reset(new SDFMap);
        grid_map_->initMap(x_size,y_size,z_size,resolution,origin,inflate_values);
        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam();
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        // bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(500, 500, 100));
        // 修复：根据实际地图尺寸计算A*搜索器大小，确保足够大
        Eigen::Vector3i grid_size(
            ceil(x_size / resolution),  // 增加缓冲区
            ceil(y_size / resolution),
            ceil(z_size / resolution)
        );
        
        std::cout << "A* Grid size: " << grid_size.transpose() << std::endl;
        
        // 确保搜索器足够大以覆盖整个地图
        bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, grid_size);
       
    }

    void PlannerInterface::setPathPoint(std::vector<PathPoint> &plan_traj)
    {
        _global_plan_traj_.clear();
        _global_plan_traj_ = plan_traj;
    }
    
    void PlannerInterface::setObstacles(std::vector<ObstacleInfo> &obstacle, Eigen::Vector3d current_drone_pos)
    {
        // ================= 1. 参数配置 =================
        // 定义局部感知半径（扩大至 60米，大于规划视野50m，确保安全）
        const double SENSING_HORIZON = 60.0;
        const double SENSING_HEIGHT_Z = 10.0; // 高度方向可以小一点，比如上下10米

        // 【优化】改为每2秒打印一次，避免刷屏 (使用静态变量实现节流)
        static std::chrono::high_resolution_clock::time_point last_sensing_print = std::chrono::high_resolution_clock::now();
        auto current_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_sensing_print).count() > 2000) {
            std::cout << "🔍 Sensing: " << SENSING_HORIZON << "m radius, " << SENSING_HEIGHT_Z << "m height | Drone: ("
                      << current_drone_pos.x() << ", " << current_drone_pos.y() << ", " << current_drone_pos.z() << ")" << std::endl;
            last_sensing_print = current_time;
        }

        // ================= 2. 执行地图清除 (Reset) =================
        // 计算以飞机为中心的清除包围盒
        Eigen::Vector3d clear_min, clear_max;

        clear_min = current_drone_pos - Eigen::Vector3d(SENSING_HORIZON, SENSING_HORIZON, SENSING_HEIGHT_Z);
        clear_max = current_drone_pos + Eigen::Vector3d(SENSING_HORIZON, SENSING_HORIZON, SENSING_HEIGHT_Z);

        // 【修正开始】从 grid_map_ 动态获取边界，适应滚动地图
        Eigen::Vector3d map_origin = grid_map_->getOrigin();
        Eigen::Vector3d map_size = grid_map_->getMapSize();
        Eigen::Vector3d map_boundary_min = map_origin;
        Eigen::Vector3d map_boundary_max = map_origin + map_size;

        // 打印一下调试信息，确保地图卷动时这里的值是变的
        // std::cout << "🗺️ Map Bound: " << map_origin.transpose() << " -> " << map_boundary_max.transpose() << std::endl;

        // 取交集，确保不超出地图边界
        clear_min = clear_min.cwiseMax(map_boundary_min);
        clear_max = clear_max.cwiseMin(map_boundary_max);
        // 【修正结束】

        // 【核心操作】瞬间清空该区域
        // 【优化】注释掉频繁打印的调试信息
        // std::cout << "🧹 Clearing map: [" << clear_min.x() << ", " << clear_max.x() << "] x ["
        //           << clear_min.y() << ", " << clear_max.y() << "] x ["
        //           << clear_min.z() << ", " << clear_max.z() << "]" << std::endl;

        // 检查 SDFMap 是否有 resetBuffer 方法
        // 如果没有，请注释掉下面这行或者添加该方法
        grid_map_->resetBuffer(clear_min, clear_max);

        // ================= 3. 障碍物过滤与写入 (Insert) =================
        double resolution = grid_map_->getResolution();
        int filtered_count = 0;
        int added_count = 0;

        for(size_t i = 0; i < obstacle.size(); i++)
        {
            // 【关键过滤】只处理在感知视界内的障碍物！
            // 如果障碍物在清除范围之外，绝对不要加进去，否则它变成了"不死之身"
            double dist_x = std::abs(obstacle[i].x - current_drone_pos.x());
            double dist_y = std::abs(obstacle[i].y - current_drone_pos.y());
            double dist_z = std::abs(obstacle[i].z - current_drone_pos.z());

            if (dist_x > SENSING_HORIZON || dist_y > SENSING_HORIZON || dist_z > SENSING_HEIGHT_Z) {
                filtered_count++;
                continue; // 丢弃远处的点
            }

            added_count++;
            double half_x = obstacle[i].size_x / 2.0;
            double half_y = obstacle[i].size_y / 2.0;
            double half_z = obstacle[i].size_z / 2.0;

            // 根据新的分辨率调整生成的点数
            // 对于2m分辨率，每个维度生成的点数应该适当减少
            int points_x = std::max(2, (int)(obstacle[i].size_x / resolution) + 1);
            int points_y = std::max(2, (int)(obstacle[i].size_y / resolution) + 1);
            int points_z = std::max(2, (int)(obstacle[i].size_z / resolution) + 1);

            // 限制最大点数以避免计算爆炸
            points_x = std::min(points_x, 10);
            points_y = std::min(points_y, 10);
            points_z = std::min(points_z, 8);

            // 【优化】注释掉障碍物循环内的打印，避免大量障碍物时刷屏
            // ROS_DEBUG("✅ Adding obstacle at (%.1f, %.1f, %.1f), size: (%.1f, %.1f, %.1f), points: %dx%dx%d",
            //           obstacle[i].x, obstacle[i].y, obstacle[i].z,
            //           obstacle[i].size_x, obstacle[i].size_y, obstacle[i].size_z,
            //           points_x, points_y, points_z);

            // 生成长方体障碍物的点云
            for (int ix = 0; ix < points_x; ix++) {
                for (int iy = 0; iy < points_y; iy++) {
                    for (int iz = 0; iz < points_z; iz++) {
                        Eigen::Vector3d obstacle_pos;
                        obstacle_pos[0] = obstacle[i].x - half_x + ix * (obstacle[i].size_x / (points_x-1));
                        obstacle_pos[1] = obstacle[i].y - half_y + iy * (obstacle[i].size_y / (points_y-1));
                        obstacle_pos[2] = obstacle[i].z - half_z + iz * (obstacle[i].size_z / (points_z-1));

                        grid_map_->addLaserPoints(obstacle_pos, 1);
                    }
                }
            }
        }

        // 【优化】改为每2秒打印一次，避免刷屏 (使用静态变量实现节流)
        static std::chrono::high_resolution_clock::time_point last_obstacle_print = std::chrono::high_resolution_clock::now();
        current_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_obstacle_print).count() > 2000) {
            std::cout << "📊 Obstacles: " << obstacle.size() << " total, " << filtered_count << " filtered" << std::endl;
            last_obstacle_print = current_time;
        }

        // ================= 4. 更新 ESDF =================
        // 记录时间，监控性能
        auto start_time = std::chrono::high_resolution_clock::now();

        grid_map_->startUpdateMapInfo();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::cout << "⏱️ ESDF Update Time: " << elapsed << " ms" << std::endl;

        // 如果耗时过长，说明 SENSING_HORIZON 设太大了，需要调小
        if (elapsed > 20) {
            std::cout << "⚠️ Map update slow! Consider reducing horizon." << std::endl;
        }
    }

    void PlannerInterface::makePlan()
    {
    
        Eigen::Vector3d start_pt;
        Eigen::Vector3d start_vel;
        Eigen::Vector3d start_acc;
        Eigen::Vector3d local_target_pt;
        Eigen::Vector3d local_target_vel;
        vector<Eigen::Vector3d> point_set;


        vector<Eigen::Vector3d> traj_pts;

        for(size_t i = 0; i< _global_plan_traj_.size();i++)
        {
            Eigen::Vector3d plan_pt(_global_plan_traj_[i].x,_global_plan_traj_[i].y,_global_plan_traj_[i].z);
            point_set.push_back(plan_pt);
        }

        start_pt[0] = _global_plan_traj_[0].x;
        start_pt[1] = _global_plan_traj_[0].y;
        start_pt[2] = _global_plan_traj_[0].z;
        
        local_target_pt[0] = _global_plan_traj_[_global_plan_traj_.size()-1].x;
        local_target_pt[1] = _global_plan_traj_[_global_plan_traj_.size()-1].y;
        local_target_pt[2] = _global_plan_traj_[_global_plan_traj_.size()-1].z;

        start_vel[0] = 0;
        start_vel[1] = 0; 
        start_vel[2] = 0;

        local_target_vel[0] = 0;//根据实际需求修改接入
        local_target_vel[1] = 0;
        local_target_vel[2] = 0;

        start_acc[0] = 0;
        start_acc[1] = 0;
        start_acc[2] = 0;


        auto start = std::chrono::high_resolution_clock::now();

        bool plan_success = reboundReplan(start_pt,start_vel, start_acc,local_target_pt,local_target_vel, point_set);
        if (plan_success)
           getTraj();


        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // 修复：使用更精确的时间显示
        if (elapsed.count() < 1000) {
            printf("MotionPlanner Total Running Time: %ld microseconds \n", elapsed.count());
        } else {
            printf("MotionPlanner Total Running Time: %.3f ms \n", elapsed.count() / 1000.0);
        }

    }

    void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results)
    {
        plan_traj_results = _plan_traj_results_;
    }


    bool PlannerInterface::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set)
    {
        // 关键修复：确保路径点集合正确使用传入的起点位置
        if (!point_set.empty()) {
            // 强制修正第一个点为传入的起点位置
            point_set[0] = start_pt;
            // std::cout << "🔧 Forced path point start to match planner start: ("
            //           << start_pt.x() << ", " << start_pt.y() << ", " << start_pt.z() << ")" << std::endl;
        }

        // --- 添加调试打印 ---
        std::cout << "[DEBUG] reboundReplan start. Bspline Opt Ptr: "
                  << (bspline_optimizer_rebound_ ? "Valid" : "NULL") << std::endl;

        vector<Eigen::Vector3d> start_end_derivatives;
        // double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        // 【修复】平衡时间约束，在虚高上限下实现平滑轨迹
        // 由于 pp_.max_vel_ 现在是 6.0，分母变大了，系数用 1.3 是很安全的
        // 计算：Velocity_Capacity = 6.0 / 1.3 = 4.6 m/s
        // 目标是 3.0 m/s，所以 4.6 的容量绰绰有余，轨迹会非常平滑
        double dist = (start_pt - local_target_pt).norm();
        double ts = dist > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.3 : 1.0;

        start_end_derivatives.push_back(start_vel);         // 起点速度
        start_end_derivatives.push_back(local_target_vel);  //终点速度

        // 修复：计算合理的终点速度方向 - 使用显式赋值避免类型错误
        if (local_target_vel.norm() < 0.1) {
            // 如果终点速度很小，设置为从最后两个点计算的方向
            if (point_set.size() >= 2) {
                Eigen::Vector3d dir = (point_set.back() - point_set[point_set.size()-2]).normalized();
                double target_speed = std::min(pp_.max_vel_, start_vel.norm());
                start_end_derivatives[1] = dir * target_speed;
            } else {
                // 使用起点速度方向 - 修复类型错误
                if (start_vel.norm() > 0.1) {
                    Eigen::Vector3d normalized_vel = start_vel.normalized();
                    double speed = std::min(pp_.max_vel_, start_vel.norm());
                    start_end_derivatives[1] = normalized_vel * speed;
                } else {
                    start_end_derivatives[1] = Eigen::Vector3d::Zero();
                }
            }
        }

        start_end_derivatives.push_back(start_acc);     // 起点加速度
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());     // 终点加速度为0

        // std::cout << "🔧 Planner start state: pos=(" << start_pt.x() << ", " << start_pt.y() << ", " << start_pt.z()
        //           << "), vel=" << start_vel.norm() << " m/s, acc=" << start_acc.norm() << " m/s²" << std::endl;

        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

        std::cout << "[DEBUG] Initializing control points (A*)..." << std::endl;

        vector<vector<Eigen::Vector3d>> a_star_pathes;
        // 这一行通常是崩溃点，如果这里打印出来了，说明 A* 过了
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        std::cout << "[DEBUG] A* finished. Starting optimization..." << std::endl;

        // static int vis_id = 0;

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);

        std::cout << "[DEBUG] Optimization finished. Result: " << flag_step_1_success << std::endl;
        cout << "first_optimize_step_success=" << flag_step_1_success << endl;
        if (!flag_step_1_success)
        {
        continous_failures_count_++;
        return false;
        }

        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;
        if (!pos.checkFeasibility(ratio, false))
        {
            cout << "Need to reallocate time." << endl;

            Eigen::MatrixXd optimal_control_points;
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success)
                pos = UniformBspline(optimal_control_points, 3, ts);
        }

        if (!flag_step_2_success)
        {
            printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continous_failures_count_++;
            return false;
        }
    
        updateTrajInfo(pos);

        continous_failures_count_ = 0;

        return true;
    }

    bool PlannerInterface::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {
        double t_inc;

        Eigen::MatrixXd ctrl_pts; 

        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        traj = UniformBspline(ctrl_pts, 3, ts);

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);

        bspline_optimizer_rebound_->ref_pts_.clear();

        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

        return success;
    }

    void PlannerInterface::updateTrajInfo(const UniformBspline &position_traj)
    {
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void PlannerInterface::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                            Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }

        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

    void PlannerInterface::getTraj()
{
    auto info = &local_data_;

    obj_planner::Bspline bspline;
    bspline.order = 3;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());

    for (int i = 0; i < pos_pts.cols(); ++i)
    {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());

    for (int i = 0; i < knots.rows(); ++i)
    {
        bspline.knots.push_back(knots(i));
    }

    vector<obj_planner::UniformBspline> traj_;
    double traj_duration_;

    obj_planner::UniformBspline pos_traj(pos_pts, bspline.order, 0.1);
    pos_traj.setKnot(knots);
    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());  // 速度轨迹
    traj_.push_back(traj_[1].getDerivative());  // 加速度轨迹
    traj_duration_ = traj_[0].getTimeSum();

    Eigen::Vector3d pos, vel, acc;
    _plan_traj_results_.clear();

    // std::cout << "🎯 Generating trajectory with " << traj_duration_ << "s duration" << std::endl;

    // 确保生成足够多的点用于可视化
    // double time_step = 0.1; // 100ms间隔
    // 【修改后】提高分辨率到 0.01s (100Hz)，高于控制频率 (50Hz)
    // 这样 ControlThread 在取点时，几乎不需要大幅度插值
    double time_step = 0.01;
    int min_points = 50;    // 最少50个点
    int max_points = 2000;  // 【修改】由于密度增加，最大点数限制要放大，防止轨迹被截断
    
    int num_points = std::min(max_points, std::max(min_points, (int)(traj_duration_ / time_step)));
    
    // 调整时间步长以确保足够的点数
    if (num_points < min_points && traj_duration_ > 0) {
        time_step = traj_duration_ / min_points;
        num_points = min_points;
    }

    // 用于平滑姿态变化
    double prev_yaw = 0.0;
    double prev_yaw_rate = 0.0; // [新增] 保存上一时刻的偏航角速度
    // 获取轨迹起点的速度向量
    Eigen::Vector3d start_vel_vec = traj_[1].evaluateDeBoorT(0.0);
    if (start_vel_vec.norm() > 0.1) {
        prev_yaw = atan2(start_vel_vec(1), start_vel_vec(0));
    }

    double prev_pitch = 0.0;
    
    // 【修改】循环步长也必须对应修改
    for (double t_cur = 0; t_cur <= traj_duration_; t_cur += time_step)
    {
        pos = traj_[0].evaluateDeBoorT(t_cur);
        vel = traj_[1].evaluateDeBoorT(t_cur);
        acc = traj_[2].evaluateDeBoorT(t_cur);
        
        PathPoint tempPath;
        // 位置信息
        tempPath.x = pos(0);
        tempPath.y = pos(1);
        tempPath.z = pos(2);
        
        // 速度信息
        tempPath.vx = vel(0);
        tempPath.vy = vel(1);
        tempPath.vz = vel(2);
        tempPath.speed = vel.norm();
        
        // 加速度信息
        tempPath.ax = acc(0);
        tempPath.ay = acc(1);
        tempPath.az = acc(2);
        
        // 改进的姿态计算
        double current_yaw, current_pitch;
        
        // 计算偏航角 (yaw)
        if (vel.norm() > 0.1) {
            current_yaw = atan2(vel(1), vel(0));
            
            // 平滑处理
            double yaw_diff = current_yaw - prev_yaw;
            while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
            
            double max_yaw_rate = 1.0; // 弧度/秒
            if (std::abs(yaw_diff) > max_yaw_rate * time_step) {
                current_yaw = prev_yaw + std::copysign(max_yaw_rate * time_step, yaw_diff);
            }
        } else {
            current_yaw = prev_yaw;
        }
        
        tempPath.yaw = current_yaw;
        // prev_yaw将在后面统一更新

        // 计算俯仰角 (pitch)
        double horizontal_speed = sqrt(vel(0)*vel(0) + vel(1)*vel(1));
        if (horizontal_speed > 0.1 && vel.norm() > 0.1) {
            current_pitch = atan2(vel(2), horizontal_speed);
            
            double pitch_diff = current_pitch - prev_pitch;
            double max_pitch_rate = 0.5; // 弧度/秒
            if (std::abs(pitch_diff) > max_pitch_rate * time_step) {
                current_pitch = prev_pitch + std::copysign(max_pitch_rate * time_step, pitch_diff);
            }
        } else {
            current_pitch = prev_pitch;
        }
        
        tempPath.pitch = current_pitch;
        prev_pitch = current_pitch;

        // [新增] 计算偏航角速度（前馈控制用）
        double yaw_rate = 0.0;
        if (vel.norm() > 0.1) {
            // 计算当前时刻的偏航角速度
            double yaw_diff_raw = current_yaw - prev_yaw;
            // 角度归一化，处理跳变
            while (yaw_diff_raw > M_PI) yaw_diff_raw -= 2 * M_PI;
            while (yaw_diff_raw < -M_PI) yaw_diff_raw += 2 * M_PI;

            yaw_rate = yaw_diff_raw / time_step; // 简单差分法

            // 平滑偏航角速度，避免突变
            double alpha = 0.3; // 滤波系数
            yaw_rate = alpha * yaw_rate + (1 - alpha) * prev_yaw_rate;

            // 限制最大偏航角速度
            double max_yaw_rate = 2.0; // 弧度/秒
            if (yaw_rate > max_yaw_rate) yaw_rate = max_yaw_rate;
            if (yaw_rate < -max_yaw_rate) yaw_rate = -max_yaw_rate;
        }

        tempPath.yaw_rate = yaw_rate;

        // [新增] 添加时间戳
        tempPath.time_from_start = t_cur;

        _plan_traj_results_.push_back(tempPath);

        // [更新] 保存当前状态为下一时刻的"前一时刻"状态
        prev_yaw = current_yaw;
        prev_yaw_rate = yaw_rate;
    }

    // std::cout << "✅ Generated " << _plan_traj_results_.size() << " trajectory points with time step " << time_step << "s" << std::endl;
    
    // 打印一些点的姿态信息用于调试
    // if (_plan_traj_results_.size() > 5) {
    //     std::cout << "Sample attitude values:" << std::endl;
    //     for (int i = 0; i < std::min(5, (int)_plan_traj_results_.size()); i++) {
    //         const auto& point = _plan_traj_results_[i];
    //         std::cout << "  Point " << i << ": yaw=" << point.yaw * 180/M_PI 
    //                   << "°, pitch=" << point.pitch * 180/M_PI << "°" << std::endl;
    //     }
    // }
}


    

    // 转换函数：将Object3d转换为内部的ObstacleInfo
    ObstacleInfo PlannerInterface::convertObject3dToObstacleInfo(const Object3d& obj)
    {
        ObstacleInfo obstacle;
        obstacle.x = obj.rect3d.center.x;
        obstacle.y = obj.rect3d.center.y;
        obstacle.z = obj.rect3d.center.z;
        obstacle.size_x = obj.rect3d.size.x;
        obstacle.size_y = obj.rect3d.size.y;
        obstacle.size_z = obj.rect3d.size.z;
        obstacle.type = 0; // 长方体
        
        return obstacle;
    }

    // 新的障碍物设置接口
    void PlannerInterface::setObstacles(const std::vector<Object3d>& objects, Eigen::Vector3d current_drone_pos)
    {
        std::vector<ObstacleInfo> internal_obstacles;

        for (const auto& obj : objects) {
            // 只处理存在概率较高的障碍物（可选）
            if (obj.existProb > 0.5) {
                ObstacleInfo obstacle = convertObject3dToObstacleInfo(obj);
                internal_obstacles.push_back(obstacle);
            }
        }

        // 调用修改后的障碍物设置函数，传入无人机位置
        setObstacles(internal_obstacles, current_drone_pos);
    }

    LargeScalePlanner::LargeScalePlanner() 
        : segment_length_(500.0), current_segment_(0) 
    {
        local_planner_ = std::make_shared<PlannerInterface>();
        // 初始化局部规划器参数
        local_planner_->initParam(15.0, 3.0, 1.5);
    }

    void LargeScalePlanner::setGlobalPath(const std::vector<PathPoint>& global_path) {
        global_path_ = global_path;
        current_segment_ = 0;
    }

    bool LargeScalePlanner::planNextSegment() {
        if (current_segment_ >= static_cast<int>(global_path_.size()) - 1) {
            return false;
        }
        
        // 计算当前段的起点和终点
        int start_idx = current_segment_;
        int end_idx = std::min(current_segment_ + 10, (int)global_path_.size() - 1); // 每次规划10个点
        
        std::vector<PathPoint> segment_path(global_path_.begin() + start_idx, 
                                        global_path_.begin() + end_idx + 1);
        
        // 设置局部规划器的路径
        local_planner_->setPathPoint(segment_path);
        
        // 执行规划
        local_planner_->makePlan();
        
        current_segment_ = end_idx;
        return true;
    }

    std::vector<PathPoint> LargeScalePlanner::getCurrentTrajectory() {
        std::vector<PathPoint> trajectory;
        local_planner_->getLocalPlanTrajResults(trajectory);
        return trajectory;
    }

    bool LargeScalePlanner::isFinished() const {
        return current_segment_ >= static_cast<int>(global_path_.size()) - 1;
    }

    bool PlannerInterface::makePlanWithState(const Eigen::Vector3d& start_pt, 
                                        const Eigen::Vector3d& start_vel,
                                        const Eigen::Vector3d& start_acc,
                                        const Eigen::Vector3d& target_pt,
                                        const Eigen::Vector3d& target_vel,
                                        const Eigen::Vector3d& target_acc)
    {
        // 1. 【新增】强制清空旧轨迹结果，防止规划失败时上层读取到脏数据
        _plan_traj_results_.clear();

        // 设置起点和终点状态
        Eigen::Vector3d start_vel_used = start_vel;
        Eigen::Vector3d target_vel_used = target_vel;
        
        // 速度限制
        if (start_vel_used.norm() > pp_.max_vel_) {
            start_vel_used = start_vel_used.normalized() * pp_.max_vel_;
        }
        if (target_vel_used.norm() > pp_.max_vel_) {
            target_vel_used = target_vel_used.normalized() * pp_.max_vel_;
        }
        
        vector<Eigen::Vector3d> point_set;

        for(size_t i = 0; i < _global_plan_traj_.size(); i++)
        {
            Eigen::Vector3d plan_pt(_global_plan_traj_[i].x, _global_plan_traj_[i].y, _global_plan_traj_[i].z);
            point_set.push_back(plan_pt);
        }

        // 使用传入的状态进行规划
        bool plan_success = reboundReplan(start_pt, start_vel_used, start_acc, target_pt, target_vel_used, point_set);
        
        if (plan_success) {
            getTraj();
            return true; // 【新增】返回成功
        }
        
        return false; // 【新增】返回失败
    }


}