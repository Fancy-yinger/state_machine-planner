#include "utils.h"
#include <cmath>
#include <iostream>

/*
 * @Function:Obj Planner:Utils
 * @Author:Fancy
 * @Date:2026 01
 */

std::vector<obj_planner::Object3d> createRealTimeObstacles() {
    std::vector<obj_planner::Object3d> obstacles;
    std::vector<std::vector<double>> obstacle_positions = {
        {200.0, 200.0, 10.0, 1.0, 1.0, 10.0},
        {600.0, 600.0, 10.0, 5.0, 5.0, 15.0},
        {1000.0, 1000.0, 10.0, 8.0, 8.0, 10.0},
        {1400.0, 1400.0, 10.0, 9.0, 9.0, 10.0},
    };
    
    for (size_t i = 0; i < obstacle_positions.size(); ++i) {
        obj_planner::Object3d obj;
        obj.rect3d.center = {obstacle_positions[i][0], obstacle_positions[i][1], obstacle_positions[i][2]};
        obj.rect3d.size = {obstacle_positions[i][3], obstacle_positions[i][4], obstacle_positions[i][5]};
        obj.existProb = 1.0;
        obj.id = i + 1;
        obstacles.push_back(obj);
    }
    std::cout << "Created " << obstacles.size() << " obstacles." << std::endl;
    return obstacles;
}

std::vector<obj_planner::PathPoint> createRealTimePath() {
    std::vector<obj_planner::PathPoint> path;
    double start_x = 50.0; double start_y = 50.0; double start_z = 10.0;
    double end_x = 1500.0; double end_y = 1500.0; double end_z = 10.0;
    int num_points = 1000;

    for (int i = 0; i <= num_points; i++) {
        obj_planner::PathPoint point;
        double t = (double)i / num_points;
        point.x = start_x + (end_x - start_x) * t;
        point.y = start_y + (end_y - start_y) * t;
        point.z = start_z + (end_z - start_z) * t;
        path.push_back(point);
    }
    return path;
}

// 新增：支持自定义起点的路径生成函数（避免与原函数冲突）
std::vector<obj_planner::PathPoint> createPathFromStart(const Eigen::Vector3d& start_pos,
                                                      const Eigen::Vector3d& end_pos,
                                                      int num_points) {
    std::vector<obj_planner::PathPoint> path;
    std::cout << "🎯 Creating path from " << start_pos.transpose()
              << " to " << end_pos.transpose() << std::endl;

    for (int i = 0; i <= num_points; i++) {
        obj_planner::PathPoint point;
        double t = (double)i / num_points;
        point.x = start_pos.x() + (end_pos.x() - start_pos.x()) * t;
        point.y = start_pos.y() + (end_pos.y() - start_pos.y()) * t;
        point.z = start_pos.z() + (end_pos.z() - start_pos.z()) * t;
        point.vx = point.vy = point.vz = 0;  // 默认速度为0
        point.ax = point.ay = point.az = 0;  // 默认加速度为0
        point.yaw = 0.0;                  // 默认偏航角为0
        path.push_back(point);
    }
    return path;
}

// 新增：创建以起点为基准的相对路径（实机模式推荐）
std::vector<obj_planner::PathPoint> createRelativePath(const Eigen::Vector3d& start_pos,
                                                       double distance_x, double distance_y, double distance_z,
                                                       int num_points) {
    Eigen::Vector3d end_pos = start_pos + Eigen::Vector3d(distance_x, distance_y, distance_z);
    return createPathFromStart(start_pos, end_pos, num_points);
}