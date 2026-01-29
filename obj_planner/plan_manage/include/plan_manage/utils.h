#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "planner_interface.h"
#include "obstacle_types.h"

std::vector<obj_planner::Object3d> createRealTimeObstacles();
std::vector<obj_planner::PathPoint> createRealTimePath();

// 新增：支持自定义起点的路径生成函数
std::vector<obj_planner::PathPoint> createPathFromStart(const Eigen::Vector3d& start_pos,
                                                      const Eigen::Vector3d& end_pos,
                                                      int num_points = 200);

// 新增：创建以起点为基准的相对路径（实机模式推荐）
std::vector<obj_planner::PathPoint> createRelativePath(const Eigen::Vector3d& start_pos,
                                                       double distance_x, double distance_y, double distance_z,
                                                       int num_points = 200);

#endif