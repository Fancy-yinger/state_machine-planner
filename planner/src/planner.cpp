/**
 * @file planner.cpp
 *
 * @date 03 June 2022
 * 
 * @brief Trajectory Planner using mav_trajectory_generation
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */

#include <planner/planner.h>

#include <queue>
#include <unordered_set>
#include <functional>
#include <cmath>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry> // 用于四元数计算
#include <boost/functional/hash.hpp> // 用于更好的哈希组合
#include <unordered_map>
#include <set>
#include <xmlrpcpp/XmlRpcValue.h> // 用于从参数服务器读取障碍物配置


// TODO add service to receive the global coordinates of the goal position

//BasicPlanner类的构造函数，初始化了轨迹规划器的基本参数和状态
BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()),
        goal_received_(false)
{
    ROS_WARN("[INIT]: Starting Trajectory Planner ...");

    // Initialize parameters
    state_ = 0;
    dimension_ = 4;
    planner_done_ = false;
    derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;

    goal_velocity_ << 0, 0, 0, 0;
    goal_acceleration_ << 0, 0, 0, 0;
    // valid_states_ = {0, 1, 2, 3};
    //valid_states_ = {0, 1};
    valid_states_ = {0, 1, 2, 3, 4}; // 添加4对应ATRAJECTORY状态


    // 初始化A*相关
    // obstacle_map_ = new ObstacleMap(200, 200, 100, 0.2); // 40x40x20米，分辨率0.2米
    obstacle_map_ = new ObstacleMap(200, 200, 100, 0.4);

    // --- 新增代码开始：从参数服务器读取障碍物 ---
    ROS_INFO("[INIT]: Loading obstacles from parameter server...");
    XmlRpc::XmlRpcValue obstacle_list;

    // 1. 获取当前节点的名称 (例如 "/planner")
    std::string node_name = ros::this_node::getName();

    // 2. 拼接完整的参数路径 (例如 "/planner/obstacles")
    std::string param_path = node_name + "/obstacles";

    // 3. 尝试读取障碍物列表
    // 先试私有路径 /planner/obstacles，如果失败再试全局路径 /obstacles
    if (nh_.getParam(param_path, obstacle_list) || nh_.getParam("/obstacles", obstacle_list))
    {
        ROS_INFO("[INIT]: Successfully read obstacles from: %s", param_path.c_str());

        // 辅助 lambda：安全地从 XmlRpcValue 获取 double
        auto getDouble = [](XmlRpc::XmlRpcValue& v) -> double {
            if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
            if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<double>(static_cast<int>(v));
            return 0.0;
        };

        for (int i = 0; i < obstacle_list.size(); ++i)
        {
            XmlRpc::XmlRpcValue obstacle = obstacle_list[i];
            if (obstacle.hasMember("type"))
            {
                std::string type = std::string(obstacle["type"]);
                if (type == "box") {
                     double x = getDouble(obstacle["x"]);
                     double y = getDouble(obstacle["y"]);
                     double z = getDouble(obstacle["z"]);
                     double w = getDouble(obstacle["width"]);
                     double l = getDouble(obstacle["length"]);
                     double h = getDouble(obstacle["height"]);
                     obstacle_map_->addBoxObstacle(x, y, z, w, l, h);
                     ROS_INFO("Loaded Box Obstacle: x=%.1f, y=%.1f, z=%.1f, w=%.1f, l=%.1f, h=%.1f", x, y, z, w, l, h);
                } else if (type == "sphere") {
                     double x = getDouble(obstacle["x"]);
                     double y = getDouble(obstacle["y"]);
                     double z = getDouble(obstacle["z"]);
                     double r = getDouble(obstacle["radius"]);
                     obstacle_map_->addObstacle(x, y, z, r); // planner.cpp 中 addObstacle 是添加球体
                     ROS_INFO("Loaded Sphere Obstacle: x=%.1f, y=%.1f, z=%.1f, r=%.1f", x, y, z, r);
                }
            }
        }
        ROS_INFO("[INIT]: Successfully loaded %d obstacles from parameter server", obstacle_list.size());
    }
    else
    {
        ROS_WARN("[INIT]: No obstacles found in parameter server (tried %s and /obstacles)!", param_path.c_str());
    }
    // --- 新增代码结束 ---

    readParameters();
    initializePublishers();
    initializeSubscribers();
    initializeServices();

    ROS_WARN("[INIT]: Trajectory Planner is ready!");

    // 初始化障碍物可视化发布器
    obstacle_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("planner/obstacle_markers", 1, true);

    // 初始发布一次障碍物
    publishObstacleMarkers();

}


void BasicPlanner::publishObstacleMarkers() {
    visualization_msgs::MarkerArray markers;
    int id = 0;
    int obstacle_count = 0;
    
    // 获取障碍物地图
    BasicPlanner::ObstacleMap* map = this->obstacle_map_;
    
    // 首先添加DELETEALL消息清除之前的标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "obstacles";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    id++;
    
    // 发布长方体障碍物（只显示z轴以上的部分）
    for (const auto& box : map->getBoxObstacles()) {
        // 计算障碍物的最低点和最高点
        double box_bottom_z = box.z - box.height / 2.0;
        double box_top_z = box.z + box.height / 2.0;
        
        // 如果障碍物完全在z轴以下，则跳过不显示
        if (box_top_z <= 0) {
            continue;
        }
        
        obstacle_count++;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = box.x;
        marker.pose.position.y = box.y;
        marker.pose.orientation.w = 1.0;
        
        // 设置颜色（红色半透明）
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;
        
        // 设置生命周期
        marker.lifetime = ros::Duration();
        
        // 如果障碍物完全在z轴以上，直接显示整个障碍物
        // if (box_bottom_z >= 0) {
        //     marker.pose.position.z = box.z;
        //     marker.scale.x = box.width;
        //     marker.scale.y = box.length;
        //     marker.scale.z = box.height;
        // } else {
        //     // 如果障碍物部分在z轴以下，只显示z轴以上的部分
        //     double visible_height = box_top_z;  // 只显示从z=0到box_top_z的部分
        //     marker.pose.position.z = visible_height / 2.0;  // 位置在可见部分的中心
        //     marker.scale.x = box.width;
        //     marker.scale.y = box.length;
        //     marker.scale.z = visible_height;
        // }
        
        marker.pose.position.z = box.z;
        marker.scale.x = box.width;
        marker.scale.y = box.length;
        marker.scale.z = box.height;

        markers.markers.push_back(marker);
    }
    
    // 发布球形障碍物（只显示z轴以上的部分）
    for (const auto& sphere : map->getSphereObstacles()) {
        // 计算球体的最低点
        double sphere_bottom_z = sphere.z - sphere.radius;
        double sphere_top_z = sphere.z + sphere.radius;
        
        // 如果球体完全在z轴以下，则跳过不显示
        if (sphere_top_z <= 0) {
            continue;
        }
        
        obstacle_count++;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = id++;
        marker.pose.position.x = sphere.x;
        marker.pose.position.y = sphere.y;
        marker.pose.orientation.w = 1.0;
        
        // 设置颜色（蓝色半透明）
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.7f;
        
        // 设置生命周期
        marker.lifetime = ros::Duration();
        
        // 如果球体完全在z轴以上，直接显示整个球体
        // if (sphere_bottom_z >= 0) {
        //     marker.type = visualization_msgs::Marker::SPHERE;
        //     marker.pose.position.z = sphere.z;
        //     marker.scale.x = sphere.radius * 2;  // 直径
        //     marker.scale.y = sphere.radius * 2;
        //     marker.scale.z = sphere.radius * 2;
        // } else {
        //     // 如果球体部分在z轴以下，使用圆柱体模拟半球来只显示z轴以上的部分
        //     marker.type = visualization_msgs::Marker::CYLINDER;
            
        //     // 计算可见部分的高度
        //     double visible_height = sphere_top_z;
            
        //     // 设置圆柱体位置和大小
        //     marker.pose.position.z = visible_height / 2.0;  // 位置在可见部分的中心
        //     marker.scale.x = sphere.radius * 2;  // 直径
        //     marker.scale.y = sphere.radius * 2;
        //     marker.scale.z = visible_height;     // 高度为可见部分
        // }

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.z = sphere.z;
        marker.scale.x = sphere.radius * 2;  // 直径
        marker.scale.y = sphere.radius * 2;
        marker.scale.z = sphere.radius * 2;

        
        markers.markers.push_back(marker);
    }
    
    // 发布标记数组
    obstacle_markers_pub_.publish(markers);
    
    ROS_INFO_STREAM("[PLANNER] Published " << obstacle_count << " obstacles as " << (id-1) << " markers");
}

// 实现障碍物地图
BasicPlanner::ObstacleMap::ObstacleMap(int sx, int sy, int sz, double res) : 
    size_x_(sx), size_y_(sy), size_z_(sz), resolution_(res) 
{
    grid_.resize(size_x_, std::vector<std::vector<bool>>(size_y_, std::vector<bool>(size_z_, false)));
    
    // 添加示例障碍物
    // 使用长方体障碍物替代球形障碍物
    // addBoxObstacle(2, 2, 2, 0.5, 0.5, 2.0);  // 宽度0.5米，长度0.5米，高度0.5米的立方体
    // addBoxObstacle(3, 2, 1, 1.0, 0.3, 3.0);  // 可以添加更多不同大小的长方体障碍物
    // addBoxObstacle(3, 4, 1, 1.0, 0.5, 5.0); 
    // addBoxObstacle(6, 3, 1, 1.0, 0.3, 4.0); 
    // addBoxObstacle(5, 5, 1, 1.0, 0.3, 6.0);
}

void BasicPlanner::ObstacleMap::addObstacle(double x, double y, double z, double radius) {
    // 存储原始障碍物信息
    sphere_obstacles_.push_back({x, y, z, radius});
    
    // 使用与A*算法相同的坐标转换方式
    int grid_x = static_cast<int>(x / resolution_);
    int grid_y = static_cast<int>(y / resolution_);
    int grid_z = static_cast<int>(z / resolution_);
    int radius_cells = static_cast<int>(radius / resolution_);
    
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
        for (int dy = -radius_cells; dy <= radius_cells; dy++) {
            for (int dz = -radius_cells; dz <= radius_cells; dz++) {
                int nx = grid_x + dx;
                int ny = grid_y + dy;
                int nz = grid_z + dz;
                if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_ && nz >= 0 && nz < size_z_) {
                    if (dx*dx + dy*dy + dz*dz <= radius_cells*radius_cells) {
                        grid_[nx][ny][nz] = true;
                    }
                }
            }
        }
    }
}

void BasicPlanner::ObstacleMap::addBoxObstacle(double x, double y, double z, double width, double length, double height) {
    // 存储原始障碍物信息
    box_obstacles_.push_back({x, y, z, width, length, height});
    
    // 使用与A*算法相同的坐标转换方式
    int grid_x = static_cast<int>(x / resolution_);
    int grid_y = static_cast<int>(y / resolution_);
    int grid_z = static_cast<int>(z / resolution_);
    
    // 将宽度、长度和高度转换为网格单元数
    // int width_cells = static_cast<int>(width / resolution_ / 2);  // 半宽度
    // int length_cells = static_cast<int>(length / resolution_ / 2); // 半长度
    // int height_cells = static_cast<int>(height / resolution_ / 2); // 半高度
    // 修复：使用ceil向上取整，并确保至少为1个单元
    int width_cells = std::max(1, static_cast<int>(std::ceil(width / resolution_ / 2)));
    int length_cells = std::max(1, static_cast<int>(std::ceil(length / resolution_ / 2)));
    int height_cells = std::max(1, static_cast<int>(std::ceil(height / resolution_ / 2)));
    
    
    // 填充长方体区域内的所有网格点
    for (int dx = -width_cells; dx <= width_cells; dx++) {
        for (int dy = -length_cells; dy <= length_cells; dy++) {
            for (int dz = -height_cells; dz <= height_cells; dz++) {
                int nx = grid_x + dx;
                int ny = grid_y + dy;
                int nz = grid_z + dz;
                if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_ && nz >= 0 && nz < size_z_) {
                    grid_[nx][ny][nz] = true;
                }
            }
        }
    }
}

bool BasicPlanner::ObstacleMap::isOccupied(int x, int y, int z) const {
    // 安全距离膨胀（半径3个网格）
    const int safe_radius = 1;
    
    for (int dx = -safe_radius; dx <= safe_radius; ++dx) {
        for (int dy = -safe_radius; dy <= safe_radius; ++dy) {
            for (int dz = -safe_radius; dz <= safe_radius; ++dz) {
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;
                
                // 边界检查
                if (nx >= 0 && nx < size_x_ && 
                    ny >= 0 && ny < size_y_ &&
                    nz >= 0 && nz < size_z_) {
                    
                    if (grid_[nx][ny][nz]) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool BasicPlanner::ObstacleMap::isValid(int x, int y, int z) const {
    return x >= 0 && x < size_x_ && y >= 0 && y < size_y_ && z >= 0 && z < size_z_;
}

// A* 路径规划实现
std::vector<Eigen::Vector4d> BasicPlanner::planAStarPath(const Eigen::Vector4d& start, const Eigen::Vector4d& goal)
{
    // 获取障碍物地图
    ObstacleMap* map = this->obstacle_map_;
    double resolution = map->getResolution();

    // 世界坐标转网格坐标
    auto worldToGrid = [map](double x, double y, double z) -> std::tuple<int, int, int> {
        int grid_x = static_cast<int>(x / map->getResolution());
        int grid_y = static_cast<int>(y / map->getResolution());
        int grid_z = static_cast<int>(z / map->getResolution());
        return std::make_tuple(grid_x, grid_y, grid_z);
    };
    
    // 获取起点和终点的网格坐标
    std::tuple<int, int, int> start_grid = worldToGrid(start(0), start(1), start(2));
    int start_x = std::get<0>(start_grid);
    int start_y = std::get<1>(start_grid);
    int start_z = std::get<2>(start_grid);
    
    std::tuple<int, int, int> goal_grid = worldToGrid(goal(0), goal(1), goal(2));
    int goal_x = std::get<0>(goal_grid);
    int goal_y = std::get<1>(goal_grid);
    int goal_z = std::get<2>(goal_grid);

    // 保存起点网格坐标用于yaw计算
    const int start_gx = start_x;
    const int start_gy = start_y;
    const int start_gz = start_z;
    
    // 网格坐标转世界坐标（包含yaw计算）
    auto gridToWorld = [map, start_gx, start_gy](int gx, int gy, int gz) -> Eigen::Vector4d {
        Eigen::Vector4d world;
        world(0) = gx * map->getResolution();
        world(1) = gy * map->getResolution();
        world(2) = gz * map->getResolution();
        
        // 计算朝向起点到当前点的方向
        double dx = gx - start_gx;
        double dy = gy - start_gy;
        double yaw = (dx == 0 && dy == 0) ? 0 : atan2(dy, dx);
        world(3) = yaw;
        
        return world;
    };

    ROS_INFO("[PLANNER] World Start: (%.2f, %.2f, %.2f) -> Grid Start: (%d, %d, %d)", 
             start(0), start(1), start(2), start_x, start_y, start_z);
    ROS_INFO("[PLANNER] World Goal: (%.2f, %.2f, %.2f) -> Grid Goal: (%d, %d, %d)", 
             goal(0), goal(1), goal(2), goal_x, goal_y, goal_z);

    
    // 检查起点和终点是否有效
    if (obstacle_map_->isOccupied(start_x, start_y, start_z)) {
        ROS_WARN("[PLANNER] Start position is occupied!");
        return {};
    }
    
    if (obstacle_map_->isOccupied(goal_x, goal_y, goal_z)) {
        ROS_WARN("[PLANNER] Goal position is occupied!");
        return {};
    }

    // 定义方向向量（8方向简化版）
    // const std::vector<std::tuple<int, int, int>> directions = {
    //     {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
    //     {1,1,0}, {-1,1,0}
    // };
    // 修改后（增加更多对角线方向，包括3D对角线）
    const std::vector<std::tuple<int, int, int>> directions = {
        // 6个轴向
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
        // 12个面对角线
        {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
        {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
        {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
        // 4个体对角线（可选，根据需求添加）
        {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
        {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
    };

    // 成本函数（曼哈顿距离）
    // auto heuristic = [](int x1, int y1, int z1, int x2, int y2, int z2) {
    //     return std::abs(x1 - x2) + std::abs(y1 - y2) + std::abs(z1 - z2);
    // };
    // 优化建议：
    auto heuristic = [](int x1, int y1, int z1, int x2, int y2, int z2) {
        // 欧几里得距离，Z轴权重降低（假设Z方向运动限制较小）
        double dx = x1 - x2;
        double dy = y1 - y2;
        double dz = (z1 - z2) * 0.5; // Z轴权重降低
        return sqrt(dx*dx + dy*dy + dz*dz);
    };

    // 优先队列（最小堆）
    // auto cmp = [](AStarNode* a, AStarNode* b) { return a->f_cost > b->f_cost; };
    // std::priority_queue<AStarNode*, std::vector<AStarNode*>, decltype(cmp)> openSet(cmp);

    // 优化建议（支持节点更新）：
    // 1. 定义比较函数（处理f_cost相等的情况）
    struct CompareNode {
        bool operator()(const AStarNode* a, const AStarNode* b) const {
            if (a->f_cost != b->f_cost) {
                return a->f_cost < b->f_cost;
            }
            // 当f_cost相等时，使用g_cost作为第二排序标准
            return a->g_cost < b->g_cost;
        }
    };
    std::set<AStarNode*, CompareNode> openSet;
    std::unordered_map<int, AStarNode*> nodeMap; // 使用网格坐标哈希作为key
    
    // 关闭集合（使用指针集合）
    std::unordered_set<AStarNode*> closedSet;
    
    // 访问标记数组（避免重复访问）
    std::vector<std::vector<std::vector<bool>>> visited(
        map->getSizeX(), 
        std::vector<std::vector<bool>>(
            map->getSizeY(), 
            std::vector<bool>(map->getSizeZ(), false)
        )
    );
    visited[start_x][start_y][start_z] = true;
    
    // 创建起始节点
    AStarNode* startNode = new AStarNode(start_x, start_y, start_z, 
                                      0.0, 
                                      heuristic(start_x, start_y, start_z, goal_x, goal_y, goal_z));
    // openSet.push(startNode);
    openSet.insert(startNode);
    
    std::vector<AStarNode*> allNodes; // 跟踪所有创建的节点
    allNodes.push_back(startNode);
    
    AStarNode* goalNode = nullptr;
    bool pathFound = false;
    int maxIterations = 50000;
    int iterations = 0;
    
    ros::Time startTime = ros::Time::now();

    // A*主循环
    while (!openSet.empty() && iterations < maxIterations) {
        iterations++;
        
        // AStarNode* currentNode = openSet.top();
        // openSet.pop();
        // 获取最小f_cost的节点
        AStarNode* currentNode = *openSet.begin();
        openSet.erase(openSet.begin());  // 删除该节点
        
        // 如果节点已经在关闭列表中，跳过
        if (closedSet.find(currentNode) != closedSet.end()) {
            continue;
        }
        
        // 添加到关闭列表
        closedSet.insert(currentNode);
        
        // 检查是否到达目标
        if (currentNode->x == goal_x && 
            currentNode->y == goal_y && 
            currentNode->z == goal_z) {
            goalNode = currentNode;
            pathFound = true;
            break;
        }
        
        // 探索所有可能的移动方向
        for (const auto& dir : directions) {
            int dx = std::get<0>(dir);
            int dy = std::get<1>(dir);
            int dz = std::get<2>(dir);
            
            int nx = currentNode->x + dx;
            int ny = currentNode->y + dy;
            int nz = currentNode->z + dz;
            
            // 检查新位置是否有效
            if (!map->isValid(nx, ny, nz) || map->isOccupied(nx, ny, nz)) {
                continue;
            }
            
            // 检查是否已访问
            if (visited[nx][ny][nz]) {
                continue;
            }
            
            visited[nx][ny][nz] = true;
            
            // 计算新位置的g成本
            double stepCost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
            
            double new_g = currentNode->g_cost + stepCost;
            
            // 计算新位置的f成本（g + h）
            double new_f = new_g + heuristic(nx, ny, nz, goal_x, goal_y, goal_z);
            
            // 创建新节点
            AStarNode* neighbor = new AStarNode(nx, ny, nz, new_g, new_f, currentNode);
            // openSet.push(neighbor);
            openSet.insert(neighbor);
            allNodes.push_back(neighbor);
        }
    }
    
    // 结果路径
    std::vector<Eigen::Vector4d> path;
    
    if (pathFound) {
        ROS_INFO("[PLANNER] A* Path found in %d iterations!", iterations);
        
        // 回溯重建路径
        std::vector<AStarNode*> tempPath;
        AStarNode* current = goalNode;
        while (current != nullptr) {
            tempPath.push_back(current);
            current = current->parent;
        }
        
        // 反转路径
        std::reverse(tempPath.begin(), tempPath.end());
        
        // 转换为世界坐标
        for (AStarNode* node : tempPath) {
            path.push_back(gridToWorld(node->x, node->y, node->z));
        }
    } else {
        ROS_WARN("[PLANNER] A* Path not found after %d iterations! Using fallback path", iterations);
        
        // 回退策略：生成直线路径
        ROS_WARN("[PLANNER] Generating straight line fallback path");
        
        const int steps = 10;
        for (int i = 0; i <= steps; i++) {
            double ratio = static_cast<double>(i) / steps;
            double wx = start(0) + ratio * (goal(0) - start(0));
            double wy = start(1) + ratio * (goal(1) - start(1));
            double wz = start(2) + ratio * (goal(2) - start(2));
            
            // 计算朝向
            double dx = goal(0) - start(0);
            double dy = goal(1) - start(1);
            double yaw = (dx == 0 && dy == 0) ? 0 : atan2(dy, dx);
            
            path.push_back(Eigen::Vector4d(wx, wy, wz, yaw));
        }
    }
    
    // 性能统计
    ros::Duration duration = ros::Time::now() - startTime;
    ROS_INFO("[PLANNER] A* planning took %.3f seconds", duration.toSec());
    
    // 清理内存
    for (auto node : allNodes) {
        delete node;
    }
    
    // 发布A*路径用于可视化
    if (!path.empty()) {
        nav_msgs::Path astar_path;
        astar_path.header.stamp = ros::Time::now();
        astar_path.header.frame_id = "map";

        for (const auto& waypoint : path) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = astar_path.header.stamp;
            pose_stamped.header.frame_id = astar_path.header.frame_id;
            pose_stamped.pose.position.x = waypoint(0);
            pose_stamped.pose.position.y = waypoint(1);
            pose_stamped.pose.position.z = waypoint(2);
            
            // 使用Eigen计算四元数
            Eigen::AngleAxisd rotation(waypoint(3), Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q(rotation);
                
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();
            
            astar_path.poses.push_back(pose_stamped);
        }

        astar_path_pub_.publish(astar_path);
        ROS_INFO("[PLANNER] Published A* path with %zu waypoints", path.size());
    }

    // 在规划完成后重新发布障碍物
    publishObstacleMarkers();

    return path;
}


//处理目标位姿消息
void BasicPlanner::goalposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{  
    goal_pose_msg_ = *msg;
    tf::poseMsgToEigen(msg->pose, goal_pose_affine_);                                       //将PoseStamped消息转换为Eigen仿射变换 
    goal_pose_ << goal_pose_affine_.translation(),                                          //提取仿射变换的平移和偏航角，更新goal_pose_
        mav_msgs::yawFromQuaternion((Eigen::Quaterniond)goal_pose_affine_.rotation());
    goal_received_ = true;
    ROS_INFO("[PLANNER] Goal received: x=%.2f, y=%.2f, z=%.2f", 
             goal_pose_msg_.pose.position.x,
             goal_pose_msg_.pose.position.y,
             goal_pose_msg_.pose.position.z);
}

//用于处理接收到的局部位置信息
void BasicPlanner::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::poseMsgToEigen(msg->pose, current_pose_);
}

//用于处理接收到的局部速度信息
void BasicPlanner::localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    tf::vectorMsgToEigen(msg->twist.linear, current_velocity_);
}

//用于获取当前规划器是否处于激活状态
bool BasicPlanner::getActiveFlag()
{
    return planner_active_;
}

//设置最大速度
void BasicPlanner::setMaxSpeed(const double max_v) 
{
    max_v_ = max_v;
}

//向轨迹生成的顶点列表中添加一个顶点，并根据输入参数动态设置或移除位置、速度、加速度等约束条件。默认值为-1时表示不设置对应约束。
bool BasicPlanner::add_vertex(mav_trajectory_generation::Vertex::Vector *p_vertices, // pointer to vertices
                                mav_trajectory_generation::Vertex *p_middle,  // pointer to middle
                                Eigen::Vector4d pos = Eigen::Vector4d(-1, -1, -1, -1),  // some defaults
                                Eigen::Vector4d vel = Eigen::Vector4d(-1, -1, -1, -1),
                                Eigen::Vector4d acc = Eigen::Vector4d(-1, -1, -1, -1)) 
{

    /* how to set and remove constraints:
     middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
     middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel);
     middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc);

     middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
     middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
     middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);

     vertices.push_back(middle);
    */

    if (pos[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    }

    if (pos[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, pos);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::ORIENTATION);
    }


    if (vel[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    }

    if (acc[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    }

    p_vertices->push_back(*p_middle);
    return true;
}

//实现了一个轨迹规划器的核心逻辑
//配置起点、中间点和终点的约束条件（位置、速度、加速度等）   密度控制：相邻航点间距建议为最大速度的2-3倍   必经点：设置位置约束（POSITION） 速度衔接点：添加速度约束（VELOCITY） 紧急制动点：设置加速度约束（ACCELERATION）
//根据状态（如WAYPOINT、TRAJECTORY等）动态加载航点参数。
//估计分段时间并使用非线性优化生成满足速度和加速度约束的轨迹。
//返回生成的轨迹
bool BasicPlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) 
{
	assert(trajectory);      //初始化轨迹
	trajectory->clear();

    // Array for all waypoints and their constraints 初始化顶点
    mav_trajectory_generation::Vertex::Vector vertices;

    // Start:   current position
    // Middle:  all middle waypoints with optional derivative constraints
    // End:     final position with all derivates equal zero  
    mav_trajectory_generation::Vertex start(dimension_), middle(dimension_), end(dimension_);

    // 在这里声明astar_waypoints，使其在函数作用域内可见
    std::vector<Eigen::Vector4d> astar_waypoints;
    

    /******* Configure start point *******/
    //配置起点约束
	Eigen::Vector4d current_p, current_v;
    current_p << current_pose_.translation(),mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation());
	current_v << current_velocity_, 0.0;

    // 添加调试日志
    ROS_INFO("[PLANNER] Current position: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", current_p(0), current_p(1), current_p(2), current_p(3));
    ROS_INFO("[PLANNER] Goal position: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", goal_pose_(0), goal_pose_(1), goal_pose_(2), goal_pose_(3));

    // 检查当前位置是否有效（不是Identity矩阵的原点）
    if (current_p.norm() < 0.01) {
        ROS_ERROR("[PLANNER] Current pose not received yet! Using origin as start position");
        // 使用原点作为起点
        current_p << 0.0, 0.0, 0.0, 0.0;
    }

	// Set start point constraints to current position
    start.makeStartOrEnd(current_p, derivative_to_optimize_);
    // Set all derivatives to zero
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_v);
    // Add waypoint to list
    vertices.push_back(start);

    
    /******* Configure trajectory *******/
    Eigen::Vector4d pos, vel, acc;
    bool no_fail = true;
    int i = 0;

    switch (state_)
    {

    case BasicPlanner::WAYPOINT:
    {
        /******* Configure end point *******/
        // Set end point constraints to desired position 配置终点约束
        end.makeStartOrEnd(goal_pose_, derivative_to_optimize_);
        // Set all derivatives to zero
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_velocity_);
        end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, goal_acceleration_);
        // Add waypoint to list
        vertices.push_back(end);
      }
    break;
    case BasicPlanner::TRAJECTORY:    //加载航点参数
    {
        while (no_fail){
            // std::cout<<"wp "<<i<<": preparing for import"<<std::endl;
            std::string idx = "/"+ std::to_string(i) + "/";
            std::string wp_str = ros::this_node::getName() + "/waypoints" + idx;

            // positions
            float pos_x, pos_y, pos_z, pos_h;
            no_fail = nh_.getParam(wp_str + "pos/x", pos_x);
            no_fail = nh_.getParam(wp_str + "pos/y", pos_y);
            no_fail = nh_.getParam(wp_str + "pos/z", pos_z);
            no_fail = nh_.getParam(wp_str + "pos/h", pos_h);
            if (!no_fail){
                // std::cout<<"      failed to add waypoint"<<std::endl;
                // std::cout<<"   -> ending import loop"<<std::endl;
                // std::cout<<"      now starting to compute trajectory"<<std::endl;
                ROS_WARN("[PLANNER] RECEIVED ALL WAYPOINTS, COMPUTING TRAJECTORY");
                break;
            }
            pos << pos_x, pos_y, pos_z, pos_h;  // write position constraints to vector
            // std::cout<<"      position constraints received"<<std::endl;
            // std::cout<<"      wp pos: "<<pos_x<<" "<<pos_y<<" "<<pos_z<<" "<<pos_h<<std::endl;

            // velocities
            float vel_x, vel_y, vel_z, vel_h;
            no_fail = nh_.getParam(wp_str + "vel/x", vel_x);
            no_fail = nh_.getParam(wp_str + "vel/y", vel_y);
            no_fail = nh_.getParam(wp_str + "vel/z", vel_z);
            no_fail = nh_.getParam(wp_str + "vel/h", vel_h);
            if (!no_fail){
                // std::cout<<"      failed to add velocity constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position constraint only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos);
                no_fail = true;  // reset failure indicator for next while loop
                i++;  // increase counter since we added the waypoint with position constraints only
                continue;
            }
            vel << vel_x, vel_y, vel_z, vel_h;  // write velocity constraints to vector; we got the position at least!
            // std::cout<<"      velocity constraints received"<<std::endl;
            // std::cout<<"      wp vel: "<<vel_x<<" "<<vel_y<<" "<<vel_z<<" "<<vel_h<<std::endl;

            // accelerations
            float acc_x, acc_y, acc_z, acc_h;
            no_fail = nh_.getParam(wp_str + "acc/x", acc_x);
            no_fail = nh_.getParam(wp_str + "acc/y", acc_y);
            no_fail = nh_.getParam(wp_str + "acc/z", acc_z);
            no_fail = nh_.getParam(wp_str + "acc/h", acc_h);
            if (!no_fail){
                // std::cout<<"      failed to add acceleration constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position and velocity constraints only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos, vel);
                no_fail = true;  // reset failure indicator for next while loop; we got the position at least!
                i++;  // increase counter since we added the waypoint with position and velocity constraints only
                continue;
            }
            acc << acc_x, acc_y, acc_z, acc_h;  // write acceleration constraints to vector
            // std::cout<<"      acceleration constraints received"<<std::endl;
            // std::cout<<"      wp acc: "<<acc_x<<" "<<acc_y<<" "<<acc_z<<" "<<acc_h<<std::endl;
            // std::cout<<"      wp "<<i<<" added with position, velocity and acceleration constraints"<<std::endl;
            // std::cout<<std::endl;

            BasicPlanner::add_vertex(&vertices, &middle, pos, vel, acc);
            i++;  // increase counter
        }
      }
    break;

    case BasicPlanner::VER_PERCHING:    //加载垂直降落航点
    {
        while (no_fail){
            // std::cout<<"wp "<<i<<": preparing for import"<<std::endl;
            std::string idx = "/"+ std::to_string(i) + "/";
            std::string wp_str = ros::this_node::getName() + "/waypoints_vertical_perching" + idx;

            // positions
            float pos_x, pos_y, pos_z, pos_h;
            no_fail = nh_.getParam(wp_str + "pos/x", pos_x);
            no_fail = nh_.getParam(wp_str + "pos/y", pos_y);
            no_fail = nh_.getParam(wp_str + "pos/z", pos_z);
            no_fail = nh_.getParam(wp_str + "pos/h", pos_h);
            if (!no_fail){
                // std::cout<<"      failed to add waypoint"<<std::endl;
                // std::cout<<"   -> ending import loop"<<std::endl;
                // std::cout<<"      now starting to compute trajectory"<<std::endl;
                ROS_WARN("[PLANNER] VERTICAL PERCHING, COMPUTING TRAJECTORY");
                break;
            }
            pos << pos_x, pos_y, pos_z, pos_h;  // write position constraints to vector
            // std::cout<<"      position constraints received"<<std::endl;
            // std::cout<<"      wp pos: "<<pos_x<<" "<<pos_y<<" "<<pos_z<<" "<<pos_h<<std::endl;

            // velocities
            float vel_x, vel_y, vel_z, vel_h;
            no_fail = nh_.getParam(wp_str + "vel/x", vel_x);
            no_fail = nh_.getParam(wp_str + "vel/y", vel_y);
            no_fail = nh_.getParam(wp_str + "vel/z", vel_z);
            no_fail = nh_.getParam(wp_str + "vel/h", vel_h);
            if (!no_fail){
                // std::cout<<"      failed to add velocity constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position constraint only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos);
                no_fail = true;  // reset failure indicator for next while loop
                i++;  // increase counter since we added the waypoint with position constraints only
                continue;
            }
            vel << vel_x, vel_y, vel_z, vel_h;  // write velocity constraints to vector; we got the position at least!
            // std::cout<<"      velocity constraints received"<<std::endl;
            // std::cout<<"      wp vel: "<<vel_x<<" "<<vel_y<<" "<<vel_z<<" "<<vel_h<<std::endl;

            // accelerations
            float acc_x, acc_y, acc_z, acc_h;
            no_fail = nh_.getParam(wp_str + "acc/x", acc_x);
            no_fail = nh_.getParam(wp_str + "acc/y", acc_y);
            no_fail = nh_.getParam(wp_str + "acc/z", acc_z);
            no_fail = nh_.getParam(wp_str + "acc/h", acc_h);
            if (!no_fail){
                // std::cout<<"      failed to add acceleration constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position and velocity constraints only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos, vel);
                no_fail = true;  // reset failure indicator for next while loop; we got the position at least!
                i++;  // increase counter since we added the waypoint with position and velocity constraints only
                continue;
            }
            acc << acc_x, acc_y, acc_z, acc_h;  // write acceleration constraints to vector
            // std::cout<<"      acceleration constraints received"<<std::endl;
            // std::cout<<"      wp acc: "<<acc_x<<" "<<acc_y<<" "<<acc_z<<" "<<acc_h<<std::endl;
            // std::cout<<"      wp "<<i<<" added with position, velocity and acceleration constraints"<<std::endl;
            // std::cout<<std::endl;

            BasicPlanner::add_vertex(&vertices, &middle, pos, vel, acc);
            i++;  // increase counter
        }

      }
    break;

    case BasicPlanner::INC_PERCHING:       //加载倾斜降落航点
    {
        while (no_fail){
            // std::cout<<"wp "<<i<<": preparing for import"<<std::endl;
            std::string idx = "/"+ std::to_string(i) + "/";
            std::string wp_str = ros::this_node::getName() + "/waypoints_inclined_perching" + idx;

            // positions
            float pos_x, pos_y, pos_z, pos_h;
            no_fail = nh_.getParam(wp_str + "pos/x", pos_x);
            no_fail = nh_.getParam(wp_str + "pos/y", pos_y);
            no_fail = nh_.getParam(wp_str + "pos/z", pos_z);
            no_fail = nh_.getParam(wp_str + "pos/h", pos_h);
            if (!no_fail){
                // std::cout<<"      failed to add waypoint"<<std::endl;
                // std::cout<<"   -> ending import loop"<<std::endl;
                // std::cout<<"      now starting to compute trajectory"<<std::endl;
                ROS_WARN("[PLANNER] INCLINED PERCHING, COMPUTING TRAJECTORY");
                break;
            }
            pos << pos_x, pos_y, pos_z, pos_h;  // write position constraints to vector
            // std::cout<<"      position constraints received"<<std::endl;
            // std::cout<<"      wp pos: "<<pos_x<<" "<<pos_y<<" "<<pos_z<<" "<<pos_h<<std::endl;

            // velocities
            float vel_x, vel_y, vel_z, vel_h;
            no_fail = nh_.getParam(wp_str + "vel/x", vel_x);
            no_fail = nh_.getParam(wp_str + "vel/y", vel_y);
            no_fail = nh_.getParam(wp_str + "vel/z", vel_z);
            no_fail = nh_.getParam(wp_str + "vel/h", vel_h);
            if (!no_fail){
                // std::cout<<"      failed to add velocity constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position constraint only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos);
                no_fail = true;  // reset failure indicator for next while loop
                i++;  // increase counter since we added the waypoint with position constraints only
                continue;
            }
            vel << vel_x, vel_y, vel_z, vel_h;  // write velocity constraints to vector; we got the position at least!
            // std::cout<<"      velocity constraints received"<<std::endl;
            // std::cout<<"      wp vel: "<<vel_x<<" "<<vel_y<<" "<<vel_z<<" "<<vel_h<<std::endl;

            // accelerations
            float acc_x, acc_y, acc_z, acc_h;
            no_fail = nh_.getParam(wp_str + "acc/x", acc_x);
            no_fail = nh_.getParam(wp_str + "acc/y", acc_y);
            no_fail = nh_.getParam(wp_str + "acc/z", acc_z);
            no_fail = nh_.getParam(wp_str + "acc/h", acc_h);
            if (!no_fail){
                // std::cout<<"      failed to add acceleration constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position and velocity constraints only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos, vel);
                no_fail = true;  // reset failure indicator for next while loop; we got the position at least!
                i++;  // increase counter since we added the waypoint with position and velocity constraints only
                continue;
            }
            acc << acc_x, acc_y, acc_z, acc_h;  // write acceleration constraints to vector
            // std::cout<<"      acceleration constraints received"<<std::endl;
            // std::cout<<"      wp acc: "<<acc_x<<" "<<acc_y<<" "<<acc_z<<" "<<acc_h<<std::endl;
            // std::cout<<"      wp "<<i<<" added with position, velocity and acceleration constraints"<<std::endl;
            // std::cout<<std::endl;

            BasicPlanner::add_vertex(&vertices, &middle, pos, vel, acc);
            i++;  // increase counter
        }

      }
    break;
    case BasicPlanner::ATRAJECTORY:
        {
            // 使用A*算法规划路径
            if (!goal_received_) {
                ROS_WARN("[PLANNER] No goal received for A* trajectory");
                return false;
            }
            
            ROS_INFO("[PLANNER] Current position: (%.2f, %.2f, %.2f)", current_p(0), current_p(1), current_p(2));
            ROS_INFO("[PLANNER] Goal position: (%.2f, %.2f, %.2f)", goal_pose_(0), goal_pose_(1), goal_pose_(2));
        
            // 使用A*算法生成路径点
            // std::vector<Eigen::Vector4d> astar_waypoints = planAStarPath(current_p, goal_pose_);
            // 使用A*算法生成路径点 - 直接赋值给函数作用域的变量
            astar_waypoints = planAStarPath(current_p, goal_pose_);
            
            if (astar_waypoints.empty()) {
                ROS_ERROR("[PLANNER] A* failed to find a path");
                return false;
            }

            ROS_INFO("[PLANNER] A* generated %zu waypoints", astar_waypoints.size());

            // 新增：路径简化（使用Douglas-Peucker算法）epsilon越大越简化
            // std::vector<Eigen::Vector4d> simplified_waypoints = simplifyPath(astar_waypoints, 0.3);
            std::vector<Eigen::Vector4d> simplified_waypoints = simplifyPath(astar_waypoints, 0.1);
            ROS_INFO("[PLANNER] Simplified to %zu waypoints", simplified_waypoints.size());
            
            // 新增：路径平滑（使用迭代平滑） 减少平滑迭代次数和调整平滑权重
            // std::vector<Eigen::Vector4d> smoothed_waypoints = smoothPath(simplified_waypoints, 0.5, 5);
            std::vector<Eigen::Vector4d> smoothed_waypoints = smoothPath(simplified_waypoints, 0.3, 5);
            smoothed_waypoints = applyBSplineSmoothing(smoothed_waypoints, 0.5);
    
            // 将平滑后的路径点添加为轨迹顶点
            for (size_t i = 0; i < smoothed_waypoints.size(); i++) {
                Eigen::Vector4d waypoint_pos = smoothed_waypoints[i];
                
                // 对于中间点，只添加位置约束
                if (i < smoothed_waypoints.size() - 1) {
                    mav_trajectory_generation::Vertex middle_wp(dimension_);
                    middle_wp.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoint_pos);
                    vertices.push_back(middle_wp);
                } else {
                    // 最后一个点作为终点，添加位置和速度约束
                    end.makeStartOrEnd(waypoint_pos, derivative_to_optimize_);
                    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d::Zero());
                    vertices.push_back(end);
                }
            }

            // // 将A*路径点添加为轨迹顶点
            // for (size_t i = 0; i < astar_waypoints.size(); i++) {
            //     Eigen::Vector4d waypoint_pos = astar_waypoints[i];
                
            //     // 对于中间点，只添加位置约束
            //     if (i < astar_waypoints.size() - 1) {
            //         mav_trajectory_generation::Vertex middle_wp(dimension_);
            //         middle_wp.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoint_pos);
            //         vertices.push_back(middle_wp);
            //     } else {
            //         // 最后一个点作为终点，添加位置和速度约束
            //         end.makeStartOrEnd(waypoint_pos, derivative_to_optimize_);
            //         end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d::Zero());
            //         vertices.push_back(end);
            //     }
            // }
        }
        break;

    }

    //基于多项式优化的轨迹生成方法，具体来说是采用10阶多项式非线性优化
    /******* Estimate initial segment times *******/
    //估计分段时间
    std::vector<double> segment_times;
    // double smoothness_weight = 2.0;  // 增加这个值会使轨迹更平滑
    // segment_times = estimateSegmentTimes(vertices, max_v_, max_a_, smoothness_weight);
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
    
    // std::copy(segment_times.begin(), segment_times.end(), std::ostream_iterator<double>(std::cout," "));
    // std::cout<<std::endl;


    /******* Set up polynomial solver and compute trajectory *******/
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem 设置优化问题
    const int N = 10;     //使用基于多项式优化的轨迹生成方法，采用10阶多项式非线性优化
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension_, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);

    // 添加平滑约束（关键！）
    std::vector<double> weighted_segment_times = segment_times;
    for (size_t i = 0; i < weighted_segment_times.size(); ++i) {
        weighted_segment_times[i] *= 2.5;  // 增加分段时间，使轨迹更平滑
    }
    opt.setupFromVertices(vertices, weighted_segment_times, derivative_to_optimize_);

    // constrain velocity and acceleration   添加速度和加速度约束，硬性约束：速度/加速度不超过最大值，软性约束：轨迹点位置/方向约束
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory 
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
	trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
	

    // 验证轨迹是否安全
    if (!isTrajectorySafe(*trajectory)) {
        ROS_ERROR("[PLANNER] Generated trajectory collides with obstacles. Replanning with safety constraints...");
        
        // 尝试使用更保守的参数重新规划
        // std::vector<Eigen::Vector4d> safe_waypoints = ensureTrajectorySafety(astar_waypoints);
        // 只有在ATRAJECTORY状态下才使用astar_waypoints
        std::vector<Eigen::Vector4d> safe_waypoints;
        if (state_ == BasicPlanner::ATRAJECTORY) {
            safe_waypoints = ensureTrajectorySafety(astar_waypoints);
        } else {
            // 对于其他状态，可能需要不同的安全重新规划策略
            // 例如，可以使用原始轨迹点
            safe_waypoints = getOriginalWaypoints(vertices);
        }

        // 重新生成轨迹
        vertices.clear();
        vertices.push_back(start);
        for (size_t i = 0; i < safe_waypoints.size(); i++) {
            if (i < safe_waypoints.size() - 1) {
                mav_trajectory_generation::Vertex middle_wp(dimension_);
                middle_wp.addConstraint(mav_trajectory_generation::derivative_order::POSITION, safe_waypoints[i]);
                vertices.push_back(middle_wp);
            } else {
                end.makeStartOrEnd(safe_waypoints[i], derivative_to_optimize_);
                end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d::Zero());
                vertices.push_back(end);
            }
        }
        
        // 重新估计分段时间
        segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
        
        // 重新设置优化问题
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
        opt.optimize();
        opt.getTrajectory(&(*trajectory));
        trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
        
        // 再次验证
        if (!isTrajectorySafe(*trajectory)) {
            ROS_ERROR("[PLANNER] Safety replanning failed. Using fallback trajectory.");
            // 可以使用更简单的安全轨迹作为后备
        }
    }

    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory)
{
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.4; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "map";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "map";
    pub_trajectory_.publish(msg);

    return true;
}

void BasicPlanner::initializePublishers()
{

    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
    // 添加A*路径发布者
    astar_path_pub_ = nh_.advertise<nav_msgs::Path>("astar_path", 1);

}

void BasicPlanner::initializeSubscribers()
{

    // goal_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
    //         ("planner/goal_pose", 10, &BasicPlanner::goalposeCallback, this);

    // state_machine_sub_ = nh_.subscribe
    //         ("planner/activation", 1, &BasicPlanner::plannerTriggerCallback, this);

    local_pos_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &BasicPlanner::localposeCallback, this);

    local_pos_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 10, &BasicPlanner::localvelCallback, this);

    // 添加目标点订阅
    goal_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("planner/goal_pose", 10, &BasicPlanner::goalposeCallback, this);
    
    
}

void BasicPlanner::initializeServices()
{
    trigger_service_ = nh_.advertiseService
            ("planner/trigger", &BasicPlanner::plannerTriggerServiceCallback,this);
    
}

bool BasicPlanner::plannerTriggerServiceCallback(planner::GetTrajectory::Request &req,
                                                 planner::GetTrajectory::Response &res)
{
    int key = req.type;
    // Check if the type is valid, if not land on current position
    if(std::count(valid_states_.begin(), valid_states_.end(), key)){
        state_ = req.type;
        if (key==0)
            goal_pose_ << req.x, req.y, req.z, req.h;
        // 对于ATRAJECTORY状态，如果还没有收到目标点，则使用请求中的目标点
        else if (key == 4) { // ATRAJECTORY
            if (!goal_received_) {
                // 如果没有通过话题接收到目标点，则使用服务请求中的目标点
                if (req.x != -1 && req.y != -1 && req.z != -1) {
                    goal_pose_ << req.x, req.y, req.z, req.h;
                    goal_received_ = true;
                    ROS_INFO("[PLANNER] Using goal from service request: x=%.2f, y=%.2f, z=%.2f", 
                             req.x, req.y, req.z);
                }
            } else {
                ROS_INFO("[PLANNER] Using goal from topic subscription");
            }
        }
    }else{
        state_ = 0;
        Eigen::Vector4d current_p;
        current_p << current_pose_.translation(),mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation());
        goal_pose_ << current_p(0), current_p(1), 0.0, current_p(3);
        ROS_ERROR("[PLANNER] INVALID TYPE, GOING DOWN!");
    }
    
    mav_trajectory_generation::Trajectory trajectory;

    planTrajectory(&trajectory);
    publishTrajectory(trajectory);
    
    ROS_WARN_STREAM("[PLANNER] TRAJECTORY PUBLISHED");

    res.success = true;

    return true;
}

void BasicPlanner::readParameters()
{
    nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v", max_v_);
    ROS_WARN("[PLANNER] Max Vel: %.1f m/s", max_v_);

    nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a", max_a_);
    ROS_WARN("[PLANNER] Max Acc: %.1f m/s^2", max_a_);
}

// 计算点到线段的距离
double BasicPlanner::pointToLineDistance(const Eigen::Vector4d& p, 
                                       const Eigen::Vector4d& lineStart, 
                                       const Eigen::Vector4d& lineEnd) const {
    Eigen::Vector3d p3d(p(0), p(1), p(2));
    Eigen::Vector3d start3d(lineStart(0), lineStart(1), lineStart(2));
    Eigen::Vector3d end3d(lineEnd(0), lineEnd(1), lineEnd(2));
    
    Eigen::Vector3d lineVec = end3d - start3d;
    Eigen::Vector3d pointVec = p3d - start3d;
    
    double t = pointVec.dot(lineVec) / lineVec.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));
    
    Eigen::Vector3d projection = start3d + t * lineVec;
    return (p3d - projection).norm();
}

// Douglas-Peucker算法简化路径
std::vector<Eigen::Vector4d> BasicPlanner::simplifyPath(const std::vector<Eigen::Vector4d>& path, 
                                                     double epsilon) const {
    if (path.size() < 3) return path;
    
    // 找到离起点-终点线段最远的点
    int maxIndex = 0;
    double maxDistance = 0;
    
    for (int i = 1; i < path.size()-1; i++) {
        double distance = pointToLineDistance(path[i], path[0], path.back());
        if (distance > maxDistance) {
            maxDistance = distance;
            maxIndex = i;
        }
    }
    
    if (maxDistance > epsilon) {
        // 递归处理
        std::vector<Eigen::Vector4d> part1(path.begin(), path.begin() + maxIndex + 1);
        std::vector<Eigen::Vector4d> part2(path.begin() + maxIndex, path.end());
        
        std::vector<Eigen::Vector4d> simplified1 = simplifyPath(part1, epsilon);
        std::vector<Eigen::Vector4d> simplified2 = simplifyPath(part2, epsilon);
        
        // 合并结果（去除重复点）
        simplified1.pop_back(); // 移除最后一个点，因为它在part2中
        simplified1.insert(simplified1.end(), simplified2.begin(), simplified2.end());
        return simplified1;
    } else {
        // 只保留起点和终点
        return {path[0], path.back()};
    }
}

// 迭代平滑算法
std::vector<Eigen::Vector4d> BasicPlanner::smoothPath(const std::vector<Eigen::Vector4d>& path, 
                                                   double weight_data, 
                                                   int iterations) const {
    if (path.size() < 2) return path;
    
    std::vector<Eigen::Vector4d> smoothed = path;
    int n = path.size();
    const double safety_margin = 0.1; // 安全距离（米）
    
    for (int iter = 0; iter < iterations; iter++) {
        for (int i = 1; i < n-1; i++) {
            Eigen::Vector4d prev = smoothed[i-1];
            Eigen::Vector4d curr = smoothed[i];
            Eigen::Vector4d next = smoothed[i+1];
            
            // 平滑公式：curr = (1-weight_data)*curr + weight_data*(prev+next)/2
            Eigen::Vector4d smoothed_point = curr * (1 - weight_data) + (prev + next) * 0.5 * weight_data;
            
            // // 保持yaw角连续性
            // smoothed_point(3) = curr(3); // 或者使用更复杂的yaw平滑
            // 保持yaw角连续性（关键改进）
            double dx = next(0) - prev(0);
            double dy = next(1) - prev(1);
            if (dx != 0 || dy != 0) {
                smoothed_point(3) = atan2(dy, dx);
            } else {
                smoothed_point(3) = curr(3); // 保持原有方向
            }

            // smoothed[i] = smoothed_point;
            // 检查平滑点是否安全（不在障碍物内且有安全距离）
            if (!isPointInObstacle(smoothed_point, safety_margin)) {
                smoothed[i] = smoothed_point;
            }
            // 否则保留原始点（不进行平滑）
        }
    }
    
    return smoothed;
}

bool BasicPlanner::isPointInObstacle(const Eigen::Vector4d& point, double safety_margin) const {
    // 将世界坐标转换为网格坐标
    int grid_x = static_cast<int>(point(0) / obstacle_map_->getResolution());
    int grid_y = static_cast<int>(point(1) / obstacle_map_->getResolution());
    int grid_z = static_cast<int>(point(2) / obstacle_map_->getResolution());
    
    // 检查网格坐标是否有效
    if (!obstacle_map_->isValid(grid_x, grid_y, grid_z)) {
        return true;  // 无效坐标视为障碍物
    }
    
    // 计算安全距离对应的网格单元数
    int safe_radius = static_cast<int>(safety_margin / obstacle_map_->getResolution());
    
    // 检查安全距离内的所有网格点
    for (int dx = -safe_radius; dx <= safe_radius; dx++) {
        for (int dy = -safe_radius; dy <= safe_radius; dy++) {
            for (int dz = -safe_radius; dz <= safe_radius; dz++) {
                int nx = grid_x + dx;
                int ny = grid_y + dy;
                int nz = grid_z + dz;
                
                if (obstacle_map_->isValid(nx, ny, nz) && obstacle_map_->isOccupied(nx, ny, nz)) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool BasicPlanner::isTrajectorySafe(const mav_trajectory_generation::Trajectory& trajectory) const {
    double total_time = trajectory.getMaxTime();
    double check_interval = 0.1;  // 每0.1秒检查一次
    const double safety_margin = 0.3; // 安全距离（米）
    
    for (double t = 0.0; t < total_time; t += check_interval) {
        Eigen::VectorXd pos = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
        Eigen::Vector4d point(pos(0), pos(1), pos(2), 0.0);
        
        if (isPointInObstacle(point, safety_margin)) {
            ROS_WARN("[PLANNER] Trajectory collision detected at t=%.2f: (%.2f, %.2f, %.2f)", 
                     t, point(0), point(1), point(2));
            return false;
        }
    }
    
    return true;
}

std::vector<Eigen::Vector4d> BasicPlanner::ensureTrajectorySafety(const std::vector<Eigen::Vector4d>& original_path) const {
    std::vector<Eigen::Vector4d> safe_path;
    const double safety_margin = 0.3;
    
    for (const auto& point : original_path) {
        Eigen::Vector4d safe_point = point;
        int max_attempts = 5;
        double step_size = 0.1; // 每次调整0.1米
        
        // 尝试向不同方向移动，直到找到安全点
        for (int attempt = 0; attempt < max_attempts; attempt++) {
            if (!isPointInObstacle(safe_point, safety_margin)) {
                break;
            }
            
            // 尝试向远离障碍物的方向移动
            // 这里可以实现更智能的避障策略
            safe_point(0) += step_size * (rand() % 2 ? 1 : -1);
            safe_point(1) += step_size * (rand() % 2 ? 1 : -1);
            safe_point(2) += step_size * (rand() % 2 ? 1 : -1);
        }
        
        safe_path.push_back(safe_point);
    }
    
    return safe_path;
}

std::vector<Eigen::Vector4d> BasicPlanner::getOriginalWaypoints(const mav_trajectory_generation::Vertex::Vector& vertices) const {
    std::vector<Eigen::Vector4d> waypoints;
    
    for (const auto& vertex : vertices) {
        if (vertex.hasConstraint(mav_trajectory_generation::derivative_order::POSITION)) {
            // 修正：使用正确的API调用方式
            Eigen::VectorXd pos;
            bool hasConstraint = vertex.getConstraint(mav_trajectory_generation::derivative_order::POSITION, &pos);
            
            if (hasConstraint && pos.size() >= 3) {
                Eigen::Vector4d waypoint;
                waypoint(0) = pos(0);
                waypoint(1) = pos(1);
                waypoint(2) = pos(2);
                waypoint(3) = 0.0; // 默认偏航角
                waypoints.push_back(waypoint);
            }
        }
    }
    
    return waypoints;
}

std::vector<Eigen::Vector4d> BasicPlanner::applyBSplineSmoothing(const std::vector<Eigen::Vector4d>& path, 
                                                              double smoothness, 
                                                              int degree) const {
    if (path.size() < 2) return path;
    
    std::vector<Eigen::Vector4d> smoothed = path;
    int n = path.size();
    
    // 创建控制点
    std::vector<Eigen::Vector4d> control_points = path;
    
    // 应用B样条平滑
    for (int i = 1; i < n-1; i++) {
        Eigen::Vector4d prev = (i > 0) ? smoothed[i-1] : smoothed[i];
        Eigen::Vector4d curr = smoothed[i];
        Eigen::Vector4d next = (i < n-1) ? smoothed[i+1] : smoothed[i];
        
        // B样条公式
        Eigen::Vector4d bspline_point = 
            (1-smoothness) * curr + 
            (smoothness/6.0) * (prev + next);
        
        // 保持yaw角连续性
        double dx = next(0) - prev(0);
        double dy = next(1) - prev(1);
        if (dx != 0 || dy != 0) {
            bspline_point(3) = atan2(dy, dx);
        }
        
        smoothed[i] = bspline_point;
    }
    
    return smoothed;
}