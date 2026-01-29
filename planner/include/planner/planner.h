/**
 * @file planner.h
 *
 * @date 03 June 2022
 * 
 * @brief Trajectory Planner using mav_trajectory_generation
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */
#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include "planner/GetTrajectory.h"
#include <nav_msgs/Path.h>


class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    enum States
    {
        WAYPOINT = 0,       // From current pose directly to an end goal
        TRAJECTORY = 1,     // Multiple waypoints from YAML file
        VER_PERCHING = 2,   // Perching from above, vertical
        INC_PERCHING = 3,   // Perching from side, inclined
        ATRAJECTORY = 4      // 新增的 A* 轨迹规划状态

    };

    // Callback function to get the goal pose
    void goalposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
    // Callback function to get the current pose
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Callback function to get the current linear and angular velocity
    void localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Trigger that generates and publishes trajectory
    //void plannerTriggerCallback(const std_msgs::Bool::ConstPtr& msg);

    // Get the value for activation flag
    bool getActiveFlag();

    // Set the maximum speed read from yaml file
    void setMaxSpeed(double max_v);
    
    // Function to set or removes constraints from middle waypoints
    bool add_vertex(mav_trajectory_generation::Vertex::Vector* p_vertices,
                    mav_trajectory_generation::Vertex* p_middle,
                    Eigen::Vector4d pos,
                    Eigen::Vector4d vel,
                    Eigen::Vector4d acc);

    // Plans a trajectory from the current position to a goal position and velocity
    bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

    // Initialize publishers, subscribers and services
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();

    bool plannerTriggerServiceCallback(planner::GetTrajectory::Request &req,
                                       planner::GetTrajectory::Response &res);

    // Read the parameters from yaml file, max velocity and acceleration
    void readParameters();

    // A* 算法方法
    std::vector<Eigen::Vector4d> planAStarPath(const Eigen::Vector4d& start, const Eigen::Vector4d& goal);

    // 在 BasicPlanner 内部定义 AStarNode
    struct AStarNode {
        int x, y, z;
        double g_cost;  // 实际代价
        double f_cost;  // 总代价 (g + h)
        AStarNode* parent;
        
        AStarNode(int x_, int y_, int z_, double g = 0, double f = 0, AStarNode* p = nullptr) 
            : x(x_), y(y_), z(z_), g_cost(g), f_cost(f), parent(p) {}
    };
        
    
private:
    
    ros::NodeHandle& nh_;

    ros::Publisher pub_markers_;                // RVIZ trajectory markers
    ros::Publisher pub_trajectory_;             // Polynomial trajectory
    ros::Publisher astar_path_pub_;              // A*路径发布者
    // 添加障碍物可视化发布器
    ros::Publisher obstacle_markers_pub_;

    
    //ros::Subscriber goal_pose_sub_;             // Position & Orientation of the goal
    //ros::Subscriber state_machine_sub_;         // State machine trigger
    ros::Subscriber local_pos_vel_sub_;         // Linear & Angular velocity from mavros
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation from mavros


    ros::ServiceServer trigger_service_;        // ####################

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;

    Eigen::Affine3d goal_pose_affine_;
    Eigen::Vector4d goal_pose_;
    Eigen::Vector4d goal_velocity_;
    Eigen::Vector4d goal_acceleration_;
    
    bool planner_active_;                       // Trigger flag to generate a trajectory
    bool planner_done_;                         // Flag used to stop after publishing trajectory

    int state_;                                 // State to define the type of generated trajectory
    int dimension_;                             // 3D or 4D trajectories, 4th dim is heading 
    int derivative_to_optimize_;                // Optimize up to 4th order derivative (SNAP)

    double max_v_;                              // Maximum velocity constraint [m/s]
    double max_a_;                              // Maximum acceleration constraint [m/s^2]
    double max_ang_v_;
    double max_ang_a_;

    std::vector<int> valid_states_;          // Includes the neums of valid states

    // A* 算法相关成员
    ros::Subscriber goal_pose_sub_;             // Position & Orientation of the goal
    geometry_msgs::PoseStamped goal_pose_msg_;  // 外部目标点
    bool goal_received_;                        // 是否收到目标点
    
    // A* 算法结构
    // struct AStarNode {
    //     int x, y, z;
    //     double g_cost, h_cost, f_cost;
    //     AStarNode* parent;
        
    //     AStarNode(int x_, int y_, int z_) : x(x_), y(y_), z(z_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    // };
    


    // 障碍物地图类
    class ObstacleMap {
    private:
    std::vector<std::vector<std::vector<bool>>> grid_;
    double resolution_;
    int size_x_, size_y_, size_z_;  // 确保这些成员变量存在
    
    //用于存储原始障碍物信息的数据结构
    struct BoxObstacle {
        double x, y, z, width, length, height;
    };
    struct SphereObstacle {
        double x, y, z, radius;
    };
    std::vector<BoxObstacle> box_obstacles_;
    std::vector<SphereObstacle> sphere_obstacles_;

    public:
    ObstacleMap(int sx, int sy, int sz, double res);
    void addObstacle(double x, double y, double z, double radius);
    void addBoxObstacle(double x, double y, double z, double width, double length, double height);
    bool isOccupied(int x, int y, int z) const;
    bool isValid(int x, int y, int z) const;
    double getResolution() const { return resolution_; }
    
    // 添加以下方法
    int getSizeX() const { return size_x_; }
    int getSizeY() const { return size_y_; }
    int getSizeZ() const { return size_z_; }

    const std::vector<BoxObstacle>& getBoxObstacles() const { return box_obstacles_; }
    const std::vector<SphereObstacle>& getSphereObstacles() const { return sphere_obstacles_; }
    
    };
    
    
    ObstacleMap* obstacle_map_;
    // 添加障碍物发布函数
    void publishObstacleMarkers();

    double pointToLineDistance(const Eigen::Vector4d& p, 
                             const Eigen::Vector4d& lineStart, 
                             const Eigen::Vector4d& lineEnd) const;
    std::vector<Eigen::Vector4d> simplifyPath(const std::vector<Eigen::Vector4d>& path, 
                                           double epsilon) const;
    std::vector<Eigen::Vector4d> smoothPath(const std::vector<Eigen::Vector4d>& path, 
                                         double weight_data, 
                                         int iterations) const;

    bool isPointInObstacle(const Eigen::Vector4d& point, double safety_margin = 0.0) const;

    bool isTrajectorySafe(const mav_trajectory_generation::Trajectory& trajectory) const;
    std::vector<Eigen::Vector4d> ensureTrajectorySafety(const std::vector<Eigen::Vector4d>& original_path) const;

    std::vector<Eigen::Vector4d> getOriginalWaypoints(const mav_trajectory_generation::Vertex::Vector& vertices) const;

    std::vector<Eigen::Vector4d> applyBSplineSmoothing(const std::vector<Eigen::Vector4d>& path, 
                                                 double smoothness, 
                                                 int degree = 3) const;

};


namespace std {
    template<>
    struct hash<BasicPlanner::AStarNode> {
        size_t operator()(const BasicPlanner::AStarNode& n) const {
            // 高效的哈希组合
            auto hash_combine = [](size_t seed, size_t value) {
                return seed ^ (value + 0x9e3779b9 + (seed << 6) + (seed >> 2));
            };
            
            size_t seed = hash<int>()(n.x);
            seed = hash_combine(seed, hash<int>()(n.y));
            seed = hash_combine(seed, hash<int>()(n.z));
            return seed;
        }
    };
}

#endif // PLANNER_H

