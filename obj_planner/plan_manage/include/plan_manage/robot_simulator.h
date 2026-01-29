#ifndef ROBOT_SIMULATOR_H
#define ROBOT_SIMULATOR_H

#include <Eigen/Core>
#include <vector>
#include <fstream>
#include "planner_interface.h"

class RobotSimulator {
private:
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d acceleration_;
    double yaw_ = 0.0;
    
    Eigen::Vector3d last_sent_position_;
    Eigen::Vector3d last_sent_velocity_;
    Eigen::Vector3d last_sent_acceleration_;
    double last_sent_yaw_ = 0.0;
    
    bool use_fc_feedback_ = false;
    std::ofstream fc_data_log_;
    int data_point_index_ = 0;
    
    std::vector<obj_planner::PathPoint> current_trajectory_; // 仅用于内部模拟积分

public:
    RobotSimulator(const Eigen::Vector3d& start_pos);
    ~RobotSimulator();

    void setPosition(const Eigen::Vector3d& pos);
    Eigen::Vector3d getPosition() const;
    
    void enableFlightControlFeedback(bool enable);
    
    void sendToFlightControl(const Eigen::Vector3d& target_pos, 
                           const Eigen::Vector3d& target_vel, 
                           const Eigen::Vector3d& target_acc, 
                           double target_yaw);
                           
    void receiveFromFlightControl(Eigen::Vector3d& feedback_pos,
                                 Eigen::Vector3d& feedback_vel,
                                 Eigen::Vector3d& feedback_acc,
                                 double& feedback_yaw);
                                 
    void updateStateFromTrajectory(const std::vector<obj_planner::PathPoint>& trajectory); // 可选，仅用于Debug
};

#endif