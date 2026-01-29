#include "robot_simulator.h"
#include <iostream>
#include <chrono>
#include <cmath>

RobotSimulator::RobotSimulator(const Eigen::Vector3d& start_pos) 
    : position_(start_pos), velocity_(Eigen::Vector3d::Zero()), acceleration_(Eigen::Vector3d::Zero()),
      last_sent_position_(start_pos), last_sent_velocity_(Eigen::Vector3d::Zero()),
      last_sent_acceleration_(Eigen::Vector3d::Zero()) {
    
    std::string timestamp = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    std::string log_file = "flight_control_sent_data_" + timestamp + ".txt";
    fc_data_log_.open(log_file);
    if (fc_data_log_.is_open()) {
        fc_data_log_ << "# Format: index timestamp x y z vx vy vz speed yaw pitch ax ay az" << std::endl;
    }
}

RobotSimulator::~RobotSimulator() {
    if (fc_data_log_.is_open()) fc_data_log_.close();
}

void RobotSimulator::setPosition(const Eigen::Vector3d& pos) { position_ = pos; }
Eigen::Vector3d RobotSimulator::getPosition() const { return position_; }
void RobotSimulator::enableFlightControlFeedback(bool enable) { use_fc_feedback_ = enable; }

void RobotSimulator::sendToFlightControl(const Eigen::Vector3d& target_pos, 
                                       const Eigen::Vector3d& target_vel, 
                                       const Eigen::Vector3d& target_acc, 
                                       double target_yaw) {
    last_sent_position_ = target_pos;
    last_sent_velocity_ = target_vel;
    last_sent_acceleration_ = target_acc;
    last_sent_yaw_ = target_yaw;
    
    if (fc_data_log_.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        double speed = target_vel.norm();
        fc_data_log_ << data_point_index_++ << " " << timestamp << " "
                    << target_pos.x() << " " << target_pos.y() << " " << target_pos.z() << " "
                    << target_vel.x() << " " << target_vel.y() << " " << target_vel.z() << " "
                    << speed << " " << target_yaw << " 0 " 
                    << target_acc.x() << " " << target_acc.y() << " " << target_acc.z() << std::endl;
    }
}

void RobotSimulator::receiveFromFlightControl(Eigen::Vector3d& feedback_pos, Eigen::Vector3d& feedback_vel, Eigen::Vector3d& feedback_acc, double& feedback_yaw) {
    if (use_fc_feedback_) {
        feedback_pos = last_sent_position_;
        feedback_vel = last_sent_velocity_;
        feedback_acc = last_sent_acceleration_;
        feedback_yaw = last_sent_yaw_;
    } else {
        feedback_pos = position_;
        feedback_vel = velocity_;
        feedback_acc = acceleration_;
        feedback_yaw = yaw_;
    }
}

/*
 * @Function:Obj Planner:Robot Simulator
 * @Author:Fancy
 * @Date:2026 01
 */

void RobotSimulator::updateStateFromTrajectory(const std::vector<obj_planner::PathPoint>& trajectory) {
    current_trajectory_ = trajectory;
}