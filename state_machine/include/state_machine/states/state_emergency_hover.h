/****************************************************************************
 *
 * Copyright (c) 2026 Emergency Response Module
 * Author: Fancy
 * Date: 2026 01
 *
 ****************************************************************************/
/**
 * @file state_emergency_hover.h
 * @brief [论文 3.2.3] 紧急悬停状态 (EMERGENCY_HOVER)
 * 实现系统的安全冗余：立即刹车 -> 保持位置 -> 尝试恢复
 */

#ifndef STATE_EMERGENCY_HOVER_H
#define STATE_EMERGENCY_HOVER_H

#include <state_machine/state_base.h>

namespace statemachine
{

class StateEmergencyHover : public StateBase
{
private:
    geometry_msgs::PoseStamped emergency_pose_; // 锁定的紧急悬停点
    ros::Time start_time_;                      // 进入状态的时间戳
    bool visualization_published_;              // 控制日志打印频率

public:
    StateEmergencyHover() : visualization_published_(false) {}

    void enter(StateContext* ctx) override
    {
        ROS_ERROR("[State: EMERGENCY_HOVER] !!! SAFETY TRIGGERED !!! Holding Position.");

        // 1. 立即锁定当前位置作为悬停点 (最快刹车方式)
        if (ctx->current_pose) {
            emergency_pose_ = *ctx->current_pose;
            emergency_pose_.header.frame_id = "world"; // 确保坐标系正确
        } else {
            ROS_FATAL("No pose available for emergency hover!");
            return;
        }

        start_time_ = ros::Time::now();
        visualization_published_ = false;

        // 2. 停止当前的轨迹规划（清除旧轨迹，防止冲突）
        if (ctx->egoplanner_wrapper) {
            ctx->egoplanner_wrapper->clearTrajectory();
            // 立即刷新一次状态，确保 RViz 显示正确
            ctx->egoplanner_wrapper->updateDroneState(*ctx->current_pose, *ctx->current_vel);
        }
    }

    std::string execute(StateContext* ctx) override
    {
        ros::Time now = ros::Time::now();

        // 1. [核心] 持续发布位置锁定指令 (位置控制模式)
        // 这会覆盖掉之前的速度或轨迹指令，强制拉住飞机
        if (ctx->local_pos_pub) {
            emergency_pose_.header.stamp = now;
            ctx->local_pos_pub->publish(emergency_pose_);
        }

        // 2. [态势感知] 即使在悬停，也要持续更新地图和障碍物
        // 否则如果在悬停期间障碍物移动了，恢复规划时会撞上去
        if (ctx->egoplanner_wrapper) {
            ctx->egoplanner_wrapper->updateDroneState(*ctx->current_pose, *ctx->current_vel);
            ctx->egoplanner_wrapper->updateSensor(); // 持续更新FOV感知

            // 降低可视化频率到 1Hz，节省资源
            if (!visualization_published_ || (now.toSec() - (int)now.toSec() < 0.05)) {
                ctx->egoplanner_wrapper->publishObstacleVisualization();
                visualization_published_ = true;
            }
        }

        // 3. [恢复策略] 自动恢复机制 (Auto-Recovery)
        double hover_duration = (now - start_time_).toSec();

        // 策略：悬停 5.0 秒
        // 这段时间给予规划器缓冲，也给予环境变化的可能（例如动态障碍物离开）
        if (hover_duration > 5.0) {
            ROS_WARN("[State: EMERGENCY_HOVER] Recovery timeout (5s). Retrying OBJ_PLANNER...");
            return "OBJ_PLANNER"; // <--- 尝试切回规划状态
        }

        return ""; // 继续悬停
    }

    void exit(StateContext* ctx) override
    {
        ROS_INFO("[State: EMERGENCY_HOVER] Exiting safety mode. Resuming mission.");
    }
};

} // namespace statemachine

#endif // STATE_EMERGENCY_HOVER_H
