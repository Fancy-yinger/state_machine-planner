/****************************************************************************
 *
 *   Copyright (c) 2022-2025 Batuhan Yumurtaci. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file state_takeoff.h
 * @brief [论文 3.3] 起飞状态模板
 */

#ifndef STATE_TAKEOFF_H
#define STATE_TAKEOFF_H

#include <state_machine/state_base.h>
#include <cmath>

namespace statemachine
{

class StateTakeoff : public StateBase
{
private:
    double takeoff_height_;
    bool altitude_reached_;
    bool takeoff_called_;           // planner 服务调用标志
    ros::Time hover_start_time_;    // 悬停计时器
    bool hover_started_;            // 悬停开始标志

public:
    StateTakeoff() : takeoff_height_(1.0), altitude_reached_(false),
                     takeoff_called_(false), hover_started_(false) {}

    void enter(StateContext* ctx) override
    {
        ROS_INFO("[State: TAKEOFF] Initiating takeoff...");
        altitude_reached_ = false;
        takeoff_called_ = false;
        hover_started_ = false;

        // 设置起飞模式 (Motor on)
        std_msgs::Int32 msg;
        msg.data = 1; // FLIGHT
        ctx->actuator_control_pub->publish(msg);

        // 从参数服务器读取起飞高度
        ctx->nh->param(ros::this_node::getName() + "/takeoff/z", takeoff_height_, 1.0);
        ROS_INFO("[State: TAKEOFF] Target altitude: %.2f m", takeoff_height_);
    }

    std::string execute(StateContext* ctx) override
    {
        // 1. 更新 Wrapper 状态
        if (ctx->egoplanner_wrapper) {
            ctx->egoplanner_wrapper->updateDroneState(*ctx->current_pose, *ctx->current_vel);
        }

        // 2. 检查高度
        double current_z = ctx->current_pose->pose.position.z;
        bool altitudeReached = std::abs(current_z - takeoff_height_) < 0.2;

        // 检查是否停止
        Eigen::Vector3d vel(ctx->current_vel->twist.linear.x,
                           ctx->current_vel->twist.linear.y,
                           ctx->current_vel->twist.linear.z);
        bool stoppedWP = vel.norm() < 0.1;

        // 3. 每个周期都发布 controller_trigger 激活 geometric_controller
        if (ctx->controller_trigger_pub) {
            std_msgs::Bool trigger_msg;
            trigger_msg.data = true;
            ctx->controller_trigger_pub->publish(trigger_msg);
        }

        ROS_INFO_THROTTLE(1.0, "[State: TAKEOFF] Altitude: %.2f/%.2f m, Arrived: %d, Stopped: %d",
                          current_z, takeoff_height_, altitudeReached, stoppedWP);

        // 4. 垂直爬升逻辑
        if (altitudeReached && stoppedWP)
        {
            // 高度到达且已停止，开始悬停倒计时
            if (!hover_started_)
            {
                ROS_INFO("[State: TAKEOFF] Take-off completed! Hovering...");
                hover_start_time_ = ros::Time::now();
                hover_started_ = true;
            }

            // 悬停 1 秒后自动切换到 OBJ_PLANNER
            double hover_duration = (ros::Time::now() - hover_start_time_).toSec();
            if (hover_duration >= 1.0) {
                ROS_INFO("[State: TAKEOFF] Hover complete, transitioning to OBJ_PLANNER...");
                return "OBJ_PLANNER";
            }
        }
        else if (!altitudeReached && !takeoff_called_)
        {
            // 垂直爬升到目标高度（在当前位置）
            ROS_INFO("[State: TAKEOFF] Calling planner for vertical climb to %.2f m", takeoff_height_);

            planner::GetTrajectory takeoff_cmd;
            takeoff_cmd.request.type = 0;  // type=0 表示起飞
            takeoff_cmd.request.x = ctx->current_pose->pose.position.x;  // 当前X
            takeoff_cmd.request.y = ctx->current_pose->pose.position.y;  // 当前Y
            takeoff_cmd.request.z = takeoff_height_;                     // 目标Z
            takeoff_cmd.request.h = 0.0;  // 偏航角保持

            if (ctx->planner_client->call(takeoff_cmd))
            {
                if (takeoff_cmd.response.success)
                {
                    ROS_INFO("[State: TAKEOFF] Takeoff trajectory generated!");
                    takeoff_called_ = true;
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0, "[State: TAKEOFF] Waiting for trajectory generation...");
                }
            }
            else
            {
                ROS_ERROR("[State: TAKEOFF] Failed to call planner service!");
            }
        }

        return ""; // 保持当前状态
    }

    void exit(StateContext* ctx) override
    {
        ROS_INFO("[State: TAKEOFF] Takeoff complete.");
    }
};

} // namespace statemachine

#endif
