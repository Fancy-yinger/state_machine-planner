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
 * @file state_obj_planner.h
 * @brief [论文 3.3] ObjPlanner 状态实现 (恢复滚动地图与可视化逻辑)
 */

#ifndef STATE_OBJ_PLANNER_H
#define STATE_OBJ_PLANNER_H

#include <state_machine/state_base.h>

namespace statemachine
{

class StateObjPlanner : public StateBase
{
private:
    ros::Time last_replan_time_;
    ros::Time traj_start_time_;
    bool has_active_traj_;
    bool goal_reached_;
    ros::Time finish_time_;  // 记录到达终点的时刻

    // 可视化频率控制
    ros::Time last_vis_time_;

    double replan_interval_ = 0.5; // 2Hz 重规划

public:
    StateObjPlanner()
        : last_replan_time_(0), has_active_traj_(false),
          goal_reached_(false), last_vis_time_(0) {}

    void enter(StateContext* ctx) override
    {
        ROS_INFO("[State: OBJ_PLANNER] Entering state...");

        if (!ctx->egoplanner_wrapper) {
            ROS_ERROR("Planner wrapper is null!");
            return;
        }

        // 1. 【找回逻辑】进状态时，强制刷新一次障碍物可视化
        // 这样 RViz 里才能立马看到红色的障碍物块
        ctx->egoplanner_wrapper->publishObstacleVisualization();

        // 2. 激活控制器
        if (ctx->controller_trigger_pub) {
            std_msgs::Bool trigger_msg;
            trigger_msg.data = true;
            ctx->controller_trigger_pub->publish(trigger_msg);
        }

        // 3. 重置变量
        goal_reached_ = false;
        has_active_traj_ = false;
        last_replan_time_ = ros::Time(0);
        finish_time_ = ros::Time(0);  // 重置完成时间
    }

    std::string execute(StateContext* ctx) override
    {
        if (!ctx->egoplanner_wrapper) return "";
        ros::Time now = ros::Time::now();

        // 1. 【核心】更新无人机状态
        // Wrapper 内部会检测：如果飞出中心区域，就移动地图 (Rolling Logic)
        ctx->egoplanner_wrapper->updateDroneState(*ctx->current_pose, *ctx->current_vel);

        // 2. 【新增】感知更新 - FOV 筛选 + 障碍物注入
        // 从全局障碍物库中筛选出传感器视野内的障碍物，并注入到规划器
        ctx->egoplanner_wrapper->updateSensor();

        // 3. 【找回逻辑】持续发布障碍物 (1Hz)
        // 之前可能是在主循环发的，现在必须在这里发，否则地图动了障碍物没动
        if ((now - last_vis_time_).toSec() > 1.0) {
            ctx->egoplanner_wrapper->publishObstacleVisualization();
            last_vis_time_ = now;
        }

        // 4. 终点判定
        if (ctx->egoplanner_wrapper->isGoalReached(0.5))
        {
            if (!goal_reached_) {
                ROS_INFO("[State: OBJ_PLANNER] Goal Reached! Hovering 3s before landing...");
                goal_reached_ = true;
                finish_time_ = ros::Time::now(); // 开始倒计时
            }

            // 【新增】悬停倒计时逻辑
            double hover_duration = (ros::Time::now() - finish_time_).toSec();
            if (hover_duration > 3.0) {
                ROS_INFO("[State: OBJ_PLANNER] Hover complete. Switching to LANDING.");
                return "LANDING"; // <--- 触发状态跳转的关键信号！
            }

            return ""; // 没到时间就继续悬停
        }

        // 5. 重规划逻辑
        bool time_to_replan = (now - last_replan_time_).toSec() >= replan_interval_;
        bool new_goal_received = *ctx->goal_received;

        if ((new_goal_received || time_to_replan) && !goal_reached_)
        {
            if(new_goal_received) {
                // Wrapper 会自动处理长距离目标，截取到 40m 地图范围内
                ctx->egoplanner_wrapper->setTarget(*ctx->goal_pose);
                *ctx->goal_received = false;
            }

            if (ctx->egoplanner_wrapper->planTrajectory())
            {
                traj_start_time_ = now;
                has_active_traj_ = true;
                last_replan_time_ = now;

                // 发布轨迹可视化
                ctx->egoplanner_wrapper->publishTrajectoryVisualization();
            }
        }

        // 6. 执行控制
        if (has_active_traj_)
        {
            double time_elapsed = (now - traj_start_time_).toSec();
            controller_msgs::FlatTarget target_msg;

            if (ctx->egoplanner_wrapper->getSetpointAtTime(time_elapsed, target_msg))
            {
                target_msg.header.stamp = now;
                target_msg.header.frame_id = "world";
                if (ctx->flat_target_pub) ctx->flat_target_pub->publish(target_msg);
            }
            else
            {
                has_active_traj_ = false;
            }
        }

        return "";
    }

    void exit(StateContext* ctx) override
    {
        has_active_traj_ = false;
    }
};

} // namespace statemachine

#endif
