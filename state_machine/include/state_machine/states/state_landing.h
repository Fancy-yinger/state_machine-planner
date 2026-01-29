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
 * @file state_landing.h
 * @brief [论文 3.3] 降落状态实现 (含可视化闭环)
 */

#ifndef STATE_LANDING_H
#define STATE_LANDING_H

#include <state_machine/state_base.h>
#include <mavros_msgs/SetMode.h>

namespace statemachine
{

class StateLanding : public StateBase
{
private:
    ros::Time last_req_time_; // 用于控制请求频率
    bool landing_started_;

public:
    StateLanding() : landing_started_(false) {}

    void enter(StateContext* ctx) override
    {
        ROS_INFO("[State: LANDING] Entering landing sequence...");
        landing_started_ = false;
        last_req_time_ = ros::Time::now();

        // 进入状态时立即尝试切换一次
        requestLandingMode(ctx);
    }

    std::string execute(StateContext* ctx) override
    {
        // 1. [核心] 维持可视化与状态更新
        // 如果不加这一行，降落过程的红线轨迹就不会画出来！
        if (ctx->egoplanner_wrapper) {
            ctx->egoplanner_wrapper->updateDroneState(*ctx->current_pose, *ctx->current_vel);
        }

        // 2. 检查并维持降落模式 (AUTO.LAND)
        if (ctx->current_state->mode != "AUTO.LAND") {
            // 每隔 1 秒尝试发送一次降落指令，防止指令丢失
            if ((ros::Time::now() - last_req_time_).toSec() > 1.0) {
                ROS_WARN("[State: LANDING] Mode is '%s'. Retrying AUTO.LAND...", ctx->current_state->mode.c_str());
                requestLandingMode(ctx);
                last_req_time_ = ros::Time::now();
            }
        } else {
            if (!landing_started_) {
                ROS_INFO("[State: LANDING] Vehicle in AUTO.LAND mode. Descending...");
                landing_started_ = true;
            }
        }

        // 3. 检查是否着陆并上锁 (Disarmed)
        // MAVROS 在检测到落地后会自动 Disarm，这是最准确的结束标志
        if (!ctx->current_state->armed) {
            ROS_INFO_THROTTLE(2.0, "[State: LANDING] Vehicle Disarmed. Mission Complete!");
            return "";  // 保持在此状态，任务完成
        }

        // 4. (可选) 高度监测日志
        if (ctx->current_pose->pose.position.z < 0.3 && ctx->current_state->armed) {
             ROS_INFO_THROTTLE(0.5, "[State: LANDING] Near ground (%.2fm)...", ctx->current_pose->pose.position.z);
        }

        return "";
    }

    void exit(StateContext* ctx) override {
        ROS_INFO("[State: LANDING] Exiting state.");
    }

private:
    // 封装原本 switch-case 中的服务调用逻辑
    void requestLandingMode(StateContext* ctx) {
        if (ctx->set_mode_client) {
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "AUTO.LAND";

            // 异步调用或快速调用，不要阻塞主循环太久
            if (ctx->set_mode_client->call(land_set_mode)) {
                if (land_set_mode.response.mode_sent) {
                    ROS_INFO("[State: LANDING] Set Mode AUTO.LAND Sent.");
                } else {
                    ROS_WARN("[State: LANDING] Set Mode AUTO.LAND Rejected by FCU.");
                }
            } else {
                ROS_ERROR("[State: LANDING] Failed to call SetMode service.");
            }
        }
    }
};

} // namespace statemachine

#endif
