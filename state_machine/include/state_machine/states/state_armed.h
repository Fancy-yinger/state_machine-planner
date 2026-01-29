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
 * @file state_armed.h
 * @brief [论文 3.3] 准备状态 (ARMED) 实现
 */

#ifndef STATE_ARMED_H
#define STATE_ARMED_H

#include <state_machine/state_base.h>

namespace statemachine
{

class StateArmed : public StateBase
{
private:
    bool waiting_for_takeoff_;  // 等待起飞命令标志

public:
    StateArmed() : waiting_for_takeoff_(false) {}

    void enter(StateContext* ctx) override
    {
        ROS_INFO("[State: ARMED] Entering armed state...");
        waiting_for_takeoff_ = false;

        // 重置输入状态
        if (ctx->set_input) {
            ctx->set_input(100);
        }
    }

    std::string execute(StateContext* ctx) override
    {
        // 检查是否已解锁
        if (ctx->current_state->armed)
        {
            // 已解锁，等待用户按下 ENTER 启动起飞
            if (!waiting_for_takeoff_) {
                ROS_INFO("[State: ARMED] Press ENTER for Take-off");
                waiting_for_takeoff_ = true;

                // 请求键盘输入
                if (ctx->request_input) {
                    ctx->request_input();
                }
            }

            // 检查用户输入
            if (ctx->get_input && ctx->get_input() == 0)
            {
                ROS_INFO("[State: ARMED] Take-off command received! Transitioning to TAKEOFF...");

                // 激活控制器
                if (ctx->controller_trigger_pub) {
                    std_msgs::Bool trigger_msg;
                    trigger_msg.data = true;
                    ctx->controller_trigger_pub->publish(trigger_msg);
                }

                return "TAKEOFF";  // 切换到起飞状态
            }
        }
        else
        {
            // 未解锁，显示正在解锁
            ROS_INFO_THROTTLE(2.0, "[State: ARMED] Arming...");

            // 保持输入为非零值
            if (ctx->set_input) {
                ctx->set_input(100);
            }
        }

        return ""; // 保持当前状态
    }

    void exit(StateContext* ctx) override
    {
        ROS_INFO("[State: ARMED] Exiting armed state.");
        waiting_for_takeoff_ = false;
    }
};

} // namespace statemachine

#endif
