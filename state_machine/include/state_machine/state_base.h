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
 * @file state_base.h
 * @brief 定义状态机通用接口上下文与基类 (对应大论文 3.1 节)
 */

#ifndef STATE_BASE_H
#define STATE_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <planner/GetTrajectory.h>
#include <state_machine/objplanner_wrapper.h>
#include <memory>

namespace statemachine
{

// 前向声明，避免循环引用
class StateMachine;

/**
 * @brief [论文 3.1.2] 通用函数接口上下文 (StateContext)
 * 封装了状态所需的传感器数据、ROS接口和共享资源，实现状态与底层的解耦。
 * 目前使用指针引用 StateMachine 中的资源，未来可完全接管数据所有权。
 */
struct StateContext
{
    // --- 基础资源 ---
    ros::NodeHandle* nh;

    // --- 感知数据 (Sensory Data) ---
    geometry_msgs::PoseStamped* current_pose;
    geometry_msgs::TwistStamped* current_vel;
    mavros_msgs::State* current_state; // FCU 状态 (Armed/Offboard)

    // --- 任务数据 (Mission Data) ---
    geometry_msgs::PoseStamped* goal_pose;
    bool* goal_received;

    // --- 控制接口 (Control Interfaces) ---
    ros::Publisher* local_pos_pub;          // 位置设定点发布
    ros::Publisher* actuator_control_pub;   // 执行器控制
    ros::Publisher* flat_target_pub;        // 【Phase 3】轨迹设定点发布 (用于 ObjPlanner)
    ros::Publisher* controller_trigger_pub; // 【Phase 3.5】控制器触发器 (激活轨迹跟踪模式)
    ros::ServiceClient* planner_client;     // 规划器服务客户端
    ros::ServiceClient* set_mode_client;    // 【新增】模式切换客户端 (用于降落)

    // --- 算法模块 (Modules) ---
    std::shared_ptr<ObjPlannerWrapper> egoplanner_wrapper;

    // --- 键盘输入接口 (Keyboard Input Interface) ---
    std::function<int()> get_input;      // 获取输入值
    std::function<void(int)> set_input;  // 设置输入值
    std::function<void()> request_input; // 请求键盘输入

    // --- 辅助方法 (可扩展为论文中的标准接口函数) ---
    // 例如: void send_velocity_command(...)
};

/**
 * @brief [论文 3.1.3] 状态基类 (StateBase)
 * 定义状态的标准生命周期：进入(Enter) -> 执行(Execute) -> 退出(Exit)
 */
class StateBase
{
public:
    virtual ~StateBase() = default;

    /**
     * @brief 进入状态时调用一次
     * 用于初始化参数、重置计时器或打印日志
     */
    virtual void enter(StateContext* ctx) = 0;

    /**
     * @brief 每个控制周期调用 (20Hz)
     * 包含核心逻辑：检查跳转条件、计算控制量、发布指令
     * @return std::string 返回需要跳转到的下一个状态ID，若保持当前状态则返回空字符串
     */
    virtual std::string execute(StateContext* ctx) = 0;

    /**
     * @brief 退出状态时调用一次
     * 用于清理资源、停止计时器
     */
    virtual void exit(StateContext* ctx) = 0;
};

} // namespace statemachine

#endif
