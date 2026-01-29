# Optimize the performance of the ESDF map update in sloth-core

# 提示词

角色: C++ ROS 开发人员 / 无人机规划算法工程师

任务: 优化 sloth-core 中 ESDF 地图更新的性能。目前地图更新耗时约 64ms（对于 10Hz 的循环来说太慢），主要原因是地图尺寸过大 (100x100x10m)。

目标: 将地图尺寸减小到 40m x 40m x 5m，从而将更新时间降低到 10ms 以下，同时确保代码能正确适配这些新的尺寸参数。

需修改的文件:

sloth-core/state_machine/launch/objplanner_state_machine.launch

sloth-core/state_machine/src/objplanner_wrapper.cpp

sloth-core/state_machine/include/state_machine/objplanner_wrapper.h

(可选) sloth-core/state_machine/src/state_machine_node.cpp (如果 init 函数的签名发生变化)

行动计划与要求:

修改 Launch 文件 (objplanner_state_machine.launch):

将 obj_planner_map_size 的默认值从 100.0 改为 40.0。

新增一个参数 obj_planner_map_height，默认值设为 5.0。

将这个新的 obj_planner/map_height 参数传递给 state_machine 节点。

更新 Wrapper 类 (objplanner_wrapper.cpp / .h):

在类成员中添加 double map_height_;。

更新 init() 方法，使其能够接收 map_height 作为参数（或者在方法内部从 ros::param 读取）。

在 init() 中，使用传入的值初始化 map_height_。

修复 initEsdfMap 调用: 将硬编码的 Z 轴尺寸 (10.0) 替换为 map_height_。

原代码: planner_->initEsdfMap(map_size_, map_size_, 10.0, ...)

新代码: planner_->initEsdfMap(map_size_, map_size_, map_height_, ...)

适配 planning_horizon (规划视界):

当前硬编码的 const double planning_horizon = 30.0; 对于 40m 的地图（半径 20m）来说太大了，会导致规划越界。

将其改为动态计算或更小的值：planning_horizon = std::min(30.0, map_size_ * 0.45);。

这能确保局部目标点始终位于已知地图范围内（例如对于 40m 地图，视界约为 18m）。

验证地图中心逻辑:

检查这行代码: Eigen::Vector3d map_center = drone_pos - Eigen::Vector3d(map_size_/2, map_size_/2, 2.0);

确认 2.0 (Z轴向下偏移量) 在地图高度为 5.0m 时是否仍然安全。（这意味着地图原点在无人机下方 2m。如果地图高 5m，则覆盖范围是无人机相对高度 [-2m, +3m]，这是合理的）。

更新调用处:

如果你修改了头文件中 ObjPlannerWrapper::init 的参数列表，请找到调用它的地方（通常在 state_machine_node.cpp 中），并传入从节点句柄获取的新参数 obj_planner_map_height。

输出: 请提供受影响文件的完整修改代码块（或 Unified Diff 格式）。
