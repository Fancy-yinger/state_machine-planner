# ObjPlanner 集成到 State Machine

## 概述

本文档说明了如何将 ObjPlanner 实时轨迹规划器集成到现有的状态机中。

## 新增功能

### 1. 新状态：OBJ_PLANNER

在状态机中添加了新的状态 `OBJ_PLANNER` (状态码 = 7)，该状态：
- 接收目标点 (通过 `/planner/goal_pose` 话题)
- 从配置文件读取障碍物数据
- 执行实时轨迹规划
- 避障导航
- 到达目标后切换到降落状态

### 2. 核心组件

#### ObjPlannerWrapper
文件位置: `include/state_machine/objplanner_wrapper.h` 和 `src/objplanner_wrapper.cpp`

功能：
- 封装 ObjPlanner 的复杂接口
- 处理 ROS 消息与 ObjPlanner 数据格式之间的转换
- 管理规划器生命周期
- 提供简单的 API 供状态机调用

主要方法：
```cpp
// 初始化
bool init(double max_vel, double max_acc, double max_jerk,
          double map_size, double map_resolution, double map_inflate);

// 设置目标点
void setTarget(const geometry_msgs::PoseStamped& target);

// 更新无人机状态
void updateDroneState(const geometry_msgs::PoseStamped& pose,
                     const geometry_msgs::TwistStamped& vel);

// 更新障碍物（推荐方法 - 直接使用障碍物定义）
void updateObstacles(const std::vector<BoxObstacle>& box_obstacles,
                    const std::vector<SphereObstacle>& sphere_obstacles,
                    const Eigen::Vector3d& drone_pos);

// 执行轨迹规划
bool planTrajectory();

// 获取规划结果
std::vector<controller_msgs::FlatTarget> getControllerTrajectory() const;

// 检查是否到达目标
bool isGoalReached(double threshold = 0.3) const;
```

## 使用方法

### 1. 配置参数

在启动文件或参数服务器中设置以下参数：

```yaml
# ObjPlanner 参数
obj_planner:
  max_vel: 8.0          # 最大速度 (m/s)
  max_acc: 4.0          # 最大加速度 (m/s²)
  max_jerk: 2.0         # 最大加加速度 (m/s³)
  map_size: 100.0       # 地图大小 (m)
  map_resolution: 0.4   # 地图分辨率 (m)
  map_inflate: 0.2      # 障碍物膨胀距离 (m)

# 状态机参数
state_machine:
  perching_type: 5      # 5 = OBJ_PLANNER 状态
  wp_convergence_threshold: 0.3
  wp_stopped_threshold: 0.1
```

### 2. 启动状态机

```bash
roslaunch state_machine objplanner_state_machine.launch
```

或使用现有 launch 文件并设置 `perching_type=5`：
```bash
roslaunch state_machine sim_state_machine.launch perching_type:=5
```

### 3. 发送目标点

起飞完成后，通过 `/planner/goal_pose` 话题发布目标点：

```bash
# 示例：发送目标点
rostopic pub /planner/goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 10.0
    y: 10.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### 4. 障碍物数据

**重要**：ObjPlanner 直接从配置文件读取障碍物数据，**不需要点云数据**！

障碍物定义在 `launch/obstacles.yaml` 文件中：

```yaml
obstacles:
  # 长方体障碍物
  - type: box
    x: 2.0      # 中心X坐标 (米)
    y: 2.0      # 中心Y坐标 (米)
    z: 2.0      # 中心Z坐标 (米)
    width: 0.5  # X方向尺寸
    length: 0.5 # Y方向尺寸
    height: 2.0 # Z方向尺寸

  # 球形障碍物
  - type: sphere
    x: 4.0
    y: 3.0
    z: 1.5
    radius: 0.5
```

**坐标系说明**：
- 使用 ENU 坐标系 (East-North-Up)
- X轴：东，Y轴：北，Z轴：上
- 单位：米
- 原点：起飞位置

**注意**：这些障碍物与 `planner/src/planner.cpp` 中定义的障碍物应该保持一致！

## 状态转换流程

```
ARMED (解锁)
  ↓ [用户按下 ENTER]
TAKEOFF (起飞)
  ↓ [起飞完成，用户按下 ENTER, perching_type=5]
OBJ_PLANNER (实时规划)
  ↓ [接收目标点 → 规划轨迹 → 执行]
  ↓ [到达目标，用户按下 ENTER]
LANDING (降落)
  ↓ [降落完成]
结束
```

## 数据流

```
配置文件 (obstacles.yaml)
  ↓ [障碍物数据]
参数服务器
  ↓
State Machine
  ↓

目标点 (/planner/goal_pose)
  ↓
State Machine
  ↓
ObjPlannerWrapper
  ↓
ObjPlanner::PlannerInterface
  ↓
轨迹生成 (B-spline)
  ↓
转换为 FlatTarget 消息
  ↓
发布到 /geometric_controller/flat_target
  ↓
Geometric Controller
  ↓
MAVROS → PX4 飞控
```

## 关键文件

| 文件 | 说明 |
|------|------|
| `include/state_machine/objplanner_wrapper.h` | ObjPlanner 包装器头文件 |
| `src/objplanner_wrapper.cpp` | ObjPlanner 包装器实现 |
| `include/state_machine/state_machine.h` | 状态机头文件 (新增 OBJ_PLANNER 状态) |
| `src/state_machine.cpp` | 状态机实现 (新增 OBJ_PLANNER 逻辑) |
| `launch/objplanner_state_machine.launch` | ObjPlanner 专用启动文件 |
| `launch/obstacles.yaml` | 障碍物配置文件 |
| `CMakeLists.txt` | 构建配置 (已更新) |

## 调试和监控

### 查看日志
```bash
rosrun rqt_console rqt_console
```

### 查看状态
状态机会输出以下日志信息：
- `[OBJ_PLANNER]`: ObjPlanner 相关状态
- `[STATE]`: 通用状态信息
- `[OBJ_PLANNER_WRAPPER]`: 包装器内部状态

### 检查轨迹
在 RViz 中可视化：
- 轨迹点
- 障碍物
- 无人机位置
- 目标点

## 性能调优

### 提高规划速度
1. 增加 `map_resolution` (如 0.4 → 0.6)
2. 减小 `map_size` (如 100 → 50)
3. 减少障碍物采样率 (修改 `step` 参数)

### 提高轨迹质量
1. 增加 `max_vel`, `max_acc`, `max_jerk`
2. 减小 `map_resolution`
3. 增加障碍物采样精度

### 降低CPU使用率
1. 降低规划频率
2. 增加地图分辨率
3. 限制障碍物数量

## 故障排除

### 问题：规划失败
- 检查目标点是否在地图范围内
- 检查障碍物数据是否有效
- 增加 `map_inflate` 参数

### 问题：轨迹不平滑
- 增加 `map_resolution`
- 调整 `max_jerk` 参数
- 检查起点和终点状态是否合理

### 问题：无法到达目标
- 检查障碍物密度
- 增大地图范围
- 调整收敛阈值

## 注意事项

1. **坐标系统一性**：
   - 使用 **ENU坐标系** (East-North-Up)
   - X轴：东方向，Y轴：北方向，Z轴：向上
   - 所有位置数据必须使用相同坐标系
   - `planner`、`state_machine`、`obj_planner` 必须使用一致的坐标系

2. **障碍物数据一致性**：
   - `obstacles.yaml` 中的障碍物必须与 `planner/src/planner.cpp` 中的障碍物一致
   - 建议只在 `obstacles.yaml` 中维护障碍物定义
   - 可以考虑让planner也从该配置文件读取障碍物

3. **实时性**：规划频率建议设置为 2-5 Hz
4. **安全性**：在实际飞行前，先在仿真环境中充分测试

## 未来改进

- [ ] 支持动态目标点
- [ ] 添加轨迹预览和确认功能
- [ ] 优化障碍物处理算法
- [ ] 支持多个目标点队列
- [ ] 添加任务规划和重规划功能

## 联系方式

如有问题，请查看：
- ObjPlanner 原始文档
- 状态机原始文档
- GitHub Issues
