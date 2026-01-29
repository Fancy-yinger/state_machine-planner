cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

# 强制使用 C++14 以支持 Eigen 对齐
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# 添加编译选项，优化 Eigen 性能并打印详细警告
add_compile_options(-O3 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  controller_msgs
)

find_package(Eigen3 REQUIRED)
find_package(catkin_simple REQUIRED)

# Add obj_planner subdirectories to build the required libraries
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../obj_planner/plan_env ${CMAKE_CURRENT_BINARY_DIR}/obj_planner/plan_env)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../obj_planner/path_searching ${CMAKE_CURRENT_BINARY_DIR}/obj_planner/path_searching)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../obj_planner/bspline_opt ${CMAKE_CURRENT_BINARY_DIR}/obj_planner/bspline_opt)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ../obj_planner/plan_manage/include/plan_manage
  ../obj_planner/plan_manage/include
  ../obj_planner/plan_manage/src
  ../obj_planner/bspline_opt/include
  ../obj_planner/bspline_opt/src  # 添加源文件目录
  ../obj_planner/path_searching/include
  ../obj_planner/path_searching/src  # 添加源文件目录
  ../obj_planner/plan_env/include
  ../obj_planner/include
  /usr/include/eigen3
)

# --- 【关键修改】将核心算法源文件加入可执行文件 ---
add_executable(state_machine
  src/state_machine.cpp
  src/objplanner_wrapper.cpp
  src/state_machine_node.cpp

  # Plan Manage (直接编译)
  ../obj_planner/plan_manage/src/planner_interface.cpp
  ../obj_planner/plan_manage/src/planning_thread.cpp
  ../obj_planner/plan_manage/src/control_thread.cpp
  ../obj_planner/plan_manage/src/real_time_planner.cpp
  ../obj_planner/plan_manage/src/robot_simulator.cpp
  ../obj_planner/plan_manage/src/utils.cpp

  # BSpline Opt (直接编译，解决 crash 的关键!)
  ../obj_planner/bspline_opt/src/bspline_optimizer.cpp
  ../obj_planner/bspline_opt/src/gradient_descent_optimizer.cpp
  ../obj_planner/bspline_opt/src/uniform_bspline.cpp

  # Path Searching (直接编译)
  ../obj_planner/path_searching/src/dyn_a_star.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# --- 【关键修改】不再链接 bspline_opt 和 path_searching 库 ---
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  # bspline_opt     <-- 移除 (已包含源文件)
  # path_searching  <-- 移除 (已包含源文件)
  local_map          # 保留 plan_env (SDFMap)，因为它依赖 PCL，编译源文件比较麻烦
)

# cs_install()
# cs_export()
