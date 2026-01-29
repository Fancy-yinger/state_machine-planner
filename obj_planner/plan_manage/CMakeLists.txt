
# Try to find Eigen3 or Eigen
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(include)
include_directories(../bspline_opt/include)
include_directories(../path_searching/include)
include_directories(../plan_env/include)
include_directories(./include/plan_manage)
include_directories(../include)
include_directories(/usr/include/eigen3)

link_directories(/usr/include/python2.7)

add_library(    obj_planner  STATIC
                src/planner_interface.cpp
                src/real_time_planner.cpp
                src/robot_simulator.cpp
                src/control_thread.cpp
                src/planning_thread.cpp
                src/utils.cpp
)

target_link_libraries(obj_planner
                      bspline_opt
                       path_searching
                       local_map)


