cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Try to find Eigen3 or Eigen
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(../path_searching/include)
include_directories(../plan_env/include)
include_directories(include)
include_directories(/usr/include/eigen3)

add_library(    bspline_opt STATIC
                src/uniform_bspline.cpp
                src/bspline_optimizer.cpp
                src/gradient_descent_optimizer.cpp
 )

