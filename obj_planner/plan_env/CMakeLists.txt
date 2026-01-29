
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")

# Try to find Eigen3 or Eigen
if(NOT Eigen3_FOUND)
  find_package(Eigen3 QUIET)
  if(NOT Eigen3_FOUND)
    find_package(Eigen REQUIRED)
    set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  endif()
endif()

include_directories(
    include/plan_env
    ${Eigen3_INCLUDE_DIRS}
    /usr/include/eigen3
)

add_library(
        local_map STATIC
        src/sdf_map.cpp
    )

