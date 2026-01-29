#ifndef PLANNING_THREAD_H
#define PLANNING_THREAD_H

#include <memory>
#include <vector>
#include "real_time_planner.h"
#include "shared_data.h"
#include "obstacle_types.h"
#include "io_adapter.h"

void PlanningLoop(std::shared_ptr<obj_planner::RealTimePlanner> planner,
                  std::shared_ptr<SharedContext> ctx,
                  std::shared_ptr<IOAdapter> io,
                  std::vector<obj_planner::PathPoint> global_path);

bool getStateFromTrajectory(const std::vector<obj_planner::PathPoint>& trajectory,
                            double t_rel,
                            Eigen::Vector3d& pos,
                            Eigen::Vector3d& vel,
                            Eigen::Vector3d& acc);
#endif