#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

#include <memory>
#include <Eigen/Core>
#include "io_adapter.h"
#include "shared_data.h"

void ControlLoop(std::shared_ptr<IOAdapter> io,
                 std::shared_ptr<SharedContext> ctx,
                 Eigen::Vector3d goal_pos);

#endif