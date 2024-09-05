#pragma once

#include <cstdint>
#include "common.pb.h"
#include "func_state_machine.pb.h"
#include "opencv_viz.h"

namespace planning {

int viz_func_state_machine(const FuncStateMachine::FuncStateMachine &state,
                           viz2d_image *viz2d, const Pose2D *veh_pose);

int viz2d_draw_chassis_feedback(viz2d_image *viz2d, const double v,
                                const double wheel, const uint32_t gear);

int viz2d_draw_control_commond_info(viz2d_image *viz2d, const double acc,
                                    const double wheel,
                                    Common::GearCommandValue control_gear);
}  // namespace planning
