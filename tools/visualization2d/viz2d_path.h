#pragma once

#include "opencv_viz.h"
#include "planning_debug_info.pb.h"
#include "planning_plan.pb.h"

namespace planning {

int viz2d_draw_global_trajectory(viz2d_image *viz2d,
                                 const PlanningOutput::PlanningOutput &planning,
                                 const Pose2D *base_pose,
                                 viz2d_color color_index,
                                 const Polygon2D *veh_local_polygon,
                                 const bool ref_pose_is_map_ref_frame);

int DrawHybridAstarPath(viz2d_image *viz2d,
                        const planning::common::PlanningDebugInfo &info,
                        const Pose2D *base_pose,
                        const bool ref_pose_is_map_ref_frame);

}  // namespace planning