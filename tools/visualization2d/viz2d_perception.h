#pragma once

#include "opencv_viz.h"
#include "parking_fusion.pb.h"
#include "pose2d.h"

namespace planning {

struct VirtualSlotSpace {
  Position2D right_up;
  Position2D right_lower;
  Position2D left_up;
  Position2D left_lower;
};

int draw_obstacle_points(const std::vector<Position2D> &obs_list,
                         const Pose2D &base_pose, viz2d_image *window,
                         const bool ref_pose_is_map_ref_frame);

int draw_ground_line_points(const std::vector<Position2D> &obs_list,
                            const Pose2D &base_pose, viz2d_image *window,
                            const viz2d_color color_index,
                            const bool ref_pose_is_map_ref_frame);

int draw_park_slot(const ParkingFusion::ParkingFusionInfo &slot_list,
                   const Pose2D &base_pose, viz2d_image *window,
                   const viz2d_color color_index,
                   const bool ref_pose_is_map_ref_frame);
}  // namespace planning