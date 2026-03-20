#pragma once
#include <iostream>
#include <memory>

#include "ad_common/math/vec2d.h"
#include "debug_mode.h"
#include "opencv_viz.h"
#include "pose2d.h"

namespace planning {
int viz2d_draw_local_dash_line(viz2d_image *viz2d, const Position2D *start,
                               const Position2D *end, viz2d_color color_index,
                               int width);

int viz2d_draw_polygon(viz2d_image *viz2d, const Polygon2D *poly,
                       const Pose2D *ref_pose, viz2d_color color_index);

int viz2d_draw_line(viz2d_image *viz2d, const Position2D *start,
                    const Position2D *end, const Pose2D *ref_pose,
                    viz2d_color color_index, int width);

int viz2d_draw_line(viz2d_image *viz2d, const Pose2D *start, const Pose2D *end,
                    const Pose2D *ref_pose, viz2d_color color_index, int width);

int viz2d_draw_direction(viz2d_image *viz2d, const Pose2D *start, double len,
                         const Pose2D *ref_pose, viz2d_color color_index,
                         int width);

int viz2d_draw_circle_wrapper(viz2d_image *viz2d,
                              const Position2D *circle_center,
                              const Pose2D *base_pose, viz2d_color color_index,
                              int radius, bool filled,
                              const bool ref_pose_is_map_ref_frame);

int viz2d_draw_local_line(viz2d_image *viz2d, const Position2D *start,
                          const Position2D *end, viz2d_color color_index,
                          int width);

int viz2d_draw_grid(viz2d_image *viz2d, double left, double right, double front,
                    double back, const Pose2D *base_pose,
                    viz2d_color color_index, int line_width);

int cv_draw_polygon(viz2d_image *viz2d, const Polygon2D *poly,
                    const Pose2D *ref_pose, viz2d_color color_index, int width,
                    const bool ref_pose_is_map_ref_frame);

int viz_draw_virtual_wall(Pose2D *wall_pose, viz2d_image *viz2d,
                          const Pose2D *veh_pose, int perception_id,
                          viz2d_color wall_color);

int viz_draw_box_in_cv_frame(viz2d_image *viz2d, const CvPoint *center,
                             double height, double length, viz2d_color color,
                             bool fill);

const int GenerateVirtualWall(std::vector<Polygon2D> &obs_list,
                              const double channel_length,
                              const double channel_width,
                              const double slot_width,
                              const double slot_length);

}  // namespace planning
