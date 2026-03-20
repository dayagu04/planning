#include "viz2d_perception.h"

#include <cstddef>
#include <vector>

#include "polygon_base.h"
#include "pose2d.h"
#include "tools/visualization2d/opencv_viz.h"
#include "viz2d_geometry.h"

namespace planning {

int draw_obstacle_points(const std::vector<Position2D> &obs_list,
                         const Pose2D &base_pose, viz2d_image *window,
                         const bool ref_pose_is_map_ref_frame) {
  for (size_t i = 0; i < obs_list.size(); i++) {
    const Position2D &center = obs_list[i];

    viz2d_draw_circle_wrapper(window, &center, &base_pose, viz2d_colors_red, 1,
                              true, ref_pose_is_map_ref_frame);
  }

  return 0;
}

int draw_ground_line_points(const std::vector<Position2D> &obs_list,
                            const Pose2D &base_pose, viz2d_image *window,
                            const viz2d_color color_index,
                            const bool ref_pose_is_map_ref_frame) {
  for (size_t i = 0; i < obs_list.size(); i++) {
    const Position2D &center = obs_list[i];

    viz2d_draw_circle_wrapper(window, &center, &base_pose, color_index, 2, true,
                              ref_pose_is_map_ref_frame);
  }

  return 0;
}

static int draw_slot_polygon(viz2d_image *viz2d, const Polygon2D *poly,
                             const Pose2D *ref_pose, viz2d_color color_index,
                             int width, const bool ref_pose_is_map_ref_frame) {
  int i;
  CvPoint pt1, pt2;
  CvScalar color;
  Position2D poly_pts[MAX_POLYGON_VERTEX_NUM];
  Position2D poly_pts_local[MAX_POLYGON_VERTEX_NUM];
  int ret;

  if (poly->vertex_num < 2) return 1;

  ret = viz2d_get_color(&color, color_index);

  for (i = 0; i < poly->vertex_num; i++) {
    poly_pts[i].x = poly->vertexes[i].x;
    poly_pts[i].y = poly->vertexes[i].y;

    if (ref_pose_is_map_ref_frame) {
      poly_pts_local[i] = poly_pts[i];
    } else {
      CvtPosGlobalToLocal(&poly_pts_local[i], &poly_pts[i], ref_pose);
    }
  }

  for (i = 0; i < poly->vertex_num; i++) {
    viz2d_get_index(viz2d, &pt1, poly_pts_local[i].x, poly_pts_local[i].y);

    switch (i) {
      case 0:
        viz2d_get_index(viz2d, &pt2, poly_pts_local[1].x, poly_pts_local[1].y);
        break;
      case 1:
        viz2d_get_index(viz2d, &pt2, poly_pts_local[3].x, poly_pts_local[3].y);
        break;
      case 2:
        viz2d_get_index(viz2d, &pt2, poly_pts_local[0].x, poly_pts_local[0].y);
        break;
      case 3:
        viz2d_get_index(viz2d, &pt2, poly_pts_local[2].x, poly_pts_local[2].y);
        break;
      default:
        break;
    }

    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
  }

  return 1;
}

int draw_park_slot(const ParkingFusion::ParkingFusionInfo &slot_list,
                   const Pose2D &base_pose, viz2d_image *window,
                   const viz2d_color color_index,
                   const bool ref_pose_is_map_ref_frame) {
  Polygon2D polygon;

  for (int i = 0; i < slot_list.parking_fusion_slot_lists_size(); i++) {
    auto &tmp = slot_list.parking_fusion_slot_lists(i);

    polygon.vertex_num = tmp.corner_points_size();

    for (int j = 0; j < tmp.corner_points_size(); j++) {
      auto &point = tmp.corner_points(j);

      polygon.vertexes[j].x = point.x();
      polygon.vertexes[j].y = point.y();
    }

    draw_slot_polygon(window, &polygon, &base_pose, color_index, 2,
                      ref_pose_is_map_ref_frame);
  }

  return 0;
}

}  // namespace planning