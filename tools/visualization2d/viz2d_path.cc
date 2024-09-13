#include "viz2d_path.h"

#include "tools/visualization2d/opencv_viz.h"

namespace planning {

int viz2d_draw_global_trajectory(viz2d_image *viz2d,
                                 const PlanningOutput::PlanningOutput &planning,
                                 const Pose2D *base_pose,
                                 viz2d_color color_index,
                                 const Polygon2D *veh_local_polygon,
                                 const bool ref_pose_is_map_ref_frame) {
  int ret;
  CvPoint point1, point2;
  int i;

  CvScalar color;
  Polygon2D global_polygon;
  Polygon2D traj_local_polygon;
  traj_local_polygon = *veh_local_polygon;

  // 为了美观，不需要画出整个车体，画出车宽即可
  traj_local_polygon.vertexes[0].y = 0.05;
  traj_local_polygon.vertexes[1].y = 0.05;
  traj_local_polygon.vertexes[2].y = -0.05;
  traj_local_polygon.vertexes[3].y = -0.05;

  if (planning.trajectory().trajectory_points_size() <= 0 ||
      base_pose == nullptr || viz2d == nullptr) {
    return -1;
  }

  auto &traj = planning.trajectory();

  Pose2D local_pose, global_pose;

  ret = viz2d_get_color(&color, color_index);

  for (i = 0; i < traj.trajectory_points_size() - 1; i++) {
    global_pose.x = traj.trajectory_points(i).x();
    global_pose.y = traj.trajectory_points(i).y();
    global_pose.theta = traj.trajectory_points(i).heading_yaw();

    RULocalPolygonToGlobal(&global_polygon, &traj_local_polygon, &global_pose);

    viz2d_draw_filled_polygon(viz2d, &global_polygon, viz2d_colors_dodgerblue1,
                              base_pose, ref_pose_is_map_ref_frame);
  }

  for (i = 0; i < traj.trajectory_points_size() - 1; i++) {
    global_pose.x = traj.trajectory_points(i).x();
    global_pose.y = traj.trajectory_points(i).y();
    global_pose.theta = traj.trajectory_points(i).heading_yaw();

    if (ref_pose_is_map_ref_frame) {
      local_pose = global_pose;
    } else {
      CvtPoseGlobalToLocal(&local_pose, &global_pose, base_pose);
    }

    viz2d_get_index(viz2d, &point1, local_pose.x, local_pose.y);

    global_pose.x = traj.trajectory_points(i + 1).x();
    global_pose.y = traj.trajectory_points(i + 1).y();
    global_pose.theta = traj.trajectory_points(i + 1).heading_yaw();

    if (ref_pose_is_map_ref_frame) {
      local_pose = global_pose;
    } else {
      CvtPoseGlobalToLocal(&local_pose, &global_pose, base_pose);
    }

    viz2d_get_index(viz2d, &point2, local_pose.x, local_pose.y);

    cvLine(viz2d->image, point1, point2, color, 2, CV_AA, 0);
  }

  return 1;
}

int DrawHybridAstarPath(viz2d_image *viz2d,
                        const planning::common::PlanningDebugInfo &info,
                        const Pose2D *base_pose,
                        const bool ref_pose_is_map_ref_frame) {
  CvScalar color;
  CvScalar rs_color;
  viz2d_get_color(&color, viz2d_colors_green);
  viz2d_get_color(&rs_color, viz2d_colors_orange);

  CvPoint point1, point2;

  Position2D local, global;

  // std::cout << "ref line pt size " << ref_pts.size() << std::endl;

  for (int i = 0; i < info.refline_info_size() - 1; i++) {
    auto &start = info.refline_info(i);
    auto &end = info.refline_info(i + 1);

    global.x = start.x();
    global.y = start.y();

    if (ref_pose_is_map_ref_frame) {
      local = global;
    } else {
      CvtPosGlobalToLocal(&local, &global, base_pose);
    }

    viz2d_get_index(viz2d, &point1, local.x, local.y);

    // draw end
    global.x = end.x();
    global.y = end.y();

    if (ref_pose_is_map_ref_frame) {
      local = global;
    } else {
      CvtPosGlobalToLocal(&local, &global, base_pose);
    }

    viz2d_get_index(viz2d, &point2, local.x, local.y);
    if (start.l() > 0.0) {
      cvLine(viz2d->image, point1, point2, color, 1, CV_AA, 0);
    } else {
      cvLine(viz2d->image, point1, point2, rs_color, 1, CV_AA, 0);
    }
  }

  return 0;
}

}  // namespace planning