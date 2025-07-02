
#include "reeds_shepp_interface.h"

#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "log_glog.h"
#include "rs_path_interpolate.h"

namespace planning {

void RSPathInterface::GeneShortestRSPath(
    RSPath *rs_path, bool *is_connected_to_goal, const Pose2D *start,
    const Pose2D *end, const double min_radius, const bool need_interpolate,
    const bool need_anchor_point, const RSPathRequestType request_type,
    const double rs_path_sample_dist) {
  // init
  rs_path->size = 0;
  rs_path->total_length = 0.0;
  rs_path->radius = min_radius;

  int i;
  AstarPathGear veh_dir;
  RSPathKappaParam kappa_list;

  *is_connected_to_goal = false;

  /* when the two poses are equal, it is shown that the best node
     arrives to goal pose.
  */
  if (IsPointEqual(start, end)) {
    *is_connected_to_goal = true;

    ILOG_INFO << "same point";

    return;
  }

  double inverse_radius = 1.0 / min_radius;
  // generate shortest path
  rs_interpolate_.CalcShortestRSPathKappa(&kappa_list, start, end, min_radius,
                                          inverse_radius, request_type);

  // shrink path if distance is invalid.
  double shrink_dist = 0.001;
  rs_interpolate_.ShrinkRSPathByInvalidDist(&kappa_list, shrink_dist);

  // ILOG_INFO << "check gear";

  // check gear
  veh_dir = AstarPathGear::NONE;
  rs_interpolate_.UpdateGearSwitchNum(&kappa_list, veh_dir);

  if (kappa_list.gear_change_num > 8) {
    *is_connected_to_goal = false;

    ILOG_ERROR << "gear too much";
    return;
  }

  // update gear info
  rs_path->size = kappa_list.size;
  RSPathInfo *path_info = GetRSPathGlobalInfo();
  rs_path->total_length = path_info->path.total_length;

  for (i = 0; i < rs_path->size; i++) {
    rs_path->paths[i].gear =
        get_signed_segment_dir(kappa_list.path_kappa[i].length);
    rs_path->paths[i].length = kappa_list.path_kappa[i].length;
    rs_path->paths[i].kappa = kappa_list.path_kappa[i].kappa;
    rs_path->paths[i].size = 0;

    // update steer
    if (kappa_list.path_kappa[i].kappa > 0.0) {
      rs_path->paths[i].steer = RS_LEFT;
    } else if (kappa_list.path_kappa[i].kappa < 0.0) {
      rs_path->paths[i].steer = RS_RIGHT;
    } else {
      rs_path->paths[i].steer = RS_STRAIGHT;
    }
  }

  // just get anchor points
  if (need_anchor_point || need_interpolate) {
    rs_interpolate_.UpdateAnchorPoint(&rs_anchor_points_, start, &kappa_list);
    // ILOG_INFO << "update anchor point";

    // attention:
    // rs_anchor_points_.size = kappa_list.size + 1
  }

  if (need_interpolate) {
    UpdateRSPathByAnchorPoint(rs_path, min_radius);
  }

  rs_path->gear_change_number = kappa_list.gear_change_num;
  *is_connected_to_goal = true;
  // ILOG_INFO << "rs finish";

  return;
}

void RSPathInterface::GeneSCSPath(RSPath *rs_path, bool *is_connected_to_goal,
                                 const Pose2D *start, const Pose2D *end,
                                 const double min_radius,
                                 const RSPathRequestType request_type) {
  // init
  rs_path->size = 0;
  rs_path->total_length = 0.0;

  int i;
  AstarPathGear veh_dir;
  RSPathKappaParam kappa_list;

  *is_connected_to_goal = false;

  /* when the two poses are equal, it is shown that the best node
     arrives to goal pose.
  */
  if (IsPointEqual(start, end)) {
    *is_connected_to_goal = true;
    ILOG_INFO << "same point";

    return;
  }

  double inverse_radius = 1.0 / min_radius;
  // generate shortest path
  rs_interpolate_.CalcSCSPathKappa(&kappa_list, start, end, min_radius,
                                   inverse_radius, request_type);

  // shrink path if distance is invalid.
  double shrink_dist = 0.001;
  rs_interpolate_.ShrinkRSPathByInvalidDist(&kappa_list, shrink_dist);

  // ILOG_INFO << "check gear";

  // check gear
  veh_dir = AstarPathGear::NONE;
  rs_interpolate_.UpdateGearSwitchNum(&kappa_list, veh_dir);

  if (kappa_list.gear_change_num > 8) {
    *is_connected_to_goal = false;

    ILOG_ERROR << "gear too much";
    return;
  }

  // update gear info
  rs_path->size = kappa_list.size;
  RSPathInfo *path_info = GetRSPathGlobalInfo();
  rs_path->total_length = path_info->path.total_length;

  for (i = 0; i < rs_path->size; i++) {
    rs_path->paths[i].gear =
        get_signed_segment_dir(kappa_list.path_kappa[i].length);
    rs_path->paths[i].length = kappa_list.path_kappa[i].length;
    rs_path->paths[i].kappa = kappa_list.path_kappa[i].kappa;

    // update steer
    if (kappa_list.path_kappa[i].kappa > 0.0) {
      rs_path->paths[i].steer = RS_LEFT;
    } else if (kappa_list.path_kappa[i].kappa < 0.0) {
      rs_path->paths[i].steer = RS_RIGHT;
    } else {
      rs_path->paths[i].steer = RS_STRAIGHT;
    }
  }

  RSPathInterpolate(rs_path, start, min_radius);

  rs_path->gear_change_number = kappa_list.gear_change_num;
  *is_connected_to_goal = true;
  // ILOG_INFO << "rs finish";

  return;
}

void RSPathInterface::RSPathInterpolate(RSPath *rs_path, const Pose2D *start,
                                       const double min_radius) {
  // update anchor
  rs_interpolate_.UpdateAnchorPoint(&rs_anchor_points_, start, rs_path);
  // attention:
  // rs_anchor_points.size = kappa_list.size + 1

  UpdateRSPathByAnchorPoint(rs_path, min_radius);

  return;
}

void RSPathInterface::UpdateRSPathByAnchorPoint(RSPath *rs_path,
                                                const double min_radius) {
  double rs_path_sample_dist;
  RSPoint *anchor_point;
  double inverse_radius = 1.0 / min_radius;

  // interpolate
  rs_path_sample_dist = 0.1;
  for (int i = 0; i < rs_path->size; i++) {
    anchor_point = &rs_anchor_points_.points[i];

    // ILOG_INFO << "i " << i;
    // ILOG_INFO << "steer " << kappa_list.path_kappa[i].steer_type;

    // interpolate by line
    if (rs_path->paths[i].steer == RS_STRAIGHT) {
      rs_interpolate_.PathSegmentInterpolateByLine(
          &rs_path->paths[i], anchor_point, &rs_anchor_points_.points[i + 1],
          rs_path->paths[i].kappa, rs_path->paths[i].length,
          rs_path_sample_dist);
    } else {
      // interpolate by arc
      rs_interpolate_.PathSegmentInterpolateByArc(
          &rs_path->paths[i], anchor_point, rs_path->paths[i].kappa,
          rs_path->paths[i].length, rs_path_sample_dist, min_radius,
          inverse_radius);
    }
  }

  return;
}

}  // namespace planning