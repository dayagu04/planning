
#include "rs_path_interpolate.h"

#include <cmath>

#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"
#include "reeds_shepp.h"

namespace planning {
#define DEBUG_RS_INTERPOLATE (0)

int RSPathInterpolator::CopyRSPathKappaParam(RSPathKappaParam *dst_list,
                                             const RSPathKappaParam *src_list) {
  int i;

  for (i = 0; i < src_list->size; i++) {
    dst_list->path_kappa[i] = src_list->path_kappa[i];
  }

  dst_list->size = src_list->size;

  return 1;
}

int RSPathInterpolator::CalcShortestRSPathKappa(
    RSPathKappaParam *kappa_list, const Pose2D *start_pose,
    const Pose2D *goal_pose, float min_turn_radius,
    const float inverse_radius, const RSPathRequestType request_type) {
  bool is_same;
  RSPathParam *path;
  RSPathInfo *path_info = GetRSPathGlobalInfo();

  if (kappa_list == nullptr || start_pose == nullptr || goal_pose == nullptr) {
    ILOG_INFO << "nullptr";

    return 0;
  }

  path = &path_info->path;
  is_same = start_pose->IsSame(goal_pose);
  if (is_same) {
    ILOG_INFO << "same point";

    if (!path_info->is_path_valid) {
      CalcRSPathKappa(&path_info->kappa_param, path, inverse_radius);

      path_info->is_path_valid = true;
    }
  } else {
    path->Clear();
    GetShortestRSPathParam(path, start_pose, goal_pose, min_turn_radius,
                           inverse_radius, request_type);

    // ILOG_INFO << " rs param success ";

    CalcRSPathKappa(&path_info->kappa_param, path, inverse_radius);

    // ILOG_INFO << " rs kappa success ";

    path_info->is_path_valid = true;

    path_info->start = *start_pose;
    path_info->end = *goal_pose;
    path_info->min_radius = min_turn_radius;
  }

  CopyRSPathKappaParam(kappa_list, &path_info->kappa_param);

  // ILOG_INFO << " len " << path_info->path.total_length;

  return 0;
}

int RSPathInterpolator::CalcSCSPathKappa(RSPathKappaParam *kappa_list,
                                         const Pose2D *start_pose,
                                         const Pose2D *goal_pose,
                                         float min_turn_radius,
                                         const float inverse_radius,
                                         const RSPathRequestType request_type) {
  bool is_same;
  RSPathParam *path;
  RSPathInfo *path_info = GetRSPathGlobalInfo();

  if (kappa_list == nullptr || start_pose == nullptr || goal_pose == nullptr) {
    ILOG_INFO << "nullptr";

    return 0;
  }

  path = &path_info->path;
  is_same = start_pose->IsSame(goal_pose);
  if (is_same) {
    ILOG_INFO << "same point";

    if (!path_info->is_path_valid) {
      CalcRSPathKappa(&path_info->kappa_param, path, inverse_radius);

      path_info->is_path_valid = true;
    }
  } else {
    path->Clear();
    TestSCS(path, start_pose, goal_pose, min_turn_radius, inverse_radius,
            request_type);

    // ILOG_INFO << " rs param success ";

    CalcRSPathKappa(&path_info->kappa_param, path, inverse_radius);

    // ILOG_INFO << " rs kappa success ";

    path_info->is_path_valid = true;

    path_info->start = *start_pose;
    path_info->end = *goal_pose;
    path_info->min_radius = min_turn_radius;
  }

  CopyRSPathKappaParam(kappa_list, &path_info->kappa_param);

  // ILOG_INFO << " len " << path_info->path.total_length;

  return 0;
}

int RSPathInterpolator::CalcRSPathKappa(RSPathKappaParam *kappa_list,
                                        const RSPathParam *path,
                                        float inverse_radius) {
  int i;

  if (kappa_list == nullptr || path == nullptr) {
    return 0;
  }

  kappa_list->size = 0;

  for (i = 0; i < MAX_RS_PATH_NUM; i++) {
    if (ifly_fgreater(std::fabs(path->length[i]), 0.0)) {
      kappa_list->path_kappa[kappa_list->size].length = path->length[i];
      kappa_list->path_kappa[kappa_list->size].steer_type = path->type[i];

      if (RS_LEFT == path->type[i]) {
        kappa_list->path_kappa[kappa_list->size].kappa = inverse_radius;
      } else if (RS_RIGHT == path->type[i]) {
        kappa_list->path_kappa[kappa_list->size].kappa = -inverse_radius;
      } else {
        kappa_list->path_kappa[kappa_list->size].kappa = 0.0;
      }

      kappa_list->size++;
    }
  }

#if DEBUG_RS_INTERPOLATE
  ILOG_INFO << "size " << kappa_list->size;

  for (i = 0; i < kappa_list->size; i++) {
    ILOG_INFO << "length: " << kappa_list->path_kappa[i].length
              << " ,kappa: " << kappa_list->path_kappa[i].kappa
              << " ,type: " << static_cast<int>(path->type[i]);
  }
#endif

  return 0;
}

int RSPathInterpolator::ShrinkRSPathByInvalidDist(RSPathKappaParam *kappa_list,
                                                  float dist_thres) {
  int i, j;

  for (i = 0, j = 0; i < kappa_list->size; i++) {
    if (ifly_fgreater(std::fabs(kappa_list->path_kappa[i].length),
                      dist_thres)) {
      if (j != i) {
        kappa_list->path_kappa[j] = kappa_list->path_kappa[i];
      }

      j++;
    }
  }

  kappa_list->size = j;

  return 1;
}

int RSPathInterpolator::GetStraightLinePoint(RSPoint *goal_state,
                                             const RSPoint *start_state,
                                             AstarPathGear direction,
                                             float length) {
  if (direction == AstarPathGear::DRIVE) {
    goal_state->x = start_state->x + length * ifly_cos(start_state->theta);
    goal_state->y = start_state->y + length * ifly_sin(start_state->theta);
  } else {
    goal_state->x = start_state->x - length * ifly_cos(start_state->theta);
    goal_state->y = start_state->y - length * ifly_sin(start_state->theta);
  }

  goal_state->theta = start_state->theta;
  goal_state->kappa = 0.0;
  goal_state->dir = direction;

  return 1;
}

int RSPathInterpolator::GetStraightLinePoint(RSPoint *goal_state,
                                             const RSPoint *start_state,
                                             AstarPathGear direction,
                                             const float dist_to_start,
                                             const Pose2D *unit_vector) {
  goal_state->x = start_state->x + dist_to_start * unit_vector->x;
  goal_state->y = start_state->y + dist_to_start * unit_vector->y;

  goal_state->theta = start_state->theta;
  goal_state->kappa = 0.0;
  goal_state->dir = direction;

  return 1;
}

int RSPathInterpolator::GetCirclePoint(RSPoint *goal_state,
                                       const RSPoint *start_state,
                                       AstarPathGear direction, float length) {
  float radius, theta;

  radius = 1.0 / start_state->kappa;
  if (direction == AstarPathGear::DRIVE) {
    theta = start_state->theta + length * start_state->kappa;
  } else {
    theta = start_state->theta - length * start_state->kappa;
  }

  goal_state->x = start_state->x +
                  radius * (-ifly_sin(start_state->theta) + ifly_sin(theta));

  goal_state->y = start_state->y +
                  radius * (ifly_cos(start_state->theta) - ifly_cos(theta));

  goal_state->theta = IflyUnifyTheta(theta, M_PI);
  goal_state->kappa = start_state->kappa;
  goal_state->dir = direction;

  return 1;
}

int RSPathInterpolator::InterpolateByKappa(RSPoint *next_point,
                                           const RSPoint *point,
                                           const AstarPathGear dir,
                                           const float abs_len) {
  if (!ifly_fequal(point->kappa, 0.0)) {
    GetCirclePoint(next_point, point, dir, abs_len);
  } else {
    GetStraightLinePoint(next_point, point, dir, abs_len);
  }

  return 1;
}

int RSPathInterpolator::UpdateGearSwitchNum(
    RSPathKappaParam *kappa_list, const AstarPathGear initial_pose_dir) {
  int i;
  AstarPathGear cur_dir, next_dir;
  const RSSegmentKappaParam *curr_kappa, *next_kappa;

  kappa_list->gear_change_num = 0;
  for (i = 0; i < kappa_list->size - 1; i++) {
    curr_kappa = &kappa_list->path_kappa[i];
    next_kappa = &kappa_list->path_kappa[i + 1];

    cur_dir = get_signed_segment_dir(curr_kappa->length);
    next_dir = get_signed_segment_dir(next_kappa->length);
    if (cur_dir != next_dir) {
      kappa_list->gear_change_num++;
    }
  }

  if (initial_pose_dir != AstarPathGear::NONE && kappa_list->size > 0) {
    curr_kappa = &kappa_list->path_kappa[0];
    cur_dir = get_signed_segment_dir(curr_kappa->length);

    if (initial_pose_dir != cur_dir) {
      kappa_list->gear_change_num++;
    }
  }

  return 1;
}

int RSPathInterpolator::UpdateAnchorPoint(RSAnchorPoints *point_set,
                                          const Pose2D *start_pose,
                                          const RSPathKappaParam *path_list) {
  RSPoint *cur_point, *next_point;
  int i;
  float abs_length, length, kappa;
  AstarPathGear dir;

  // update first anchor point
  point_set->size = 1;
  cur_point = &point_set->points[0];

  cur_point->x = start_pose->x;
  cur_point->y = start_pose->y;
  cur_point->theta = start_pose->theta;

  for (i = 0; i < path_list->size; i++) {
    length = path_list->path_kappa[i].length;
    abs_length = std::fabs(length);
    kappa = path_list->path_kappa[i].kappa;

    /* update current state if curvature discontinuity or
       or direction change
     */
    dir = GetGearBySignLen(length);
    cur_point->kappa = kappa;
    cur_point->dir = dir;

    next_point = &point_set->points[point_set->size];
    InterpolateByKappa(next_point, cur_point, dir, abs_length);

    cur_point = next_point;
    point_set->size++;
  }

  for (i = 0; i < point_set->size; i++) {
    point_set->points[i].theta =
        IflyUnifyTheta(point_set->points[i].theta, M_PI);
  }

#if 0

  ILOG_INFO << "path_list " << path_list->size;
  for (i = 0; i < path_list->size; i++) {
    ILOG_INFO << "k " << path_list->path_kappa[i].kappa << ", l "
              << path_list->path_kappa[i].length;
  }

  ILOG_INFO << "anchor point size:" << point_set->size;
  for (i = 0; i < point_set->size; i++) {
    ILOG_INFO << "i " << i << "," << point_set->points[i].x << " ,"
              << point_set->points[i].y << " ," << point_set->points[i].theta;
    }
#endif

  return 1;
}

int RSPathInterpolator::UpdateAnchorPoint(RSAnchorPoints *point_set,
                                          const Pose2D *start_pose,
                                          const RSPath *path) {
  RSPoint *cur_point, *next_point;
  int i;
  float abs_length, length, kappa;
  AstarPathGear dir;

  // update first anchor point
  point_set->size = 1;
  cur_point = &point_set->points[0];

  cur_point->x = start_pose->x;
  cur_point->y = start_pose->y;
  cur_point->theta = start_pose->theta;

  for (i = 0; i < path->size; i++) {
    length = path->paths[i].length;
    abs_length = std::fabs(length);
    kappa = path->paths[i].kappa;

    /* update current state if curvature discontinuity or
       or direction change
     */
    dir = GetGearBySignLen(length);
    cur_point->kappa = kappa;
    cur_point->dir = dir;

    next_point = &point_set->points[point_set->size];
    InterpolateByKappa(next_point, cur_point, dir, abs_length);

    cur_point = next_point;
    point_set->size++;
  }

  for (i = 0; i < point_set->size; i++) {
    point_set->points[i].theta =
        IflyUnifyTheta(point_set->points[i].theta, M_PI);
  }

#if 0

  ILOG_INFO << "path_list " << path->size;
  for (i = 0; i < path->size; i++) {
    ILOG_INFO << "k " << path->paths[i].length;
  }

  ILOG_INFO << "anchor point size:" << point_set->size;
  for (i = 0; i < point_set->size; i++) {
    ILOG_INFO << "i " << i << "," << point_set->points[i].x << " ,"
              << point_set->points[i].y << " ," << point_set->points[i].theta;
    }
#endif

  return 1;
}

int RSPathInterpolator::RSPathSegmentInterpolate(
    RSPathSegment *path, const RSPoint *start_pose,
    const RSSegmentKappaParam *path_kappa, float inc_dist) {
  RSPoint *state_curr, *state_next;
  int j, state_num;
  float abs_length, length, s_seg, integration_step;
  bool is_last;

  // update first point
  path->points[0].x = start_pose->x;
  path->points[0].y = start_pose->y;
  path->points[0].theta = start_pose->theta;
  path->points[0].dir = GetGearBySignLen(path_kappa->length);
  path->points[0].kappa = path_kappa->kappa;
  path->size = 1;

  state_curr = &path->points[0];

  length = path_kappa->length;
  abs_length = std::fabs(length);

  s_seg = 0.0;
  state_num = ifly_ceil(abs_length / inc_dist);
  is_last = false;

  for (j = 0; j < state_num; j++) {
    if (is_last) {
      break;
    }

    if (path->size >= RS_PATH_MAX_POINT) {
      ILOG_INFO << "rs path segment point size too big";

      break;
    }

    state_next = &path->points[path->size];

    s_seg += inc_dist;
    if (!ifly_fless(s_seg, abs_length)) {
      integration_step = inc_dist - (s_seg - abs_length);
      s_seg = abs_length;
      is_last = true;
    } else {
      integration_step = inc_dist;
    }

    InterpolateByKappa(state_next, state_curr, state_curr->dir,
                       integration_step);

    state_curr = state_next;

    path->size++;
  }

  // normalize angle
  for (j = 0; j < path->size; j++) {
    path->points[j].theta = IflyUnifyTheta(path->points[j].theta, M_PI);
  }

  return 0;
}

int RSPathInterpolator::PathSegmentInterpolateByLine(
    RSPathSegment *path, const RSPoint *start_pose, const RSPoint *next_pose,
    const float path_kappa, const float length, const float inc_dist) {
  RSPoint *state_next;
  int j, state_num;
  float abs_length, acc_s;
  bool is_last;

  // update first point
  path->points[0].x = start_pose->x;
  path->points[0].y = start_pose->y;
  path->points[0].theta = IflyUnifyTheta(start_pose->theta, M_PI);
  path->points[0].dir = GetGearBySignLen(length);
  path->points[0].kappa = path_kappa;
  path->size = 1;

  abs_length = std::fabs(length);

  acc_s = 0.0;
  state_num = ifly_ceil(abs_length / inc_dist);
  is_last = false;

  // get unit vector
  Pose2D unit_vector;
  unit_vector.x = (next_pose->x - start_pose->x) / abs_length;
  unit_vector.y = (next_pose->y - start_pose->y) / abs_length;

  // ILOG_INFO << "start " << start_pose->x << " " << start_pose->y;

  for (j = 0; j < state_num; j++) {
    if (is_last) {
      break;
    }

    if (path->size >= RS_PATH_MAX_POINT) {
      // ILOG_INFO << "rs path segment point size too big";

      break;
    }

    state_next = &path->points[path->size];

    acc_s += inc_dist;
    if (!ifly_fless(acc_s, abs_length)) {
      acc_s = abs_length;
      is_last = true;
    }

    GetStraightLinePoint(state_next, start_pose, path->points[0].dir, acc_s,
                         &unit_vector);

    // ILOG_INFO << "start " << state_next->x << " " << state_next->y << " acc_s
    // "
    //           << acc_s;

    path->size++;
  }

  return 0;
}

int RSPathInterpolator::PathSegmentInterpolateByArc(
    RSPathSegment *path, const RSPoint *start_pose, const float path_kappa,
    const float length, const float inc_dist, const float radius,
    const float inv_radius) {
  RSPoint *state_next;
  int j, state_num;
  float abs_length, acc_s;
  bool is_last;

  AstarPathGear gear = GetGearBySignLen(length);

  // update first point
  path->points[0].x = start_pose->x;
  path->points[0].y = start_pose->y;
  path->points[0].theta = start_pose->theta;
  path->points[0].dir = gear;
  path->points[0].kappa = path_kappa;
  path->size = 1;

  abs_length = std::fabs(length);

  acc_s = 0.0;
  state_num = ifly_ceil(abs_length / inc_dist);
  is_last = false;

  // get vehicle circle
  VehicleCircle veh_circle;
  float min_radius;
  if (path_kappa > 0.0) {
    min_radius = radius;
  } else {
    min_radius = -radius;
  }

  GetVehCircleByPose(&veh_circle, start_pose, min_radius, gear);

  float length_eps = 0.001;
  for (j = 0; j < state_num; j++) {
    if (is_last) {
      break;
    }

    if (path->size >= RS_PATH_MAX_POINT) {
      // ILOG_INFO << "rs path segment point size too big";

      break;
    }

    state_next = &path->points[path->size];
    state_next->dir = gear;
    state_next->kappa = path_kappa;

    acc_s += inc_dist;
    if (acc_s > (abs_length - length_eps)) {
      acc_s = abs_length;
      is_last = true;
    }

    InterpolateByArcOffset(state_next, &veh_circle, start_pose, acc_s,
                           inv_radius);

    path->size++;
  }

  return 0;
}

int RSPathInterpolator::GetVehCircleByPose(VehicleCircle *veh_circle,
                                           const RSPoint *pose, float radius,
                                           AstarPathGear gear) {
  veh_circle->radius = radius;
  veh_circle->gear = gear;

  veh_circle->center.x = pose->x - radius * std::sin(pose->theta);
  veh_circle->center.y = pose->y + radius * std::cos(pose->theta);

  return 1;
}

int RSPathInterpolator::InterpolateByArcOffset(RSPoint *pose,
                                               const VehicleCircle *veh_circle,
                                               const RSPoint *start_pose,
                                               const float arc,
                                               const float inverse_radius) {
  float delta_theta, theta;

  delta_theta = arc * inverse_radius;

  // left turn
  if (veh_circle->radius > 0.0) {
    if (veh_circle->gear == AstarPathGear::REVERSE) {
      delta_theta = -delta_theta;
    }
  } else {
    // right turn, gear is d
    if (veh_circle->gear == AstarPathGear::DRIVE) {
      delta_theta = -delta_theta;
    }
  }

  // update next point theta
  theta = start_pose->theta + delta_theta;

  float radius = veh_circle->radius;

  pose->x = veh_circle->center.x + radius * std::sin(theta);
  pose->y = veh_circle->center.y - radius * std::cos(theta);
  pose->theta = IflyUnifyTheta(theta, M_PI);

  return 1;
}

}  // namespace planning