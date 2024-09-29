
#pragma once

#include <cmath>
#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "pose2d.h"
#include "reeds_shepp.h"
#include "rs_path_request.h"

namespace planning {

#define RS_PATH_MAX_POINT 256

struct RSPathSegment {
  AstarPathGear gear;
  RSPoint points[RS_PATH_MAX_POINT];
  int size;

  RSPathSteer steer;

  // signed length
  double length;

  // left is positive
  double kappa;
};

struct RSPath {
  // per rs segment is a path segment
  RSPathSegment paths[MAX_RS_PATH_NUM];
  int size;

  // always > 0
  double total_length;

  int gear_change_number;

  // todo
  int steering_change_number;

  double radius;

  void BackPoint(RSPoint *point) {
    if (size < 1) {
      return;
    }

    RSPathSegment *back_seg = &paths[size - 1];

    if (back_seg->size < 1) {
      return;
    }

    *point = back_seg->points[back_seg->size - 1];

    return;
  }

  void Clear() {
    size = 0;
    return;
  }
};

struct VehicleCircle {
  Position2D center;

  // left is positive
  double radius;
  AstarPathGear gear;
};

// if generate rs param by start pose and end pose, then use this interpolator
// to interpolate points.
class RSPathInterpolator {
 public:
  RSPathInterpolator() = default;

  /**
   * \brief Get reeds shepp control list.
   *        The unit of start_pose.pos and goal_pose.pos should be metres.
   *
   * \param[out]      kappa_list path segment kappa info;
   * \param[in]         start_pose start pose, unit(x, y, theta): m, m, rad;
   * \param[in]          goal_pose goal pose, unit(x, y, theta): m, m, rad;
   * \param[in]    min_turn_radius min turn radius of vehicle, unit: m;
   * \return 0 or error code.
   **/
  int CalcShortestRSPathKappa(RSPathKappaParam *kappa_list,
                              const Pose2D *start_pose, const Pose2D *goal_pose,
                              double min_turn_radius,
                              const double inverse_radius,
                              const RSPathRequestType request_type);

  int CalcSCSPathKappa(RSPathKappaParam *kappa_list, const Pose2D *start_pose,
                       const Pose2D *goal_pose, double min_turn_radius,
                       const double inverse_radius,
                       const RSPathRequestType request_type);

  int ShrinkRSPathByInvalidDist(RSPathKappaParam *kappa_list,
                                double dist_thres);

  int UpdateGearSwitchNum(RSPathKappaParam *kappa_list,
                          const AstarPathGear initial_pose_dir);

  int UpdateAnchorPoint(RSAnchorPoints *point_set, const Pose2D *start_pose,
                        const RSPathKappaParam *path_list);

  int UpdateAnchorPoint(RSAnchorPoints *point_set, const Pose2D *start_pose,
                        const RSPath *path);

  int RSPathSegmentInterpolate(RSPathSegment *path, const RSPoint *start_pose,
                               const RSSegmentKappaParam *path_kappa,
                               double inc_dist);

  int PathSegmentInterpolateByLine(RSPathSegment *path,
                                   const RSPoint *start_pose,
                                   const RSPoint *next_pose,
                                   const double path_kappa, const double length,
                                   const double inc_dist);

  int PathSegmentInterpolateByArc(RSPathSegment *path,
                                  const RSPoint *start_pose,
                                  const double path_kappa, const double length,
                                  const double inc_dist, const double radius,
                                  const double inv_radius);

 public:
  int CalcRSPathKappa(RSPathKappaParam *kappa_list, const RSPathParam *path,
                      double inverse_radius);

  int GetStraightLinePoint(RSPoint *goal_state, const RSPoint *start_state,
                           AstarPathGear direction, double length);

  int GetStraightLinePoint(RSPoint *goal_state, const RSPoint *start_state,
                           AstarPathGear direction, const double dist_to_start,
                           const Pose2D *unit_vector);

  int GetCirclePoint(RSPoint *goal_state, const RSPoint *start_state,
                     AstarPathGear direction, double length);

  int InterpolateByKappa(RSPoint *next_point, const RSPoint *point,
                         const AstarPathGear dir, const double abs_len);

  int CopyRSPathKappaParam(RSPathKappaParam *dst_list,
                           const RSPathKappaParam *src_list);

  AstarPathGear GetGearBySignLen(const double signed_length) {
    if (signed_length > 0.0) {
      return AstarPathGear::drive;
    } else if (signed_length < 0.0) {
      return AstarPathGear::reverse;
    }

    return AstarPathGear::none;
  }

  int GetVehCircleByPose(VehicleCircle *veh_circle, const RSPoint *pose,
                         double radius, AstarPathGear gear);

  int InterpolateByArcOffset(RSPoint *pose, const VehicleCircle *veh_circle,
                             const RSPoint *start_pose, const double arc,
                             const double inverse_radius);
};

}  // namespace planning
