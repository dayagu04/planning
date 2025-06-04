#pragma once

#include <cstddef>

#include "dp_speed_common.h"
#include "geometry_math.h"
#include "pose2d.h"
#include "speed/speed_data.h"
#include "src/library/reeds_shepp/rs_path_interpolate.h"
#include "trajectory/trajectory.h"

namespace planning {
namespace apa_planner {
// add trajectory stitcher.
// If do path/speed planning in every frame, need a stitcher.
// Caution:
// If car is overshoot in path terminal, define a proper stitching rule;
// If car is not reached the start point of a path, define a proper stitching
// rule;
class ApaTrajectoryStitcher {
 public:
  ApaTrajectoryStitcher() = default;

  /**
   * [out]: path
   * [in]: front_wheel_angle, left is positive
   * [in]: ego_v, (-inf, +inf)
   */
  void Execute(const Pose2D& ego_pose,
               const std::vector<pnc::geometry_lib::PathPoint>& path,
               const double front_wheel_angle, const SVPoint& ego_lon_point,
               const double predict_horizon,
               const trajectory::Trajectory& trajectory,
               const pnc::geometry_lib::PathSegGear gear);

  const std::vector<pnc::geometry_lib::PathPoint>& GetConstStitchPath() const;

  const trajectory::Trajectory& GetConstCombinedTraj() const;

  std::vector<pnc::geometry_lib::PathPoint>& GetMutableStitchPath();

  trajectory::Trajectory& GetCombinedTraj() { return trajectory_; }

  const double GetStitchPathLength() const;

  const pnc::geometry_lib::PathPoint& GetStitchPathPoint() const;

  const pnc::geometry_lib::PathPoint* GetStitchPathPointPtr();

  // based on time to compute path point.
  void CombineTrajBasedOnTime(const SpeedData& speed_profile);

  // based on path point to compute time.
  // TODO: complete it.
  void CombineTrajBasedOnPath();

  void TaskDebug(const std::vector<pnc::geometry_lib::PathPoint>& path,
                 const trajectory::Trajectory& trajectory);

  const SVPoint GetStitchSpeed() const;

 private:
  // If vehicle speed is zero, use vehicle state to generate stitch point.
  void GenePathPointFromVehicleState(const Pose2D& ego_pose);

  // If history traj is null, and vehicle speed is not zero, need to generate
  // predict point.
  // ego_v: gear drive, v is positive
  // kappa: left is positive.
  Pose2D ComputeTrajPointByPrediction(const Pose2D& pose, const double ego_v,
                                      const double kappa,
                                      const double predict_horizon);

  // move_dist is > 0
  void PredictByLine(const Pose2D& ego, const double move_dist,
                     const bool is_forward, Pose2D* new_pose);

  // radius: if left turn, radius is positive
  void GetVehCircleByPose(VehicleCircle* veh_circle, const Pose2D* pose,
                          const double radius, const AstarPathGear gear);

  // arc is positive.
  // inverse_radius is positive
  void GetCirclePoint(const VehicleCircle* veh_circle, const Pose2D* start_pose,
                      const double arc, const double inverse_radius,
                      Pose2D* pose);

  // if left, radius is positive. arc is > 0
  void PredictByCircle(const Pose2D& ego, const double arc, const double radius,
                       const bool is_forward, Pose2D* new_pose);

  bool QueryNearestPoint(const Pose2D& ego_pose,
                         const std::vector<pnc::geometry_lib::PathPoint>& path,
                         size_t* index) const;

  void ClearSticthPoint();

  bool QueryNearestPoint(const Pose2D& ego_pose,
                         const std::vector<pnc::geometry_lib::PathPoint>& path,
                         size_t* min_dist_point,
                         size_t* min_dist_point_neighbor);

  void EvaluateStitchPath(const Pose2D& ego_pose,
                          const std::vector<pnc::geometry_lib::PathPoint>& path,
                          const size_t min_dist_point_id,
                          const size_t min_dist_point_neighbor_id);

  void GeneSpeedPointFromVehicleState(const SVPoint& init_point);

 private:
  SVPoint ego_lon_state_;
  pnc::geometry_lib::PathPoint stitch_path_point_;
  std::vector<pnc::geometry_lib::PathPoint> stitch_path_;

  trajectory::TrajectoryPoint lon_stitch_point_;
  trajectory::Trajectory trajectory_;
  pnc::geometry_lib::PathSegGear gear_;
};
}  // namespace apa_planner
}  // namespace planning