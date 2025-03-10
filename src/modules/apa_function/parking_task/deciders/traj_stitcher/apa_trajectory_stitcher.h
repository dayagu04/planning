#pragma once

#include "geometry_math.h"
#include "pose2d.h"
#include "src/library/reeds_shepp/rs_path_interpolate.h"
#include "trajectory/trajectory.h"

namespace planning {

// add trajectory stitcher.
// If do path/speed planning in every frame, need a stitcher.
// If navigation and parking switch in hpp, need trajectory stitcher for
// navigation trajectory and parking trajectory.
class ApaTrajectoryStitcher {
 public:
  ApaTrajectoryStitcher() = default;

  /**
   * [out]: trajectory
   * [in]: front_wheel_angle, left is positive
   */
  void Process(const Pose2D& ego_pose,
               const std::vector<pnc::geometry_lib::PathPoint>& path,
               const double ego_v, const double front_wheel_angle,
               trajectory::Trajectory* trajectory);

 private:
  // If vehicle speed is zero, use vehicle state to generate stitch point.
  void GeneTrajPointFromVehicleState(const Pose2D& ego_pose);

  // If history traj is null, and vehicle speed is not zero, need to generate
  // predict point.
  // ego_v: gear drive, v is positive
  // kappa: left is positive.
  Pose2D ComputeTrajPointByPrediction(const Pose2D& pose, const double ego_v,
                                      const double kappa);

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

  void GeneTrajPointFromPath(const pnc::geometry_lib::PathPoint& point);

  void ClearSticthPoint();

 private:
  TrajectoryPoint stitch_point_;
};
}  // namespace planning