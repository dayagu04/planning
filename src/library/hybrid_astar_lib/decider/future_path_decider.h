#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include "astar_decider.h"
#include "euler_distance_transform.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "park_reference_line.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"
#include "hybrid_astar_request.h"

namespace planning {

// use bicycle model to inference next drive info.
struct InferenceDriveDist {
  // Eigen::Vector2d: the first is dist, the second is obstacle distance.
  std::vector<Eigen::Vector2d> gear_drive_path;
  std::vector<Eigen::Vector2d> gear_reverse_path;

  double dist_to_ref_line;
  // default drive distance is 1.2 meter, you can shrink it by obstacle.
  double advised_gear_drive_dist;
  double advised_gear_reverse_dist;
};

// decider: check path drive distance by obstacle distance check.
class FuturePathDecider : public AstarDecider {
 public:
  FuturePathDecider() = default;

  /**
   * [in]: ref_line, min_turn_radius, sampling_lon_resolution
   * [out]: edt, request
   */
  void Process(const ParkReferenceLine *ref_line, const double min_turn_radius,
               const double sampling_lon_resolution, EulerDistanceTransform *edt,
               AstarRequest &request);

  void Process(const Pose2D &start, const Pose2D &end);

  // if left, radius is positive
  void GetPathByRadius(const Pose2D *start_pose, const double length,
                       const double radius, const bool is_forward,
                       std::vector<Pose2D> *path);

 private:
  void CalcDriveDistByLineModel(const Pose2D &ego_pose,
                                EulerDistanceTransform *edt,
                                const ParkReferenceLine *ref_line);

  void UpdateFuturePathRequest(ParkFirstActionRequest *future_path_request);

  // radius: if left turn, radius is positive
  void GetVehCircleByPose(const Pose2D *pose, const double radius,
                          const AstarPathGear gear, VehicleCircle *veh_circle);

  // if left, radius is positive
  void GetPathByCircle(const Pose2D *start_pose, const double arc,
                       const double radius, const bool is_forward,
                       std::vector<Pose2D> *path);

  void GetPathByLine(const Pose2D *start_pose, const double length,
                     const bool is_forward, std::vector<Pose2D> *path);

  void GetStraightLinePoint(const Pose2D *start_state,
                            const double dist_to_start,
                            const Pose2D *unit_vector, Pose2D *goal_state);
  // arc is positive.
  // inverse_radius is positive
  void InterpolateByArcOffset(const VehicleCircle *veh_circle,
                              const Pose2D *start_pose, const double arc,
                              const double inverse_radius, Pose2D *pose);

  void CalcDriveDistByCircleModel(const Pose2D &ego_pose,
                                  EulerDistanceTransform *edt);

  void Clear();

  int32_t SearchIdByS(const double s, const std::vector<Eigen::Vector2d> &path);

  const double SearchAdvisedDriveDist(
      const double safe_buffer, const std::vector<Eigen::Vector2d> &path) const;

  void UpdatePathDistInfo(const std::vector<Pose2D> &path,
                          const AstarPathGear gear, EulerDistanceTransform *edt,
                          std::vector<Eigen::Vector2d> &path_dist_info);

 private:
  // use bicycle path safe distance as an heuristic info
  InferenceDriveDist future_drive_dist_info_;
  AstarPathGear gear_request_;
  double path_check_dist_;
  double min_turn_radius_;
  double point_resolution_;
  double sampling_lon_resolution_;
  double path_inference_lat_buffer_;

  bool swap_start_goal_;

  AstarPathGenerateType path_generate_type_;
};

}  // namespace planning