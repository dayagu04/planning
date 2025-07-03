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
  // Eigen::Vector2f: the first is dist, the second is obstacle distance.
  std::vector<Eigen::Vector2f> gear_drive_path;
  std::vector<Eigen::Vector2f> gear_reverse_path;

  float dist_to_ref_line;
  // default drive distance is 1.2 meter, you can shrink it by obstacle.
  float advised_gear_drive_dist;
  float advised_gear_reverse_dist;
};

// decider: check path drive distance by obstacle distance check.
class FuturePathDecider : public AstarDecider {
 public:
  FuturePathDecider() = default;

  /**
   * [in]: ref_line, min_turn_radius, sampling_lon_resolution
   * [out]: edt, request
   */
  void Process(const ParkReferenceLine *ref_line, const float min_turn_radius,
               const float sampling_lon_resolution, EulerDistanceTransform *edt,
               AstarRequest &request);

  void Process(const Pose2f &start, const Pose2f &end);

  // if left, radius is positive
  void GetPathByRadius(const Pose2f *start_pose, const float length,
                       const float radius, const bool is_forward,
                       std::vector<Pose2f> *path);

 private:
  void CalcDriveDistByLineModel(const Pose2f &ego_pose,
                                EulerDistanceTransform *edt,
                                const ParkReferenceLine *ref_line);

  void UpdateFuturePathRequest(ParkFirstActionRequest *future_path_request);

  // radius: if left turn, radius is positive
  void GetVehCircleByPose(const Pose2f *pose, const float radius,
                          const AstarPathGear gear, VehicleCircle *veh_circle);

  // if left, radius is positive
  void GetPathByCircle(const Pose2f *start_pose, const float arc,
                       const float radius, const bool is_forward,
                       std::vector<Pose2f> *path);

  void GetPathByLine(const Pose2f *start_pose, const float length,
                     const bool is_forward, std::vector<Pose2f> *path);

  void GetStraightLinePoint(const Pose2f *start_state,
                            const float dist_to_start,
                            const Pose2f *unit_vector, Pose2f *goal_state);
  // arc is positive.
  // inverse_radius is positive
  void InterpolateByArcOffset(const VehicleCircle *veh_circle,
                              const Pose2f *start_pose, const float arc,
                              const float inverse_radius, Pose2f *pose);

  void CalcDriveDistByCircleModel(const Pose2f &ego_pose,
                                  EulerDistanceTransform *edt);

  void Clear();

  int32_t SearchIdByS(const float s, const std::vector<Eigen::Vector2f> &path);

  const float SearchAdvisedDriveDist(
      const float safe_buffer, const std::vector<Eigen::Vector2f> &path) const;

  void UpdatePathDistInfo(const std::vector<Pose2f> &path,
                          const AstarPathGear gear, EulerDistanceTransform *edt,
                          std::vector<Eigen::Vector2f> &path_dist_info);

 private:
  // use bicycle path safe distance as an heuristic info
  InferenceDriveDist future_drive_dist_info_;
  AstarPathGear gear_request_;
  float path_check_dist_;
  float min_turn_radius_;
  float point_resolution_;
  float sampling_lon_resolution_;
  float path_inference_lat_buffer_;

  bool swap_start_goal_;

  AstarPathGenerateType path_generate_type_;
};

}  // namespace planning