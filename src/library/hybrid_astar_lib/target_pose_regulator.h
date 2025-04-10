#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "astar_decider.h"
#include "euler_distance_transform.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "park_reference_line.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"

namespace planning {

struct PoseRegulateCandidate {
  Pose2D pose;
  // 位姿对应的车辆真实外壳到障碍物的距离.
  float dist_to_obs;
  float lat_offset;
};

// 目标pose调节器.
// Todo: all target pose decisions should be moved to here.
// 1. Move goal to outside a few meters in vertical/parallel slot scene.
// 2. If origin goal is collided, move goal to left/right/top/bottom.
class TargetPoseRegulator : public AstarDecider {
 public:
  TargetPoseRegulator() = default;

  void Process(EulerDistanceTransform *edt, const AstarRequest *request,
               const Pose2D &ego_pose, const Pose2D &center_line_target,
               const VehicleParam &veh_param);

  void Process(const Pose2D &start, const Pose2D &end);

  void Clear();

  // Get most safe target pose
  const std::pair<Pose2D, float> GetCandidatePose(
      const float lat_buffer) const;

  const float GetEgoObsDist() const { return ego_dist_to_obs_; }

 private:
  const bool IsParkingIn(const AstarRequest *request);

  void GenerateCandidatesForVerticalSlot(EulerDistanceTransform *edt,
                                         const AstarRequest *request,
                                         const VehicleParam &veh_param);

  void GenerateCandidatesForParallelSlot(EulerDistanceTransform *edt,
                                         const AstarRequest *request,
                                         const VehicleParam &veh_param);

  // 检查目标点直线入库路径，和障碍物距离
  // return true: 直线路径没有障碍物
  const float GetDistToObs(const Pose2D *global_pose,
                           EulerDistanceTransform *edt);

  void DebugString();

  void UpdateDefaultPoseInfo(const AstarRequest *request,
                             const VehicleParam &veh_param,
                             EulerDistanceTransform *edt);

  bool IsDefaultPoseSafeEnough();

  const bool IsCandidatePoseSafe(const float lat_buffer) const;

  const std::pair<Pose2D, float> GetCandidatePoseForHeadIn(
      const float lat_buffer) const;

  // 0: none,
  // -1: left;
  // 1: right
  const int GenerateOffsetPreference() const;

  const PoseRegulateCandidate *GetCandidatePoseByOffset(const float lat_buffer,
                                                        const int offset) const;

  const std::pair<Pose2D, float> GetCandidatePoseForTailIn(
      const float lat_buffer) const;

 private:
  // decide by end straight distance
  Pose2D center_line_target_;
  std::vector<PoseRegulateCandidate> candidate_info_;
  const AstarRequest *request_;

  float x_check_upper_;
  float x_check_lower_;
  float x_step_;
  int x_sample_num_;

  float ego_dist_to_obs_;
};

}  // namespace planning