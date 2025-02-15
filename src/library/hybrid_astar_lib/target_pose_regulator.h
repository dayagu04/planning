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
  double dist_to_obs;
  double lat_offset;
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

  const Pose2D GetCandidatePose(const double lat_buffer) const;

  const bool IsCandidatePoseSafe(const double lat_buffer) const;

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
  const float GetDistToObs(Pose2D *global_pose, EulerDistanceTransform *edt);

  void DebugString();

 private:
  Pose2D center_line_target_;

  std::vector<PoseRegulateCandidate> candidate_info_;

  double x_check_upper_;
  double x_check_lower_;
  double x_step_;
  int x_sample_num_;
};

}  // namespace planning