#pragma once

#include "astar_decider.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

// todo: add all rs expansion related scenarios to this decider.
class RSExpansionDecider : public AstarDecider {
 public:
  RSExpansionDecider() = default;

  void Process(const float min_radius, const float slot_width,
               const float slot_length, const Pose2D &ego_pose,
               const Pose2D &astar_end, const float veh_width,
               const ParkSpaceType slot_type,
               const ParkingVehDirection park_dir);

  void Process(const Pose2D &start, const Pose2D &end) override;

  const float GetEndPointMaxDepth();

  const Pose2D &GetRSEndPose();

  const bool IsSameEndPointForRsWithAstar();

  bool IsNeedRsExpansion(const Node3d *node, const AstarRequest *request) const;

  /**
   * [out] request: update next path rs info
   */
  static void UpdateRSPathRequest(AstarRequest *request);

 private:
  // 对于车辆在ref line，需要注意
  const bool NeedRsLinkByNodeHeadingForTailIn(const Node3d *node) const;

  const bool NeedRsLinkByNodeHeadingForHeadIn(const Node3d *node) const;

  const bool NeedRsLinkByOffset(const Node3d *node) const;

  const bool NeedRsLinkByRequestDist(
    const Node3d *node, const AstarRequest *request) const;

  bool same_point_for_rs_with_astar_;

  float rs_end_max_depth_;
  Pose2D rs_end_pose_;
};

}  // namespace planning