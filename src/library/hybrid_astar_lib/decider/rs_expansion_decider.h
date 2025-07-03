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
               const float slot_length, const Pose2f &ego_pose,
               const Pose2f &astar_end, const float veh_width,
               const ParkSpaceType slot_type,
               const ParkingVehDirection park_dir);

  void Process(const Pose2f &start, const Pose2f &end) override;

  const Pose2f &GetRSEndPose();

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

  Pose2f rs_end_pose_;
};

}  // namespace planning