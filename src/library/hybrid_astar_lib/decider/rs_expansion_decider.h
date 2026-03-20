#pragma once

#include "astar_decider.h"
#include "hierarchy_euler_distance_transform.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "pose2d.h"
#include "euler_distance_transform.h"
#include "./../node_collision_detect.h"

namespace planning {

#define RS_MAX_ROUND_ROBIN_NUMBER (3)

// todo: add all rs expansion related scenarios to this decider.
class RSExpansionDecider : public AstarDecider {
 public:
  RSExpansionDecider() = default;

  void Process(const Pose2f &start, const Pose2f &end) override;

  // round robin strategy for fallback point
  void UpdateRoundRobinStrategy(
      const Pose2f &end, const AstarRequest *request,
      HierarchyEulerDistanceTransform *edt, const VehicleParam &veh_param,
      std::shared_ptr<NodeCollisionDetect> &collision_detect);

  const Pose2f &GetRSEndPose();

  bool IsNeedRsExpansion(const Node3d *node, const AstarRequest *request) const;

  /**
   * [out] request: update next path rs info
   */
  static void UpdateRSPathRequest(AstarRequest *request);

  // for debug
  void GetRoundRobinTarget(std::vector<Pose2f> &candidates);

 private:
  const bool NeedRsLinkByNodeHeadingForTailIn(const Node3d *node) const;

  const bool NeedRsLinkByNodeHeadingForHeadIn(const Node3d *node) const;

  const bool NeedRsLinkByOffset(const Node3d *node) const;

  const bool NeedRsLinkByRequestDist(const Node3d *node,
                                     const AstarRequest *request) const;

  Pose2f GenerateCandidatePoint(const AstarRequest *request,
                                HierarchyEulerDistanceTransform *edt,
                                const VehicleParam &veh_param,
                                const Pose2f &end);

  void DebugDecider();

 private:
  Pose2f round_robin_end_[RS_MAX_ROUND_ROBIN_NUMBER];
  int round_robin_num_;
  int round_robin_id_;
};

}  // namespace planning