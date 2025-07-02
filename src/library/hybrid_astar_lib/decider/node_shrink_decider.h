#pragma once

#include <cstddef>

#include "astar_decider.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

struct NodeHeadingShrink {
  bool limit_search_heading_;
  double heading_low_bound_;
  double heading_up_bound_;
};

struct XCoordinateShrinkBound {
  double upper;
  double lower;
};

// in searching, we need shrink some nodes which are not necessary directions or
// gears for accelerate computation.
class NodeShrinkDecider : public AstarDecider {
 public:
  NodeShrinkDecider() = default;

  void Process(const Pose2D &start, const Pose2D &end) override;

  void Process(const Pose2D &start, const Pose2D &end,
               const ParkingVehDirection park_dir, const Pose2D &limiter_pose,
               const MapBound &XYbounds);

  bool IsLegalForHeading(const double heading);

  bool IsLegalForPos(const double x, const double y, const double x_limit,
                     const double y_limit);

  bool IsShrinkByParent(const Node3d *parent, const Node3d *child_node);

  bool IsShrinkByStartNode(const size_t start_id, Node3d *child);

  bool IsShrinkByGearSwitchNumber(Node3d *child);

  bool IsShrinkByHeadOutDirection(const AstarRequest &request,
                                  const Node3d *child);

  const bool IsLoopBackNode(const Node3d *new_node,
                            const Node3d *old_node) const;

  // Even if two nodes in same grid, and their grid id is same. But if they are
  // not continuous in pose, they are different node.
  const bool IsSameGridNodeContinuous(const Node3d *new_node,
                                      const Node3d *old_node) const;

  bool IsLegalByXBound(const double x);

 private:
  void ShrinkChildrenByHeadingForTailIn();

  void ShrinkChildrenByHeadingForHeadIn();

  // [0,+pi]
  NodeHeadingShrink heading_shrink_;

  ParkingVehDirection park_dir_;

  XCoordinateShrinkBound x_bound_;
};

}  // namespace planning