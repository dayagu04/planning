#pragma once

#include <cstddef>

#include "astar_decider.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {
constexpr double kXBoundLowerForHeadOut = 1.0;

struct NodeHeadingShrink {
  bool limit_search_heading_;
  float heading_low_bound_;
  float heading_up_bound_;
};

struct XCoordinateShrinkBound {
  float upper;
  float lower;
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

  bool IsLegalForHeading(const float heading);

  bool IsLegalForPos(const float x, const float y, const float x_limit,
                     const float y_limit);

  bool IsShrinkByParent(const Node3d *parent, const Node3d *child_node);

  bool IsShrinkByStartNode(const size_t start_id, Node3d *child);

  bool IsShrinkByGearSwitchNumber(Node3d *child);

  const bool IsLoopBackNode(const Node3d *new_node,
                            const Node3d *old_node) const;

  // Even if two nodes in same grid, and their grid id is same. But if they are
  // not continuous in pose, they are different node.
  const bool IsSameGridNodeContinuous(const Node3d *new_node,
                                      const Node3d *old_node) const;

  bool IsLegalByXBound(const float x);

 private:
  void ShrinkChildrenByHeadingForTailIn();

  void ShrinkChildrenByHeadingForHeadIn();

  // [0,+pi]
  NodeHeadingShrink heading_shrink_;

  ParkingVehDirection park_dir_;

  XCoordinateShrinkBound x_bound_;
};

}  // namespace planning