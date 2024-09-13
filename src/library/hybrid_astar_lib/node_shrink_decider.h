#pragma once

#include <cstddef>
#include "astar_decider.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

struct NodeHeadingShrink {
  bool limit_search_heading_;
  double heading_low_bound_;
  double heading_up_bound_;
};

// in searching, we need shrink some nodes which are not necessary directions or
// gears for accelerate computation.
class NodeShrinkDecider : public AstarDecider {
 public:
  NodeShrinkDecider() = default;

  void Process(const Pose2D &start, const Pose2D &end) override;

  bool IsLegalForHeading(const double heading);

  bool IsShrinkByParent(const Node3d *parent, const Node3d *child_node);

  bool IsShrinkByStartNode(const size_t start_id, Node3d *child);

  bool IsShrinkByGearSwitchNumber(Node3d *child);

 private:

  void ShrinkChildrenByHeading();

  // [-pi,+pi)
  NodeHeadingShrink heading_shrink_;
};

}  // namespace planning