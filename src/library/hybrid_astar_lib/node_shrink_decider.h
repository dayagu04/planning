#pragma once

#include "astar_decider.h"
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

 private:
  // [-pi,+pi)
  NodeHeadingShrink heading_shrink_;
};

}  // namespace planning