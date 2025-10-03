#pragma once

#include <cstddef>

#include "astar_decider.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

struct NodeHeadingShrink {
  bool limit_search_heading_;
  // [0, Pi]
  float max_ref_line_heading_error;
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

  void Process(const Pose2f &start, const Pose2f &end) override;

  /**
   * ego: ego pose
   * end: pose decided by limiter pose
   */
  void Process(const Pose2f &ego, const Pose2f &end, const MapBound &XYbounds,
               const AstarRequest &request);

  bool IsShrinkByParent(const Node3d *parent, const Node3d *child_node);

  bool IsShrinkByStartNode(const int start_id, Node3d *child);

  bool IsShrinkByGearSwitchNumber(Node3d *child);

  bool IsShrinkByHeadOutDirection(const AstarRequest &request,
                                  const Node3d *child);

  const bool IsLoopBackNode(const Node3d *new_node,
                            const Node3d *old_node) const;

  // Even if two nodes in same grid, and their grid id is same. But if they are
  // not continuous in pose, they are different node.
  const bool IsSameGridNodeContinuous(const Node3d *new_node,
                                      const Node3d *old_node) const;

  bool IsLegalForPose(const Pose2f &pose);

 private:
  void UpdateHeadingErrorWithRefLine();

  bool IsLegalForHeading(const Pose2f &pose);

  bool IsLegalByXBound(const float x);

  // [0,+pi]
  NodeHeadingShrink heading_shrink_;

  // searching zone, shrink node
  XCoordinateShrinkBound x_bound_;

  // passage zone, if pose is in this zone, keep it. If pose is out of this
  // zone, check pose heading.
  MapBound passage_zone_;
};

}  // namespace planning