#include "node_shrink_decider.h"

#include "astar_decider.h"
#include "node3d.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {

void NodeShrinkDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  ShrinkChildrenByHeading();

  return;
}

bool NodeShrinkDecider::IsLegalForHeading(const double heading) {
  if (!heading_shrink_.limit_search_heading_) {
    return true;
  }

  double normalize_heading = IflyUnifyTheta(heading, M_PI);

  if (normalize_heading < heading_shrink_.heading_low_bound_ ||
      normalize_heading > heading_shrink_.heading_up_bound_) {
    return false;
  }

  return true;
}

void NodeShrinkDecider::ShrinkChildrenByHeading() {
  heading_shrink_.limit_search_heading_ = false;

  // heading shrink
  double theta_diff = start_.theta - end_.theta;
  theta_diff = IflyUnifyTheta(theta_diff, M_PI);

  double heading_check_bound = ifly_deg2rad(150.0);
  double heading_buffer = ifly_deg2rad(20.0);

  if (std::fabs(theta_diff) < heading_check_bound) {
    heading_shrink_.limit_search_heading_ = true;

    heading_shrink_.heading_low_bound_ =
        std::min(-heading_check_bound, start_.theta - heading_buffer);

    heading_shrink_.heading_up_bound_ =
        std::max(heading_check_bound, start_.theta + heading_buffer);
  }

  return;
}

bool NodeShrinkDecider::IsShrinkByParent(const Node3d *parent,
                                         const Node3d *child_node) {
  if (parent == nullptr || child_node == nullptr) {
    return false;
  }

  // child == parent
  if (child_node->GetGlobalID() == parent->GetGlobalID()) {
    return true;
  }

  // grandpa == child
  const Node3d *grandpa = parent->GetPreNode();

  if (grandpa != nullptr) {
    if (grandpa->GetGlobalID() == child_node->GetGlobalID()) {
      return true;
    }
  }

  return false;
}

bool NodeShrinkDecider::IsShrinkByStartNode(const size_t start_id, Node3d *child) {
  if (child->GetGlobalID() == start_id) {
    return true;
  }

  return false;
}

bool NodeShrinkDecider::IsShrinkByGearSwitchNumber(Node3d *child) {
  if (child->GetGearSwitchNum() > 20) {
    return true;
  }

  return false;
}

}  // namespace planning