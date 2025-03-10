#include "node_shrink_decider.h"
#include <algorithm>

#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {

void NodeShrinkDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  ShrinkChildrenByHeadingForTailIn();

  return;
}

void NodeShrinkDecider::Process(const Pose2D &start, const Pose2D &end,
                                const ParkingVehDirection park_dir) {
  AstarDecider::Process(start, end);

  heading_shrink_.limit_search_heading_ = false;
  if (park_dir == ParkingVehDirection::TAIL_IN) {
    ShrinkChildrenByHeadingForTailIn();
  } else if (park_dir == ParkingVehDirection::HEAD_IN) {
    ShrinkChildrenByHeadingForHeadIn();
  }

  return;
}

bool NodeShrinkDecider::IsLegalForHeading(const double heading) {
  if (!heading_shrink_.limit_search_heading_) {
    return true;
  }

  double normalize_heading = IflyUnifyTheta(heading, M_PI);
  normalize_heading = std::fabs(normalize_heading);

  if (normalize_heading < heading_shrink_.heading_low_bound_ ||
      normalize_heading > heading_shrink_.heading_up_bound_) {
    return false;
  }

  return true;
}

bool NodeShrinkDecider::IsLegalForPos(const double x, const double y,
                                      const double x_limit,
                                      const double y_limit) {
  if (std::fabs(y) < y_limit && x < x_limit) {
    return false;
  }
  return true;
}

void NodeShrinkDecider::ShrinkChildrenByHeadingForTailIn() {
  // heading shrink

  // 搜索时,heading 尽量不超过150度
  double heading_check_bound = ifly_deg2rad(150.0);
  double heading_buffer = ifly_deg2rad(20.0);

  heading_shrink_.limit_search_heading_ = true;

  heading_shrink_.heading_low_bound_ = 0.0;

  heading_shrink_.heading_up_bound_ =
      std::max(heading_check_bound, std::fabs(start_.theta) + heading_buffer);
  heading_shrink_.heading_up_bound_ =
      std::min(heading_shrink_.heading_up_bound_, M_PI);

  return;
}

void NodeShrinkDecider::ShrinkChildrenByHeadingForHeadIn() {
  // heading shrink
  double theta_diff = start_.theta - end_.theta;
  theta_diff = IflyUnifyTheta(theta_diff, M_PI);

  // 搜索时,heading 尽量大于30度. 且heading接近180度更好.
  double heading_check_bound = ifly_deg2rad(30.0);
  double heading_buffer = ifly_deg2rad(20.0);

  heading_shrink_.limit_search_heading_ = true;

  heading_shrink_.heading_low_bound_ =
      std::min(heading_check_bound, std::fabs(start_.theta) - heading_buffer);
  heading_shrink_.heading_low_bound_ =
      std::max(0.0, heading_shrink_.heading_low_bound_);

  heading_shrink_.heading_up_bound_ = M_PI;

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

bool NodeShrinkDecider::IsShrinkByStartNode(const size_t start_id,
                                            Node3d *child) {
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

const bool NodeShrinkDecider::IsLoopBackNode(const Node3d *new_node,
                                             const Node3d *old_node) const {
  if (new_node == nullptr || old_node == nullptr) {
    return false;
  }

  const Node3d *parent = new_node->GetPreNode();
  if (parent == nullptr) {
    return false;
  }

  const size_t new_node_id = new_node->GetGlobalID();
  for (size_t i = 0; i < 10000; i++) {
    if (parent->GetGlobalID() == new_node_id) {
      return true;
    }

    parent = parent->GetPreNode();
    if (parent == nullptr) {
      return false;
    }
  }

  return false;
}

const bool NodeShrinkDecider::IsSameGridNodeContinuous(
    const Node3d *new_node, const Node3d *old_node) const {
  if (new_node->GetEulerDist(old_node) > 0.01) {
    return false;
  }

  double theta_diff = std::fabs(new_node->GetPhi() - old_node->GetPhi());
  if (theta_diff > 0.0087) {
    return false;
  }

  return true;
}

}  // namespace planning