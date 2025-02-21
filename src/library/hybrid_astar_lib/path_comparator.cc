#include "path_comparator.h"

#include <cmath>

#include "ad_common/math/math_utils.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "pose2d.h"

namespace planning {

#define DEBUG_DECIDER (0)

void PathComparator::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

bool PathComparator::Compare(const AstarRequest *request,
                             const Node3d *best_node,
                             const Node3d *node_challenger) {
#if DEBUG_DECIDER
  ILOG_INFO << "=========compare==========";
  best_node->DebugString();
  node_challenger->DebugString();
#endif

  // cost超出太多
  if (node_challenger->GetFCost() > best_node->GetFCost() + 10.0) {
    return false;
  }

  if (node_challenger->GetGearSwitchNum() < best_node->GetGearSwitchNum()) {
    return true;
  }

  // 无效node
  if (best_node->GetStepSize() < 1) {
    return true;
  }

  if (node_challenger->GetGearSwitchNum() == 0 &&
      best_node->GetGearSwitchNum() == 0) {
    return false;
  }

  // 换档次数一致，继续比较
  if (request->space_type == ParkSpaceType::VERTICAL) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      if (CheckVerticalSlotTailIn(request, best_node, node_challenger)) {
        return true;
      }
    } else if (request->direction_request == ParkingVehDirection::HEAD_IN) {
      if (CheckVerticalSlotHeadIn(request, best_node, node_challenger)) {
        return true;
      }
    }
  }

  return false;
}

bool PathComparator::CheckVerticalSlotTailIn(const AstarRequest *request,
                                             const Node3d *best_node,
                                             const Node3d *node_challenger) {
  // 为了耗时考虑，暂时只会比较第一个换挡点的cost.
  // 相同换档点，不需要比较
  if (best_node->GearSwitchNode() != nullptr &&
      node_challenger->GearSwitchNode() != nullptr) {
    if (best_node->GearSwitchNode()->GetGlobalID() ==
        node_challenger->GearSwitchNode()->GetGlobalID()) {
      return false;
    }
  }

  Pose2D gear_switch_pose_best;
  if (best_node->GearSwitchNode() != nullptr) {
    gear_switch_pose_best = best_node->GearSwitchNode()->GetPose();
  } else {
    gear_switch_pose_best = best_node->GetPose();
  }
  double heading_error_best = std::fabs(gear_switch_pose_best.theta);

  Pose2D gear_switch_pose_challenger;
  if (node_challenger->GearSwitchNode() != nullptr) {
    gear_switch_pose_challenger = node_challenger->GearSwitchNode()->GetPose();
  } else {
    gear_switch_pose_challenger = node_challenger->GetPose();
  }

  double heading_error_challenger =
      std::fabs(gear_switch_pose_challenger.theta);

#if DEBUG_DECIDER
  ILOG_INFO << "head 1 = " << heading_error_best * 57.4
            << ", head 2 = " << heading_error_challenger * 57.4;
#endif

  // 在换档次数一致，换挡点的坐标建议在这个区间。不在这个区间的，说明借用空间太深.
  double x_upper = 10.0;
  double x_lower = 4.0;
  if (gear_switch_pose_challenger.x < x_lower ||
      gear_switch_pose_challenger.x > x_upper) {
    return false;
  }

  // check heading
  if (heading_error_challenger < heading_error_best &&
      node_challenger->GetGCost() < best_node->GetGCost() + 4.0) {
    return true;
  }

  return false;
}

const bool PathComparator::PolynomialPathBetter(
    const PolynomialPathCost &path, const PolynomialPathCost &base) {
  if (std::fabs(path.tail_heading) < std::fabs(base.tail_heading)) {
    if (std::fabs(path.offset_to_center) <
        std::fabs(base.offset_to_center) + 0.01) {
      return true;
    }
  } else if (std::fabs(path.tail_heading) == std::fabs(base.tail_heading)) {
    if (std::fabs(path.offset_to_center) < std::fabs(base.offset_to_center)) {
      return true;
    } else if (std::fabs(path.offset_to_center) ==
               std::fabs(base.offset_to_center)) {
      if (path.accumulated_s < base.accumulated_s) {
        return true;
      }
    }
  }

  return false;
}


bool PathComparator::CheckVerticalSlotHeadIn(const AstarRequest *request,
                                             const Node3d *best_node,
                                             const Node3d *node_challenger) {
  // 为了耗时考虑，暂时只会比较第一个换挡点的cost.
  // 相同换档点，不需要比较
  if (best_node->GearSwitchNode() != nullptr &&
      node_challenger->GearSwitchNode() != nullptr) {
    if (best_node->GearSwitchNode()->GetGlobalID() ==
        node_challenger->GearSwitchNode()->GetGlobalID()) {
      return false;
    }
  }

  // best node pose
  Pose2D gear_switch_pose_best;
  if (best_node->GearSwitchNode() != nullptr) {
    gear_switch_pose_best = best_node->GearSwitchNode()->GetPose();
  } else {
    gear_switch_pose_best = best_node->GetPose();
  }
  double heading_error_best = ad_common::math::NormalizeAngle(
      gear_switch_pose_best.theta - request->real_goal.theta);
  heading_error_best = std::fabs(heading_error_best);

  // challenger pose
  Pose2D gear_switch_pose_challenger;
  if (node_challenger->GearSwitchNode() != nullptr) {
    gear_switch_pose_challenger = node_challenger->GearSwitchNode()->GetPose();
  } else {
    gear_switch_pose_challenger = node_challenger->GetPose();
  }

  double heading_error_challenger = ad_common::math::NormalizeAngle(
      gear_switch_pose_challenger.theta - request->real_goal.theta);
  heading_error_challenger = std::fabs(heading_error_challenger);

#if DEBUG_DECIDER
  ILOG_INFO << "head 1 = " << heading_error_best * 57.4
            << ", head 2 = " << heading_error_challenger * 57.4;
#endif

  // 在换档次数一致，换挡点的坐标建议在这个区间。不在这个区间的，说明借用空间太深.
  double x_upper = 12.0;
  double x_lower = 4.0;
  if (gear_switch_pose_challenger.x < x_lower ||
      gear_switch_pose_challenger.x > x_upper) {
    return false;
  }

  // check heading
  if (heading_error_challenger < heading_error_best &&
      node_challenger->GetGCost() < best_node->GetGCost() + 6.0) {
    return true;
  }

  return false;
}

}  // namespace planning