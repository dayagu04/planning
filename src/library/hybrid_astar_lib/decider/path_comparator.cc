#include "path_comparator.h"

#include <cmath>

#include "ad_common/math/math_utils.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "pose2d.h"
#include "vecf32.h"

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
  if (best_node->GetGearSwitchNum() <= 1) {
    ILOG_INFO << "=========compare==========";
    best_node->DebugString();
    node_challenger->DebugString();
  }
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

  request_ = request;
  // 换档次数一致，继续比较
  if (request->space_type == ParkSpaceType::VERTICAL) {
    switch (request->direction_request) {
      case ParkingVehDirection::TAIL_IN:
        if (CheckVerticalSlotTailIn(best_node, node_challenger)) {
          return true;
        }
        break;

      case ParkingVehDirection::HEAD_IN:
        if (CheckVerticalSlotHeadIn(best_node, node_challenger)) {
          return true;
        }
        break;
      case ParkingVehDirection::HEAD_OUT_TO_LEFT:
      case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
      case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
        if (CheckVerticalSlotHeadOut(request, best_node, node_challenger)) {
          return true;
        }
        break;

      default:
        break;
    }
  }

  return false;
}

bool PathComparator::CheckVerticalSlotTailIn(const Node3d *best_node,
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
  float heading_error_best = std::fabs(gear_switch_pose_best.theta);

  Pose2D gear_switch_pose_challenger;
  if (node_challenger->GearSwitchNode() != nullptr) {
    gear_switch_pose_challenger = node_challenger->GearSwitchNode()->GetPose();
  } else {
    gear_switch_pose_challenger = node_challenger->GetPose();
  }

  float heading_error_challenger = std::fabs(gear_switch_pose_challenger.theta);

#if DEBUG_DECIDER
  if (best_node->GetGearSwitchNum() <= 1) {
    ILOG_INFO << "head 1 = " << heading_error_best * 57.4
              << ", head 2 = " << heading_error_challenger * 57.4;
  }
#endif

  // 在换档次数一致，换挡点的坐标建议在这个区间。不在这个区间的，说明借用空间太深.
  float x_upper = 10.0;
  float x_lower = 4.0;
  if (gear_switch_pose_challenger.x < x_lower ||
      gear_switch_pose_challenger.x > x_upper) {
    return false;
  }

  // check heading
  if (heading_error_challenger > heading_error_best ||
      node_challenger->GetGCost() > best_node->GetGCost() + 4.0) {
    return false;
  }

  // check heuristic point projection.
  if (!CheckHeuristicPointIsNice(gear_switch_pose_best,
                                 gear_switch_pose_challenger)) {
    return false;
  }

  return true;
}

bool PathComparator::CheckVerticalSlotHeadOut(const AstarRequest *request,
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
  double heading_error_best =
      std::fabs(gear_switch_pose_best.theta - request->real_goal.theta);

  Pose2D gear_switch_pose_challenger;
  if (node_challenger->GearSwitchNode() != nullptr) {
    gear_switch_pose_challenger = node_challenger->GearSwitchNode()->GetPose();
  } else {
    gear_switch_pose_challenger = node_challenger->GetPose();
  }

  double heading_error_challenger =
      std::fabs(gear_switch_pose_challenger.theta - request->real_goal.theta);

#if DEBUG_DECIDER
  ILOG_INFO << "head 1 = " << heading_error_best * 57.4
            << ", head 2 = " << heading_error_challenger * 57.4;
#endif

  // 在换档次数一致，换挡点的坐标建议在这个区间。不在这个区间的，说明借用空间太深.
  float x_upper = 10.0;
  float x_lower = 4.0;
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

bool PathComparator::CheckVerticalSlotHeadIn(const Node3d *best_node,
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
  float best_s;
  if (best_node->GearSwitchNode() != nullptr) {
    gear_switch_pose_best = best_node->GearSwitchNode()->GetPose();
    best_s = best_node->GearSwitchNode()->GetDistToStart();
  } else {
    gear_switch_pose_best = best_node->GetPose();
    best_s = best_node->GetDistToStart();
  }
  float heading_error_best = ad_common::math::NormalizeAngle(
      gear_switch_pose_best.theta - heuristic_pose_.theta);
  heading_error_best = std::fabs(heading_error_best);

  // challenger pose
  Pose2D gear_switch_pose_challenger;
  float challenger_s;
  if (node_challenger->GearSwitchNode() != nullptr) {
    gear_switch_pose_challenger = node_challenger->GearSwitchNode()->GetPose();
    challenger_s = node_challenger->GearSwitchNode()->GetDistToStart();
  } else {
    gear_switch_pose_challenger = node_challenger->GetPose();
    challenger_s = node_challenger->GetDistToStart();
  }

  float heading_error_challenger = ad_common::math::NormalizeAngle(
      gear_switch_pose_challenger.theta - heuristic_pose_.theta);
  heading_error_challenger = std::fabs(heading_error_challenger);

#if DEBUG_DECIDER
  if (best_node->GetGearSwitchNum() <= 1) {
    ILOG_INFO << "best node heading = " << gear_switch_pose_best.theta * 57.4
              << ", y = " << gear_switch_pose_challenger.y
              << ", s = " << challenger_s << ", challenger heading = "
              << gear_switch_pose_challenger.theta * 57.4
              << ", y = " << gear_switch_pose_best.y << ", s = " << best_s;
  }
#endif

  // 在换档次数一致，换挡点的坐标建议在这个区间。不在这个区间的，说明借用空间太深.
  float x_upper = 12.0;
  float x_lower = 4.0;
  if (gear_switch_pose_challenger.x < x_lower ||
      gear_switch_pose_challenger.x > x_upper) {
    // ILOG_INFO << "boundary is big";
    return false;
  }

  // check heading
  if (heading_error_challenger > heading_error_best ||
      node_challenger->GetGCost() > best_node->GetGCost() + 4.0) {
    // ILOG_INFO << "g cost is big";
    return false;
  }

  // check heuristic point projection.
  if (!CheckHeuristicPointIsNice(gear_switch_pose_best,
                                 gear_switch_pose_challenger)) {
    return false;
  }

  // ILOG_INFO << "found better";

  return true;
}

const bool PathComparator::NodeCompare(const Pose2D &goal,
                                       const Node3d *best_node,
                                       const Node3d *node_challenger) {
  float dist1 = std::fabs(goal.y - best_node->GetPose().y);
  float dist2 = std::fabs(goal.y - node_challenger->GetPose().y);

  // 距离较近，比较heading
  const float dist_bound = 0.05;
  if (dist2 < dist_bound && dist1 < dist_bound) {
    float heading_error_challenger = ad_common::math::NormalizeAngle(
        node_challenger->GetPose().theta - goal.theta);
    heading_error_challenger = std::fabs(heading_error_challenger);

    float heading_error_best = ad_common::math::NormalizeAngle(
        best_node->GetPose().theta - goal.theta);
    heading_error_best = std::fabs(heading_error_best);
    if (heading_error_challenger < heading_error_best) {
      return true;
    }
  }
  // 距离有大于0.05的,比较距离即可
  else {
    // closer
    if (dist2 < dist1) {
      return true;
    }
  }

  return false;
}

void PathComparator::SetHeuristicPose(const AstarRequest &request) {
  heuristic_pose_.x = request.slot_length;
  heuristic_pose_.y = 0.0;
  heuristic_pose_.theta = request.real_goal.theta;

  return;
}

const float PathComparator::GetHeuristicPointDistance(const Pose2D &node) {
  Vec2df32 line_base = Vec2df32::CreateUnitVec2df32(node.theta);

  Vec2df32 line_project;
  line_project.set_x(heuristic_pose_.x - node.x);
  line_project.set_y(heuristic_pose_.y - node.y);

  return std::fabs(line_base.CrossProd(line_project));
}

const bool PathComparator::CheckHeuristicPointIsNice(
    const Pose2D &best_node, const Pose2D &node_challenger) {
  // check heuristic point projection.
  const float challenger_dist = GetHeuristicPointDistance(node_challenger);
  if (challenger_dist > 100.0) {
    return false;
  }

  const float best_dist = GetHeuristicPointDistance(best_node);
  if (challenger_dist > best_dist - 0.04) {
#if DEBUG_DECIDER
    ILOG_INFO << "challenger dist = " << challenger_dist
              << ", best dist = " << best_dist;
#endif
    return false;
  }

  return true;
}

const bool PathComparator::CheckDistanceRequest(
    const float &best_node_s, const float &node_challenger_s) {
  if (node_challenger_s > request_->first_action_request.dist_request &&
      best_node_s < request_->first_action_request.dist_request) {
    return true;
  }

  if (node_challenger_s < request_->first_action_request.dist_request &&
      best_node_s < request_->first_action_request.dist_request &&
      node_challenger_s > best_node_s) {
    return true;
  }

  return false;
}

}  // namespace planning