#include "rs_expansion_decider.h"
#include <cmath>

#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"

namespace planning {

#define DEBUG_DECIDER (0)

void RSExpansionDecider::Process(const Pose2f &start,
                                 const Pose2f &end) {
  AstarDecider::Process(start, end);

  return;
}

void RSExpansionDecider::UpdateRoundRobinStrategy(
    const Pose2f &end, const AstarRequest *request, EulerDistanceTransform *edt,
    const VehicleParam &veh_param,
    std::shared_ptr<NodeCollisionDetect> &collision_detect) {
  if (request->swap_start_goal) {
    round_robin_num_ = 1;
    round_robin_id_ = 0;
    round_robin_end_[0] = end;
    return;
  }

  Pose2f pose;
  if (request->direction_request == ParkingVehDirection::TAIL_IN) {
    round_robin_num_ = 0;
    round_robin_id_ = 0;
    round_robin_end_[0] = end;
    round_robin_num_++;

    pose = GenerateCandidatePoint(request, edt, veh_param, end);
    if (!collision_detect->IsCircleFootPrintCollision(pose)) {
      round_robin_end_[1] = pose;
      round_robin_num_++;

      pose.x += 0.8f;
      if (!collision_detect->IsCircleFootPrintCollision(pose)) {
        round_robin_end_[2] = pose;
        round_robin_num_++;
      }
    }
  } else if (request->direction_request == ParkingVehDirection::HEAD_IN) {
    round_robin_num_ = 0;
    round_robin_id_ = 0;
    round_robin_end_[0] = end;
    round_robin_num_++;

    pose = end;
    pose.x += 0.8f;
    if (!collision_detect->IsCircleFootPrintCollision(pose)) {
      round_robin_end_[1] = pose;
      round_robin_num_++;
    }

    pose.x += 0.8f;
    if (!collision_detect->IsCircleFootPrintCollision(pose)) {
      round_robin_end_[2] = pose;
      round_robin_num_++;
    }
  } else {
    round_robin_num_ = 1;
    round_robin_id_ = 0;
    round_robin_end_[0] = end;
  }

#if DEBUG_DECIDER
  DebugDecider();
#endif

  return;
}

const Pose2f &RSExpansionDecider::GetRSEndPose() {
  round_robin_id_++;
  if (round_robin_id_ >= round_robin_num_) {
    round_robin_id_ = 0;
  }

  return round_robin_end_[round_robin_id_];
}

bool RSExpansionDecider::IsNeedRsExpansion(const Node3d *node,
                                           const AstarRequest *request) const {
  if (request->space_type == ParkSpaceType::VERTICAL ||
      request->space_type == ParkSpaceType::SLANTING) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      if (!NeedRsLinkByNodeHeadingForTailIn(node)) {
        return false;
      }
    } else if (request->direction_request == ParkingVehDirection::HEAD_IN) {
      if (!NeedRsLinkByNodeHeadingForHeadIn(node)) {
        return false;
      }
    }

    if (!NeedRsLinkByOffset(node)) {
      return false;
    }

    if (!NeedRsLinkByRequestDist(node, request)) {
      return false;
    }
  }

  return true;
}

void RSExpansionDecider::UpdateRSPathRequest(AstarRequest *request) {
  // check rs last path gear
  request->rs_request = RSPathRequestType::NONE;
  if (request->space_type == ParkSpaceType::VERTICAL ||
      request->space_type == ParkSpaceType::SLANTING) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      request->rs_request = RSPathRequestType::LAST_PATH_FORBID_FORWARD;
      ILOG_INFO << "last rs path forbid forward";
    } else if (request->direction_request == ParkingVehDirection::HEAD_IN) {
      request->rs_request = RSPathRequestType::LAST_PATH_FORBID_REVERSE;
      ILOG_INFO << "last rs path forbid reverse";
    }
  }

  return;
}

const bool RSExpansionDecider::NeedRsLinkByNodeHeadingForTailIn(
    const Node3d *node) const {
  // use heuristic rule to do rs path expansion
  // use node heading to check.
  // if heading > 90 degree, shrink some rs expansion.
  if (std::fabs(node->GetPhi()) > 1.57f) {
    return false;
  }

  // If node y offset is big, and node is not in passage zone, do not link to
  // goal.
  if (std::fabs(node->GetY()) > 3.0f && node->GetX() < 4.0f) {
    return false;
  }

  return true;
}

const bool RSExpansionDecider::NeedRsLinkByOffset(const Node3d *node) const {
  if (std::fabs(node->GetY()) > 6.0f || std::fabs(node->GetX()) > 15.0f) {
    return false;
  }

  return true;
}

const bool RSExpansionDecider::NeedRsLinkByRequestDist(
    const Node3d *node, const AstarRequest *request) const {
  if (request->first_action_request.has_request &&
      node->GetDistToStart() < request->first_action_request.dist_request) {
    return false;
  }

  return true;
}

const bool RSExpansionDecider::NeedRsLinkByNodeHeadingForHeadIn(
    const Node3d *node) const {
  // use heuristic rule to do rs path expansion
  // use node heading to check.
  // if heading < 30 degree, shrink some rs expansion.
  float heading = node->GetPhi();
  if (std::fabs(heading) < 0.523f) {
    return false;
  }

  return true;
}

Pose2f RSExpansionDecider::GenerateCandidatePoint(const AstarRequest *request,
                                                  EulerDistanceTransform *edt,
                                                  const VehicleParam &veh_param,
                                                  const Pose2f &end) {
  // mirror is in slot upper line.
  Pose2f candidate = end;
  candidate.x = request->slot_length - veh_param.mirror_lon_dist_to_rear_axle;
  // candidate must away from end.
  candidate.x = std::max(end.x + 0.4f, candidate.x);

  Pose2f left_mirror;
  left_mirror.x = candidate.x + veh_param.mirror_lon_dist_to_rear_axle;
  left_mirror.y = end.y + veh_param.mirror_lat_dist_to_center;
  left_mirror.theta = 0.0f;
  float radius = veh_param.mirror_width / 2.0;

  Pose2f right_mirror;
  right_mirror.x = left_mirror.x;
  right_mirror.y = end.y - veh_param.mirror_lat_dist_to_center;
  right_mirror.theta = 0.0f;

  // do search, find wide zone from slot upper line, max search distance is 1
  // meter. use mirror safe distance to search.
  // 1. [0.5, inf], break;
  // 2. [0,0.5], record max distance;
  float resolution = 0.1f;
  float max_search_dist = 1.0f;
  int size = std::ceil(max_search_dist / resolution);
  float safe_dist;
  float best_safe_dist = 0.0f;
  float best_x = 0;
  for (int i = 0; i < size; i++) {
    safe_dist = std::min(edt->DistanceCheckForPoint(left_mirror, radius),
                         edt->DistanceCheckForPoint(right_mirror, radius));

    if (safe_dist > 0.8f) {
      best_safe_dist = safe_dist;
      candidate.x = left_mirror.x - veh_param.mirror_lon_dist_to_rear_axle;
      break;
    }

    if (safe_dist > best_safe_dist) {
      best_safe_dist = safe_dist;
      candidate.x = left_mirror.x - veh_param.mirror_lon_dist_to_rear_axle;
    }

    left_mirror.x += resolution;
    right_mirror.x += resolution;
  }

  return candidate;
}

void RSExpansionDecider::DebugDecider() {
  for (int i = 0; i < round_robin_num_; i++) {
    round_robin_end_[i].DebugString();
  }

  return;
}

void RSExpansionDecider::GetRoundRobinTarget(std::vector<Pose2f> &candidates) {
  candidates.clear();
  candidates.reserve(std::min(round_robin_num_, RS_MAX_ROUND_ROBIN_NUMBER));
  for (int i = 0; i < std::min(round_robin_num_, RS_MAX_ROUND_ROBIN_NUMBER);
       i++) {
    candidates.emplace_back(round_robin_end_[i]);
  }
  return;
}

}  // namespace planning