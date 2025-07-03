#include "rs_expansion_decider.h"
#include <cmath>

#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"

namespace planning {

void RSExpansionDecider::Process(
    const float min_radius, const float slot_width, const float slot_length,
    const Pose2f &ego_pose, const Pose2f &astar_end, const float veh_width,
    const ParkSpaceType slot_type, const ParkingVehDirection park_dir) {
  rs_end_pose_ = astar_end;
  AstarDecider::Process(ego_pose, astar_end);

  if (slot_type == ParkSpaceType::PARALLEL) {
    return;
  }

  if (park_dir == ParkingVehDirection::HEAD_IN) {
    return;
  }

  return;
}

const Pose2f &RSExpansionDecider::GetRSEndPose() { return rs_end_pose_; }

bool RSExpansionDecider::IsNeedRsExpansion(const Node3d *node,
                                           const AstarRequest *request) const {
  if (request->space_type == ParkSpaceType::VERTICAL) {
    bool need_rs = false;
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      need_rs = NeedRsLinkByNodeHeadingForTailIn(node);
      if (!need_rs) {
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

void RSExpansionDecider::Process(const Pose2f &start, const Pose2f &end) {
  AstarDecider::Process(start, end);

  return;
}

void RSExpansionDecider::UpdateRSPathRequest(AstarRequest *request) {
  // check rs last path gear
  request->rs_request = RSPathRequestType::NONE;
  if (request->space_type == ParkSpaceType::VERTICAL) {
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
  // if heading > 150 degree, shrink some rs expansion.
  float heading = node->GetPhi();
  if (std::fabs(heading) > ifly_deg2rad(150.0)) {
    return false;
  }

  return true;
}

const bool RSExpansionDecider::NeedRsLinkByOffset(const Node3d *node) const {
  if (std::fabs(node->GetY()) > 10.0 || std::fabs(node->GetX()) > 15.0) {
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
  if (std::fabs(heading) < ifly_deg2rad(30.0)) {
    return false;
  }

  return true;
}

}  // namespace planning