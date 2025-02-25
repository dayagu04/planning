#include "rs_expansion_decider.h"
#include <cmath>

#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"

namespace planning {

#define rs_end_point_buffer (0.2)

void RSExpansionDecider::Process(
    const double min_radius, const double slot_width, const double slot_length,
    const Pose2D &ego_pose, const Pose2D &astar_end, const double veh_width,
    const ParkSpaceType slot_type, const ParkingVehDirection park_dir) {
  same_point_for_rs_with_astar_ = true;
  rs_end_pose_ = astar_end;
  AstarDecider::Process(ego_pose, astar_end);

  if (slot_type == ParkSpaceType::PARALLEL) {
    return;
  }

  if (park_dir == ParkingVehDirection::HEAD_IN) {
    return;
  }

  double half_width = slot_width / 2.0;

  // check width
  if (min_radius <= half_width) {
    return;
  }

  // check ego position
  if (ego_pose.y > -half_width && ego_pose.y < half_width) {
    return;
  }

  double dist_to_left_bord = min_radius - half_width;
  rs_end_max_depth_ = std::sqrt(min_radius * min_radius -
                                dist_to_left_bord * dist_to_left_bord);

  rs_end_max_depth_ -= rs_end_point_buffer;

  double depth = slot_length - astar_end.x;
  if (depth > rs_end_max_depth_) {
    rs_end_pose_.x = slot_length - rs_end_max_depth_;

    same_point_for_rs_with_astar_ = false;
  }

  ILOG_INFO << "rs same point with astar: " << same_point_for_rs_with_astar_
            << " slot_length " << slot_length << " astar_end " << astar_end.x
            << " rs_end_max_depth_ " << rs_end_max_depth_ << " depth " << depth;

  return;
}

const double RSExpansionDecider::GetEndPointMaxDepth() {
  return rs_end_max_depth_;
}

const Pose2D &RSExpansionDecider::GetRSEndPose() { return rs_end_pose_; }

const bool RSExpansionDecider::IsSameEndPointForRsWithAstar() {
  return same_point_for_rs_with_astar_;
}

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

void RSExpansionDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

void RSExpansionDecider::UpdateRSPathRequest(AstarRequest *request) {
  // check rs last path gear
  request->rs_request = RSPathRequestType::none;
  if (request->space_type == ParkSpaceType::VERTICAL) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      request->rs_request = RSPathRequestType::last_path_forbid_forward;
      ILOG_INFO << "last rs path forbid forward";
    } else if (request->direction_request == ParkingVehDirection::HEAD_IN) {
      request->rs_request = RSPathRequestType::last_path_forbid_reverse;
      ILOG_INFO << "last rs path forbid reverse";
    }
  }

  return;
}

const bool RSExpansionDecider::NeedRsLinkByNodeHeadingForTailIn(
    const Node3d *node) const {
  // use heuristic rule to do rs path expansion
  // use node heading and steering to check.
  // if heading > 150 degree, shrink some rs expansion.
  double heading = node->GetPhi();
  if (std::fabs(heading) > ifly_deg2rad(150.0)) {
    return false;
  }

  // check node steering angle
  double check_heading_buffer = M_PI_2;
  if (node->GetGearType() == AstarPathGear::DRIVE) {
    // need turn right
    if (heading > check_heading_buffer && node->GetSteer() > 0.0) {
      return false;
    } else if (heading < -check_heading_buffer && node->GetSteer() < 0.0) {
      // need turn left
      return false;
    }
  } else if (node->GetGearType() == AstarPathGear::REVERSE) {
    // need turn left
    if (heading > check_heading_buffer && node->GetSteer() < 0.0) {
      return false;
    } else if (heading < -check_heading_buffer && node->GetSteer() > 0.0) {
      // need turn right
      return false;
    }
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
  // use node heading and steering to check.
  // if heading < 30 degree, shrink some rs expansion.
  double heading = node->GetPhi();
  if (std::fabs(heading) < ifly_deg2rad(30.0)) {
    return false;
  }

  return true;
}

}  // namespace planning