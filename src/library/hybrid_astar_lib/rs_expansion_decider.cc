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
    const Pose2D &ego_pose, const Pose2D &astar_end, const double veh_width) {
  same_point_for_rs_with_astar_ = true;
  rs_end_pose_ = astar_end;
  AstarDecider::Process(ego_pose, astar_end);

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

const bool RSExpansionDecider::IsSameEndPointForRsWithAtar() {
  return same_point_for_rs_with_astar_;
}

bool RSExpansionDecider::IsNeedRsExpansion(const Node3d *node) {
  bool need_rs = false;
  need_rs = NeedRsLinkByNodeHeading(node);
  if (!need_rs) {
    return false;
  }

  if (!NeedRsLinkByOffset(node)) {
    return false;
  }

  return true;
}

void RSExpansionDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

void RSExpansionDecider::UpdateRSPathRequest(
    RSPathRequestType *rs_request, const bool is_single_shot,
    const AstarPathGear single_shot_gear, const Pose2D &ego_pose,
    const double slot_width) {
  if (is_single_shot) {
    if (single_shot_gear == AstarPathGear::reverse) {
      *rs_request = RSPathRequestType::all_path_forbid_forward;
    } else if (single_shot_gear == AstarPathGear::drive) {
      *rs_request = RSPathRequestType::all_path_forbid_reverse;
    }
  } else {
    // check rs last path gear
    *rs_request = RSPathRequestType::none;
    double y_diff = std::fabs(ego_pose.y);
    if (y_diff < slot_width / 2 + 1.5) {
      *rs_request = RSPathRequestType::last_path_forbid_forward;

      ILOG_INFO << "last rs path forbid forward";
    }

    return;
  }
}

const bool RSExpansionDecider::NeedRsLinkByNodeHeading(const Node3d *node) {
  // use heuristic rule to do rs path expansion
  // use node heading and steering to check.
  // if heading > 150 degree, shrink some rs expansion.
  double heading = node->GetPhi();
  if (std::fabs(heading) > ifly_deg2rad(150.0)) {
    return false;
  }

  // check node steering angle
  double check_heading_buffer = M_PI_2;
  if (node->GetGearType() == AstarPathGear::drive) {
    // need turn right
    if (heading > check_heading_buffer && node->GetSteer() > 0.0) {
      return false;
    } else if (heading < -check_heading_buffer && node->GetSteer() < 0.0) {
      // need turn left
      return false;
    }
  } else if (node->GetGearType() == AstarPathGear::reverse) {
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

const bool RSExpansionDecider::NeedRsLinkByOffset(
    const Node3d *node) {
  if (std::fabs(node->GetY()) > 10.0 || std::fabs(node->GetX()) > 15.0) {
    return false;
  }

  return true;
}

}  // namespace planning