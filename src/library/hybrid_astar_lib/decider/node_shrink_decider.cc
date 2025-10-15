#include "node_shrink_decider.h"

#include <algorithm>
#include <cmath>

#include "astar_decider.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "node3d.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {

void NodeShrinkDecider::Process(const Pose2f &start, const Pose2f &end) {
  AstarDecider::Process(start, end);

  return;
}

void NodeShrinkDecider::Process(const Pose2f &ego, const Pose2f &end,
                                const MapBound &XYbounds,
                                const AstarRequest &request) {
  AstarDecider::Process(ego, end);

  // limit heading
  heading_shrink_.limit_search_heading_ = false;
  if (request.direction_request == ParkingVehDirection::TAIL_IN ||
      request.direction_request == ParkingVehDirection::HEAD_IN) {
    UpdateHeadingErrorWithRefLine();
  }

  // limit x position
  x_bound_.upper = XYbounds.x_max;
  constexpr float kXBoundLowerForHeadOut = 0.0f;

  switch (request.direction_request) {
    case ParkingVehDirection::HEAD_OUT_TO_LEFT:
    case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
    case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
    case ParkingVehDirection::TAIL_OUT_TO_LEFT:
    case ParkingVehDirection::TAIL_OUT_TO_MIDDLE:
    case ParkingVehDirection::TAIL_OUT_TO_RIGHT:
      x_bound_.lower = std::min(kXBoundLowerForHeadOut, ego.x - 0.1f);
      break;
    default:
      x_bound_.lower = std::min(end.x + 0.4f, ego.x - 0.1f);
      break;
  }

  // limit heading
  passage_zone_ =
      MapBound(request.slot_length - 1.0f, request.slot_length + 6.0f,
               XYbounds.y_min, XYbounds.y_max);
  passage_zone_.MergePoint(request.start_pose);

  return;
}

bool NodeShrinkDecider::IsLegalForHeading(const Pose2f &pose) {
  if (!heading_shrink_.limit_search_heading_) {
    return true;
  }

  float heading_error = std::fabs(Getf32ThetaDiff(pose.theta, end_.theta));
  if (heading_error > heading_shrink_.max_ref_line_heading_error + 0.1f) {
    return false;
  }

  if (!passage_zone_.Contain(pose)) {
    // if point x value is nearby limiter, it's heading error need small, 45
    // degree check.
    if (std::fabs(pose.x - end_.x) < 2.0f) {
      if (heading_error > 0.784f) {
        return false;
      }
    } else {
      // 90 degree check
      if (heading_error > 1.57f) {
        return false;
      }
    }
  }

  return true;
}

bool NodeShrinkDecider::IsLegalForPose(const Pose2f &pose) {
  if (!IsLegalForHeading(pose) || !IsLegalByXBound(pose.x)) {
    // pose.DebugString();
    return false;
  }

  return true;
}

bool NodeShrinkDecider::IsLegalByXBound(const float x) {
  if (x < x_bound_.lower || x > x_bound_.upper) {
    return false;
  }
  return true;
}

void NodeShrinkDecider::UpdateHeadingErrorWithRefLine() {
  float error = std::fabs(Getf32ThetaDiff(start_.theta, end_.theta));

  // 搜索时,heading 尽量不超过角度
  heading_shrink_.limit_search_heading_ = true;

  // 135 degree
  heading_shrink_.max_ref_line_heading_error = std::max(2.35f, error + 0.35f);
  heading_shrink_.max_ref_line_heading_error =
      std::min(heading_shrink_.max_ref_line_heading_error, M_PIf32);

  // ILOG_INFO << "heading error "
  //           << heading_shrink_.max_ref_line_heading_error * 57.4;

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

bool NodeShrinkDecider::IsShrinkByStartNode(const int start_id, Node3d *child) {
  if (child->GetGlobalID() == start_id) {
    return true;
  }

  return false;
}

bool NodeShrinkDecider::IsShrinkByGearSwitchNumber(Node3d *child) {
  if (child->GetGearSwitchNum() > 15) {
    return true;
  }

  return false;
}

bool NodeShrinkDecider::IsShrinkByHeadOutDirection(const AstarRequest &request,
                                                   const Node3d *child) {
  if (request.direction_request != ParkingVehDirection::HEAD_OUT_TO_LEFT &&
      request.direction_request != ParkingVehDirection::HEAD_OUT_TO_RIGHT &&
      request.direction_request != ParkingVehDirection::HEAD_OUT_TO_MIDDLE) {
    return false;
  }

  constexpr float ANGLE_THRESHOLD_DEG = 15.0f;

  // 计算角度并转换为度数
  const float heading_deg = child->GetPhi() * 180.0 / M_PIf32;

  // 检查是否为前进方向
  const bool is_forward = child->IsForward();

  switch (request.direction_request) {
    case ParkingVehDirection::HEAD_OUT_TO_LEFT:
      return is_forward && heading_deg < -ANGLE_THRESHOLD_DEG;

    case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
      return is_forward && heading_deg > ANGLE_THRESHOLD_DEG;

    default:
      return false;
  }
};

const bool NodeShrinkDecider::IsLoopBackNode(const Node3d *new_node,
                                             const Node3d *old_node) const {
  if (new_node == nullptr || old_node == nullptr) {
    return false;
  }

  const Node3d *parent = new_node->GetPreNode();
  if (parent == nullptr) {
    return false;
  }

  const int new_node_id = new_node->GetGlobalID();
  for (int i = 0; i < 10000; i++) {
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

  float theta_diff = std::fabs(new_node->GetPhi() - old_node->GetPhi());
  if (theta_diff > 0.0087f) {
    return false;
  }

  return true;
}

}  // namespace planning