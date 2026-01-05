#include "side_nudge_lateral_offset_calculate.h"

#include "environmental_model.h"
#include "frenet_obstacle.h"
#include "obstacle.h"
#include "planning_context.h"
#include "task_interface/lateral_obstacle_decider_output.h"

namespace planning {

constexpr double kDefaultLaneWidth = 1.9;
constexpr double kOffsetChangeRateLow = 0.02;
constexpr double kOffsetChangeRateMedium = 0.05;
constexpr double kOffsetChangeRateHigh = 0.1;
constexpr double kExtendLonDistance = 10;
constexpr double kExtendLatDistance = 10;
SideNudgeLateralOffsetDecider::SideNudgeLateralOffsetDecider(
    framework::Session* session,
    const EgoPlanningConfigBuilder* config_builder) {
  session_ = session;
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
}

bool SideNudgeLateralOffsetDecider::Process() {
  if (!Init()) {
    return false;
  }

  RunStateMachine();
  Log();
  return true;
}

bool SideNudgeLateralOffsetDecider::Init() {
  reference_path_ptr_ = session_->planning_context()
                            .lane_change_decider_output()
                            .coarse_planning_info.reference_path;
  if (reference_path_ptr_ == nullptr) {
    return false;
  }
  ego_cart_state_manager_ =
      session_->environmental_model().get_ego_state_manager();
  return true;
}

void SideNudgeLateralOffsetDecider::Reset() {
  lateral_offset_ = 0;
  current_state_ = SideNudgeState::IDLE;
  is_control_time_enough_.SetInvalidCount();
  is_control_.SetInvalidCount();
  is_coodown_time_enough_.SetInvalidCount();
  nudge_info_.Reset();
}

void SideNudgeLateralOffsetDecider::RunStateMachine() {
  switch (current_state_) {
    case SideNudgeState::IDLE: {
      if (IsStartNudge()) {
        LatOffsetCalculate();
        is_control_.SetValidByCount();
      } else {
        Reset();
      }
      break;
    }
    case SideNudgeState::CONTROL: {
      if (IsStopNudgeDirectly()) {
        is_control_.Reset();
      } else if (IsStopNudge()) {
        if (is_control_time_enough_.IsValid()) {
          is_control_.SetInvalidCount();
        }
      } else {
        if (LatOffsetCalculate()) {
          is_control_.SetValidByCount();
        } else if (is_control_time_enough_.IsValid()) {
          is_control_.SetInvalidCount();
        }
      }
      is_control_time_enough_.SetValidByCount();
      break;
    }
    case SideNudgeState::COOLING_DONW: {
      lateral_offset_ = clip(0.0, lateral_offset_ + kOffsetChangeRateLow, lateral_offset_ - kOffsetChangeRateLow);
      if (fabs(lateral_offset_) < 1e-6) {
        nudge_info_.Reset();
      }
      is_coodown_time_enough_.SetValidByCount();
      break;
    }
  }

  UpdateCurrentState();
}

bool SideNudgeLateralOffsetDecider::IsStartNudge() {
  const auto& target_state = session_->planning_context()
                                 .lane_change_decider_output()
                                 .coarse_planning_info.target_state;
  bool is_in_lane_change_scene =
      (target_state == kLaneChangeExecution ||
       target_state == kLaneChangeHold || target_state == kLaneChangeCancel);
  if (is_in_lane_change_scene) {
    return false;
  }

  if (session_->planning_context()
          .spatio_temporal_union_plan_output()
          .enable_using_st_plan) {
    return false;
  }

  bool left_need_nudge = false;
  bool right_need_nudge = false;
  // TODO 自车处于中心线附近时

  const auto& lateral_obstacle_history_info =
      session_->planning_context()
          .lateral_obstacle_decider_output()
          .lateral_obstacle_history_info;
  const auto& obstacles_map = reference_path_ptr_->get_obstacles_map();
  const auto& ego_frenet_state = reference_path_ptr_->get_frenet_ego_state();

  const auto& ego_boundary = ego_frenet_state.boundary();
  const auto care_area_center = planning_math::Vec2d(
      (ego_boundary.s_start + ego_boundary.s_end + kExtendLonDistance) * 0.5,
      ego_frenet_state.l());
  const double care_area_length =
      ego_boundary.s_end + kExtendLonDistance - ego_boundary.s_start;
  const auto ego_extend_polygon = planning_math::Polygon2d(planning_math::Box2d(
      care_area_center, 0, care_area_length, kExtendLatDistance));

  std::vector<std::shared_ptr<FrenetObstacle>> left_side_obstacles;
  std::vector<std::shared_ptr<FrenetObstacle>> right_side_obstacles;
  for (const auto& iter : lateral_obstacle_history_info) {
    const auto id = iter.first;

    if (iter.second.side_car && !iter.second.front_car) {
      if (obstacles_map.find(id) != obstacles_map.end()) {
        const auto frenet_obstacle = obstacles_map.at(id);
        if (frenet_obstacle->is_static()) {
          continue;
        }

        const auto& obstacle_boundary =
            frenet_obstacle->frenet_obstacle_boundary();
        std::vector<planning_math::Vec2d> obstacle_points;
        frenet_obstacle->obstacle()->perception_bounding_box().GetAllCorners(
            &obstacle_points);
        const auto frenet_obstacle_polygon =
            planning_math::Polygon2d(obstacle_points);

        planning_math::Polygon2d care_overlap_polygon;
        double limit_overlap_min_y = obstacle_boundary.l_start;
        double limit_overlap_max_y = obstacle_boundary.l_end;
        if (frenet_obstacle_polygon.ComputeOverlap(ego_extend_polygon,
                                                   &care_overlap_polygon)) {
          limit_overlap_min_y = care_overlap_polygon.min_y();
          limit_overlap_max_y = care_overlap_polygon.max_y();
        }

        if (frenet_obstacle->frenet_l() > 0 &&
            limit_overlap_min_y - ego_boundary.l_end < 1.5 &&
            frenet_obstacle->obstacle()->is_normal()) {
          left_side_obstacles.emplace_back(frenet_obstacle);
        } else if (frenet_obstacle->frenet_l() < 0 &&
                   ego_boundary.l_start - limit_overlap_max_y < 1.5 &&
                   frenet_obstacle->obstacle()->is_normal()
                   //  ego_boundary.l_start - limit_overlap_max_y > 0.2
        ) {
          right_side_obstacles.emplace_back(frenet_obstacle);
        }
      }
    }
  }

  if (left_side_obstacles.size() == 0 && right_side_obstacles.size() == 0) {
    return false;
  }

  std::sort(left_side_obstacles.begin(), left_side_obstacles.end(),
            [&](std::shared_ptr<FrenetObstacle> o1,
                std::shared_ptr<FrenetObstacle> o2) {
              return o1->frenet_obstacle_boundary().l_start <
                     o2->frenet_obstacle_boundary().l_start;
            });
  std::sort(right_side_obstacles.begin(), right_side_obstacles.end(),
            [&](std::shared_ptr<FrenetObstacle> o1,
                std::shared_ptr<FrenetObstacle> o2) {
              return o1->frenet_obstacle_boundary().l_end >
                     o2->frenet_obstacle_boundary().l_end;
            });

  double left_lane_width = kDefaultLaneWidth;
  double right_lane_width = kDefaultLaneWidth;
  ReferencePathPoint reference_path_point;
  if (reference_path_ptr_->get_reference_point_by_lon(ego_frenet_state.s(),
                                                      reference_path_point)) {
    left_lane_width = reference_path_point.distance_to_left_lane_border;
    right_lane_width = reference_path_point.distance_to_right_lane_border;
  }

  NudgeInfo left_nudge_info;
  NudgeInfo right_nudge_info;
  if (left_side_obstacles.size() > 0) {
    const auto& obstacle_boundary =
        left_side_obstacles[0]->frenet_obstacle_boundary();
    planning_math::Polygon2d care_overlap_polygon;
    double limit_overlap_min_y = obstacle_boundary.l_start;
    double limit_overlap_max_y = obstacle_boundary.l_end;

    planning_math::Polygon2d frenet_obstacle_polygon;

    if (planning_math::Polygon2d::ComputeConvexHull(
            left_side_obstacles[0]->corner_points(),
            &frenet_obstacle_polygon) &&
        frenet_obstacle_polygon.ComputeOverlap(ego_extend_polygon,
                                               &care_overlap_polygon)) {
      limit_overlap_min_y = care_overlap_polygon.min_y();
      limit_overlap_max_y = care_overlap_polygon.max_y();
    }
    if (limit_overlap_min_y < left_lane_width) {  // TODO：优化
      left_need_nudge = true;
      left_nudge_info = NudgeInfo(left_side_obstacles[0]->id(),
                                  NudgeDirection::LEFT, limit_overlap_min_y,
                                  limit_overlap_max_y, EmergecyLevel::LOW);
    }
  }

  if (right_side_obstacles.size() > 0) {
    const auto& obstacle_boundary =
        right_side_obstacles[0]->frenet_obstacle_boundary();
    planning_math::Polygon2d care_overlap_polygon;
    double limit_overlap_min_y = obstacle_boundary.l_start;
    double limit_overlap_max_y = obstacle_boundary.l_end;

    planning_math::Polygon2d frenet_obstacle_polygon;
    if (planning_math::Polygon2d::ComputeConvexHull(
            right_side_obstacles[0]->corner_points(),
            &frenet_obstacle_polygon) &&
        frenet_obstacle_polygon.ComputeOverlap(ego_extend_polygon,
                                               &care_overlap_polygon)) {
      limit_overlap_min_y = care_overlap_polygon.min_y();
      limit_overlap_max_y = care_overlap_polygon.max_y();
    }
    if (limit_overlap_max_y > -right_lane_width) {
      right_need_nudge = true;
      right_nudge_info = NudgeInfo(right_side_obstacles[0]->id(),
                                   NudgeDirection::RIGHT, limit_overlap_min_y,
                                   limit_overlap_max_y, EmergecyLevel::LOW);
    }
  }

  if ((left_need_nudge && right_need_nudge) ||
      (!left_need_nudge && !right_need_nudge)) {
    return false;
  }

  if (left_need_nudge) {
    nudge_info_ = left_nudge_info;
  } else if (right_need_nudge) {
    nudge_info_ = right_nudge_info;
  }
  return true;
}

bool SideNudgeLateralOffsetDecider::LatOffsetCalculate() {
  if (nudge_info_.id == 0) {
    return false;
  }

  UpdateNudgeInfo();
  double desire_lateral_offset =
      DesireLateralOffsetSideWay(config_.base_nudge_distance);
  desire_lateral_offset = std::min(0.3, desire_lateral_offset);
  desire_lateral_offset = nudge_info_.nudge_direction == NudgeDirection::LEFT
                              ? -desire_lateral_offset
                              : desire_lateral_offset;
  double offset_change_rate = kOffsetChangeRateLow;
  lateral_offset_ =
      clip(desire_lateral_offset, lateral_offset_ + offset_change_rate,
           lateral_offset_ - offset_change_rate);
  return true;
}

void SideNudgeLateralOffsetDecider::UpdateNudgeInfo() {
  const auto& obstacles_map = reference_path_ptr_->get_obstacles_map();
  // const auto &ego_boundary = ego_frenet_state.boundary();
  if (obstacles_map.find(nudge_info_.id) == obstacles_map.end()) {
    return;
  }
  const auto frenet_obstacle = obstacles_map.at(nudge_info_.id);
  if (frenet_obstacle == nullptr) {
    return;
  }

  const auto& ego_frenet_state = reference_path_ptr_->get_frenet_ego_state();

  const auto& ego_boundary = ego_frenet_state.boundary();
  const auto care_area_center = planning_math::Vec2d(
      (ego_boundary.s_start + ego_boundary.s_end + kExtendLonDistance) * 0.5,
      ego_frenet_state.l());
  const double care_area_length =
      ego_boundary.s_end + kExtendLonDistance - ego_boundary.s_start;
  const auto ego_extend_polygon = planning_math::Polygon2d(planning_math::Box2d(
      care_area_center, 0, care_area_length, kExtendLatDistance));

  const auto& obstacle_boundary = frenet_obstacle->frenet_obstacle_boundary();
  planning_math::Polygon2d care_overlap_polygon;
  double limit_overlap_min_y = obstacle_boundary.l_start;
  double limit_overlap_max_y = obstacle_boundary.l_end;

  planning_math::Polygon2d frenet_obstacle_polygon;
  if (planning_math::Polygon2d::ComputeConvexHull(
          frenet_obstacle->corner_points(), &frenet_obstacle_polygon) &&
      frenet_obstacle_polygon.ComputeOverlap(ego_extend_polygon,
                                             &care_overlap_polygon)) {
    limit_overlap_min_y = care_overlap_polygon.min_y();
    limit_overlap_max_y = care_overlap_polygon.max_y();
  }

  nudge_info_.min_l_to_ref = limit_overlap_min_y;
  nudge_info_.max_l_to_ref = limit_overlap_max_y;
}
bool SideNudgeLateralOffsetDecider::IsStopNudge() {
  if (nudge_info_.id == 0) {
    nudge_info_.cancel_nudge_reason = CancelNudgeReason::OTHER;
    return true;
  }

  const auto& lateral_obstacle_history_info =
      session_->planning_context()
          .lateral_obstacle_decider_output()
          .lateral_obstacle_history_info;
  const auto& obstacles_map = reference_path_ptr_->get_obstacles_map();

  if (lateral_obstacle_history_info.find(nudge_info_.id) ==
      lateral_obstacle_history_info.end()) {
    nudge_info_.cancel_nudge_reason = CancelNudgeReason::OTHER;
    return true;
  }

  if (!lateral_obstacle_history_info.at(nudge_info_.id).side_car) {
    return true;
  }

  // TODO 另外一侧bound推到，需要做特殊处理
  // 另外一侧存在压线障碍物

  const auto& ego_frenet_state = reference_path_ptr_->get_frenet_ego_state();
  const auto& ego_boundary = ego_frenet_state.boundary();
  if (obstacles_map.find(nudge_info_.id) == obstacles_map.end()) {
    nudge_info_.cancel_nudge_reason = CancelNudgeReason::OTHER;
    return true;
  }

  const auto frenet_obstacle = obstacles_map.at(nudge_info_.id);
  if (nudge_info_.nudge_direction == NudgeDirection::LEFT
          ? fabs(frenet_obstacle->frenet_obstacle_boundary().l_start -
                 ego_boundary.l_end) > 1.5
          : fabs(frenet_obstacle->frenet_obstacle_boundary().l_end -
                 ego_boundary.l_start) > 1.5) {
    nudge_info_.cancel_nudge_reason = CancelNudgeReason::LARGE_LAT_DISTANCE;
    return true;
  }

  // 路口

  return false;
}

bool SideNudgeLateralOffsetDecider::IsStopNudgeDirectly() {
  const auto& target_state = session_->planning_context()
                                 .lane_change_decider_output()
                                 .coarse_planning_info.target_state;
  bool is_in_lane_change_scene =
      (target_state == kLaneChangeExecution ||
       target_state == kLaneChangeHold || target_state == kLaneChangeCancel);
  if (is_in_lane_change_scene) {
    nudge_info_.cancel_nudge_reason = CancelNudgeReason::LANE_CHANGE;
    return true;
  }

  if (session_->planning_context()
          .spatio_temporal_union_plan_output()
          .enable_using_st_plan) {
    nudge_info_.cancel_nudge_reason = CancelNudgeReason::ST_UNION;
    return true;
  }

  return false;
}

void SideNudgeLateralOffsetDecider::UpdateCurrentState() {
  SideNudgeState state = current_state_;
  switch (current_state_) {
    case SideNudgeState::IDLE: {
      if (is_control_.IsValid()) {
        state = SideNudgeState::CONTROL;
      }
      break;
    }

    case SideNudgeState::CONTROL: {
      if (!is_control_.IsValid()) {
        state = SideNudgeState::COOLING_DONW;
        is_control_time_enough_.Reset();
      } else {
      }
      break;
    }

    case SideNudgeState::COOLING_DONW: {
      if (is_coodown_time_enough_.IsValid()) {
        state = SideNudgeState::IDLE;
        is_coodown_time_enough_.Reset();
      }
      break;
    }
  }

  current_state_ = state;
}

void SideNudgeLateralOffsetDecider::ResetIdleState() {
  is_control_time_enough_.Reset();
  is_coodown_time_enough_.Reset();
}

double SideNudgeLateralOffsetDecider::DesireLateralOffsetSideWay(
    double base_distance) {
  double lat_offset = 0.0;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;

  const auto& ego_frenet_state = reference_path_ptr_->get_frenet_ego_state();
  const auto& obstacles_map = reference_path_ptr_->get_obstacles_map();
  // const auto &ego_boundary = ego_frenet_state.boundary();
  if (obstacles_map.find(nudge_info_.id) == obstacles_map.end()) {
    return true;
  }
  const auto frenet_obstacle = obstacles_map.at(nudge_info_.id);
  double nearest_l_to_ref =
      fabs(nudge_info_.nudge_direction == NudgeDirection::LEFT
               ? nudge_info_.min_l_to_ref
               : nudge_info_.max_l_to_ref);
  double uncertainty_distance_compensation = 0.0;

  if (frenet_obstacle->obstacle()->is_oversize_vehicle()) {
    base_distance = base_distance + 0.1 + config_.extra_truck_nudge_lat_offset;
    // const std::vector<double> ttc_fp{0, 2, 3, 5};
    // const std::vector<double> distance_compensation_fp{0, 0, 0.1, 0.12};
    // uncertainty_distance_compensation =
    //     planning::interp(pred_ts, ttc_fp, distance_compensation_fp);
  } else if (frenet_obstacle->obstacle()->is_VRU()) {
    base_distance += 0.1;
  } else if (frenet_obstacle->obstacle()->is_traffic_facilities()) {
    base_distance = 0.7;
  }

  double extra_buffer =
      interp(ego_cart_state_manager_->ego_v() * 3.6,
             config_.lateral_offset_obstacle_nudge_buffer_v_bp,
             config_.lateral_offset_nudge_buffer);
  double distance_ego_to_obstacle =
      base_distance + extra_buffer + uncertainty_distance_compensation;
  lat_offset = half_ego_width + distance_ego_to_obstacle - nearest_l_to_ref;

  return std::max(lat_offset, 0.0);
}

void SideNudgeLateralOffsetDecider::Log() {
  JSON_DEBUG_VALUE("side_nudge_info_id", nudge_info_.id);
  JSON_DEBUG_VALUE("side_nudge_info_nudge_direction", (int)nudge_info_.nudge_direction);
  JSON_DEBUG_VALUE("side_nudge_info_emergency_level", (int)nudge_info_.emergency_level);
  JSON_DEBUG_VALUE("side_nudge_current_state", (int)current_state_);
}
}  // namespace planning