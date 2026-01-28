#include "lateral_offset_decider.h"

<<<<<<< HEAD
#include "common/math/filter/mean_filter.h"
=======
>>>>>>> 523f7848f0... opt avoid hmi
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "environmental_model_manager.h"
#include "lateral_offset_decider_info.pb.h"
#include "planning_context.h"
#include "task_interface/lane_change_decider_output.h"
#include "utils/pose2d_utils.h"
namespace planning {

const double kMaxLateralOffsetChangeRate = 0.05;
const double kMaxChangeRateEgoSpeed = 10.;
const int kCoolDownCount = 5;
LateralOffsetDecider::LateralOffsetDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
  lateral_offset_calculatorv2_ = LateralOffsetCalculatorV2(config_builder);
  side_nudge_lateral_offset_decider_ =
      SideNudgeLateralOffsetDecider(session, config_builder);
}

bool LateralOffsetDecider::Execute() {
  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  double lat_offset = 0.0;
  auto last_fix_lane_id = session_->environmental_model()
                              .get_virtual_lane_manager()
                              ->get_last_fix_lane_id();
  auto current_fix_lane_id = session_->planning_context()
                                 .lane_change_decider_output()
                                 .fix_lane_virtual_id;
  const bool dbw_status = session_->environmental_model().GetVehicleDbwStatus();
  if (last_fix_lane_id != current_fix_lane_id || !dbw_status) {
    avoid_obstacle_maintainer5v_.Reset();
    lateral_offset_calculatorv2_
        .ResetOffsetHysteresisMaps();  // 变道和非自动的时候，HysteresisType
                                       // 四个Map都清空
    Reset();
  }

  CalLaneInfo();

  avoid_obstacle_maintainer5v_.Process(session_);

  // 判断障碍物决策是否发生变化
  CheckAvoidObstaclesDecision();

  lateral_offset_calculatorv2_.Process(
      session_, avoid_obstacle_maintainer5v_.avd_obstacles(),
      avoid_obstacle_maintainer5v_.avd_obstacles_history(),
      avoid_obstacle_maintainer5v_.avd_sp_obstacles(), lane_info_,
      avoid_obstacle_maintainer5v_.dist_rblane(),
      avoid_obstacle_maintainer5v_.flag_avd());

  side_nudge_lateral_offset_decider_.Process(lane_info_);

  // lat_offset = lateral_offset_calculatorv2_.lat_offset();

  // SmoothLateralOffset(lat_offset);

  PostProcess();
  GenerateOutput();

  SaveDebugInfo();
  return true;
}

void LateralOffsetDecider::PostProcess() {
  const auto& front_avoid_info = lateral_offset_calculatorv2_.avoid_info();
  const auto& side_nudge_info = side_nudge_lateral_offset_decider_.nudge_info();
  double front_lat_offset = lateral_offset_calculatorv2_.lat_offset();
  double side_lat_offset = side_nudge_lateral_offset_decider_.lat_offset();
  const auto side_nudge_state =
      side_nudge_lateral_offset_decider_.nudge_state();

  double lateral_offset_tmp = front_lat_offset;
  NudgeDirection front_direction = NudgeDirection::NONE;
  if (front_avoid_info.avoid_way != AvoidWay::None) {
    if (front_avoid_info.avoid_way == AvoidWay::Left) {
      front_direction = NudgeDirection::LEFT;
    } else if (front_avoid_info.avoid_way == AvoidWay::Right) {
      front_direction = NudgeDirection::RIGHT;
    } else if (lateral_offset_ > 0) {
      front_direction = NudgeDirection::RIGHT;
    } else if (lateral_offset_ < 0) {
      front_direction = NudgeDirection::LEFT;
    }
  }

  NudgeDirection side_direction = side_nudge_info.nudge_direction;
  if (side_nudge_info.nudge_direction != NudgeDirection::NONE) {
    if (front_direction == side_nudge_info.nudge_direction) {
      lateral_offset_tmp = front_direction == NudgeDirection::LEFT
                               ? std::min(side_lat_offset, front_lat_offset)
                               : std::max(side_lat_offset, front_lat_offset);
    } else if (side_nudge_state != SideNudgeState::CONTROL) {
      lateral_offset_tmp = front_lat_offset;
    } else {
      lateral_offset_tmp = side_lat_offset;
    }
  }

  constexpr double lateral_offset_change_rate = 0.05;
  lateral_offset_ =
      clip(lateral_offset_tmp, lateral_offset_ + lateral_offset_change_rate,
           lateral_offset_ - lateral_offset_change_rate);
}

void LateralOffsetDecider::CheckAvoidObstaclesDecision() {
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  auto check_and_update = [&](int index) {
    if (index >= avoid_obstacle_maintainer5v_.avd_obstacles().size()) {
      return;
    }
    const auto& obs = avoid_obstacle_maintainer5v_.avd_obstacles()[index];
    auto iter = lat_obstacle_decision.find(obs.track_id);
    if (iter == lat_obstacle_decision.end()) {
      return;
    }
    int track_id = obs.track_id;
    const auto& decision = iter->second;
    if (track_id == last_first_obstacle_id_) {
      if (IsObstacleDecisionSwitch(last_first_obstacle_decision_, decision)) {
        Reset();
      }
      last_first_obstacle_decision_ = decision;
      last_first_obstacle_id_ = track_id;
    } else if (track_id == last_second_obstacle_id_) {
      if (IsObstacleDecisionSwitch(last_second_obstacle_decision_, decision)) {
        Reset();
      }
      last_second_obstacle_decision_ = decision;
      last_second_obstacle_id_ = track_id;
    }
  };
  // 对两个障碍物执行检查
  check_and_update(0);
  check_and_update(1);
}

bool LateralOffsetDecider::IsObstacleDecisionSwitch(
    LatObstacleDecisionType last_decision,
    LatObstacleDecisionType current_decision) {
  return (last_decision == LatObstacleDecisionType::RIGHT &&
          current_decision == LatObstacleDecisionType::LEFT) ||
         (last_decision == LatObstacleDecisionType::LEFT &&
          current_decision == LatObstacleDecisionType::RIGHT);
}

void LateralOffsetDecider::SmoothLateralOffset(double in_lat_offset) {
  const std::array<AvoidObstacleInfo, 2> avoid_obstacles =
      avoid_obstacle_maintainer5v_.avd_obstacles();
  bool is_overlap[2] = {false, false};
  const double t_buffer = 0.9;
  if (avoid_obstacles[0].flag != AvoidObstacleFlag::INVALID) {
    is_overlap[0] = lateral_offset_decider::HasOverlap(
        session_, avoid_obstacles[0],
        std::max(t_buffer * (-avoid_obstacles[0].vs_lon_relative), 3.0), 2.0);
    if (avoid_obstacles[1].flag != AvoidObstacleFlag::INVALID) {
      is_overlap[1] = lateral_offset_decider::HasOverlap(
          session_, avoid_obstacles[1],
          std::max(t_buffer * (-avoid_obstacles[1].vs_lon_relative), 3.0), 2.0);
    }
  }

  double overlap_lateral_offset_change_rate = kMaxLateralOffsetChangeRate;
  if (is_overlap[0] && is_overlap[1]) {
    if ((avoid_obstacles[0].min_l_to_ref > 0 &&
         avoid_obstacles[1].min_l_to_ref < 0) ||
        (avoid_obstacles[0].min_l_to_ref < 0 &&
         avoid_obstacles[1].min_l_to_ref > 0)) {
      overlap_lateral_offset_change_rate = 0.01;
    } else if ((avoid_obstacles[0].min_l_to_ref > 0 &&
                in_lat_offset > lateral_offset_) ||
               (avoid_obstacles[0].min_l_to_ref < 0 &&
                in_lat_offset < lateral_offset_)) {
      if (lateral_offset_decider::IsTruck(avoid_obstacles[0]) ||
          lateral_offset_decider::IsTruck(avoid_obstacles[1])) {
        overlap_lateral_offset_change_rate = 0.01;
      } else {
        overlap_lateral_offset_change_rate = 0.02;
      }
    }
  } else if (is_overlap[0]) {
    if ((avoid_obstacles[0].min_l_to_ref > 0 &&
         in_lat_offset > lateral_offset_) ||
        (avoid_obstacles[0].min_l_to_ref < 0 &&
         in_lat_offset < lateral_offset_)) {
      if (lateral_offset_decider::IsTruck(avoid_obstacles[0])) {
        overlap_lateral_offset_change_rate = 0.01;
      } else {
        overlap_lateral_offset_change_rate = 0.02;
      }
    }
  } else if (is_overlap[1]) {
    if ((avoid_obstacles[1].min_l_to_ref > 0 &&
         in_lat_offset > lateral_offset_) ||
        (avoid_obstacles[1].min_l_to_ref < 0 &&
         in_lat_offset < lateral_offset_)) {
      if (lateral_offset_decider::IsTruck(avoid_obstacles[1])) {
        overlap_lateral_offset_change_rate = 0.01;
      } else {
        overlap_lateral_offset_change_rate = 0.02;
      }
    }
  }

  const auto& avd_obstacles_history =
      avoid_obstacle_maintainer5v_.avd_obstacles_history();
  const auto& avd_obstacles = avoid_obstacle_maintainer5v_.avd_obstacles();
  if (avd_obstacles[0].flag == AvoidObstacleFlag::INVALID) {
    if (avd_obstacles_history[0].flag != AvoidObstacleFlag::INVALID &&
        avd_obstacles_history[0].s_to_ego >= 0) {
      overlap_lateral_offset_change_rate = 0.01;
    }
  }

  const double speed_ratio =
      kMaxLateralOffsetChangeRate / kMaxChangeRateEgoSpeed;
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double lateral_offset_change_rate =
      clip(speed_ratio * ego_state_manager->ego_v(),
           kMaxLateralOffsetChangeRate, 0.);
  lateral_offset_change_rate =
      std::min(lateral_offset_change_rate, overlap_lateral_offset_change_rate);
  lateral_offset_ =
      clip(in_lat_offset, lateral_offset_ + lateral_offset_change_rate,
           lateral_offset_ - lateral_offset_change_rate);
}

void LateralOffsetDecider::CalLaneInfo() {
  last_lane_info_ = lane_info_;
  auto& coarse_planning_info = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info;

  const auto flane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  lane_info_.lane_width = CalLaneWidth(flane);
  CalculateNormalLateralOffsetThreshold(flane);
}

double LateralOffsetDecider::CalLaneWidth(
    const std::shared_ptr<VirtualLane> flane) {
  double lane_width = kDefaultLaneWidth;
  if (flane == nullptr) {
    return lane_width;
  }
  auto& coarse_planning_info = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info;

  const auto& reference_path = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info.reference_path;
  const auto ego_frenet_state = reference_path->get_frenet_ego_state();
  if (1) {
    double width = 0.0;
    double preview_s = 20 + ego_frenet_state.s();
    double start_s = 5 + ego_frenet_state.s();
    double interval_s = 5;
    int point_num = 0;
    for (double s = start_s; s <= preview_s; s += interval_s) {
      width += flane->width_by_s(s);
      point_num += 1;
    }
    width /= point_num;
    static planning_math::MeanFilter width_filter(10);
    lane_width = width_filter.Update(width);
  } else {
    lane_width = flane->width();
  }

  return lane_width;
}

// Calculate max avoid threshold
void LateralOffsetDecider::CalculateNormalLateralOffsetThreshold(
    const std::shared_ptr<VirtualLane> flane) {
  if (flane == nullptr) {
    return;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const double ego_width = vehicle_param.max_width;

  const double half_lane_width = lane_info_.lane_width * 0.5;
  const double road_avoid_threshold = std::min(
      half_lane_width - config_.nudge_buffer_road_boundary - ego_width * 0.5,
      config_.nudge_lat_offset_threshold);
  const double lane_avoid_threshold = std::min(
      half_lane_width - config_.nudge_buffer_lane_boundary - ego_width * 0.5,
      config_.nudge_lat_offset_threshold);

  int right_lane_virtual_id = flane->get_virtual_id() + 1;
  int left_lane_virtual_id = flane->get_virtual_id() - 1;
  int fix_lane_virtual_id = flane->get_virtual_id();
  bool has_left_lane = virtual_lane_manager->get_lane_with_virtual_id(
                           left_lane_virtual_id) != nullptr;
  bool has_right_lane = virtual_lane_manager->get_lane_with_virtual_id(
                            right_lane_virtual_id) != nullptr;
  if (!has_left_lane && !has_right_lane) {
    lane_info_.normal_left_avoid_threshold = road_avoid_threshold;
    lane_info_.normal_right_avoid_threshold = road_avoid_threshold;
  } else if (!has_right_lane) {
    lane_info_.normal_left_avoid_threshold = lane_avoid_threshold;
    lane_info_.normal_right_avoid_threshold = road_avoid_threshold;
  } else if (!has_left_lane) {
    lane_info_.normal_left_avoid_threshold = road_avoid_threshold;
    lane_info_.normal_right_avoid_threshold = lane_avoid_threshold;
  } else {
    lane_info_.normal_left_avoid_threshold = lane_avoid_threshold;
    lane_info_.normal_right_avoid_threshold = lane_avoid_threshold;
  }

  auto last_fix_lane_virtual_id = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->get_last_fix_lane_id();

  if (last_fix_lane_virtual_id == fix_lane_virtual_id) {
    const double change_rate = 0.02;
    if (last_lane_info_.normal_left_avoid_threshold > 1e-2) {
      lane_info_.normal_left_avoid_threshold =
          clip(lane_info_.normal_left_avoid_threshold,
               last_lane_info_.normal_left_avoid_threshold + change_rate,
               last_lane_info_.normal_left_avoid_threshold - change_rate);
    }
    if (last_lane_info_.normal_right_avoid_threshold > 1e-2) {
      lane_info_.normal_right_avoid_threshold =
          clip(lane_info_.normal_right_avoid_threshold,
               last_lane_info_.normal_right_avoid_threshold + change_rate,
               last_lane_info_.normal_right_avoid_threshold - change_rate);
    }
  }

  lane_info_.normal_left_avoid_threshold =
      std::max(lane_info_.normal_left_avoid_threshold, 0.0);
  lane_info_.normal_right_avoid_threshold =
      std::max(lane_info_.normal_right_avoid_threshold, 0.0);
}

void LateralOffsetDecider::Reset() {
  lateral_offset_ = 0;
  lane_info_.Reset();
}

void LateralOffsetDecider::SaveDebugInfo() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_manager = DebugInfoManager::GetInstance();
  auto& planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lateral_offset_decider_info =
      planning_debug_data->mutable_lateral_offset_decider_info();
  lateral_offset_decider_info->set_smooth_lateral_offset(lateral_offset_);
#endif

  JSON_DEBUG_VALUE("smooth_lateral_offset", lateral_offset_);
}

void LateralOffsetDecider::GenerateOutput() {
  LateralOffsetDeciderOutput& lateral_offset_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  lateral_offset_decider_output.is_valid =
      config_.is_valid_lateral_offset && fabs(lateral_offset_) > 1e-2;
  lateral_offset_decider_output.lateral_offset = lateral_offset_;
  lateral_offset_decider_output.enable_bound =
      lateral_offset_calculatorv2_.enable_bound();
  lateral_offset_decider_output.avoid_ids.clear();
}

}  // namespace planning