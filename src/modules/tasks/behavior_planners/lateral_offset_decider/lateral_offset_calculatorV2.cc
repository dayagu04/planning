#include "lateral_offset_calculatorV2.h"
#include <cassert>
#include <iomanip>
#include <utility>
#include <vector>
#include "common.pb.h"
#include "common/math/filter/mean_filter.h"
#include "common/utils/hysteresis_decision.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "fusion_road_c.h"
#include "lateral_behavior_planner.pb.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_offset_decider_utils.h"
#include "planning_context.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "utils/hysteresis_decision.h"
#include "utils/pose2d_utils.h"

namespace planning {

LateralOffsetCalculatorV2::LateralOffsetCalculatorV2(
    const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
  std::map<std::pair<int, int>, HysteresisDecision> hysteresis_map;
  max_opposite_offset_hysteresis_maps_[HysteresisType::EnoughSpaceHysteresis] =
      hysteresis_map;
  max_opposite_offset_hysteresis_maps_[HysteresisType::AvoidWaySelect] =
      hysteresis_map;

  std::map<int, HysteresisDecision> hysteresis_map2;
  max_opposite_offset_hysteresis_maps_
      [HysteresisType::IsObstacleConsideredHysteresis] = hysteresis_map2;
  max_opposite_offset_hysteresis_maps_
      [HysteresisType::IsInConsiderLateralRangeHysteresis] = hysteresis_map2;

  avoid_hysteresis_maps_[HysteresisType::EnoughSpaceHysteresis] =
      hysteresis_map;
  avoid_hysteresis_maps_[HysteresisType::AvoidWaySelect] = hysteresis_map;
  avoid_hysteresis_maps_[HysteresisType::IsObstacleConsideredHysteresis] =
      hysteresis_map2;
  avoid_hysteresis_maps_[HysteresisType::IsInConsiderLateralRangeHysteresis] =
      hysteresis_map2;

  has_enough_speed_hysteresis_.SetThreValue(config_.v_limit_max + 5,
                                            config_.v_limit_max - 5);
}

bool LateralOffsetCalculatorV2::Process(
    framework::Session *session,
    const std::array<AvoidObstacleInfo, 2> &avd_obstacle,
    const std::array<AvoidObstacleInfo, 2> &avd_sp_obstacle, double dist_rblane,
    bool flag_avd) {
  // NTRACE_CALL(7);
  auto current_time = IflyTime::Now_ms();
  session_ = session;

  auto &planning_context = session_->planning_context();
  // auto &ego_prediction_result = pipeline_context_->planning_result;
  // auto &ego_prediction_info = pipeline_context_->planning_info;
  auto &coarse_planning_info =
      planning_context.lane_change_decider_output().coarse_planning_info;
  bool b_success = false;

  // obtain the session information
  const auto &lane_change_decider_output =
      planning_context.lane_change_decider_output();
  const auto &status = lane_change_decider_output.curr_state;
  const auto &should_premove = lane_change_decider_output.should_premove;

  // init info
  flane_ = session_->environmental_model()
               .get_virtual_lane_manager()
               ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  auto reference_path_ptr = coarse_planning_info.reference_path;
  ego_frenet_state_ = reference_path_ptr->get_frenet_ego_state();
  ego_cart_state_manager_ =
      session_->environmental_model().get_ego_state_manager();

  virtual_lane_manager_ =
      session_->environmental_model().get_virtual_lane_manager();

  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  has_enough_speed_hysteresis_.SetIsValidByValue(ego_v * 3.6);
  enable_bound_ = !has_enough_speed_hysteresis_.IsValid();

  b_success = update(status, flag_avd, should_premove, dist_rblane,
                     avd_obstacle, avd_sp_obstacle);

  if (!b_success) {
    // TBD : add logs
  }
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralOffsetCalculatorCost", end_time - current_time);
  return b_success;
}

bool LateralOffsetCalculatorV2::update(
    int status, bool flag_avd, bool should_premove, double dist_rblane,
    const std::array<AvoidObstacleInfo, 2> &avd_obstacle,
    const std::array<AvoidObstacleInfo, 2> &avd_sp_obstacle) {
  last_avoid_info_ = avoid_info_;
  if (status >= kLaneKeeping && status <= kLaneChangeHold) {
    UpdateAvoidPath(status, flag_avd, should_premove, dist_rblane, avd_obstacle,
                    avd_sp_obstacle);
  } else {
    Reset();
    auto &enough_space_hysteresis_map =
        std::get<std::map<std::pair<int, int>, HysteresisDecision>>(
            max_opposite_offset_hysteresis_maps_
                [HysteresisType::EnoughSpaceHysteresis]);
    enough_space_hysteresis_map.clear();
  }

  SaveDebugInfo();
  return true;
}

bool LateralOffsetCalculatorV2::UpdateAvoidPath(
    int status, bool flag_avd, bool should_premove, double dist_rblane,
    const std::array<AvoidObstacleInfo, 2> &avd_obstacle,
    const std::array<AvoidObstacleInfo, 2> &avd_sp_obstacle) {
  Reset();
  CalLaneWidth();
  CalculateNormalLateralOffsetThreshold();
  PreacquireMaxOppositeOffsetIds();

  if (avd_obstacle[0].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacle[1].flag == AvoidObstacleFlag::INVALID) {
    LateralOffsetCalculateOneObstacle(avd_obstacle[0]);
  } else if (avd_obstacle[0].flag != AvoidObstacleFlag::INVALID &&
             avd_obstacle[1].flag != AvoidObstacleFlag::INVALID) {
    LateralOffsetCalculateTwoObstacle(avd_obstacle[0], avd_obstacle[1]);
  } else {
    auto &enough_space_hysteresis_map =
        std::get<std::map<std::pair<int, int>, HysteresisDecision>>(
            max_opposite_offset_hysteresis_maps_
                [HysteresisType::EnoughSpaceHysteresis]);
    enough_space_hysteresis_map.clear();
  }

  PostProcess(avd_obstacle);
  return true;
}

// Calculate max avoid threshold
void LateralOffsetCalculatorV2::CalculateNormalLateralOffsetThreshold() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_width = vehicle_param.max_width;

  const double road_avoid_threshold =
      lane_width_ * 0.5 - config_.nudge_buffer_road_boundary - ego_width * 0.5;
  const double lane_avoid_threshold =
      lane_width_ * 0.5 - config_.nudge_buffer_lane_boundary - ego_width * 0.5;
  const double static_lane_avoid_threshold =
      lane_width_ * 0.5 - config_.static_nudge_buffer_lane_boundary -
      ego_width * 0.5;
  int right_lane_virtual_id = flane_->get_virtual_id() + 1;
  int left_lane_virtual_id = flane_->get_virtual_id() - 1;
  int fix_lane_virtual_id = flane_->get_virtual_id();
  bool has_left_lane = virtual_lane_manager_->get_lane_with_virtual_id(
                           left_lane_virtual_id) != nullptr;
  bool has_right_lane = virtual_lane_manager_->get_lane_with_virtual_id(
                            right_lane_virtual_id) != nullptr;
  if (!has_left_lane && !has_right_lane) {
    avoid_info_.normal_left_avoid_threshold = road_avoid_threshold;
    avoid_info_.normal_right_avoid_threshold = road_avoid_threshold;
  } else if (!has_right_lane) {
    avoid_info_.normal_left_avoid_threshold = lane_avoid_threshold;
    avoid_info_.normal_right_avoid_threshold = road_avoid_threshold;
  } else if (!has_left_lane) {
    avoid_info_.normal_left_avoid_threshold = road_avoid_threshold;
    avoid_info_.normal_right_avoid_threshold = lane_avoid_threshold;
  } else {
    avoid_info_.normal_left_avoid_threshold = lane_avoid_threshold;
    avoid_info_.normal_right_avoid_threshold = lane_avoid_threshold;
  }

  avoid_info_.static_left_avoid_threshold = static_lane_avoid_threshold;
  avoid_info_.static_right_avoid_threshold = static_lane_avoid_threshold;

  auto last_fix_lane_virtual_id = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->get_last_fix_lane_id();

  if (last_fix_lane_virtual_id == fix_lane_virtual_id) {
    const double change_rate = 0.02;
    if (last_avoid_info_.normal_left_avoid_threshold > 1e-2) {
      avoid_info_.normal_left_avoid_threshold =
          clip(avoid_info_.normal_left_avoid_threshold,
               last_avoid_info_.normal_left_avoid_threshold + change_rate,
               last_avoid_info_.normal_left_avoid_threshold - change_rate);
    }
    if (last_avoid_info_.normal_right_avoid_threshold > 1e-2) {
      avoid_info_.normal_right_avoid_threshold =
          clip(avoid_info_.normal_right_avoid_threshold,
               last_avoid_info_.normal_right_avoid_threshold + change_rate,
               last_avoid_info_.normal_right_avoid_threshold - change_rate);
    }
  }

  avoid_info_.normal_left_avoid_threshold =
      std::max(avoid_info_.normal_left_avoid_threshold, 0.0);
  avoid_info_.normal_right_avoid_threshold =
      std::max(avoid_info_.normal_right_avoid_threshold, 0.0);
  avoid_info_.static_left_avoid_threshold =
      std::max(avoid_info_.static_left_avoid_threshold, 0.0);
  avoid_info_.static_right_avoid_threshold =
      std::max(avoid_info_.static_right_avoid_threshold, 0.0);
}

void LateralOffsetCalculatorV2::LateralOffsetCalculateOneObstacle(
    const AvoidObstacleInfo &avoid_obstacle) {
  CalcMaxOppositeOffset(avoid_obstacle);
  bool is_left = avoid_obstacle.min_l_to_ref > 0.0;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_length = vehicle_param.length;
  double lat_offset = 0.0;
  double lat_offset_compensate = LateralOffsetCompensate(avoid_obstacle);

  avoid_info_.avoid_way = is_left ? AvoidWay::Left : AvoidWay::Right;
  if ((avoid_obstacle.flag == AvoidObstacleFlag::NORMAL ||
       avoid_obstacle.flag == AvoidObstacleFlag::SIDE) &&
      avoid_obstacle.s_to_ego > -(ego_length + kSafeDistance)) {  // 前方障碍物
    avoid_info_.desire_lat_offset = DesireLateralOffsetSideWay(
        avoid_obstacle, avoid_info_.avoid_way, 0.5, lat_offset_compensate,
        config_.base_nudge_distance);

    lat_offset = LimitLateralOffset(
        avoid_obstacle, avoid_info_.desire_lat_offset, avoid_info_.avoid_way);
    double smooth_lateral_offset =
        SmoothLateralOffset(avoid_obstacle, is_left ? -lat_offset : lat_offset,
                            &avoid_info_.avoid_way);
    avoid_info_.lat_offset = smooth_lateral_offset;
  } else {
    avoid_info_.lat_offset = 0.0;
  }
}

void LateralOffsetCalculatorV2::LateralOffsetCalculateTwoObstacle(
    const AvoidObstacleInfo &avoid_obstacle_1,
    const AvoidObstacleInfo &avoid_obstacle_2) {
  double t_exceed_obstacle_1;
  auto &avoid_way_select_hysteresis_map =
      std::get<std::map<std::pair<int, int>, HysteresisDecision>>(
          avoid_hysteresis_maps_[HysteresisType::AvoidWaySelect]);

  bool is_side_way = AvoidWaySelectForTwoObstacle(
      avoid_obstacle_1, avoid_obstacle_2, &t_exceed_obstacle_1);

  const int num_hysteresis = 3;
  if (!avoid_way_select_hysteresis_map.empty()) {
    if (avoid_obstacle_1.track_id ==
            avoid_way_select_hysteresis_map.begin()->first.first &&
        avoid_obstacle_2.track_id ==
            avoid_way_select_hysteresis_map.begin()->first.second) {
      if (is_side_way) {
        avoid_way_select_hysteresis_map.begin()->second.SetValidByCount();
      } else {
        avoid_way_select_hysteresis_map.begin()->second.SetInvalidCount();
      }
    } else {
      HysteresisDecision avoid_way_select_hysteresis(num_hysteresis,
                                                     num_hysteresis);
      if (is_side_way) {
        for (int i = 0; i < num_hysteresis; i++) {
          avoid_way_select_hysteresis.SetValidByCount();
        }
      } else {
        for (int i = 0; i < num_hysteresis; i++) {
          avoid_way_select_hysteresis.SetInvalidCount();
        }
      }
      avoid_way_select_hysteresis_map[std::make_pair(
          avoid_obstacle_1.track_id, avoid_obstacle_2.track_id)] =
          std::move(avoid_way_select_hysteresis);
    }
  } else {
    HysteresisDecision avoid_way_select_hysteresis(num_hysteresis,
                                                   num_hysteresis);
    if (is_side_way) {
      for (int i = 0; i < num_hysteresis; i++) {
        avoid_way_select_hysteresis.SetValidByCount();
      }
    } else {
      for (int i = 0; i < num_hysteresis; i++) {
        avoid_way_select_hysteresis.SetInvalidCount();
      }
    }
    avoid_way_select_hysteresis_map[std::make_pair(avoid_obstacle_1.track_id,
                                                   avoid_obstacle_2.track_id)] =
        std::move(avoid_way_select_hysteresis);
  }

  is_side_way =
      avoid_way_select_hysteresis_map[std::make_pair(avoid_obstacle_1.track_id,
                                                     avoid_obstacle_2.track_id)]
          .IsValid();
  double lateral_offset = 0;
  if (t_exceed_obstacle_1 > 0) {
    if ((avoid_obstacle_1.min_l_to_ref > 0 &&
         avoid_obstacle_2.min_l_to_ref < 0) ||
        (avoid_obstacle_1.min_l_to_ref < 0 &&
         avoid_obstacle_2.min_l_to_ref > 0)) {
      lateral_offset = DealwithTwoObstacleTwoSide(
          avoid_obstacle_1, avoid_obstacle_2, is_side_way);
    } else if ((avoid_obstacle_1.min_l_to_ref > 0 &&
                avoid_obstacle_2.min_l_to_ref > 0) ||
               (avoid_obstacle_1.min_l_to_ref < 0 &&
                avoid_obstacle_2.min_l_to_ref < 0)) {
      // TODO(clren):consider min_l_to_ref > 0, max_l_to_ref < 0; use obstacle
      // lat decision
      lateral_offset = DealwithTwoObstacleOneSide(
          avoid_obstacle_1, avoid_obstacle_2, is_side_way);
    }
  }

  double smooth_lateral_offset = SmoothLateralOffset(
      avoid_obstacle_1, lateral_offset, &avoid_info_.avoid_way);
  avoid_info_.lat_offset = smooth_lateral_offset;
}

// return true: avoid obstacle_1 first, then avoid obstacle_2
// return false: decide other way later
bool LateralOffsetCalculatorV2::AvoidWaySelectForTwoObstacle(
    const AvoidObstacleInfo &avoid_obstacle_1,
    const AvoidObstacleInfo &avoid_obstacle_2, double *t_exceed_obstacle_1) {
  const double v_ego = ego_cart_state_manager_->ego_v();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_length = vehicle_param.length;
  const double v_obstacle_2 = v_ego + avoid_obstacle_2.vs_lon_relative;
  const double kDistanceOffset = 3.5;
  std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
  std::array<double, 3> t_gap_vego_bp{5, 15, 30};
  const double safety_dist = 2.0 + v_ego * 0.2;

  // desired t_gap to obstacle_2 when exceed obstacle_1
  const double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);

  // desired distance to obstacle_2 when exceed obstacle_1
  const double desired_distance_to_obstacle_2 =
      kDistanceOffset + v_obstacle_2 * t_gap;
  double relative_distance_nudge_obstacle =
      avoid_obstacle_2.tail_s_to_ego - avoid_obstacle_1.tail_s_to_ego;
  const double relative_vel_nudge_obstacle =
      avoid_obstacle_2.vs_lon_relative - avoid_obstacle_1.vs_lon_relative;
  const double distance_exceed_obstacle_1 = avoid_obstacle_1.tail_s_to_ego +
                                            avoid_obstacle_1.length +
                                            ego_length + safety_dist;

  if (equal_zero(avoid_obstacle_1.vs_lon_relative) == false) {
    if (avoid_obstacle_1.vs_lon_relative < -1.0e-3) {
      *t_exceed_obstacle_1 =
          -distance_exceed_obstacle_1 / avoid_obstacle_1.vs_lon_relative;
    } else {
      *t_exceed_obstacle_1 = 5.;
    }
  } else {
    *t_exceed_obstacle_1 = (v_ego < 1) ? 5. : 0.;
  }

  // relative distance between obstacle_1 and obstacle_2 when exceed obstacle_1
  relative_distance_nudge_obstacle +=
      relative_vel_nudge_obstacle * (*t_exceed_obstacle_1);

  bool is_side_way =
      relative_distance_nudge_obstacle >=
      desired_distance_to_obstacle_2 + 5.0 + safety_dist + 0.5 * v_ego;
  return is_side_way;
}

double LateralOffsetCalculatorV2::DealwithTwoObstacleTwoSide(
    const AvoidObstacleInfo &avoid_obstacle_1,
    const AvoidObstacleInfo &avoid_obstacle_2, bool is_side_way) {
  double lateral_offset = 0.0;
  double lat_offset_compensate_1 = LateralOffsetCompensate(avoid_obstacle_1);
  double lat_offset_compensate_2 = LateralOffsetCompensate(avoid_obstacle_2);

  bool is_left = avoid_obstacle_1.min_l_to_ref > 0;
  if ((lateral_offset_decider::HasOverlap(session_, avoid_obstacle_1, 0, 0) &&
       lateral_offset_decider::HasOverlap(session_, avoid_obstacle_2, 0, 0)) ||
      (!is_side_way &&
       //  !(lateral_offset_decider::IsTruck(avoid_obstacle_1) ||
       //    lateral_offset_decider::IsTruck(avoid_obstacle_2)) &&
       lateral_offset_decider::HasEnoughSpace(avoid_obstacle_1,
                                              avoid_obstacle_2))) {
    // it has enough space to go through the center
    lateral_offset = DesireLateralOffsetCenterWay(
        avoid_obstacle_1, avoid_obstacle_2, is_left, lat_offset_compensate_1,
        lat_offset_compensate_2);
    curr_time_ = IflyTime::Now_s();
    avoid_info_.avoid_way = AvoidWay::Center;
  } else if (is_side_way) {
    if (IflyTime::Now_s() - curr_time_ > 2) {
      // avoid obstacle_1 first, then avoid obstacle_2
      avoid_info_.avoid_way = is_left ? AvoidWay::Left : AvoidWay::Right;
      CalcMaxOppositeOffset(avoid_obstacle_1, avoid_obstacle_2.track_id);
      lateral_offset = DesireLateralOffsetSideWay(
          avoid_obstacle_1, avoid_info_.avoid_way, 0.5, lat_offset_compensate_1,
          config_.base_nudge_distance);
      lateral_offset = LimitLateralOffset(avoid_obstacle_1, lateral_offset,
                                          avoid_info_.avoid_way);
      lateral_offset = is_left ? -lateral_offset : lateral_offset;
    } else {
      // TODO(clren)
      // keep
      avoid_info_.avoid_way = AvoidWay::Center;
      lateral_offset = ego_frenet_state_.l();
      avoid_info_.is_use_ego_position = true;
    }
  } else {
    if (avoid_obstacle_1.vs_lon_relative - avoid_obstacle_2.vs_lon_relative <
        2) {
      // avoid obstacle_1, follow obstacle_2
      // TODO(clren):添加障碍物决策
      avoid_info_.avoid_way = is_left ? AvoidWay::Left : AvoidWay::Right;
      CalcMaxOppositeOffset(avoid_obstacle_1, avoid_obstacle_2.track_id);
      lateral_offset = DesireLateralOffsetSideWay(
          avoid_obstacle_1, avoid_info_.avoid_way, 0.6, lat_offset_compensate_1,
          config_.base_nudge_distance);
      lateral_offset = LimitLateralOffset(avoid_obstacle_1, lateral_offset,
                                          avoid_info_.avoid_way);
      lateral_offset = is_left ? -lateral_offset : lateral_offset;
    } else {
      // avoid obstacle_2, follow obstacle_1
      avoid_info_.avoid_way = is_left ? AvoidWay::Right : AvoidWay::Left;
      CalcMaxOppositeOffset(avoid_obstacle_2, avoid_obstacle_1.track_id);
      lateral_offset = DesireLateralOffsetSideWay(
          avoid_obstacle_2, avoid_info_.avoid_way, 0.6, lat_offset_compensate_1,
          config_.base_nudge_distance);
      lateral_offset = LimitLateralOffset(avoid_obstacle_2, lateral_offset,
                                          avoid_info_.avoid_way);
      lateral_offset = is_left ? lateral_offset : -lateral_offset;
    }
  }

  return lateral_offset;
}

double LateralOffsetCalculatorV2::DealwithTwoObstacleOneSide(
    const AvoidObstacleInfo &avoid_obstacle_1,
    const AvoidObstacleInfo &avoid_obstacle_2, bool is_side_way) {
  double lateral_offset = 0.0;
  double lat_offset_compensate_1 = LateralOffsetCompensate(avoid_obstacle_1);
  bool is_left = avoid_obstacle_1.min_l_to_ref > 0;

  avoid_info_.avoid_way = is_left ? AvoidWay::Left : AvoidWay::Right;
  if (!is_side_way) {
    const AvoidObstacleInfo *nearest_avoid_obstacle;
    if (is_left) {
      nearest_avoid_obstacle =
          avoid_obstacle_1.min_l_to_ref < avoid_obstacle_2.min_l_to_ref
              ? &avoid_obstacle_1
              : &avoid_obstacle_2;
    } else {
      nearest_avoid_obstacle =
          avoid_obstacle_1.max_l_to_ref > avoid_obstacle_2.max_l_to_ref
              ? &avoid_obstacle_1
              : &avoid_obstacle_2;
    }
    avoid_id_ = nearest_avoid_obstacle->track_id;
    CalcMaxOppositeOffset(*nearest_avoid_obstacle, -1);
    lateral_offset = DesireLateralOffsetSideWay(
        *nearest_avoid_obstacle, avoid_info_.avoid_way, 0.5,
        lat_offset_compensate_1, config_.base_nudge_distance);
  } else {
    CalcMaxOppositeOffset(avoid_obstacle_1);
    lateral_offset = DesireLateralOffsetSideWay(
        avoid_obstacle_1, avoid_info_.avoid_way, 0.5, lat_offset_compensate_1,
        config_.base_nudge_distance);
  }
  lateral_offset = LimitLateralOffset(avoid_obstacle_1, lateral_offset,
                                      avoid_info_.avoid_way);
  lateral_offset = is_left ? -lateral_offset : lateral_offset;

  return lateral_offset;
}

double LateralOffsetCalculatorV2::SmoothLateralOffset(
    const AvoidObstacleInfo &avoid_obstacle, double lat_offset,
    const AvoidWay *avoid_way) {
  if (avoid_info_.is_use_ego_position) {
    return lat_offset;
  }
  double smooth_lat_offset = lat_offset;
  bool is_positive = lat_offset > 0;

  const std::array<double, 5> _AVD_DISTANCE_V = {
      config_.avd_lon_distance_5, config_.avd_lon_distance_4,
      config_.avd_lon_distance_3, config_.avd_lon_distance_2,
      config_.avd_lon_distance_1};
  const std::array<double, 5> _AVD_VREL_BP = {
      config_.avd_vrel_bp_5, config_.avd_vrel_bp_4, config_.avd_vrel_bp_3,
      config_.avd_vrel_bp_2, config_.avd_vrel_bp_1};

  double pre_str_dist =
      interp(avoid_obstacle.vs_lon_relative, _AVD_VREL_BP, _AVD_DISTANCE_V);

  if ((avoid_obstacle.flag == AvoidObstacleFlag::NORMAL ||
       avoid_obstacle.flag == AvoidObstacleFlag::SIDE) &&
      avoid_obstacle.first_s_to_ego != 0.) {
    if (avoid_obstacle.first_s_to_ego >= avoid_obstacle.s_to_ego) {
      smooth_lat_offset = (avoid_obstacle.first_s_to_ego -
                           (avoid_obstacle.s_to_ego - pre_str_dist)) /
                          avoid_obstacle.first_s_to_ego * lat_offset;

      if (last_avoid_info_.lat_offset * lat_offset >= 0 &&
          std::fabs(last_avoid_info_.lat_offset) > fabs(lat_offset)) {
        smooth_lat_offset = lat_offset;
      } else if (std::min(smooth_lat_offset, lat_offset) <=
                     last_avoid_info_.lat_offset &&
                 last_avoid_info_.lat_offset <=
                     std::max(smooth_lat_offset, lat_offset)) {
        smooth_lat_offset = last_avoid_info_.lat_offset;
      }

      if (is_positive) {
        smooth_lat_offset =
            std::min(std::max(last_avoid_info_.lat_offset, smooth_lat_offset),
                     lat_offset);
      } else {
        smooth_lat_offset =
            std::max(std::min(last_avoid_info_.lat_offset, smooth_lat_offset),
                     lat_offset);
      }
    } else {
      if (is_positive) {
        smooth_lat_offset =
            last_avoid_info_.lat_offset > 0
                ? std::min(last_avoid_info_.lat_offset, lat_offset)
                : 0.0;
      } else {
        smooth_lat_offset =
            last_avoid_info_.lat_offset < 0
                ? std::max(last_avoid_info_.lat_offset, lat_offset)
                : 0.0;
      }
    }
  }
  return smooth_lat_offset;
}
double LateralOffsetCalculatorV2::LateralOffsetCompensate(
    const AvoidObstacleInfo &avoid_obstacle) {
  const std::array<double, 3> near_obstacle_vrel_bp{-7.5, -3.5, 1};
  const std::array<double, 3> near_obstacle_vrel_v{0.06 * lane_width_,
                                                   0.02 * lane_width_, 0};
  const std::array<double, 3> near_obstacle_drel_v{0.08 * lane_width_,
                                                   0.05 * lane_width_, 0};
  const std::array<double, 3> near_obstacle_drel_bp{0, 20, 60};
  double plus1 = interp(avoid_obstacle.vs_lon_relative, near_obstacle_vrel_bp,
                        near_obstacle_vrel_v);
  double plus1_rel = interp(avoid_obstacle.s_to_ego, near_obstacle_drel_bp,
                            near_obstacle_drel_v);
  double lat_compensate = 0.5 * plus1 + 0.5 * plus1_rel;
  return lat_compensate;
}

double LateralOffsetCalculatorV2::DesireLateralOffsetSideWay(
    const AvoidObstacleInfo &avoid_obstacle, const AvoidWay &avoid_way,
    double coeff, double lat_compensate, double base_distance) {
  assert(avoid_way == AvoidWay::Left || avoid_way == AvoidWay::Right);

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;

  double nearest_l_to_ref =
      fabs(avoid_way == AvoidWay::Left ? avoid_obstacle.min_l_to_ref
                                       : avoid_obstacle.max_l_to_ref);

  double lat_offset;
  if (config_.nudge_value_way) {
    lat_offset = coeff * (lane_width_ - half_ego_width - nearest_l_to_ref) +
                 lat_compensate;
  } else {
    if (avoid_obstacle.is_passive) {
      base_distance = 1.0;
    }

    if (lateral_offset_decider::IsTruck(avoid_obstacle)) {
      base_distance += 0.1;
    } else if (lateral_offset_decider::IsVRU(avoid_obstacle)) {
      base_distance += 0.1;
    } else if (lateral_offset_decider::IsCone(avoid_obstacle)) {
      base_distance = 0.7;
    }
    const double pred_ts =
        clip(std::max(avoid_obstacle.s_to_ego - 4, 0.0) /
                 std::max(-avoid_obstacle.vs_lon_relative, 1e-6),
             5.0, 0.0);
    double distance_ego_to_obstacle = base_distance +
                                      0.015 * ego_cart_state_manager_->ego_v() -
                                      0.02 * pred_ts;
    lat_offset = half_ego_width + distance_ego_to_obstacle - nearest_l_to_ref;
  }

  return std::max(lat_offset, 0.0);
}

double LateralOffsetCalculatorV2::DesireLateralOffsetCenterWay(
    const AvoidObstacleInfo &avoid_obstacle_1,
    const AvoidObstacleInfo &avoid_obstacle_2, bool is_left,
    double lat_compensate_1, double lat_compensate_2) {
  double lat_offset = 0.0;
  if (is_left) {
    lat_offset =
        (avoid_obstacle_1.min_l_to_ref + avoid_obstacle_2.max_l_to_ref) * 0.5;
    double diff_lat_compen = lat_compensate_1 - lat_compensate_2;
    if (diff_lat_compen > 0) {
      if (lat_offset - diff_lat_compen - avoid_obstacle_2.max_l_to_ref >= 1.4) {
        lat_offset -= diff_lat_compen;
      } else {
        lat_offset -= 0.1;
      }
    } else if (lat_compensate_1 < lat_compensate_2) {
      if (avoid_obstacle_1.min_l_to_ref - (lat_offset - (diff_lat_compen)) >=
          1.4) {
        lat_offset -= diff_lat_compen;
      } else {
        lat_offset += 0.1;
      }
    }
  } else {
    lat_offset =
        (avoid_obstacle_1.max_l_to_ref + avoid_obstacle_2.min_l_to_ref) * 0.5;
    double diff_lat_compen = lat_compensate_1 - lat_compensate_2;
    if (diff_lat_compen > 0) {
      if (avoid_obstacle_2.min_l_to_ref - (lat_offset + (diff_lat_compen)) >=
          1.4) {
        lat_offset += diff_lat_compen;
      } else {
        lat_offset += 0.1;
      }
    } else if (lat_compensate_1 < lat_compensate_2) {
      if (lat_offset + (diff_lat_compen)-avoid_obstacle_1.max_l_to_ref >= 1.4) {
        lat_offset += diff_lat_compen;
      } else {
        lat_offset -= 0.1;
      }
    }
  }

  if (lat_offset >= 0) {
    lat_offset = std::min(lat_offset, avoid_info_.normal_left_avoid_threshold);
  } else {
    lat_offset = -std::min(std::fabs(lat_offset),
                           avoid_info_.normal_right_avoid_threshold);
  }
  return lat_offset;
}

double LateralOffsetCalculatorV2::LimitLateralOffset(
    const AvoidObstacleInfo &avoid_obstacle, double lateral_offset,
    const AvoidWay &avoid_way) {
  bool is_static_avoid_scene = session_->environmental_model()
                                   .get_lateral_obstacle()
                                   ->is_static_avoid_scene();
  if (is_static_avoid_scene) {
    lateral_offset =
        avoid_way == AvoidWay::Left
            ? std::min(lateral_offset, avoid_info_.static_right_avoid_threshold)
            : std::min(lateral_offset, avoid_info_.static_left_avoid_threshold);
  } else {
    lateral_offset =
        avoid_way == AvoidWay::Left
            ? std::min(lateral_offset, avoid_info_.normal_right_avoid_threshold)
            : std::min(lateral_offset, avoid_info_.normal_left_avoid_threshold);
  }

  if (avoid_info_.allow_front_max_opposite_offset <
      avoid_info_.allow_side_max_opposite_offset) {
    lateral_offset =
        std::min(lateral_offset, avoid_info_.allow_front_max_opposite_offset);
  } else {
    lateral_offset =
        std::min(lateral_offset, avoid_info_.allow_front_max_opposite_offset);
    const double ego_l = ego_frenet_state_.l();
    // const double ego_l = last_avoid_info_.lat_offset;
    if (avoid_way == AvoidWay::Right) {
      if (ego_l > 0) {
        if (ego_l > avoid_info_.allow_side_max_opposite_offset &&
            ego_l < lateral_offset) {
          avoid_info_.is_use_ego_position = true;
        }
        lateral_offset = std::min(
            lateral_offset,
            std::max(ego_l, avoid_info_.allow_side_max_opposite_offset));
      } else {
        lateral_offset = std::min(lateral_offset,
                                  avoid_info_.allow_side_max_opposite_offset);
      }
    } else {
      if (ego_l < 0) {
        if (-ego_l > avoid_info_.allow_side_max_opposite_offset &&
            -ego_l < lateral_offset) {
          avoid_info_.is_use_ego_position = true;
        }
        lateral_offset = std::min(
            lateral_offset,
            std::max(-ego_l, avoid_info_.allow_side_max_opposite_offset));
      } else {
        lateral_offset = std::min(lateral_offset,
                                  avoid_info_.allow_side_max_opposite_offset);
      }
    }
  }
  return lateral_offset;
}

void LateralOffsetCalculatorV2::CalcFrontMaxOppositeOffset(
    const vector<int> &obstacle_ids, bool is_left,
    const AvoidObstacleInfo &avoid_obstacle,
    std::map<std::pair<int, int>, HysteresisDecision> &hysteresis_map) {
  const auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  TrackedObject track_object;
  double front_limit_lateral_distance_tmp =
      is_left ? kDefaultLimitLateralDistance : -kDefaultLimitLateralDistance;
  int debug_front_id = kDefaultLimitId;
  for (auto id : obstacle_ids) {
    bool is_found = lateral_obstacle->find_track(id, track_object);
    if (is_found) {
      std::pair<int, int> id_pair = std::make_pair(avoid_obstacle.track_id, id);
      if (hysteresis_map.find(id_pair) == hysteresis_map.end()) {
        HysteresisDecision has_enough_space_hysteresis(3.0, 2.8);
        hysteresis_map[id_pair] = std::move(has_enough_space_hysteresis);
      }

      hysteresis_map[id_pair].SetIsValidByValue(
          std::max(avoid_obstacle.min_l_to_ref - track_object.d_max_cpath,
                   track_object.d_min_cpath - avoid_obstacle.max_l_to_ref));
      double obstacle_lat_distance =
          is_left ? track_object.d_min_cpath : track_object.d_max_cpath;
      if (hysteresis_map[id_pair].IsValid()) {
        if (track_object.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
          obstacle_lat_distance = is_left ? obstacle_lat_distance + 0.7
                                          : obstacle_lat_distance - 0.7;
        }

        if (is_left) {
          if (front_limit_lateral_distance_tmp > obstacle_lat_distance) {
            front_limit_lateral_distance_tmp = obstacle_lat_distance;
            debug_front_id = id;
          }
        } else {
          if (front_limit_lateral_distance_tmp < obstacle_lat_distance) {
            front_limit_lateral_distance_tmp = obstacle_lat_distance;
            debug_front_id = id;
          }
        }
      } else {
      }
    }
  }

  front_limit_lateral_distance_tmp =
      is_left
          ? (front_limit_lateral_distance_tmp + avoid_obstacle.max_l_to_ref) *
                0.5
          : (front_limit_lateral_distance_tmp + avoid_obstacle.min_l_to_ref) *
                0.5;
  avoid_info_.allow_front_max_opposite_offset =
      std::max(fabs(front_limit_lateral_distance_tmp), 0.0);
  avoid_info_.allow_front_max_opposite_offset_id = debug_front_id;
}

void LateralOffsetCalculatorV2::CalcSideMaxOppositeOffset(
    const vector<int> &obstacle_ids, const AvoidObstacleInfo &avoid_obstacle,
    bool is_left) {
  const auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  TrackedObject track_object;
  double limit_lateral_distance_tmp =
      is_left ? kDefaultLimitLateralDistance : -kDefaultLimitLateralDistance;
  int debug_id = kDefaultLimitId;
  for (auto id : obstacle_ids) {
    bool is_found = lateral_obstacle->find_track(id, track_object);
    if (is_found) {
      double obstacle_lat_distance =
          is_left ? track_object.d_min_cpath : track_object.d_max_cpath;
      if (is_left) {
        if (limit_lateral_distance_tmp > obstacle_lat_distance) {
          limit_lateral_distance_tmp = obstacle_lat_distance;
          debug_id = id;
        }
      } else {
        if (limit_lateral_distance_tmp < obstacle_lat_distance) {
          limit_lateral_distance_tmp = obstacle_lat_distance;
          debug_id = id;
        }
      }
    }
  }

  limit_lateral_distance_tmp =
      is_left
          ? (limit_lateral_distance_tmp + avoid_obstacle.max_l_to_ref) * 0.5
          : (limit_lateral_distance_tmp + avoid_obstacle.min_l_to_ref) * 0.5;
  avoid_info_.allow_side_max_opposite_offset =
      std::max(fabs(limit_lateral_distance_tmp), 0.0);
  // avoid_info_.allow_side_max_opposite_offset =
  // std::max(fabs(limit_lateral_distance_tmp) - 2.0, 0.0);
  avoid_info_.allow_side_max_opposite_offset_id = debug_id;
}

void LateralOffsetCalculatorV2::ResetHysteresisMap(HysteresisType type,
                                                   int avoid_obstacle_id) {
  const auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  const auto &obstacles = lateral_obstacle->all_tracks();

  switch (type) {
    case HysteresisType::IsObstacleConsideredHysteresis: {
      auto &is_obstacle_considered_hysteresis_map =
          std::get<std::map<int, HysteresisDecision>>(
              max_opposite_offset_hysteresis_maps_
                  [HysteresisType::IsObstacleConsideredHysteresis]);
      for (auto it = is_obstacle_considered_hysteresis_map.begin();
           it != is_obstacle_considered_hysteresis_map.end();) {
        int obstacle_id = it->first;
        auto iter = std::find_if(
            obstacles.begin(), obstacles.end(), [=](const TrackedObject &tr) {
              return lateral_offset_decider::IsCameraObstacle(tr) &&
                     tr.track_id == obstacle_id;
            });

        if (iter == obstacles.end()) {
          it = is_obstacle_considered_hysteresis_map.erase(it);
        } else {
          it++;
        }
      }
      break;
    }

    case HysteresisType::IsInConsiderLateralRangeHysteresis: {
      auto &is_in_consider_lateral_range_hysteresis_map =
          std::get<std::map<int, HysteresisDecision>>(
              max_opposite_offset_hysteresis_maps_
                  [HysteresisType::IsInConsiderLateralRangeHysteresis]);

      for (auto it = is_in_consider_lateral_range_hysteresis_map.begin();
           it != is_in_consider_lateral_range_hysteresis_map.end();) {
        int obstacle_id = it->first;
        auto iter = std::find_if(
            obstacles.begin(), obstacles.end(), [=](const TrackedObject &tr) {
              return lateral_offset_decider::IsCameraObstacle(tr) &&
                     tr.track_id == obstacle_id;
            });

        if (iter == obstacles.end()) {
          it = is_in_consider_lateral_range_hysteresis_map.erase(it);
        } else {
          it++;
        }
      }
      break;
    }

    case HysteresisType::EnoughSpaceHysteresis: {
      auto &enough_space_hysteresis_map =
          std::get<std::map<std::pair<int, int>, HysteresisDecision>>(
              max_opposite_offset_hysteresis_maps_
                  [HysteresisType::EnoughSpaceHysteresis]);
      if (!enough_space_hysteresis_map.empty()) {
        int last_avoid_obstacle_id =
            enough_space_hysteresis_map.begin()->first.first;
        if (last_avoid_obstacle_id != avoid_obstacle_id) {
          enough_space_hysteresis_map.clear();
        } else {
          for (auto it = enough_space_hysteresis_map.begin();
               it != enough_space_hysteresis_map.end();) {
            int obstacle_id = it->first.second;
            auto iter = std::find_if(
                obstacles.begin(), obstacles.end(),
                [=](const TrackedObject &tr) {
                  return lateral_offset_decider::IsCameraObstacle(tr) &&
                         tr.track_id == obstacle_id;
                });

            if (iter == obstacles.end()) {
              it = enough_space_hysteresis_map.erase(it);
            } else {
              it++;
            }
          }
        }
      }
      break;
    }
    default:
      break;
  }
}

void LateralOffsetCalculatorV2::PreacquireMaxOppositeOffsetIds() {
  ResetHysteresisMap(HysteresisType::IsObstacleConsideredHysteresis);
  ResetHysteresisMap(HysteresisType::IsInConsiderLateralRangeHysteresis);

  auto &is_obstacle_considered_hysteresis_map =
      std::get<std::map<int, HysteresisDecision>>(
          max_opposite_offset_hysteresis_maps_
              [HysteresisType::IsObstacleConsideredHysteresis]);
  auto &is_in_consider_lateral_range_hysteresis_map =
      std::get<std::map<int, HysteresisDecision>>(
          max_opposite_offset_hysteresis_maps_
              [HysteresisType::IsInConsiderLateralRangeHysteresis]);

  const auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  const auto &front_obstacles = lateral_obstacle->front_tracks_copy();

  front_left_max_opposite_offset_ids_.clear();
  front_right_max_opposite_offset_ids_.clear();
  side_left_max_opposite_offset_ids_.clear();
  side_right_max_opposite_offset_ids_.clear();

  for (int i = 0; i < 2; i++) {
    bool is_left = i == 0;
    for (auto &tr : front_obstacles) {
      if (!lateral_offset_decider::IsCameraObstacle(tr)) {
        continue;
      }

      if (is_left ? tr.d_min_cpath >= 0 : tr.d_max_cpath <= 0) {
        continue;
      }
      if (is_obstacle_considered_hysteresis_map.find(tr.track_id) ==
          is_obstacle_considered_hysteresis_map.end()) {
        HysteresisDecision is_obstacle_considered_hysteresis(3, 3);
        is_obstacle_considered_hysteresis_map[tr.track_id] =
            std::move(is_obstacle_considered_hysteresis);
      }

      if (is_in_consider_lateral_range_hysteresis_map.find(tr.track_id) ==
          is_in_consider_lateral_range_hysteresis_map.end()) {
        HysteresisDecision is_in_consider_lateral_range_hysteresis(0.15, -0.15);
        is_in_consider_lateral_range_hysteresis_map[tr.track_id] =
            std::move(is_in_consider_lateral_range_hysteresis);
      }

      if (!lateral_offset_decider::IsFrontObstacleConsider(
              session_, tr, !is_left, avoid_info_,
              max_opposite_offset_hysteresis_maps_)) {
        is_obstacle_considered_hysteresis_map[tr.track_id].SetInvalidCount();
      } else {
        is_obstacle_considered_hysteresis_map[tr.track_id].SetValidByCount();
      }
      if (is_obstacle_considered_hysteresis_map[tr.track_id].IsValid()) {
        is_left ? front_right_max_opposite_offset_ids_.emplace_back(tr.track_id)
                : front_left_max_opposite_offset_ids_.emplace_back(tr.track_id);
      }
    }

    const auto &side_obstacles = is_left ? lateral_obstacle->side_tracks_r()
                                         : lateral_obstacle->side_tracks_l();

    for (auto &tr : side_obstacles) {
      if (!lateral_offset_decider::IsCameraObstacle(tr)) {
        continue;
      }
      if (is_left ? tr.d_min_cpath >= 0 : tr.d_max_cpath <= 0) {
        continue;
      }
      if (is_obstacle_considered_hysteresis_map.find(tr.track_id) ==
          is_obstacle_considered_hysteresis_map.end()) {
        HysteresisDecision is_obstacle_considered_hysteresis(3, 3);
        is_obstacle_considered_hysteresis_map[tr.track_id] =
            std::move(is_obstacle_considered_hysteresis);
      }

      if (is_in_consider_lateral_range_hysteresis_map.find(tr.track_id) ==
          is_in_consider_lateral_range_hysteresis_map.end()) {
        HysteresisDecision is_in_consider_lateral_range_hysteresis(0.15, -0.15);
        is_in_consider_lateral_range_hysteresis_map[tr.track_id] =
            std::move(is_in_consider_lateral_range_hysteresis);
      }

      if (!lateral_offset_decider::IsSideObstacleConsider(
              session_, tr, !is_left, max_opposite_offset_hysteresis_maps_)) {
        is_obstacle_considered_hysteresis_map[tr.track_id].SetInvalidCount();
      } else {
        is_obstacle_considered_hysteresis_map[tr.track_id].SetValidByCount();
      }

      if (is_obstacle_considered_hysteresis_map[tr.track_id].IsValid()) {
        is_left ? side_right_max_opposite_offset_ids_.emplace_back(tr.track_id)
                : side_left_max_opposite_offset_ids_.emplace_back(tr.track_id);
      }
    }
  }
}
void LateralOffsetCalculatorV2::CalcMaxOppositeOffset(
    const AvoidObstacleInfo &avoid_obstacle, int except_id) {
  avoid_info_.allow_front_max_opposite_offset = kDefaultLimitLateralDistance;
  avoid_info_.allow_side_max_opposite_offset = kDefaultLimitLateralDistance;
  avoid_info_.allow_front_max_opposite_offset_id = kDefaultLimitId;
  avoid_info_.allow_side_max_opposite_offset_id = kDefaultLimitId;
  // InitHysteresisMap(avoid_obstacle);
  double except_lon_area = 1000;
  const auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  // Init enough_space_hysteresis_map
  auto &enough_space_hysteresis_map =
      std::get<std::map<std::pair<int, int>, HysteresisDecision>>(
          max_opposite_offset_hysteresis_maps_
              [HysteresisType::EnoughSpaceHysteresis]);

  vector<int> front_ids;
  vector<int> side_ids;

  TrackedObject except_obstacle;
  if (lateral_obstacle->find_track(except_id, except_obstacle)) {
    except_lon_area = except_obstacle.d_rel;
  }

  auto &avoid_way_select_hysteresis_map =
      std::get<std::map<std::pair<int, int>, HysteresisDecision>>(
          max_opposite_offset_hysteresis_maps_[HysteresisType::AvoidWaySelect]);
  if (avoid_obstacle.flag != AvoidObstacleFlag::INVALID) {
    bool is_left = avoid_obstacle.min_l_to_ref > 0;
    auto front_max_opposite_offset_ids =
        is_left ? front_right_max_opposite_offset_ids_
                : front_left_max_opposite_offset_ids_;
    auto side_max_opposite_offset_ids =
        is_left ? side_right_max_opposite_offset_ids_
                : side_left_max_opposite_offset_ids_;

    for (auto id : front_max_opposite_offset_ids) {
      TrackedObject tr;
      bool is_found = lateral_obstacle->find_track(id, tr);
      if (!is_found) {
        continue;
      }
      if (id == except_id || id == avoid_obstacle.track_id ||
          tr.d_rel >= except_lon_area) {
        continue;
      }

      bool is_side_way = lateral_offset_decider::AvoidWaySelectForTwoObstaclev2(
          session_, avoid_obstacle, tr);
      const int num_hysteresis = 3;
      if (!avoid_way_select_hysteresis_map.empty()) {
        if (avoid_obstacle.track_id ==
                avoid_way_select_hysteresis_map.begin()->first.first &&
            tr.track_id ==
                avoid_way_select_hysteresis_map.begin()->first.second) {
          if (is_side_way) {
            avoid_way_select_hysteresis_map.begin()->second.SetValidByCount();
          } else {
            avoid_way_select_hysteresis_map.begin()->second.SetInvalidCount();
          }
        } else {
          HysteresisDecision avoid_way_select_hysteresis(num_hysteresis,
                                                         num_hysteresis);
          if (is_side_way) {
            for (int i = 0; i < num_hysteresis; i++) {
              avoid_way_select_hysteresis.SetValidByCount();
            }
          } else {
            for (int i = 0; i < num_hysteresis; i++) {
              avoid_way_select_hysteresis.SetInvalidCount();
            }
          }
          avoid_way_select_hysteresis_map[std::make_pair(
              avoid_obstacle.track_id, tr.track_id)] =
              std::move(avoid_way_select_hysteresis);
        }
      } else {
        HysteresisDecision avoid_way_select_hysteresis(num_hysteresis,
                                                       num_hysteresis);
        if (is_side_way) {
          for (int i = 0; i < num_hysteresis; i++) {
            avoid_way_select_hysteresis.SetValidByCount();
          }
        } else {
          for (int i = 0; i < num_hysteresis; i++) {
            avoid_way_select_hysteresis.SetInvalidCount();
          }
        }
        avoid_way_select_hysteresis_map[std::make_pair(avoid_obstacle.track_id,
                                                       tr.track_id)] =
            std::move(avoid_way_select_hysteresis);
      }

      is_side_way = avoid_way_select_hysteresis_map[std::make_pair(
                                                        avoid_obstacle.track_id,
                                                        tr.track_id)]
                        .IsValid();
      if (!is_side_way) {
        front_ids.emplace_back(tr.track_id);
      }
    }

    if (config_.care_dynamic_object_t_threshold < 0.0 ||
        config_.care_static_object_t_threshold < 0.0 || !enable_bound_) {
      CalcFrontMaxOppositeOffset(front_ids, !is_left, avoid_obstacle,
                                 enough_space_hysteresis_map);
      CalcSideMaxOppositeOffset(side_max_opposite_offset_ids, avoid_obstacle,
                                !is_left);
    }

    if (std::fabs(avoid_info_.allow_front_max_opposite_offset -
                  last_avoid_info_.allow_front_max_opposite_offset) >= 0.0) {
      // avoid_info_.allow_front_max_opposite_offset =
      // avoid_info_.allow_front_max_opposite_offset;
      // avoid_info_.allow_front_max_opposite_offset_id = debug_front_id;
    } else {
      avoid_info_.allow_front_max_opposite_offset =
          last_avoid_info_.allow_front_max_opposite_offset;
      avoid_info_.allow_front_max_opposite_offset_id =
          last_avoid_info_.allow_front_max_opposite_offset_id;
    }

    if (std::fabs(avoid_info_.allow_side_max_opposite_offset -
                  last_avoid_info_.allow_side_max_opposite_offset) >= 0.0) {
    } else {
      avoid_info_.allow_side_max_opposite_offset =
          last_avoid_info_.allow_side_max_opposite_offset;
      avoid_info_.allow_side_max_opposite_offset_id =
          last_avoid_info_.allow_side_max_opposite_offset_id;
    }
  } else {
    avoid_info_.allow_front_max_opposite_offset = kDefaultLimitLateralDistance;
    avoid_info_.allow_side_max_opposite_offset = kDefaultLimitLateralDistance;
    avoid_info_.allow_front_max_opposite_offset_id = kDefaultLimitId;
    avoid_info_.allow_side_max_opposite_offset_id = kDefaultLimitId;
  }

  return;
}

void LateralOffsetCalculatorV2::PostProcess(
    const std::array<AvoidObstacleInfo, 2> &avoid_obstacles) {
  if (avoid_info_.avoid_way == AvoidWay::Left ||
      avoid_info_.avoid_way == AvoidWay::Right) {
    bool is_overlap[2] = {false, false};
    const double t_buffer = 0.9;
    if (avoid_obstacles[0].flag != AvoidObstacleFlag::INVALID) {
      is_overlap[0] = lateral_offset_decider::HasOverlap(
          session_, avoid_obstacles[0],
          std::max(t_buffer * (-avoid_obstacles[0].vs_lon_relative), 3.0), 2.0);
      if (avoid_obstacles[1].flag != AvoidObstacleFlag::INVALID) {
        is_overlap[1] = lateral_offset_decider::HasOverlap(
            session_, avoid_obstacles[1],
            std::max(t_buffer * (-avoid_obstacles[1].vs_lon_relative), 3.0),
            2.0);
      }
    }

    if (is_overlap[0] &&
        (avoid_obstacles[1].flag == AvoidObstacleFlag::INVALID ||
         avoid_id_ != avoid_obstacles[1].track_id)) {
      if ((avoid_info_.avoid_way == AvoidWay::Left &&
           avoid_info_.lat_offset - last_avoid_info_.lat_offset < 0) ||
          (avoid_info_.avoid_way == AvoidWay::Right &&
           avoid_info_.lat_offset - last_avoid_info_.lat_offset > 0)) {
        double lat_offset_compensate =
            LateralOffsetCompensate(avoid_obstacles[0]);
        double desire_lat_offset = 0.0;
        if (avoid_obstacles[0].s_to_ego <= 0.0) {
          desire_lat_offset = fabs(last_avoid_info_.lat_offset);
        } else {
          desire_lat_offset = DesireLateralOffsetSideWay(
              avoid_obstacles[0], avoid_info_.avoid_way, 0.2,
              lat_offset_compensate, 1.0);
        }

        double limit_lateral_offset = LimitLateralOffset(
            avoid_obstacles[0], desire_lat_offset, avoid_info_.avoid_way);
        if (avoid_info_.avoid_way == AvoidWay::Left) {
          avoid_info_.lat_offset =
              std::min(last_avoid_info_.lat_offset,
                       std::max(avoid_info_.lat_offset, -limit_lateral_offset));
        } else {
          avoid_info_.lat_offset =
              std::max(last_avoid_info_.lat_offset,
                       std::min(avoid_info_.lat_offset, limit_lateral_offset));
        }
        // avoid_info_.lat_offset = avoid_info_.avoid_way == AvoidWay::Left ?
        // std::max(avoid_info_.lat_offset, -limit_lateral_offset) :
        // std::min(avoid_info_.lat_offset, limit_lateral_offset);
      }
    }
  }

  // double diff_lat_offset = avoid_info_.lat_offset -
  // last_avoid_info_.lat_offset; if (fabs(diff_lat_offset) > 0.1) {
  //   if (avoid_info_.avoid_way == AvoidWay::None) {
  //     return;
  //   }

  //   // dealwith lateral_offset limited by allow_max_opposite_offset
  //   if (avoid_info_.avoid_way == last_avoid_info_.avoid_way) {
  //     if ((avoid_info_.avoid_way == AvoidWay::Left && diff_lat_offset < 0) ||
  //         (avoid_info_.avoid_way == AvoidWay::Right && diff_lat_offset > 0))
  //         {
  //       double diff_desire_lat_offset =
  //       std::min(avoid_info_.desire_lat_offset,
  //       avoid_info_.normal_avoid_threshold) -
  //             std::min(last_avoid_info_.desire_lat_offset,
  //             last_avoid_info_.normal_avoid_threshold);
  //       diff_desire_lat_offset = std::max(diff_desire_lat_offset, 0.0);

  //       if (diff_desire_lat_offset < fabs(diff_lat_offset)) {
  //         // it shows that lat_offset limited by
  //         allow_side_max_opposite_offset or allow_front_max_opposite_offset

  //       }
  //       if (fabs(last_avoid_info_.desire_lat_offset -
  //       avoid_info_.desire_lat_offset) < 0.1 ||
  //           fabs(last_avoid_info_.normal_avoid_threshold -
  //           avoid_info_.normal_avoid_threshold) < 0.1) {
  //         if (fabs(std::min(last_avoid_info_.allow_front_max_opposite_offset,
  //         last_avoid_info_.allow_side_max_opposite_offset) -
  //             std::min(avoid_info_.allow_front_max_opposite_offset,
  //             avoid_info_.allow_side_max_opposite_offset)) > 0.1) {
  //           avoid_info_.lat_offset = clip(avoid_info_.lat_offset,
  //           last_avoid_info_.lat_offset + 0.005, last_avoid_info_.lat_offset
  //           - 0.005);
  //         }
  //       }
  //     }
  //   }
  // }
  return;
}

void LateralOffsetCalculatorV2::CalLaneWidth() {
  if (1) {
    double width = 0.0;
    double preview_s = 20 + ego_frenet_state_.s();
    double start_s = 5 + ego_frenet_state_.s();
    double interval_s = 5;
    int point_num = (int)((preview_s - start_s) / interval_s) + 1;
    for (double s = start_s; s <= preview_s; s += interval_s) {
      width += flane_->width_by_s(s);
    }
    width /= point_num;
    static planning_math::MeanFilter width_filter(10);
    lane_width_ = width_filter.Update(width);
  } else {
    lane_width_ = flane_->width();
  }
}

void LateralOffsetCalculatorV2::Reset() {
  avoid_info_.Reset();
  avoid_id_ = -1;
  SaveDebugInfo();
  // TODO(clren)
}

void LateralOffsetCalculatorV2::SaveDebugInfo() {
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lateral_offset_decider_info =
      planning_debug_data->mutable_lateral_offset_decider_info();
  lateral_offset_decider_info->set_lateral_offset(avoid_info_.lat_offset);
  lateral_offset_decider_info->set_avoid_way((int)avoid_info_.avoid_way);
  lateral_offset_decider_info->set_allow_max_opposite_offset(
      avoid_info_.allow_front_max_opposite_offset);
  lateral_offset_decider_info->set_allow_max_opposite_offset_id(
      avoid_info_.allow_front_max_opposite_offset_id);

  if (flane_ != nullptr) {
    JSON_DEBUG_VALUE("lane_width", flane_->width());
  } else {
    JSON_DEBUG_VALUE("lane_width", -1000);
  }

  JSON_DEBUG_VALUE("smooth_lane_width", lane_width_);
  JSON_DEBUG_VALUE("lat_offset", avoid_info_.lat_offset);
  JSON_DEBUG_VALUE("normal_left_avoid_threshold",
                   avoid_info_.normal_left_avoid_threshold);
  JSON_DEBUG_VALUE("normal_right_avoid_threshold",
                   avoid_info_.normal_right_avoid_threshold);
  JSON_DEBUG_VALUE("avoid_way", (int)avoid_info_.avoid_way);
  JSON_DEBUG_VALUE("allow_side_max_opposite_offset",
                   avoid_info_.allow_side_max_opposite_offset);
  JSON_DEBUG_VALUE("allow_side_max_opposite_offset_id",
                   avoid_info_.allow_side_max_opposite_offset_id)
  JSON_DEBUG_VALUE("allow_front_max_opposite_offset",
                   avoid_info_.allow_front_max_opposite_offset);
  JSON_DEBUG_VALUE("allow_front_max_opposite_offset_id",
                   avoid_info_.allow_front_max_opposite_offset_id)
  JSON_DEBUG_VALUE("ego_l", ego_frenet_state_.l());
  JSON_DEBUG_VALUE("is_use_ego_position", avoid_info_.is_use_ego_position);
}

}  // namespace planning
