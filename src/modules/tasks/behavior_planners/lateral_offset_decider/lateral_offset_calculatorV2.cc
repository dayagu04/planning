#include "lateral_offset_calculatorV2.h"
#include <iomanip>
#include <vector>
#include "common.pb.h"
#include "common/math/filter/mean_filter.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "fusion_road_c.h"
#include "lateral_behavior_planner.pb.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_offset_decider_utils.h"
#include "planning_context.h"
namespace planning {

const double kSafeDistance = 1.0;
const double kDefaultLimitLateralDistance = 10.0;
LateralOffsetCalculatorV2::LateralOffsetCalculatorV2(
    const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
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
  const auto &accident_ahead = lane_change_decider_output.accident_ahead;
  const auto &should_premove = lane_change_decider_output.should_premove;
  const auto &should_suspend = lane_change_decider_output.should_suspend;

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

  b_success =
      update(status, flag_avd, accident_ahead, should_premove, should_suspend,
             dist_rblane, avd_obstacle, avd_sp_obstacle);

  if (!b_success) {
    // TBD : add logs
  }
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralOffsetCalculatorCost", end_time - current_time);
  return b_success;
}

bool LateralOffsetCalculatorV2::update(
    int status, bool flag_avd, bool accident_ahead, bool should_premove,
    bool should_suspend, double dist_rblane,
    const std::array<AvoidObstacleInfo, 2> &avd_obstacle,
    const std::array<AvoidObstacleInfo, 2> &avd_sp_obstacle) {
  last_lat_offset_ = lat_offset_;
  if (status >= ROAD_NONE && status <= ROAD_LC_RBACK) {
    UpdateAvoidPath(status, flag_avd, accident_ahead, should_premove,
                    dist_rblane, avd_obstacle, avd_sp_obstacle);
  } else {
    lat_offset_ = 0;
  }

  SaveDebugInfo();
  return true;
}

bool LateralOffsetCalculatorV2::UpdateAvoidPath(
    int status, bool flag_avd, bool accident_ahead, bool should_premove,
    double dist_rblane, const std::array<AvoidObstacleInfo, 2> &avd_obstacle,
    const std::array<AvoidObstacleInfo, 2> &avd_sp_obstacle) {
  last_front_allow_max_opposite_offset_ = allow_front_max_opposite_offset_;
  last_side_allow_max_opposite_offset_ = allow_side_max_opposite_offset_;
  Reset();
  CalLaneWidth();
  CalculateNormalLateralOffsetThreshold();

  if (status >= ROAD_NONE && status <= ROAD_LC_RBACK) {
    if (avd_obstacle[0].flag != AvoidObstacleFlag::INVALID &&
        avd_obstacle[1].flag == AvoidObstacleFlag::INVALID) {
      LateralOffsetCalculateOneObstacle(avd_obstacle[0]);
    } else if (avd_obstacle[0].flag != AvoidObstacleFlag::INVALID &&
               avd_obstacle[1].flag != AvoidObstacleFlag::INVALID) {
      LateralOffsetCalculateTwoObstacle(avd_obstacle[0], avd_obstacle[1]);
    }
  } else {
  }
  return true;
}

// Calculate max avoid threshold
void LateralOffsetCalculatorV2::CalculateNormalLateralOffsetThreshold() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_width = vehicle_param.width + vehicle_param.width_mirror;
  int flane_lane_index = virtual_lane_manager_->get_lane_index(flane_);
  if (virtual_lane_manager_ != nullptr &&
      (flane_lane_index == 0 || flane_lane_index ==
           virtual_lane_manager_->get_lane_num() -
               1)) {  // 非最左车道;
                      // hack(clren):当前无法给出lane_border时,最右车道也减小避让距离
    normal_avoid_threshold_ = lane_width_ * 0.5 -
                              config_.nudge_buffer_road_boundary -
                              ego_width * 0.5;
  } else {
    normal_avoid_threshold_ = lane_width_ * 0.5 -
                              config_.nudge_buffer_lane_boundary -
                              ego_width * 0.5;
  }

  normal_avoid_threshold_ = std::max(normal_avoid_threshold_, 0.0);

  // TODO:clren add lateral offset limit base on curve
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

  avoid_way_ = is_left ? AvoidWay::Left : AvoidWay::Right;
  if ((avoid_obstacle.flag == AvoidObstacleFlag::NORMAL ||
       avoid_obstacle.flag == AvoidObstacleFlag::SIDE) &&
      avoid_obstacle.s_to_ego > -(ego_length + kSafeDistance)) {  // 前方障碍物
    lat_offset = DesireLateralOffsetSideWay(avoid_obstacle, is_left, 0.5,
                                            lat_offset_compensate);

    lat_offset = LimitLateralOffset(lat_offset, !is_left, &avoid_way_);
    double smooth_lateral_offset =
        SmoothLateralOffset(avoid_obstacle, is_left ? -lat_offset : lat_offset, &avoid_way_);
    lat_offset_ = smooth_lateral_offset;
  } else {
    lat_offset_ = 0.0;
  }
}

void LateralOffsetCalculatorV2::LateralOffsetCalculateTwoObstacle(
    const AvoidObstacleInfo &avoid_obstacle_1,
    const AvoidObstacleInfo &avoid_obstacle_2) {
  double t_exceed_obstacle_1;

  bool is_side_way = AvoidWaySelectForTwoObstacle(
      avoid_obstacle_1, avoid_obstacle_2, &t_exceed_obstacle_1);

  double lateral_offset = 0;
  if (t_exceed_obstacle_1 > 0) {
    if ((avoid_obstacle_1.min_l_to_ref > 0 &&
        avoid_obstacle_2.min_l_to_ref < 0) || (avoid_obstacle_1.min_l_to_ref < 0 &&
               avoid_obstacle_2.min_l_to_ref > 0)) {
      lateral_offset = DealwithTwoObstacleTwoSide(avoid_obstacle_1, avoid_obstacle_2, is_side_way);
    } else if ((avoid_obstacle_1.min_l_to_ref > 0 &&
               avoid_obstacle_2.min_l_to_ref > 0) || (avoid_obstacle_1.min_l_to_ref < 0 &&
               avoid_obstacle_2.min_l_to_ref < 0)) {
      // TODO(clren):consider min_l_to_ref > 0, max_l_to_ref < 0; use obstacle lat decision
      lateral_offset =
          DealwithTwoObstacleOneSide(avoid_obstacle_1, avoid_obstacle_2, is_side_way);
    }
  }

  double smooth_lateral_offset =
      SmoothLateralOffset(avoid_obstacle_1, lateral_offset, &avoid_way_);
  lat_offset_ = smooth_lateral_offset;
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
       !(lateral_offset_decider::IsTruck(avoid_obstacle_1) ||
         lateral_offset_decider::IsTruck(avoid_obstacle_2)) &&
       lateral_offset_decider::HasEnoughSpace(avoid_obstacle_1,
                                              avoid_obstacle_2))) {
    // it has enough space to go through the center
    lateral_offset = DesireLateralOffsetCenterWay(
        avoid_obstacle_1, avoid_obstacle_2, is_left, lat_offset_compensate_1,
        lat_offset_compensate_2);
    curr_time_ = IflyTime::Now_s();
    avoid_way_ = AvoidWay::Center;
  } else if (is_side_way && avoid_obstacle_2.max_l_to_ref < -1.5) {
    if (IflyTime::Now_s() - curr_time_ > 2) {
      // avoid obstacle_1 first, then avoid obstacle_2
      avoid_way_ = is_left ? AvoidWay::Left : AvoidWay::Right;
      CalcMaxOppositeOffset(avoid_obstacle_1, avoid_obstacle_2.track_id);
      lateral_offset = DesireLateralOffsetSideWay(avoid_obstacle_1, is_left, 0.5,
                                                  lat_offset_compensate_1);
      lateral_offset = LimitLateralOffset(lateral_offset, !is_left, &avoid_way_);
      lateral_offset = is_left ? -lateral_offset : lateral_offset;
    } else {
      // TODO(clren)
      // keep
      lateral_offset = ego_frenet_state_.l();
      avoid_way_ = AvoidWay::Ego;
    }
  } else {
    if (avoid_obstacle_1.vs_lon_relative - avoid_obstacle_2.vs_lon_relative <
        2) {
      // avoid obstacle_1, follow obstacle_2
      // TODO(clren):添加障碍物决策
      avoid_way_ = is_left ? AvoidWay::Left : AvoidWay::Right;
      CalcMaxOppositeOffset(avoid_obstacle_1, avoid_obstacle_2.track_id);
      lateral_offset = DesireLateralOffsetSideWay(avoid_obstacle_1, is_left, 0.6,
                                                  lat_offset_compensate_1);
      lateral_offset = LimitLateralOffset(lateral_offset, !is_left, &avoid_way_);
      lateral_offset = is_left ? -lateral_offset : lateral_offset;
    } else {
      // avoid obstacle_2, follow obstacle_1
      avoid_way_ = is_left ? AvoidWay::Right : AvoidWay::Left;
      CalcMaxOppositeOffset(avoid_obstacle_2, avoid_obstacle_1.track_id);
      lateral_offset = DesireLateralOffsetSideWay(avoid_obstacle_2, !is_left, 0.6,
                                                  lat_offset_compensate_1);
      lateral_offset = LimitLateralOffset(lateral_offset, is_left, &avoid_way_);
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

  avoid_way_ = is_left ? AvoidWay::Left : AvoidWay::Right;
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

    CalcMaxOppositeOffset(avoid_obstacle_1, -1, avoid_obstacle_2);
    lateral_offset = DesireLateralOffsetSideWay(*nearest_avoid_obstacle, is_left,
                                                0.5, lat_offset_compensate_1);
  } else {
    CalcMaxOppositeOffset(avoid_obstacle_1);
    lateral_offset = DesireLateralOffsetSideWay(avoid_obstacle_1, is_left, 0.5,
                                                lat_offset_compensate_1);
  }
  lateral_offset = LimitLateralOffset(lateral_offset, !is_left, &avoid_way_);
  lateral_offset = is_left ? -lateral_offset : lateral_offset;

  return lateral_offset;
}

double LateralOffsetCalculatorV2::SmoothLateralOffset(
    const AvoidObstacleInfo &avoid_obstacle, double lat_offset,
    const AvoidWay *avoid_way) {
  if (*avoid_way == AvoidWay::Ego) {
    return lat_offset;
  }
  double smooth_lat_offset = lat_offset;
  bool is_positive = lat_offset > 0;

  const double lat_control_delay = 0.5;
  const std::array<double, 3> avd_vRel_v = {20. + lat_control_delay * 7.5,
                                            5. + lat_control_delay * 2.5, 3};
  const std::array<double, 3> avd_vRel_bp = {-7.5, -2.5, 1.};

  double pre_str_dist =
      interp(avoid_obstacle.vs_lon_relative, avd_vRel_bp, avd_vRel_v);

  if ((avoid_obstacle.flag == AvoidObstacleFlag::NORMAL ||
       avoid_obstacle.flag == AvoidObstacleFlag::SIDE) &&
      avoid_obstacle.first_s_to_ego != 0.) {
    if (avoid_obstacle.first_s_to_ego >= avoid_obstacle.s_to_ego) {
      smooth_lat_offset = (avoid_obstacle.first_s_to_ego -
                           (avoid_obstacle.s_to_ego - pre_str_dist)) /
                          avoid_obstacle.first_s_to_ego * lat_offset;

      // 0.1值 需要考虑距离
      if (min(smooth_lat_offset, lat_offset) <= last_lat_offset_ &&
          last_lat_offset_ <= max(smooth_lat_offset, lat_offset)) {
        smooth_lat_offset = last_lat_offset_;
      }

      if (is_positive) {
        smooth_lat_offset = std::min(std::max(last_lat_offset_, smooth_lat_offset), lat_offset);
      } else {
        smooth_lat_offset = std::max(smooth_lat_offset, lat_offset);
      }
    } else {
      if (is_positive) {
        smooth_lat_offset =
            last_lat_offset_ > 0 ? std::min(last_lat_offset_, lat_offset) : 0.0;
      } else {
        smooth_lat_offset =
            last_lat_offset_ < 0 ? std::max(last_lat_offset_, lat_offset) : 0.0;
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
    const AvoidObstacleInfo &avoid_obstacle, bool is_left, double coeff,
    double lat_compensate) {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_width = vehicle_param.width + vehicle_param.width_mirror;

  double nearest_l_to_ref =
      fabs(is_left ? avoid_obstacle.min_l_to_ref : avoid_obstacle.max_l_to_ref);

  double lat_offset;
  if (config_.nudge_value_way) {
    lat_offset =
        coeff * (lane_width_ - ego_width * 0.5 - nearest_l_to_ref) +
        lat_compensate;
  } else {
    double base_distance = config_.base_nudge_distance;
    if (lateral_offset_decider::IsTruck(avoid_obstacle)) {
      base_distance += 0.2;
    } else if (lateral_offset_decider::IsVRU(avoid_obstacle)) {
      base_distance += 0.2;
    } else if (lateral_offset_decider::IsCone(avoid_obstacle)) {

    }
    const double pred_ts =
        clip(std::max(avoid_obstacle.s_to_ego - 4, 0.0) /
                std::min(-avoid_obstacle.vs_lon_relative, 1e-6),
            5.0, 0.0);
    double distance_ego_to_obstacle =
        base_distance + 0.015 * ego_cart_state_manager_->ego_v() - 0.02 * pred_ts;
    lat_offset = ego_width * 0.5 + distance_ego_to_obstacle - nearest_l_to_ref;
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
    lat_offset = std::min(lat_offset, normal_avoid_threshold_);
  } else {
    lat_offset = -std::min(std::fabs(lat_offset), normal_avoid_threshold_);
  }
  return lat_offset;
}

double LateralOffsetCalculatorV2::LimitLateralOffset(double lateral_offset,
                                                     bool is_left,
                                                     AvoidWay *avoid_way) {
  lateral_offset = std::min(lateral_offset, normal_avoid_threshold_);
  if (allow_front_max_opposite_offset_ < allow_side_max_opposite_offset_) {
    lateral_offset = std::min(lateral_offset, allow_front_max_opposite_offset_);
  } else {
    lateral_offset = std::min(lateral_offset, allow_front_max_opposite_offset_);
    const double ego_l = ego_frenet_state_.l();
    // const double ego_l = last_lat_offset_;
    if (is_left) {
      if (ego_l > 0) {
        if (ego_l > allow_side_max_opposite_offset_ && ego_l < lateral_offset) {
          *avoid_way = AvoidWay::Ego;
        }
        lateral_offset = std::min(
            lateral_offset, std::max(ego_l, allow_side_max_opposite_offset_));
      } else {
        lateral_offset =
            std::min(lateral_offset, allow_side_max_opposite_offset_);
      }
    } else {
      if (ego_l < 0) {
        if (-ego_l > allow_side_max_opposite_offset_ &&
            -ego_l < lateral_offset) {
          *avoid_way = AvoidWay::Ego;
        }
        lateral_offset = std::min(
            lateral_offset, std::max(-ego_l, allow_side_max_opposite_offset_));
      } else {
        lateral_offset =
            std::min(lateral_offset, allow_side_max_opposite_offset_);
      }
    }
  }
  return lateral_offset;
}

void LateralOffsetCalculatorV2::CalcMaxOppositeOffset(
    const AvoidObstacleInfo &avoid_obstacle_1, int except_id,
    const AvoidObstacleInfo &avoid_obstacle_2) {
  double front_limit_lateral_distance_tmp = kDefaultLimitLateralDistance;
  double side_limit_lateral_distance_tmp = kDefaultLimitLateralDistance;
  double front_limit_lateral_distance = kDefaultLimitLateralDistance;
  double side_limit_lateral_distance = kDefaultLimitLateralDistance;

  double ego_car_width = 2.2;
  double lat_safety_buffer = 0.2;
  const double half_lane_width = lane_width_ * 0.5;

  const auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  const auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();

  const double v_ego = ego_state->ego_v();
  const auto &front_obstacles = lateral_obstacle->front_tracks_copy();
  const auto &side_obstacles_l = lateral_obstacle->side_tracks_l();
  const auto &side_obstacles_r = lateral_obstacle->side_tracks_r();

  int debug_front_id = -1000;
  int debug_side_id = -1000;
  if (avoid_obstacle_1.flag != AvoidObstacleFlag::INVALID) {
    if (avoid_obstacle_1.min_l_to_ref >
            std::min(((ego_car_width + lat_safety_buffer) - half_lane_width),
                     0.9) &&
        (avoid_obstacle_2.flag == AvoidObstacleFlag::INVALID ||
         avoid_obstacle_2.min_l_to_ref >
             std::min(((ego_car_width + lat_safety_buffer) - half_lane_width),
                      0.9))) {
      // left obstacle
      for (auto &tr : front_obstacles) {
        if (tr.track_id == except_id) {
          continue;
        }
        if (!lateral_offset_decider::IsFrontObstacleConsider(
                session_, tr, avoid_obstacle_1, false)) {
          continue;
        }

        // TODO(clren):考虑横向速度
        if (tr.d_max_cpath > -front_limit_lateral_distance_tmp) {
          front_limit_lateral_distance_tmp = -tr.d_max_cpath;
          if (tr.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
            front_limit_lateral_distance_tmp -= 0.7;
          }
          debug_front_id = tr.track_id;
        }
      }

      front_limit_lateral_distance =
          std::fabs(front_limit_lateral_distance_tmp);
      // TODO(clren): 2.0
      front_limit_lateral_distance =
          std::max(front_limit_lateral_distance - 2.0, 0.0);
      if (avoid_obstacle_1.min_l_to_ref < 1.8) {
        if (std::fabs(avoid_obstacle_1.min_l_to_ref - 1.8) >
            front_limit_lateral_distance) {
          front_limit_lateral_distance =
              std::fabs(avoid_obstacle_1.min_l_to_ref - 1.8);
        }
      }

      for (auto &tr : side_obstacles_r) {
        if (!lateral_offset_decider::IsSideObstacleConsider(session_, tr,
                                                            false)) {
          continue;
        }

        if (tr.d_max_cpath > -side_limit_lateral_distance_tmp) {
          side_limit_lateral_distance_tmp = -tr.d_max_cpath;
          if (tr.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
            side_limit_lateral_distance_tmp -= 0.7;
          }
          debug_side_id = tr.track_id;
        }
      }
      side_limit_lateral_distance = std::fabs(side_limit_lateral_distance_tmp);
      side_limit_lateral_distance =
          std::max(side_limit_lateral_distance - 2.0, 0.0);

      if (avoid_obstacle_1.min_l_to_ref < 1.8) {
        if (std::fabs(avoid_obstacle_1.min_l_to_ref - 1.8) >
            side_limit_lateral_distance) {
          side_limit_lateral_distance =
              std::fabs(avoid_obstacle_1.min_l_to_ref - 1.8);
        }
      }

      if (std::fabs(front_limit_lateral_distance -
                    last_front_allow_max_opposite_offset_) > 0.1) {
        allow_front_max_opposite_offset_ = front_limit_lateral_distance;
        allow_front_max_opposite_offset_id_ = debug_front_id;
      } else {
        allow_front_max_opposite_offset_ =
            last_front_allow_max_opposite_offset_;
      }

      if (std::fabs(side_limit_lateral_distance -
                    last_side_allow_max_opposite_offset_) > 0.1) {
        allow_side_max_opposite_offset_ = side_limit_lateral_distance;
        allow_side_max_opposite_offset_id_ = debug_side_id;
      } else {
        allow_side_max_opposite_offset_ = last_side_allow_max_opposite_offset_;
      }
    } else if (avoid_obstacle_1.min_l_to_ref < 0 &&
               (avoid_obstacle_2.flag == AvoidObstacleFlag::INVALID ||
                (avoid_obstacle_2.flag != AvoidObstacleFlag::LEAD_ONE &&
                 avoid_obstacle_2.min_l_to_ref < 0))) {
      // right obstacle
      for (auto &tr : front_obstacles) {
        if (tr.track_id == except_id) {
          continue;
        }

        if (!lateral_offset_decider::IsFrontObstacleConsider(
                session_, tr, avoid_obstacle_1, true)) {
          continue;
        }

        if (tr.d_min_cpath < front_limit_lateral_distance_tmp) {
          front_limit_lateral_distance_tmp = tr.d_min_cpath;
          if (tr.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
            front_limit_lateral_distance_tmp += 0.7;
          }
          front_limit_lateral_distance =
              std::fabs(front_limit_lateral_distance_tmp);
          debug_front_id = tr.track_id;
        } else if (avoid_obstacle_1.max_l_to_ref >= 0 && tr.d_max_cpath < -1 &&
                   tr.d_max_cpath >
                       -front_limit_lateral_distance_tmp &&  //???什么场景
                   tr.d_rel < std::max(std::min(std::fabs(tr.v_rel * 15), 60.0),
                                       20.0)) {
          front_limit_lateral_distance_tmp = -tr.d_max_cpath;
          if (tr.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
            front_limit_lateral_distance_tmp -= 0.7;
          }
          front_limit_lateral_distance =
              std::fabs(front_limit_lateral_distance_tmp);
          debug_front_id = tr.track_id;
        }
      }

      front_limit_lateral_distance = std::max(
          front_limit_lateral_distance - 2.0, 0.0);  // 可以考虑对弱势群体做区分
      if (avoid_obstacle_1.max_l_to_ref < 0 &&
          avoid_obstacle_1.max_l_to_ref > -1.8) {
        if (std::fabs(avoid_obstacle_1.max_l_to_ref + 1.8) >
            front_limit_lateral_distance) {
          front_limit_lateral_distance =
              std::fabs(avoid_obstacle_1.max_l_to_ref + 1.8);
        }
      }

      if (avoid_obstacle_1.max_l_to_ref < 0) {
        for (auto &tr : side_obstacles_l) {
          if (!lateral_offset_decider::IsSideObstacleConsider(session_, tr,
                                                              true)) {
            continue;
          }
          if (tr.d_min_cpath < side_limit_lateral_distance_tmp) {
            side_limit_lateral_distance_tmp = tr.d_min_cpath;
            if (tr.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
              side_limit_lateral_distance_tmp += 0.7;
            }
            side_limit_lateral_distance =
                std::fabs(side_limit_lateral_distance_tmp);
            debug_side_id = tr.track_id;
          }
        }
      }

      side_limit_lateral_distance =
          std::max(side_limit_lateral_distance - 2.0, 0.0);

      if (avoid_obstacle_1.max_l_to_ref > -1.8 &&
          avoid_obstacle_1.max_l_to_ref < 0) {
        if (std::fabs(avoid_obstacle_1.max_l_to_ref + 1.8) >
            side_limit_lateral_distance) {
          side_limit_lateral_distance =
              std::fabs(avoid_obstacle_1.max_l_to_ref + 1.8);
        }
      }

      if (std::fabs(front_limit_lateral_distance -
                    last_front_allow_max_opposite_offset_) > 0.1) {
        allow_front_max_opposite_offset_ = front_limit_lateral_distance;
        allow_front_max_opposite_offset_id_ = debug_front_id;
      } else {
        allow_front_max_opposite_offset_ =
            last_front_allow_max_opposite_offset_;
      }

      if (std::fabs(side_limit_lateral_distance -
                    last_side_allow_max_opposite_offset_) > 0.1) {
        allow_side_max_opposite_offset_ = side_limit_lateral_distance;
        allow_side_max_opposite_offset_id_ = debug_side_id;
      } else {
        allow_side_max_opposite_offset_ = last_side_allow_max_opposite_offset_;
      }
    }
  } else {
    allow_front_max_opposite_offset_ = kDefaultLimitLateralDistance;
    allow_side_max_opposite_offset_ = kDefaultLimitLateralDistance;
    allow_front_max_opposite_offset_id_ = -1;
    allow_side_max_opposite_offset_id_ = -1;
  }
  return;
}

void LateralOffsetCalculatorV2::PostProcess() { return; }

void LateralOffsetCalculatorV2::CalLaneWidth() {
  if (1) {
    double width = 0.0;
    double preview_s = 20;
    double start_s = 5;
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
  lat_offset_ = 0.0;
  avoid_way_ = AvoidWay::None;
  allow_front_max_opposite_offset_ = kDefaultLimitLateralDistance;
  allow_side_max_opposite_offset_ = kDefaultLimitLateralDistance;
  SaveDebugInfo();
  // TODO(clren)
}

void LateralOffsetCalculatorV2::SaveDebugInfo() {
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lateral_offset_decider_info =
      planning_debug_data->mutable_lateral_offset_decider_info();
  lateral_offset_decider_info->set_lateral_offset(lat_offset_);
  lateral_offset_decider_info->set_avoid_way((int)avoid_way_);
  lateral_offset_decider_info->set_allow_max_opposite_offset(
      allow_front_max_opposite_offset_);
  lateral_offset_decider_info->set_allow_max_opposite_offset_id(
      allow_front_max_opposite_offset_id_);

  JSON_DEBUG_VALUE("lane_width", flane_->width());
  JSON_DEBUG_VALUE("smooth_lane_width", lane_width_);
  JSON_DEBUG_VALUE("lat_offset", lat_offset_);
  JSON_DEBUG_VALUE("normal_avoid_threshold", normal_avoid_threshold_);
  JSON_DEBUG_VALUE("avoid_way", (int)avoid_way_);
  JSON_DEBUG_VALUE("allow_side_max_opposite_offset",
                   allow_side_max_opposite_offset_);
  JSON_DEBUG_VALUE("allow_side_max_opposite_offset_id",
                   allow_side_max_opposite_offset_id_)
  JSON_DEBUG_VALUE("allow_front_max_opposite_offset",
                   allow_front_max_opposite_offset_);
  JSON_DEBUG_VALUE("allow_front_max_opposite_offset_id",
                   allow_front_max_opposite_offset_id_)
  JSON_DEBUG_VALUE("ego_l", ego_frenet_state_.l());
}

}  // namespace planning
