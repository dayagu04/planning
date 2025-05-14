#include "lateral_offset_decider.h"
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "environmental_model_manager.h"
#include "lateral_offset_decider_info.pb.h"
#include "planning_context.h"
#include "utils/pose2d_utils.h"
namespace planning {

const double kMaxLateralOffsetChangeRate = 0.05;
const double kMaxChangeRateEgoSpeed = 10.;
LateralOffsetDecider::LateralOffsetDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
  lateral_offset_calculator_ = LateralOffsetCalculator(config_builder);
  lateral_offset_calculatorv2_ = LateralOffsetCalculatorV2(config_builder);
}

bool LateralOffsetDecider::Execute() {
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  double lat_offset = 0.0;
  auto last_fix_lane_id = session_->environmental_model()
                              .get_virtual_lane_manager()
                              ->get_last_fix_lane_id();
  auto current_fix_lane_id = session_->planning_context()
                                 .lane_change_decider_output()
                                 .fix_lane_virtual_id;
  if (last_fix_lane_id != current_fix_lane_id) {
    avoid_obstacle_maintainer5v_.Reset();
    lateral_offset_calculatorv2_.Reset();
    Reset();
  }

  avoid_obstacle_maintainer5v_.Process(session_);
  lateral_offset_calculatorv2_.Process(
      session_, avoid_obstacle_maintainer5v_.avd_obstacles(),
      avoid_obstacle_maintainer5v_.avd_sp_obstacles(),
      avoid_obstacle_maintainer5v_.dist_rblane(),
      avoid_obstacle_maintainer5v_.flag_avd());

  lat_offset = lateral_offset_calculatorv2_.lat_offset();
  SmoothLateralOffset(lat_offset);
  GenerateOutput();

  SaveDebugInfo();
  return true;
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

  const auto &avd_obstacles_history =
      avoid_obstacle_maintainer5v_.avd_obstacles_history();
  const auto &avd_obstacles = avoid_obstacle_maintainer5v_.avd_obstacles();
  if (avd_obstacles[0].flag == AvoidObstacleFlag::INVALID) {
    if (avd_obstacles_history[0].flag != AvoidObstacleFlag::INVALID &&
        avd_obstacles_history[0].s_to_ego >= 0) {
      overlap_lateral_offset_change_rate = 0.01;
    }
  }

  const double speed_ratio =
      kMaxLateralOffsetChangeRate / kMaxChangeRateEgoSpeed;
  const auto &ego_state_manager =
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

void LateralOffsetDecider::Reset() { lateral_offset_ = 0; }

void LateralOffsetDecider::SaveDebugInfo() {
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lateral_offset_decider_info =
      planning_debug_data->mutable_lateral_offset_decider_info();
  lateral_offset_decider_info->set_smooth_lateral_offset(lateral_offset_);
  JSON_DEBUG_VALUE("smooth_lateral_offset", lateral_offset_);
}

void LateralOffsetDecider::GenerateOutput() {
  LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  lateral_offset_decider_output.is_valid =
      config_.is_valid_lateral_offset && fabs(lateral_offset_) > 1e-2;
  lateral_offset_decider_output.lateral_offset = lateral_offset_;
  lateral_offset_decider_output.enable_bound =
      lateral_offset_calculatorv2_.enable_bound();
  lateral_offset_decider_output.avoid_ids.clear();

  // for hmi
  const std::array<AvoidObstacleInfo, 2> avd_obstacles =
      avoid_obstacle_maintainer5v_.avd_obstacles();
  lateral_offset_decider_output.avoid_id = -1;
  lateral_offset_decider_output.avoid_direction = 0;
  if (avd_obstacles[0].flag != AvoidObstacleFlag::INVALID) {
    if (lateral_offset_ > 0.1) {
      if (avd_obstacles[0].max_l_to_ref < 0) {
        lateral_offset_decider_output.avoid_id = avd_obstacles[0].track_id;
        lateral_offset_decider_output.avoid_direction = 1;
      } else {
        if (avd_obstacles[1].flag != AvoidObstacleFlag::INVALID &&
            avd_obstacles[1].max_l_to_ref < 0) {
          lateral_offset_decider_output.avoid_id = avd_obstacles[1].track_id;
          lateral_offset_decider_output.avoid_direction = 1;
        }
      }
    } else if (lateral_offset_ < -0.1) {
      if (avd_obstacles[0].min_l_to_ref > 0) {
        lateral_offset_decider_output.avoid_id = avd_obstacles[0].track_id;
        lateral_offset_decider_output.avoid_direction = 2;
      } else {
        if (avd_obstacles[1].flag != AvoidObstacleFlag::INVALID &&
            avd_obstacles[1].min_l_to_ref > 0) {
          lateral_offset_decider_output.avoid_id = avd_obstacles[1].track_id;
          lateral_offset_decider_output.avoid_direction = 2;
        }
      }
    }
  }
  // if (lateral_offset_decider_output.avoid_id != -1) {
  //   if (std::find(lateral_offset_decider_output.avoid_ids.begin(),
  //       lateral_offset_decider_output.avoid_ids.end(),
  //       lateral_offset_decider_output.avoid_id) == lateral_offset_decider_output.avoid_ids.end()) {
  //     lateral_offset_decider_output.avoid_ids.emplace_back(lateral_offset_decider_output.avoid_id);
  //   }
  // }
}
}  // namespace planning