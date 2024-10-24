#include "longitudinal_decision_decider.h"

#include <type_traits>

#include "src/modules/context/environmental_model.h"
#include "src/modules/context/planning_context.h"

namespace planning {

LongitudinalDecisionDecider::LongitudinalDecisionDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LongitudinalDecisionDecider";
  // 读取配置文件
  config_ = config_builder->cast<EgoPlanningConfig>();
  plan_time_ = config_.trajectory_time_length;
  dt_ = config_.planning_dt;
  plan_points_num_ = static_cast<size_t>(plan_time_ / dt_) + 1;
}

void LongitudinalDecisionDecider::Reset() {
  cruise_accelerate_count_.first = 0.0;
  cruise_accelerate_count_.second = 0.0;
}

bool LongitudinalDecisionDecider::Execute() {
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  bool need_reset = false;  // binwang33 TBD: Add reset conditions
  if (need_reset) {
    Reset();
  }

  const auto &environmental_model = session_->environmental_model();
  DetermineKinematicBoundForCruiseScenario();

  // 对于横向侵入无避让的障碍物, 放入ST图中
  // UpdateInvadeNeighborResults();

  // 存储debug信息，这种形式挺好的
  MakeDebugMessage();

  return true;
}

void LongitudinalDecisionDecider::DetermineKinematicBoundForCruiseScenario() {
  constexpr double kCruiseSpeedMinThd = 60.0 / 3.6;
  constexpr double kEgoSpeedWithCruiseSpeedDiffThd = 15.0 / 3.6;

  const auto &environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();

  // 获取init point
  const auto &plannig_init_point = ego_state_mgr->planning_init_point();
  const double ego_vel = plannig_init_point.v;
  bool can_increase_acc_bound = true;

  // 1.lane change
  const auto lane_change_status =
      session_->planning_context().lane_change_decider_output().curr_state;
  const bool is_in_lane_keeping =
      lane_change_status == StateMachineLaneChangeStatus::kLaneKeeping;
  if (!is_in_lane_keeping) {
    can_increase_acc_bound = false;
  }

  // 2.cruise speed
  const double cruise_speed = ego_state_mgr->ego_v_cruise();
  if (cruise_speed < kCruiseSpeedMinThd) {
    can_increase_acc_bound = false;
  }

  // 3.speed diff
  const double ego_vel_diff_cruise = ego_vel - cruise_speed;
  if (ego_vel_diff_cruise > -kEgoSpeedWithCruiseSpeedDiffThd) {
    can_increase_acc_bound = false;
  }

  // 4.cutin

  // 5.path curv

  // 6.agents average speed

  // 7.max_acc_curv VS st_corridor
  // 需要获取st信息，判断最大减速度时，和前车是否安全，前车从 agents_headway_map
  // 中获取
}

void LongitudinalDecisionDecider::MakeDebugMessage() {
  // auto planning_debug_msg = planning_data->mutable_planning_debug_message();
  // if (planning_debug_msg == nullptr) {
  // }
  // auto* ptr_debug_string = planning_debug_msg->mutable_debug_string();
  // if (ptr_debug_string == nullptr) {
  //   return;
  // }

  // std::string cruise_acc_count_info = " DetermineKinematicBound: count ";
  // cruise_acc_count_info += std::to_string(cruise_accelerate_count_.first);
  // cruise_acc_count_info += " increase_acc ";
  // cruise_acc_count_info += std::to_string(cruise_accelerate_count_.second);
  // std::stringstream acc_bound;
  // acc_bound << std::fixed << std::setprecision(2) << " acc_bound "
  //           << planning_data->decision_output()
  //                  .longitudinal_decision_decider_output()
  //                  .determined_cruise_bound()
  //                  .acc_positive_mps2;
  // cruise_acc_count_info += acc_bound.str();
  // ptr_debug_string->append(cruise_acc_count_info);
}

}  // namespace planning
