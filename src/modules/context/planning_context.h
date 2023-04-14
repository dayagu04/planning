#pragma once

#include "common/macro.h"
#include "modules/common/config/basic_type.h"
#include "modules/common/config/vehicle_param.h"
#include "modules/common/speed/speed_limit.h"
// #include "modules/context/ego_planning_config.h"
#include "modules/common/define/planning_status.h"
#include "src/modules/common/define/lateral_behavior_planner_output.h"

namespace planning {

// class EgoPlanningConfigBuilder;
class ScenarioManager;
class ScenarioStateMachine;
class AdaptiveCruiseControl;
class StartStopEnable;
class MrcCondition;

struct StatusInfo {
  std::string error_info;
  std::string debug_info;
  std::string source;
  int planning_loop = 0;
  double timestamp_ego_prediction = 0.0;
  double last_vel_limit = 100.0;
};

class StartStopEnable;

class PlanningContext {
 public:
  const bool planning_success() const { return planning_success_; }

  bool &mutable_planning_success() { return planning_success_; }

  const bool last_planning_success() const { return last_planning_success_; }

  bool &mutable_last_planning_success() { return last_planning_success_; }

  const bool replan_trajectory() const { return replan_trajectory_; }

  bool &mutable_replan_trajectory() { return replan_trajectory_; }

  const LatDecisionInfos &lateral_decisions_for_show() const {
    return lateral_decisions_for_show_;
  }

  LatDecisionInfos &mutable_lateral_decisions_for_show() {
    return lateral_decisions_for_show_;
  }

  const StartStopInfo &start_stop_result() const { return start_stop_result_; }

  StartStopInfo &mutable_start_stop_result() { return start_stop_result_; }

  const AdaptiveCruiseControlInfo &adaptive_cruise_control_result() const {
    return adaptive_cruise_control_result_;
  }

  AdaptiveCruiseControlInfo &mutable_adaptive_cruise_control_result() {
    return adaptive_cruise_control_result_;
  }

  const PlanningResult &planning_result() const { return planning_result_; }

  PlanningResult &mutable_planning_result() { return planning_result_; }

  const LonDecisionInfo &lon_decision_result() const {
    return lon_decision_result_;
  }
  LonDecisionInfo &mutable_lon_decision_result() {
    return lon_decision_result_;
  }

  const FaultDiagnosisInfo &fault_diagnosis_result() const {
    return fault_diagnosis_result_;
  }
  FaultDiagnosisInfo &mutable_fault_diagnosis_result() {
    return fault_diagnosis_result_;
  }
  const LatBehaviorInfo &lat_behavior_info() const {
    return lat_behavior_info_;
  }

  LatBehaviorInfo &mutable_lat_behavior_info() { return lat_behavior_info_; }

  const LatBehaviorStateMachineOutput &lat_behavior_state_machine_output()
      const {
    return lat_behavior_state_machine_output_;
  }

  LatBehaviorStateMachineOutput &mutable_lat_behavior_state_machine_output() {
    return lat_behavior_state_machine_output_;
  }

  const LateralBehaviorPlannerOutput &lateral_behavior_planner_output() const {
    return lateral_behavior_planner_output_;
  }

  LateralBehaviorPlannerOutput &mutable_lateral_behavior_planner_output() {
    return lateral_behavior_planner_output_;
  }

  const std::shared_ptr<AdaptiveCruiseControl>
      &adaptive_cruise_control_function() {
    return adaptive_cruise_control_ptr_;
  }
  void set_adaptive_cruise_control_function(
      std::shared_ptr<AdaptiveCruiseControl> adaptive_cruise_control) {
    adaptive_cruise_control_ptr_ = adaptive_cruise_control;
  }

  const std::shared_ptr<StartStopEnable> &start_stop() {
    return start_stop_ptr_;
  }
  void set_start_stop_enable(std::shared_ptr<StartStopEnable> start_stop_ptr) {
    start_stop_ptr_ = start_stop_ptr;
  }

  const std::shared_ptr<MrcCondition> &mrc_condition() {
    return mrc_condition_ptr_;
  }
  void set_mrc_condition(
      const std::shared_ptr<MrcCondition> &mrc_condition_ptr) {
    mrc_condition_ptr_ = mrc_condition_ptr;
  }

  void set_last_planning_result(
      const std::shared_ptr<PlanningResult> planning_result) {
    last_planning_result_ = planning_result;
  }

  const std::shared_ptr<PlanningResult> &last_planning_result() const {
    return last_planning_result_;
  }

  const common::PlanningResult &last_frame_planning_result() const {
    return last_frame_planning_result_;
  }
  common::PlanningResult &mutable_last_frame_planning_result() {
    return last_frame_planning_result_;
  }

  const VehicleParam &vehicle_param() const { return vehicle_param_; }

  void set_vehicle_param(const VehicleParam &vehicle_param) {
    vehicle_param_ = vehicle_param;
  }

  const std::shared_ptr<ScenarioStateMachine> &scenario_state_machine() const {
    return scenario_state_machine_ptr_;
  }
  void set_scenario_state_machine(
      std::shared_ptr<ScenarioStateMachine> scenario_state_machine) {
    scenario_state_machine_ptr_ = scenario_state_machine;
  }

  const StatusInfo &status_info() { return status_info_; }

  StatusInfo &mutable_status_info() { return status_info_; }

  const SpeedLimit &speed_limit() { return speed_limit_; }

  SpeedLimit &mutable_speed_limit() { return speed_limit_; }

  void clear() {
    last_planning_success_ = planning_success_;
    planning_success_ = false;
    planning_result_.raw_traj_points.clear();
    planning_result_.traj_points.clear();
  }

  void reset() {
    planning_success_ = false;
    last_planning_success_ = false;
    planning_result_ = PlanningResult();
    adaptive_cruise_control_result_ = AdaptiveCruiseControlInfo();
    start_stop_result_ = StartStopInfo();
    lon_decision_result_ = LonDecisionInfo();
    fault_diagnosis_result_ = FaultDiagnosisInfo();
    lateral_decisions_for_show_ = LatDecisionInfos();
    status_info_ = StatusInfo();
  }

 private:
  bool planning_success_{false};
  bool last_planning_success_{false};
  bool replan_trajectory_{false};
  PlanningResult planning_result_;
  common::PlanningResult last_frame_planning_result_;
  std::shared_ptr<PlanningResult> last_planning_result_;
  AdaptiveCruiseControlInfo adaptive_cruise_control_result_;
  StartStopInfo start_stop_result_;
  LonDecisionInfo lon_decision_result_;
  FaultDiagnosisInfo fault_diagnosis_result_;
  LatDecisionInfos lateral_decisions_for_show_;
  VehicleParam vehicle_param_;
  StatusInfo status_info_;
  SpeedLimit speed_limit_;
  LatBehaviorInfo lat_behavior_info_;
  LatBehaviorStateMachineOutput lat_behavior_state_machine_output_;
  LateralBehaviorPlannerOutput lateral_behavior_planner_output_;
  std::shared_ptr<ScenarioManager> scenario_manager_ptr_;
  std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ptr_;
  std::shared_ptr<MrcCondition> mrc_condition_ptr_;
  std::shared_ptr<StartStopEnable> start_stop_ptr_;
  std::shared_ptr<AdaptiveCruiseControl> adaptive_cruise_control_ptr_;
};

}  // namespace planning