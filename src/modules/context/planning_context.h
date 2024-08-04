#pragma once

#include "../tasks/task_interface/gap_selcector_decider_output.h"
#include "../tasks/task_interface/general_lateral_decider_output.h"
#include "../tasks/task_interface/hpp_general_lateral_decider_output.h"
#include "../tasks/task_interface/lane_change_decider_output.h"
#include "../tasks/task_interface/lateral_motion_planner_output.h"
#include "../tasks/task_interface/longitudinal_decider_output.h"
#include "../tasks/task_interface/motion_planner_output.h"
#include "../tasks/task_interface/vision_lateral_behavior_planner_output.h"
#include "../tasks/task_interface/vision_lateral_motion_planner_output.h"
#include "../tasks/task_interface/vision_longitudinal_behavior_planner_output.h"
#include "config/basic_type.h"
#include "config/vehicle_param.h"
#include "define/lateral_behavior_planner_output.h"
#include "define/planning_status.h"
#include "lon_behavior_planner.pb.h"
#include "macro.h"
#include "planning_hmi_c.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "speed/speed_limit.h"

namespace planning {

// class EgoPlanningConfigBuilder;
class ScenarioManager;
class ObjectSelector;
class ScenarioStateMachine;
class AdaptiveCruiseControl;
class StartStopEnable;
class MrcCondition;
class LaneKeepAssistManager;
class IntelligentHeadlightControl;
class TrafficSignRecognition;

struct StatusInfo {
  std::string error_info;
  std::string debug_info;
  std::string source;
  int planning_loop = 0;
  double timestamp_ego_prediction = 0.0;
  double last_vel_limit = 100.0;
  void Clear() {
    error_info.clear();
    debug_info.clear();
    source.clear();
  }
};

class StartStopEnable;

class PlanningContext {
 public:
  const bool planning_success() const { return planning_success_; }

  bool &mutable_planning_success() { return planning_success_; }

  const bool planning_completed() const { return planning_completed_; }

  bool &mutable_planning_completed() { return planning_completed_; }

  const bool last_planning_success() const { return last_planning_success_; }

  bool &mutable_last_planning_success() { return last_planning_success_; }

  const LaneChangeDeciderOutput &lane_change_decider_output() const {
    return lane_change_decider_output_;
  }

  LaneChangeDeciderOutput &mutable_lane_change_decider_output() {
    return lane_change_decider_output_;
  }

  const GapSelectorDeciderOutput &gap_selector_decider_output() const {
    return gap_selector_decider_output_;
  }

  GapSelectorDeciderOutput &mutable_gap_selector_decider_output() {
    return gap_selector_decider_output_;
  }

  const VisionLateralBehaviorPlannerOutput &
  vision_lateral_behavior_planner_output() const {
    return vision_lateral_behavior_planner_output_;
  }

  VisionLateralBehaviorPlannerOutput &
  mutable_vision_lateral_behavior_planner_output() {
    return vision_lateral_behavior_planner_output_;
  }

  const VisionLateralMotionPlannerOutput &vision_lateral_motion_planner_output()
      const {
    return vision_lateral_motion_planner_output_;
  }

  VisionLateralMotionPlannerOutput &
  mutable_vision_lateral_motion_planner_output() {
    return vision_lateral_motion_planner_output_;
  }

  const GeneralLateralDeciderOutput &general_lateral_decider_output() const {
    return general_lateral_decider_output_;
  }

  GeneralLateralDeciderOutput &mutable_general_lateral_decider_output() {
    return general_lateral_decider_output_;
  }

  const HppGeneralLateralDeciderOutput &hpp_general_lateral_decider_output()
      const {
    return hpp_general_lateral_decider_output_;
  }

  HppGeneralLateralDeciderOutput &mutable_hpp_general_lateral_decider_output() {
    return hpp_general_lateral_decider_output_;
  }

  const VisionLongitudinalBehaviorPlannerOutput &
  vision_longitudinal_behavior_planner_output() const {
    return vision_longitudinal_behavior_planner_output_;
  }

  VisionLongitudinalBehaviorPlannerOutput &
  mutable_vision_longitudinal_behavior_planner_output() {
    return vision_longitudinal_behavior_planner_output_;
  }

  const LongitudinalDeciderOutput &longitudinal_decider_output() const {
    return longitudinal_decider_output_;
  }

  LongitudinalDeciderOutput &mutable_longitudinal_decider_output() {
    return longitudinal_decider_output_;
  }

  const MotionPlannerOutput &motion_planner_output() const {
    return motion_planner_output_;
  }

  MotionPlannerOutput &mutable_motion_planner_output() {
    return motion_planner_output_;
  }

  void feed_planning_hmi_info(
      iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info) {
    planning_hmi_info_ = planning_hmi_info;
  }
  const iflyauto::PlanningHMIOutputInfoStr planning_hmi_info() const {
    return *planning_hmi_info_;
  }
  iflyauto::PlanningHMIOutputInfoStr *mutable_planning_hmi_info() {
    return planning_hmi_info_;
  }

  const common::StartStopInfo &start_stop_result() const {
    return start_stop_result_;
  }
  common::StartStopInfo &mutable_start_stop_result() {
    return start_stop_result_;
  }

  const AdaptiveCruiseControlInfo &adaptive_cruise_control_result() const {
    return adaptive_cruise_control_result_;
  }

  AdaptiveCruiseControlInfo &mutable_adaptive_cruise_control_result() {
    return adaptive_cruise_control_result_;
  }

  const PlanningResult &planning_result() const { return planning_result_; }

  PlanningResult &mutable_planning_result() { return planning_result_; }

  PlanningResult &mutable_last_planning_result() {
    return last_planning_result_;
  }

  const PlanningResult &last_planning_result() const {
    return last_planning_result_;
  }

  const iflyauto::PlanningOutput &planning_output() const {
    return planning_output_;
  }

  iflyauto::PlanningOutput &mutable_planning_output() {
    return planning_output_;
  }

  const common::LonDecisionInfo &lon_decision_result() const {
    return lon_decision_result_;
  }
  common::LonDecisionInfo &mutable_lon_decision_result() {
    return lon_decision_result_;
  }

  const LateralBehaviorPlannerOutput &lateral_behavior_planner_output() const {
    return lateral_behavior_planner_output_;
  }

  LateralBehaviorPlannerOutput &mutable_lateral_behavior_planner_output() {
    return lateral_behavior_planner_output_;
  }

  const LateralOffsetDeciderOutput &lateral_offset_decider_output() const {
    return lateral_offset_decider_output_;
  }

  LateralOffsetDeciderOutput &mutable_lateral_offset_decider_output() {
    return lateral_offset_decider_output_;
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

  const std::shared_ptr<LaneKeepAssistManager> &lane_keep_assit_function() {
    return lane_keep_assit_ptr_;
  }
  void set_lane_keep_assit_function(
      std::shared_ptr<LaneKeepAssistManager> lane_keep_assit) {
    lane_keep_assit_ptr_ = lane_keep_assit;
  }

  const std::shared_ptr<class IntelligentHeadlightControl>
      &intelligent_headlight_control_function() {
    return intelligent_headlight_control_;
  }
  void set_intelligent_headlight_control_function(
      std::shared_ptr<class IntelligentHeadlightControl>
          intelligent_headlight_control) {
    intelligent_headlight_control_ = intelligent_headlight_control;
  }

  const std::shared_ptr<TrafficSignRecognition>
      &traffic_sign_recognition_function() {
    return traffic_sign_recognition_;
  }
  void set_traffic_sign_recognition_function(
      std::shared_ptr<TrafficSignRecognition> traffic_sign_recognition) {
    traffic_sign_recognition_ = traffic_sign_recognition;
  }

  const StatusInfo &status_info() { return status_info_; }

  StatusInfo &mutable_status_info() { return status_info_; }

  const common::LaneStatus &lane_status() { return lane_status_; }

  common::LaneStatus &mutable_lane_status() { return lane_status_; }

  const double &v_ref_cruise() const { return v_ref_cruise_; }

  void set_v_ref_cruise(const double &v_ref_cruise) {
    v_ref_cruise_ = v_ref_cruise;
  }

  const double &v_limit() const { return v_limit_; }

  void set_v_limit(const double &v_limit) { v_limit_ = v_limit; }

  void Clear() {
    planning_success_ = false;
    planning_completed_ = false;
    planning_result_.Clear();
    // planning_output_.Clear();
    memset(&planning_output_, 0, sizeof(planning_output_));
    status_info_.Clear();
  }

  void reset() {
    planning_success_ = false;
    last_planning_success_ = false;
    planning_completed_ = false;
    planning_result_ = PlanningResult();
    last_planning_result_ = PlanningResult();
    // planning_output_.Clear();
    memset(&planning_output_, 0, sizeof(planning_output_));
    adaptive_cruise_control_result_ = AdaptiveCruiseControlInfo();
    start_stop_result_.Clear();
    lon_decision_result_.Clear();
    status_info_ = StatusInfo();
  }

 private:
  bool planning_success_{false};
  bool last_planning_success_{false};
  bool planning_completed_{false};
  double v_ref_cruise_;
  double v_limit_;
  PlanningResult planning_result_;
  // std::shared_ptr<PlanningResult> last_planning_result_;
  PlanningResult last_planning_result_;
  iflyauto::PlanningOutput planning_output_;
  StatusInfo status_info_;
  LateralOffsetDeciderOutput lateral_offset_decider_output_;

  common::LaneStatus lane_status_;  // TODO: 拆分到独立的Task里面
  LateralBehaviorPlannerOutput
      lateral_behavior_planner_output_;  // TODO: 拆分到独立的Task里面

  iflyauto::PlanningHMIOutputInfoStr *planning_hmi_info_;

  // NOTE:注意Task成员变量的清空
  // lane change task pipeline
  LaneChangeDeciderOutput lane_change_decider_output_;
  GapSelectorDeciderOutput gap_selector_decider_output_;

  // lateral task pipeline
  VisionLateralBehaviorPlannerOutput vision_lateral_behavior_planner_output_;
  VisionLateralMotionPlannerOutput vision_lateral_motion_planner_output_;
  // used in HppGeneralLateralDecider and GeneralLateralDecider
  GeneralLateralDeciderOutput general_lateral_decider_output_;
  HppGeneralLateralDeciderOutput hpp_general_lateral_decider_output_;

  // longitudinal task pipeline
  VisionLongitudinalBehaviorPlannerOutput
      vision_longitudinal_behavior_planner_output_;
  // used in GeneralLongitudinalDecider and SccLonBehaviorPlanner
  LongitudinalDeciderOutput longitudinal_decider_output_;
  // used in LateralMotionPlanner, SccLongitudinalMotionPlanner,
  // LongitudinalMotionPlanner
  MotionPlannerOutput motion_planner_output_;  // TODO: 拆分到独立的Task里面

  // TODO(xjli32)：将adas功能的输出暂时保持不变
  AdaptiveCruiseControlInfo adaptive_cruise_control_result_;
  common::StartStopInfo start_stop_result_;
  common::LonDecisionInfo lon_decision_result_;

  // TODO(xjli32)：将adas相关的功能从PlanningContext移出去
  std::shared_ptr<MrcCondition> mrc_condition_ptr_;
  std::shared_ptr<StartStopEnable> start_stop_ptr_;
  std::shared_ptr<AdaptiveCruiseControl> adaptive_cruise_control_ptr_;
  std::shared_ptr<LaneKeepAssistManager> lane_keep_assit_ptr_;
  std::shared_ptr<IntelligentHeadlightControl> intelligent_headlight_control_;
  std::shared_ptr<TrafficSignRecognition> traffic_sign_recognition_;
};

}  // namespace planning