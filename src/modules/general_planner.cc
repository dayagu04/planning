#include "src/modules/general_planner.h"

#include <math.h>

#include "src/modules/scc_function/adaptive_cruise_control.h"
#include "src/modules/scc_function/mrc_condition.h"
#include "src/modules/scc_function/start_stop_enable.h"
#include "src/modules/scenario/scenario_state_machine.h"
#include "src/modules/scenario/lateral_behavior_object_selector.h"
// #include "mjson/mjson.hpp"

namespace planning {
namespace planner {

GeneralPlanner::GeneralPlanner() { LOG_DEBUG("GeneralPlanner created"); }

void GeneralPlanner::Init(planning::framework::Session *session) {
  session_ = session;
  InitContext();
}

void GeneralPlanner::InitContext() {
  planning::common::SceneType scene_type = session_->get_scene_type();
  auto config_builder =
      session_->environmental_model().config_builder(scene_type);

  // init state machine
  scenario_state_machine_ =
      std::make_shared<ScenarioStateMachine>(config_builder, session_);
  scenario_state_machine_->init();
  session_->mutable_planning_context()->set_scenario_state_machine(
      scenario_state_machine_);
  session_->mutable_planning_context()->set_vehicle_param(
      session_->vehicel_config_context().get_vehicle_param());
  
  object_selector_ = std::make_shared<ObjectSelector>(config_builder, session_);
  session_->mutable_planning_context()->set_object_selector(
      object_selector_);

  // SCC Function
  adaptive_cruise_control_ =
      std::make_shared<AdaptiveCruiseControl>(config_builder, session_);
  session_->mutable_planning_context()->set_adaptive_cruise_control_function(
      adaptive_cruise_control_);

  start_stop_ = std::make_shared<StartStopEnable>(config_builder, session_);
  session_->mutable_planning_context()->set_start_stop_enable(start_stop_);

  mrc_condition_ = std::make_shared<MrcCondition>(config_builder, session_);
  session_->mutable_planning_context()->set_mrc_condition(mrc_condition_);
}

bool GeneralPlanner::Run(planning::framework::Frame *frame) {
  frame_ = frame;
  // Step 1) clear info

  auto start_time = IflyTime::Now_ms();
  common::PlanningStatus *pnc_status =
      session_->mutable_planning_output_context()->mutable_planning_status();
  common::PlanningResult &pnc_result = pnc_status->planning_result;
  session_->mutable_planning_context()->clear();

  bool dbw_status = session_->environmental_model().GetVehicleDbwStatus();

  auto updated_time = IflyTime::Now_ms();
  LOG_DEBUG("update time:%f", updated_time - start_time);

  object_selector_->update(session_->planning_context().lat_behavior_state_machine_output().curr_state, 
                           session_->planning_context().scenario_state_machine()->get_start_move_dist_lane(),
                           false, 80., false, false, false, false, false, -1);

  // Step 2) update state machine
  (void)scenario_state_machine_->update(frame_);
  auto state_machine_time = IflyTime::Now_ms();
  LOG_DEBUG("state machine update time:%f", state_machine_time - updated_time);

  // Step 3) copy planning result
  auto &ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();
  bool planning_failed =
      !session_->mutable_planning_context()->planning_success();
  // Step 4) check vehicle state is steady to getin automode

  // Step 5) Check the state of the location and control difference within 2 s
  // when need to disable auto

  // Step 6) check proposal match
  if (planning_failed) {
    // TODO：backup
    LOG_DEBUG("general_planner failed");
  } else {
    // SetPlanningResult(ego_planning_result, pnc_result);
    // session_->mutable_planning_context()
    //     ->set_last_planning_result(
    //         std::make_shared<PlanningResult>(ego_planning_result));
    // session_->mutable_planning_context()
    //     ->planning_result_manager()
    //     ->add_planning_result(ego_planning_result);
    LOG_DEBUG("general_planner run success");
  }

  auto exit_time = IflyTime::Now_ms();
  LOG_DEBUG("copy_result time:%f", exit_time - state_machine_time);
  return (not planning_failed);
}

void GeneralPlanner::SetPlanningResult(
    const PlanningResult &ego_planning_result,
    common::PlanningResult &pnc_result) {
  const auto &traj_points = ego_planning_result.traj_points;

  ClearPlanningResult(pnc_result);
  if (ego_planning_result.turn_signal == NO_CHANGE) {
    pnc_result.planning_output.mutable_turn_signal_command()->set_turn_signal_value(Common::TurnSignalType::TURN_SIGNAL_TYPE_NONE);
  } else if (ego_planning_result.turn_signal == LEFT_CHANGE) {
    pnc_result.planning_output.mutable_turn_signal_command()->set_turn_signal_value(Common::TurnSignalType::TURN_SIGNAL_TYPE_LEFT);
  } else {
    pnc_result.planning_output.mutable_turn_signal_command()->set_turn_signal_value(Common::TurnSignalType::TURN_SIGNAL_TYPE_RIGHT);
  }
  LOG_DEBUG("turn_signal: %d", (int)pnc_result.planning_output.turn_signal_command().turn_signal_value());

  // add stitcher trajectory
}

void GeneralPlanner::ClearPlanningResult(common::PlanningResult &pnc_result) {
  pnc_result.traj_vel_array.clear();
  pnc_result.traj_acceleration.clear();
  // pnc_result.traj_pose_array.clear();
  // pnc_result.jerk_output.clear();
}

}  // namespace planner
}  // namespace planning
