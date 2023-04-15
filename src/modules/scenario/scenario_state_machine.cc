#include "modules/scenario/scenario_state_machine.h"

#include <limits>
#include <numeric>

#include "modules/common/utils/pose2d_utils.h"
#include "modules/context/vehicle_config_context.h"
#include "modules/context/ego_state_manager.h"
#include "modules/context/obstacle_manager.h"
#include "modules/context/reference_path.h"
#include "modules/context/reference_path_manager.h"

namespace planning {

ScenarioStateMachine::ScenarioStateMachine(
    const EgoPlanningConfigBuilder *config_builder,
    planning::framework::Session *session)
    : scenario_fsm_(fsm_context_), session_(session) {
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  lc_lane_mgr_ =
      std::make_shared<LaneChangeLaneManager>(virtual_lane_mgr, session);
  lc_req_mgr_ = std::make_shared<LaneChangeRequestManager>(
      session, config_builder, virtual_lane_mgr, lc_lane_mgr_);
}

ScenarioStateMachine::~ScenarioStateMachine() {}

void ScenarioStateMachine::init() {
  fsm_context_.state_machine = shared_from_this();
  fsm_context_.state = ROAD_NONE;
  fsm_context_.external = false;
  fsm_context_.name = "ROAD_NONE";
  fsm_context_.session = session_;
  scenario_fsm_.changeTo<RoadState::None>();
  // scenario_fsm_.update(); // TODO fix null frame
  lc_lane_mgr_->reset_lc_lanes();
}

bool ScenarioStateMachine::update(planning::framework::Frame *frame) {
  fsm_context_.frame = frame;
  bool active = session_->environmental_model().GetVehicleDbwStatus();

  if (!session_->mutable_planning_context()->last_planning_success()) {
    reset_state_machine();
  }
  session_->mutable_planning_context()->mutable_planning_success() =
      false;

  update_scenario();  // cruise or low speed or ...

  if (active == false) {
    lc_req_mgr_->finish_request();
    map_turn_signal_ = NO_CHANGE;
    reset_state_machine();
  }

  if (!fsm_context_.initialized) {
    reset_state_machine();
    fsm_context_.initialized = true;
  }

  if (active) {
    LOG_DEBUG("[scenario_state_machine] active");
    if (scenario_ == SCENARIO_CRUISE) {
      // update lc_req_mgr_
      lc_req_mgr_->update(fsm_context_.state,
                          session_->environmental_model().IsOnRoute());
      gen_map_turn_signal();
      update_state_machine();
      post_process();
    } else {
      return false;
    }
  } else {
    LOG_DEBUG("[scenario_state_machine] not active");
    if (scenario_ == SCENARIO_CRUISE) {
      // update lc_req_mgr_
      lc_req_mgr_->update(fsm_context_.state,
                          session_->environmental_model().IsOnRoute());
      gen_map_turn_signal();
      update_state_machine();
      post_process();
    } else {
      return false;
    }
    // post_process();
  }
  return true;
}

void ScenarioStateMachine::update_state_machine() {

  if (fsm_context_.state >= ROAD_NONE && fsm_context_.state <= INTER_UT_NONE) {
    if (scenario_ == SCENARIO_CRUISE) {
      scenario_fsm_.update();
    } else {
      LOG_ERROR("Scenario Not Implemented!");
      change_state_external<RoadState::None>();
      lc_lane_mgr_->reset_lc_lanes();
      map_turn_signal_ = NO_CHANGE;
    }
  } else if (fsm_context_.state > INTER_UT_NONE) {
    LOG_ERROR("State Not Implemented!");
    change_state_external<RoadState::None>();
    lc_lane_mgr_->reset_lc_lanes();
    map_turn_signal_ = NO_CHANGE;
  }
}

void ScenarioStateMachine::reset_state_machine() {
  change_state_external<RoadState::None>();
  lc_lane_mgr_->reset_lc_lanes();
  lc_req_mgr_->finish_request();
}

void ScenarioStateMachine::update_scenario() { scenario_ = SCENARIO_CRUISE; }

void ScenarioStateMachine::post_process() {
  RequestType turn_signal_this_frame = (map_turn_signal_ == NO_CHANGE)
                                           ? lc_req_mgr_->turn_signal()
                                           : map_turn_signal_;
  turn_signal_this_frame = (turn_signal_this_frame == NO_CHANGE)
                               ? merge_split_turn_signal_
                               : turn_signal_this_frame;
  if (turn_signal_ == NO_CHANGE && turn_signal_this_frame != NO_CHANGE) {
    turn_signal_on_time_ = get_system_time();
  }
  turn_signal_ = turn_signal_this_frame;
  session_->mutable_planning_context()
      ->mutable_planning_result()
      .turn_signal = turn_signal_;

  LOG_DEBUG(
      "[ScenarioStateMachine] turn_signal_on_time_ %f map_turn_signal: %d, "
      "lc_turn_signal: %d, merge_split_turn_signal: %d, turn_signal_: %d",
      turn_signal_on_time_, (int)map_turn_signal_,
      (int)lc_req_mgr_->turn_signal(), (int)merge_split_turn_signal_,
      (int)turn_signal_);
}

void ScenarioStateMachine::gen_map_turn_signal() {
  std::shared_ptr<VirtualLaneManager> map_info_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
}

void ScenarioStateMachine::gen_merge_split_turn_signal() {

}

bool ScenarioStateMachine::gap_available(RequestType direction,
                                         std::vector<int> &overtake_obstacles,
                                         std::vector<int> &yield_obstacles) {
  bool b_gap_available{true};
  return b_gap_available;
}

bool ScenarioStateMachine::check_lc_change_finish(RequestType direction) {
  bool lc_change_finish{false};
  return lc_change_finish;
}

bool ScenarioStateMachine::check_lc_back_finish(RequestType direction) {
  bool lc_back_finish{false};
  return lc_back_finish;
}

}  // namespace planning
