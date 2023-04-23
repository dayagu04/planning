#include "modules/scenario/normal_road_state.h"

#include "modules/context/reference_path_manager.h"
#include "modules/scenario/scenario_state_machine.h"
#include "src/common/ifly_time.h"

namespace planning {

void RoadBase::prepare_for_none_state(
    std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
    std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
    std::vector<ScenarioStateEnum> &candidate_states,
    std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers) {
  auto lc_lane_manager_tmp =
      std::make_shared<LaneChangeLaneManager>(lc_lane_manager);
  lc_lane_manager_tmp->reset_lc_lanes();
  candidate_states.push_back(ROAD_NONE);
  lc_lane_managers.emplace_back(lc_lane_manager_tmp);
}

void RoadBase::prepare_for_change_state(
    std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
    std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
    std::vector<ScenarioStateEnum> &candidate_states,
    std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers) {
  auto lc_lane_manager_tmp =
      std::make_shared<LaneChangeLaneManager>(lc_lane_manager);
  if (lc_lane_manager_tmp->has_target_lane()) {
    lc_lane_manager_tmp->set_fix_lane_to_target();
  } else {
    int target_lane_virtual_id = lc_req_manager->target_lane_virtual_id();
    lc_lane_manager_tmp->assign_lc_lanes(target_lane_virtual_id);
    lc_lane_manager_tmp->set_fix_lane_to_target();
  }

  // make sure origin lane exists, fitting reference line may need it.
  if (!lc_lane_manager_tmp->has_origin_lane()) {
    lc_lane_manager_tmp->reset_origin_lane();
  }

  if (!lc_lane_manager_tmp->has_target_lane() &&
      !lc_lane_manager_tmp->has_origin_lane()) {
    lc_lane_manager_tmp->reset_origin_lane();
    lc_lane_manager_tmp->set_fix_lane_to_origin();
    candidate_states.push_back(ROAD_NONE);
  } else if (!lc_lane_manager_tmp->has_target_lane()) {
    lc_lane_manager_tmp->set_fix_lane_to_origin();
    if (lc_req_manager->request() == LEFT_CHANGE) {
      candidate_states.push_back(ROAD_LC_LBACK);
    } else if (lc_req_manager->request() == RIGHT_CHANGE) {
      candidate_states.push_back(ROAD_LC_RBACK);
    }
  } else {
    if (!lc_lane_manager_tmp->has_origin_lane()) {
      lc_lane_manager_tmp->reset_origin_lane();
    }

    if (lc_req_manager->request() == LEFT_CHANGE) {
      candidate_states.push_back(ROAD_LC_LCHANGE);
    } else if (lc_req_manager->request() == RIGHT_CHANGE) {
      candidate_states.push_back(ROAD_LC_RCHANGE);
    }
  }

  lc_lane_managers.emplace_back(lc_lane_manager_tmp);
}

void RoadBase::prepare_for_wait_state(
    std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
    std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
    std::vector<ScenarioStateEnum> &candidate_states,
    std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers) {
  auto lc_lane_manager_tmp =
      std::make_shared<LaneChangeLaneManager>(lc_lane_manager);
  if (lc_lane_manager_tmp->has_origin_lane()) {
    lc_lane_manager_tmp->set_fix_lane_to_origin();
    if (lc_req_manager->request() == LEFT_CHANGE) {
      candidate_states.push_back(ROAD_LC_LWAIT);
    } else if (lc_req_manager->request() == RIGHT_CHANGE) {
      candidate_states.push_back(ROAD_LC_RWAIT);
    }
  } else {
    lc_lane_manager_tmp->reset_lc_lanes();
    candidate_states.push_back(ROAD_NONE);
  }
  lc_lane_managers.emplace_back(lc_lane_manager_tmp);
}

void RoadBase::prepare_for_back_state(
    std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
    std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
    std::vector<ScenarioStateEnum> &candidate_states,
    std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers) {
  auto lc_lane_manager_tmp =
      std::make_shared<LaneChangeLaneManager>(lc_lane_manager);
  if (lc_lane_manager_tmp->has_origin_lane()) {
    lc_lane_manager_tmp->set_fix_lane_to_origin();
    if (lc_req_manager->request() == LEFT_CHANGE) {
      candidate_states.push_back(ROAD_LC_LBACK);
    } else if (lc_req_manager->request() == RIGHT_CHANGE) {
      candidate_states.push_back(ROAD_LC_RBACK);
    }
  } else {
    lc_lane_manager_tmp->reset_lc_lanes();
    candidate_states.push_back(ROAD_NONE);
  }
  lc_lane_managers.emplace_back(lc_lane_manager_tmp);
}

void RoadState::None::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  std::shared_ptr<ScenarioStateMachine> state_machine = context.state_machine;
  std::shared_ptr<LaneChangeRequestManager> lc_req_manager =
      state_machine->get_lane_change_request_manager();

  std::vector<ScenarioStateEnum> candidate_states;
  std::vector<std::shared_ptr<LaneChangeLaneManager>> lc_lane_managers;

  auto lc_lane_manager = std::make_shared<LaneChangeLaneManager>(
      state_machine->get_lane_change_lane_manager());

  RequestType lc_request = lc_req_manager->request();
  int target_lane_virtual_id = lc_req_manager->target_lane_virtual_id();
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lc_info;
  if (lc_request == LEFT_CHANGE || lc_request == RIGHT_CHANGE) {
    lc_lane_manager->assign_lc_lanes(target_lane_virtual_id);
    prepare_for_wait_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
    // prepare_for_change_state(lc_lane_manager, lc_req_manager,
    // candidate_states,
    //                        lc_lane_managers);
  } else {
    prepare_for_none_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
  }
  state_machine->generate_state_machine_output(lc_info);

  for (size_t i = 0; i < candidate_states.size(); ++i) {
    int o_lane_virtual_id = lc_lane_managers[i]->olane_virtual_id();
    int fix_lane_virtual_id = lc_lane_managers[i]->flane_virtual_id();
    auto lc_request_source = lc_req_manager->request_source();
    bool bind_end_state{true};
    transition_contexts.emplace_back(StateTransitionContext{
        static_cast<ScenarioStateEnum>(context.state), candidate_states[i],
        lc_request_source, o_lane_virtual_id, fix_lane_virtual_id,
        lc_lane_managers[i], bind_end_state});
  }
  for (auto &transition_context : transition_contexts) {
    if (transition_context.target_state == ROAD_LC_LWAIT ||
        transition_context.target_state == ROAD_LC_RWAIT ||
        transition_context.target_state == ROAD_LC_LBACK ||
        transition_context.target_state == ROAD_LC_RBACK) {
      transition_context.overtake_obstacles = overtake_obstacles;
      transition_context.yield_obstacles = yield_obstacles;
    }
  }
}

void RoadState::LC::process_wait(FsmContext &context,
                                 StateTransitionContexts &transition_contexts) {
  std::shared_ptr<ScenarioStateMachine> state_machine = context.state_machine;
  std::shared_ptr<LaneChangeRequestManager> lc_req_manager =
      state_machine->get_lane_change_request_manager();

  std::vector<ScenarioStateEnum> candidate_states;
  std::vector<std::shared_ptr<LaneChangeLaneManager>> lc_lane_managers;

  auto lc_lane_manager = std::make_shared<LaneChangeLaneManager>(
      state_machine->get_lane_change_lane_manager());

  std::shared_ptr<EnvironmentalModel> environmental_model =
      context.environmental_model;

  RequestType lc_request = lc_req_manager->request();
  RequestSource lc_source = lc_req_manager->request_source();
  bool aggressive_change{lc_req_manager->AggressiveChange()};
  bool gap_available{true};
  bool hdmap_valid = environmental_model->get_hdmap_valid();
  double lc_tstart = lc_req_manager->GetReqStartTime(lc_source);
  double delay_time = 2.0;  // TODO(Rui):后面根据请求来源做成配置项
  double curr_time = IflyTime::Now_ms();  // 注意
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lane_change_info;
  if (lc_request != NO_CHANGE && lc_request == context.direction) {
    int target_lane_virtual_id = lc_req_manager->target_lane_virtual_id();
    if (!lc_lane_manager->has_target_lane() ||
        lc_lane_manager->tlane_virtual_id() != target_lane_virtual_id) {
      lc_lane_manager->assign_lc_lanes(target_lane_virtual_id);
    }
    gap_available = state_machine->gap_available(lc_request, overtake_obstacles,
                                                 yield_obstacles);
    lane_change_info = state_machine->decide_lc_valid_info(lc_request);
    LOG_DEBUG("[CruiseState::Wait] gap_available: %d, aggressive_change: %d, ",
              gap_available, aggressive_change);
    // if (gap_available || aggressive_change) {
    // //TODO(Rui):后面把安全检查坐在gap_available里，统一通过gap_available判断
    if (curr_time > lc_tstart + delay_time && lane_change_info.gap_insertable) {
      prepare_for_change_state(lc_lane_manager, lc_req_manager,
                               candidate_states, lc_lane_managers);
      if (candidate_states.size() > 0 &&
          (candidate_states[0] == ROAD_LC_LCHANGE ||
           candidate_states[0] == ROAD_LC_RCHANGE) &&
          hdmap_valid) {
        prepare_for_wait_state(lc_lane_manager, lc_req_manager,
                               candidate_states, lc_lane_managers);
      }
    } else {
      prepare_for_wait_state(lc_lane_manager, lc_req_manager, candidate_states,
                             lc_lane_managers);
    }
  } else {
    prepare_for_none_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
  }

  state_machine->generate_state_machine_output(lane_change_info);

  for (size_t i = 0; i < candidate_states.size(); ++i) {
    auto o_lane_virtual_id = lc_lane_managers[i]->olane_virtual_id();
    auto fix_lane_virtual_id = lc_lane_managers[i]->flane_virtual_id();
    auto lc_request_source = lc_req_manager->request_source();
    bool bind_end_state = (candidate_states[i] == ROAD_LC_LCHANGE ||
                           candidate_states[i] == ROAD_LC_RCHANGE)
                              ? gap_available
                              : true;
    transition_contexts.emplace_back(StateTransitionContext{
        static_cast<ScenarioStateEnum>(context.state), candidate_states[i],
        lc_request_source, o_lane_virtual_id, fix_lane_virtual_id,
        lc_lane_managers[i], bind_end_state});
  }
  for (auto &transition_context : transition_contexts) {
    if (transition_context.target_state == ROAD_LC_LWAIT ||
        transition_context.target_state == ROAD_LC_RWAIT ||
        transition_context.target_state == ROAD_LC_LBACK ||
        transition_context.target_state == ROAD_LC_RBACK) {
      transition_context.overtake_obstacles = overtake_obstacles;
      transition_context.yield_obstacles = yield_obstacles;
    }
  }
}

void RoadState::LC::process_change(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  std::shared_ptr<ScenarioStateMachine> state_machine = context.state_machine;
  std::shared_ptr<LaneChangeRequestManager> lc_req_manager =
      state_machine->get_lane_change_request_manager();

  std::vector<ScenarioStateEnum> candidate_states;
  std::vector<std::shared_ptr<LaneChangeLaneManager>> lc_lane_managers;

  auto lc_lane_manager = std::make_shared<LaneChangeLaneManager>(
      state_machine->get_lane_change_lane_manager());

  std::shared_ptr<EnvironmentalModel> environmental_model =
      context.environmental_model;

  RequestType lc_request = lc_req_manager->request();
  bool gap_available{true};
  bool hdmap_valid = environmental_model->get_hdmap_valid();
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lc_back_info;
  if (state_machine->check_lc_change_finish(context.direction)) {
    LOG_DEBUG("[RoadState::Change] Lane Change Finished");
    prepare_for_none_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
    lc_req_manager->FinishRequest();
  } else if ((lc_request != NO_CHANGE && lc_request == context.direction) ||
             (lc_request == NO_CHANGE &&
              lc_lane_manager->is_ego_on(lc_lane_manager->tlane()))) {
    int target_lane_virtual_id = lc_req_manager->target_lane_virtual_id();
    if (!lc_lane_manager->has_target_lane() ||
        target_lane_virtual_id != lc_lane_manager->tlane_virtual_id()) {
      lc_lane_manager->assign_lc_lanes(target_lane_virtual_id);
    }

    gap_available = state_machine->gap_available(lc_request, overtake_obstacles,
                                                 yield_obstacles);

    if (lc_lane_manager->has_origin_lane() &&
        lc_lane_manager->olane_virtual_id() != lc_lane_manager->tlane_virtual_id()) {
      lc_back_info = state_machine->decide_lc_back_info(lc_request);
      if (lc_back_info.lc_should_back) {
        prepare_for_back_state(lc_lane_manager, lc_req_manager, candidate_states,
                               lc_lane_managers);
        if (candidate_states.size() > 0 &&
            (candidate_states[0] == ROAD_LC_LBACK ||
            candidate_states[0] == ROAD_LC_RBACK) &&
            lc_lane_manager->has_target_lane() &&
            hdmap_valid) {
          prepare_for_change_state(lc_lane_manager, lc_req_manager, candidate_states,
                                lc_lane_managers);
        }
      } else {
        prepare_for_change_state(lc_lane_manager, lc_req_manager, candidate_states,
                                 lc_lane_managers);
        if (candidate_states.size() > 0 &&
            (candidate_states[0] == ROAD_LC_LCHANGE ||
            candidate_states[0] == ROAD_LC_RCHANGE) &&
            !lc_lane_manager->is_ego_on(lc_lane_manager->tlane()) &&
            hdmap_valid) {
          prepare_for_back_state(lc_lane_manager, lc_req_manager, candidate_states,
                                lc_lane_managers);
        }
      }
    } else if (lc_lane_manager->has_target_lane()) {
      prepare_for_change_state(lc_lane_manager, lc_req_manager, candidate_states,
                               lc_lane_managers);
    } else {
      prepare_for_none_state(lc_lane_manager, lc_req_manager, candidate_states,
                             lc_lane_managers);
    }
  } else {
    LOG_DEBUG("[RoadState::Change] Change to Back");
    prepare_for_back_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
  }

  state_machine->generate_state_machine_output(lc_back_info);

  for (size_t i = 0; i < candidate_states.size(); ++i) {
    auto o_lane_virtual_id = lc_lane_managers[i]->olane_virtual_id();
    auto fix_lane_virtual_id = lc_lane_managers[i]->flane_virtual_id();
    auto lc_request_source = lc_req_manager->request_source();
    bool bind_end_state = (candidate_states[i] == ROAD_LC_LCHANGE ||
                           candidate_states[i] == ROAD_LC_RCHANGE)
                              ? gap_available
                              : true;
    transition_contexts.emplace_back(StateTransitionContext{
        static_cast<ScenarioStateEnum>(context.state), candidate_states[i],
        lc_request_source, o_lane_virtual_id, fix_lane_virtual_id,
        lc_lane_managers[i], bind_end_state});
  }
  for (auto &transition_context : transition_contexts) {
    if (transition_context.target_state == ROAD_LC_LWAIT ||
        transition_context.target_state == ROAD_LC_RWAIT ||
        transition_context.target_state == ROAD_LC_LBACK ||
        transition_context.target_state == ROAD_LC_RBACK) {
      transition_context.overtake_obstacles = overtake_obstacles;
      transition_context.yield_obstacles = yield_obstacles;
    }
  }
}

void RoadState::LC::process_back(FsmContext &context,
                                 StateTransitionContexts &transition_contexts) {
  std::shared_ptr<ScenarioStateMachine> state_machine = context.state_machine;
  std::shared_ptr<LaneChangeRequestManager> lc_req_manager =
      state_machine->get_lane_change_request_manager();

  std::vector<ScenarioStateEnum> candidate_states;
  std::vector<std::shared_ptr<LaneChangeLaneManager>> lc_lane_managers;

  auto lc_lane_manager = std::make_shared<LaneChangeLaneManager>(
      state_machine->get_lane_change_lane_manager());

  RequestType lc_request = lc_req_manager->request();
  bool aggressive_change{lc_req_manager->AggressiveChange()};
  bool gap_available{true};
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lane_change_info;
  if (state_machine->check_lc_back_finish(context.direction)) {
    // prepare for WAIT state
    LOG_DEBUG("[RoadeState::Back] Lane Back Finished");
    prepare_for_wait_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
  } else if (lc_request != NO_CHANGE && lc_request == context.direction) {
    if (!lc_lane_manager->has_target_lane()) {
      int target_lane_virtual_id = lc_req_manager->target_lane_virtual_id();
      lc_lane_manager->assign_lc_lanes(target_lane_virtual_id);
    }
    gap_available = state_machine->gap_available(lc_request, overtake_obstacles,
                                                 yield_obstacles);
    lane_change_info = state_machine->decide_lc_valid_info(lc_request);
    // if (gap_available || aggressive_change) { //TODO(Rui):后面把安全检查坐在gap_available里，统一通过gap_available判断
    if (lane_change_info.gap_insertable) {
      prepare_for_change_state(lc_lane_manager, lc_req_manager,
                               candidate_states, lc_lane_managers);
    } else {
      LOG_DEBUG("[CruiseState haowen] Wait to Back");
      prepare_for_back_state(lc_lane_manager, lc_req_manager, candidate_states,
                             lc_lane_managers);
    }
  } else {
    prepare_for_none_state(lc_lane_manager, lc_req_manager, candidate_states,
                           lc_lane_managers);
  }

  state_machine->generate_state_machine_output(lane_change_info);

  for (size_t i = 0; i < candidate_states.size(); ++i) {
    auto o_lane_virtual_id = lc_lane_managers[i]->olane_virtual_id();
    auto fix_lane_virtual_id = lc_lane_managers[i]->flane_virtual_id();
    auto lc_request_source = lc_req_manager->request_source();
    bool bind_end_state = (candidate_states[i] == ROAD_LC_LCHANGE ||
                           candidate_states[i] == ROAD_LC_RCHANGE)
                              ? gap_available
                              : true;
    transition_contexts.emplace_back(StateTransitionContext{
        static_cast<ScenarioStateEnum>(context.state), candidate_states[i],
        lc_request_source, o_lane_virtual_id, fix_lane_virtual_id,
        lc_lane_managers[i], bind_end_state});
  }
  for (auto &transition_context : transition_contexts) {
    if (transition_context.target_state == ROAD_LC_LWAIT ||
        transition_context.target_state == ROAD_LC_RWAIT ||
        transition_context.target_state == ROAD_LC_LBACK ||
        transition_context.target_state == ROAD_LC_RBACK) {
      transition_context.overtake_obstacles = overtake_obstacles;
      transition_context.yield_obstacles = yield_obstacles;
    }
  }
}

void RoadState::LC::LWait::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  // process_wait(context, transition_contexts);
}

void RoadState::LC::RWait::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  // process_wait(context, transition_contexts);
}

void RoadState::LC::LChange::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  // process_change(context, transition_contexts);
}

void RoadState::LC::RChange::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  // process_change(context, transition_contexts);
}

void RoadState::LC::LBack::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  // process_back(context, transition_contexts);
}

void RoadState::LC::RBack::get_state_transition_candidates(
    FsmContext &context, StateTransitionContexts &transition_contexts) {
  // process_back(context, transition_contexts);
}

// void RoadState::UTurn::get_state_transition_candidates(
//     FsmContext &context, StateTransitionContexts &transition_contexts) {}

}  // namespace planning
