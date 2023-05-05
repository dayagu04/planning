#include "scenario/scenario_state.h"

#include "ifly_time.h"
#include "context/ego_planning_config.h"
#include "context/reference_path_manager.h"
#include "scenario/ego_planning_candidate.h"
#include "scenario/evaluator.h"
#include "scenario/scenario_state_machine.h"

namespace planning {

std::shared_ptr<Evaluator> StateBase::get_evaluator(framework::Frame *frame) {
  common::SceneType scene_type = frame->session()->get_scene_type();
  auto config_builder =
      frame->session()->environmental_model().config_builder(scene_type);
  return std::make_shared<Evaluator>(config_builder, frame);
}

void StateBase::process(Control &control, FsmContext &context) {
  LOG_DEBUG("StateBase::process(); ");

  if (!is_leaf()) {
    return;
  }

  LOG_DEBUG("[StateBase] entering process");
  auto start_time = IflyTime::Now_ms();

  // Step 1) get state transition candidates
  auto state_machine = context.state_machine;
  state_machine->set_entry_time(context.entry_time);
  StateTransitionContexts transition_contexts;
  get_state_transition_candidates(context, transition_contexts);
  assert(transition_contexts.size() > 0);
  auto candidates_time = IflyTime::Now_ms();
  LOG_DEBUG("[StateBase] get candidates time: %f",
            candidates_time - start_time);

  // Step 2) refine candidates
  EgoPlanningCandidates candidates;
  for (size_t i = 0; i < transition_contexts.size(); ++i) {
    auto &transition_context = transition_contexts[i];

    EgoPlanningCandidate candidate(context.frame);
    candidate.set_coarse_planning_info(transition_context);

    auto last_planning_result = context.frame->mutable_session()
                                    ->mutable_planning_context()
                                    ->last_planning_result();
    auto state_machine_output =
        context.frame->mutable_session()
            ->mutable_planning_context()
            ->mutable_lat_behavior_state_machine_output();
    state_machine_output.curr_state =
        candidate.coarse_planning_info().target_state;
    state_machine_output.fix_lane_virtual_id =
        candidate.coarse_planning_info().target_lane_id;
    state_machine_output.origin_lane_virtual_id =
        candidate.coarse_planning_info().source_lane_id;
    state_machine_output.target_lane_virtual_id =
        transition_context.lane_change_lane_manager->tlane_virtual_id();
    // state_machine_output.state_name = type2name<RoadState::None>::name;
    // //TODO(Rui):add name transfer
    if (last_planning_result != nullptr and
        last_planning_result->target_lane_id ==
            candidate.coarse_planning_info().target_lane_id and
        last_planning_result->traj_points.size() >= 10) {
      candidate.set_last_planning_result(last_planning_result);
    }

    if (candidate.pre_check()) {
      // wait for task_pipeline
      std::shared_ptr<TaskPipeline> task_pipeline =
          get_ego_planning_task_pipeline(context.frame);
      candidate.refine(task_pipeline);
    } else {
      LOG_ERROR("pre_check failed");
    }

    candidates.emplace_back(std::move(candidate));
  }
  LOG_DEBUG("candidates: size: %d", candidates.size());
  auto refined_time = IflyTime::Now_ms();
  LOG_DEBUG("[StateBase] refine time: %f", refined_time - candidates_time);

  // Step 3) evaluate
  auto evaluator = get_evaluator(context.frame);
  int best_solution_id(0);
  // wait evaluator
  bool success = true;
  // bool success = evaluator->evaluate(candidates, best_solution_id);
  auto evaluated_time = IflyTime::Now_ms();
  LOG_DEBUG("[StateBase] evaluate time: %f", evaluated_time - refined_time);

  if (not success) {
    // run pipeline failure
    context.frame->mutable_session()
        ->mutable_planning_context()
        ->mutable_planning_success() = false;
    // failure error log
    LOG_DEBUG("[StateBase] evaluate failed");
    return;
  }
  context.frame->mutable_session()
      ->mutable_planning_context()
      ->mutable_planning_success() = true;

  // Step 4) copy planning context
  candidates[best_solution_id].copy_to_planning_context();

  // Step 5) copy lange change state
  auto &best_transition_context = transition_contexts[best_solution_id];
  context.direction =
      state_machine->get_lane_change_request_manager()->request();
  auto lane_change_lane_manager = state_machine->get_lane_change_lane_manager();
  lane_change_lane_manager->copy_lane_change_lanes(
      *best_transition_context.lane_change_lane_manager);
  lane_change_lane_manager->upload_fix_lane_virtual_id();

  // Step 7) change state
  auto next_state = best_transition_context.target_state;
  if (best_transition_context.target_state != context.state) {
    if (next_state == ROAD_NONE) {
      state_machine->clear_lc_variables();
    } else if ((next_state == ROAD_LC_LCHANGE &&
                (context.state == ROAD_LC_LWAIT ||
                 context.state == ROAD_LC_LBACK)) ||
               (next_state == ROAD_LC_RCHANGE &&
                (context.state == ROAD_LC_RWAIT ||
                 context.state == ROAD_LC_RBACK))) {
      state_machine->update_start_move_dist_lane();
    }
    change_state(next_state, control, context);
    context.state = next_state;
    // TODO(Rui):fix me
    auto lat_behavior_state_machine_output =
        context.frame->mutable_session()
            ->mutable_planning_context()
            ->mutable_lat_behavior_state_machine_output();
    lat_behavior_state_machine_output.fix_lane_virtual_id =
        lane_change_lane_manager->flane_virtual_id();
    lat_behavior_state_machine_output.origin_lane_virtual_id =
        lane_change_lane_manager->olane_virtual_id();
    lat_behavior_state_machine_output.target_lane_virtual_id =
        lane_change_lane_manager->tlane_virtual_id();
  }
  LOG_DEBUG("[StateBase] evaluate success");
}

void StateBase::change_state(ScenarioStateEnum next_state, Control &control,
                             FsmContext &context) {
  switch (next_state) {
    case ROAD_NONE:
      change_state<RoadState::None>(control, context);
      break;
    case ROAD_LC_LWAIT:
      change_state<RoadState::LC::LWait>(control, context);
      break;
    case ROAD_LC_RWAIT:
      change_state<RoadState::LC::RWait>(control, context);
      break;
    case ROAD_LC_LCHANGE:
      change_state<RoadState::LC::LChange>(control, context);
      break;
    case ROAD_LC_RCHANGE:
      change_state<RoadState::LC::RChange>(control, context);
      break;
    case ROAD_LC_LBACK:
      change_state<RoadState::LC::LBack>(control, context);
      break;
    case ROAD_LC_RBACK:
      change_state<RoadState::LC::RBack>(control, context);
      break;
    default:
      change_state<RoadState::None>(control, context);
  }
}

}  // namespace planning
