#pragma once

#include <array>
#include <iostream>

#include "common/utils/lateral_utils.h"
#include "context/virtual_lane_manager.h"
#include "frame.h"
#include "ifly_time.h"
#include "log.h"
#include "machine_single.hpp"
#include "scenario/lane_change_request_manager.h"
#include "scenario/lane_change_requests/lane_change_lane_manager.h"
#include "session.h"
#include "tasks/task_pipeline.h"

namespace planning {

class ScenarioStateMachine;
struct FsmContext {
  int state = 0;
  RequestType direction = NO_CHANGE;
  bool external = false;
  bool initialized = false;
  std::shared_ptr<EnvironmentalModel> environmental_model;
  planning::framework::Session *session;
  planning::framework::Frame *frame;
  std::shared_ptr<ScenarioStateMachine> state_machine;
  std::string name;
  double entry_time;
};

using M = hfsm::Machine<FsmContext>;

template <typename T>
struct type2int {};

template <typename T>
struct type2name {};

struct StateTransitionContext {
  ScenarioStateEnum source_state;
  ScenarioStateEnum target_state;
  RequestSource lane_change_request_source;
  int source_lane_id;
  int target_lane_id;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_manager;
  bool bind_end_state{true};
  // overtake_obstacles and yield_obstacles are used only under wait state
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
};
using StateTransitionContexts = std::vector<StateTransitionContext>;

class Evaluator;
class EgoPlanningCandidate;
struct StateBase : M::Base {
  void transition(Control &control, FsmContext &context) {
    if (context.external == true) {
      return;
    }

    process(control, context);
  }

  void process(Control &control, FsmContext &context);

  template <typename T>
  static void change_state(Control &control, FsmContext &context) {
    if (context.state == type2int<T>::value) {
      return;
    }

    LOG_DEBUG("change_state from [%s] to [%s]", context.name.c_str(),
              type2name<T>::name);

    control.changeTo<T>();
    context.state = type2int<T>::value;
    context.name = type2name<T>::name;
    context.entry_time = IflyTime::Now_s();
  }

  void change_state(ScenarioStateEnum next_state, Control &control,
                    FsmContext &context);

  // wait for TaskPipeline
  virtual std::shared_ptr<TaskPipeline> get_ego_planning_task_pipeline(
      planning::framework::Frame *frame) {
    common::SceneType scene_type = frame->session()->get_scene_type();
    auto config_builder =
        frame->session()->environmental_model().config_builder(scene_type);
    // return TaskPipeline::Make(TaskPipelineType::VISION_ONLY, config_builder,
    //                           frame);
    return TaskPipeline::Make(TaskPipelineType::NORMAL, config_builder, frame);
  }

  virtual std::shared_ptr<Evaluator> get_evaluator(framework::Frame *frame);

  virtual bool is_leaf() { return false; }

  virtual void get_state_transition_candidates(
      FsmContext &context, StateTransitionContexts &transition_contexts) {}
};

}  // namespace planning
