#pragma once

#include "framework/session.h"
#include "modules/context/ego_planning_config.h"
#include "modules/context/reference_path.h"
#include "modules/context/environmental_model.h"
#include "modules/scenario/normal_road_state.h"
#include "modules/scenario/scenario_state.h"

namespace planning {

using ScenarioFsm = M::PeerRoot<
    M::Composite<
        RoadState, RoadState::None,
        M::Composite<RoadState::LC, RoadState::LC::LWait, RoadState::LC::RWait,
                     RoadState::LC::LChange, RoadState::LC::RChange,
                     RoadState::LC::LBack, RoadState::LC::RBack>,

        M::Composite<RoadState::LB, RoadState::LB::LBorrow,
                     RoadState::LB::RBorrow, RoadState::LB::LBack,
                     RoadState::LB::RBack, RoadState::LB::LReturn,
                     RoadState::LB::RReturn, RoadState::LB::LSuspend,
                     RoadState::LB::RSuspend>>>;

class ScenarioStateMachine
    : public std::enable_shared_from_this<ScenarioStateMachine> {
 public:
  ScenarioStateMachine(const EgoPlanningConfigBuilder* config_builder,
                       planning::framework::Session* session);

  void init();

  virtual ~ScenarioStateMachine();

  bool update(planning::framework::Frame* frame);
  std::shared_ptr<LaneChangeRequestManager> get_lane_change_request_manager() {
    return lc_req_mgr_;
  }
  std::shared_ptr<LaneChangeLaneManager> get_lane_change_lane_manager() {
    return lc_lane_mgr_;
  }

  bool gap_available(RequestType direction, std::vector<int>& overlap_obstacles,
                     std::vector<int>& yield_obstacles);
  bool check_lc_change_finish(RequestType direction);
  bool check_lc_back_finish(RequestType direction);
  void set_entry_time(double t) { state_entry_time_ = t; }
  void post_process();
  void reset_state_machine();
  double turn_signal_on_time() const { return turn_signal_on_time_; }
  RequestType turn_signal() const { return turn_signal_; }
  RequestType merge_split_turn_signal() const {
    return merge_split_turn_signal_;
  }

 private:
  void update_scenario();
  void update_state_machine();

  template <typename T>
  void change_state_external() {
    if (fsm_context_.state == type2int<T>::value) {
      return;
    }

    LOG_DEBUG("change_state_external from [%s] to [%s]", fsm_context_.name.c_str(),
          type2name<T>::name);

    fsm_context_.external = true;
    scenario_fsm_.changeTo<T>();
    scenario_fsm_.update();

    fsm_context_.external = false;
    fsm_context_.state = type2int<T>::value;
    fsm_context_.name = type2name<T>::name;
  }

  void gen_map_turn_signal();
  void gen_merge_split_turn_signal();

  FsmContext fsm_context_;
  ScenarioFsm scenario_fsm_;
  planning::framework::Session* session_;
  // std::shared_ptr<EnvironmentalModel> environmental_model_;  session已包含
  std::shared_ptr<LaneChangeRequestManager> lc_req_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lc_lane_mgr_;
  int scenario_ = SCENARIO_CRUISE;
  double state_entry_time_ = 0.0;
  double turn_signal_on_time_ = 0.0;
  RequestType map_turn_signal_ = NO_CHANGE;
  RequestType merge_split_turn_signal_ = NO_CHANGE;
  RequestType turn_signal_ = NO_CHANGE;
};

struct GapInfo {
  int id_front;
  double s_front;
  double v_front;
  int id_rear;
  double s_rear;
  double v_rear;
};

}  // namespace planning
