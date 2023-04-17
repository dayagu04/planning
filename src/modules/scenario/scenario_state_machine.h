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

typedef struct {
  std::pair<int, int> gap;
  bool gap_valid{false};
  bool gap_approached{false};
  bool gap_insertable{false};
  bool side_approach{false};
  bool should_suspend{false};

  // lc valid related
  bool lc_pause{false};
  int lc_pause_id{-1000};
  bool should_premove{false};
  std::string lc_invalid_reason{"none"};

  // lc back related
  double tr_pause_dv{0.0};
  double tr_pause_l{0.0};
  double tr_pause_s{-100.0};
  bool accident_back{false};

  // clear lc related
  bool need_clear_lb_car{false};

  // not related but needed
  bool accident_ahead{false};
  bool close_to_accident{false};

  // back to state machine only for debug
  bool lc_should_back{false};
  bool lc_valid{false};
  std::string lc_back_reason{"none"};

} LaneChangeStageInfo;

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
  LaneChangeStageInfo compute_lc_valid_info(RequestType direction);
  LaneChangeStageInfo decide_lc_valid_info(RequestType direction);
  LaneChangeStageInfo compute_lc_back_info(RequestType direction);
  LaneChangeStageInfo decide_lc_back_info(RequestType direction);
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
  void generate_state_machine_output() {}

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
  int lc_back_cnt_ = 0;
  int lb_back_cnt_ = 0;
  int lc_valid_cnt_ = 0;

  bool lc_valid_ = true;
  bool lc_valid_back_ = true;
  bool lc_should_back_ = false;
  bool must_change_lane_ = false;
  bool behavior_suspend_ = false;        // lateral suspend
  std::vector<int> suspend_obs_; // lateral suspend obstacles


  bool lc_pause_ = false;
  int lc_pause_id_ = -1000;
  double tr_pause_l_ = 0.0;
  double tr_pause_s_ = -100.0;
  double tr_pause_dv_ = 0.0;

  std::string lc_back_reason_ = "none";
  std::string lc_invalid_reason_ = "none";
  std::string invalid_back_reason_ = "none";

  std::vector<TrackInfo> near_cars_target_;
  std::vector<TrackInfo> near_cars_origin_;

  TrackInfo lc_invalid_track_;
  TrackInfo lc_back_track_;

  bool side_approaching_ = false;
  bool get_dist_lane_ = false;

  bool should_premove_ = false;
  bool should_suspend_ = false;

  bool accident_ahead_ = false;
  bool close_to_accident_ = false;
  bool accident_back_ = false;
  bool not_accident_ = true;

  double start_move_dist_lane_ = 0.;

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
