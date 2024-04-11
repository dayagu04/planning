#pragma once

#include "ego_planning_config.h"
#include "environmental_model.h"
#include "lane_change_request_manager.h"
#include "lateral_behavior_object_selector.h"
#include "reference_path.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

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

struct StateTransitionContext {
  ScenarioStateEnum source_state = ROAD_NONE;
  ScenarioStateEnum target_state = ROAD_NONE;
  RequestType direction = NO_CHANGE;
  bool bind_end_state = true;
  // overtake_obstacles and yield_obstacles are used only under wait state
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  void Reset() {
    source_state = ROAD_NONE;
    target_state = ROAD_NONE;
    direction = NO_CHANGE;
    bind_end_state = true;
    overtake_obstacles.clear();
    yield_obstacles.clear();
  }
};

class LaneChangeDecider : public Task {
 public:
  LaneChangeDecider(const EgoPlanningConfigBuilder* config_builder,
                    framework::Session* session);

  void Init();

  virtual ~LaneChangeDecider() = default;

  bool Execute() override;

  std::shared_ptr<LaneChangeRequestManager> get_lane_change_request_manager() {
    return lc_req_mgr_;
  }
  std::shared_ptr<LaneChangeLaneManager> get_lane_change_lane_manager() {
    return lc_lane_mgr_;
  }
  std::shared_ptr<ObjectSelector> get_object_selector() {
    return object_selector_;
  }

  /**
   * @brief 判断和构造目标车道中的gap
    -|                |-▅-|     |
    ▅...........................
    ----▅-|   |-▅-|    |-▅-|  |
   */
  bool GapAvailable(RequestType direction, std::vector<int>& overlap_obstacles,
                    std::vector<int>& yield_obstacles);

  void compute_lc_valid_info(RequestType direction);
  LaneChangeStageInfo decide_lc_valid_info(const bool localation_valid,
                                           RequestType direction);
  void compute_lc_back_info(RequestType direction);
  LaneChangeStageInfo decide_lc_back_info(const bool localation_valid,
                                          RequestType direction);
  bool check_lc_change_finish(RequestType direction);
  bool check_lc_back_finish(RequestType direction);
  void set_entry_time(double t) { state_entry_time_ = t; }
  void post_process();
  void reset_state_machine();
  void clear_lc_variables();
  void clear_lc_stage_info();
  void clear_lc_valid_cnt() { lc_valid_cnt_ = 0; }
  void clear_lc_back_cnt() { lc_back_cnt_ = 0; }
  void set_get_dist_lane(bool value) { get_dist_lane_ = value; }
  void update_start_move_dist_lane();
  double get_start_move_dist_lane() const { return start_move_dist_lane_; }
  double turn_signal_on_time() const { return turn_signal_on_time_; }
  RequestType turn_signal() const { return turn_signal_; }
  RequestType merge_split_turn_signal() const {
    return merge_split_turn_signal_;
  }
  void generate_state_machine_output(const LaneChangeStageInfo& lc_info);

 private:
  void update_scenario();
  void update_state_machine();

  // Determine whether the gap distance between vehicles is sufficient
  bool IsFollowBufferEnough(const double front_s, const double behind_s,
                            const double front_v, const double behind_v);

  void gen_map_turn_signal();
  void gen_merge_split_turn_signal();

 private:
  bool CheckEgoPosition() const;

  void UpdateCoarsePlanningInfo();

  void UpdateStateMachineDebugInfo();

  void UpdateAdInfo();

  bool UpdateObjectSelector(bool active);

  void UpdateFixLaneVirtualId();

  void UpdateTransitionContext(std::vector<int>&& overtake_obstacles,
                               std::vector<int>&& yield_obstacles,
                               const bool gap_available);

  void PrepareForNoneState();
  void PrepareForWaitState();
  void PrepareForChangeState();
  void PrepareForBackState();

  void ProcessNoneState();
  void ProcessWaitState();
  void ProcessChangeState();
  void ProcessBackState();

  void UpdateLaneChangeState();

 private:
  ScenarioStateMachineConfig config_;
  std::shared_ptr<LaneChangeRequestManager> lc_req_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lc_lane_mgr_;
  std::shared_ptr<ObjectSelector> object_selector_;
  int scenario_ = SCENARIO_CRUISE;
  double state_entry_time_ = 0.0;
  double turn_signal_on_time_ = 0.0;
  RequestType map_turn_signal_ = NO_CHANGE;
  RequestType merge_split_turn_signal_ = NO_CHANGE;
  RequestType turn_signal_ = NO_CHANGE;
  int lc_back_cnt_ = 0;
  int lb_back_cnt_ = 0;
  int lc_valid_cnt_ = 0;

  bool must_change_lane_ = false;
  bool behavior_suspend_ = false;  // lateral suspend
  std::vector<int> suspend_obs_;   // lateral suspend obstacles

  std::vector<TrackInfo> near_cars_target_;
  std::vector<TrackInfo> near_cars_origin_;

  TrackInfo lc_invalid_track_;
  TrackInfo lc_back_track_;

  bool get_dist_lane_ = false;
  bool not_accident_ = true;

  double start_move_dist_lane_ = 0.;
  LaneChangeStageInfo lane_change_stage_info_;
  bool last_should_premove_{false};

  StateTransitionContext transition_context_;
};

}  // namespace planning
