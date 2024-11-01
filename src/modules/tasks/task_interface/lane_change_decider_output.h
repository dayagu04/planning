#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "../adas_function/display_state_types.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "virtual_lane.h"
namespace planning {

struct LaneChangeStateMachineInfo {
  TrackInfo lc_invalid_track;
  TrackInfo lc_back_track;
  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackInfo> near_cars_origin;
  int lc_back_cnt = 0;
  int lc_valid_cnt = 0;

  bool initialized = false;
  bool behavior_suspend = false;
  std::vector<int> suspend_obs;
  double start_move_dist_lane = 0.;
  bool not_accident = false;
};

struct LaneChangeDeciderOutput {
  int scenario;
  StateMachineLaneChangeStatus curr_state;
  int fix_lane_virtual_id;
  int origin_lane_virtual_id;
  int target_lane_virtual_id;
  bool has_target_lane = false;
  std::string state_name;
  std::string lc_back_reason = "none";
  std::string lc_invalid_reason = "none";

  int turn_light;
  int map_turn_light;
  bool accident_back;
  bool accident_ahead;
  bool close_to_accident;
  bool should_premove;
  bool should_suspend;
  bool must_change_lane;
  bool behavior_suspend;         // lateral suspend
  std::vector<int> suspend_obs;  // lateral suspend

  bool lc_pause;
  int lc_pause_id;
  double lc_timer;
  double tr_pause_l;
  double tr_pause_s;

  bool disable_l;
  bool disable_r;
  bool enable_l;
  bool enable_r;

  bool need_clear_lb_car;
  std::unordered_map<int, int> front_tracks_slow_cnt;
  std::unordered_map<int, int> avd_track_cnt;
  bool enable_lb;
  int enable_id;

  int lc_request;
  int lc_request_source;
  int lc_turn_light;
  std::string act_request_source;
  int lb_request;
  int lb_turn_light;

  bool premovel;
  bool premover;
  double premove_dist;

  bool left_is_faster;
  bool right_is_faster;

  bool neg_left_lb_car;
  bool neg_right_lb_car;
  bool neg_left_alc_car;
  bool neg_right_alc_car;

  std::vector<int> left_lb_car;
  std::vector<int> left_alc_car;
  std::vector<int> right_lb_car;
  std::vector<int> right_alc_car;

  bool enable_alc_car_protection{false};
  std::vector<int> alc_cars_;
  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackInfo> near_cars_origin;
  TrackInfo lc_invalid_track;
  TrackInfo lc_back_track;

  bool is_lc_valid;
  int lc_valid_cnt;
  int lc_back_cnt;
  std::string lc_back_invalid_reason;

  bool turn_signal =
      false;  // session_->mutable_planning_context()->mutable_planning_result().turn_signal
  double start_move_dist_lane;

  bool s_search_status = false;
  std::vector<double> st_search_vec;

  CoarsePlanningInfo coarse_planning_info;
  bool is_merge_region = false;
  MergeDirection merge_direction = NONE_LANE_MERGE;
  int merge_lane_virtual_id;
  RampDirection dir_turn_signal_road_to_ramp = RAMP_NONE;
  IntCancelReasonType int_request_cancel_reason = NO_CANCEL;
  RequestType ilc_virtual_req = NO_CHANGE;
  Point2D merge_point;
  Point2D boundary_merge_point;
  bool cur_lane_is_continue;
  bool boundary_merge_point_valid = false;
};

}  // namespace planning