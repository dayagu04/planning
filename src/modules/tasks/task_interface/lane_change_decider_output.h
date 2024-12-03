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
  bool should_premove;
  bool must_change_lane;

  int lc_request;
  int lc_request_source;
  int lc_turn_light;
  std::string act_request_source;
  double lat_premove_dist;

  TrackInfo lc_invalid_track;
  TrackInfo lc_back_track;

  bool is_lc_valid;
  int lc_valid_cnt;
  int lc_back_cnt;
  std::string lc_back_invalid_reason;

  bool turn_signal = false;  // session_->mutable_planning_context()->mutable_planning_result().turn_signal

  bool s_search_status = false;
  std::vector<double> st_search_vec;

  CoarsePlanningInfo coarse_planning_info;
  RampDirection dir_turn_signal_road_to_ramp = RAMP_NONE;
  IntCancelReasonType int_request_cancel_reason = NO_CANCEL;
  RequestType ilc_virtual_req = NO_CHANGE;
  double lateral_close_boundary_offset = 0.0;
};

}  // namespace planning