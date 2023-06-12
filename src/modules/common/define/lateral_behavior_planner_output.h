#ifndef COMMON_LATERAL_BEHAVIOR_PLANNER_OUTPUT_
#define COMMON_LATERAL_BEHAVIOR_PLANNER_OUTPUT_

#include <array>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/basic_type.h"
#include "geometry.h"
#include "path_point.h"
#include "tracked_object.h"

namespace planning {

struct AvdMsg {
  int id;
  std::string property;
  bool ignore;
  std::string avd_direction;
  int avd_priority;
  float blocked_time_begin;
  float blocked_time_end;
};

// struct TrackInfo {
//   TrackInfo() {}

//   TrackInfo(int id, double drel, double vrel)
//       : track_id(id), d_rel(drel), v_rel(vrel) {}

//   TrackInfo(const TrackInfo &track_info) {
//     track_id = track_info.track_id;
//     d_rel = track_info.d_rel;
//     v_rel = track_info.v_rel;
//   }

//   TrackInfo &operator=(const TrackInfo &track_info) {
//     track_id = track_info.track_id;
//     d_rel = track_info.d_rel;
//     v_rel = track_info.v_rel;
//     return *this;
//   }

//   void set_value(int id, double drel, double vrel) {
//     track_id = id;
//     d_rel = drel;
//     v_rel = vrel;
//   }

//   void reset() {
//     track_id = -10000;
//     d_rel = 0.0;
//     v_rel = 0.0;
//   }

//   int track_id = -10000;
//   double d_rel = 0.0;
//   double v_rel = 0.0;
// };

struct CarCount {
  int pos;
  int neg;

  CarCount() {
    pos = 0;
    neg = 0;
  }

  CarCount(int p, int n) {
    pos = p;
    neg = n;
  }

  CarCount(const CarCount &car_cnt) {
    pos = car_cnt.pos;
    neg = car_cnt.neg;
  }

  CarCount &operator=(const CarCount &car_cnt) {
    pos = car_cnt.pos;
    neg = car_cnt.neg;
    return *this;
  }

  bool operator==(const CarCount &car_cnt) const { return (pos == car_cnt.pos && neg == car_cnt.neg); }
};

struct PathPlannerContext {
  bool sb_lane = false;
  bool sb_blane = false;
  bool premoving = false;
  bool large_lat = false;
  bool force_pause = false;
  double lane_width = 3.8;
  double lat_offset = 0;
  double curr_time = 0;
  int lb_suspend_cnt = 0;
  double suspend_lat_offset = 0;
  std::array<double, 4> c_poly;
  std::array<double, 4> d_poly;
  std::array<std::vector<double>, 2> avd_car_past;
};

struct IntRequestContext {
  int request = 0;
  int turn_light = 0;
  int counter_left = 0;
  int counter_right = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
};

struct MapRequestContext {
  int request = 0;
  int turn_light = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
  int order = 0;
};

struct ActRequestContext {
  int request = 0;
  int turn_light = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
  bool enforced_l = false;
  bool enforced_r = false;
  std::string act_request_source = "none";
};

struct LCRequestManagerContext {
  int request = 0;
  int request_source = 0;
  int turn_light = 0;
  IntRequestContext int_request;
  MapRequestContext map_request;
  ActRequestContext act_request;
};

struct LBRequestManagerContext {
  int request = 0;
  int turn_light = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
};

struct ObjectSelectorContext {
  double v_rel_l = 0.0;
  double v_rel_r = 0.0;
  double v_rel_f = 15.0;
  double d_stop_l = 0;
  double d_stop_r = 0;
  double d_lb_car_l = 0;
  double d_lb_car_r = 0;
  double t_surpass_l = -1000;
  double t_surpass_r = -1000;
  double premove_dist = 0.0;

  bool premovel = false;
  bool premover = false;
  bool left_is_faster = false;
  bool right_is_faster = false;
  int left_is_faster_cnt = 0;
  int right_is_faster_cnt = 0;
  int premoved_id = -1000;
  int neg_premoved_id = -1000;

  int l_accident_cnt = 0;
  int r_accident_cnt = 0;

  std::vector<int> left_lb_car;
  std::vector<int> left_alc_car;
  std::vector<int> right_lb_car;
  std::vector<int> right_alc_car;

  std::map<int, CarCount> left_lb_car_cnt;
  std::map<int, CarCount> left_alc_car_cnt;
  std::map<int, CarCount> right_lb_car_cnt;
  std::map<int, CarCount> right_alc_car_cnt;
};

struct RawRefLineContext {
  bool exist = false;
  int position = -100;
  std::vector<Point2D> waypoints;
  std::map<std::string, bool> point_ids;
};

struct VirtualLaneContext {
  bool exist = false;
  int type = 0;
  int status = 3;
  int source = 0;
  int position = -100;
  double width = 3.8;

  std::array<int, 2> rids;
  std::array<unsigned long, 2> ids;
  std::array<double, 2> intercepts;
  std::array<std::vector<double>, 2> polys;
  std::vector<double> c_poly;
  std::array<double, 2> heads;
  std::array<double, 2> tails;
  RawRefLineContext raw_refline;
  bool has_raw_refline = false;
  int master_position = -100;
};

struct FixRefLineContext {
  int position = -100;
  std::vector<PathPoint> path_points;
};

struct VirtualLaneManagerContext {
  bool flane_update = false;
  VirtualLaneContext flane;
  VirtualLaneContext olane;
  VirtualLaneContext tlane;
  FixRefLineContext f_refline;
};

struct LaneTracksManagerContext {};

struct LateralBehaviorPlannerContext {
  bool no_sp_car = true;

  double t_avd_sp_car = 3.0;
  double final_y_rel = 10;

  int ncar_change = 0;
  int lbcar_change = 0;
  int flag_avd = 0;
  int avd_back_cnt = 0;
  int avd_leadone = 0;
  int pre_leadone_id = 0;

  int intersection_cnt = 0;
  double dist_rblane = 0;

  int ignore_track_id = -10000;
  int stand_still_count = 0;
  double angle_steers_limit = 0;

  PathPlannerContext path_planner;

  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;

  std::vector<int> left_lb_car;
  std::vector<int> right_lb_car;

  std::vector<double> vel_sequence;
  std::set<int> ignore_change_false;
  std::set<int> ignore_change_true;
};
struct LateralBehaviorPlannerOutput {
  bool enable = false;
  int track_id = -10000;
  double v_limit = 40.0;
  double lat_offset = 0.0;
  bool lane_borrow = false;
  int lane_borrow_range = -1000;
  std::string which_lane = "current_line";
  std::string lb_request = "none";
  std::string lc_request = "none";
  std::string lc_status = "none";
  std::string lb_status = "none";
  std::vector<AvdMsg> avd_info;

  int scenario = 1;
  double flane_width = 3.8;
  double dist_rblane = 0.0;
  std::vector<PathPoint> path_points;
  bool borrow_bicycle_lane = false;
  bool enable_intersection_planner = false;
  bool tleft_lane = false;
  bool rightest_lane = false;
  bool isFasterStaticAvd = false;
  bool isOnHighway = false;
  bool isRedLightStop = false;
  double dist_intersect = 1000;
  double intersect_length = 1000;
  double lc_end_dis = 10000;
  double dis_to_ramp = 10000;

  bool sb_lane = false;
  bool sb_blane = false;
  bool force_pause = false;
  bool large_lat = false;
  bool premoving = false;
  bool accident_ahead = false;
  bool accident_back = false;
  int lc_pause_id = -1000;
  bool lc_pause = false;
  double tr_pause_l = 0.0;
  double tr_pause_s = -100.0;
  bool must_change_lane = false;
  bool left_faster = false;
  bool right_faster = false;
  bool close_to_accident = false;
  bool premove = false;
  bool behavior_suspension = false;  // lateral suspend
  std::vector<int> suspension_obs;   // lateral suspend
  double angle_steers_limit = 0.0;
  double premove_dist = 0.0;
  std::vector<double> c_poly;
  std::vector<double> d_poly;
  uint64_t planner_scene = 0;
  uint64_t planner_action = 0;
  uint64_t planner_status = 0;

  double lead_one_drel = 0.0;
  double lead_one_vrel = 0.0;

  std::string state_name = "none";
  std::string scenario_name = "none";
  std::string turn_light = "none";
  std::string lc_request_source = "none";
  std::string act_request_source = "none";
  std::string turn_light_source = "none";

  double lb_width = 0.0;
  std::vector<std::tuple<double, double>> s_v_limit;
  std::vector<std::tuple<double, double>> s_a_limit;
  std::vector<std::tuple<double, double>> s_r_offset;

  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;
  std::vector<double> vel_sequence;

  std::set<int> ignore_change_false;
  std::set<int> ignore_change_true;

  bool cross_lsolid_line = false;
  bool cross_rsolid_line = false;

  std::array<double, 4> l_poly;
  std::array<double, 4> r_poly;

  bool disable_l = false;
  bool disable_r = false;
  bool enable_l = false;
  bool enable_r = false;
  int enable_id = -1;

  bool lc_warning = false;
  bool avd_in_lane = false;
  bool lc_exit_ramp_start = false;

  LateralBehaviorPlannerContext planner_context;
};

}  // namespace planning

#endif
