#pragma once

#include <cstddef>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "general_planning_context.h"
#include "log.h"
#include "nlohmann_json.hpp"
#include "task_basic_types.h"

namespace planning {

using Json = nlohmann::json;

struct Config;

class EgoPlanningConfigBuilder {
 public:
  explicit EgoPlanningConfigBuilder(const Json &json, const char *name)
      : json_(json), name_(name) {}

  template <typename T>
  T cast() const {
    static_assert(std::is_base_of<Config, T>::value,
                  "cast only works on derived Config");
    T target;
    target.init(json_);
    std::cout << "created" << typeid(T).name() << std::endl;
    // LOG_DEBUG("created %s with %s", typeid(T).name(), name());
    return target;
  }

 private:
  inline const char *name() const { return name_ ? name_ : ""; }
  const Json json_;
  const char *name_ = nullptr;
};

template <typename T>
T read_json_key(const Json &json, const char *key) {
  if (json.find(key) != json.end()) {
    return json[key];
  } else {
    return {};
  }
}

template <typename T>
T read_json_key(const Json &json, const char *key, T default_value) {
  if (json.find(key) != json.end()) {
    return json[key];
  } else {
    return default_value;
  }
}

template <typename T>
T read_json_keys(const Json &json, const std::vector<std::string> &keys) {
  if (keys.empty()) return {};
  Json json_new = json;
  for (int i = 0; i < (int)keys.size() - 1; i++) {
    if (json_new.find(keys[i]) != json_new.end()) {
      json_new = json_new[keys[i]];
    } else
      return {};
  }
  if (json_new.find(keys.back()) != json_new.end())
    return json_new[keys.back()];
  else
    return {};
}

template <typename T>
T read_json_keys(const Json &json, const std::vector<std::string> &keys,
                 T default_value) {
  if (keys.empty()) return default_value;
  Json json_new = json;
  for (int i = 0; i < (int)keys.size() - 1; i++) {
    if (json_new.find(keys[i]) != json_new.end()) {
      json_new = json_new[keys[i]];
    } else
      return default_value;
  }
  if (json_new.find(keys.back()) != json_new.end())
    return json_new[keys.back()];
  else
    return default_value;
}

template <typename T>
void read_json_vec(const Json &json, const std::string &key,
                   std::vector<T> &vec,
                   const std::vector<T> &default_vec = {}) {
  if (json.find(key) != json.end() && json[key].is_array()) {
    vec.clear();
    for (size_t i = 0; i < json[key].size(); i++) {
      vec.push_back(json[key][i]);
    }
  } else {
    vec = default_vec;
  }
}

template <typename T>
void read_json_vec(const Json &json, const std::vector<std::string> &keys,
                   std::vector<T> &vec,
                   const std::vector<T> &default_vec = {}) {
  Json json_new = json;
  for (int i = 0; i < (int)keys.size() - 1; i++) {
    if (json_new.find(keys[i]) != json_new.end()) {
      json_new = json_new[keys[i]];
    }
  }
  if (json_new.find(keys.back()) != json_new.end()) {
    vec.clear();
    for (size_t i = 0; i < json_new[keys.back()].size(); i++) {
      vec.emplace_back(json_new[keys.back()][i]);
    }
    return;
  }
  vec = default_vec;
}

struct Config {
  virtual void init(const Json &json) = 0;
};

/***************************************************************************************/
struct EgoPlanningConfig : public Config {
  void init(const Json &json) override {
    trajectory_time_length =
        read_json_key<double>(json, "trajectory_time_length");
    planning_dt = read_json_key<double>(json, "planning_dt");
    enable_raw_ego_prediction =
        read_json_key<bool>(json, "enable_raw_ego_prediction");
    enable_dagger = read_json_key<bool>(json, "enable_dagger");
    use_ego_prediction_model_in_planning = read_json_key<bool>(
        json, "use_ego_prediction_model_in_planning", false);
    planner_type = read_json_key<int>(json, "planner_type");
    active_lane_change_min_duration_threshold =
        read_json_key<int>(json, "active_lane_change_min_duration_threshold");
    use_lateral_distance_to_judge_cutout_in_active_lane_change =
        read_json_key<bool>(
            json, "use_lateral_distance_to_judge_cutout_in_active_lane_change");
    use_overtake_lane_change_request_instead_of_active_lane_change_request =
        read_json_key<bool>(json,
                            "use_overtake_lane_change_request_instead_of_"
                            "active_lane_change_request");
    minimum_distance_nearby_ramp_to_surpress_overtake_lane_change =
        read_json_key<double>(
            json,
            "minimum_distance_nearby_ramp_to_surpress_overtake_lane_change");
    minimum_ego_cruise_speed_for_active_lane_change = read_json_key<double>(
        json, "minimum_ego_cruise_speed_for_active_lane_change");
    enable_use_emergency_avoidence_lane_change_request = read_json_key<bool>(
        json, "enable_use_emergency_avoidence_lane_change_request");
    enable_use_cone_change_request =
        read_json_key<bool>(json, "enable_use_cone_change_request");
    enable_use_merge_change_request =
        read_json_key<bool>(json, "enable_use_merge_change_request");
  }
  double trajectory_time_length = 5.0;
  double planning_dt = 0.2;
  bool enable_raw_ego_prediction = false;
  bool enable_dagger = false;
  bool use_ego_prediction_model_in_planning = false;
  int planner_type = planning::context::PlannerType::REALTIME_PLANNER;
  int active_lane_change_min_duration_threshold = 150;
  bool use_lateral_distance_to_judge_cutout_in_active_lane_change = true;
  bool use_overtake_lane_change_request_instead_of_active_lane_change_request =
      true;
  double minimum_distance_nearby_ramp_to_surpress_overtake_lane_change = 1500;
  double minimum_ego_cruise_speed_for_active_lane_change = 16.67;
  bool enable_use_emergency_avoidence_lane_change_request = false;
  bool enable_use_cone_change_request = false;
  bool enable_use_merge_change_request = false;
};

struct GeneralPlanningConfig : public EgoPlanningConfig {
  double d_poly_min_lat_offset = 0.0;
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    lc_back_smooth_thr = read_json_key<double>(json, "lc_back_smooth_thr");
    lc_back_consider_smooth_dpoly_thr =
        read_json_key<double>(json, "lc_back_consider_smooth_dpoly_thr");
    d_poly_lat_offset_rate =
        read_json_key<double>(json, "d_poly_lat_offset_rate");
    d_poly_max_lat_offset =
        read_json_key<double>(json, "d_poly_max_lat_offset");
    d_poly_min_lat_offset =
        read_json_key<double>(json, "d_poly_min_lat_offset");
    enable_none_smooth = read_json_key<bool>(json, "enable_none_smooth");
    none_consider_slope_thr =
        read_json_key<double>(json, "none_consider_slope_thr");
    failure_counter_thrshld =
        read_json_key<double>(json, "failure_counter_thrshld");
  }
  double lc_back_smooth_thr = 0.3;
  double lc_back_consider_smooth_dpoly_thr = 0.6;
  bool enable_none_smooth = true;
  double none_consider_slope_thr = 0.003;
  double d_poly_lat_offset_rate = 0.2;
  double d_poly_max_lat_offset = 2.0;
  uint failure_counter_thrshld = 20;
};

struct EgoPlanningCandidateConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    // reference_point_velocity =
    //     read_json_key<double>(json, "reference_point_velocity");
  }

  double reference_point_velocity = 22.22;  // m/s
  double delta_t = 0.2;
  int num_point = 26;
};

struct LateralObstacleConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    obstacle_detect_distance_upper = read_json_key<double>(
        json, "obstacle_detect_distance_upper", obstacle_detect_distance_upper);
    obstacle_detect_distance_lower = read_json_key<double>(
        json, "obstacle_detect_distance_lower", obstacle_detect_distance_lower);
  }
  double obstacle_detect_distance_upper = 220;
  double obstacle_detect_distance_lower = -50;
};

struct HistoryObstacleConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    ego_near_bound_s0 =
        read_json_key<double>(json, "ego_near_bound_s0", ego_near_bound_s0);
    ego_near_bound_s1 =
        read_json_key<double>(json, "ego_near_bound_s1", ego_near_bound_s1);
    /* read config from json */
  }
  double ego_near_bound_s0 = -10.0;
  double ego_near_bound_s1 = 10.0;
  double ego_near_bound_l = 5.0;
};

struct VisionLateralBehaviorPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct ObstacleDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct ScenarioStateMachineConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    lc_t_actuator_delay = read_json_key<double>(json, "lc_t_actuator_delay");
    lc_back_available_thr =
        read_json_key<double>(json, "lc_back_available_thr");
    lc_finish_dist_thr = read_json_key<double>(json, "lc_finish_dist_thr");
    lc_finish_heading_deg_thr =
        read_json_key<double>(json, "lc_finish_heading_deg_thr");
    read_json_vec<double>(json, "lc_finished_dist_thr", lc_finished_dist_thr);
    min_ego_v_cruise = read_json_key<double>(json, "min_ego_v_cruise");
  }
  double lc_t_actuator_delay = 0.03;
  double lc_back_available_thr = 1.5;
  double delta_t = 0.2;
  int num_point = 26;
  double lc_finish_dist_thr = 0.5;
  double lc_finish_heading_deg_thr = 1.0;
  std::vector<double> lc_finished_dist_thr{0.1, 0.15, 0.2, 0.3};
  double min_ego_v_cruise = 2.0;
};

struct SpeedAdjustDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    min_dec_adjust_limit = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_dec_adjust_limit"});
    max_acc_adjust_ratio_lower = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_adjust", "max_acc_adjust_ratio_lower"});
    max_acc_adjust_ratio_upper = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_adjust", "max_acc_adjust_ratio_upper"});
    enable_speed_adjust = read_json_keys<bool>(
        json, std::vector<std::string>{"speed_adjust", "enable_speed_adjust"});
    min_acc_limit = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_acc_limit"});
    min_jerk_limit = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_jerk_limit"});
    min_dec_filter_speed = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_dec_filter_speed"});
    max_acc_filter_speed = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "max_acc_filter_speed"});
    max_acc_limit_lower = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "max_acc_limit_lower"});
    max_acc_limit_upper = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "max_acc_limit_upper"});
    max_jerk_limit_lower = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "max_jerk_limit_lower"});
    max_jerk_limit_upper = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "max_jerk_limit_upper"});
    min_acc_limit_upper = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_acc_limit_upper"});
    min_acc_limit_lower = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_acc_limit_lower"});
    min_jerk_limit_lower = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_jerk_limit_lower"});
    min_jerk_limit_upper = read_json_keys<double>(
        json, std::vector<std::string>{"speed_adjust", "min_jerk_limit_upper"});
    min_dec_filter_speed_in_deceleration_scene = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_adjust",
                                 "min_dec_filter_speed_in_deceleration_scene"});
  }

  double min_acc_limit = -4.0;
  double min_jerk_limit = -6.0;
  double min_dec_adjust_limit = 9.0;         // kph
  double max_acc_adjust_ratio_lower = 1.07;  // lower ratio
  double max_acc_adjust_ratio_upper = 1.15;  // upper ratio
  double min_dec_filter_speed = 12.0;        // filter lower slot speed
  double max_acc_filter_speed = 12.0;        // filter lower slot speed
  bool enable_speed_adjust = true;
  double max_acc_limit_lower = 0.9;
  double max_acc_limit_upper = 1.8;
  double max_jerk_limit_lower = 1.0;
  double max_jerk_limit_upper = 2.5;
  double min_acc_limit_upper = -0.8;
  double min_acc_limit_lower = -1.8;
  double min_jerk_limit_upper = -1.0;
  double min_jerk_limit_lower = -2.5;
  double min_dec_filter_speed_in_deceleration_scene = 30.0;
};

struct LaneBorrowDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);

    kMaxConcernObsDistance = read_json_keys<double>(
    json, std::vector<std::string>{"lane_borrow",
                                    "kMaxConcernObsDistance"});
    kObsStaticVelThold = read_json_keys<double>(
    json, std::vector<std::string>{"lane_borrow",
                                    "kObsStaticVelThold"});
    kObserveFrames = read_json_keys<int>(
    json, std::vector<std::string>{"lane_borrow",
                                    "kObserveFrames"});
    static_obs_buffer =
    read_json_key<double>(json, "static_obs_buffer", static_obs_buffer);

  }
  double kMaxConcernObsDistance =  40.0;
  double kObsStaticVelThold =  0.1;
  int kObserveFrames =  30;
  double static_obs_buffer = 0.5;
};

struct ActRequestConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    // /* read config from json */
    enable_act_request_function = read_json_keys<bool>(
        json,
        std::vector<std::string>{"act_request", "enable_act_request_function"});
    enable_speed_threshold = read_json_keys<int>(
        json,
        std::vector<std::string>{"act_request", "enable_speed_threshold"});
  }
  bool enable_act_request_function = true;
  double enable_speed_threshold = 60.0;
};

struct ScenarioDisplayStateConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    // /* read config from json */
    ready_remain_time = read_json_keys<int>(
        json, std::vector<std::string>{"display_state", "ready_remain_time"});
    wait_remain_time = read_json_keys<int>(
        json, std::vector<std::string>{"display_state", "wait_remain_time"});
    int_rqt_cnt_threshold = read_json_keys<int>(
        json,
        std::vector<std::string>{"int_request", "request_count_threshold"});
    map_int_cancel_freeze_cnt = read_json_keys<int>(
        json,
        std::vector<std::string>{"int_request", "map_int_cancel_freeze_cnt"});
    model_int_cancel_freeze_cnt = read_json_keys<int>(
        json,
        std::vector<std::string>{"int_request", "model_int_cancel_freeze_cnt"});
    finish_remain_time = read_json_keys<int>(
        json, std::vector<std::string>{"display_state", "finish_remain_time"});
    enable_confirm_mode = read_json_keys<bool>(
        json, std::vector<std::string>{"confirm_mode", "enable_confirm_mode"});
    map_confirm_cancel_freeze_cnt = read_json_keys<int>(
        json, std::vector<std::string>{"confirm_mode",
                                       "map_confirm_cancel_freeze_cnt"});
    model_confirm_cancel_freeze_cnt = read_json_keys<int>(
        json, std::vector<std::string>{"confirm_mode",
                                       "model_confirm_cancel_freeze_cnt"});
    enalbe_display_function = read_json_keys<bool>(
        json,
        std::vector<std::string>{"display_state", "enable_display_function"});
    int_vel_limit = read_json_keys<double>(
        json, std::vector<std::string>{"int_request", "int_vel_limit"});
    into_ramp_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"display_state", "into_ramp_threshold"});
    close_to_split_merge_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"display_state",
                                       "close_to_split_merge_threshold"});
    avoid_truck_time_distance_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"display_state",
                                       "avoid_truck_time_distance_threshold"});
    disallow_cancel_int_lc_lateral_thr = read_json_keys<double>(
        json, std::vector<std::string>{"int_request",
                                       "disallow_cancel_int_lc_lateral_thr"});
    enable_int_request_function = read_json_keys<bool>(
        json,
        std::vector<std::string>{"int_request", "enable_int_request_function"});
    enable_hnp_function = read_json_key<bool>(json, "enable_hnp_functions");
  }

  int ready_remain_time = 2;
  int wait_remain_time = 100;
  int finish_remain_time = 1;
  double int_vel_limit = 0.0;
  double Kkph2m_s = 3.6;
  double into_ramp_threshold = 2000.0;               // meter
  double close_to_split_merge_threshold = 100.0;     // meter
  double avoid_truck_time_distance_threshold = 1.5;  // seconds
  double disallow_cancel_int_lc_lateral_thr = 1.5;
  int int_rqt_cnt_threshold = 2;
  int map_int_cancel_freeze_cnt = 50;
  int model_int_cancel_freeze_cnt = 100;
  int map_confirm_cancel_freeze_cnt = 50;
  int model_confirm_cancel_freeze_cnt = 100;
  bool enable_hnp_function = false;
  bool enalbe_display_function = false;
  bool enable_int_request_function = false;
  bool enable_confirm_mode = false;
};
struct GapSelectorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    default_lc_time = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "default_lc_time"});
    collision_check_length_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector",
                                       "collision_check_length_threshold"});
    lc_premove_time = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "lc_premove_time"});
    near_car_ttc = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "near_car_ttc"});
    use_ego_v = read_json_keys<bool>(
        json, std::vector<std::string>{"gap_selector", "use_ego_v"});
    lb_t_max = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "lb_t_max"});
    lb_t_min = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "lb_t_min"});
    lb_heading_error_max = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "lb_heading_error_max"});
    lb_heading_error_min = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "lb_heading_error_min"});
    use_gs = read_json_keys<bool>(
        json, std::vector<std::string>{"gap_selector", "use_gs"});
    read_json_vec<double>(
        json, std::vector<std::string>{"gap_selector", "lat_ref_offset"},
        lat_ref_offset);
    min_ego_v_cruise = read_json_keys<double>(
        json, std::vector<std::string>{"gap_selector", "min_ego_v_cruise"});
  }

  double default_lc_time = 6.0;
  double collision_check_length_threshold = 2.2;
  double lc_premove_time = 1.5;
  double near_car_ttc = 0.2;
  bool use_ego_v = false;
  double lb_t_min = 1.5;
  double lb_t_max = 5.0;
  double lb_heading_error_min = 4.5;
  double lb_heading_error_max = 0.5;
  bool use_gs = true;
  std::vector<double> lat_ref_offset{0.0, 0.1, 0.2, 0.3};
  double min_ego_v_cruise = 2.0;
};

struct PotentialAvoidDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    near_car_thr = read_json_key<double>(json, "near_car_thr", near_car_thr);
    lat_safety_buffer =
        read_json_key<double>(json, "lat_safety_buffer", lat_safety_buffer);
    oversize_veh_addition_buffer = read_json_key<double>(
        json, "oversize_veh_addition_buffer", oversize_veh_addition_buffer);
    traffic_cone_thr =
        read_json_key<double>(json, "traffic_cone_thr", traffic_cone_thr);
    static_obs_buffer =
        read_json_key<double>(json, "static_obs_buffer", static_obs_buffer);
    near_car_hysteresis =
        read_json_key<double>(json, "near_car_hysteresis", near_car_hysteresis);
    in_range_v = read_json_key<double>(json, "in_range_v", in_range_v);
    in_range_v_hysteresis = read_json_key<double>(json, "in_range_v_hysteresis",
                                                  in_range_v_hysteresis);
    potential_near_car_thr = read_json_key<double>(
        json, "potential_near_car_thr", potential_near_car_thr);
    potential_near_car_v_ub = read_json_key<double>(
        json, "potential_near_car_v_ub", potential_near_car_v_ub);
    potential_near_car_v_lb = read_json_key<double>(
        json, "potential_near_car_v_lb", potential_near_car_v_lb);
    use_lat_offset_in_tracklet_maintainer =
        read_json_key<bool>(json, "use_lat_offset_in_tracklet_maintainer",
                            use_lat_offset_in_tracklet_maintainer);
    enable_static_scene =
        read_json_key<bool>(json, "enable_static_scene", enable_static_scene);
    car_addition_decre_factor = read_json_key<double>(
        json, "car_addition_decre_factor", car_addition_decre_factor);
    car_addition_decre_buffer = read_json_key<double>(
        json, "car_addition_decre_buffer", car_addition_decre_buffer);
    emegency_cutin_ttc_lower = read_json_key<double>(
        json, "emegency_cutin_ttc_lower", emegency_cutin_ttc_lower);
    emegency_cutin_ttc_upper = read_json_key<double>(
        json, "emegency_cutin_ttc_upper", emegency_cutin_ttc_upper);
    emegency_cutin_front_area = read_json_key<double>(
        json, "emegency_cutin_front_area", emegency_cutin_front_area);
    read_json_vec<double>(json, "expand_ego_vel", expand_ego_vel);
    read_json_vec<double>(json, "expand_obs_rel_vel", expand_obs_rel_vel);
  }
  double near_car_thr = 0.3;
  double lat_safety_buffer = 0.7;
  double oversize_veh_addition_buffer = 0.15;
  double traffic_cone_thr = 0.15;
  double static_obs_buffer = 3.4;
  double near_car_hysteresis = 1.3;
  double in_range_v = 1.0;
  double in_range_v_hysteresis = 1.5;
  double potential_near_car_thr = 0.5;
  double potential_near_car_v_ub = -0.2;
  double potential_near_car_v_lb = -0.03;
  bool use_lat_offset_in_tracklet_maintainer = false;
  bool enable_static_scene = false;
  double car_addition_decre_factor = 1;
  double car_addition_decre_buffer = 0.1;
  double emegency_cutin_ttc_lower = 1.5;
  double emegency_cutin_ttc_upper = 3;
  double emegency_cutin_front_area = 5;
  std::vector<double> expand_ego_vel{0, 10};
  std::vector<double> expand_obs_rel_vel{0, 1};
};

struct LateralOffsetDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    is_valid_lateral_offset = read_json_key<bool>(
        json, "is_valid_lateral_offset", is_valid_lateral_offset);
    base_nudge_distance =
        read_json_key<double>(json, "base_nudge_distance", base_nudge_distance);
    nudge_buffer_road_boundary =
        read_json_key<double>(json, "nudge_buffer_road_boundary");
    nudge_buffer_lane_boundary =
        read_json_key<double>(json, "nudge_buffer_lane_boundary");
    static_nudge_buffer_lane_boundary =
        read_json_key<double>(json, "static_nudge_buffer_lane_boundary");
    nudge_value_way = read_json_key<bool>(json, "nudge_value_way");
    avd_lon_distance_1 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider",
                                 "avd_lon_distance_1"},
        avd_lon_distance_1);
    avd_lon_distance_2 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider",
                                 "avd_lon_distance_2"},
        avd_lon_distance_2);
    avd_lon_distance_3 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider",
                                 "avd_lon_distance_3"},
        avd_lon_distance_3);
    avd_lon_distance_4 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider",
                                 "avd_lon_distance_4"},
        avd_lon_distance_4);
    avd_lon_distance_5 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider",
                                 "avd_lon_distance_5"},
        avd_lon_distance_5);
    avd_vrel_bp_1 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider", "avd_vrel_bp_1"},
        avd_vrel_bp_1);
    avd_vrel_bp_2 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider", "avd_vrel_bp_2"},
        avd_vrel_bp_2);
    avd_vrel_bp_3 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider", "avd_vrel_bp_3"},
        avd_vrel_bp_3);
    avd_vrel_bp_4 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider", "avd_vrel_bp_4"},
        avd_vrel_bp_4);
    avd_vrel_bp_5 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lateral_offset_decider", "avd_vrel_bp_5"},
        avd_vrel_bp_5);
    care_dynamic_object_t_threshold =
        read_json_key<double>(json, "care_dynamic_object_t_threshold",
                              care_dynamic_object_t_threshold);
    care_static_object_t_threshold = read_json_key<double>(
        json, "care_static_object_t_threshold", care_static_object_t_threshold);
    v_limit_max = read_json_key<double>(json, "v_limit_max", v_limit_max);
  }
  double v_limit_max = 30;
  bool is_valid_lateral_offset = false;
  double nudge_buffer_road_boundary = 0.3;
  double nudge_buffer_lane_boundary = 0.1;
  double static_nudge_buffer_lane_boundary = 0.1;
  double base_nudge_distance = 0.6;
  bool nudge_value_way = true;
  double avd_lon_distance_1;
  double avd_lon_distance_2;
  double avd_lon_distance_3;
  double avd_lon_distance_4;
  double avd_lon_distance_5;
  double avd_vrel_bp_1;
  double avd_vrel_bp_2;
  double avd_vrel_bp_3;
  double avd_vrel_bp_4;
  double avd_vrel_bp_5;
  double care_dynamic_object_t_threshold = 0.0;
  double care_static_object_t_threshold = 0.0;
};

struct GeneralLateralDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    hard_buffer2static_agent = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "hard_buffer2static_agent"},
        hard_buffer2static_agent);

    soft_buffer2lane = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "soft_buffer2lane"},
        soft_buffer2lane);
    extra_soft_buffer2road = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_soft_buffer2road"},
        extra_soft_buffer2road);
    hard_buffer2lane = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "hard_buffer2lane"},
        hard_buffer2lane);
    hard_buffer2road = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "hard_buffer2road"},
        hard_buffer2road);
    lateral_ref_traj_type =
        read_json_keys<bool>(json,
                             std::vector<std::string>{"general_lateral_decider",
                                                      "lateral_ref_traj_type"},
                             lateral_ref_traj_type);
    care_dynamic_object_t_threshold = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "care_dynamic_object_t_threshold"},
        care_dynamic_object_t_threshold);
    care_static_object_t_threshold = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "care_static_object_t_threshold"},
        care_static_object_t_threshold);

    soft_min_distance_road2center = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "soft_min_distance_road2center"},
        soft_min_distance_road2center);
    hard_min_distance_road2center = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "hard_min_distance_road2center"},
        hard_min_distance_road2center);

    care_lon_area_road_border = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "care_lon_area_road_border"},
        care_lon_area_road_border);

    ramp_limit_v = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "ramp_limit_v"},
        ramp_limit_v);
    ramp_limit_v_valid =
        read_json_keys<bool>(json,
                             std::vector<std::string>{"general_lateral_decider",
                                                      "ramp_limit_v_valid"},
                             ramp_limit_v_valid);
    min_v_cruise = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "min_v_cruise"},
        min_v_cruise);
    lc_second_dist_thr = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "lc_second_dist_thr"},
        lc_second_dist_thr);

    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "dynamic_ref_buffer"},
                          dynamic_ref_buffer);
    not_use_gap_flag = read_json_keys<bool>(
        json,
        std::vector<std::string>{"general_lateral_decider", "not_use_gap_flag"},
        not_use_gap_flag);

    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "lateral_road_boader_collision_ttc_bp"},
        lateral_road_boader_collision_ttc_bp);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_collision_lateral_buffer"},
        extra_collision_lateral_buffer);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "lateral_road_boader_v_bp"},
                          lateral_road_boader_v_bp);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "extra_lateral_buffer"},
                          extra_lateral_buffer);

    nudge_extra_buffer_in_intersection = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "nudge_extra_buffer_in_intersection"},
        nudge_extra_buffer_in_intersection);

    map_bound_weight[BoundType::AGENT] = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "bound_static_agent_weight"},
        0.1);
    map_bound_weight[BoundType::DYNAMIC_AGENT] = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "bound_dynamic_agent_weight"},
        0.1);
    map_bound_weight[BoundType::ADJACENT_AGENT] = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "bound_adjacent_agent_weight"},
        0.1);
    map_bound_weight[BoundType::ROAD_BORDER] = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "bound_road_border_weight"},
        0.1);

    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "relative_positon_bp"},
                          _relative_positon_bp);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "relative_position_decrease_extra_buffer"},
        _relative_positon_decrease_extra_buffer);

    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "relative_v_bp"},
        _relative_v_bp);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "relative_v_decrease_extra_buffer"},
        _relative_v_decrease_extra_buffer);

    truck_decrease_extra_buffer = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "truck_decrease_extra_buffer"},
        truck_decrease_extra_buffer);
    care_exceed_distance_with_blocked_obstacle = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "care_exceed_distance_with_blocked_obstacle"},
        care_exceed_distance_with_blocked_obstacle);
    extra_hard_buffer2blockobstacle = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_hard_buffer2blockobstacle"},
        extra_hard_buffer2blockobstacle);
    extra_front_lon_buffer2blockobstacle = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_front_lon_buffer2blockobstacle"},
        extra_front_lon_buffer2blockobstacle);
    extra_rear_lon_buffer2blockobstacle = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_rear_lon_buffer2blockobstacle"},
        extra_rear_lon_buffer2blockobstacle);
    /* read config from json */
  }
  double desired_vel = 11.11;                    // KPH_40;
  double l_care_width = 10.;                     // TBD: more beautiful
  double care_obj_lat_distance_threshold = 30.;  // TBD: more beautiful
  double care_obj_lon_distance_threshold = 60.;  // TBD: more beautiful
  double hard_buffer2static_agent = 0.15;
  double dynamic_obj_safe_buffer = 0.8;      //
  double min_obstacle_avoid_distance = 0.2;  // check it
  double lateral_bound_converge_speed = 1.0;
  double kPhysicalBoundWeight = 10.;
  double kSolidLaneBoundWeight = 5;
  double kVirtualLaneBoundWeight = 1;
  double kHardBoundWeight = -1.;
  double dynamic_bound_slack_coefficient = 1.;
  double soft_buffer2lane = 0.0;
  double extra_soft_buffer2road = 0.0;
  double hard_buffer2lane = 0.0;
  double hard_buffer2road = 0.0;
  double soft_min_distance_road2center = 0.2;
  double hard_min_distance_road2center = 0.2;
  double l_offset_limit = 0.1;
  double min_gain_vel = 1.0;
  double refine_lat_ref_threshold = 0.5;
  double delta_t = 0.2;
  double num_step = 25;
  double sample_step = 1.4;
  double sample_forward_distance = 1.0;
  double lane_change_duration = 6.6;
  double care_dynamic_object_t_threshold = 3.5;
  double care_static_object_t_threshold = 4.5;
  double care_area_s_len = 5.0;
  double max_ref_curvature = 0.5;
  bool lateral_ref_traj_type = false;
  double max_lateral_ttc = 5.0;
  double care_lon_area_road_border = 100;
  double ramp_limit_v = 19.44;
  bool ramp_limit_v_valid = false;
  double lc_second_dist_thr = 1.5;
  std::vector<double> dynamic_ref_buffer{0.0, 0.1, 0.2, 0.3};
  bool not_use_gap_flag = true;
  double min_v_cruise = 5.0;

  std::vector<double> lateral_road_boader_collision_ttc_bp{0, 1.5, 3, 4.5, 5};
  std::vector<double> extra_collision_lateral_buffer{0.1, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> lateral_road_boader_v_bp{10, 40, 60, 100, 130};
  std::vector<double> extra_lateral_buffer{0.0, 0.0, 0.0, 0.0, 0.0};

  std::unordered_map<BoundType, double> map_bound_weight;
  double nudge_extra_buffer_in_intersection = 0.1;

  std::vector<double> _relative_positon_bp = {0, 1, 2, 3, 4, 5};
  std::vector<double> _relative_positon_decrease_extra_buffer = {0,   0.1, 0.2,
                                                                 0.3, 0.4, 0.5};
  std::vector<double> _relative_v_bp = {0, 1, 2, 3, 4, 5};
  std::vector<double> _relative_v_decrease_extra_buffer = {0,   0.02, 0.05,
                                                           0.1, 0.15, 0.23};
  double truck_decrease_extra_buffer = 0.05;
  double care_exceed_distance_with_blocked_obstacle = 2.0;
  double extra_hard_buffer2blockobstacle = 2.0;
  double extra_front_lon_buffer2blockobstacle = 1.0;
  double extra_rear_lon_buffer2blockobstacle = 2.0;
};

struct HppGeneralLateralDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    dynamic_obj_safe_buffer = read_json_key<double>(
        json, "dynamic_obj_safe_buffer", dynamic_obj_safe_buffer);
    static_obj_safe_buffer = read_json_key<double>(
        json, "static_obj_safe_buffer", static_obj_safe_buffer);
    care_area_s_start_buffer = read_json_key<double>(
        json, "care_area_s_start_buffer", care_area_s_start_buffer);
    max_avoid_edge =
        read_json_key<double>(json, "max_avoid_edge", max_avoid_edge);
    /* read config from json */
  }
  double desired_vel = 11.11;                    // KPH_40;
  double l_care_width = 10.;                     // TBD: more beautiful
  double care_obj_lat_distance_threshold = 30.;  // TBD: more beautiful
  double care_obj_lon_distance_threshold = 60.;  // TBD: more beautiful
  double static_obj_safe_buffer = 0.15;
  double dynamic_obj_safe_buffer = 0.8;      //
  double min_obstacle_avoid_distance = 0.2;  // check it
  double lateral_bound_converge_speed = 1.0;
  double kPhysicalBoundWeight = 10.;
  double kSolidLaneBoundWeight = 5;
  double kVirtualLaneBoundWeight = 1;
  double kHardBoundWeight = -1.;
  double dynamic_bound_slack_coefficient = 1.;
  double buffer2border = 0.15;
  double buffer2lane = 0.3;
  double l_offset_limit = 0.1;
  double min_gain_vel = 1.0;
  double lon_rear_car_filter_buffer = 10.;
  double refine_lat_ref_threshold = 0.5;
  double delta_t = 0.2;
  double sample_step = 1.4;
  double sample_forward_distance = 1.0;
  double lane_change_duration = 6.6;
  double care_dynamic_object_t_threshold = 3.0;
  double care_area_s_len = 5.0;
  double max_ref_curvature = 0.5;
  double care_area_s_start_buffer = 0.0;
  double max_avoid_edge = 2.0;
};

struct LateralMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    warm_start_enable = read_json_keys<bool>(
        json, std::vector<std::string>{"lat_motion_ilqr", "warm_start_enable"});
    delta_t = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "delta_t"});
    curv_factor = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "curv_factor"});
    min_v_cruise = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "min_v_cruise"});
    min_ego_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "min_ego_vel"});
    acc_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "acc_bound"});
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_jerk_bound"},
        map_jerk_bound);
    jerk_bound_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "jerk_bound_avoid"});
    acc_bound_lane_change = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "acc_bound_lane_change"});
    jerk_bound_lane_change = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "jerk_bound_lane_change"});
    q_ref_x = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_x"});
    q_ref_y = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_y"});
    q_ref_theta = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_theta"});
    q_continuity = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_continuity"});
    q_acc = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc"});
    q_jerk = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk"});
    q_acc_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc_bound"});
    q_jerk_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_bound"});
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qsoft_bound"},
        map_qsoft_bound);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qhard_bound"},
        map_qhard_bound);
    emergence_avoid_factor = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "emergence_avoid_factor"});
    intersection_avoid_factor = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "intersection_avoid_factor"});
    jerk_bound_intersection = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "jerk_bound_intersection"});
    q_ref_xy_intersection = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_ref_xy_intersection"});
    q_ref_theta_intersection = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_intersection"});
    q_continuity_intersection = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_continuity_intersection"});
    q_acc_intersection = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_acc_intersection"});
    q_jerk_intersection_mid = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_jerk_intersection_mid"});
    q_jerk_intersection_close = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_intersection_close"});
    q_jerk_bound_intersection = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_bound_intersection"});
    avoid_back_time = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "avoid_back_time"});
    q_ref_x_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_x_avoid"});
    q_ref_y_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_y_avoid"});
    q_ref_theta_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_theta_avoid"});
    q_acc_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc_avoid"});
    q_jerk_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_avoid"});
    avoid_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "avoid_high_vel"});
    q_ref_theta_avoid_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_avoid_high_vel"});
    q_jerk_avoid_high_vel_close = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_avoid_high_vel_close"});
    q_jerk_avoid_high_vel_middle = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_avoid_high_vel_middle"});
    q_jerk_bound_avoid_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_bound_avoid_high_vel"});
    q_ref_x_static_avoid = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_ref_x_static_avoid"});
    q_ref_y_static_avoid = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_ref_y_static_avoid"});
    q_ref_theta_static_avoid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_static_avoid"});
    q_acc_static_avoid = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_acc_static_avoid"});
    q_jerk_static_avoid_close = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_static_avoid_close"});
    q_jerk_static_avoid_middle = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_static_avoid_middle"});
    use_new_lc_param = read_json_keys<bool>(
        json, std::vector<std::string>{"lat_motion_ilqr", "use_new_lc_param"});
    lane_change_high_vel = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "lane_change_high_vel"});
    jerk_bound_lane_change_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "jerk_bound_lane_change_high_vel"});
    q_jerk_bound_lane_change = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_bound_lane_change"});
    q_jerk_bound_lane_change_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_bound_lane_change_high_vel"});
    q_ref_xy_lane_change_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_xy_lane_change_high_vel"});
    q_ref_theta_lane_change_high_vel1 = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_lane_change_high_vel1"});
    q_ref_theta_lane_change_high_vel2 = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_lane_change_high_vel2"});
    q_ref_theta_lane_change_high_vel3 = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_lane_change_high_vel3"});
    q_ref_theta_lane_change_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_lane_change_high_vel"});
    q_jerk_lane_change_high_vel = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_lane_change_high_vel"});
    read_json_vec<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "map_qrefxy_lc_high_vel"},
        map_qrefxy_lc_high_vel);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "map_qjerk_lc_high_vel"},
        map_qjerk_lc_high_vel);
    read_json_vec<double>(json,
                          std::vector<std::string>{"lat_motion_ilqr",
                                                   "map_qjerk_lc_high_vel_old"},
                          map_qjerk_lc_high_vel_old);
    q_ref_x_lane_change = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_ref_x_lane_change"});
    q_ref_y_lane_change = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_ref_y_lane_change"});
    q_ref_theta_lane_change = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_ref_theta_lane_change"});
    q_acc_lane_change = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc_lane_change"});
    q_jerk_lane_change = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_jerk_lane_change"});
    q_jerk_lane_change2 = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_jerk_lane_change2"});
    lane_change_ego_l_thr = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "lane_change_ego_l_thr"});
    q_ref_xy_lane_change_back = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_xy_lane_change_back"});
    q_ref_theta_lane_change_back = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_ref_theta_lane_change_back"});
    q_jerk_lane_change_back = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_jerk_lane_change_back"});
    jerk_bound_lane_change_back = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "jerk_bound_lane_change_back"});
    q_jerk_bound_lane_change_back = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "q_jerk_bound_lane_change_back"});
    road_curvature_radius = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "road_curvature_radius"});
    curvature_change_index = read_json_keys<size_t>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "curvature_change_index"});
    curvature_preview_distance = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "curvature_preview_distance"});
    curvature_preview_length = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "curvature_preview_length"});
    curvature_preview_step = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "curvature_preview_step"});
    motion_plan_concerned_start_index = read_json_keys<size_t>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "motion_plan_concerned_start_index"});
    motion_plan_concerned_end_index = read_json_keys<size_t>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "motion_plan_concerned_end_index"});
    valid_perception_range = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "valid_perception_range"});
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qxy"}, map_qxy);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qjerk1"},
        map_qjerk1);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qjerk2"},
        map_qjerk2);
    end_ratio_for_qrefxy = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "end_ratio_for_qrefxy"});
    end_ratio_for_qreftheta = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "end_ratio_for_qreftheta"});
    end_ratio_for_qjerk = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "end_ratio_for_qjerk"});
    lc_end_ratio_for_first_qrefxy = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "lc_end_ratio_for_first_qrefxy"});
    lc_end_ratio_for_second_qrefxy = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "lc_end_ratio_for_second_qrefxy"});
    lc_end_ratio_for_qreftheta = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "lc_end_ratio_for_qreftheta"});
    enter_ramp_on_road_time = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "enter_ramp_on_road_time"});
    q_ref_xy_split = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_xy_split"});
    q_ref_theta_split = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_theta_split"});
    q_jerk_split = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_split"});
    q_jerk_bound_split = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_jerk_bound_split"});
    jerk_bound_split = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "jerk_bound_split"});
    ramp_valid = read_json_keys<bool>(
        json, std::vector<std::string>{"lat_motion_ilqr", "ramp_valid"});
    acc_bound_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "acc_bound_ramp"});
    jerk_bound_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "jerk_bound_ramp"});
    q_ref_x_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_x_ramp"});
    q_ref_y_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_y_ramp"});
    q_ref_theta_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_theta_ramp"});
    q_acc_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc_ramp"});
    q_jerk_ramp_close = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_ramp_close"});
    q_jerk_ramp_mid = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_ramp_mid"});
    valid_perception_range_on_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "valid_perception_range_on_ramp"});
    big_theta_thr = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "big_theta_thr"});
    q_jerk_for_big_theta = read_json_keys<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "q_jerk_for_big_theta"});
  }

  bool warm_start_enable = true;
  double curv_factor = 0.35;
  double delta_t = 0.2;
  double min_ego_vel = 5.0;
  double min_v_cruise = 2.0;

  double acc_bound = 1.5;
  std::vector<double> map_jerk_bound{1.2, 1.0, 0.8, 0.25};
  double jerk_bound_avoid = 0.5;
  double acc_bound_lane_change = 3.0;
  double jerk_bound_lane_change = 5.0;
  double q_acc_bound = 200.0;
  double q_jerk_bound = 5000.0;

  std::vector<double> map_qsoft_bound{1000.0, 3000.0, 4000.0, 5000.0};
  std::vector<double> map_qhard_bound{3000.0, 5000.0, 6000.0, 7000.0};
  double emergence_avoid_factor = 2.0;
  double intersection_avoid_factor = 2.0;

  double q_ref_x = 20.0;
  double q_ref_y = 20.0;
  double q_ref_theta = 15.0;
  double q_continuity = 0.;
  double q_acc = 0.5;
  double q_jerk = 0.6;

  double jerk_bound_intersection = 0.25;
  double q_ref_xy_intersection = 50.0;
  double q_ref_theta_intersection = 500.0;
  double q_continuity_intersection = 2.0;
  double q_acc_intersection = 100.0;
  double q_jerk_intersection_mid = 500.0;
  double q_jerk_intersection_close = 500.0;
  double q_jerk_bound_intersection = 10000.0;

  double avoid_back_time = 2.0;
  double q_ref_x_avoid = 20.0;
  double q_ref_y_avoid = 20.0;
  double q_ref_theta_avoid = 15.0;
  double q_acc_avoid = 0.5;
  double q_jerk_avoid = 15.0;
  double avoid_high_vel = 2.0;
  double q_ref_theta_avoid_high_vel = 15.0;
  double q_jerk_avoid_high_vel_close = 2.0;
  double q_jerk_avoid_high_vel_middle = 2.0;
  double q_jerk_bound_avoid_high_vel = 25.0;

  double q_ref_x_static_avoid = 20.0;
  double q_ref_y_static_avoid = 20.0;
  double q_ref_theta_static_avoid = 15.0;
  double q_acc_static_avoid = 0.5;
  double q_jerk_static_avoid_close = 2.0;
  double q_jerk_static_avoid_middle = 2.0;

  bool use_new_lc_param = false;
  double lane_change_high_vel = 15.0;
  double jerk_bound_lane_change_high_vel = 0.5;
  double q_jerk_bound_lane_change = 5000.0;
  double q_jerk_bound_lane_change_high_vel = 5000.0;
  double q_ref_xy_lane_change_high_vel = 20.0;
  double q_ref_theta_lane_change_high_vel1 = 5000.0;
  double q_ref_theta_lane_change_high_vel2 = 5000.0;
  double q_ref_theta_lane_change_high_vel3 = 5000.0;
  double q_ref_theta_lane_change_high_vel = 5000.0;
  double q_jerk_lane_change_high_vel = 50.0;
  std::vector<double> map_qrefxy_lc_high_vel{200.0, 100.0, 50.0, 20.0};
  std::vector<double> map_qjerk_lc_high_vel{5.0, 10.0, 25.0, 40.0};
  std::vector<double> map_qjerk_lc_high_vel_old{5.0, 10.0, 30.0, 50.0};
  double q_ref_x_lane_change = 20.0;
  double q_ref_y_lane_change = 20.0;
  double q_ref_theta_lane_change = 15.0;
  double q_acc_lane_change = 0.5;
  double q_jerk_lane_change = 2.0;
  double q_jerk_lane_change2 = 2.0;
  double lane_change_ego_l_thr = 1.0;

  double q_ref_xy_lane_change_back = 20.0;
  double q_ref_theta_lane_change_back = 10000.0;
  double q_jerk_lane_change_back = 5.0;
  double jerk_bound_lane_change_back = 1.5;
  double q_jerk_bound_lane_change_back = 50000.0;

  double enter_ramp_on_road_time = 2.0;
  double q_ref_xy_split = 20.0;
  double q_ref_theta_split = 5000.0;
  double q_jerk_split = 5.0;
  double q_jerk_bound_split = 10000.0;
  double jerk_bound_split = 0.5;

  bool ramp_valid = false;
  double acc_bound_ramp = 3.0;
  double jerk_bound_ramp = 1.0;
  double q_ref_x_ramp = 400.0;
  double q_ref_y_ramp = 400.0;
  double q_ref_theta_ramp = 5000.0;
  double q_acc_ramp = 0.02;
  double q_jerk_ramp_close = 45.0;
  double q_jerk_ramp_mid = 10.0;
  double valid_perception_range_on_ramp = 30.0;

  double road_curvature_radius = 750.0;
  size_t curvature_change_index = 15;
  double curvature_preview_distance = 50.0;
  double curvature_preview_length = 20.0;
  double curvature_preview_step = 1.0;
  size_t motion_plan_concerned_start_index = 2;
  size_t motion_plan_concerned_end_index = 20;
  double valid_perception_range = 60.0;
  std::vector<double> map_qxy{80.0, 400.0, 500.0, 500.0};
  std::vector<double> map_qjerk1{120.0, 90.0, 60.0, 500.0};
  std::vector<double> map_qjerk2{40.0, 30.0, 30.0, 100.0};
  double end_ratio_for_qrefxy = 1.0;
  double end_ratio_for_qreftheta = 1.0;
  double end_ratio_for_qjerk = 1.0;
  double lc_end_ratio_for_first_qrefxy = 1.0;
  double lc_end_ratio_for_second_qrefxy = 1.0;
  double lc_end_ratio_for_qreftheta = 1.0;
  double big_theta_thr = 1.0;
  double q_jerk_for_big_theta = 2.0;
};

struct RealtimeLateralMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    warm_start_enable = read_json_keys<bool>(
        json, std::vector<std::string>{"lat_motion_ilqr", "warm_start_enable"});
    acc_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "acc_bound"});
    jerk_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "jerk_bound"});
    curv_factor = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "curv_factor"});
    q_ref_x = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_x"});
    q_ref_y = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_y"});
    q_ref_theta = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_ref_theta"});
    q_continuity = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_continuity"});
    q_acc = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc"});
    q_jerk = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk"});
    q_acc_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_acc_bound"});
    q_jerk_bound = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_bound"});
    q_soft_corridor = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_soft_corridor"});
    q_hard_corridor = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_hard_corridor"});
    delta_t = read_json_keys<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "delta_t"});
    motion_plan_concerned_index = read_json_keys<size_t>(
        json, std::vector<std::string>{"lat_motion_ilqr",
                                       "motion_plan_concerned_index"});
  }
  bool warm_start_enable = true;
  double acc_bound = 6.0;
  double jerk_bound = 3.5;
  double curv_factor = 0.35;

  double q_ref_x = 20.0;
  double q_ref_y = 20.0;
  double q_ref_theta = 15.0;

  double q_continuity = 0.;
  double q_acc = 0.5;
  double q_jerk = 0.6;

  double q_acc_bound = 200.0;
  double q_jerk_bound = 500.0;

  double q_soft_corridor = 200.0;
  double q_hard_corridor = 0.0;
  double delta_t = 0.2;
  size_t motion_plan_concerned_index = 20;
};

struct LongitudinalMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    q_ref_pos = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_ref_pos"});
    q_ref_vel = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_ref_vel"});
    q_acc = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_acc"});
    q_jerk = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_jerk"});
    q_soft_pos_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_soft_pos_bound"});
    q_hard_pos_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_hard_pos_bound"});
    q_sv_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_sv_bound"});
    q_vel_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_vel_bound"});
    q_acc_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_acc_bound"});
    q_jerk_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_jerk_bound"});
    q_stop_s = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_stop_s"});
  }
  double q_ref_pos = 1.0;
  double q_ref_vel = 0.05;
  double q_acc = 10.0;
  double q_jerk = 5.0;

  double q_soft_pos_bound = 2.0;
  double q_hard_pos_bound = 1000.0;
  double q_sv_bound = 1000.0;
  double q_vel_bound = 400.0;
  double q_acc_bound = 400.0;
  double q_jerk_bound = 100.0;
  double q_stop_s = 2000.0;
};

struct LateralOptimizerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};
using LateralOptimizerV2Config = LateralOptimizerConfig;

struct VisionLateralMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    nudge_buffer_road_boundary =
        read_json_key<double>(json, "nudge_buffer_road_boundary");
    nudge_buffer_lane_boundary =
        read_json_key<double>(json, "nudge_buffer_lane_boundary");
  }
  double nudge_buffer_road_boundary = 0.3;
  double nudge_buffer_lane_boundary = 0.1;
};

struct LongitudinalDeciderV3Config : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    velocity_upper_bound = read_json_key<double>(json, "velocity_upper_bound");
    velocity_upper_bound_scale_rate =
        read_json_key<double>(json, "velocity_upper_bound_scale_rate");
    acceleration_upper_bound =
        read_json_key<double>(json, "acceleration_upper_bound");
    stop_distance_to_destination =
        read_json_key<double>(json, "stop_distance_to_destination");
    velocity_limit_parking =
        read_json_key<double>(json, "velocity_limit_parking");
    pnp_collision_threshold =
        read_json_key<double>(json, "pnp_collision_threshold");
    lon_care_length = read_json_key<double>(json, "lon_care_length");
    lon_max_ignore_relative_time =
        read_json_key<double>(json, "lon_max_ignore_relative_time");
    rads_stop_distance_to_destination =
        read_json_key<double>(json, "rads_stop_distance_to_destination");
    narrow_space_width_stop_thrshld =
        read_json_key<double>(json, "narrow_space_width_stop_thrshld");
    narrow_space_distance_stop_thrshld =
        read_json_key<double>(json, "narrow_space_distance_stop_thrshld");
  }
  int lon_num_step = 25;
  double delta_time = 0.2;
  double lon_speed_decision_horizon = 100.0;
  double lane_width = 2.4;
  double lane_width_large = 3.4;
  double kDistanceSafe = 2.0;
  double enable_prediction_time = 4.0;
  double velocity_lower_bound = 1.3888889;
  double velocity_upper_bound = 27.777778;
  double velocity_upper_bound_scale_rate = 1.0;
  double acceleration_upper_bound = 1.0;
  double acceleration_lower_bound = 0.25;
  double acceleration_curvature_ratio = 5.0;
  double jerk_buffer = 0.3;
  double jerk_rate = 2.0;
  double ttc_thld = 5.0;
  double delta_jerk_thld = 1.5;
  double delta_acc_thld = 2.0;
  size_t num_jerk_bound = 3;
  double urban_max_lat_acceleration = 2.0;
  double highway_max_lat_acceleration = 2.5;
  double stop_distance_to_destination = 3.0;
  double velocity_limit_parking = 15.0;
  double pnp_collision_threshold = 0.1;
  double narrow_space_width_stop_thrshld = -2.0;
  double narrow_space_distance_stop_thrshld = 10.0;
  double lon_care_length = 120;
  double lon_max_ignore_relative_time = 2.0;
  double rads_stop_distance_to_destination = 0;
  double max_deceleration = -6.0;
};

struct AdaptiveCruiseControlConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    kMaxCentrifugalAcceleration = read_json_keys<double>(
        json,
        std::vector<std::string>{"acc_control", "kMaxCentrifugalAcceleration"});
    enable_navi_speed_control = read_json_keys<bool>(
        json,
        std::vector<std::string>{"acc_control", "enable_navi_speed_control"});
    enable_navi_time_distance_control = read_json_keys<bool>(
        json, std::vector<std::string>{"acc_control",
                                       "enable_navi_time_distance_control"});
    v_ratio_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "v_ratio_threshold"});
    t_actuator_delay = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "t_actuator_delay"});
    gain_tg = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "gain_tg"});
    dv_down = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "dv_down"});
    gain_s = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "gain_s"});
    gain_v = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "gain_v"});
    v_max_able = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "v_max_able"});
    v_min_able = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "v_min_able"});
    obstacle_max_brake = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "obstacle_max_brake"});
    kMinCurvature = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "kMinCurvature"});
    s_rel_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "s_rel_threshold"});
    enable_hnp_functions = read_json_key<bool>(json, "enable_hnp_functions");
    jerk_set_accel = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "jerk_set_accel"});
    jerk_set_brake = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "jerk_set_brake"});
    dx_ref_weight = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "dx_ref_weight"});
    acc_jerk_control_num = read_json_keys<size_t>(
        json, std::vector<std::string>{"acc_control", "acc_jerk_control_num"});
    acc_set_a = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "acc_set_a"});
    s_set_a = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "s_set_a"});
    time_distance_aggresive = read_json_keys<double>(
        json,
        std::vector<std::string>{"acc_control", "time_distance_aggresive"});
    time_distance_moderate_aggresive = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control",
                                       "time_distance_moderate_aggresive"});
    time_distance_normal = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "time_distance_normal"});
    time_distance_moderate_conservative = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control",
                                       "time_distance_moderate_conservative"});
    time_distance_conservcative = read_json_keys<double>(
        json,
        std::vector<std::string>{"acc_control", "time_distance_conservcative"});
    max_curvatrue_fixed = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "max_curvatrue_fixed"});
  }

  double t_actuator_delay = 0.9;
  double gain_tg = 1.5;
  double dv_down = 0.1;
  double gain_s = 1.5;
  double gain_v = 0.2;
  double acc_set_a = 0.1;
  double s_set_a = 0.5;
  double v_min_able = 10.0;
  double v_max_able = 125.0;
  double jerk_set_accel = 0.2;
  double jerk_set_brake = -0.2;
  double dx_ref_weight = 200.0;
  double s_rel_threshold = 3.0;
  double kMinCurvature = 0.0001;
  double v_ratio_threshold = 1.2;
  size_t acc_jerk_control_num = 5;
  double obstacle_max_brake = 5.0;
  double max_curvatrue_fixed = 2.0;
  double time_distance_normal = 2.0;
  double time_distance_aggresive = 1.2;
  double time_distance_conservcative = 3.0;
  double kMaxCentrifugalAcceleration = 2.0;
  double time_distance_moderate_aggresive = 1.5;
  double time_distance_moderate_conservative = 2.5;
  bool enable_hnp_functions = false;
  bool enable_navi_speed_control = false;
  bool enable_navi_time_distance_control = false;
};

struct StartStopEnableConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_start_stop_function = read_json_keys<bool>(
        json,
        std::vector<std::string>{"start_stop", "enable_start_stop_function"});
    enable_hnp_functions = read_json_key<bool>(json, "enable_hnp_functions");
    ego_stop_v = read_json_keys<double>(
        json, std::vector<std::string>{"start_stop", "ego_stop_v"});
    leadone_s_lower = read_json_keys<double>(
        json, std::vector<std::string>{"start_stop", "leadone_s_lower"});
    leadone_s_upper = read_json_keys<double>(
        json, std::vector<std::string>{"start_stop", "leadone_s_upper"});
    leadone_v_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"start_stop", "leadone_v_threshold"});
    dx_ref_weight = read_json_keys<double>(
        json, std::vector<std::string>{"start_stop", "dx_ref_weight"});
  }

  bool enable_start_stop_function = false;
  bool enable_hnp_functions = false;
  double ego_stop_v = 0.0;
  double leadone_s_lower = 0.0;
  double leadone_s_upper = 0.0;
  double leadone_v_threshold = 0.0;
  double dx_ref_weight = 200.0;
};

struct MrcConditionConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    enable_mrc_condition = read_json_keys<bool>(
        json, std::vector<std::string>{"mrc_condition", "enable_mrc_condition"},
        enable_mrc_condition);
    fcw_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "fcw_threshold"},
        fcw_threshold);
    a_brake_slow = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "a_brake_slow"},
        a_brake_slow);
    a_brake_hard = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "a_brake_hard"},
        a_brake_hard);
    a_brake_emergency = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "a_brake_emergency"},
        a_brake_emergency);
    jerk_brake_slow = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "jerk_brake_slow"},
        jerk_brake_slow);
    jerk_brake_hard = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "jerk_brake_hard"},
        jerk_brake_hard);
    jerk_brake_emergency = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "jerk_brake_emergency"},
        jerk_brake_emergency);
    time_out_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "time_out_threshold"},
        time_out_threshold);
    split_merge_dist_threshold = read_json_keys<double>(
        json,
        std::vector<std::string>{"mrc_condition", "split_merge_dist_threshold"},
        split_merge_dist_threshold);
    ramp_dist_threshold = read_json_keys<double>(
        json, std::vector<std::string>{"mrc_condition", "ramp_dist_threshold"},
        ramp_dist_threshold);
    tunnel_dist_threshold = read_json_keys<double>(
        json,
        std::vector<std::string>{"mrc_condition", "tunnel_dist_threshold"},
        tunnel_dist_threshold);
    construction_zone_dist_threshold = read_json_keys<double>(
        json,
        std::vector<std::string>{"mrc_condition",
                                 "construction_zone_dist_threshold"},
        construction_zone_dist_threshold);
    v_pull_over_threshold = read_json_keys<double>(
        json,
        std::vector<std::string>{"mrc_condition", "v_pull_over_threshold"},
        v_pull_over_threshold);
  }

  bool enable_mrc_condition = false;
  int time_out_threshold = 100;
  double fcw_threshold = 2.5;
  double a_brake_slow = -1.5;
  double a_brake_hard = -5.0;
  double a_brake_emergency = -3.0;
  double jerk_brake_slow = -0.2;
  double jerk_brake_hard = -0.4;
  double jerk_brake_emergency = -0.4;
  double split_merge_dist_threshold = 150.0;
  double ramp_dist_threshold = 1000.0;
  double tunnel_dist_threshold = 1000.0;
  double construction_zone_dist_threshold = 1000.0;
  double v_pull_over_threshold = 20 / 3.6;
};

struct LongitudinalOptimizerV3Config : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }

  std::string optimization_type = "refine";
  uint max_iteration_num = 30000;
  bool enable_longitudinal_optimization_backup{false};
  double acc_stop = -4.0;
};

struct VisionLongitudinalBehaviorPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    preview_x = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "preview_x"});
    dis_zero_speed = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_zero_speed"});
    dis_zero_speed_accident = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_zero_speed_accident"});
    ttc_brake_hysteresis = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "ttc_brake_hysteresis"});
    t_curv = read_json_keys<double>(
        json,
        std::vector<std::string>{"real_time_long_behavior_planner", "t_curv"});
    dis_curv = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_curv"});
    v_start = read_json_keys<double>(
        json,
        std::vector<std::string>{"real_time_long_behavior_planner", "v_start"});
    obstacle_v_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "obstacle_v_start"});
    distance_stop = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "distance_stop"});
    distance_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "distance_start"});
    dece_to_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dece_to_ramp"});
    v_limit_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_limit_ramp"});
  }
  double preview_x = 80.0;
  double dis_zero_speed = 3.5;
  double dis_zero_speed_accident = 6;
  double ttc_brake_hysteresis = 0.3;
  double t_curv = 3.0;
  double dis_curv = 50;
  double dece_to_ramp = -1.0;
  double v_limit_ramp = 40.0;
  // The param for StartStopState
  double v_start = 0.3;
  double obstacle_v_start = 0.5;  // start of obstacle
  double distance_stop = 1.0;
  double distance_start = 0.3;
};

struct RealTimeLonBehaviorPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    preview_x = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "preview_x"});
    dis_zero_speed = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_zero_speed"});
    dis_zero_speed_accident = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_zero_speed_accident"});
    safe_distance_ttc = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "safe_distance_ttc"});
    enable_lead_two = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_lead_two"});
    enable_rss_model = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_rss_model"});
    cruise_set_acc = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "cruise_set_acc"});
    cruise_set_dec = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "cruise_set_dec"});
    v_start = read_json_keys<double>(
        json,
        std::vector<std::string>{"real_time_long_behavior_planner", "v_start"});
    obstacle_v_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "obstacle_v_start"});
    distance_stop = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "distance_stop"});
    distance_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "distance_start"});
    fast_lead_distance_step = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "fast_lead_distance_step"});
    slow_lead_distance_step = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "slow_lead_distance_step"});

    cut_in_desired_distance_step = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "cut_in_desired_distance_step"});
    soft_bound_corridor_t = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "soft_bound_corridor_t"});
  }
  int lon_num_step = 25;
  double delta_time = 0.2;
  bool enable_lead_two = false;
  double safe_distance_base = 0.5;  // 最小安全距离
  double safe_distance_ttc = 0.3;   // 安全距离ttc
  bool enable_rss_model = false;    // 是否采用RSS跟车模型
  double t_actuator_delay = 0.0;    // 纵向执行响应延时
  double a_max_comfort_accel =
      2.0;  // maximum comfortable acceleration of the ego
  double a_max_comfort_brake =
      -2.5;  // maximum comfortable braking deceleration of the ego
  double CIPV_max_brake = -3.0;  // maximum braking deceleration of the CIPV
  double lane_keep_cutinp_threshold = 0.2;  // 车道保持时cut in车辆判断阈值
  double lane_change_cutinp_threshold = 0.6;  // 换道时cut in车辆判断阈值
  double corridor_width = 1.5;  // 通道半个宽度，用于判断cut in车辆是否侵入
  double cruise_set_acc = 1.5;   // 巡航车速增加速率 0.8m/s2
  double cruise_set_dec = -1.0;  // 巡航车速减小速率 -1m/s2

  double preview_x = 80.0;
  double dis_zero_speed = 3.5;
  double dis_zero_speed_accident = 6;
  double ttc_brake_hysteresis = 0.3;
  double t_curv = 3.0;
  double dis_curv = 0.0;
  double velocity_upper_bound = 33.33;  // 120km/h
  // The param for StartStopState
  double v_start = 0.3;
  double obstacle_v_start = 0.5;  // start of obstacle
  double distance_stop = 1.0;
  double distance_start = 0.3;
  // param for st graph
  double fast_lead_distance_step = 1.0;  // fast lead跟车距离膨胀速率1.0m/s
  double slow_lead_distance_step = 2.0;  // slow lead跟车距离膨胀速率2.0m/s
  double cut_in_desired_distance_step = 1.0;
  double soft_bound_corridor_t = 0.3;
};

struct SccLonBehaviorPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    preview_x = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "preview_x"});
    dis_zero_speed = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_zero_speed"});
    dis_zero_speed_accident = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_zero_speed_accident"});
    safe_distance_ttc = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "safe_distance_ttc"});
    enable_lead_two = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_lead_two"});
    enable_rss_model = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_rss_model"});
    cruise_set_acc = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "cruise_set_acc"});
    cruise_set_dec = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "cruise_set_dec"});
    v_start = read_json_keys<double>(
        json,
        std::vector<std::string>{"real_time_long_behavior_planner", "v_start"});
    v_startmode = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_startmode"});
    obstacle_v_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "obstacle_v_start"});
    distance_stop = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "distance_stop"});
    distance_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "distance_start"});
    fast_lead_distance_step = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "fast_lead_distance_step"});
    slow_lead_distance_step = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "slow_lead_distance_step"});

    cut_in_desired_distance_step = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "cut_in_desired_distance_step"});
    soft_bound_corridor_t = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "soft_bound_corridor_t"});
    acc_start_max_bound = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "acc_start_max_bound"});
    acc_start = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "acc_start"});
    v_target_stop_thrd = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_target_stop_thrd"});
    acc_stop_max_bound = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "acc_stop_max_bound"});
    enable_jlt = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_jlt"});
    enable_narrow_agent_limit = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_narrow_agent_limit"});
    v_limit_ramp = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_limit_ramp"});
    v_limit_near_ramp_zone = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_limit_near_ramp_zone"});
    dis_near_ramp_zone = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_near_ramp_zone"});
    brake_dis_near_ramp_zone = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "brake_dis_near_ramp_zone"});
    t_curv = read_json_keys<double>(
        json,
        std::vector<std::string>{"real_time_long_behavior_planner", "t_curv"});
    dis_curv = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dis_curv"});
    pre_accelerate_distance_for_merge = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "pre_accelerate_distance_for_merge"});
    enable_intersection_v_limit = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_intersection_v_limit"});
    v_lc_speed_adjust = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_lc_speed_adjust"});
    search_sdmap_curv_dis = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "search_sdmap_curv_dis"});
    sdmap_curv_thred = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "sdmap_curv_thred"});
    straight_ramp_v_limit = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "straight_ramp_v_limit"});
    enable_sdmap_curv_v_adjust = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_sdmap_curv_v_adjust"});
    enable_speed_adjust = read_json_keys<bool>(
        json, std::vector<std::string>{"speed_adjust", "enable_speed_adjust"});
    dangerous_ttc_thrd = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "dangerous_ttc_thrd"});
    tense_ttc_thrd = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "tense_ttc_thrd"});
    merge_desired_distance_sharp_rate = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "merge_desired_distance_sharp_rate"});
    merge_desired_distance_slow_rate = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "merge_desired_distance_slow_rate"});
    enable_merge_decision_process = read_json_keys<bool>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "enable_merge_decision_process"});
    enabe_right_lane_merge_to_ego_lane_decision_process = read_json_keys<bool>(
        json, std::vector<std::string>{
                  "real_time_long_behavior_planner",
                  "enabe_right_lane_merge_to_ego_lane_decision_process"});
    v_intersection_min_limit = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "v_intersection_min_limit"});
    acc_lower_bound_in_large_curv = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "acc_lower_bound_in_large_curv"});
    jerk_lower_in_large_curv = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "jerk_lower_in_large_curv"});
    stop_dis_before_stopline = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "stop_dis_before_stopline"});
    stop_dis_before_crosswalk = read_json_keys<double>(
        json, std::vector<std::string>{"real_time_long_behavior_planner",
                                       "stop_dis_before_crosswalk"});
  }
  int lon_num_step = 25;
  double delta_time = 0.2;
  bool enable_lead_two = false;
  double safe_distance_base = 0.5;  // 最小安全距离
  double safe_distance_ttc = 0.3;   // 安全距离ttc
  bool enable_rss_model = false;    // 是否采用RSS跟车模型
  double t_actuator_delay = 0.0;    // 纵向执行响应延时
  double a_max_comfort_accel =
      2.0;  // maximum comfortable acceleration of the ego
  double a_max_comfort_brake =
      -2.5;  // maximum comfortable braking deceleration of the ego
  double CIPV_max_brake = -3.0;  // maximum braking deceleration of the CIPV
  double lane_keep_cutinp_threshold = 0.2;  // 车道保持时cut in车辆判断阈值
  double lane_change_cutinp_threshold = 0.6;  // 换道时cut in车辆判断阈值
  double corridor_width = 1.5;  // 通道半个宽度，用于判断cut in车辆是否侵入
  double cruise_set_acc = 1.5;   // 巡航车速增加速率 0.8m/s2
  double cruise_set_dec = -1.0;  // 巡航车速减小速率 -1m/s2

  double preview_x = 80.0;
  double dis_zero_speed = 3.5;
  double dis_zero_speed_accident = 6;
  double ttc_brake_hysteresis = 0.3;
  double t_curv = 2.0;
  double dis_curv = 50.0;
  double velocity_upper_bound_in_lane_change = 37.5;  // 135km/h
  double velocity_upper_bound = 33.33;                // 135km/h
  // The param for StartStopState
  double v_start = 0.3;
  double v_startmode = 5.0;
  double obstacle_v_start = 0.5;  // start of obstacle
  double distance_stop = 1.0;
  double distance_start = 0.3;
  double acc_start_max_bound = 1.0;
  double acc_start = 1.0;
  // param for st graph
  double fast_lead_distance_step = 1.0;  // fast lead跟车距离膨胀速率1.0m/s
  double slow_lead_distance_step = 2.0;  // slow lead跟车距离膨胀速率2.0m/s
  double cut_in_desired_distance_step = 1.0;
  double soft_bound_corridor_t = 0.3;
  // bound make
  // acc
  double lower_speed_acc_upper_bound = 1.2;
  double high_speed_acc_upper_bound = 1.8;
  double low_speed_threshold_with_acc_upper_bound = 5.5;
  double high_speed_threshold_with_acc_upper_bound = 16.67;
  double lane_change_low_speed_acc_upper_bound = 2.0;
  double lane_change_high_speed_acc_upper_bound = 1.2;
  double acc_lower_bound = -4.5;
  // vel
  double kSpeedBoundFactor = 1.1;
  // jerk
  double kSlowJerkUpperBound = 6.0;
  double jerk_upper_bound = 10.0;
  double jerk_lower_bound = -5.0;
  // stop condition judge
  double v_target_stop_thrd = 0.2;
  double acc_stop_max_bound = 1.0;
  // jlt
  bool enable_jlt = true;
  // narrow agent
  bool enable_narrow_agent_limit = true;
  double v_limit_ramp = 40.0;
  double v_limit_near_ramp_zone = 40.0;
  double dis_near_ramp_zone = 1100.0;
  double brake_dis_near_ramp_zone = 800.0;
  double pre_accelerate_distance_for_merge = 80.0;
  bool enable_intersection_v_limit = false;

  double v_lc_speed_adjust = 0.5;
  double search_sdmap_curv_dis = 300.0;
  double sdmap_curv_thred = 1000.0;
  double straight_ramp_v_limit = 22.22;
  double v_intersection_min_limit = 11.11;
  double stop_dis_before_stopline = 1.0;
  double stop_dis_before_crosswalk = 3.0;
  bool enable_sdmap_curv_v_adjust = true;
  bool enable_speed_adjust = true;
  // merge
  double dangerous_ttc_thrd = 0.5;
  double tense_ttc_thrd = 1.5;
  double merge_desired_distance_sharp_rate = 1.5;
  double merge_desired_distance_slow_rate = 1.0;
  bool enable_merge_decision_process = false;
  bool enabe_right_lane_merge_to_ego_lane_decision_process = false;

  double acc_lower_bound_in_large_curv = -2.0;
  double jerk_lower_in_large_curv = -2.0;
};

struct SccLonMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    q_ref_pos = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_ref_pos"});
    q_ref_vel = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_ref_vel"});
    q_acc = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_acc"});
    q_jerk = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_jerk"});
    q_soft_pos_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_soft_pos_bound"});
    q_hard_pos_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_hard_pos_bound"});
    q_sv_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_sv_bound"});
    q_vel_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_vel_bound"});
    q_acc_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_acc_bound"});
    q_jerk_bound = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_jerk_bound"});
    q_stop_s = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_stop_s"});

    // start mode q
    q_ref_pos_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_ref_pos_startmode"});
    q_ref_vel_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_ref_vel_startmode"});
    q_acc_startmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_acc_startmode"});
    q_jerk_startmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_jerk_startmode"});
    q_soft_pos_bound_startmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr",
                                       "q_soft_pos_bound_startmode"});
    q_hard_pos_bound_startmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr",
                                       "q_hard_pos_bound_startmode"});
    q_sv_bound_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_sv_bound_startmode"});
    q_vel_bound_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_vel_bound_startmode"});
    q_acc_bound_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_acc_bound_startmode"});
    q_jerk_bound_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_jerk_bound_startmode"});
    q_stop_s_startmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_stop_s_startmode"});

    // stop mode q
    q_ref_pos_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_ref_pos_stopmode"});
    q_ref_vel_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_ref_vel_stopmode"});
    q_acc_stopmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_acc_stopmode"});
    q_jerk_stopmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr", "q_jerk_stopmode"});
    q_soft_pos_bound_stopmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr",
                                       "q_soft_pos_bound_stopmode"});
    q_hard_pos_bound_stopmode = read_json_keys<double>(
        json, std::vector<std::string>{"long_motion_ilqr",
                                       "q_hard_pos_bound_stopmode"});
    q_sv_bound_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_sv_bound_stopmode"});
    q_vel_bound_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_vel_bound_stopmode"});
    q_acc_bound_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_acc_bound_stopmode"});
    q_jerk_bound_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_jerk_bound_stopmode"});
    q_stop_s_stopmode = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_stop_s_stopmode"});
    // v_target stop threshold
    v_target_stop_thrd = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "v_target_stop_thrd"});
    q_ref_pos_speed_adjust = read_json_keys<double>(
        json,
        std::vector<std::string>{"long_motion_ilqr", "q_ref_pos_speed_adjust"});
    enable_speed_adjust = read_json_keys<bool>(
        json, std::vector<std::string>{"speed_adjust", "enable_speed_adjust"});
  }
  double q_ref_pos = 1.0;
  double q_ref_vel = 0.05;
  double q_acc = 10.0;
  double q_jerk = 5.0;

  double q_soft_pos_bound = 5.0;
  double q_hard_pos_bound = 1000.0;
  double q_sv_bound = 1000.0;
  double q_vel_bound = 400.0;
  double q_acc_bound = 400.0;
  double q_jerk_bound = 100.0;
  double q_stop_s = 2000.0;

  // only in start mode
  double q_ref_pos_startmode = 1.0;
  double q_ref_vel_startmode = 0.05;
  double q_acc_startmode = 10.0;
  double q_jerk_startmode = 5.0;

  double q_soft_pos_bound_startmode = 5.0;
  double q_hard_pos_bound_startmode = 1000.0;
  double q_sv_bound_startmode = 1000.0;
  double q_vel_bound_startmode = 400.0;
  double q_acc_bound_startmode = 400.0;
  double q_jerk_bound_startmode = 100.0;
  double q_stop_s_startmode = 2000.0;

  // only in stop mode
  double v_target_stop_thrd = 0.2;
  double q_ref_pos_stopmode = 1.0;
  double q_ref_vel_stopmode = 0.05;
  double q_acc_stopmode = 10.0;
  double q_jerk_stopmode = 5.0;

  double q_soft_pos_bound_stopmode = 5.0;
  double q_hard_pos_bound_stopmode = 1000.0;
  double q_sv_bound_stopmode = 1000.0;
  double q_vel_bound_stopmode = 400.0;
  double q_acc_bound_stopmode = 400.0;
  double q_jerk_bound_stopmode = 100.0;
  double q_stop_s_stopmode = 2000.0;

  bool enable_speed_adjust = true;
  double q_ref_pos_speed_adjust = 10.0;
};

struct ResultTrajectoryGeneratorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_lat_traj = read_json_keys<bool>(
        json, std::vector<std::string>{"result_trajectory_generator",
                                       "enable_lat_traj"});
  }
  double planning_result_delta_time = 0.025;
  bool is_pwj_planning = false;
  bool enable_lat_traj = false;
};

struct TrafficLightDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_tfl_decider = read_json_keys<bool>(
        json, std::vector<std::string>{"traffic_light_decider",
                                       "enable_tfl_decider"});
    virtual_dis_before_stopline = read_json_keys<double>(
        json, std::vector<std::string>{"traffic_light_decider",
                                       "virtual_dis_before_stopline"});
    stopline_tfl_dis_thred = read_json_keys<double>(
        json, std::vector<std::string>{"traffic_light_decider",
                                       "stopline_tfl_dis_thred"});
  }

  bool enable_tfl_decider = false;
  double virtual_dis_before_stopline = 20;
  double stopline_tfl_dis_thred = 50;
};

struct CrossingAgentDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_crossing_decider = read_json_keys<bool>(
        json, std::vector<std::string>{"crossing_agent_decider",
                                       "enable_crossing_decider"});
    /*                                   
    virtual_dis_before_stopline = read_json_keys<double>(
        json, std::vector<std::string>{"traffic_light_decider",
                                       "virtual_dis_before_stopline"});
    stopline_tfl_dis_thred = read_json_keys<double>(
        json, std::vector<std::string>{"traffic_light_decider",
                                       "stopline_tfl_dis_thred"});
    */
  }

  bool enable_crossing_decider = false;
};

struct MapRequestConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    aggressive_lane_change_distance_highway =
        read_json_key<double>(json, "aggressive_lane_change_distance_highway");
    aggressive_lane_change_distance_urban =
        read_json_key<double>(json, "aggressive_lane_change_distance_urban");
  }
  double aggressive_lane_change_distance_highway{0.0};
  double aggressive_lane_change_distance_urban{0.0};
};

struct EgoPlanningObstacleManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_bbox_mode = read_json_key<bool>(
        json, "enable_bbox_mode",
        enable_bbox_mode);  // obstacle boundary construction
    max_speed_static_obstacle =
        read_json_key<double>(json, "max_speed_static_obstacle");
  }
  double frenet_obstacle_range_s_min = -50.0;
  double frenet_obstacle_range_s_max = 180.0;
  double frenet_obstacle_range_l_min = -50.0;
  double frenet_obstacle_range_l_max = 50.0;
  bool enable_bbox_mode = true;
  double max_speed_static_obstacle = 0.5;
};

struct EgoPlanningEgoStateManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    parking_cruise_speed = read_json_key<double>(json, "parking_cruise_speed",
                                                 parking_cruise_speed);
    max_replan_lat_err =
        read_json_key<double>(json, "max_replan_lat_err", max_replan_lat_err);
    max_replan_theta_err = read_json_key<double>(json, "max_replan_theta_err",
                                                 max_replan_theta_err);
    max_replan_lon_err =
        read_json_key<double>(json, "max_replan_lon_err", max_replan_lon_err);
    max_replan_dist_err =
        read_json_key<double>(json, "max_replan_dist_err", max_replan_dist_err);
    hpp_max_replan_lat_err = read_json_key<double>(
        json, "hpp_max_replan_lat_err", hpp_max_replan_lat_err);
    hpp_max_replan_theta_err = read_json_key<double>(
        json, "hpp_max_replan_theta_err", hpp_max_replan_theta_err);
    hpp_max_replan_lon_err = read_json_key<double>(
        json, "hpp_max_replan_lon_err", hpp_max_replan_lon_err);
    hpp_max_replan_dist_err = read_json_key<double>(
        json, "hpp_max_replan_dist_err", hpp_max_replan_dist_err);
    kEpsilon_v = read_json_key<double>(json, "kEpsilon_v", kEpsilon_v);
    kEpsilon_a = read_json_key<double>(json, "kEpsilon_a", kEpsilon_a);
    enable_constanct_velocity_in_predicted_vehicle_state = read_json_key<bool>(
        json, "enable_constanct_velocity_in_predicted_vehicle_state",
        enable_constanct_velocity_in_predicted_vehicle_state);
    steer_ratio = read_json_key<double>(json, "steer_ratio", steer_ratio);
    enable_delta_stitch_in_replan = read_json_key<bool>(
        json, "enable_delta_stitch_in_replan", enable_delta_stitch_in_replan);
    enable_ego_state_compensation = read_json_key<bool>(
        json, "enable_ego_state_compensation", enable_ego_state_compensation);
    read_json_vec<double>(json, "replan_longitudinal_distance_threshold_speed",
                          replan_longitudinal_distance_threshold_speed);
    read_json_vec<double>(json, "replan_longitudinal_distance_threshold_value",
                          replan_longitudinal_distance_threshold_value);
  }
  double parking_cruise_speed = 5.55;

  double max_replan_lat_err = 0.6;
  double max_replan_theta_err = 10.0;
  double max_replan_lon_err = 1.0;
  double max_replan_dist_err = 1.5;
  bool enable_delta_stitch_in_replan = false;
  std::vector<double> replan_longitudinal_distance_threshold_speed{11.111,
                                                                   27.778};
  std::vector<double> replan_longitudinal_distance_threshold_value{1.0, 1.1};
  double hpp_max_replan_lat_err = 0.45;
  double hpp_max_replan_theta_err = 12.0;
  double hpp_max_replan_lon_err = 0.55;
  double hpp_max_replan_dist_err = 0.8;

  double kEpsilon_v = 0.0;
  double kEpsilon_a = 0.0;

  double steer_ratio = 16.5;
  bool enable_constanct_velocity_in_predicted_vehicle_state = false;
  bool enable_ego_state_compensation = false;
};

struct EgoPlanningVirtualLaneManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    is_select_split_nearing_ramp = read_json_key<bool>(
        json, "is_select_split_nearing_ramp", is_select_split_nearing_ramp);
  }
  bool is_select_split_nearing_ramp = true;
};

struct EgoPlanningTrafficLightDecisionManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_traffic_light =
        read_json_key<bool>(json, "enable_traffic_light", enable_traffic_light);
  }
  bool enable_traffic_light = true;
};

struct EgoPlanningMapInfoManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct EgoPlanningObjectSelectorManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct EgoPlanningTaskPipelineConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct EgoPlanningTaskPipelineNormalConfig
    : public EgoPlanningTaskPipelineConfig {
  void init(const Json &json) override {
    EgoPlanningTaskPipelineConfig::init(json);
    /* read config from json */
    pipeline_version =
        read_json_key<std::string>(json, "pipeline_version", "v1");
  }

  std::string pipeline_version = "v1";
};

struct EgoPlanningTaskPipelineHppConfig : public EgoPlanningTaskPipelineConfig {
  void init(const Json &json) override {
    EgoPlanningTaskPipelineConfig::init(json);
    /* read config from json */
    pipeline_version =
        read_json_key<std::string>(json, "pipeline_version_hpp", "v1");
  }

  std::string pipeline_version = "v1";
};

struct EgoPlanningEvaluatorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    // enable_evaluator_check = read_json_key<bool>(
    //     json, "enable_evaluator_check",
    //     enable_evaluator_check);  // evaluator collsion check
  }

  double kFailedPenalty = -1;
  double kCollisionPenalty = -1;
  double kSafeTimeRear = 0;
  double kSafeTimeFront = 0;
  double kSafeDistanceSide = 0;
  double kMaxSafeDistanceFront = 10;
  double kMaxSafeDistanceRear = 5;
  double kMarginCoefficient = 0;
  double kCarePredictionTime = 0;
  bool enable_evaluator_check = true;
};

struct EgoPlanningKeepEvaluatorConfig : public EgoPlanningEvaluatorConfig {
  void init(const Json &json) override {
    EgoPlanningEvaluatorConfig::init(json);
    /* read config from json */
    kSafeTimeRear = 0.5;
    kSafeTimeFront = 0.5;
    kSafeDistanceSide = 0.0;
    kMarginCoefficient = 0.9;
    kCarePredictionTime = 1.0;
  }
};

struct EgoPlanningNormalChangeEvaluatorConfig
    : public EgoPlanningEvaluatorConfig {
  void init(const Json &json) override {
    EgoPlanningEvaluatorConfig::init(json);
    /* read config from json */
    kSafeTimeRear = 0.8;
    kSafeTimeFront = 0.8;
    kSafeDistanceSide = 0.3;
    kMarginCoefficient = 0.9;
    kCarePredictionTime = 3.0;
  }
};

struct EgoPlanningUrgentChangeEvaluatorConfig
    : public EgoPlanningEvaluatorConfig {
  void init(const Json &json) override {
    EgoPlanningEvaluatorConfig::init(json);
    /* read config from json */
    kSafeTimeRear = 0.0;
    kSafeTimeFront = 0.0;
    kSafeDistanceSide = 0.5;
    kMarginCoefficient = 0.9;
    kCarePredictionTime = 1.0;
  }
};

struct VisionOnlyAdasFunctionTaskConfig : public EgoPlanningConfig {
  void init(const Json &json) override { EgoPlanningConfig::init(json); }
};

struct STGraphConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
  bool enable_backward_extend_st_boundary = false;
  double backward_extend_length_for_lane_change = 50.0;
  double backward_extend_sample_resolution = 3.0;
  double lane_keeping_lower_lateral_buffer_m = 0.3;
  double lane_keeping_upper_lateral_buffer_m = 0.3;
  double lane_keeping_lower_speed_kph = 10.0;
  double lane_keeping_upper_speed_kph = 30.0;
  double lane_keeping_large_agent_lateral_buffer_m = 0.2;
  double lane_change_lateral_buffer_m = 0.2;
  double lane_keeping_large_agent_lower_lateral_buffer_m = 0.20;
  double lane_keeping_large_agent_upper_lateral_buffer_m = 0.20;
  double lane_keeping_large_agent_lower_speed_kph = 10.0;
  double lane_keeping_large_agent_upper_speed_kph = 30.0;
  double front_agent_lower_s_safety_buffer_for_lane_change = 8.0;
  double large_agent_expand_param_for_consistency = 0.20;
  double large_agent_small_expand_param_for_consistency = 0.15;
  double cone_lateral_buffer_m = 0.20;
  double lane_keeping_large_heading_diff_lon_buffer_m = 0.30;
  double person_lat_buffer_m = 0.4;
  double person_lon_buffer_m = 0.4;
  double bycicle_lat_buffer_m = 0.4;
  double bycicle_lon_buffer_m = 0.4;
  double tricycle_lat_buffer_m = 0.4;
  double tricycle_lon_buffer_m = 0.4;
  double backward_extend_time_s = 2.0;
  double reverse_vehicle_lat_buffer_m = 0.2;
};

struct StGraphSearcherConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    //  st search config
    enable_only_s_t_hash = read_json_keys<bool>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "enable_only_s_t_hash"},
        enable_only_s_t_hash);
    upper_collision_dist = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "upper_collision_dist"},
        upper_collision_dist);
    lower_collision_dist = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "lower_collision_dist"},
        lower_collision_dist);
    max_accel_limit = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "max_accel_limit"},
        max_accel_limit);
    min_accel_limit = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "min_accel_limit"},
        min_accel_limit);
    max_jerk_limit = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "max_jerk_limit"},
        max_jerk_limit);
    min_jerk_limit = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "min_jerk_limit"},
        min_jerk_limit);
    accel_sample_num = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "accel_sample_num"},
        accel_sample_num);
    s_step = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "s_step"},
        s_step);
    t_step = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "t_step"},
        t_step);
    vel_step = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                 "vel_step"},
        vel_step);
  }
  double planning_time_horizon = 5.0;
  double upper_collision_dist = 1.0;
  double lower_collision_dist = 5.0;
  double max_accel_limit = 5.0;
  double min_accel_limit = -6.0;
  double max_jerk_limit = 10.0;
  double min_jerk_limit = -10.0;
  double accel_sample_num = 20;
  double s_step = 0.25;
  double t_step = 0.25;
  double vel_step = 0.4;

  double weight_yield = 2.0;
  double weight_overtake = 2.0;
  double weight_vel = 0.10;
  double weight_accel = 0.10;
  double weight_accel_sign = 0.5;
  double weight_jerk = 1.0;

  double weight_hcost_s = 2.0;
  double weight_hcost_t = 20.0;

  double upper_truncation_time_buffer = 0.5;
  double lower_truncation_time_buffer = 0.5;

  double velocity_tolerance = 2.0;
  double proper_accel_value = 1.0;

  double max_search_time_s = 0.1;
  bool is_visualize_st_search_process = false;

  double speed_limit_scale = 1.2;

  double min_lower_collision_dist = 5.0;
  double max_lower_collision_dist = 15.0;
  double lower_collision_dist_speed_scale = 0.5;

  double weight_length_s = 0.0;
  double weight_length_t = 0.0;

  double weight_virtual_yield = 20.0;

  double min_lower_distance_buffer = 3.0;
  double min_upper_distance_buffer = 3.0;
  double rear_agent_max_start_yield_time_s = 4.0;
  double yield_front_vehicle_min_decrease_max_check_time_s = 5.0;
  double yield_front_vehicle_collision_s_buffer = 1.0;
  bool enable_only_s_t_hash = false;
};

struct AgentHeadwayConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
  double plan_time = 5.0;
  double dt = 0.2;
  double cutin_headway_threshold = 1.0;
  double smallest_headway_threshold = 1.2;
  double headway_step = 0.025;
  std::vector<std::pair<int32_t, double>> normal_headway_table = {
      {0, 1.2}, {1, 1.5}, {2, 2.0}, {3, 2.5}, {4, 3.0}};
  std::vector<std::pair<int32_t, double>> aggressive_headway_table = {
      {0, 1.2}, {1, 1.5}, {2, 2.0}, {3, 2.5}, {4, 3.0}};
  std::vector<std::pair<int32_t, double>> conservative_headway_table = {
      {0, 1.2}, {1, 1.5}, {2, 2.0}, {3, 2.5}, {4, 3.0}};
  std::vector<std::pair<int32_t, double>> aggressive_acc_max_table = {
      {5.0, 2.5}, {10.0, 1.9}, {20.0, 1.5}, {25.0, 1.3}, {35.0, 0.9}};
  std::vector<std::pair<int32_t, double>> normal_acc_max_table = {
      {5.0, 1.7}, {10.0, 1.4}, {20.0, 1.0}, {25.0, 0.8}, {35.0, 0.6}};
  std::vector<std::pair<int32_t, double>> conservative_acc_max_table = {
      {5.0, 1.2}, {10.0, 1.0}, {20.0, 0.7}, {25.0, 0.6}, {35.0, 0.4}};
  std::vector<std::pair<int32_t, double>> normal_cut_in_headway_table = {
      {0, 1.0}, {1, 1.25}, {2, 1.67}, {3, 2.08}, {4, 2.5}};
  double cut_in_velocity_upper_bound = 4.16;
  double cut_in_headway_lower_bound = 0.0;
  double cut_in_velocity_lower_bound = 3.33;
  double cut_in_headway_upper_bound = 0.2;
  // follow_distance_gap
  double lower_speed_min_follow_distance_gap = 3.0;
  double high_speed_min_follow_distance_gap = 4.5;
  double low_speed_threshold_kmph = 18;
  double high_speed_threshold_kmph = 30;
  double large_vehicle_min_follow_distance_gap = 4.5;
  double cone_min_follow_distance_gap = 4.5;
  double traffic_light_min_follow_distance_gap = 2.0;
};

struct SpeedPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    // comfort_kinematic_param
    {
      comfort_kinematic_param.acc_positive_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "acc_positive_upper",
                                 },
                                 comfort_kinematic_param.acc_positive_upper);
      comfort_kinematic_param.acc_positive_speed_lower = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "comfort_kinematic_param",
              "acc_positive_speed_lower",
          },
          comfort_kinematic_param.acc_positive_speed_lower);

      comfort_kinematic_param.acc_positive_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "acc_positive_lower",
                                 },
                                 comfort_kinematic_param.acc_positive_lower);

      comfort_kinematic_param.acc_positive_speed_upper = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "comfort_kinematic_param",
              "acc_positive_speed_upper",
          },
          comfort_kinematic_param.acc_positive_speed_upper);

      comfort_kinematic_param.acc_negative_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "acc_negative_lower",
                                 },
                                 comfort_kinematic_param.acc_negative_lower);

      comfort_kinematic_param.acc_negative_speed_lower = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "comfort_kinematic_param",
              "acc_negative_speed_lower",
          },
          comfort_kinematic_param.acc_negative_speed_lower);

      comfort_kinematic_param.acc_negative_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "acc_negative_upper",
                                 },
                                 comfort_kinematic_param.acc_negative_upper);

      comfort_kinematic_param.acc_negative_speed_upper = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "comfort_kinematic_param",
              "acc_negative_speed_upper",
          },
          comfort_kinematic_param.acc_negative_speed_upper);

      comfort_kinematic_param.jerk_positive_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "jerk_positive_upper",
                                 },
                                 comfort_kinematic_param.jerk_positive_upper);

      comfort_kinematic_param.jerk_positive_speed_lower =
          read_json_keys<double>(
              json,
              std::vector<std::string>{
                  "speed_planning",
                  "comfort_kinematic_param",
                  "jerk_positive_speed_lower",
              },
              comfort_kinematic_param.jerk_positive_speed_lower);

      comfort_kinematic_param.jerk_positive_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "jerk_positive_lower",
                                 },
                                 comfort_kinematic_param.jerk_positive_lower);

      comfort_kinematic_param.jerk_positive_speed_upper =
          read_json_keys<double>(
              json,
              std::vector<std::string>{
                  "speed_planning",
                  "comfort_kinematic_param",
                  "jerk_positive_speed_upper",
              },
              comfort_kinematic_param.jerk_positive_speed_upper);

      comfort_kinematic_param.jerk_negative_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "jerk_negative_lower",
                                 },
                                 comfort_kinematic_param.jerk_negative_lower);

      comfort_kinematic_param.jerk_negative_speed_lower =
          read_json_keys<double>(
              json,
              std::vector<std::string>{
                  "speed_planning",
                  "comfort_kinematic_param",
                  "jerk_negative_speed_lower",
              },
              comfort_kinematic_param.jerk_negative_speed_lower);

      comfort_kinematic_param.jerk_negative_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "comfort_kinematic_param",
                                     "jerk_negative_upper",
                                 },
                                 comfort_kinematic_param.jerk_negative_upper);

      comfort_kinematic_param.jerk_negative_speed_upper =
          read_json_keys<double>(
              json,
              std::vector<std::string>{
                  "speed_planning",
                  "comfort_kinematic_param",
                  "jerk_negative_speed_upper",
              },
              comfort_kinematic_param.jerk_negative_speed_upper);
    }

    // kappa_kinematic_param
    {
      kappa_kinematic_param.acc_positive_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "acc_positive_upper",
                                 },
                                 kappa_kinematic_param.acc_positive_upper);
      kappa_kinematic_param.acc_positive_speed_lower = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "acc_positive_speed_lower",
          },
          kappa_kinematic_param.acc_positive_speed_lower);

      kappa_kinematic_param.acc_positive_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "acc_positive_lower",
                                 },
                                 kappa_kinematic_param.acc_positive_lower);

      kappa_kinematic_param.acc_positive_speed_upper = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "acc_positive_speed_upper",
          },
          kappa_kinematic_param.acc_positive_speed_upper);

      kappa_kinematic_param.acc_negative_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "acc_negative_lower",
                                 },
                                 kappa_kinematic_param.acc_negative_lower);

      kappa_kinematic_param.acc_negative_speed_lower = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "acc_negative_speed_lower",
          },
          kappa_kinematic_param.acc_negative_speed_lower);

      kappa_kinematic_param.acc_negative_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "acc_negative_upper",
                                 },
                                 kappa_kinematic_param.acc_negative_upper);

      kappa_kinematic_param.acc_negative_speed_upper = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "acc_negative_speed_upper",
          },
          kappa_kinematic_param.acc_negative_speed_upper);

      kappa_kinematic_param.jerk_positive_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "jerk_positive_upper",
                                 },
                                 kappa_kinematic_param.jerk_positive_upper);

      kappa_kinematic_param.jerk_positive_speed_lower = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "jerk_positive_speed_lower",
          },
          kappa_kinematic_param.jerk_positive_speed_lower);

      kappa_kinematic_param.jerk_positive_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "jerk_positive_lower",
                                 },
                                 kappa_kinematic_param.jerk_positive_lower);

      kappa_kinematic_param.jerk_positive_speed_upper = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "jerk_positive_speed_upper",
          },
          kappa_kinematic_param.jerk_positive_speed_upper);

      kappa_kinematic_param.jerk_negative_lower =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "jerk_negative_lower",
                                 },
                                 kappa_kinematic_param.jerk_negative_lower);

      kappa_kinematic_param.jerk_negative_speed_lower = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "jerk_negative_speed_lower",
          },
          kappa_kinematic_param.jerk_negative_speed_lower);

      kappa_kinematic_param.jerk_negative_upper =
          read_json_keys<double>(json,
                                 std::vector<std::string>{
                                     "speed_planning",
                                     "kappa_kinematic_param",
                                     "jerk_negative_upper",
                                 },
                                 kappa_kinematic_param.jerk_negative_upper);

      kappa_kinematic_param.jerk_negative_speed_upper = read_json_keys<double>(
          json,
          std::vector<std::string>{
              "speed_planning",
              "kappa_kinematic_param",
              "jerk_negative_speed_upper",
          },
          kappa_kinematic_param.jerk_negative_speed_upper);
    }

    //  kappa_speed_limit_table
    {
      read_json_vec(json,
                    std::vector<std::string>{"normal_kappa_speed_limit_table",
                                             "kappa_table"},
                    normal_kappa_speed_limit_table.kappa_table);
      read_json_vec(json,
                    std::vector<std::string>{"normal_kappa_speed_limit_table",
                                             "speed_table"},
                    normal_kappa_speed_limit_table.speed_table);
      read_json_vec(json,
                    std::vector<std::string>{
                        "lane_change_kappa_speed_limit_table", "kappa_table"},
                    lane_change_kappa_speed_limit_table.kappa_table);
      read_json_vec(json,
                    std::vector<std::string>{
                        "lane_change_kappa_speed_limit_table", "speed_table"},
                    lane_change_kappa_speed_limit_table.speed_table);
    }
    lane_change_upper_speed_limit_kph = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning",
                                 "lane_change_upper_speed_limit_kph"},
        lane_change_upper_speed_limit_kph);
    enable_always_cruise = read_json_keys<bool>(
        json,
        std::vector<std::string>{"speed_planning", "debug_switch",
                                 "enable_always_cruise"},
        enable_always_cruise);
  }
  // debug switch - cruise target maker
  bool enable_always_cruise = false;

  double planning_time = 5.0;
  double dt = 0.2;
  double zero_acc_jerk_max = 0.5;
  double zero_acc_jerk_min = -1.0;
  double slow_jerk_upper_bound = 6.0;
  double max_speed_limit_curve_acc_upper_bound = 2.4;
  double max_speed_limit_curve_acc_lower_bound = -1.0;
  double max_speed_limit_curve_jerk_upper_bound = 10.0;
  double max_speed_limit_curve_jerk_lower_bound = -1.0;
  double max_speed_limit_curve_velocity_upper_bound = 37.5;
  double curr_vel_thres = 0.2;
  double max_v_thres = 0.9;
  double mean_a_thres = 0.15;
  double stop_acc_upper_bound = 0.1;
  double stop_acc_lower_bound = -0.3;
  double stop_jerk_upper_bound = 0.8;
  double stop_jerk_lower_bound = -0.8;

  // follow target
  double lower_speed_min_follow_distance_gap = 3.0;
  double high_speed_min_follow_distance_gap = 4.5;
  double low_speed_threshold_kmph = 18;
  double high_speed_threshold_kmph = 30;
  double large_vehicle_min_follow_distance_gap = 4.5;
  double cone_min_follow_distance_gap = 4.5;
  double traffic_light_min_follow_distance_gap = 2.0;

  // cruise target relevance
  double lane_change_upper_speed_limit_kph = 150.0;
  struct KinematicParam {
    double acc_positive_upper = 1.35;
    double acc_positive_speed_lower = 4.2;
    double acc_positive_lower = 0.5;
    double acc_positive_speed_upper = 12.0;
    double acc_negative_lower = -2.5;
    double acc_negative_speed_lower = 5.0;
    double acc_negative_upper = -2.0;
    double acc_negative_speed_upper = 20.0;
    double jerk_positive_upper = 4.0;
    double jerk_positive_speed_lower = 5.0;
    double jerk_positive_lower = 2.0;
    double jerk_positive_speed_upper = 25.0;
    double jerk_negative_lower = -6.0;
    double jerk_negative_speed_lower = 1.0;
    double jerk_negative_upper = -5.0;
    double jerk_negative_speed_upper = 25.0;
  };

  struct SpeedPlanningBound {
    double low_speed_acc_upper_bound = 1.8;
    double high_speed_acc_upper_bound = 1.2;
    double lane_change_low_speed_acc_upper_bound = 2.4;
    double lane_change_high_speed_acc_upper_bound = 1.6;
    double low_speed_threshold_with_acc_upper_bound = 5.5;
    double high_speed_threshold_with_acc_upper_bound = 16.67;
    double acc_lower_bound = -5.0;
    double jerk_lower_bound = -5.0;
    double jerk_upper_bound = 10.0;
  };
  
  struct KappaSpeedLimitTable {
    std::vector<double> kappa_table{
        0.0005, 0.00074, 0.00142, 0.00167, 0.0018, 0.002, 0.0025, 0.0033,
        0.005,  0.01,    0.02,    0.0333,  0.04,   0.069, 0.15,   0.2};
    std::vector<double> speed_table{150.0, 130.0, 105.0, 95.0, 90.0, 85.0,
                                    80.0,  70.0,  55.0,  47.0, 27.0, 23.0,
                                    20.0,  15.0,  10.0,  7.0};
  };

  KinematicParam comfort_kinematic_param;
  KinematicParam kappa_kinematic_param;

  SpeedPlanningBound speed_planning_bound;

  KappaSpeedLimitTable normal_kappa_speed_limit_table;
  KappaSpeedLimitTable lane_change_kappa_speed_limit_table;
};
}  // namespace planning
