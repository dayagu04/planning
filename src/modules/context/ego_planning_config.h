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

template <typename T, typename StringType>
void ReadItem(const Json &json, T &value, StringType &&key1) {
  auto key = std::string(std::forward<StringType>(key1));
  if (json.find(key) != json.end()) {
    value = json[key];
  }
}

/**
 * @brief 从json对象中读一个值。
 * 支持指定一个或者多个key，从json对象中读值。
 * 当且仅当按照给定的key列表找到对应项时，将其赋值给value，否则value保留原值。
 *
 * @tparam T 目标值类型
 * @tparam StringType 字符串类型，用于指定一个key(string literal or std::string)
 * @tparam RestKeys 其余key的类型，每个key均为字符串类型，用于递归查找
 * @param json 目标json对象
 * @param value 目标值，如果找到对应项则被赋值
 * @param key1  第一个key，用于查找json对象中的项
 * @param rest_keys 其余key
 */
template <typename T, typename StringType, typename... RestKeys>
void ReadItem(const Json &json, T &value, StringType &&key1,
              RestKeys &&...rest_keys) {
  auto key = std::string(std::forward<StringType>(key1));
  if (json.find(key) != json.end()) {
    ReadItem(json[key], value, std::forward<RestKeys>(rest_keys)...);
  }
}

template <typename T, typename StringType>
void ReadVector(const Json &json, std::vector<T> &vec, StringType &&key1) {
  auto key = std::string(std::forward<StringType>(key1));
  if (json.find(key) != json.end() && json[key].is_array()) {
    std::vector<T> res{};
    for (const auto &value : json[key]) {
      res.push_back(value);
    }
    vec.swap(res);
  }
}

template <typename T, typename StringType, typename... RestKeys>
void ReadVector(const Json &json, std::vector<T> &vec, StringType &&key1,
                RestKeys &&...rest_keys) {
  auto key = std::string(std::forward<StringType>(key1));
  if (json.find(key) != json.end()) {
    ReadVector<T>(json[key], vec, std::forward<RestKeys>(rest_keys)...);
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
    enable_use_spatio_temporal_planning = read_json_key<bool>(
        json, "enable_use_spatio_temporal_planning", false);
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
    enable_use_speed_limit_to_suppress_interactive_lane_change =
        read_json_key<bool>(json,
                            "enable_use_speed_limit_to_suppress_interactive_lane_change");

    minimum_distance_nearby_ramp_to_surpress_overtake_lane_change =
        read_json_key<double>(
            json,
            "minimum_distance_nearby_ramp_to_surpress_overtake_lane_change");
    minimum_distance_nearby_split_to_surpress_specific_direction_overtake =
        read_json_key<double>(json,
                              "minimum_distance_nearby_split_to_surpress_"
                              "specific_direction_overtake");
    minimum_ego_cruise_speed_for_active_lane_change = read_json_key<double>(
        json, "minimum_ego_cruise_speed_for_active_lane_change");
    enable_use_emergency_avoidence_lane_change_request = read_json_key<bool>(
        json, "enable_use_emergency_avoidence_lane_change_request");
    enable_use_cone_change_request =
        read_json_key<bool>(json, "enable_use_cone_change_request");
    enable_use_merge_change_request =
        read_json_key<bool>(json, "enable_use_merge_change_request");
    enable_use_ground_mark_process_split =
        read_json_key<bool>(json, "enable_use_ground_mark_process_split");
    enable_fusion_occupancy_objects =
        read_json_key<bool>(json, "enable_fusion_occupancy_objects");
    enable_fusion_parking_slot =
        read_json_key<bool>(json, "enable_fusion_parking_slot");
    enable_fusion_ground_line =
        read_json_key<bool>(json, "enable_fusion_ground_line");
    is_ground_line_cluster =
        read_json_key<bool>(json, "is_ground_line_cluster");
    enable_ehr_column_box = read_json_key<bool>(json, "enable_ehr_column_box");
    hpp_min_search_range = read_json_key<double>(json, "hpp_min_search_range");
    raw_ref_extend_buff =
        read_json_key<double>(json, "raw_ref_extend_buff", raw_ref_extend_buff);
    enable_lane_borrow_deciderV2 = read_json_key<bool>(json, "enable_lane_borrow_deciderV2");
  }
  double trajectory_time_length = 5.0;
  double planning_dt = 0.2;
  bool enable_raw_ego_prediction = false;
  bool enable_dagger = false;
  bool use_ego_prediction_model_in_planning = false;
  bool use_init_point_restore = false;
  bool enable_use_spatio_temporal_planning = false;
  int planner_type = planning::context::PlannerType::REALTIME_PLANNER;
  int active_lane_change_min_duration_threshold = 150;
  bool use_lateral_distance_to_judge_cutout_in_active_lane_change = true;
  bool use_overtake_lane_change_request_instead_of_active_lane_change_request =
      true;
  bool enable_use_speed_limit_to_suppress_interactive_lane_change =
      true;
  double minimum_distance_nearby_ramp_to_surpress_overtake_lane_change = 1500;
  double minimum_distance_nearby_split_to_surpress_specific_direction_overtake =
      1500;
  double minimum_ego_cruise_speed_for_active_lane_change = 16.67;
  bool enable_use_ground_mark_process_split = false;
  bool enable_use_emergency_avoidence_lane_change_request = false;
  bool enable_use_cone_change_request = false;
  bool enable_use_merge_change_request = false;
  bool enable_fusion_occupancy_objects = false;
  bool enable_fusion_parking_slot = false;
  bool enable_fusion_ground_line = true;
  bool is_ground_line_cluster = false;
  bool enable_ehr_column_box = false;
  double hpp_min_search_range = 20;
  double raw_ref_extend_buff = 0;
  bool enable_lane_borrow_deciderV2 =  true;
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

struct SplitProcessConfig : public EgoPlanningConfig {
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
    ReadItem<double>(json, min_dec_adjust_limit, "speed_adjust",
                     "min_dec_adjust_limit");
    ReadItem<double>(json, max_acc_adjust_ratio_lower, "speed_adjust",
                     "max_acc_adjust_ratio_lower");
    ReadItem<double>(json, max_acc_adjust_ratio_upper, "speed_adjust",
                     "max_acc_adjust_ratio_upper");
    ReadItem<bool>(json, enable_speed_adjust, "speed_adjust",
                   "enable_speed_adjust");
    ReadItem<double>(json, min_acc_limit, "speed_adjust", "min_acc_limit");
    ReadItem<double>(json, min_jerk_limit, "speed_adjust", "min_jerk_limit");
    ReadItem<double>(json, min_dec_filter_speed, "speed_adjust",
                     "min_dec_filter_speed");
    ReadItem<double>(json, max_acc_filter_speed, "speed_adjust",
                     "max_acc_filter_speed");
    ReadItem<double>(json, max_acc_limit_lower, "speed_adjust",
                     "max_acc_limit_lower");
    ReadItem<double>(json, max_acc_limit_upper, "speed_adjust",
                     "max_acc_limit_upper");
    ReadItem<double>(json, max_jerk_limit_lower, "speed_adjust",
                     "max_jerk_limit_lower");
    ReadItem<double>(json, max_jerk_limit_upper, "speed_adjust",
                     "max_jerk_limit_upper");
    ReadItem<double>(json, min_acc_limit_upper, "speed_adjust",
                     "min_acc_limit_upper");
    ReadItem<double>(json, min_acc_limit_lower, "speed_adjust",
                     "min_acc_limit_lower");
    ReadItem<double>(json, min_jerk_limit_lower, "speed_adjust",
                     "min_jerk_limit_lower");
    ReadItem<double>(json, min_jerk_limit_upper, "speed_adjust",
                     "min_jerk_limit_upper");
    ReadItem<double>(json, min_dec_filter_speed_in_deceleration_scene,
                     "speed_adjust",
                     "min_dec_filter_speed_in_deceleration_scene");
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

    ReadItem<double>(json, max_concern_obs_distance, "lane_borrow",
                     "max_concern_obs_distance");
    ReadItem<double>(json, obs_static_vel_thold, "lane_borrow",
                     "obs_static_vel_thold");
    ReadItem<int>(json, observe_frames, "lane_borrow", "observe_frames");
    centric_obs_frames = read_json_keys<int>(
        json, std::vector<std::string>{"lane_borrow", "centric_obs_frames"});
    dense_obstacle_dist = read_json_keys<double>(
        json, std::vector<std::string>{"lane_borrow", "dense_obstacle_dist"});
    extend_obs_distance = read_json_keys<double>(
        json, std::vector<std::string>{"lane_borrow", "extend_obs_distance"});
  }
  double max_concern_obs_distance = 40.0;
  double obs_static_vel_thold = 0.1;
  int observe_frames = 30;
  int centric_obs_frames = 10;
  double dense_obstacle_dist = 8.0;
  double extend_obs_distance = 10.0;
};


struct DPRoadGraphConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);

    coeff_l_cost = read_json_keys<double>(
        json,std::vector<std::string>{"dp_path_decider", "coeff_l_cost"});
    coeff_dl_cost = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_dl_cost"});
    coeff_ddl_cost = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_ddl_cost"});
    path_resolution = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "path_resolution"});
    coeff_end_l_cost = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_end_l_cost"});
    coeff_collision_cost = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_collision_cost"});
    collision_distance = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "collision_distance"});
    coeff_stitch_cost = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_stitch_cost"});
    min_sample_distance = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "min_sample_distance"});
    sample_forward_time = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "sample_forward_time"});
    min_level_distance = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "min_level_distance"});


    coeff_l_cost2 = read_json_keys<double>(
        json,std::vector<std::string>{"dp_path_decider", "coeff_l_cost2"});
    coeff_dl_cost2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_dl_cost2"});
    coeff_ddl_cost2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_ddl_cost2"});
    path_resolution2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "path_resolution2"});
    coeff_end_l_cost2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_end_l_cost2"});
    coeff_collision_cost2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_collision_cost2"});
    collision_distance2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "collision_distance2"});
    coeff_stitch_cost2 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_stitch_cost2"});

    coeff_l_cost3 = read_json_keys<double>(
        json,std::vector<std::string>{"dp_path_decider", "coeff_l_cost3"});
    coeff_dl_cost3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_dl_cost3"});
    coeff_ddl_cost3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_ddl_cost3"});
    path_resolution3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "path_resolution3"});
    coeff_end_l_cost3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_end_l_cost3"});
    coeff_collision_cost3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_collision_cost3"});
    collision_distance3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "collision_distance3"});
    coeff_stitch_cost3 = read_json_keys<double>(
        json, std::vector<std::string>{"dp_path_decider", "coeff_stitch_cost3"});


  }
   double coeff_l_cost = 6e6;
   double coeff_dl_cost = 8e3;
   double coeff_ddl_cost = 5e3;
   double path_resolution = 2.0;
   double coeff_end_l_cost = 1e6;

    double coeff_collision_cost = 1e6;
    double collision_distance = 0.8;
    double coeff_stitch_cost = 5e5;

    double min_sample_distance = 32.0;
    double sample_forward_time = 1.8;
    double min_level_distance = 10.0;
    double max_level_distance = 30.0;


    double coeff_l_cost2 = 6e6;
    double coeff_dl_cost2 = 8e3;
    double coeff_ddl_cost2 = 5e3;
    double path_resolution2 = 2.0;
    double coeff_end_l_cost2 = 1e6;

    double coeff_l_cost3 = 6e6;
    double coeff_dl_cost3 = 8e3;
    double coeff_ddl_cost3 = 5e3;
    double path_resolution3 = 2.0;
    double coeff_end_l_cost3 = 1e6;


    double coeff_collision_cost2 = 1e6;
    double collision_distance2 = 0.8;
    double coeff_stitch_cost2 = 5e5;

    double coeff_collision_cost3 = 1e6;
    double collision_distance3 = 0.8;
    double coeff_stitch_cost3 = 5e5;

};

struct DPSpeedGraphConfig: public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);

    total_time = read_json_keys<double>(
        json,std::vector<std::string>{"dp_speed_decider", "total_time"});
    total_path_length = read_json_keys<double>(
        json,std::vector<std::string>{"dp_speed_decider", "total_path_length"});
    total_path_length = read_json_keys<double>(
       json,std::vector<std::string>{"dp_speed_decider", "matrix_dimension_s"});
    total_path_length = read_json_keys<double>(
      json,std::vector<std::string>{"dp_speed_decider", "matrix_dimension_t"});
  }
  double total_path_length = 50.0;
  double total_time = 5.0;
  double matrix_dimension_s = 10;
  double matrix_dimension_t = 10;

};

struct SamplePolySpeedAdjustDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    sample_v_nums = read_json_keys<int>(
        json,
        std::vector<std::string>{"sample_poly_speed_adjust", "sample_v_nums"});
    sample_t_nums = read_json_keys<int>(
        json,
        std::vector<std::string>{"sample_poly_speed_adjust", "sample_t_nums"});
    sample_v_upper = read_json_keys<double>(
        json,
        std::vector<std::string>{"sample_poly_speed_adjust", "sample_v_upper"});
    sample_v_lower = read_json_keys<double>(
        json,
        std::vector<std::string>{"sample_poly_speed_adjust", "sample_v_lower"});
    sample_t_upper = read_json_keys<double>(
        json,
        std::vector<std::string>{"sample_poly_speed_adjust", "sample_t_upper"});
    sample_t_lower = read_json_keys<double>(
        json,
        std::vector<std::string>{"sample_poly_speed_adjust", "sample_t_lower"});
    maximum_speed_adjustment = read_json_keys<int>(
        json, std::vector<std::string>{"sample_poly_speed_adjust",
                                       "maximum_speed_adjustment"});
  }

  int sample_v_nums = 15;
  int sample_t_nums = 10;
  double sample_v_upper = 35.0;
  double sample_v_lower = 0.0;
  double sample_t_upper = 5.0;
  double sample_t_lower = 1.0;
  double maximum_speed_adjustment = 15.0 / 3.6;
};

struct ActRequestConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    // /* read config from json */
    ReadItem<bool>(json, enable_act_request_function, "act_request",
                   "enable_act_request_function");
    ReadItem<double>(json, enable_speed_threshold, "act_request",
                     "enable_speed_threshold");
  }
  bool enable_act_request_function = true;
  double enable_speed_threshold = 60.0;
};

struct DpPolyPathConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    //  st search config
    sample_points_num_each_level = read_json_keys<int>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                   "sample_points_num_each_level"},
        sample_points_num_each_level);
    step_length_max = read_json_keys<int>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "step_length_max"},
        step_length_max);
    step_length_min = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "step_length_min"},
        step_length_min);
    lateral_sample_offset = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "lateral_sample_offset"},
        lateral_sample_offset);
    lateral_adjust_coeff = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "lateral_adjust_coeff"},
        lateral_adjust_coeff);
    eval_time_interval = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "eval_time_interval"},
        eval_time_interval);
    path_resolution = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_resolution"},
        path_resolution);
    obstacle_ignore_distance = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_ignore_distance"},
        obstacle_ignore_distance);
    obstacle_longit_collision_distance = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_longit_collision_distance"},
        obstacle_longit_collision_distance);
    obstacle_longit_risk_distance = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_longit_risk_distance"},
        obstacle_longit_risk_distance);
    obstacle_collision_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_collision_cost"},
        obstacle_collision_cost);
    dynamic_obstacle_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "dynamic_obstacle_weight"},
        dynamic_obstacle_weight);
    default_obstacle_cost_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "default_obstacle_cost_weight"},
        default_obstacle_cost_weight);
    obstacle_lateral_risk_distance = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_lateral_risk_distance"},
        obstacle_lateral_risk_distance);
    obstacle_lateral_collision_distance = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_lateral_collision_distance"},
        obstacle_lateral_collision_distance);
    obstacle_collision_cost_without_lateral_overlap = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "obstacle_collision_cost_without_lateral_overlap"},
        obstacle_collision_cost_without_lateral_overlap);

    path_l_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_l_cost"},
        path_l_cost);
    path_dl_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_dl_cost"},
        path_dl_cost);
    path_ddl_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_ddl_cost"},
        path_ddl_cost);
    path_l_cost_param_l0 = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_l_cost_param_l0"},
        path_l_cost_param_l0);
    path_l_cost_param_b = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_l_cost_param_b"},
        path_l_cost_param_b);
    path_l_cost_param_k = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_l_cost_param_k"},
        path_l_cost_param_k);
    path_out_lane_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_out_lane_cost"},
        path_out_lane_cost);
    path_end_l_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_end_l_cost"},
        path_end_l_cost);
    path_l_stitching_cost_param = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "path_l_stitching_cost_param"},
        path_l_stitching_cost_param);
    stitching_cost_time_decay_factor = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "dp_poly_path_config",
                                 "stitching_cost_time_decay_factor"},
        stitching_cost_time_decay_factor);
  }

  int sample_points_num_each_level = 21;
  double step_length_max = 40.0;
  double step_length_min = 20.0;
  double lateral_sample_offset = 0.2;
  double lateral_adjust_coeff = 0.5;
  double eval_time_interval = 0.1;
  double path_resolution = 2.0;
  double obstacle_ignore_distance = 20.0;
  double obstacle_longit_collision_distance = 0.5;
  double obstacle_longit_risk_distance = 2.0;
  double obstacle_collision_cost = 1e7;
  double dynamic_obstacle_weight = 1e-4;
  double default_obstacle_cost_weight = 0.1;
  double obstacle_lateral_risk_distance = 1.0;
  double obstacle_lateral_collision_distance = 0.5;
  double obstacle_collision_cost_without_lateral_overlap = 1e5;

  double path_l_cost = 6.5;
  double path_dl_cost = 8e3;
  double path_ddl_cost = 5e1;
  double path_l_cost_param_l0 = 1.5;
  double path_l_cost_param_b = 0.4;
  double path_l_cost_param_k = 1.5;
  double path_out_lane_cost = 1e8;

  double path_end_l_cost = 1.0e2;
  double path_l_stitching_cost_param = 1.0e2;
  double stitching_cost_time_decay_factor = 0.2;
};

struct DpStSpeedOptimizerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    //  st direction search config
    total_length_t = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                   "total_length_t"},
        total_length_t);
    default_unit_t = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                   "default_unit_t"},
        default_unit_t);
    dense_dimension_s = read_json_keys<int>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "dense_dimension_s"},
        dense_dimension_s);
    dense_unit_s = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "dense_unit_s"},
        dense_unit_s);
    sparse_unit_s = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "sparse_unit_s"},
        sparse_unit_s);
    speed_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "speed_weight"},
        speed_weight);
    accel_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "accel_weight"},
        accel_weight);
    jerk_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "jerk_weight"},
        jerk_weight);
    obstacle_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "obstacle_weight"},
        obstacle_weight);
    reference_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "reference_weight"},
        reference_weight);
    go_down_buffer = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "go_down_buffer"},
        go_down_buffer);
    go_up_buffer = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "go_up_buffer"},
        go_up_buffer);

    default_obstacle_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "default_obstacle_cost"},
        default_obstacle_cost);
    default_speed_cost = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "default_speed_cost"},
        default_speed_cost);
    exceed_speed_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "exceed_speed_penalty"},
        exceed_speed_penalty);
    low_speed_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "low_speed_penalty"},
        low_speed_penalty);
    reference_speed_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "reference_speed_penalty"},
        reference_speed_penalty);
    keep_clear_low_speed_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "keep_clear_low_speed_penalty"},
        keep_clear_low_speed_penalty);
    accel_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "accel_penalty"},
        accel_penalty);
    decel_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "decel_penalty"},
        decel_penalty);
    positive_jerk_coeff = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "positive_jerk_coeff"},
        positive_jerk_coeff);
    negative_jerk_coeff = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "negative_jerk_coeff"},
        negative_jerk_coeff);
    max_acceleration = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "max_acceleration"},
        max_acceleration);
    max_deceleration = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "max_deceleration"},
        max_deceleration);
    spatial_potential_penalty = read_json_keys<double>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "spatial_potential_penalty"},
        spatial_potential_penalty);
    enable_use_parallel_calculate_cost = read_json_keys<bool>(
        json,
        std::vector<std::string>{"spatio_temporal_planner", "default_speed_config",
                                 "enable_use_parallel_calculate_cost"},
        enable_use_parallel_calculate_cost);
  }
  double total_length_t = 5.0;
  double default_unit_t = 1.0;
  int dense_dimension_s = 101;
  double dense_unit_s = 0.1;
  double sparse_unit_s = 1.0;
  double speed_weight = 0.0;
  double accel_weight = 10.0;
  double jerk_weight = 10.0;
  double obstacle_weight = 1.0;
  double reference_weight = 0.0;
  double go_down_buffer = 5.0;
  double go_up_buffer = 5.0;

  double default_obstacle_cost = 1e4;

  double default_speed_cost = 1.0;
  double exceed_speed_penalty = 1.5;
  double low_speed_penalty = 1.5;
  double reference_speed_penalty = 10.0;
  double keep_clear_low_speed_penalty = 10.0;
  double accel_penalty = 1.0;
  double decel_penalty = 1.0;

  double positive_jerk_coeff = 1.0;
  double negative_jerk_coeff = 1.0;

  double max_acceleration = 2.0;
  double max_deceleration = -4.0;
  double spatial_potential_penalty = 1.0e2;
  bool enable_use_parallel_calculate_cost = true;
};

struct SpatioTemporalGridMap : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    // /* read config from json */
    map_size_x = read_json_keys<int>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "map_size_x"});
    map_size_y = read_json_keys<int>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "map_size_y"});
    map_size_z = read_json_keys<int>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "map_size_y"});
    map_resl_x = read_json_keys<double>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "map_resl_x"});
    map_resl_y = read_json_keys<double>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "map_resl_y"});
    map_resl_z = read_json_keys<double>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "map_resl_z"});
    s_back_len = read_json_keys<double>(
        json, std::vector<std::string>{"spatio_temporal_grid_map", "s_back_len"});
  }

  int map_size_x = 1000;
  int map_size_y = 100;
  int map_size_z = 51;

  double map_resl_x = 0.25;
  double map_resl_y = 0.2;
  double map_resl_z = 0.1;
  double s_back_len = 0.0;
};

struct ScenarioDisplayStateConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    // /* read config from json */
    ReadItem<int>(json, ready_remain_time, "display_state",
                  "ready_remain_time");
    ReadItem<int>(json, wait_remain_time, "display_state", "wait_remain_time");
    ReadItem<int>(json, int_rqt_cnt_threshold, "int_request",
                  "request_count_threshold");
    ReadItem<int>(json, map_int_cancel_freeze_cnt, "int_request",
                  "map_int_cancel_freeze_cnt");
    ReadItem<int>(json, model_int_cancel_freeze_cnt, "int_request",
                  "model_int_cancel_freeze_cnt");
    ReadItem<int>(json, finish_remain_time, "display_state",
                  "finish_remain_time");
    ReadItem<bool>(json, enable_confirm_mode, "confirm_mode",
                   "enable_confirm_mode");
    ReadItem<int>(json, map_confirm_cancel_freeze_cnt, "confirm_mode",
                  "map_confirm_cancel_freeze_cnt");
    ReadItem<int>(json, model_confirm_cancel_freeze_cnt, "confirm_mode",
                  "model_confirm_cancel_freeze_cnt");
    ReadItem<bool>(json, enalbe_display_function, "display_state",
                   "enable_display_function");
    ReadItem<double>(json, int_vel_limit, "int_request", "int_vel_limit");
    ReadItem<double>(json, into_ramp_threshold, "display_state",
                     "into_ramp_threshold");
    ReadItem<double>(json, close_to_split_merge_threshold, "display_state",
                     "close_to_split_merge_threshold");
    ReadItem<double>(json, avoid_truck_time_distance_threshold, "display_state",
                     "avoid_truck_time_distance_threshold");
    ReadItem<double>(json, disallow_cancel_int_lc_lateral_thr, "int_request",
                     "disallow_cancel_int_lc_lateral_thr");
    ReadItem<bool>(json, enable_int_request_function, "int_request",
                   "enable_int_request_function");
    enable_hnp_function = read_json_key<bool>(json, "enable_hnp_functions");
    ReadItem<bool>(json, enable_speed_interal_surpress, "int_request",
                   "enable_speed_interal_surpress");
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
  bool enable_speed_interal_surpress = true;
};
struct GapSelectorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, default_lc_time, "gap_selector", "default_lc_time");
    ReadItem<double>(json, default_lh_time, "gap_selector", "default_lh_time");
    ReadItem<double>(json, collision_check_length_threshold, "gap_selector",
                     "collision_check_length_threshold");
    ReadItem<double>(json, lc_premove_time, "gap_selector", "lc_premove_time");
    ReadItem<double>(json, near_car_ttc, "gap_selector", "near_car_ttc");
    ReadItem<bool>(json, use_ego_v, "gap_selector", "use_ego_v");
    ReadItem<double>(json, lb_t_max, "gap_selector", "lb_t_max");
    ReadItem<double>(json, lb_t_min, "gap_selector", "lb_t_min");
    ReadItem<double>(json, lb_heading_error_max, "gap_selector",
                     "lb_heading_error_max");
    ReadItem<double>(json, lb_heading_error_min, "gap_selector",
                     "lb_heading_error_min");
    ReadItem<double>(json, lh_t_max, "gap_selector", "lh_t_max");
    ReadItem<double>(json, lh_t_min, "gap_selector", "lh_t_min");
    ReadItem<double>(json, lh_heading_error_max, "gap_selector",
                     "lh_heading_error_max");
    ReadItem<double>(json, lh_heading_error_min, "gap_selector",
                     "lh_heading_error_min");

    ReadItem<bool>(json, use_gs, "gap_selector", "use_gs");
    read_json_vec<double>(
        json, std::vector<std::string>{"gap_selector", "lat_ref_offset"},
        lat_ref_offset);
    ReadItem<double>(json, min_ego_v_cruise, "gap_selector",
                     "min_ego_v_cruise");
  }

  double default_lc_time = 6.0;
  double default_lh_time = 3.0;
  double collision_check_length_threshold = 2.2;
  double lc_premove_time = 1.5;
  double near_car_ttc = 0.2;
  bool use_ego_v = false;
  double lb_t_min = 1.5;
  double lb_t_max = 5.0;
  double lh_t_min = 1.5;
  double lh_t_max = 4.5;

  double lb_heading_error_min = 4.5;
  double lb_heading_error_max = 0.5;
  double lh_heading_error_min = 0.5;
  double lh_heading_error_max = 4.5;
  bool use_gs = true;
  std::vector<double> lat_ref_offset{0.0, 0.1, 0.2, 0.3};
  double min_ego_v_cruise = 2.0;
};

struct LateralObstacleDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    near_car_thr = read_json_key<double>(json, "near_car_thr", near_car_thr);
    lat_safety_buffer =
        read_json_key<double>(json, "lat_safety_buffer", lat_safety_buffer);
    oversize_veh_addition_buffer = read_json_key<double>(
        json, "oversize_veh_addition_buffer", oversize_veh_addition_buffer);
    traffic_cone_thr =
        read_json_key<double>(json, "traffic_cone_thr", traffic_cone_thr);
    avoid_persistence_front_buffer =
        read_json_key<double>(json, "avoid_persistence_front_buffer", avoid_persistence_front_buffer);
    large_static_obs_buffer =
        read_json_key<double>(json, "large_static_obs_buffer", large_static_obs_buffer);
    small_static_obs_buffer =
        read_json_key<double>(json, "small_static_obs_buffer", small_static_obs_buffer);
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
    static_buffer_for_search = read_json_key<double>(
        json, "static_buffer_for_search", static_buffer_for_search);
    column_static_buffer_for_search =
        read_json_key<double>(json, "column_static_buffer_for_search",
                              column_static_buffer_for_search);
    enable_hybrid_ara =
        read_json_key<bool>(json, "enable_hybrid_ara", enable_hybrid_ara);
    hybrid_ara_s_range =
        read_json_key<double>(json, "hybrid_ara_s_range", hybrid_ara_s_range);
    l_buffer_for_lat_decision = read_json_key<double>(
        json, "l_buffer_for_lat_decision", l_buffer_for_lat_decision);
    column_l_buffer_for_decision = read_json_key<double>(
        json, "column_l_buffer_for_decision", column_l_buffer_for_decision);
  }
  double near_car_thr = 0.3;
  double lat_safety_buffer = 0.7;
  double oversize_veh_addition_buffer = 0.15;
  double traffic_cone_thr = 0.15;
  double avoid_persistence_front_buffer = 0.15;
  double large_static_obs_buffer = 0.5;
  double small_static_obs_buffer = 0.5;
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
  double static_buffer_for_search = 2;
  double column_static_buffer_for_search = 2;
  bool enable_hybrid_ara = false;
  double hybrid_ara_s_range = 20;
  double l_buffer_for_lat_decision = 2;
  double column_l_buffer_for_decision = 2;
  double delta_t = 0.2;
  double num_step = 25;
};

struct HybridAraStarConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    x_grid_resolution = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "x_grid_resolution"});
    y_grid_resolution = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "y_grid_resolution"});
    phi_grid_resolution = read_json_keys<double>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "phi_grid_resolution"});
    next_node_num = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "next_node_num"});
    step_size = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "step_size"});
    one_shot_distance = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "one_shot_distance"});
    small_shot_distance = read_json_keys<double>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "small_shot_distance"});
    use_percentage_of_steering = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star",
                                       "use_percentage_of_steering"});
    heuristic_factor = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "heuristic_factor"});
    agent_cost_weight = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "agent_cost_weight"});
    center_cost_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "center_cost_weight"});
    motion_cost_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "motion_cost_weight"});
    boundary_cost_weight = read_json_keys<double>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "boundary_cost_weight"});
    boundary_soft_extra_buffer = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star",
                                       "boundary_soft_extra_buffer"});
    crosslinebuffer = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "crosslinebuffer"});
    enable_middle_final_node = read_json_keys<bool>(
        json, std::vector<std::string>{"hybrid_ara_star",
                                       "enable_middle_final_node"});
    l_limit = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "l_limit"});
    collision_buffer = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "collision_buffer"});
    rear_obs_s = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "rear_obs_s"});
    front_obs_s = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "front_obs_s"});
    enable_parking_space = read_json_keys<bool>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "enable_parking_space"});
    longitudinal_extend = read_json_keys<double>(
        json,
        std::vector<std::string>{"hybrid_ara_star", "longitudinal_extend"});
    lateral_extend = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "lateral_extend"});
    search_once = read_json_keys<bool>(
        json, std::vector<std::string>{"hybrid_ara_star", "search_once"});
    use_occ_s_dist = read_json_keys<double>(
        json, std::vector<std::string>{"hybrid_ara_star", "use_occ_s_dist"});
  }
  double x_grid_resolution = 0.3;
  double y_grid_resolution = 0.3;
  double phi_grid_resolution = 0.5;
  double next_node_num = 7;
  double step_size = 4;
  double one_shot_distance = 2;
  double small_shot_distance = 2;
  double use_percentage_of_steering = 1.0;
  double heuristic_factor = 15;
  double agent_cost_weight = 0.2;
  double center_cost_weight = 0.15;
  double motion_cost_weight = 0;
  double boundary_cost_weight = 0.1;
  double boundary_soft_extra_buffer = 0.6;
  double crosslinebuffer = 0.0;
  bool enable_middle_final_node = false;
  double l_limit = 3;
  double collision_buffer = 0.1;
  double rear_obs_s = 0.1;
  double front_obs_s = 0.1;
  bool enable_parking_space = false;
  double longitudinal_extend = 0.1;
  double lateral_extend = 0.1;
  bool search_once = true;
  double use_occ_s_dist = 7;
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
    nudge_lat_offset_threshold =
        read_json_key<double>(json, "nudge_lat_offset_threshold");
    nudge_value_way = read_json_key<bool>(json, "nudge_value_way");
    ReadItem<double>(json, avd_lon_distance_1, "lateral_offset_decider",
                     "avd_lon_distance_1");
    ReadItem<double>(json, avd_lon_distance_2, "lateral_offset_decider",
                     "avd_lon_distance_2");
    ReadItem<double>(json, avd_lon_distance_3, "lateral_offset_decider",
                     "avd_lon_distance_3");
    ReadItem<double>(json, avd_lon_distance_4, "lateral_offset_decider",
                     "avd_lon_distance_4");
    ReadItem<double>(json, avd_lon_distance_5, "lateral_offset_decider",
                     "avd_lon_distance_5");
    ReadItem<double>(json, avd_vrel_bp_1, "lateral_offset_decider",
                     "avd_vrel_bp_1");
    ReadItem<double>(json, avd_vrel_bp_2, "lateral_offset_decider",
                     "avd_vrel_bp_2");
    ReadItem<double>(json, avd_vrel_bp_3, "lateral_offset_decider",
                     "avd_vrel_bp_3");
    ReadItem<double>(json, avd_vrel_bp_4, "lateral_offset_decider",
                     "avd_vrel_bp_4");
    ReadItem<double>(json, avd_vrel_bp_5, "lateral_offset_decider",
                     "avd_vrel_bp_5");
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
  double nudge_lat_offset_threshold = 0.1;
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
    ReadItem<double>(json, hard_buffer2static_agent, "general_lateral_decider",
                     "hard_buffer2static_agent");
    ReadItem<double>(json, bound2center_line_distance_thr, "general_lateral_decider",
                     "bound2center_line_distance_thr");
    ReadItem<double>(json, soft_buffer2lane, "general_lateral_decider",
                     "soft_buffer2lane");
    ReadItem<double>(json, extra_soft_buffer2road, "general_lateral_decider",
                     "extra_soft_buffer2road");
    ReadItem<double>(json, hard_buffer2lane, "general_lateral_decider",
                     "hard_buffer2lane");
    ReadItem<double>(json, hard_buffer2road, "general_lateral_decider",
                     "hard_buffer2road");
    ReadItem<bool>(json, lateral_ref_traj_type, "general_lateral_decider",
                   "lateral_ref_traj_type");
    ReadItem<double>(json, care_dynamic_object_t_threshold,
                     "general_lateral_decider",
                     "care_dynamic_object_t_threshold");
    ReadItem<double>(json, care_static_object_t_threshold,
                     "general_lateral_decider",
                     "care_static_object_t_threshold");
    ReadItem<double>(json, trust_prediction_t_threshold,
                     "general_lateral_decider",
                     "trust_prediction_t_threshold");
    ReadItem<double>(json, soft_min_distance_road2center,
                     "general_lateral_decider",
                     "soft_min_distance_road2center");
    ReadItem<double>(json, hard_min_distance_road2center,
                     "general_lateral_decider",
                     "hard_min_distance_road2center");

    ReadItem<double>(json, care_lon_area_road_border, "general_lateral_decider",
                     "care_lon_area_road_border");

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
    lc_min_v_cruise = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "lc_min_v_cruise"},
        lc_min_v_cruise);
    lc_second_dist_thr = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "lc_second_dist_thr"},
        lc_second_dist_thr);
    lc_finished_second_dist_thr = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "lc_finished_second_dist_thr"},
        lc_finished_second_dist_thr);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "dynamic_lc_ref_buffer"},
                          dynamic_lc_ref_buffer);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "dynamic_lc_finished_ref_buffer"},
                          dynamic_lc_finished_ref_buffer);
    ReadItem<bool>(json, not_use_gap_flag, "general_lateral_decider",
                   "not_use_gap_flag");

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
    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_buffer_for_lane_width_bp"},
        extra_buffer_for_lane_width_bp);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "extra_lane_width_buffer"},
                          extra_lane_width_buffer);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_road_decrease_buffer_for_lane_width_bp"},
        extra_road_decrease_buffer_for_lane_width_bp);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "extra_road_decrease_buffer"},
                          extra_road_decrease_buffer);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "lateral_road_boader_v_bp"},
                          lateral_road_boader_v_bp);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "extra_lateral_buffer"},
                          extra_lateral_buffer);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "obstacle_pred_ts_bp"},
                          obstacle_pred_ts_bp);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "obstacle_pred_decrease_buffer"},
                          obstacle_pred_decrease_buffer);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "lateral_obstacle_nudge_buffer_v_bp"},
                          lateral_obstacle_nudge_buffer_v_bp);
    read_json_vec<double>(json,
                          std::vector<std::string>{"general_lateral_decider",
                                                   "lateral_nudge_buffer"},
                          lateral_nudge_buffer);

    ReadItem<double>(json, nudge_extra_buffer_in_intersection,
                     "general_lateral_decider",
                     "nudge_extra_buffer_in_intersection");

    ReadItem<double>(json, map_bound_weight[BoundType::AGENT],
                     "general_lateral_decider", "bound_static_agent_weight");
    ReadItem<double>(json, map_bound_weight[BoundType::DYNAMIC_AGENT],
                     "general_lateral_decider", "bound_dynamic_agent_weight");
    ReadItem<double>(json, map_bound_weight[BoundType::ADJACENT_AGENT],
                     "general_lateral_decider", "bound_adjacent_agent_weight");
    ReadItem<double>(json, map_bound_weight[BoundType::ROAD_BORDER],
                     "general_lateral_decider", "bound_road_border_weight");
    ReadItem<double>(json, map_bound_weight[BoundType::REAR_AGENT],
                     "general_lateral_decider", "bound_rear_agent_weight");
    ReadItem<double>(json, map_bound_weight[BoundType::LOW_PRIORITY_AGENT],
                     "general_lateral_decider", "bound_low_priority_agent_weight");
    ReadItem<double>(json, nudge_extra_decrease_buffer_in_lane_change_scene,
                     "general_lateral_decider",
                     "nudge_extra_decrease_buffer_in_lane_change_scene");

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
        std::vector<std::string>{"general_lateral_decider",
                                 "side_obstacle_relative_position_bp"},
        _side_obstacle_relative_position_bp);
    read_json_vec<double>(
        json,
        std::vector<std::string>{
            "general_lateral_decider",
            "side_obstacle_relative_position_decrease_extra_buffer"},
        _side_obstacle_relative_position_decrease_extra_buffer);

    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider", "relative_v_bp"},
        _relative_v_bp);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "relative_v_decrease_extra_buffer"},
        _relative_v_decrease_extra_buffer);

    ReadItem<double>(json, truck_decrease_extra_buffer,
                     "general_lateral_decider", "truck_decrease_extra_buffer");
    ReadItem<double>(json, lane_borrow_extra_front_lon_buffer,
                     "general_lateral_decider",
                     "lane_borrow_extra_front_lon_buffer");
    ReadItem<double>(json, lane_borrow_extra_rear_lon_buffer,
                     "general_lateral_decider",
                     "lane_borrow_extra_rear_lon_buffer");
    ReadItem<double>(json, extra_lane_type_decrease_buffer,
                     "general_lateral_decider",
                     "extra_lane_type_decrease_buffer");
    ReadItem<double>(json, side_obstacle_lat_buffer_limit,
                     "general_lateral_decider",
                     "side_obstacle_lat_buffer_limit");
    ReadItem<double>(json, static_nudge_buffer2lane_boundary,
                     "general_lateral_decider",
                     "static_nudge_buffer2lane_boundary");
    ReadItem<int>(json, obstacle_predlonoverlap_up_total_count,
                     "general_lateral_decider",
                     "obstacle_predlonoverlap_up_total_count");
    ReadItem<int>(json, obstacle_predlonoverlap_down_total_count,
                     "general_lateral_decider",
                     "obstacle_predlonoverlap_down_total_count");
    ReadItem<double>(json, nudge_extra_buffer_reverse_obstacle,
                     "general_lateral_decider",
                     "nudge_extra_buffer_reverse_obstacle");
    ReadItem<double>(json, extra_road_buffer_in_big_curvature,
                     "general_lateral_decider",
                     "extra_road_buffer_in_big_curvature");
    ReadItem<double>(json, hard_buffer2dynamic_agent,
                     "general_lateral_decider",
                     "hard_buffer2dynamic_agent");
    /* read config from json */
  }
  double hard_buffer2dynamic_agent = 0.15;
  double desired_vel = 11.11;                    // KPH_40;
  double l_care_width = 15.;                     // TBD: more beautiful
  double care_obj_lat_distance_threshold = 30.;  // TBD: more beautiful
  double care_obj_lon_distance_threshold = 60.;  // TBD: more beautiful
  double hard_buffer2static_agent = 0.15;
  double bound2center_line_distance_thr = 0.1;
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
  double trust_prediction_t_threshold = 2.5;
  double care_area_s_len = 5.0;
  double max_ref_curvature = 0.5;
  bool lateral_ref_traj_type = false;
  double max_lateral_ttc = 5.0;
  double care_lon_area_road_border = 100;
  double ramp_limit_v = 19.44;
  bool ramp_limit_v_valid = false;
  double lc_second_dist_thr = 1.5;
  double lc_finished_second_dist_thr = 1.5;
  std::vector<double> dynamic_lc_ref_buffer{0.0, 0.1, 0.2, 0.3};
  std::vector<double> dynamic_lc_finished_ref_buffer{0.0, 0.05, 0.1, 0.2};
  bool not_use_gap_flag = true;
  double min_v_cruise = 5.0;
  double lc_min_v_cruise = 5.0;

  std::vector<double> lateral_road_boader_collision_ttc_bp{0, 1.5, 3, 4.5, 5};
  std::vector<double> extra_collision_lateral_buffer{0.1, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> extra_buffer_for_lane_width_bp{2.8, 3.1, 3.4, 3.7, 4.2};
  std::vector<double> extra_lane_width_buffer{0.2, 0.15, 0.1, 0.05, 0.0};
  std::vector<double> extra_road_decrease_buffer_for_lane_width_bp{2.8, 3.1, 3.4, 3.7, 4.2};
  std::vector<double> extra_road_decrease_buffer{0.2, 0.15, 0.1, 0.05, 0.0};

  std::vector<double> lateral_road_boader_v_bp{10, 40, 60, 100, 130};
  std::vector<double> extra_lateral_buffer{0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> obstacle_pred_ts_bp{0, 1, 2, 3, 4, 5};
  std::vector<double> obstacle_pred_decrease_buffer{0, 0.05, 0.1, 0.15, 0.2, 0.25};
  std::vector<double> lateral_obstacle_nudge_buffer_v_bp{10, 40, 60, 80, 100, 130};
  std::vector<double> lateral_nudge_buffer{0.04, 0.16, 0.25, 0.33, 0.41, 0.54};
  std::unordered_map<BoundType, double> map_bound_weight{
      {BoundType::AGENT, 0.1},
      {BoundType::DYNAMIC_AGENT, 0.1},
      {BoundType::ADJACENT_AGENT, 0.1},
      {BoundType::ROAD_BORDER, 0.1},
      {BoundType::REAR_AGENT, 0.1},
      {BoundType::LOW_PRIORITY_AGENT, 0.06}};
  double nudge_extra_buffer_in_intersection = 0.1;
  double nudge_extra_decrease_buffer_in_lane_change_scene = 0.1;

  std::vector<double> _relative_positon_bp = {0, 1, 2, 3, 4, 5};
  std::vector<double> _relative_positon_decrease_extra_buffer = {0,   0.1, 0.2,
                                                                 0.3, 0.4, 0.5};
  std::vector<double> _relative_v_bp = {0, 1, 2, 3, 4, 5};
  std::vector<double> _relative_v_decrease_extra_buffer = {0,   0.02, 0.05,
                                                           0.1, 0.15, 0.23};
  std::vector<double> _side_obstacle_relative_position_bp = {1, 2, 3, 4, 5};
  std::vector<double> _side_obstacle_relative_position_decrease_extra_buffer = {
      0.1, 0.2, 0.3, 0.4, 0.5};
  double extra_lane_type_decrease_buffer = 0.05;
  double truck_decrease_extra_buffer = 0.05;
  double lane_borrow_extra_front_lon_buffer = 1.0;
  double lane_borrow_extra_rear_lon_buffer = 2.0;
  double side_obstacle_lat_buffer_limit = 0.5;
  double static_nudge_buffer2lane_boundary = 0.1;
  int obstacle_predlonoverlap_up_total_count = 0;
  int obstacle_predlonoverlap_down_total_count = 0;
  double nudge_extra_buffer_reverse_obstacle = 0.1;
  double extra_road_buffer_in_big_curvature = 0.0;
};

struct HppGeneralLateralDeciderConfig : public EgoPlanningConfig {
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
    extra_soft_buffer2groundline = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_soft_buffer2groundline"},
        extra_soft_buffer2groundline);
    extra_hard_buffer2groundline = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "extra_hard_buffer2groundline"},
        extra_hard_buffer2groundline);
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
    ref_curvature_factor = read_json_keys<double>(
        json,
        std::vector<std::string>{"general_lateral_decider",
                                 "ref_curvature_factor"},
        ref_curvature_factor);
    enable_ara_ref =
        read_json_key<bool>(json, "enable_ara_ref", enable_ara_ref);
    enable_last_lat_path =
        read_json_key<bool>(json, "enable_last_lat_path", enable_last_lat_path);
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
  double extra_soft_buffer2groundline = 0.35;
  double extra_hard_buffer2groundline = 0.2;
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
  bool enable_ara_ref = false;
  double ref_length_thr = 0.01;
  double ref_curvature_factor = 0.0;
  bool enable_last_lat_path = false;
};

struct HppParkingSwitchConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    dist_to_parking_space_thr = read_json_key<double>(
        json, "dist_to_parking_space_thr", dist_to_parking_space_thr);
  }
  double dist_to_parking_space_thr = 2.0;
};

struct LateralMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<bool>(json, pass_acc_mode, "lat_motion_ilqr",
                   "pass_acc_mode");
    ReadItem<bool>(json, warm_start_enable, "lat_motion_ilqr",
                   "warm_start_enable");
    ReadItem<bool>(json, use_index_clip, "lat_motion_ilqr",
                   "use_index_clip");
    ReadItem<bool>(json, use_acc_compensation, "lat_motion_ilqr",
                   "use_acc_compensation");
    ReadItem<double>(json, delta_t, "lat_motion_ilqr", "delta_t");
    ReadItem<double>(json, min_v_cruise, "lat_motion_ilqr", "min_v_cruise");
    ReadItem<double>(json, min_ego_vel, "lat_motion_ilqr", "min_ego_vel");
    ReadItem<double>(json, acc_bound, "lat_motion_ilqr", "acc_bound");
    ReadItem<double>(json, jerk_bound, "lat_motion_ilqr", "jerk_bound");
    ReadItem<double>(json, jerk_bound_spatio, "lat_motion_ilqr", "jerk_bound_spatio");
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_jerk_bound"},
        map_jerk_bound, map_jerk_bound);
    ReadItem<double>(json, jerk_bound_inactivated_limit, "lat_motion_ilqr",
                     "jerk_bound_inactivated_limit");
    ReadItem<double>(json, jerk_bound_avoid, "lat_motion_ilqr",
                     "jerk_bound_avoid");
    ReadItem<double>(json, acc_bound_lane_change, "lat_motion_ilqr",
                     "acc_bound_lane_change");
    ReadItem<double>(json, jerk_bound_lane_change, "lat_motion_ilqr",
                     "jerk_bound_lane_change");
    ReadItem<double>(json, q_ref_x, "lat_motion_ilqr", "q_ref_x");
    ReadItem<double>(json, q_ref_y, "lat_motion_ilqr", "q_ref_y");
    ReadItem<double>(json, q_ref_theta, "lat_motion_ilqr", "q_ref_theta");
    ReadItem<double>(json, q_continuity, "lat_motion_ilqr", "q_continuity");
    ReadItem<double>(json, q_continuity_low_speed, "lat_motion_ilqr",
                     "q_continuity_low_speed");
    ReadItem<double>(json, q_continuity_search, "lat_motion_ilqr",
                     "q_continuity_search");
    ReadItem<double>(json, q_continuity_lane_change, "lat_motion_ilqr",
                     "q_continuity_lane_change");
    ReadItem<double>(json, q_acc, "lat_motion_ilqr", "q_acc");
    ReadItem<double>(json, q_acc_spatio, "lat_motion_ilqr", "q_acc_spatio");
    ReadItem<double>(json, q_jerk, "lat_motion_ilqr", "q_jerk");
    ReadItem<double>(json, q_acc_bound, "lat_motion_ilqr", "q_acc_bound");
    ReadItem<double>(json, q_jerk_bound, "lat_motion_ilqr", "q_jerk_bound");
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qsoft_bound"},
        map_qsoft_bound, map_qsoft_bound);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qhard_bound"},
        map_qhard_bound, map_qhard_bound);
    ReadItem<double>(json, emergence_avoid_factor, "lat_motion_ilqr",
                     "emergence_avoid_factor");
    ReadItem<double>(json, intersection_avoid_factor, "lat_motion_ilqr",
                     "intersection_avoid_factor");
    ReadItem<double>(json, jerk_bound_intersection, "lat_motion_ilqr",
                     "jerk_bound_intersection");
    ReadItem<double>(json, q_ref_xy_intersection, "lat_motion_ilqr",
                     "q_ref_xy_intersection");
    ReadItem<double>(json, q_ref_theta_intersection, "lat_motion_ilqr",
                     "q_ref_theta_intersection");
    ReadItem<double>(json, q_continuity_intersection, "lat_motion_ilqr",
                     "q_continuity_intersection");
    ReadItem<double>(json, q_acc_intersection, "lat_motion_ilqr",
                     "q_acc_intersection");
    ReadItem<double>(json, q_jerk_intersection_mid, "lat_motion_ilqr",
                     "q_jerk_intersection_mid");
    ReadItem<double>(json, q_jerk_intersection_close, "lat_motion_ilqr",
                     "q_jerk_intersection_close");
    ReadItem<double>(json, q_jerk_bound_intersection, "lat_motion_ilqr",
                     "q_jerk_bound_intersection");
    ReadItem<double>(json, avoid_back_time, "lat_motion_ilqr",
                     "avoid_back_time");
    ReadItem<double>(json, q_ref_x_avoid, "lat_motion_ilqr", "q_ref_x_avoid");
    ReadItem<double>(json, q_ref_y_avoid, "lat_motion_ilqr", "q_ref_y_avoid");
    ReadItem<double>(json, q_ref_theta_avoid, "lat_motion_ilqr",
                     "q_ref_theta_avoid");
    ReadItem<double>(json, q_acc_avoid, "lat_motion_ilqr", "q_acc_avoid");
    ReadItem<double>(json, q_jerk_avoid, "lat_motion_ilqr", "q_jerk_avoid");
    ReadItem<double>(json, avoid_high_vel, "lat_motion_ilqr", "avoid_high_vel");
    ReadItem<double>(json, q_ref_theta_avoid_high_vel, "lat_motion_ilqr",
                     "q_ref_theta_avoid_high_vel");
    ReadItem<double>(json, q_jerk_avoid_high_vel, "lat_motion_ilqr",
                     "q_jerk_avoid_high_vel");
    ReadItem<double>(json, q_jerk_bound_avoid_high_vel, "lat_motion_ilqr",
                     "q_jerk_bound_avoid_high_vel");
    ReadItem<double>(json, q_ref_x_static_avoid, "lat_motion_ilqr",
                     "q_ref_x_static_avoid");
    ReadItem<double>(json, q_ref_y_static_avoid, "lat_motion_ilqr",
                     "q_ref_y_static_avoid");
    ReadItem<double>(json, q_ref_theta_static_avoid, "lat_motion_ilqr",
                     "q_ref_theta_static_avoid");
    ReadItem<double>(json, q_acc_static_avoid, "lat_motion_ilqr",
                     "q_acc_static_avoid");
    ReadItem<double>(json, q_jerk_static_avoid_close, "lat_motion_ilqr",
                     "q_jerk_static_avoid_close");
    ReadItem<double>(json, q_jerk_static_avoid_middle, "lat_motion_ilqr",
                     "q_jerk_static_avoid_middle");
    ReadItem<bool>(json, use_new_lc_param, "lat_motion_ilqr",
                   "use_new_lc_param");
    ReadItem<double>(json, lane_change_high_vel, "lat_motion_ilqr",
                     "lane_change_high_vel");
    ReadItem<double>(json, jerk_bound_lane_change_high_vel, "lat_motion_ilqr",
                     "jerk_bound_lane_change_high_vel");
    ReadItem<double>(json, q_jerk_bound_lane_change, "lat_motion_ilqr",
                     "q_jerk_bound_lane_change");
    ReadItem<double>(json, q_jerk_bound_lane_change_high_vel, "lat_motion_ilqr",
                     "q_jerk_bound_lane_change_high_vel");
    ReadItem<double>(json, q_ref_xy_lane_change_high_vel, "lat_motion_ilqr",
                     "q_ref_xy_lane_change_high_vel");
    ReadItem<double>(json, q_ref_theta_lane_change_high_vel1, "lat_motion_ilqr",
                     "q_ref_theta_lane_change_high_vel1");
    ReadItem<double>(json, q_ref_theta_lane_change_high_vel2, "lat_motion_ilqr",
                     "q_ref_theta_lane_change_high_vel2");
    ReadItem<double>(json, q_ref_theta_lane_change_high_vel3, "lat_motion_ilqr",
                     "q_ref_theta_lane_change_high_vel3");
    ReadItem<double>(json, q_ref_theta_lane_change_high_vel, "lat_motion_ilqr",
                     "q_ref_theta_lane_change_high_vel");
    ReadItem<double>(json, q_jerk_lane_change_high_vel, "lat_motion_ilqr",
                     "q_jerk_lane_change_high_vel");
    read_json_vec<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "map_qrefxy_lc_high_vel"},
        map_qrefxy_lc_high_vel, map_qrefxy_lc_high_vel);
    read_json_vec<double>(
        json,
        std::vector<std::string>{"lat_motion_ilqr", "map_qjerk_lc_high_vel"},
        map_qjerk_lc_high_vel, map_qjerk_lc_high_vel);
    read_json_vec<double>(json,
                          std::vector<std::string>{"lat_motion_ilqr",
                                                   "map_qjerk_lc_high_vel_old"},
                          map_qjerk_lc_high_vel_old);
    ReadItem<double>(json, q_ref_x_lane_change, "lat_motion_ilqr",
                     "q_ref_x_lane_change");
    ReadItem<double>(json, q_ref_y_lane_change, "lat_motion_ilqr",
                     "q_ref_y_lane_change");
    ReadItem<double>(json, q_ref_theta_lane_change, "lat_motion_ilqr",
                     "q_ref_theta_lane_change");
    ReadItem<double>(json, q_acc_lane_change, "lat_motion_ilqr",
                     "q_acc_lane_change");
    ReadItem<double>(json, q_jerk_lane_change, "lat_motion_ilqr",
                     "q_jerk_lane_change");
    ReadItem<double>(json, q_jerk_lane_change2, "lat_motion_ilqr",
                     "q_jerk_lane_change2");
    ReadItem<double>(json, lane_change_ego_l_thr, "lat_motion_ilqr",
                     "lane_change_ego_l_thr");
    ReadItem<double>(json, q_ref_xy_lane_change_back, "lat_motion_ilqr",
                     "q_ref_xy_lane_change_back");
    ReadItem<double>(json, q_ref_theta_lane_change_back, "lat_motion_ilqr",
                     "q_ref_theta_lane_change_back");
    ReadItem<double>(json, q_jerk_lane_change_back, "lat_motion_ilqr",
                     "q_jerk_lane_change_back");
    ReadItem<double>(json, jerk_bound_lane_change_back, "lat_motion_ilqr",
                     "jerk_bound_lane_change_back");
    ReadItem<double>(json, q_jerk_bound_lane_change_back, "lat_motion_ilqr",
                     "q_jerk_bound_lane_change_back");
    ReadItem<double>(json, road_curvature_radius, "lat_motion_ilqr",
                     "road_curvature_radius");
    ReadItem<size_t>(json, curvature_change_index, "lat_motion_ilqr",
                     "curvature_change_index");
    ReadItem<double>(json, curvature_preview_distance, "lat_motion_ilqr",
                     "curvature_preview_distance");
    ReadItem<double>(json, curvature_preview_length, "lat_motion_ilqr",
                     "curvature_preview_length");
    ReadItem<double>(json, curvature_preview_step, "lat_motion_ilqr",
                     "curvature_preview_step");
    ReadItem<size_t>(json, motion_plan_concerned_start_index, "lat_motion_ilqr",
                     "motion_plan_concerned_start_index");
    ReadItem<size_t>(json, motion_plan_concerned_end_index, "lat_motion_ilqr",
                     "motion_plan_concerned_end_index");
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "valid_perception_range"}, valid_perception_range);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qxy"},
        map_qxy, map_qxy);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qtheta"},
        map_qtheta, map_qtheta);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qjerk1"},
        map_qjerk1, map_qjerk1);
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_qjerk2"},
        map_qjerk2, map_qjerk2);
    ReadItem<double>(json, end_ratio_for_qrefxy, "lat_motion_ilqr",
                     "end_ratio_for_qrefxy");
    ReadItem<double>(json, end_ratio_for_qreftheta, "lat_motion_ilqr",
                     "end_ratio_for_qreftheta");
    ReadItem<double>(json, end_ratio_for_qjerk, "lat_motion_ilqr",
                     "end_ratio_for_qjerk");
    ReadItem<double>(json, lc_end_ratio_for_first_qrefxy, "lat_motion_ilqr",
                     "lc_end_ratio_for_first_qrefxy");
    ReadItem<double>(json, lc_end_ratio_for_second_qrefxy, "lat_motion_ilqr",
                     "lc_end_ratio_for_second_qrefxy");
    ReadItem<double>(json, lc_end_ratio_for_qreftheta, "lat_motion_ilqr",
                     "lc_end_ratio_for_qreftheta");
    ReadItem<double>(json, enter_ramp_on_road_time, "lat_motion_ilqr",
                     "enter_ramp_on_road_time");
    ReadItem<double>(json, q_ref_xy_split, "lat_motion_ilqr", "q_ref_xy_split");
    ReadItem<double>(json, q_ref_theta_split, "lat_motion_ilqr",
                     "q_ref_theta_split");
    ReadItem<double>(json, q_jerk_split, "lat_motion_ilqr", "q_jerk_split");
    ReadItem<double>(json, q_jerk_bound_split, "lat_motion_ilqr",
                     "q_jerk_bound_split");
    ReadItem<double>(json, jerk_bound_split, "lat_motion_ilqr",
                     "jerk_bound_split");
    ReadItem<bool>(json, ramp_valid, "lat_motion_ilqr", "ramp_valid");
    ReadItem<double>(json, acc_bound_ramp, "lat_motion_ilqr", "acc_bound_ramp");
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "map_jerk_bound_ramp"},
        map_jerk_bound_ramp, map_jerk_bound_ramp);
    ReadItem<double>(json, q_ref_x_ramp, "lat_motion_ilqr", "q_ref_x_ramp");
    ReadItem<double>(json, q_ref_y_ramp, "lat_motion_ilqr", "q_ref_y_ramp");
    ReadItem<double>(json, q_ref_theta_ramp, "lat_motion_ilqr",
                     "q_ref_theta_ramp");
    ReadItem<double>(json, q_acc_ramp, "lat_motion_ilqr", "q_acc_ramp");
    ReadItem<double>(json, q_jerk_ramp_close, "lat_motion_ilqr",
                     "q_jerk_ramp_close");
    ReadItem<double>(json, q_jerk_ramp_mid, "lat_motion_ilqr",
                     "q_jerk_ramp_mid");
    ReadItem<double>(json, big_theta_thr, "lat_motion_ilqr", "big_theta_thr");
    read_json_vec<double>(
        json, std::vector<std::string>{"lat_motion_ilqr", "q_jerk_for_big_theta"},
        q_jerk_for_big_theta);
    ReadItem<double>(json, path_backward_appended_length, "lat_motion_ilqr",
                     "path_backward_appended_length");
    ReadItem<double>(json, max_steer_angle_dot, "lat_motion_ilqr",
                     "max_steer_angle_dot");
    ReadItem<double>(json, max_steer_angle_dot_low_speed, "lat_motion_ilqr",
                     "max_steer_angle_dot_low_speed");
  }

  bool pass_acc_mode = false;
  bool warm_start_enable = true;
  bool use_index_clip = true;
  bool use_acc_compensation = true;
  double delta_t = 0.2;
  double min_ego_vel = 5.0;
  double min_v_cruise = 2.0;
  double max_steer_angle_dot = 200.0;
  double max_steer_angle_dot_low_speed = 10.0;

  double acc_bound = 1.5;
  double jerk_bound = 1.0;
  double jerk_bound_spatio = 1.0;
  std::vector<double> map_jerk_bound{0.4, 0.35, 0.3, 0.25};
  double jerk_bound_inactivated_limit = 1.0;
  double jerk_bound_avoid = 0.5;
  double acc_bound_lane_change = 3.0;
  double jerk_bound_lane_change = 5.0;
  double q_acc_bound = 20000.0;
  double q_jerk_bound = 500000.0;

  std::vector<double> map_qsoft_bound{200, 500, 1500.0, 3000.0, 4000.0, 5000.0};
  std::vector<double> map_qhard_bound{500, 1000, 3000.0, 5000.0, 6000.0, 7000.0};
  double emergence_avoid_factor = 2.0;
  double intersection_avoid_factor = 2.0;

  double q_ref_x = 20.0;
  double q_ref_y = 20.0;
  double q_ref_theta = 15.0;
  double q_continuity = 0.;
  double q_continuity_low_speed = 0.5;
  double q_continuity_search = 0.5;
  double q_continuity_lane_change = 0.;
  double q_acc = 0.5;
  double q_acc_spatio = 0.5;
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
  double q_jerk_avoid_high_vel = 2.0;
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
  std::vector<double> map_jerk_bound_ramp{1.0, 0.8, 0.6, 0.5};
  double q_ref_x_ramp = 400.0;
  double q_ref_y_ramp = 400.0;
  double q_ref_theta_ramp = 5000.0;
  double q_acc_ramp = 0.02;
  double q_jerk_ramp_close = 45.0;
  double q_jerk_ramp_mid = 10.0;

  double road_curvature_radius = 750.0;
  size_t curvature_change_index = 15;
  double curvature_preview_distance = 50.0;
  double curvature_preview_length = 20.0;
  double curvature_preview_step = 1.0;
  size_t motion_plan_concerned_start_index = 2;
  size_t motion_plan_concerned_end_index = 20;
  std::vector<double> valid_perception_range{15.0, 30.0, 50.0, 70.0, 90.0};
  std::vector<double> map_qxy{80.0, 80.0, 400.0, 500.0};
  std::vector<double> map_qtheta{3000.0, 5000.0, 7000.0, 10000.0};
  std::vector<double> map_qjerk1{45.0, 10.0, 100.0, 300.0, 600.0};
  std::vector<double> map_qjerk2{10.0, 5.0, 50.0, 150.0, 400.0};
  double end_ratio_for_qrefxy = 1.0;
  double end_ratio_for_qreftheta = 1.0;
  double end_ratio_for_qjerk = 1.0;
  double lc_end_ratio_for_first_qrefxy = 1.0;
  double lc_end_ratio_for_second_qrefxy = 1.0;
  double lc_end_ratio_for_qreftheta = 1.0;
  double big_theta_thr = 1.0;
  std::vector<double> q_jerk_for_big_theta{20.0, 5.0};
  double path_backward_appended_length = 2.5;
};

struct RealtimeLateralMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<bool>(json, warm_start_enable, "lat_motion_ilqr",
                   "warm_start_enable");
    ReadItem<double>(json, acc_bound, "lat_motion_ilqr", "acc_bound");
    ReadItem<double>(json, jerk_bound, "lat_motion_ilqr", "jerk_bound");
    ReadItem<double>(json, q_ref_x, "lat_motion_ilqr", "q_ref_x");
    ReadItem<double>(json, q_ref_y, "lat_motion_ilqr", "q_ref_y");
    ReadItem<double>(json, q_ref_theta, "lat_motion_ilqr", "q_ref_theta");
    ReadItem<double>(json, q_continuity, "lat_motion_ilqr", "q_continuity");
    ReadItem<double>(json, q_acc, "lat_motion_ilqr", "q_acc");
    ReadItem<double>(json, q_jerk, "lat_motion_ilqr", "q_jerk");
    ReadItem<double>(json, q_acc_bound, "lat_motion_ilqr", "q_acc_bound");
    ReadItem<double>(json, q_jerk_bound, "lat_motion_ilqr", "q_jerk_bound");
    ReadItem<double>(json, q_soft_corridor, "lat_motion_ilqr",
                     "q_soft_corridor");
    ReadItem<double>(json, q_hard_corridor, "lat_motion_ilqr",
                     "q_hard_corridor");
    ReadItem<double>(json, delta_t, "lat_motion_ilqr", "delta_t");
    ReadItem<size_t>(json, motion_plan_concerned_index, "lat_motion_ilqr",
                     "motion_plan_concerned_index");
  }
  bool warm_start_enable = true;
  double acc_bound = 6.0;
  double jerk_bound = 3.5;

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
    ReadItem<double>(json, q_ref_pos, "long_motion_ilqr", "q_ref_pos");
    ReadItem<double>(json, q_ref_vel, "long_motion_ilqr", "q_ref_vel");
    ReadItem<double>(json, q_acc, "long_motion_ilqr", "q_acc");
    ReadItem<double>(json, q_jerk, "long_motion_ilqr", "q_jerk");
    ReadItem<double>(json, q_soft_pos_bound, "long_motion_ilqr",
                     "q_soft_pos_bound");
    ReadItem<double>(json, q_hard_pos_bound, "long_motion_ilqr",
                     "q_hard_pos_bound");
    ReadItem<double>(json, q_sv_bound, "long_motion_ilqr", "q_sv_bound");
    ReadItem<double>(json, q_vel_bound, "long_motion_ilqr", "q_vel_bound");
    ReadItem<double>(json, q_acc_bound, "long_motion_ilqr", "q_acc_bound");
    ReadItem<double>(json, q_jerk_bound, "long_motion_ilqr", "q_jerk_bound");
    ReadItem<double>(json, q_stop_s, "long_motion_ilqr", "q_stop_s");
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
    hpp_collision_threshold =
        read_json_key<double>(json, "hpp_collision_threshold");
    lon_care_length = read_json_key<double>(json, "lon_care_length");
    lon_max_ignore_relative_time =
        read_json_key<double>(json, "lon_max_ignore_relative_time");
    rads_stop_distance_to_destination =
        read_json_key<double>(json, "rads_stop_distance_to_destination");
    narrow_space_width_stop_thrshld =
        read_json_key<double>(json, "narrow_space_width_stop_thrshld");
    narrow_space_distance_stop_thrshld =
        read_json_key<double>(json, "narrow_space_distance_stop_thrshld");
    narrow_v_limit_attention =
        read_json_key<double>(json, "narrow_v_limit_attention");
    narrow_v_limit_warn = read_json_key<double>(json, "narrow_v_limit_warn");
    narrow_v_limit_danger =
        read_json_key<double>(json, "narrow_v_limit_danger");
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
  double hpp_collision_threshold = 0.1;
  double narrow_space_width_stop_thrshld = -2.0;
  double narrow_space_distance_stop_thrshld = 10.0;
  double lon_care_length = 120;
  double lon_max_ignore_relative_time = 2.0;
  double rads_stop_distance_to_destination = 0;
  double max_deceleration = -6.0;
  double narrow_v_limit_attention = 2.5;  // 9kph
  double narrow_v_limit_warn = 1.67;      // 6kph
  double narrow_v_limit_danger = 0.83;    // 3kph
};

struct AdaptiveCruiseControlConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, kMaxCentrifugalAcceleration, "acc_control",
                     "kMaxCentrifugalAcceleration");
    ReadItem<bool>(json, enable_navi_speed_control, "acc_control",
                   "enable_navi_speed_control");
    ReadItem<bool>(json, enable_navi_time_distance_control, "acc_control",
                   "enable_navi_time_distance_control");
    ReadItem<double>(json, v_ratio_threshold, "acc_control",
                     "v_ratio_threshold");
    ReadItem<double>(json, t_actuator_delay, "acc_control", "t_actuator_delay");
    ReadItem<double>(json, gain_tg, "acc_control", "gain_tg");
    ReadItem<double>(json, dv_down, "acc_control", "dv_down");
    ReadItem<double>(json, gain_s, "acc_control", "gain_s");
    ReadItem<double>(json, gain_v, "acc_control", "gain_v");
    ReadItem<double>(json, v_max_able, "acc_control", "v_max_able");
    ReadItem<double>(json, v_min_able, "acc_control", "v_min_able");
    ReadItem<double>(json, obstacle_max_brake, "acc_control",
                     "obstacle_max_brake");
    ReadItem<double>(json, kMinCurvature, "acc_control", "kMinCurvature");
    ReadItem<double>(json, s_rel_threshold, "acc_control", "s_rel_threshold");
    enable_hnp_functions = read_json_key<bool>(json, "enable_hnp_functions");
    ReadItem<double>(json, jerk_set_accel, "acc_control", "jerk_set_accel");
    ReadItem<double>(json, jerk_set_brake, "acc_control", "jerk_set_brake");
    ReadItem<double>(json, dx_ref_weight, "acc_control", "dx_ref_weight");
    ReadItem<size_t>(json, acc_jerk_control_num, "acc_control",
                     "acc_jerk_control_num");
    ReadItem<double>(json, acc_set_a, "acc_control", "acc_set_a");
    ReadItem<double>(json, s_set_a, "acc_control", "s_set_a");
    ReadItem<double>(json, time_distance_aggresive, "acc_control",
                     "time_distance_aggresive");
    ReadItem<double>(json, time_distance_moderate_aggresive, "acc_control",
                     "time_distance_moderate_aggresive");
    ReadItem<double>(json, time_distance_normal, "acc_control",
                     "time_distance_normal");
    ReadItem<double>(json, time_distance_moderate_conservative, "acc_control",
                     "time_distance_moderate_conservative");
    ReadItem<double>(json, time_distance_conservcative, "acc_control",
                     "time_distance_conservcative");
    ReadItem<double>(json, max_curvatrue_fixed, "acc_control",
                     "max_curvatrue_fixed");
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
    ReadItem<bool>(json, enable_start_stop_function, "start_stop",
                   "enable_start_stop_function");
    enable_hnp_functions = read_json_key<bool>(json, "enable_hnp_functions");
    ReadItem<double>(json, ego_stop_v, "start_stop", "ego_stop_v");
    ReadItem<double>(json, leadone_s_lower, "start_stop", "leadone_s_lower");
    ReadItem<double>(json, leadone_s_upper, "start_stop", "leadone_s_upper");
    ReadItem<double>(json, leadone_v_threshold, "start_stop",
                     "leadone_v_threshold");
    ReadItem<double>(json, dx_ref_weight, "start_stop", "dx_ref_weight");
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
    ReadItem<bool>(json, enable_mrc_condition, "mrc_condition",
                   "enable_mrc_condition");
    ReadItem<double>(json, fcw_threshold, "mrc_condition", "fcw_threshold");
    ReadItem<double>(json, a_brake_slow, "mrc_condition", "a_brake_slow");
    ReadItem<double>(json, a_brake_hard, "mrc_condition", "a_brake_hard");
    ReadItem<double>(json, a_brake_emergency, "mrc_condition",
                     "a_brake_emergency");
    ReadItem<double>(json, jerk_brake_slow, "mrc_condition", "jerk_brake_slow");
    ReadItem<double>(json, jerk_brake_hard, "mrc_condition", "jerk_brake_hard");
    ReadItem<double>(json, jerk_brake_emergency, "mrc_condition",
                     "jerk_brake_emergency");
    ReadItem<int>(json, time_out_threshold, "mrc_condition",
                  "time_out_threshold");
    ReadItem<double>(json, split_merge_dist_threshold, "mrc_condition",
                     "split_merge_dist_threshold");
    ReadItem<double>(json, ramp_dist_threshold, "mrc_condition",
                     "ramp_dist_threshold");
    ReadItem<double>(json, tunnel_dist_threshold, "mrc_condition",
                     "tunnel_dist_threshold");
    ReadItem<double>(json, construction_zone_dist_threshold, "mrc_condition",
                     "construction_zone_dist_threshold");
    ReadItem<double>(json, v_pull_over_threshold, "mrc_condition",
                     "v_pull_over_threshold");
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
    ReadItem<double>(json, preview_x, "real_time_long_behavior_planner",
                     "preview_x");
    ReadItem<double>(json, dis_zero_speed, "real_time_long_behavior_planner",
                     "dis_zero_speed");
    ReadItem<double>(json, dis_zero_speed_accident,
                     "real_time_long_behavior_planner",
                     "dis_zero_speed_accident");
    ReadItem<double>(json, ttc_brake_hysteresis,
                     "real_time_long_behavior_planner", "ttc_brake_hysteresis");
    ReadItem<double>(json, t_curv, "real_time_long_behavior_planner", "t_curv");
    ReadItem<double>(json, dis_curv, "real_time_long_behavior_planner",
                     "dis_curv");
    ReadItem<double>(json, v_start, "real_time_long_behavior_planner",
                     "v_start");
    ReadItem<double>(json, obstacle_v_start, "real_time_long_behavior_planner",
                     "obstacle_v_start");
    ReadItem<double>(json, distance_stop, "real_time_long_behavior_planner",
                     "distance_stop");
    ReadItem<double>(json, distance_start, "real_time_long_behavior_planner",
                     "distance_start");
    ReadItem<double>(json, dece_to_ramp, "real_time_long_behavior_planner",
                     "dece_to_ramp");
    ReadItem<double>(json, v_limit_ramp, "real_time_long_behavior_planner",
                     "v_limit_ramp");
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
    ReadItem<double>(json, preview_x, "real_time_long_behavior_planner",
                     "preview_x");
    ReadItem<double>(json, dis_zero_speed, "real_time_long_behavior_planner",
                     "dis_zero_speed");
    ReadItem<double>(json, dis_zero_speed_accident,
                     "real_time_long_behavior_planner",
                     "dis_zero_speed_accident");
    ReadItem<double>(json, safe_distance_ttc, "real_time_long_behavior_planner",
                     "safe_distance_ttc");
    ReadItem<bool>(json, enable_lead_two, "real_time_long_behavior_planner",
                   "enable_lead_two");
    ReadItem<bool>(json, enable_rss_model, "real_time_long_behavior_planner",
                   "enable_rss_model");
    ReadItem<double>(json, cruise_set_acc, "real_time_long_behavior_planner",
                     "cruise_set_acc");
    ReadItem<double>(json, cruise_set_dec, "real_time_long_behavior_planner",
                     "cruise_set_dec");
    ReadItem<double>(json, v_start, "real_time_long_behavior_planner",
                     "v_start");
    ReadItem<double>(json, obstacle_v_start, "real_time_long_behavior_planner",
                     "obstacle_v_start");
    ReadItem<double>(json, distance_stop, "real_time_long_behavior_planner",
                     "distance_stop");
    ReadItem<double>(json, distance_start, "real_time_long_behavior_planner",
                     "distance_start");
    ReadItem<double>(json, fast_lead_distance_step,
                     "real_time_long_behavior_planner",
                     "fast_lead_distance_step");
    ReadItem<double>(json, slow_lead_distance_step,
                     "real_time_long_behavior_planner",
                     "slow_lead_distance_step");

    ReadItem<double>(json, cut_in_desired_distance_step,
                     "real_time_long_behavior_planner",
                     "cut_in_desired_distance_step");
    ReadItem<double>(json, soft_bound_corridor_t,
                     "real_time_long_behavior_planner",
                     "soft_bound_corridor_t");
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
    ReadItem<double>(json, preview_x, "real_time_long_behavior_planner",
                     "preview_x");
    ReadItem<double>(json, dis_zero_speed, "real_time_long_behavior_planner",
                     "dis_zero_speed");
    ReadItem<double>(json, dis_zero_speed_accident,
                     "real_time_long_behavior_planner",
                     "dis_zero_speed_accident");
    ReadItem<double>(json, safe_distance_ttc, "real_time_long_behavior_planner",
                     "safe_distance_ttc");
    ReadItem<bool>(json, enable_lead_two, "real_time_long_behavior_planner",
                   "enable_lead_two");
    ReadItem<bool>(json, enable_rss_model, "real_time_long_behavior_planner",
                   "enable_rss_model");
    ReadItem<double>(json, cruise_set_acc, "real_time_long_behavior_planner",
                     "cruise_set_acc");
    ReadItem<double>(json, cruise_set_dec, "real_time_long_behavior_planner",
                     "cruise_set_dec");
    ReadItem<double>(json, v_start, "real_time_long_behavior_planner",
                     "v_start");
    ReadItem<double>(json, v_startmode, "real_time_long_behavior_planner",
                     "v_startmode");
    ReadItem<double>(json, obstacle_v_start, "real_time_long_behavior_planner",
                     "obstacle_v_start");
    ReadItem<double>(json, distance_stop, "real_time_long_behavior_planner",
                     "distance_stop");
    ReadItem<double>(json, distance_start, "real_time_long_behavior_planner",
                     "distance_start");
    ReadItem<double>(json, fast_lead_distance_step,
                     "real_time_long_behavior_planner",
                     "fast_lead_distance_step");
    ReadItem<double>(json, slow_lead_distance_step,
                     "real_time_long_behavior_planner",
                     "slow_lead_distance_step");

    ReadItem<double>(json, cut_in_desired_distance_step,
                     "real_time_long_behavior_planner",
                     "cut_in_desired_distance_step");
    ReadItem<double>(json, soft_bound_corridor_t,
                     "real_time_long_behavior_planner",
                     "soft_bound_corridor_t");
    ReadItem<double>(json, acc_start_max_bound,
                     "real_time_long_behavior_planner", "acc_start_max_bound");
    ReadItem<double>(json, acc_start, "real_time_long_behavior_planner",
                     "acc_start");
    ReadItem<double>(json, v_target_stop_thrd,
                     "real_time_long_behavior_planner", "v_target_stop_thrd");
    ReadItem<double>(json, acc_stop_max_bound,
                     "real_time_long_behavior_planner", "acc_stop_max_bound");
    ReadItem<bool>(json, enable_jlt, "real_time_long_behavior_planner",
                   "enable_jlt");
    ReadItem<bool>(json, enable_narrow_agent_limit,
                   "real_time_long_behavior_planner",
                   "enable_narrow_agent_limit");
    ReadItem<double>(json, v_limit_ramp, "real_time_long_behavior_planner",
                     "v_limit_ramp");
    ReadItem<double>(json, v_limit_near_ramp_zone,
                     "real_time_long_behavior_planner",
                     "v_limit_near_ramp_zone");
    ReadItem<double>(json, dis_near_ramp_zone,
                     "real_time_long_behavior_planner", "dis_near_ramp_zone");
    ReadItem<double>(json, brake_dis_near_ramp_zone,
                     "real_time_long_behavior_planner",
                     "brake_dis_near_ramp_zone");
    ReadItem<double>(json, t_curv, "real_time_long_behavior_planner", "t_curv");
    ReadItem<double>(json, dis_curv, "real_time_long_behavior_planner",
                     "dis_curv");
    ReadItem<double>(json, pre_accelerate_distance_for_merge,
                     "real_time_long_behavior_planner",
                     "pre_accelerate_distance_for_merge");
    ReadItem<bool>(json, enable_intersection_v_limit,
                   "real_time_long_behavior_planner",
                   "enable_intersection_v_limit");
    ReadItem<double>(json, v_lc_speed_adjust, "real_time_long_behavior_planner",
                     "v_lc_speed_adjust");
    ReadItem<double>(json, search_sdmap_curv_dis,
                     "real_time_long_behavior_planner",
                     "search_sdmap_curv_dis");
    ReadItem<double>(json, sdmap_curv_thred, "real_time_long_behavior_planner",
                     "sdmap_curv_thred");
    ReadItem<double>(json, straight_ramp_v_limit,
                     "real_time_long_behavior_planner",
                     "straight_ramp_v_limit");
    ReadItem<bool>(json, enable_sdmap_curv_v_adjust,
                   "real_time_long_behavior_planner",
                   "enable_sdmap_curv_v_adjust");
    ReadItem<bool>(json, enable_speed_adjust, "speed_adjust",
                   "enable_speed_adjust");
    ReadItem<double>(json, dangerous_ttc_thrd,
                     "real_time_long_behavior_planner", "dangerous_ttc_thrd");
    ReadItem<double>(json, tense_ttc_thrd, "real_time_long_behavior_planner",
                     "tense_ttc_thrd");
    ReadItem<double>(json, merge_desired_distance_sharp_rate,
                     "real_time_long_behavior_planner",
                     "merge_desired_distance_sharp_rate");
    ReadItem<double>(json, merge_desired_distance_slow_rate,
                     "real_time_long_behavior_planner",
                     "merge_desired_distance_slow_rate");
    ReadItem<bool>(json, enable_merge_decision_process,
                   "real_time_long_behavior_planner",
                   "enable_merge_decision_process");
    ReadItem<bool>(json, enabe_right_lane_merge_to_ego_lane_decision_process,
                   "real_time_long_behavior_planner",
                   "enabe_right_lane_merge_to_ego_lane_decision_process");
    ReadItem<double>(json, v_intersection_min_limit,
                     "real_time_long_behavior_planner",
                     "v_intersection_min_limit");
    ReadItem<double>(json, acc_lower_bound_in_large_curv,
                     "real_time_long_behavior_planner",
                     "acc_lower_bound_in_large_curv");
    ReadItem<double>(json, jerk_lower_in_large_curv,
                     "real_time_long_behavior_planner",
                     "jerk_lower_in_large_curv");
    ReadItem<double>(json, stop_dis_before_stopline,
                     "real_time_long_behavior_planner",
                     "stop_dis_before_stopline");
    ReadItem<double>(json, stop_dis_before_crosswalk,
                     "real_time_long_behavior_planner",
                     "stop_dis_before_crosswalk");
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

struct SpeedLimitConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, preview_x, "speed_limit_decider", "preview_x");
    ReadItem<double>(json, t_curv, "speed_limit_decider", "t_curv");
    ReadItem<double>(json, dis_curv, "speed_limit_decider", "dis_curv");
    ReadItem<double>(json, pre_accelerate_distance_for_merge,
                     "speed_limit_decider",
                     "pre_accelerate_distance_for_merge");
    ReadItem<double>(json, straight_ramp_v_limit, "speed_limit_decider",
                     "straight_ramp_v_limit");
    ReadItem<double>(json, v_limit_ramp, "speed_limit_decider", "v_limit_ramp");
    ReadItem<double>(json, v_limit_near_ramp_zone, "speed_limit_decider",
                     "v_limit_near_ramp_zone");
    ReadItem<double>(json, dis_near_ramp_zone, "speed_limit_decider",
                     "dis_near_ramp_zone");
    ReadItem<double>(json, brake_dis_near_ramp_zone, "speed_limit_decider",
                     "brake_dis_near_ramp_zone");
    ReadItem<double>(json, acc_to_ramp, "speed_limit_decider", "acc_to_ramp");
    ReadItem<double>(json, v_intersection_min_limit, "speed_limit_decider",
                     "v_intersection_min_limit");
  }
  int lon_num_step = 25;
  double delta_time = 0.2;
  double preview_x = 80.0;
  double t_curv = 2.0;
  double dis_curv = 80.0;
  double pre_accelerate_distance_for_merge = 80.0;
  double straight_ramp_v_limit = 22.22;
  double v_limit_ramp = 16.67;
  double v_limit_near_ramp_zone = 25.0;
  double dis_near_ramp_zone = 1200.0;
  double brake_dis_near_ramp_zone = 700.0;
  double acc_to_ramp = -0.7;
  double v_intersection_min_limit = 11.11;
};

struct SccLonMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, q_ref_pos, "long_motion_ilqr", "q_ref_pos");
    ReadItem<double>(json, q_ref_vel, "long_motion_ilqr", "q_ref_vel");
    ReadItem<double>(json, q_acc, "long_motion_ilqr", "q_acc");
    ReadItem<double>(json, q_jerk, "long_motion_ilqr", "q_jerk");
    ReadItem<double>(json, q_djerk, "long_motion_ilqr", "q_djerk");
    ReadItem<double>(json, q_soft_pos_bound, "long_motion_ilqr",
                     "q_soft_pos_bound");
    ReadItem<double>(json, q_hard_pos_bound, "long_motion_ilqr",
                     "q_hard_pos_bound");
    ReadItem<double>(json, q_sv_bound, "long_motion_ilqr", "q_sv_bound");
    ReadItem<double>(json, q_vel_bound, "long_motion_ilqr", "q_vel_bound");
    ReadItem<double>(json, q_acc_bound, "long_motion_ilqr", "q_acc_bound");
    ReadItem<double>(json, q_acc_bound_v3, "long_motion_ilqr",
                     "q_acc_bound_v3");
    ReadItem<double>(json, q_jerk_bound, "long_motion_ilqr", "q_jerk_bound");
    ReadItem<double>(json, q_stop_s, "long_motion_ilqr", "q_stop_s");

    // start mode q
    ReadItem<double>(json, q_ref_pos_startmode, "long_motion_ilqr",
                     "q_ref_pos_startmode");
    ReadItem<double>(json, q_ref_vel_startmode, "long_motion_ilqr",
                     "q_ref_vel_startmode");
    ReadItem<double>(json, q_acc_startmode, "long_motion_ilqr",
                     "q_acc_startmode");
    ReadItem<double>(json, q_jerk_startmode, "long_motion_ilqr",
                     "q_jerk_startmode");
    ReadItem<double>(json, q_soft_pos_bound_startmode, "long_motion_ilqr",
                     "q_soft_pos_bound_startmode");
    ReadItem<double>(json, q_hard_pos_bound_startmode, "long_motion_ilqr",
                     "q_hard_pos_bound_startmode");
    ReadItem<double>(json, q_djerk_startmode, "long_motion_ilqr",
                     "q_djerk_startmode");
    ReadItem<double>(json, q_vel_bound_startmode, "long_motion_ilqr",
                     "q_vel_bound_startmode");
    ReadItem<double>(json, q_acc_bound_startmode, "long_motion_ilqr",
                     "q_acc_bound_startmode");
    ReadItem<double>(json, q_jerk_bound_startmode, "long_motion_ilqr",
                     "q_jerk_bound_startmode");
    ReadItem<double>(json, q_stop_s_startmode, "long_motion_ilqr",
                     "q_stop_s_startmode");

    // stop mode q
    ReadItem<double>(json, q_ref_pos_stopmode, "long_motion_ilqr",
                     "q_ref_pos_stopmode");
    ReadItem<double>(json, q_ref_vel_stopmode, "long_motion_ilqr",
                     "q_ref_vel_stopmode");
    ReadItem<double>(json, q_acc_stopmode, "long_motion_ilqr",
                     "q_acc_stopmode");
    ReadItem<double>(json, q_jerk_stopmode, "long_motion_ilqr",
                     "q_jerk_stopmode");
    ReadItem<double>(json, q_soft_pos_bound_stopmode, "long_motion_ilqr",
                     "q_soft_pos_bound_stopmode");
    ReadItem<double>(json, q_hard_pos_bound_stopmode, "long_motion_ilqr",
                     "q_hard_pos_bound_stopmode");
    ReadItem<double>(json, q_sv_bound_stopmode, "long_motion_ilqr",
                     "q_sv_bound_stopmode");
    ReadItem<double>(json, q_vel_bound_stopmode, "long_motion_ilqr",
                     "q_vel_bound_stopmode");
    ReadItem<double>(json, q_acc_bound_stopmode, "long_motion_ilqr",
                     "q_acc_bound_stopmode");
    ReadItem<double>(json, q_jerk_bound_stopmode, "long_motion_ilqr",
                     "q_jerk_bound_stopmode");
    ReadItem<double>(json, q_stop_s_stopmode, "long_motion_ilqr",
                     "q_stop_s_stopmode");
    // v_target stop threshold
    ReadItem<double>(json, v_target_stop_thrd, "long_motion_ilqr",
                     "v_target_stop_thrd");
    ReadItem<double>(json, q_ref_pos_speed_adjust, "long_motion_ilqr",
                     "q_ref_pos_speed_adjust");
    ReadItem<bool>(json, enable_speed_adjust, "speed_adjust",
                   "enable_speed_adjust");
    ReadItem<double>(json, q_acc_speed_adjust, "long_motion_ilqr",
                     "q_acc_speed_adjust");
    ReadItem<double>(json, q_jerk_speed_adjust, "long_motion_ilqr",
                     "q_jerk_speed_adjust");
  }
  double q_ref_pos = 1.0;
  double q_ref_vel = 0.05;
  double q_acc = 10.0;
  double q_jerk = 5.0;
  double q_djerk = 5.0;

  double q_soft_pos_bound = 5.0;
  double q_hard_pos_bound = 1000.0;
  double q_sv_bound = 1000.0;
  double q_vel_bound = 400.0;
  double q_acc_bound = 400.0;
  double q_acc_bound_v3 = 3200.0;
  double q_jerk_bound = 100.0;
  double q_djerk_bound = 100.0;
  double q_stop_s = 2000.0;

  // only in start mode
  double q_ref_pos_startmode = 1.0;
  double q_ref_vel_startmode = 0.05;
  double q_acc_startmode = 10.0;
  double q_jerk_startmode = 5.0;
  double q_djerk_startmode = 20.0;

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
  double q_acc_speed_adjust = 10.0;
  double q_jerk_speed_adjust = 5.0;
};

struct ResultTrajectoryGeneratorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<bool>(json, use_dynamic_lat_jerk_thr, "result_trajectory_generator",
                   "use_dynamic_lat_jerk_thr");
    ReadItem<bool>(json, enable_lat_traj, "result_trajectory_generator",
                   "enable_lat_traj");
    ReadItem<double>(json, lat_acc_thr, "result_trajectory_generator",
                     "lat_acc_thr");
    ReadItem<double>(json, lat_jerk_thr, "result_trajectory_generator",
                     "lat_jerk_thr");
    ReadItem<double>(json, ramp_lat_jerk_thr, "result_trajectory_generator",
                     "ramp_lat_jerk_thr");
    ReadItem<double>(json, lon_acc_thr, "result_trajectory_generator",
                     "lon_acc_thr");
    ReadItem<double>(json, lon_jerk_thr, "result_trajectory_generator",
                     "lon_jerk_thr");
  }
  bool use_dynamic_lat_jerk_thr = false;
  double planning_result_delta_time = 0.025;
  bool is_pwj_planning = false;
  bool enable_lat_traj = false;
  double lat_acc_thr = 3.0;
  double lat_jerk_thr = 0.3;
  double ramp_lat_jerk_thr = 0.65;
  double lon_acc_thr = 3.0;
  double lon_jerk_thr = 0.3;
};

struct TrafficLightDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<bool>(json, enable_tfl_decider, "traffic_light_decider",
                   "enable_tfl_decider");
    ReadItem<double>(json, virtual_dis_before_stopline, "traffic_light_decider",
                     "virtual_dis_before_stopline");
    ReadItem<double>(json, stopline_tfl_dis_thred, "traffic_light_decider",
                     "stopline_tfl_dis_thred");
  }

  bool enable_tfl_decider = false;
  double virtual_dis_before_stopline = 20;
  double stopline_tfl_dis_thred = 50;
};

struct CrossingAgentDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<bool>(json, enable_crossing_decider, "crossing_agent_decider",
                   "enable_crossing_decider");
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
    supper_limit_for_OD_straight = read_json_key<double>(
        json, "supper_limit_for_OD_straight", supper_limit_for_OD_straight);
    supper_limit_for_OD_bend = read_json_key<double>(
        json, "supper_limit_for_OD_bend", supper_limit_for_OD_bend);
    processed_trajectory_filter_alpha =
        read_json_key<double>(json, "processed_trajectory_filter_alpha");
    processed_trajectory_acc_thr =
        read_json_key<double>(json, "processed_trajectory_acc_thr");
  }
  double frenet_obstacle_range_s_min = -50.0;
  double frenet_obstacle_range_s_max = 180.0;
  double frenet_obstacle_range_l_min = -50.0;
  double frenet_obstacle_range_l_max = 50.0;
  bool enable_bbox_mode = true;
  double max_speed_static_obstacle = 0.5;
  double supper_limit_for_OD_straight = 6.0;
  double supper_limit_for_OD_bend = 6.0;
  double processed_trajectory_filter_alpha = 0.4;
  double processed_trajectory_acc_thr = -0.5;
};

struct EgoPlanningEdtManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    car_body_lat_safe_buffer = read_json_key<double>(
        json, "car_body_lat_safe_buffer", car_body_lat_safe_buffer);
    lon_safe_buffer =
        read_json_key<double>(json, "lon_safe_buffer", lon_safe_buffer);
    mirror_buffer = read_json_key<double>(json, "mirror_buffer", mirror_buffer);
  }
  double car_body_lat_safe_buffer = 0.2;
  double lon_safe_buffer = 0.2;
  double mirror_buffer = 0.2;
};

struct EgoPlanningEgoStateManagerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    cruise_routing_speed = read_json_key<double>(json, "cruise_routing_speed",
                                                 cruise_routing_speed);
    cruise_searching_speed = read_json_key<double>(
        json, "cruise_searching_speed", cruise_searching_speed);
    ReadItem<double>(json, rads_cruise_speed, "rads_cruise_speed");
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
    enable_delta_stitch_in_replan = read_json_key<bool>(
        json, "enable_delta_stitch_in_replan", enable_delta_stitch_in_replan);
    enable_ego_state_compensation = read_json_key<bool>(
        json, "enable_ego_state_compensation", enable_ego_state_compensation);
    read_json_vec<double>(json, "replan_longitudinal_distance_threshold_speed",
                          replan_longitudinal_distance_threshold_speed);
    read_json_vec<double>(json, "replan_longitudinal_distance_threshold_value",
                          replan_longitudinal_distance_threshold_value);
    read_json_vec<double>(json, "hpp_replan_threshold_speed",
                          hpp_replan_threshold_speed);
    read_json_vec<double>(json, "hpp_replan_lat_err_threshold_value",
                          hpp_replan_lat_err_threshold_value);
    read_json_vec<double>(json, "hpp_replan_lon_err_threshold_value",
                          hpp_replan_lon_err_threshold_value);
    ReadItem<bool>(json, use_yaw_rate_to_delta, "use_yaw_rate_to_delta");
  }
  double cruise_routing_speed = 5.55;
  double cruise_searching_speed = 1.5;
  double rads_cruise_speed = 1.5;

  double max_replan_lat_err = 0.6;
  double max_replan_theta_err = 10.0;
  double max_replan_lon_err = 1.0;
  double max_replan_dist_err = 1.5;
  bool enable_delta_stitch_in_replan = false;
  std::vector<double> replan_longitudinal_distance_threshold_speed{11.111,
                                                                   27.778};
  std::vector<double> replan_longitudinal_distance_threshold_value{1.0, 1.1};
  std::vector<double> hpp_replan_threshold_speed{1.0, 4.167, 11.111};
  std::vector<double> hpp_replan_lat_err_threshold_value{0.2, 0.4, 0.6};
  std::vector<double> hpp_replan_lon_err_threshold_value{0.1, 0.5, 1.0};
  double hpp_max_replan_lat_err = 0.45;
  double hpp_max_replan_theta_err = 12.0;
  double hpp_max_replan_lon_err = 0.55;
  double hpp_max_replan_dist_err = 0.8;

  double kEpsilon_v = 0.0;
  double kEpsilon_a = 0.0;

  bool enable_constanct_velocity_in_predicted_vehicle_state = false;
  bool enable_ego_state_compensation = false;
  bool use_yaw_rate_to_delta = false;
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
  bool enable_backward_extend_st_boundary = true;
  double backward_extend_length_for_lane_change = 50.0;
  double backward_extend_sample_resolution = 3.0;
  double lane_keeping_lower_lateral_buffer_m = 0.1;
  double lane_keeping_upper_lateral_buffer_m = 0.3;
  double lane_keeping_lower_speed_kph = 10.0;
  double lane_keeping_upper_speed_kph = 30.0;
  double lane_keeping_large_agent_lateral_buffer_m = 0.2;
  double lane_change_lateral_buffer_m = 0.2;
  double lane_keeping_large_agent_lower_lateral_buffer_m = 0.2;
  double lane_keeping_large_agent_upper_lateral_buffer_m = 0.2;
  double lane_keeping_large_agent_lower_speed_kph = 10.0;
  double lane_keeping_large_agent_upper_speed_kph = 30.0;
  double front_agent_lower_s_safety_buffer_for_lane_change = 8.0;
  double large_agent_expand_param_for_consistency = 0.2;
  double large_agent_small_expand_param_for_consistency = 0.15;
  double cone_lateral_buffer_m = 0.2;
  double lane_keeping_large_heading_diff_lon_buffer_m = 0.3;
  double person_lat_buffer_m = 0.4;
  double person_lon_buffer_m = 0.4;
  double bycicle_lat_buffer_m = 0.4;
  double bycicle_lon_buffer_m = 0.4;
  double tricycle_lat_buffer_m = 0.4;
  double tricycle_lon_buffer_m = 0.4;
  double backward_extend_time_s = 3.0;
  double reverse_vehicle_lat_buffer_m = 0.2;
};

struct StopDestinationDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    ReadItem<double>(json, stop_destination_virtual_agent_time_headway,
                     "speed_planning", "stop_destination_decider",
                     "stop_destination_virtual_agent_time_headway");
    ReadItem<double>(json, stop_destination_extended_s_buffer,
                      "speed_planning", "stop_destination_decider",
                      "stop_destination_extended_s_buffer");
  }

  double stop_destination_virtual_agent_time_headway = 1.0;
  double stop_destination_extended_s_buffer = 2.0;
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

    // radical search style
    {
      max_accel_limit_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "max_accel_limit_radical_style"},
          max_accel_limit_radical_style);
      min_accel_limit_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "min_accel_limit_radical_style"},
          min_accel_limit_radical_style);
      max_jerk_limit_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "max_jerk_limit_radical_style"},
          max_jerk_limit_radical_style);
      min_jerk_limit_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "min_jerk_limit_radical_style"},
          min_jerk_limit_radical_style);
      accel_sample_num_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "accel_sample_num_radical_style"},
          accel_sample_num_radical_style);
      s_step_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "s_step_radical_style"},
          s_step_radical_style);
      t_step_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "t_step_radical_style"},
          t_step_radical_style);
      vel_step_radical_style = read_json_keys<double>(
          json,
          std::vector<std::string>{"speed_planning", "st_graph_searcher",
                                   "radical_search_style",
                                   "vel_step_radical_style"},
          vel_step_radical_style);
    }

    // st graph search weight
    {
      ReadItem<double>(json, weight_accel_sign_overtake, "speed_planning",
                       "st_graph_searcher", "search_weight",
                       "weight_accel_sign_overtake");
    }
    ReadItem(json, distance_ego_rear_edge_to_lower_bound_when_overtake,
             "speed_planning", "st_graph_searcher",
             "distance_ego_rear_edge_to_lower_bound_when_overtake");
    ReadItem(json, cost_ego_overtake_has_collision_with_lower_bound,
             "speed_planning", "st_graph_searcher",
             "cost_ego_overtake_has_collision_with_lower_bound");
    ReadItem(json, cutin_time_st_graph_threshold, "speed_planning",
             "st_graph_searcher", "cutin_time_st_graph_threshold");
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

  // radical search style info
  double max_accel_limit_radical_style = 5.0;
  double min_accel_limit_radical_style = -6.0;
  double max_jerk_limit_radical_style = 10.0;
  double min_jerk_limit_radical_style = -10.0;
  double accel_sample_num_radical_style = 20;
  double s_step_radical_style = 0.25;
  double t_step_radical_style = 0.25;
  double vel_step_radical_style = 0.4;

  double weight_yield = 2.0;
  double weight_overtake = 2.0;
  double weight_vel = 0.10;
  double weight_accel = 0.10;
  double weight_accel_sign = 0.5;
  double weight_accel_sign_overtake = 5.0;
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

  double distance_ego_rear_edge_to_lower_bound_when_overtake = 5.0;
  double cost_ego_overtake_has_collision_with_lower_bound = 1.0;
  double cutin_time_st_graph_threshold = 5.5;
};

struct LongitudinalDecisionDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ignore_agent_ttc_to_ego_thrd = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning",
                                 "longitudinal_decision_decider",
                                 "ignore_agent_ttc_to_ego_thrd"},
        ignore_agent_ttc_to_ego_thrd);
    ignore_ego_ttc_to_agent_thrd = read_json_keys<double>(
        json,
        std::vector<std::string>{"speed_planning",
                                 "longitudinal_decision_decider",
                                 "ignore_ego_ttc_to_agent_thrd"},
        ignore_ego_ttc_to_agent_thrd);
    lat_distance_close_enough_to_planned_path_thrd = read_json_keys<double>(
        json,
        std::vector<std::string>{
            "speed_planning", "longitudinal_decision_decider",
            "lat_distance_close_enough_to_planned_path_thrd"},
        lat_distance_close_enough_to_planned_path_thrd);
    mute_invade_neighbor_decision = read_json_keys<bool>(
        json,
        std::vector<std::string>{"speed_planning",
                                 "longitudinal_decision_decider",
                                 "mute_invade_neighbor_decision"},
        mute_invade_neighbor_decision);
  }

  double ignore_agent_ttc_to_ego_thrd = 3.0;
  double ignore_ego_ttc_to_agent_thrd = 3.0;
  double lat_distance_close_enough_to_planned_path_thrd = 0.5;
  double ego_predeceleration_distance_to_front_agent_threshold = 3.5;
  double close_to_same_velocity_difference_buffer = 1.5;
  bool mute_invade_neighbor_decision = false;
};

struct AgentHeadwayConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadVector(json, ego_vel_table, "speed_planning", "agent_headway_decider",
               "ego_vel_table");
    ReadVector(json, ego_normal_thw_table_level_1,
               "speed_planning", "agent_headway_decider",
               "ego_normal_thw_table_level_1");
    ReadVector(json, ego_normal_thw_table_level_2,
                "speed_planning", "agent_headway_decider",
                "ego_normal_thw_table_level_2");
    ReadVector(json, ego_normal_thw_table_level_3,
                "speed_planning", "agent_headway_decider",
                "ego_normal_thw_table_level_3");
    ReadVector(json, ego_normal_thw_table_level_4,
                "speed_planning", "agent_headway_decider",
                "ego_normal_thw_table_level_4");
    ReadVector(json, ego_normal_thw_table_level_5,
                "speed_planning", "agent_headway_decider",
                "ego_normal_thw_table_level_5");
  }
  double plan_time = 5.0;
  double dt = 0.2;
  double cutin_headway_threshold = 1.0;
  double smallest_headway_threshold = 1.2;
  double headway_step = 0.05;
  std::vector<double> ego_vel_table = {0.0, 3.33, 16.67, 26.67, 36.67};
  std::vector<double> ego_normal_thw_table_level_1 = {1.05, 1.14, 1.25, 1.5,
                                                      2.0};
  std::vector<double> ego_normal_thw_table_level_2 = {1.1, 1.16, 1.35, 2.1,
                                                      2.2};
  std::vector<double> ego_normal_thw_table_level_3 = {1.15, 1.18, 1.5, 2.2,
                                                      2.4};
  std::vector<double> ego_normal_thw_table_level_4 = {1.2, 1.4, 1.65, 2.3, 2.5};
  std::vector<double> ego_normal_thw_table_level_5 = {1.2, 1.5, 1.75, 2.4,
                                                      2.55};
  std::vector<std::pair<int32_t, double>> normal_headway_table = {
      {0, 1.1}, {1, 1.3}, {2, 1.5}, {3, 2.1}, {4, 2.8}};
  std::vector<std::pair<int32_t, double>> aggressive_headway_table = {
      {0, 1.1}, {1, 1.2}, {2, 1.4}, {3, 1.8}, {4, 2.0}};
  std::vector<std::pair<int32_t, double>> conservative_headway_table = {
      {0, 1.2}, {1, 1.8}, {2, 2.5}, {3, 3.0}, {4, 4.0}};
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

struct StartStopDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, ego_vel_begin_stop_threshold, "speed_planning",
                     "start_stop_decider", "ego_vel_begin_stop_threshold");
    ReadItem<double>(json, ego_vel_start_mode_threshold, "speed_planning",
                     "start_stop_decider", "ego_vel_start_mode_threshold");
    ReadItem<double>(json, cipv_vel_begin_start_threshold, "speed_planning",
                     "start_stop_decider", "cipv_vel_begin_start_threshold");
    ReadItem<double>(json, cipv_relative_s_begin_start_threshold,
                     "speed_planning", "start_stop_decider",
                     "cipv_relative_s_begin_start_threshold");
    ReadItem<double>(json, distance_stop_buffer_between_ego_and_cipv_threshold,

                     "speed_planning", "start_stop_decider",
                     "distance_stop_buffer_between_ego_and_cipv_threshold");
    ReadItem<double>(json, distance_start_between_ego_and_cipv_threshold,
                     "speed_planning", "start_stop_decider",
                     "distance_start_between_ego_and_cipv_threshold");
    ReadItem<double>(json, distance_to_stop_line_ego_threshold,
                     "speed_planning", "start_stop_decider",
                     "distance_to_stop_line_ego_threshold");
    ReadItem<double>(json,
                     desired_stopped_distance_between_ego_and_cipv_threshold,
                     "speed_planning", "start_stop_decider",
                     "desired_stopped_distance_between_ego_and_cipv_threshold");
    ReadItem<double>(json, distance_to_go_threshold, "speed_planning",
                     "start_stop_decider", "distance_to_go_threshold");
    ReadItem<double>(json, ego_vel_start_mode_threshold_rads, "speed_planning",
                     "start_stop_decider", "ego_vel_start_mode_threshold_rads");
  }

  double ego_vel_begin_stop_threshold = 0.5;
  double ego_vel_start_mode_threshold = 5.5;
  double ego_vel_start_mode_threshold_rads = 0.2;
  double cipv_vel_begin_start_threshold = 0.5;
  double cipv_relative_s_begin_start_threshold = 3.5;
  double distance_stop_buffer_between_ego_and_cipv_threshold = 2.0;
  double distance_start_between_ego_and_cipv_threshold = 0.3;
  double distance_to_stop_line_ego_threshold = 5.5;
  double desired_stopped_distance_between_ego_and_cipv_threshold = 3.0;
  double distance_to_go_threshold = 5.5;
  double stop_destination_to_ego_distance = 6.0;
};

struct EgoMotionPreplannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, ego_motion_simulate_dt_resolution,
                     "ego_motion_preplanner",
                     "ego_motion_simulate_dt_resolution");
    ReadItem<double>(json, pure_pursuit_ld, "ego_motion_preplanner",
                     "pure_pursuit_ld");
    ReadItem<double>(json, idm_minimum_spacing, "ego_motion_preplanner",
                     "idm_minimum_spacing");
    ReadItem<double>(json, idm_desired_time_headway, "ego_motion_preplanner",
                     "idm_desired_time_headway");
    ReadItem<double>(json, idm_max_accel, "ego_motion_preplanner",
                     "idm_max_accel");
    ReadItem<double>(json, idm_comfortable_breaking_deceleration,
                     "ego_motion_preplanner",
                     "idm_comfortable_breaking_deceleration");
    ReadItem<double>(json, idm_hard_breaking_deceleration,
                     "ego_motion_preplanner", "idm_hard_breaking_deceleration");
    ReadItem<double>(json, idm_exponent, "ego_motion_preplanner",
                     "idm_exponent");
  }

  double ego_motion_simulate_dt_resolution = 0.1;
  double pure_pursuit_ld = 6.0;
  double idm_minimum_spacing = 2.0;
  double idm_desired_time_headway = 1.5;
  double idm_max_accel = 2.0;
  double idm_comfortable_breaking_deceleration = 3.0;
  double idm_hard_breaking_deceleration = 4.5;
  double idm_exponent = 4.0;
};

struct SpeedPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    // comfort_kinematic_param
    {
      ReadItem<double>(json, comfort_kinematic_param.acc_positive_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_positive_upper");
      ReadItem<double>(json, comfort_kinematic_param.acc_positive_speed_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_positive_speed_lower");
      ReadItem<double>(json, comfort_kinematic_param.acc_positive_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_positive_lower");

      ReadItem<double>(json, comfort_kinematic_param.acc_positive_speed_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_positive_speed_upper");

      ReadItem<double>(json, comfort_kinematic_param.acc_negative_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_negative_lower");

      ReadItem<double>(json, comfort_kinematic_param.acc_negative_speed_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_negative_speed_lower");

      ReadItem<double>(json, comfort_kinematic_param.acc_negative_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_negative_upper");

      ReadItem<double>(json, comfort_kinematic_param.acc_negative_speed_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "acc_negative_speed_upper");

      ReadItem<double>(json, comfort_kinematic_param.jerk_positive_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_positive_upper");

      ReadItem<double>(json, comfort_kinematic_param.jerk_positive_speed_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_positive_speed_lower");

      ReadItem<double>(json, comfort_kinematic_param.jerk_positive_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_positive_lower");

      ReadItem<double>(json, comfort_kinematic_param.jerk_positive_speed_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_positive_speed_upper");

      ReadItem<double>(json, comfort_kinematic_param.jerk_negative_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_negative_lower");

      ReadItem<double>(json, comfort_kinematic_param.jerk_negative_speed_lower,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_negative_speed_lower");

      ReadItem<double>(json, comfort_kinematic_param.jerk_negative_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_negative_upper");

      ReadItem<double>(json, comfort_kinematic_param.jerk_negative_speed_upper,
                       "speed_planning", "cruise_target",
                       "comfort_kinematic_param", "jerk_negative_speed_upper");
    }

    // kappa_kinematic_param
    {
      ReadItem<double>(json, kappa_kinematic_param.acc_positive_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_positive_upper");
      ReadItem<double>(json, kappa_kinematic_param.acc_positive_speed_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_positive_speed_lower");

      ReadItem<double>(json, kappa_kinematic_param.acc_positive_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_positive_lower");

      ReadItem<double>(json, kappa_kinematic_param.acc_positive_speed_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_positive_speed_upper");

      ReadItem<double>(json, kappa_kinematic_param.acc_negative_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_negative_lower");

      ReadItem<double>(json, kappa_kinematic_param.acc_negative_speed_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_negative_speed_lower");

      ReadItem<double>(json, kappa_kinematic_param.acc_negative_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_negative_upper");

      ReadItem<double>(json, kappa_kinematic_param.acc_negative_speed_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "acc_negative_speed_upper");

      ReadItem<double>(json, kappa_kinematic_param.jerk_positive_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_positive_upper");

      ReadItem<double>(json, kappa_kinematic_param.jerk_positive_speed_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_positive_speed_lower");

      ReadItem<double>(json, kappa_kinematic_param.jerk_positive_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_positive_lower");

      ReadItem<double>(json, kappa_kinematic_param.jerk_positive_speed_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_positive_speed_upper");

      ReadItem<double>(json, kappa_kinematic_param.jerk_negative_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_negative_lower");

      ReadItem<double>(json, kappa_kinematic_param.jerk_negative_speed_lower,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_negative_speed_lower");

      ReadItem<double>(json, kappa_kinematic_param.jerk_negative_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_negative_upper");

      ReadItem<double>(json, kappa_kinematic_param.jerk_negative_speed_upper,
                       "speed_planning", "cruise_target",
                       "kappa_kinematic_param", "jerk_negative_speed_upper");
    }

    {
      ReadItem<double>(json, avoid_agent_kinematic_param.acc_positive_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_positive_upper");
      ReadItem<double>(json, avoid_agent_kinematic_param.acc_positive_speed_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_positive_speed_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.acc_positive_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_positive_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.acc_positive_speed_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_positive_speed_upper");

      ReadItem<double>(json, avoid_agent_kinematic_param.acc_negative_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_negative_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.acc_negative_speed_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_negative_speed_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.acc_negative_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_negative_upper");

      ReadItem<double>(json, avoid_agent_kinematic_param.acc_negative_speed_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "acc_negative_speed_upper");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_positive_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_positive_upper");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_positive_speed_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_positive_speed_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_positive_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_positive_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_positive_speed_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_positive_speed_upper");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_negative_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_negative_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_negative_speed_lower,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_negative_speed_lower");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_negative_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_negative_upper");

      ReadItem<double>(json, avoid_agent_kinematic_param.jerk_negative_speed_upper,
                       "speed_planning", "cruise_target",
                       "avoid_agent_kinematic_param", "jerk_negative_speed_upper");
    }

    //  kappa_speed_limit_table
    {
      read_json_vec(json,
                    std::vector<std::string>{"normal_kappa_speed_limit_table",
                                             "cruise_target", "kappa_table"},
                    normal_kappa_speed_limit_table.kappa_table);
      read_json_vec(json,
                    std::vector<std::string>{"normal_kappa_speed_limit_table",
                                             "cruise_target", "speed_table"},
                    normal_kappa_speed_limit_table.speed_table);
      read_json_vec(
          json,
          std::vector<std::string>{"lane_change_kappa_speed_limit_table",
                                   "cruise_target", "kappa_table"},
          lane_change_kappa_speed_limit_table.kappa_table);
      read_json_vec(
          json,
          std::vector<std::string>{"lane_change_kappa_speed_limit_table",
                                   "cruise_target", "speed_table"},
          lane_change_kappa_speed_limit_table.speed_table);
    }
    // loading low speed follow config values
    {
      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "cruise_target",
                                             "low_speed_follow_acc_traj_table",
                                             "traj_table"},
                    low_speed_follow_acc_traj_table.traj_table);
      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "cruise_target",
                                             "low_speed_follow_acc_traj_table",
                                             "acc_table"},
                    low_speed_follow_acc_traj_table.acc_table);

      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "cruise_target",
                                             "low_speed_follow_jerk_traj_table",
                                             "traj_table"},
                    low_speed_follow_jerk_traj_table.traj_table);
      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "cruise_target",
                                             "low_speed_follow_jerk_traj_table",
                                             "jerk_table"},
                    low_speed_follow_jerk_traj_table.jerk_table);
    }

    // loading cruise acc bound config values
    {
      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "bound_maker",
                                             "cruise_acc_bound_table",
                                             "vel_table"},
                                             cruise_acc_bound_table.vel_table);
      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "bound_maker",
                                             "cruise_acc_bound_table",
                                             "acc_table"},
                                             cruise_acc_bound_table.acc_table);

      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "bound_maker",
                                             "cruise_dec_bound_table",
                                             "vel_table"},
                                             cruise_dec_bound_table.vel_table);
      read_json_vec(json,
                    std::vector<std::string>{"speed_planning", "bound_maker",
                                             "cruise_dec_bound_table",
                                             "acc_table"},
                                             cruise_dec_bound_table.acc_table);
    }

    ReadItem<double>(json, lane_change_upper_speed_limit_kph, "speed_planning",
                     "lane_change_upper_speed_limit_kph");
    ReadItem<double>(json, low_speed_follow_speed_thred_mps, "speed_planning",
                     "low_speed_follow_speed_thred_mps");
    ReadItem<double>(json, low_speed_follow_accel_release_traj_len, "speed_planning",
                    "low_speed_follow_accel_release_traj_len");
    ReadItem<bool>(json, enable_speed_adjust, "speed_adjust",
                   "enable_speed_adjust");

    // neighbor target
    {
      ReadItem<double>(json, neighbor_target_min_jerk, "speed_planning",
                       "neighbor_target", "neighbor_target_min_jerk");
      ReadItem<double>(json, neighbor_target_max_jerk, "speed_planning",
                       "neighbor_target", "neighbor_target_max_jerk");
      ReadItem<double>(json, neighbor_target_min_acc, "speed_planning",
                       "neighbor_target", "neighbor_target_min_acc");
      ReadItem<double>(json, neighbor_target_max_acc_lower, "speed_planning",
                       "neighbor_target", "neighbor_target_max_acc_lower");
      ReadItem<double>(json, neighbor_target_max_acc_upper, "speed_planning",
                       "neighbor_target", "neighbor_target_max_acc_upper");
      ReadItem<double>(json, neighbor_target_vel_lower_bound, "speed_planning",
                       "neighbor_target", "neighbor_target_vel_lower_bound");
      ReadItem<double>(json, neighbor_target_vel_upper_bound, "speed_planning",
                       "neighbor_target", "neighbor_target_vel_upper_bound");
      ReadItem<double>(json, neighbor_target_vel_buffer, "speed_planning",
                       "neighbor_target", "neighbor_target_vel_buffer");
      ReadItem<double>(json, neighbor_target_p_precision, "speed_planning",
                       "neighbor_target", "neighbor_target_p_precision");
      ReadItem<double>(json, neighbor_target_kEpsilon, "speed_planning",
                       "neighbor_target", "neighbor_target_kEpsilon");
      ReadItem<double>(json, neighbor_target_neighbor_yield_distance,
                       "speed_planning", "neighbor_target",
                       "neighbor_target_neighbor_yield_distance");
      ReadItem<double>(json, neighbor_target_neighbor_overtake_distance,
                       "speed_planning", "neighbor_target",
                       "neighbor_target_neighbor_overtake_distance");
      ReadItem<double>(json, neighbor_target_neighbor_limit_velocity,
                       "speed_planning", "neighbor_target",
                       "neighbor_target_neighbor_limit_velocity");
    }

    // weight maker
    {
      ReadItem<double>(json, weight_maker_config.s_weight, "speed_planning",
                       "weight_maker", "s_weight");
      ReadItem<double>(json, weight_maker_config.start_s_weight,
                       "speed_planning", "weight_maker", "start_s_weight");
      ReadItem<double>(json, weight_maker_config.follow_s_weight,
                       "speed_planning", "weight_maker", "follow_s_weight");
      ReadItem<double>(json, weight_maker_config.overtake_s_weight,
                       "speed_planning", "weight_maker", "overtake_s_weight");
      ReadItem<double>(json, weight_maker_config.neighbor_s_weight,
                       "speed_planning", "weight_maker", "neighbor_s_weight");
      ReadItem<double>(json, weight_maker_config.end_s_weight, "speed_planning",
                       "weight_maker", "end_s_weight");
      ReadItem<double>(json, weight_maker_config.max_s_weight_time,
                       "speed_planning", "weight_maker", "max_s_weight_time");
      ReadItem<double>(json, weight_maker_config.front_lower_weight,
                       "speed_planning", "weight_maker", "front_lower_weight");
      ReadItem<double>(json, weight_maker_config.back_upper_weight,
                       "speed_planning", "weight_maker", "back_upper_weight");
      ReadItem<double>(json, weight_maker_config.max_s_weight, "speed_planning",
                       "weight_maker", "max_s_weight");
    }
  }

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
  bool enable_speed_adjust = true;

  // follow target
  double lower_speed_min_follow_distance_gap = 3.0;
  double high_speed_min_follow_distance_gap = 4.5;
  double low_speed_threshold_kmph = 18;
  double high_speed_threshold_kmph = 30;
  double large_vehicle_min_follow_distance_gap = 4.5;
  double cone_min_follow_distance_gap = 4.5;
  double traffic_light_min_follow_distance_gap = 2.0;

  // neighbor target
  double neighbor_target_min_jerk = -1.5;
  double neighbor_target_max_jerk = 2.0;
  double neighbor_target_min_acc = -1.5;
  double neighbor_target_max_acc_lower = 0.8;
  double neighbor_target_max_acc_upper = 2.0;
  double neighbor_target_vel_lower_bound = 5.0;
  double neighbor_target_vel_upper_bound = 15.0;
  double neighbor_target_vel_buffer = 5.0;
  double neighbor_target_p_precision = 0.3;
  double neighbor_target_kEpsilon = 1e-10;
  double neighbor_target_neighbor_yield_distance = 10.0;
  double neighbor_target_neighbor_overtake_distance = 10.0;
  double neighbor_target_neighbor_limit_velocity = 5.0;

  // cruise target relevance
  double lane_change_upper_speed_limit_kph = 150.0;
  double low_speed_follow_speed_thred_mps = 1.0;
  double low_speed_follow_accel_release_traj_len = 5.0;
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
    double low_speed_acc_upper_bound = 1.2;
    double high_speed_acc_upper_bound = 0.8;
    double lane_change_low_speed_acc_upper_bound = 2.4;
    double lane_change_high_speed_acc_upper_bound = 1.6;
    double low_speed_threshold_with_acc_upper_bound = 5.5;
    double high_speed_threshold_with_acc_upper_bound = 16.67;
    double acc_lower_bound = -5.0;
    double jerk_lower_bound = -5.0;
    double jerk_upper_bound = 5.0;
  };

  struct KappaSpeedLimitTable {
    std::vector<double> kappa_table{
        0.0005, 0.00074, 0.00142, 0.00167, 0.0018, 0.002, 0.0025, 0.0033,
        0.005,  0.01,    0.02,    0.0333,  0.04,   0.069, 0.15,   0.2};
    std::vector<double> speed_table{150.0, 130.0, 105.0, 95.0, 90.0, 85.0,
                                    80.0,  70.0,  55.0,  47.0, 27.0, 23.0,
                                    20.0,  15.0,  10.0,  7.0};
  };

  struct LowSpeedFollowAccTrajTable {
    std::vector<double> traj_table{10.0, 21.0, 35.0};
    std::vector<double> acc_table {0.50, 0.80, 1.35};
  };

  struct LowSpeedFollowJerkTrajTable {
    std::vector<double> traj_table{15.0, 25.0, 35.0};
    std::vector<double> jerk_table{2.50, 2.20, 2.00};
  };

  struct CruiseACCBoundTable {
    std::vector<double> vel_table{0.0, 5.0, 10.0, 20.0, 40.0};
    std::vector<double> acc_table{1.0, 0.85, 0.6, 0.5, 0.3};
  };

  struct CruiseDECBoundTable {
    std::vector<double> vel_table{0.0, 5.0, 10.0, 20.0, 40.0};
    std::vector<double> acc_table{-1.5, -1.5, -1.5, -1.0, -0.3};
  };

  KinematicParam comfort_kinematic_param;
  KinematicParam kappa_kinematic_param;
  KinematicParam avoid_agent_kinematic_param;

  SpeedPlanningBound speed_planning_bound;

  KappaSpeedLimitTable normal_kappa_speed_limit_table;
  KappaSpeedLimitTable lane_change_kappa_speed_limit_table;

  LowSpeedFollowAccTrajTable low_speed_follow_acc_traj_table;
  LowSpeedFollowJerkTrajTable low_speed_follow_jerk_traj_table;

  CruiseACCBoundTable cruise_acc_bound_table;
  CruiseDECBoundTable cruise_dec_bound_table;

  // weight maker
  struct WeightConfig {
    double s_weight = 1.0;
    double start_s_weight = 5.0;
    double v_weight = 0.0;
    double a_weight = 10.0;
    double jerk_weight = 100.0;
    double cruise_v_weight = 20.0;
    double follow_s_weight = 1.0;
    double overtake_s_weight = 1.0;
    double neighbor_s_weight = 1.0;
    double end_s_weight = 40.0;
    double max_s_weight_time = 3.0;
    double front_lower_weight = 0.5;
    double back_upper_weight = 1.5;
    double max_s_weight = 2.0;
    double s_speed_upper_weight_v = 8.33;
    double s_speed_lower_weight_v = 2.78;
    double s_speed_upper_weight = 2.0;
    double s_speed_lower_weight = 1.0;
  };
  WeightConfig weight_maker_config;
};

struct CongestionDetectionConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    ReadItem<double>(json, heavy_density,
                     "heavy_density");
    ReadItem<double>(json, jam_speed, "jam_speed");
    ReadItem<double>(json, speed_deviation, "speed_deviation");
  }

  double heavy_density = 60.0;
  double jam_speed = 12.0;
  double speed_deviation = 1000.0;
};
}  // namespace planning
