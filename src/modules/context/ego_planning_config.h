#pragma once

#include <iostream>
#include <set>
#include <vector>

#include "log.h"
#include "nlohmann_json.hpp"

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
  if (json.find(key) != json.end()) {
    for (size_t i = 0; i < json[key].size(); i++) {
      vec.push_back(json[key][i]);
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
    enable_raw_ego_prediction =
        read_json_key<bool>(json, "enable_raw_ego_prediction");
    enable_dagger = read_json_key<bool>(json, "enable_dagger");
    use_ego_prediction_model_in_planning = read_json_key<bool>(
        json, "use_ego_prediction_model_in_planning", false);
    enable_NOA = read_json_key<bool>(json, "enable_NOA");
  }
  bool enable_raw_ego_prediction = false;
  bool enable_dagger = false;
  bool use_ego_prediction_model_in_planning = false;
  bool enable_NOA = false;
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
  }
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

struct GeneralLateralDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
  double desired_vel = 11.11;                    // KPH_40;
  double l_care_width = 10.;                     // TBD: more beautiful
  double care_obj_lat_distance_threshold = 30.;  // TBD: more beautiful
  double care_obj_lon_distance_threshold = 60.;  // TBD: more beautiful
  double dynamic_obj_safe_buffer = 0.3;          //
  double min_obstacle_avoid_distance = 0.2;      // check it
  double lateral_bound_converge_speed = 1.0;
  double kPhysicalBoundWeight = 10.;
  double kHardBoundWeight = -1.;
  double dynamic_bound_slack_coefficient = 1.;
  double buffer2border = 0.15;
  double buffer2lane = 0.3;
  double l_offset_limit = 0.1;
  double min_gain_vel = 1.0;
  double lon_rear_car_filter_buffer = 10.;
  double refine_lat_ref_threshold = 0.5;
  double delta_t = 0.2;
  double sample_step = 0.2;
  double sample_forward_distance = 2.0;
};
struct LateralMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    q_continuity = read_json_key<double>(json, "ilqr_q_continuity");
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
};

struct LongitudinalMotionPlannerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
  double q_ref_pos = 0.06;
  double q_ref_vel = 1.0;
  double q_acc = 1.0;
  double q_jerk = 2.0;

  double q_pos_bound = 1000.0;
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
  }
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
    t_cons = read_json_keys<double>(
        json, std::vector<std::string>{"acc_control", "t_cons"});
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

  double t_cons = 0.9;
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
    // 目前实时无法拿到自己的配置文件
    // preview_x = read_json_key<double>(json, "preview_x");
    // dis_zero_speed = read_json_key<double>(json, "dis_zero_speed");
    // dis_zero_speed_accident =
    //     read_json_key<double>(json, "dis_zero_speed_accident");
    // ttc_brake_hysteresis = read_json_key<double>(json,
    // "ttc_brake_hysteresis");
  }
  double preview_x = 80.0;
  double dis_zero_speed = 3.5;
  double dis_zero_speed_accident = 6;
  double ttc_brake_hysteresis = 0.3;
};

struct ResultTrajectoryGeneratorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
  double planning_result_delta_time = 0.025;
  double min_path_length = 10.0;
  bool is_pwj_planning = false;
};

struct TrafficLightDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
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
  }
  double frenet_obstacle_range_s_min = -30.0;
  double frenet_obstacle_range_s_max = 100.0;
  double frenet_obstacle_range_l_min = -50.0;
  double frenet_obstacle_range_l_max = 50.0;
  bool enable_bbox_mode = true;
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

struct EgoPlanningTaskPipelineVisionOnlyConfig
    : public EgoPlanningTaskPipelineConfig {
  void init(const Json &json) override {
    EgoPlanningTaskPipelineConfig::init(json);
    /* read config from json */
    pipeline_version =
        read_json_key<std::string>(json, "pipeline_version_vision_only", "v1");
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
}  // namespace planning
