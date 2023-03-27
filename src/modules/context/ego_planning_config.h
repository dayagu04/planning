#pragma once

#include <iostream>
#include <set>
#include <vector>

#include "common/log.h"
#include "thirdparty/nlohmann_json/nlohmann_json.hpp"

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
    LOG_DEBUG("created %s with %s", typeid(T).name(), name());
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
  }
  bool enable_raw_ego_prediction = false;
  bool enable_dagger = false;
  bool use_ego_prediction_model_in_planning = false;
};

struct EgoPlanningCandidateConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    reference_point_velocity =
        read_json_key<double>(json, "reference_point_velocity");
  }

  double reference_point_velocity = 22.22;  // m/s
};

struct ObstacleDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct LateralDeciderConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct LateralOptimizerConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};
using LateralOptimizerV2Config = LateralOptimizerConfig;

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
  int lon_num_step = 20;
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

struct LongitudinalOptimizerV3Config : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
};

struct ResultTrajectoryGeneratorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
  }
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

struct EgoPlanningEvaluatorConfig : public EgoPlanningConfig {
  void init(const Json &json) override {
    EgoPlanningConfig::init(json);
    /* read config from json */
    enable_evaluator_check = read_json_key<bool>(
        json, "enable_evaluator_check",
        enable_evaluator_check);  // evaluator collsion check
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
}  // namespace planning
