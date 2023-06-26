#pragma once

#include <math.h>

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "frame.h"
#include "task_basic_types.h"

namespace planning {
class AdaptiveCruiseControl {
 public:
  AdaptiveCruiseControl(const EgoPlanningConfigBuilder *config_builder,
                        framework::Session *session);
  virtual ~AdaptiveCruiseControl() = default;

  void adaptive_cruise_control(LonDecisionInfo &lon_decision_information,
                               AdaptiveCruiseControlInfo &acc_info,
                               PlanningResult &ego_prediction_result);
  std::pair<double, double> calculate_max_acc(double ego_v);
  void acc_update_ds_refs(AdaptiveCruiseControlInfo &acc_info,
                          LonRefPath &lon_ref_path,
                          PlanningResult &ego_prediction_result,
                          PlanningInitPoint &planning_init_point);
  void acc_update_jerk_bound(AdaptiveCruiseControlInfo &acc_info,
                             LonRefPath &lon_ref_path,
                             PlanningResult &ego_prediction_result,
                             PlanningInitPoint &planning_init_point);
  bool recognize_model_speed_up(PlanningResult &ego_prediction_result);

 private:
  double s_rel_{-1.0};
  double t_cons_{0.9};
  double gain_s_{1.5};
  double gain_tg_{1.2};
  double v_map_{-1.0};
  double gain_v_{0.2};
  double dv_down_{0.1};
  double s_safe_{-1.0};
  double s_thld_{-1.0};
  double v_curv_{-1.0};
  double v_leadone_{0.0};
  double tg_cur_{1000.0};
  double v_max_able_{0.0};
  double v_min_able_{0.0};
  double a_max_accel_{0.0};
  double a_max_brake_{0.0};
  double navi_max_speed_{0.0};
  double s_rel_threshold_{3.0};
  double kMinCurvature_{0.0001};
  double navi_time_distance_{0.0};
  double obstacle_max_brake_{5.0};
  double kMaxCentrifugalAcceleration_{3.0};

  AdaptiveCruiseControlConfig config_{};
  // LongitudinalDeciderV3Config config_lon_{};
  framework::Session *session_ = nullptr;
};

}  // namespace planning