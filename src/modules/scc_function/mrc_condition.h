#pragma once

#include <math.h>

#include "session.h"
#include "ego_planning_config.h"
#include "planning_context.h"
#include "mrc_types.h"
#include "task_basic_types.h"

namespace planning {

class MrcCondition {
 public:
  MrcCondition(const EgoPlanningConfigBuilder *config_builder,
               framework::Session *session)
      : config_(config_builder->cast<MrcConditionConfig>()), session_(session) {
    init();
  }

  ~MrcCondition() = default;

  void update();
  void post_process();

  bool enable_mrc_condition() { return enable_mrc_condition_; }
  bool mrc_request_changed() { return request_type_change_; }
  const int mrc_request() { return static_cast<int>(mrc_request_type_); }
  const int mrc_execute_type() { return static_cast<int>(mrc_execute_type_); }
  const int execute_type_output() {
    return static_cast<int>(execute_type_output_);
  }
  const MrcBrakeType mrc_brake_type() { return mrc_brake_type_; }
  void mrc_engage_p_gear(double ego_v);
  const bool obtain_mrc_p_gear() { return mrc_engage_p_gear_; }
  void set_mrc_request(const MrcRequestType type) { mrc_request_type_ = type; }
  void clear_mrc_request() { mrc_request_type_ = MrcRequestType::NO_RQT; }
  void mrc_brake_execute(LonRefPath &lon_ref_path,
                         PlanningResult &ego_prediction_result,
                         PlanningInitPoint &planning_init_point);
  void update_inline_brake_by_obstacle(
      LonDecisionInfo &lon_decision_information,
      PlanningInitPoint &planning_init_point);
  std::pair<double, double> calculate_max_acc(double ego_v);
  const bool enable_mrc_pull_over() const {
    return mrc_execute_type_ == MrcExecuteType::PULL_OVER;
  }

 private:
  void init();
  void process();
  void clear_all();
  void reset_before_process();
  void set_mrc_config();
  void decide_mrc_type();
  void cal_merge_split_dis();
  void check_right_lane_type();
  void update_external_variables();
  bool check_pull_over_condition();
  void check_inline_brake_condition();
  void execute_pull_over();
  void decide_mrc_pull_over_acc();

  MrcRequestType mrc_request_type_ = MrcRequestType::NO_RQT;
  MrcRequestType pre_mrc_request_type_ = MrcRequestType::NO_RQT;
  MrcExecuteType mrc_execute_type_ = MrcExecuteType::NOT_EXECUTE;
  MrcExecuteType execute_type_output_ = MrcExecuteType::NOT_EXECUTE;
  MrcBrakeType mrc_brake_type_ = MrcBrakeType::NOT_BRAKE;
  MrcConditionConfig config_;
  framework::Session *session_ = nullptr;

  bool enable_mrc_condition_ = false;
  bool right_lane_has_emergency_ = false;
  bool pull_over_checked_ = false;
  bool request_type_change_ = false;
  bool pull_over_to_brake_ = false;
  bool has_decided_pull_over_acc_ = false;
  int clane_order_id_from_right_ = -1;
  bool dbw_status_ = false;
  double fcw_threshold_ = 2.5;
  int pull_over_wait_cnt_ = 0;
  int time_out_threshold_ = 100;
  double split_merge_dist_threshold_ = 150.0;
  double ramp_dist_threshold_ = 1000.0;
  double tunnel_dist_threshold_ = 1000.0;
  double construction_zone_dist_threshold_ = 1000.0;
  double a_brake_slow_ = -1.5;
  double a_brake_hard_ = -5.0;
  bool mrc_engage_p_gear_ = false;
  double a_brake_emergency_ = -3.0;
  double jerk_brake_slow_ = -0.2;
  double jerk_brake_hard_ = -0.4;
  double jerk_brake_emergency_ = -0.4;
  double v_pull_over_threshold_ = 10.0 / 3.6;
  double merge_split_point_distance_ = -10000.0;
  double distance_to_ramp_ = 10000.0;
  std::uint8_t display_state_ = 6;
  std::uint8_t pre_display_state_ = 6;
};

}  // namespace planning