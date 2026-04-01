#ifndef __LATERAL_MOTION_PLANNING_WEIGHT_H__
#define __LATERAL_MOTION_PLANNING_WEIGHT_H__

#include <cstddef>
#include <vector>
#include "ego_planning_config.h"
#include "lateral_motion_planner.pb.h"
#include "modules/tasks/task_interface/potential_dangerous_agent_decider_output.h"
#include "virtual_lane.h"

namespace pnc {
namespace lateral_planning {

enum class LateralMotionScene {
  LANE_KEEP = 0,
  AVOID,
  LANE_CHANGE,
  SPLIT,
  RAMP,
  LANE_BORROW
};

enum class EmergencyLevel { NONE = 0, P2, P1, P0 };

enum class LaneChangeStyle {
  STANDARD_LANE_CHANGE = 0,
  QUICKLY_LANE_CHANGE,
  EMERGENCY_LANE_CHANGE,
  LOW_SPEED_LANE_CHANGE
};

struct PathWeight {  // temp
  size_t point_num = 26;
  size_t proximal_index = 5;
  size_t remotely_index = 20;
  bool complete_follow = false;
  double dt = 0.2;
  double total_t = 5.0;
  std::vector<double> expected_acc;
  std::vector<double> acc_upper_bound;
  std::vector<double> acc_lower_bound;
  std::vector<double> jerk_upper_bound;
  std::vector<double> jerk_lower_bound;
  std::vector<double> q_ref_x;
  std::vector<double> q_ref_y;
  std::vector<double> q_ref_theta;
  std::vector<double> q_continuity;
  std::vector<double> q_front_ref_x;
  std::vector<double> q_front_ref_y;
  std::vector<double> q_virtual_ref_x;
  std::vector<double> q_virtual_ref_y;
  std::vector<double> q_virtual_ref_theta;
  std::vector<double> q_acc;
  std::vector<double> q_jerk;
  std::vector<double> q_acc_bound;
  std::vector<double> q_jerk_bound;
  std::vector<double> q_pos_first_soft_bound;
  std::vector<double> q_pos_soft_bound;
  std::vector<double> q_pos_hard_bound;
  std::unordered_map<size_t, double> time2soft_ratio = {
      {0, 1.3}, {1, 1.3}, {2, 1.2}, {3, 1.0}, {4, 0.8}};
  std::unordered_map<size_t, double> time2hard_ratio = {
      {0, 1.5}, {1, 1.5}, {2, 1.4}, {3, 1.2}, {4, 1.0}};

  void Init() {
    expected_acc.resize(point_num, 0.0);
    acc_upper_bound.resize(point_num, 3.0);
    acc_lower_bound.resize(point_num, -3.0);
    jerk_upper_bound.resize(point_num, 0.3);
    jerk_lower_bound.resize(point_num, -0.3);
    q_ref_x.resize(point_num, 0);
    q_ref_y.resize(point_num, 0);
    q_ref_theta.resize(point_num, 0);
    q_continuity.resize(point_num, 0);
    q_front_ref_x.resize(point_num, 0);
    q_front_ref_y.resize(point_num, 0);
    q_virtual_ref_x.resize(point_num, 0);
    q_virtual_ref_y.resize(point_num, 0);
    q_virtual_ref_theta.resize(point_num, 0);
    q_acc.resize(point_num, 0);
    q_jerk.resize(point_num, 0);
    q_acc_bound.resize(point_num, 0);
    q_jerk_bound.resize(point_num, 0);
    q_pos_first_soft_bound.resize(point_num, 0);
    q_pos_soft_bound.resize(point_num, 0);
    q_pos_hard_bound.resize(point_num, 0);
  }

  void Clear() {
    expected_acc.clear();
    acc_upper_bound.clear();
    acc_lower_bound.clear();
    jerk_upper_bound.clear();
    jerk_lower_bound.clear();
    q_ref_x.clear();
    q_ref_y.clear();
    q_ref_theta.clear();
    q_continuity.clear();
    q_front_ref_x.clear();
    q_front_ref_y.clear();
    q_virtual_ref_x.clear();
    q_virtual_ref_y.clear();
    q_virtual_ref_theta.clear();
    q_acc.clear();
    q_jerk.clear();
    q_acc_bound.clear();
    q_jerk_bound.clear();
    q_pos_first_soft_bound.clear();
    q_pos_soft_bound.clear();
    q_pos_hard_bound.clear();
  }

  void Reset() {
    std::fill(expected_acc.begin(), expected_acc.end(), 0);
    std::fill(acc_upper_bound.begin(), acc_upper_bound.end(), 3.0);
    std::fill(acc_lower_bound.begin(), acc_lower_bound.end(), -3.0);
    std::fill(jerk_upper_bound.begin(), jerk_upper_bound.end(), 0.2);
    std::fill(jerk_lower_bound.begin(), jerk_lower_bound.end(), -0.2);
    std::fill(q_ref_x.begin(), q_ref_x.end(), 0);
    std::fill(q_ref_y.begin(), q_ref_y.end(), 0);
    std::fill(q_ref_theta.begin(), q_ref_theta.end(), 0);
    std::fill(q_continuity.begin(), q_continuity.end(), 0);
    std::fill(q_acc.begin(), q_acc.end(), 0);
    std::fill(q_jerk.begin(), q_jerk.end(), 0);
    std::fill(q_acc_bound.begin(), q_acc_bound.end(), 0);
    std::fill(q_jerk_bound.begin(), q_jerk_bound.end(), 0);
    std::fill(q_pos_soft_bound.begin(), q_pos_soft_bound.end(), 0);
    std::fill(q_pos_hard_bound.begin(), q_pos_hard_bound.end(), 0);
  }
};

class BaseWeight {
 public:
  explicit BaseWeight(
      const planning::LateralMotionPlannerConfig &config);
  ~BaseWeight() = default;

  void Init();

  void CalculateInitInfo(
      const planning::common::LateralPlanningInput &planning_input);

  void CalculateLatAvoidDistance(
      const std::vector<std::pair<double, double>> &bounds);

  void CalculateLatAvoidBoundPriority(
      const std::vector<std::pair<double, double>> &second_soft_bounds,
      const std::vector<std::pair<double, double>> &hard_bounds,
      const std::vector<planning::WeightedBounds>& soft_bounds_vec,
      const std::vector<planning::WeightedBounds>& hard_bounds_vec,
      const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>>
          &second_soft_bounds_info,
      const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>>
          &hard_bounds_info);

  // void CalculateExpectedLatAccAndSteerAngle(
  //     double init_s, double ref_vel, double wheel_base,
  //     double steer_ratio, double curv_factor,
  //     const planning::CoarsePlanningInfo &coarse_planning_info,
  //     const std::shared_ptr<planning::ReferencePath> &reference_path,
  //     std::vector<double>& expected_steer_vec);

  void CalculateExpectedLatAccAndSteerAngle(
      double init_s, double ref_vel, double wheel_base, double steer_ratio,
      double curv_factor, const std::shared_ptr<planning::ReferencePath>& reference_path,
      std::vector<double> &expected_steer_vec);

  void CalculateLatAccAndSteerAngleByHistoryPath(
      const bool is_in_function, const bool is_no_replan,
      const double wheel_base, const double steer_ratio,
      const double curv_factor, const double init_x, const double init_y,
      std::vector<double> &history_steer_vec);

  void CalculateJerkBoundByLastJerk(
      const bool is_high_priority_back, const bool is_in_function,
      const double enter_lccnoa_time,
      const std::shared_ptr<planning::ReferencePath> &reference_path,
      const planning::common::LateralPlanningOutput &last_planning_output,
      planning::common::LateralPlanningInput &planning_input);

  void CalculateLastPathDistToRef(
      const std::shared_ptr<planning::ReferencePath> &reference_path,
      planning::common::LateralPlanningInput &planning_input);

  void ConstructVirtualRef(
      const double wheel_base, const double curv_factor,
      const std::shared_ptr<planning::ReferencePath> &reference_path,
      planning::common::LateralPlanningInput &planning_input,
      std::vector<double> &virtual_ref_x, std::vector<double> &virtual_ref_y,
      std::vector<double> &virtual_ref_theta);

  void SetLateralMotionWeight(
      const LateralMotionScene scene,
      planning::common::LateralPlanningInput &planning_input);

  void SetLateralMotionWeightForHPP(
      planning::common::LateralPlanningInput &planning_input);

  void SetLateralMotionWeightForRADS(
      planning::common::LateralPlanningInput &planning_input);

  void SetLateralMotionWeightForNSA(
      planning::common::LateralPlanningInput &planning_input);

  void SetContinuityWeightByLastPath(size_t valid_continuity_idx,
      planning::common::LateralPlanningInput &planning_input);

  void SetInitDisToRef(const double init_dis_to_ref) {
    init_dis_to_ref_ = init_dis_to_ref;
  }

  void SetInitRefThetaError(const double init_ref_theta_error) {
    init_ref_theta_error_ = init_ref_theta_error;
  }

  void SetEgoVel(const double ego_vel) { ego_vel_ = ego_vel; }

  void SetEgoL(const double ego_l) { ego_l_ = ego_l; }

  void SetRefVel(const double ref_vel) { ref_vel_ = ref_vel; }

  void SetInitS(const double init_s) { init_s_ = init_s; }

  void SetInitL(const double init_l) { init_l_ = init_l; }

  void SetLateralOffset(const double lat_offset) { lat_offset_ = lat_offset; }

  void SetMaxAcc(const double max_acc) { max_acc_ = max_acc; }

  void SetMaxJerk(const double max_jerk) { max_jerk_ = max_jerk; }

  void SetMaxJerkLC(const double max_jerk_lc) { max_jerk_lc_ = max_jerk_lc; }

  void SetLCBackFlag(const bool is_lane_change_back) {
    is_lane_change_back_ = is_lane_change_back;
  }

  void SetLCHoldFlag(const bool is_lane_change_hold) {
    is_lane_change_hold_ = is_lane_change_hold;
  }

  void SetIsInIntersection(const bool is_in_intersection) {
    is_in_intersection_ = is_in_intersection;
  }

  void SetIsSearchSuccess(const bool is_search_success) {
    is_search_success_ = is_search_success;
  }

  void SetIsUseSpatioPlannerResult(const bool is_use_spatio_planner_result) {
    is_use_spatio_planner_result_ = is_use_spatio_planner_result;
  }

  void SetMotionPlanConcernedEndIndex(
      const bool origin_complete_follow, const bool is_divide_lane_into_two,
      const std::shared_ptr<planning::ReferencePath> &reference_path,
      planning::common::LateralPlanningInput &planning_input);

  void SetLaneChangeStyle(const LaneChangeStyle lc_style) {
    lc_style_ = lc_style;
  }

  void SetIsBoundAvoid(const bool is_bound_avoid) {
    is_bound_avoid_ = is_bound_avoid;
  }

  void SetExpectedAvoidJerk(const double expected_avoid_jerk) {
    expected_avoid_jerk_ = expected_avoid_jerk;
  }

  void SetRiskLevel(const planning::RiskLevel risk_level) {
    risk_level_ = risk_level;
  }

  double GetInitDisToRef() const { return init_dis_to_ref_; }

  double GetConcernedStartQJerk() const { return concerned_start_q_jerk_; }

  double GetConcernedEndRatioForXY() const { return end_ratio_for_qrefxy_; }

  double GetConcernedEndRatioForTheta() const {
    return end_ratio_for_qreftheta_;
  }

  double GetConcernedEndRatioForJerk() const { return end_ratio_for_qjerk_; }

  const PathWeight &GetPathWeights() const { return weight_; }

  PathWeight &MutablePathWeights() { return weight_; }

  const EmergencyLevel &GetEmergencyLevel() const { return emergency_level_; }

  const std::vector<planning::planning_math::PathPoint> &GetVirtualRef() const {
    return virtual_ref_;
  }

  const size_t GetAvoidEndIndex() const { return avoid_end_index_; }

  const double GetAvoidDist() const { return avoid_dist_; }

  const double GetAvoidTime() const { return avoid_time_; }

  const LaneChangeStyle GetLaneChangeStyle() const { return lc_style_; }

  void SetLowChangeCoolDown(const bool is_enter_low_speed_lane_change_cooldown) {
    is_enter_low_speed_lane_change_cooldown_ = is_enter_low_speed_lane_change_cooldown;
  }

  void SetIsEmergencyAvoid(const bool is_emergency_avoid) {
    is_emergency_avoid_ = is_emergency_avoid;
  }

  void SetLCRemainTime(const double lc_remain_time) {
    lc_remain_time_ = lc_remain_time;
  }

 private:
  void SetAccJerkBoundAndWeight(
      planning::common::LateralPlanningInput &planning_input);

  void SetMinJerkWeightByVel(
      planning::common::LateralPlanningInput &planning_input);

  void SetWeightProtectionForLargePosDiff(
      planning::common::LateralPlanningInput &planning_input);

  void MakeDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeDynamicWeight_Rads(
      planning::common::LateralPlanningInput &planning_input);

  void MakeLateralOffsetAvoidDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeRampDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeLaneChangeDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeLaneChangeBackDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeSplitDynamicWeight(
      const bool is_divide_lane_into_two,
      planning::common::LateralPlanningInput &planning_input);

  void MakeDynamicPosBoundWeight(
      planning::common::LateralPlanningInput &planning_input);

 private:
  planning::LateralMotionPlannerConfig config_;
  LateralMotionScene lateral_motion_scene_;
  PathWeight weight_;
  EmergencyLevel emergency_level_;
  LaneChangeStyle lc_style_;
  planning::RiskLevel risk_level_;
  double lat_offset_;
  double avoid_dist_;
  double avoid_time_;
  double init_dis_to_ref_;
  double init_ref_theta_error_;
  double concerned_start_q_jerk_;
  double ego_vel_;
  double ego_l_;
  double ref_vel_;
  double init_s_;
  double init_l_;
  double end_ratio_for_qrefxy_;
  double end_ratio_for_qreftheta_;
  double end_ratio_for_qjerk_;
  double max_acc_;
  double max_jerk_;
  double max_jerk_lc_;
  double max_jerk_low_speed_;
  double history_average_acc_;
  double last_expected_average_acc_;
  double expected_average_acc_;
  double expected_max_acc_;
  double expected_min_acc_;
  double expected_max_jerk_;
  double expected_min_jerk_;
  double min_road_radius_;
  double target_road_radius_;
  double min_q_jerk_;
  double expected_avoid_jerk_;
  double last_path_max_dist2ref_;
  double last_jerk_bound_limit_;
  double last_max_omega_;
  double last_lc_end_ratio_for_qrefxy_buffer_;
  double lc_remain_time_;
  size_t last_remotely_index_;
  size_t avoid_end_index_;
  bool is_lane_change_hold_;
  bool is_lane_change_back_;
  bool is_in_intersection_;
  bool is_search_success_;
  bool is_s_bend_;
  bool is_use_spatio_planner_result_;
  bool is_sharp_turn_;
  bool is_bound_avoid_;
  std::vector<double> soft_bound_qratio_vec_;
  std::vector<double> hard_bound_qratio_vec_;
  std::vector<double> curvature_radius_vec_;
  std::vector<planning::planning_math::PathPoint> virtual_ref_;
  std::deque<std::pair<double, double>> history_path_points_;
  pnc::mathlib::spline soft_lbound_l_s_spline_;
  pnc::mathlib::spline soft_ubound_l_s_spline_;
  pnc::mathlib::spline hard_lbound_l_s_spline_;
  pnc::mathlib::spline hard_ubound_l_s_spline_;
  bool is_enter_low_speed_lane_change_cooldown_;
  bool is_emergency_avoid_;
};

}  // namespace lateral_planning
}  // namespace pnc
#endif