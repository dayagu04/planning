#pragma once

#include <limits>
#include <vector>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "config/vehicle_param.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "reference_path.h"
#include "sample_poly_curve.h"
#include "sample_space_base.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class SamplePolySpeedAdjustDecider : public Task {
 public:
  SamplePolySpeedAdjustDecider();
  SamplePolySpeedAdjustDecider(const EgoPlanningConfigBuilder* config_builder,
                               framework::Session* session);
  virtual ~SamplePolySpeedAdjustDecider() = default;

  bool Execute() override;

 public:
  bool ProcessEnvInfos();
  void CalcTargetLaneObjsFlowVel();
  void CalcTargetLaneVehDensity();

  bool SamplePolys();
  bool Evaluate();
  bool BestTrajCheck();
  void LogDebugInfo(const double sample_cost_time,
                    const double evaluate_cost_time,
                    const double all_cost_time);
  bool CheckInitVelTraj();
  double CalcHeadwayDistance(const double& headway_v, const double ego_v,
                             const std::vector<double>& t_gap_ego_v_bp,
                             const std::vector<double>& t_gap_ego_v);
  void StitchLastBestPoly();

  void RunSampleSceneStateMachine();
  void SetPurseFlowVelSceneWeight();
  void SetNormalSceneWeight();
  void SetDeclerationSceneWeight();

  bool IsInDeceleartionScene();
  void ClearStitchedPolyPtr();

 private:
  SamplePolySpeedAdjustDeciderConfig config_;

  AgentInfo leading_veh_;

  double ego_v_;
  double ego_a_;
  double ego_s_;
  std::pair<double, double> ego_cart_point_;
  std::pair<double, double> last_ego_cart_point_;

  double delta_t_ = 0.1;
  double delta_v_ = 0.5;

  double weight_match_gap_vel_;
  double weight_match_gap_s_;
  double weight_follow_vel_;
  double weight_stop_line_;
  double weight_leading_safe_s_;
  double weight_leading_safe_v_;
  double weight_vel_variable_;
  double weight_gap_avaliable_;

  std::pair<double, double> speed_adjust_range_;  // first: upper, second: lower

  STSampleSpaceBase st_sample_space_base_;
  std::vector<AgentInfo> agent_info_;

  double v_suggestted_{25.0};
  double target_lane_objs_flow_vel_{25.0};
  double evaulation_t_{5.0};

  SampleQuarticPolynomialCurve* min_cost_traj_ptr_;

  SampleQuarticPolynomialCurve last_min_cost_traj_;
  std::shared_ptr<SampleQuarticPolynomialCurve>
      stitched_last_best_quartic_poly_ptr_;

  std::vector<std::vector<SampleQuarticPolynomialCurve>> sample_trajs_;

  int count_wait_state_{0};
  int count_normal_to_hover_state_{0};
  int count_hover_to_normal_state_{0};
  int lane_change_request_ = 0;
  int lane_change_source_ = 0;
  const std::vector<double> t_gap_ego_v_bp_{5.0, 15.0, 30.0};
  const std::vector<double> t_gap_ego_v_{1.35, 1.55, 2.0};

  SampleSpeedStatus sample_status_ = SampleSpeedStatus::OK;
  SampleScene sample_scene_ = SampleScene::NormalSampleScene;
  TrafficDensityStatus traffic_density_status_ =
      TrafficDensityStatus::LineClear;
  double traffic_density_ = 0.0;

  bool boundary_merge_point_valid_ = false;
  bool is_nearing_ramp_ = false;
  bool is_in_merge_region_ = false;

  double stop_line_distance_ = kMaxVelVariableValueInverse;
  double merge_stop_line_distance_ = kMaxMergeDistance;
  double distance_to_road_merge_ = kMaxMergeDistance;
  double distance_to_road_split_ = kMaxMergeDistance;
  double distance_to_ramp_ = kMaxMergeDistance;

 public:  // set private values
  void set_weight_match_gap_vel(const double weight) {
    weight_match_gap_vel_ = weight;
  }
  double weight_match_gap_vel() const { return weight_match_gap_vel_; };

  void set_weight_match_gap_s(const double weight) {
    weight_match_gap_s_ = weight;
  }
  double weight_match_gap_s() const { return weight_match_gap_s_; };

  void set_weight_follow_vel(const double weight) {
    weight_follow_vel_ = weight;
  }
  double weight_follow_vel() const { return weight_follow_vel_; };

  void set_weight_stop_line(const double weight) { weight_stop_line_ = weight; }
  double weight_stop_line() const { return weight_stop_line_; };

  void set_weight_leading_safe_s(const double weight) {
    weight_leading_safe_s_ = weight;
  }
  double weight_leading_safe_s() const { return weight_leading_safe_s_; };

  void set_weight_leading_safe_v(const double weight) {
    weight_leading_safe_v_ = weight;
  }
  double weight_leading_safe_v() const { return weight_leading_safe_v_; };

  void set_weight_vel_variable(const double weight) {
    weight_vel_variable_ = weight;
  }
  double weight_vel_variable() const { return weight_vel_variable_; };

  void set_weight_gap_avaliable(const double weight) {
    weight_gap_avaliable_ = weight;
  }
  double weight_gap_avaliable() const { return weight_gap_avaliable_; };
  void set_ego_v(const double ego_v) { ego_v_ = ego_v; }
  double ego_v() const { return ego_v_; }

  void set_ego_a(const double ego_a) { ego_a_ = ego_a; }
  double ego_a() const { return ego_a_; }

  void set_ego_s(const double ego_s) { ego_s_ = ego_s; }
  double ego_s() const { return ego_s_; }

  void set_delta_t(const double delta_t) { delta_t_ = delta_t; }
  double delta_t() const { return delta_t_; }

  void set_delta_v(const double delta_v) { delta_v_ = delta_v; }
  double delta_v() const { return delta_v_; }

  void set_speed_adjust_range(const std::pair<double, double>& range) {
    speed_adjust_range_ = range;
  }
  const std::pair<double, double>& speed_adjust_range() const {
    return speed_adjust_range_;
  }

  void set_v_suggestted(const double v) { v_suggestted_ = v; }
  double v_suggestted() const { return v_suggestted_; }

  void set_target_lane_objs_flow_vel(const double vel) {
    target_lane_objs_flow_vel_ = vel;
  }
  double target_lane_objs_flow_vel() const {
    return target_lane_objs_flow_vel_;
  }

  void set_evaulation_t(const double t) { evaulation_t_ = t; }
  double evaulation_t() const { return evaulation_t_; }

  void set_count_wait_state(const int count) { count_wait_state_ = count; }
  int count_wait_state() const { return count_wait_state_; }

  void set_count_normal_to_hover_state(const int count) {
    count_normal_to_hover_state_ = count;
  }
  int count_normal_to_hover_state() const {
    return count_normal_to_hover_state_;
  }

  void set_count_hover_to_normal_state(const int count) {
    count_hover_to_normal_state_ = count;
  }
  int count_hover_to_normal_state() const {
    return count_hover_to_normal_state_;
  }

  void set_lane_change_request(const int request) {
    lane_change_request_ = request;
  }
  int lane_change_request() const { return lane_change_request_; }

  void set_lane_change_source(const int source) {
    lane_change_source_ = source;
  }
  int lane_change_source() const { return lane_change_source_; }

  void set_traffic_density(const double density) { traffic_density_ = density; }
  double traffic_density() const { return traffic_density_; }

  void set_stop_line_distance(const double distance) {
    stop_line_distance_ = distance;
  }
  double stop_line_distance() const { return stop_line_distance_; }

  void set_merge_stop_line_distance(const double distance) {
    merge_stop_line_distance_ = distance;
  }
  double merge_stop_line_distance() const { return merge_stop_line_distance_; }

  void set_distance_to_road_merge(const double distance) {
    distance_to_road_merge_ = distance;
  }
  double distance_to_road_merge() const { return distance_to_road_merge_; }

  void set_distance_to_road_split(const double distance) {
    distance_to_road_split_ = distance;
  }
  double distance_to_road_split() const { return distance_to_road_split_; }

  void set_distance_to_ramp(const double distance) {
    distance_to_ramp_ = distance;
  }
  double distance_to_ramp() const { return distance_to_ramp_; }

  void set_boundary_merge_point_valid(const bool is_valid) {
    boundary_merge_point_valid_ = is_valid;
  }
  bool boundary_merge_point_valid() const {
    return boundary_merge_point_valid_;
  }

  void set_is_nearing_ramp(const bool is_near) { is_nearing_ramp_ = is_near; }
  bool is_nearing_ramp() const { return is_nearing_ramp_; }

  void set_is_in_merge_region(const bool is_in_region) {
    is_in_merge_region_ = is_in_region;
  }
  bool is_in_merge_region() const { return is_in_merge_region_; }

  void set_min_cost_traj_ptr(SampleQuarticPolynomialCurve* ptr) {
    min_cost_traj_ptr_ = ptr;
  }
  const SampleQuarticPolynomialCurve* min_cost_traj_ptr() const {
    return min_cost_traj_ptr_;
  }

  void set_last_min_cost_traj(const SampleQuarticPolynomialCurve& traj) {
    last_min_cost_traj_ = traj;
  }
  const SampleQuarticPolynomialCurve& last_min_cost_traj() const {
    return last_min_cost_traj_;
  }
  void set_stitched_last_best_quartic_poly_ptr(
      const std::shared_ptr<SampleQuarticPolynomialCurve>& ptr) {
    stitched_last_best_quartic_poly_ptr_ = ptr;
  }
  const std::shared_ptr<SampleQuarticPolynomialCurve>
  stitched_last_best_quartic_poly_ptr() const {
    return stitched_last_best_quartic_poly_ptr_;
  }
  SampleSpeedStatus sample_status() const { return sample_status_; }

  void set_sample_status(SampleSpeedStatus status) { sample_status_ = status; }
  SampleScene sample_scene() const { return sample_scene_; }

  void set_sample_scene(SampleScene scene) { sample_scene_ = scene; }

  TrafficDensityStatus traffic_density_status() const {
    return traffic_density_status_;
  }

  void set_traffic_density_status(TrafficDensityStatus status) {
    traffic_density_status_ = status;
  }

  const STSampleSpaceBase& st_sample_space_base() const {
    return st_sample_space_base_;
  }

  STSampleSpaceBase& mutable_st_sample_space_base() {
    return st_sample_space_base_;
  }

  const std::vector<AgentInfo>& agent_info() const { return agent_info_; }

  std::vector<AgentInfo>& mutable_agent_info() { return agent_info_; }

  const std::vector<std::vector<SampleQuarticPolynomialCurve>>& sample_trajs()
      const {
    return sample_trajs_;
  }
  
  std::vector<std::vector<SampleQuarticPolynomialCurve>>&
  mutable_sample_trajs() {
    return sample_trajs_;
  }

  const AgentInfo& leading_veh() const { return leading_veh_; }
  AgentInfo& mutable_leading_veh() { return leading_veh_; }
};
};  // namespace planning

// namespace planning