#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "dp_base.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "math/box2d.h"
#include "reference_path.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "task.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "trajectory_cost.h"
#include "utils/cartesian_coordinate_system.h"
#include "virtual_lane.h"
namespace planning {
class DPRoadGraph : public Task {
  /**
   * an private inner struct for the dp algorithm
   */
  struct DPRoadGraphNode {
   public:
    DPRoadGraphNode() = default;
    DPRoadGraphNode(const SLPoint sl_point, const DPRoadGraphNode* node_prev)
        : sl_point(sl_point), min_cost_prev_node_(node_prev) {}
    DPRoadGraphNode(const SLPoint sl_point, const DPRoadGraphNode* node_prev,
                    const ComparableCost& cost)
        : sl_point(sl_point), min_cost_prev_node_(node_prev), min_cost_(cost) {}
    SLPoint sl_point;
    const DPRoadGraphNode* min_cost_prev_node_ = nullptr;
    planning_math::QuinticPolynomialCurve1d min_cost_curve_;
    ComparableCost min_cost_{true, true,
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
    void UpdateCost(const ComparableCost& cost,
                    const DPRoadGraphNode* prev_node,
                    const planning_math::QuinticPolynomialCurve1d& curve) {
      if (cost <= min_cost_) {
        min_cost_ = cost;
        min_cost_prev_node_ = prev_node;
        min_cost_curve_ = curve;
      }
    }
    void Reset() {
      min_cost_prev_node_ = nullptr;
      min_cost_ =
          ComparableCost{true, true, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity()};
      min_cost_curve_ = planning_math::QuinticPolynomialCurve1d();
    }
  };

 public:
  DPRoadGraph() {
    config_ = DPRoadGraphConfig();
    sampled_points_ = std::vector<std::vector<SLPoint>>();
  }
  virtual ~DPRoadGraph() = default;
  DPRoadGraph(const EgoPlanningConfigBuilder* config_builder,
              framework::Session* session)
      : Task(config_builder, session) {
    config_ = config_builder->cast<DPRoadGraphConfig>();
  };

 public:
 private:
  DPRoadGraphConfig config_;
  std::shared_ptr<VirtualLane> current_lane_ptr_ = nullptr;
  std::shared_ptr<ReferencePath> current_reference_path_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ptr_ = nullptr;
  // ego
  FrenetEgoState ego_frenet_state_;
  CartesianState ego_cartes_state;
  FrenetBoundary ego_frenet_boundary_;
  SLPoint init_sl_point_;

  double ego_s_;
  double ego_l_;
  double ego_v_;
  double v_cruise_ = 0;
  double vehicle_width_;
  double vehicle_length_;
  // env obs
  std::vector<StaticObstacleInfo> obstacles_info_;
  std::vector<planning_math::Box2d> flatted_dynamic_obstacles_box_;
  std::vector<planning_math::Box2d> static_obstacles_box_;
  double sample_left_boundary_ = 0;
  double sample_right_boundary_ = 0;  // default boundary
  // sample output
  std::vector<std::vector<SLPoint>> sampled_points_;
  std::vector<planning_math::PathPoint> refined_paths_;  // store path points
  std::vector<planning_math::PathPoint> last_frame_paths_;
  std::vector<DPRoadGraphNode> min_cost_path_;
  pnc::mathlib::spline x_s_spline_;
  pnc::mathlib::spline y_s_spline_;  // last frame spline  for dp cost xy
  CarReferenceInfo ref_path_curve_;
  bool spline_sucess_ = false;

  // sample used
  double total_length_;
  double s_range_;
  double l_range_;
  double theta_max_;
  // dp result
  std::vector<SLPoint> dp_selected_points_;
  // dp param
  // cost coeffi
  double coeff_l_cost_ = 6.0;
  double coeff_dl_cost_ = 0.;
  double coeff_ddl_cost_ = 0.;
  double path_resolution_ = 2.0;
  double coeff_end_l_cost_ = 0.;
  double coeff_collision_cost_ = 1.0;
  double collision_distance_ = 0.8;
  double coeff_stitch_cost_ = 5.0;
  // worked values
  // "coeff_l_cost":6e6,
  // "coeff_dl_cost":8e3,
  // "coeff_ddl_cost ":5e3,
  // "coeff_end_l_cost":1e6,
  // "coeff_collision_cost":1e6,
  // "collision_distance":0.8,
  // "path_resolution":2.0,

  double dp_cost_time_ = 0.0;
  uint64 iter_num_ = 0;
  double path_cost_time_ = 0;
  double safety_cost_time_ = 0;
  double stitch_cost_time_ = 0;
  double dp_search_cost_time_ = 0.0;

 public:
  bool Execute() override;
  bool SampleSLPoints(std::vector<std::vector<SLPoint>>& sample_points);
  bool SetSampleParams(LaneBorrowStatus lane_borrow_status);
  bool SetDPCostParams(LaneBorrowStatus lane_borrow_status);
  bool SampleLanes(LaneBorrowDeciderOutput* lane_borrow_decider_output);
  bool ProcessEnvInfos();
  bool GenerateMinCostPath(std::vector<DPRoadGraphNode>* min_cost_path);
  bool DPSearchPath(const LaneBorrowStatus lane_borrow_status);
  bool FinedReferencePath();
  bool CartSpline(LaneBorrowDeciderOutput* lane_borrow_decider_output);
  bool LastFramePath();
  void LogDebugInfo();
  void ClearDPInfo();
  std::shared_ptr<planning_math::KDPath> ConstructLaneBorrowKDPath(
      const std::vector<double>& x_vec, const std::vector<double>& y_vec);

  // inputs methods
  // Setters
  void set_ego_s(double s) { ego_s_ = s; }
  void set_ego_l(double l) { ego_l_ = l; }
  void set_ego_v(double v) { ego_v_ = v; }
  void set_v_cruise(double v) { v_cruise_ = v; }
  void set_vehicle_width(double width) { vehicle_width_ = width; }
  void set_vehicle_length(double length) { vehicle_length_ = length; }

  void set_coeff_l_cost(double coeff) { coeff_l_cost_ = coeff; }
  void set_coeff_dl_cost(double coeff) { coeff_dl_cost_ = coeff; }
  void set_coeff_ddl_cost(double coeff) { coeff_ddl_cost_ = coeff; }
  void set_path_resolution(double res) { path_resolution_ = res; }
  void set_coeff_end_l_cost(double coeff) { coeff_end_l_cost_ = coeff; }
  void set_coeff_collision_cost(double coeff) { coeff_collision_cost_ = coeff; }
  void set_collision_distance(double dist) { collision_distance_ = dist; }
  void set_left_boundary(double left_boundary) {
    sample_left_boundary_ = left_boundary;
  }
  void set_right_boundary(double right_boundary) {
    sample_right_boundary_ = right_boundary;
  }
  // Getters
  double ego_s() const { return ego_s_; }
  double ego_l() const { return ego_l_; }
  double ego_v() const { return ego_v_; }
  double v_cruise() const { return v_cruise_; }
  double vehicle_width() const { return vehicle_width_; }
  double vehicle_length() const { return vehicle_length_; }

  double coeff_l_cost() const { return coeff_l_cost_; }
  double coeff_dl_cost() const { return coeff_dl_cost_; }
  double coeff_ddl_cost() const { return coeff_ddl_cost_; }
  double path_resolution() const { return path_resolution_; }
  double coeff_end_l_cost() const { return coeff_end_l_cost_; }
  double coeff_collision_cost() const { return coeff_collision_cost_; }
  double collision_distance() const { return collision_distance_; }

  const std::vector<StaticObstacleInfo>& obstacles_info() const {
    return obstacles_info_;
  }
  std::vector<StaticObstacleInfo>& mutable_obstacles_info() {
    return obstacles_info_;
  }

  // output Getters
  const std::vector<std::vector<SLPoint>>& sampled_points() const {
    return sampled_points_;
  }
  std::vector<std::vector<SLPoint>>& mutable_sampled_points() {
    return sampled_points_;
  }
  double dp_cost_time() { return dp_cost_time_; }
  double dp_search_cost_time() { return dp_search_cost_time_; }

  double total_length() const { return total_length_; }
  double s_range() const { return s_range_; }
  double l_range() const { return l_range_; }
  void set_total_length(double total_length) { total_length_ = total_length; }
  void set_s_range(double s_range) { s_range_ = s_range; }
  void set_l_range(double l_range) { l_range_ = l_range; }

  const std::vector<planning_math::PathPoint>& refined_paths() const {
    return refined_paths_;
  }
  std::vector<planning_math::PathPoint>& muteble_refined_paths() {
    return refined_paths_;
  }
  const std::vector<DPRoadGraphNode>& min_cost_path() const {
    return min_cost_path_;
  }
  std::vector<DPRoadGraphNode>& muteble_min_cost_path() {
    return min_cost_path_;
  }
  const std::vector<SLPoint>& dp_selected_points() const {
    return dp_selected_points_;
  }
  std::vector<SLPoint>& mutable_dp_selected_points() {
    return dp_selected_points_;
  }
  // dp search Params
};
};  // namespace planning