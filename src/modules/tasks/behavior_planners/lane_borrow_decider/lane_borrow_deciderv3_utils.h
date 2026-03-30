#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "math/box2d.h"
#include "reference_path.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tasks/behavior_planners/dp_path_decider/dp_base.h"
#include "tasks/task.h"
#include "utils/cartesian_coordinate_system.h"
#include "virtual_lane.h"
namespace planning {
namespace lane_borrow_deciderv3_utils {
class LaneBorrowDeciderV3Utils : public planning::Task {
 public:
  LaneBorrowDeciderV3Utils() { config_ = DPRoadGraphConfig(); }
  virtual ~LaneBorrowDeciderV3Utils() = default;
  LaneBorrowDeciderV3Utils(const EgoPlanningConfigBuilder* config_builder,
                           framework::Session* session)
      : Task(config_builder, session) {
    config_ = config_builder->cast<DPRoadGraphConfig>();
  };

 public:
 private:
  DPRoadGraphConfig config_;
  std::shared_ptr<VirtualLane> current_lane_ptr_ = nullptr;
  std::shared_ptr<ReferencePath> current_reference_path_ptr_ = nullptr;
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
  double sample_left_boundary_ = 0;
  double sample_right_boundary_ = 0;                     // default boundary
  std::vector<planning_math::PathPoint> refined_paths_;  // store path points
  std::vector<planning_math::PathPoint> last_frame_paths_;
  pnc::mathlib::spline x_s_spline_;
  pnc::mathlib::spline y_s_spline_;  // last frame spline  for dp cost xy
  CarReferenceInfo ref_path_curve_;
  bool spline_sucess_ = false;
  // 借道返回：上一帧回归终点的绝对 xy 坐标，跨帧投影回当前 s 坐标系
  double last_back_origin_merge_end_x_ = 0.0;
  double last_back_origin_merge_end_y_ = 0.0;
  bool last_back_origin_valid_ = false;
  LaneBorrowStatus lane_borrow_status_;
  std::vector<int> static_blocked_obj_id_vec_;
  std::vector<int> last_static_blocked_obj_id_vec_;
  // Hysteresis cache for rule-path-based lateral decisions to avoid frame
  // jitter. pair<stable_decision, consecutive_switch_count>
  std::unordered_map<int32_t, std::pair<LatObstacleDecisionType, int>>
      rule_lat_decision_hysteresis_map_{};
  // Per-obstacle hysteresis on "need virtual obs" decision based on clearance.
  // key: obs_id, value: last frame's need_virtual_obs flag.
  std::unordered_map<int, bool> need_virtual_obs_hysteresis_map_{};
  // Track each obstacle's static/dynamic attribute to skip stabilization on
  // change.
  std::unordered_map<int, bool> last_obs_static_map_{};

 private:
  void AdjustRefinedPathsByNonBorrowObstacles(
      const LaneBorrowDeciderOutput& lane_borrow_output,
      const std::vector<std::shared_ptr<FrenetObstacle>>& obstacles,
      const std::function<planning_math::Box2d(const Obstacle*, double)>&
          predict_box,
      std::unordered_map<uint32_t, LatObstacleDecisionType>&
          lat_obstacle_decision,
      const double lon_min_s, const double lon_max_s,
      std::shared_ptr<FrenetObstacle>& nearest_static_non_borrow_obstacle,
      std::shared_ptr<FrenetObstacle>& nearest_dynamic_non_borrow_obstacle);

 public:
  bool Execute();
  bool ProcessEnvInfos(const LaneBorrowDeciderOutput* lane_borrow_output,
                       const LaneBorrowStatus lane_borrow_status,
                       const std::vector<int>& static_blocked_obj_id_vec);
  bool GenerateOriginRulePath(
      const std::unordered_map<int, std::pair<BorrowDirection, int>>&
          obs_direction_map);
  bool CartSpline(LaneBorrowDeciderOutput* lane_borrow_decider_output);
  void UpdateObstacleLateralDecisionBaseRulePath(
      const LaneBorrowDeciderOutput& lane_borrow_output,
      std::shared_ptr<FrenetObstacle>& nearest_static_non_borrow_obstacle,
      std::shared_ptr<FrenetObstacle>& nearest_dynamic_non_borrow_obstacle,
      std::unordered_map<int32_t, LatObstacleDecisionType>*
          pending_lat_decisions);
  std::shared_ptr<planning_math::KDPath> ConstructLaneBorrowKDPath(
      const std::vector<double>& x_vec, const std::vector<double>& y_vec);
  void LogDebugInfo();
  void ClearInfo();
  void AddLaneBorrowVirtualObstacle(double obs_inner_l, double obs_start_s,
                                    double speed, bool is_reverse,
                                    int borrow_id, bool& is_hold_reset_path);
  void SetPullOverPath(double end_s, double end_l);
  void StabilizeRefinedPaths();
  bool GenerateBackOriginLaneRulePath();

  // inputs methods
  // Setters
  void set_ego_s(double s) { ego_s_ = s; }
  void set_ego_l(double l) { ego_l_ = l; }
  void set_ego_v(double v) { ego_v_ = v; }
  void set_v_cruise(double v) { v_cruise_ = v; }
  void set_vehicle_width(double width) { vehicle_width_ = width; }
  void set_vehicle_length(double length) { vehicle_length_ = length; }
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

  const std::vector<planning_math::PathPoint>& refined_paths() const {
    return refined_paths_;
  }
  std::vector<planning_math::PathPoint>& mutable_refined_paths() {
    return refined_paths_;
  }
  // dp search Params
};
}  // namespace lane_borrow_deciderv3_utils
}  // namespace planning