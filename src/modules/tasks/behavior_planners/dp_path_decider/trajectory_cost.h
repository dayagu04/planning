#pragma once
#include <memory>
#include <type_traits>
#include <vector>
#include "agent/agent.h"
#include "cmath"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "dp_base.h"
#include "frenet_obstacle.h"
#include "reference_path.h"
#include "session.h"
#include "spline_projection.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "src/modules/common/st_graph/path_border_querier.h"
#include "src/modules/common/st_graph/path_border_segment.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "virtual_lane.h"
#include "environmental_model.h"
namespace planning {
using namespace planning_math;
class TrajectoryCost {
 public:
  TrajectoryCost(
      const SLPoint &init_sl_point, double sample_left_boundary,
      double sample_right_boundary, DPRoadGraphConfig config,
      CarReferenceInfo &ref_path_curve,
      std::vector<planning_math::Box2d> &flatted_dynamic_obstacles_box,
      std::vector<planning_math::Box2d> &static_obstacles_box,
      std::shared_ptr<VirtualLane> left_lane_ptr,
      std::shared_ptr<VirtualLane> right_lane_ptr,
      std::shared_ptr<VirtualLane> current_lane_ptr,
      FrenetEgoState &ego_frenet_state)
      : sample_left_boundary_(sample_left_boundary),
        sample_right_boundary_(sample_right_boundary),
        config_(config),
        flatted_dynamic_obstacles_box_(flatted_dynamic_obstacles_box),
        static_obstacles_box_(static_obstacles_box) {
    current_reference_path_ptr_ = current_lane_ptr->get_reference_path();
    left_lane_ptr_ = left_lane_ptr;
    right_lane_ptr_ = right_lane_ptr;
    current_lane_ptr_ = current_lane_ptr;
    ref_path_curve_ = ref_path_curve;
    ego_frenet_state_ = ego_frenet_state;
    // filtered_agents_ = filtered_agents;
    // v_cruise_ = current_lane_ptr_->get_ego_state_manager()->ego_v_cruise();
  };
  TrajectoryCost(
      DPRoadGraphConfig config,
      framework::Session *session,
      const SLPoint &init_sl_point,
      CarReferenceInfo &ref_path_curve,
      std::vector<planning_math::Box2d> &flatted_dynamic_obstacles_box,
      std::vector<planning_math::Box2d> &static_obstacles_box
  )
      : config_(config),
        session_(session),
        flatted_dynamic_obstacles_box_(flatted_dynamic_obstacles_box),
        static_obstacles_box_(static_obstacles_box),
        ref_path_curve_(ref_path_curve)
  {
    // lane ptr
    const auto& virtual_lane_mgr = session_->environmental_model().get_virtual_lane_manager();
    current_lane_ptr_ = virtual_lane_mgr->get_current_lane();
    left_lane_ptr_ = virtual_lane_mgr->get_left_lane();
    right_lane_ptr_ = virtual_lane_mgr->get_right_lane();
    current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();
  }

  ComparableCost Calculate(const QuinticPolynomialCurve1d &curve,
                           const double start_s, const double end_s,
                           int current_level, int total_level,LaneBorrowStatus lane_borrow_status);
  ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                   const double start_s, const double end_s,
                                   int current_level, int total_level,LaneBorrowStatus lane_borrow_status) const;
  ComparableCost GetObsSLCost(const FrenetObstacleBoundary &frenet_obstacle_sl,
                              const double curr_s, const double curr_l) const;
  ComparableCost CalObsCartCost(const QuinticPolynomialCurve1d &curve,
                                const double start_s, const double end_s) const;
  ComparableCost CalculateStaticObsCost(const QuinticPolynomialCurve1d &curve,
                                        const double start_s,
                                        const double end_s) const;
  ComparableCost CalDynamicObsCartCost(const QuinticPolynomialCurve1d &curve,
                                       const double start_s,
                                       const double end_s) const;
  ComparableCost CalculateStitchCost(const QuinticPolynomialCurve1d &curve,
                                     const double start_s,
                                     const double end_s) const;
  void BuildCurveBorder(const QuinticPolynomialCurve1d &curve,
                        const double start_s, const double end_s);
  ComparableCost GetBoxCost(const Box2d &ego_box, const Box2d &obs_box) const;
  bool BoxHasOverlap(const Box2d &ego_box, const Box2d &obs_box) const;
  void SetCostParams(double coeff_l_cost, double coeff_dl_cost,
                     double coeff_ddl_cost, double path_resolution,
                     double coeff_end_l_cost, double coeff_collision_cost,
                     double collision_distance, double coeff_stitch_cost) {
    coeff_l_cost_ = coeff_l_cost;
    coeff_dl_cost_ = coeff_dl_cost;
    coeff_ddl_cost_ = coeff_ddl_cost;
    path_resolution_ = path_resolution;
    coeff_end_l_cost_ = coeff_end_l_cost;
    coeff_collision_cost_ = coeff_collision_cost;
    collision_distance_ = collision_distance;
    coeff_stitch_cost_ = coeff_stitch_cost;

    path_cost_time_ = 0;
    safety_cost_time_ = 0;
    stitch_cost_time_ = 0;
    return;
  }

 private:
  DPRoadGraphConfig config_;
  double coeff_l_cost_ = 0.;
  double coeff_dl_cost_ = 0.;
  double coeff_ddl_cost_ = 0.;
  double path_resolution_ = 0.;
  double coeff_end_l_cost_ = 0.;
  double coeff_stitch_cost_ = 0.0;
  double coeff_collision_cost_ = 0.;
  double collision_distance_ = 0.;
  std::shared_ptr<ReferencePath> current_reference_path_ptr_;
  std::shared_ptr<VirtualLane> right_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> current_lane_ptr_ = nullptr;
  planning::framework::Session *session_;
  double sample_left_boundary_;
  double sample_right_boundary_;
  pnc::mathlib::spline x_s_spline_;
  pnc::mathlib::spline y_s_spline_;  // last frame spline  for dp cost xy
  CarReferenceInfo ref_path_curve_;
  std::shared_ptr<speed::PathBorderQuerier> curve_border_querier_ = nullptr;
  std::vector<planning_math::Box2d> &static_obstacles_box_;
  std::vector<planning_math::Box2d> &flatted_dynamic_obstacles_box_;
  FrenetEgoState ego_frenet_state_;

 public:
  double path_cost_time_ = 0;
  double safety_cost_time_ = 0;
  double stitch_cost_time_ = 0;
};
}  // namespace planning