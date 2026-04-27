/**
 * @file lateral_motion_planner.h
 **/

#pragma once

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_motion_planner.pb.h"
#include "math_lib.h"
#include "obstacle_manager.h"
// #include "scenario_state_machine.h"

#include "task_basic_types.h"
#include "tasks/task.h"

#include "dynamic_model/dynamic_model.h"
#include "problem_solver/ilqr_solver.h"
#include "problem_solver/solver_define.h"
#include "weight/base_weight.h"

namespace planning {

class BaseLateralMotionPlanner : public Task {
 public:
  BaseLateralMotionPlanner(const EgoPlanningConfigBuilder* config_builder,
                           framework::Session* session);

  virtual ~BaseLateralMotionPlanner() = default;

  virtual bool Execute();

 protected:
  void InitInputAndOutput();

  void ResetInputAndOutput();

  void CalculateCurvFactor();

  bool HandleReferencePathData();

  bool HandleLateralBoundData();

  bool HandleFeedbackInfoData();

  void StraightPath();

  bool HandleOutputData();

  std::shared_ptr<planning_math::KDPath> ConstructLateralKDPath(
      const std::vector<double>& x_vec, const std::vector<double>& y_vec);

  void SaveDebugInfo();

 protected:
  LateralMotionPlannerConfig config_;
  planning::common::LateralPlanningInput planning_input_;
  planning::common::LateralPlanningOutput planning_output_;
  std::shared_ptr<pnc::lateral_planning::DynamicModel> dynamic_model_;
  std::shared_ptr<pnc::lateral_planning::BaseWeight> planning_weight_ptr_;
  // input
  bool is_uniform_motion_;
  bool is_need_reverse_;
  bool is_ref_consistent_;
  size_t valid_continuity_idx_;
  double curv_factor_;
  double max_wheel_angle_;
  double max_wheel_angle_rate_;
  double max_wheel_angle_rate_lc_;
  double lane_rel_theta_error_;
  std::vector<double> ref_theta_vec_;
  std::vector<double> virtual_ref_x_;
  std::vector<double> virtual_ref_y_;
  std::vector<double> virtual_ref_theta_;
  // output
  std::vector<double> x_vec_;
  std::vector<double> y_vec_;
  std::vector<double> theta_vec_;
  std::vector<double> delta_vec_;
  std::vector<double> omega_vec_;
  std::vector<double> curv_vec_;
  std::vector<double> d_curv_vec_;
  std::vector<double> s_vec_;
  std::vector<double> t_vec_;
};

}  // namespace planning