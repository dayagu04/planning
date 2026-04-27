#include "ilqr_solver.h"

#include "../constraint_terms/lateral_acc_constraint.h"
#include "../constraint_terms/lateral_jerk_constraint.h"
#include "../constraint_terms/path_corridor_constraint.h"
#include "../cost_terms/continuity_cost.h"
#include "../cost_terms/edt_distance_cost.h"
#include "../cost_terms/lateral_acc_cost.h"
#include "../cost_terms/lateral_jerk_cost.h"
#include "../cost_terms/reference_path_cost.h"
#include "log.h"
#include "solver_define.h"

using namespace ilqr_solver;

namespace pnc {
namespace lateral_planning {

iLQRSolver::iLQRSolver() {}

void iLQRSolver::SimInit() {
  // STEP 0: set solver config parmeters
  ilqr_solver::iLqrSolverConfig solver_config;
  solver_config.horizon = 25;
  solver_config.state_size = StateID::STATE_SIZE;
  solver_config.input_size = ControlID::CONTROL_SIZE;
  solver_config.model_dt = 0.2;
  solver_config.warm_start_enable = false;
  solver_config.du_tol = 0.01 / 57.3 / 13.0;
  solver_config.max_iter = 20;
  init_state_.resize(StateID::STATE_SIZE);
  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<DynamicModel>(), solver_config);

  // STEP 2: add cost
  // 2-1: rear axle reference cost
  ilqr_core_ptr_->AddCost(std::make_shared<ReferencePathCostTerm>());
  // 2-2: rear axle reference continuity cost
  ilqr_core_ptr_->AddCost(std::make_shared<ContinuityCostTerm>());
  // 2-3: front axle reference cost
  ilqr_core_ptr_->AddCost(std::make_shared<FrontReferencePathCostTerm>());
  // 2-4: virtual rear axle reference cost
  ilqr_core_ptr_->AddCost(std::make_shared<VirtualReferencePathCostTerm>());
  // 2-5: lateral acc cost
  ilqr_core_ptr_->AddCost(std::make_shared<LateralAccCostTerm>());
  // 2-6: lateral jerk cost
  ilqr_core_ptr_->AddCost(std::make_shared<LateralJerkCostTerm>());
  // 2-7: lateral acc bound cost
  ilqr_core_ptr_->AddCost(std::make_shared<LateralAccBoundCostTerm>());
  // 2-8: lateral jerk soft bound cost
  ilqr_core_ptr_->AddCost(std::make_shared<LateralJerkBoundCostTerm>());
  // 2-9: path first soft corridor cost
  ilqr_core_ptr_->AddCost(std::make_shared<PathFirstSoftCorridorCostTerm>());
  // 2-10: path second soft corridor cost
  ilqr_core_ptr_->AddCost(std::make_shared<PathSecondSoftCorridorCostTerm>());
  // 2-11: path hard corridor cost
  ilqr_core_ptr_->AddCost(std::make_shared<PathHardCorridorCostTerm>());
  // 2.12: edt distance cost
  ilqr_core_ptr_->AddCost(std::make_shared<EdtDistanceCostTerm>());

  // STEP 3: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();
}

void iLQRSolver::Init(
    const ilqr_solver::iLqrSolverConfig& solver_config,
    const std::shared_ptr<DynamicModel> dynamic_model) {
  // STEP 0: init state
  init_state_.resize(solver_config.state_size);
  // STEP 1: init ilqr core
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(dynamic_model, solver_config);
  // STEP 2: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();
}

uint8_t iLQRSolver::Update(
    const double end_ratio_for_qxy, const double end_ratio_for_qtheta,
    const double end_ratio_for_qjerk, const double concerned_start_q_jerk,
    const double wheel_base, const std::vector<double> &virtual_ref_x,
    const std::vector<double> &virtual_ref_y,
    const std::vector<double> &virtual_ref_theta,
    const std::shared_ptr<pnc::lateral_planning::BaseWeight>
        &planning_weight,
    planning::common::LateralPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  const auto &path_weights = planning_weight->GetPathWeights();

  bool is_virtual_empty = virtual_ref_x.size() != N ||
                          virtual_ref_y.size() != N ||
                          virtual_ref_theta.size() != N;
  for (size_t i = 0; i < N; ++i) {
    double ref_vel = planning_input.ref_vel_vec(i);
    if (i < N - 1) {
      ref_vel = (ref_vel + planning_input.ref_vel_vec(i + 1)) * 0.5;
    }
    double kv2 = planning_input.curv_factor() * ref_vel * ref_vel;
    double expected_delta = path_weights.expected_acc[i] / kv2;
    // calculate delta_bound and omega_bound
    double delta_upper_bound = path_weights.acc_upper_bound[i] / kv2;
    double delta_lower_bound = path_weights.acc_lower_bound[i] / kv2;
    double omega_upper_bound = path_weights.jerk_upper_bound[i] / kv2;
    double omega_lower_bound = path_weights.jerk_lower_bound[i] / kv2;
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec(i);
    cost_config_vec.at(i)[REF_VEL] = ref_vel;
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor();

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec(i);
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec(i);
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec(i);

    if (is_virtual_empty) {
      cost_config_vec.at(i)[VIRTUAL_REF_X] = planning_input.ref_x_vec(i);
      cost_config_vec.at(i)[VIRTUAL_REF_Y] = planning_input.ref_y_vec(i);
      cost_config_vec.at(i)[VIRTUAL_REF_THETA] =
          planning_input.ref_theta_vec(i);
    } else {
      cost_config_vec.at(i)[VIRTUAL_REF_X] = virtual_ref_x[i];
      cost_config_vec.at(i)[VIRTUAL_REF_Y] = virtual_ref_y[i];
      cost_config_vec.at(i)[VIRTUAL_REF_THETA] = virtual_ref_theta[i];
    }

    cost_config_vec.at(i)[EXPECTEDE_DELTA] = expected_delta;

    cost_config_vec.at(i)[FRONT_REF_X] = planning_input.front_axis_ref_x_vec(i);
    cost_config_vec.at(i)[FRONT_REF_Y] = planning_input.front_axis_ref_y_vec(i);
    cost_config_vec.at(i)[WHEEL_BASE] = wheel_base;

    // bounds
    cost_config_vec.at(i)[DELTA_UPPER_BOUND] = delta_upper_bound;
    cost_config_vec.at(i)[DELTA_LOWER_BOUND] = delta_lower_bound;
    cost_config_vec.at(i)[OMEGA_UPPER_BOUND] = omega_upper_bound;
    cost_config_vec.at(i)[OMEGA_LOWER_BOUND] = omega_lower_bound;

    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_X0] =
        planning_input.first_soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_Y0] =
        planning_input.first_soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_X1] =
        planning_input.first_soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_Y1] =
        planning_input.first_soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_X0] =
        planning_input.first_soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_Y0] =
        planning_input.first_soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_X1] =
        planning_input.first_soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_Y1] =
        planning_input.first_soft_lower_bound_y1_vec(i);

    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_X0] =
        planning_input.second_soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_Y0] =
        planning_input.second_soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_X1] =
        planning_input.second_soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_Y1] =
        planning_input.second_soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_X0] =
        planning_input.second_soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_Y0] =
        planning_input.second_soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_X1] =
        planning_input.second_soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_Y1] =
        planning_input.second_soft_lower_bound_y1_vec(i);

    cost_config_vec.at(i)[HARD_UPPER_BOUND_X0] =
        planning_input.hard_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_Y0] =
        planning_input.hard_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_X1] =
        planning_input.hard_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_Y1] =
        planning_input.hard_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[HARD_LOWER_BOUND_X0] =
        planning_input.hard_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_Y0] =
        planning_input.hard_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_X1] =
        planning_input.hard_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_Y1] =
        planning_input.hard_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();
    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta();

    cost_config_vec.at(i)[W_FRONT_REF_X] = path_weights.q_front_ref_x[i];
    cost_config_vec.at(i)[W_FRONT_REF_Y] = path_weights.q_front_ref_y[i];

    if (is_virtual_empty) {
      cost_config_vec.at(i)[W_VIRTUAL_REF_X] = 0.0;
      cost_config_vec.at(i)[W_VIRTUAL_REF_Y] = 0.0;
      cost_config_vec.at(i)[W_VIRTUAL_REF_THETA] = 0.0;
    } else {
      cost_config_vec.at(i)[W_VIRTUAL_REF_X] = path_weights.q_virtual_ref_x[i];
      cost_config_vec.at(i)[W_VIRTUAL_REF_Y] = path_weights.q_virtual_ref_y[i];
      cost_config_vec.at(i)[W_VIRTUAL_REF_THETA] =
          path_weights.q_virtual_ref_theta[i];
    }

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();

    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();

    cost_config_vec.at(i)[W_FIRST_SOFT_CORRIDOR] =
        path_weights.q_pos_first_soft_bound[i];
    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = path_weights.q_pos_soft_bound[i];
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = path_weights.q_pos_hard_bound[i];

    cost_config_vec.at(i)[W_EDT_DISTANCE] = planning_input.q_edt_distance();

    if (!planning_input.complete_follow()) {
      if (i < path_weights.proximal_index) {
        double start_step =
            std::max((concerned_start_q_jerk - planning_input.q_jerk()) /
                         path_weights.proximal_index,
                     0.0);
        cost_config_vec.at(i)[W_JERK] = concerned_start_q_jerk - start_step * i;
        // cost_config_vec.at(i)[W_SOFT_CORRIDOR] =
        //     planning_input.q_soft_corridor() * 1.5;
        // cost_config_vec.at(i)[W_HARD_CORRIDOR] =
        //     planning_input.q_hard_corridor() * 1.5;
      } else if (i > path_weights.remotely_index) {
        cost_config_vec.at(i)[EXPECTEDE_DELTA] =
            cost_config_vec.at(i - 1)[EXPECTEDE_DELTA];
        cost_config_vec.at(i)[W_REF_X] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_REF_X];
        cost_config_vec.at(i)[W_REF_Y] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_REF_Y];
        cost_config_vec.at(i)[W_REF_THETA] =
            end_ratio_for_qtheta * cost_config_vec.at(i - 1)[W_REF_THETA];
        cost_config_vec.at(i)[W_JERK] =
            end_ratio_for_qjerk * cost_config_vec.at(i - 1)[W_JERK];
        cost_config_vec.at(i)[W_FIRST_SOFT_CORRIDOR] =
            cost_config_vec.at(i - 1)[W_FIRST_SOFT_CORRIDOR] * 0.3;
        cost_config_vec.at(i)[W_SOFT_CORRIDOR] =
            cost_config_vec.at(i - 1)[W_SOFT_CORRIDOR] * 0.3;
        cost_config_vec.at(i)[W_HARD_CORRIDOR] =
            cost_config_vec.at(i - 1)[W_HARD_CORRIDOR] * 0.3;
        cost_config_vec.at(i)[W_FRONT_REF_X] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_FRONT_REF_X];
        cost_config_vec.at(i)[W_FRONT_REF_Y] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_FRONT_REF_Y];
        // cost_config_vec.at(i)[W_EDT_DISTANCE] = 2.0 * planning_input.q_edt_distance();
      }
    }

    cost_config_vec.at(i)[W_CONTINUITY_X] =
        cost_config_vec.at(i)[W_REF_X] * path_weights.q_continuity[i];
    cost_config_vec.at(i)[W_CONTINUITY_Y] =
        cost_config_vec.at(i)[W_REF_Y] * path_weights.q_continuity[i];
    cost_config_vec.at(i)[W_CONTINUITY_THETA] =
        cost_config_vec.at(i)[W_REF_THETA] * path_weights.q_continuity[i];

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set cost config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().x(),
      planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().delta();

  ilqr_core_ptr_->Solve(init_state_);

  // fail protection
  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
  const auto solver_config = ilqr_core_ptr_->GetSolverConfigPtr();

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(solver_config->horizon + 1);

  for (size_t i = 0; i < u_vec.size(); ++i) {
    Control u;
    u.resize(1);
    u[0] = planning_input.control_vec(i);
    u_vec[i] = u;
  }

  // fail protection
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    ilqr_core_ptr_->Simulation(init_state_, u_vec);
  }

  return solver_condition;
}

uint8_t iLQRSolver::SimUpdate(
    double expected_acc, double start_acc, double end_acc,
    double end_ratio_for_qxy, double end_ratio_for_qtheta,
    double end_ratio_for_qjerk, double max_iter,
    const size_t motion_plan_concerned_start_index,
    const double concerned_start_q_jerk, const double ego_vel,
    double max_delta, double max_omega,
    const double wheel_base, const double q_front_xy, double q_virtual_ref_xy,
    double q_virtual_ref_theta, std::vector<double> &virtual_ref_x,
    std::vector<double> &virtual_ref_y, std::vector<double> &virtual_ref_theta,
    planning::common::LateralPlanningInput &planning_input) {
  // set cost config
  ilqr_core_ptr_->GetSolverConfigPtr()->max_iter = max_iter;
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  bool is_virtual_empty = virtual_ref_x.size() != N ||
                          virtual_ref_y.size() != N ||
                          virtual_ref_theta.size() != N;

  for (size_t i = 0; i < N; ++i) {
    double ref_vel = planning_input.ref_vel_vec(i);
    if (i < N - 1) {
      ref_vel = (ref_vel + planning_input.ref_vel_vec(i + 1)) * 0.5;
    }
    double kv2 = planning_input.curv_factor() * ref_vel * ref_vel;
    double expected_delta = expected_acc / kv2;
    // calculate delta_bound and omega_bound
    double delta_bound = std::min(planning_input.acc_bound() / kv2, max_delta);
    double omega_bound = std::min(planning_input.jerk_bound() / kv2, max_omega);
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec(i);
    cost_config_vec.at(i)[REF_VEL] = ref_vel;
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor();

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec(i);
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec(i);
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec(i);

    if (is_virtual_empty) {
      cost_config_vec.at(i)[VIRTUAL_REF_X] = planning_input.ref_x_vec(i);
      cost_config_vec.at(i)[VIRTUAL_REF_Y] = planning_input.ref_y_vec(i);
      cost_config_vec.at(i)[VIRTUAL_REF_THETA] =
          planning_input.ref_theta_vec(i);
    } else {
      cost_config_vec.at(i)[VIRTUAL_REF_X] = virtual_ref_x[i];
      cost_config_vec.at(i)[VIRTUAL_REF_Y] = virtual_ref_y[i];
      cost_config_vec.at(i)[VIRTUAL_REF_THETA] = virtual_ref_theta[i];
    }

    cost_config_vec.at(i)[EXPECTEDE_DELTA] = expected_delta;

    cost_config_vec.at(i)[FRONT_REF_X] = planning_input.front_axis_ref_x_vec(i);
    cost_config_vec.at(i)[FRONT_REF_Y] = planning_input.front_axis_ref_y_vec(i);
    cost_config_vec.at(i)[WHEEL_BASE] = wheel_base;

    // bounds
    cost_config_vec.at(i)[DELTA_UPPER_BOUND] = delta_bound;
    cost_config_vec.at(i)[DELTA_LOWER_BOUND] = -delta_bound;
    cost_config_vec.at(i)[OMEGA_UPPER_BOUND] = omega_bound;
    cost_config_vec.at(i)[OMEGA_LOWER_BOUND] = -omega_bound;

    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_X0] =
        planning_input.first_soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_Y0] =
        planning_input.first_soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_X1] =
        planning_input.first_soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_UPPER_BOUND_Y1] =
        planning_input.first_soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_X0] =
        planning_input.first_soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_Y0] =
        planning_input.first_soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_X1] =
        planning_input.first_soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[FIRST_SOFT_LOWER_BOUND_Y1] =
        planning_input.first_soft_lower_bound_y1_vec(i);

    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_X0] =
        planning_input.second_soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_Y0] =
        planning_input.second_soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_X1] =
        planning_input.second_soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_UPPER_BOUND_Y1] =
        planning_input.second_soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_X0] =
        planning_input.second_soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_Y0] =
        planning_input.second_soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_X1] =
        planning_input.second_soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[SECOND_SOFT_LOWER_BOUND_Y1] =
        planning_input.second_soft_lower_bound_y1_vec(i);

    cost_config_vec.at(i)[HARD_UPPER_BOUND_X0] =
        planning_input.hard_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_Y0] =
        planning_input.hard_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_X1] =
        planning_input.hard_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_Y1] =
        planning_input.hard_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[HARD_LOWER_BOUND_X0] =
        planning_input.hard_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_Y0] =
        planning_input.hard_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_X1] =
        planning_input.hard_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_Y1] =
        planning_input.hard_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();
    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta();

    cost_config_vec.at(i)[W_FRONT_REF_X] = q_front_xy;
    cost_config_vec.at(i)[W_FRONT_REF_Y] = q_front_xy;

    if (is_virtual_empty) {
      cost_config_vec.at(i)[W_VIRTUAL_REF_X] = 0.0;
      cost_config_vec.at(i)[W_VIRTUAL_REF_Y] = 0.0;
      cost_config_vec.at(i)[W_VIRTUAL_REF_THETA] = 0.0;
    } else {
      cost_config_vec.at(i)[W_VIRTUAL_REF_X] = q_virtual_ref_xy;
      cost_config_vec.at(i)[W_VIRTUAL_REF_Y] = q_virtual_ref_xy;
      cost_config_vec.at(i)[W_VIRTUAL_REF_THETA] = q_virtual_ref_theta;
    }

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();

    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();

    cost_config_vec.at(i)[W_FIRST_SOFT_CORRIDOR] = 0.5 * planning_input.q_soft_corridor();
    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = planning_input.q_soft_corridor();
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = planning_input.q_hard_corridor();

    cost_config_vec.at(i)[W_EDT_DISTANCE] = planning_input.q_edt_distance();

    if (!planning_input.complete_follow()) {
      if (i < motion_plan_concerned_start_index) {
        double start_step =
            std::max((concerned_start_q_jerk - planning_input.q_jerk()) /
                         motion_plan_concerned_start_index,
                     0.0);
        cost_config_vec.at(i)[W_JERK] = concerned_start_q_jerk - start_step * i;
        cost_config_vec.at(i)[W_SOFT_CORRIDOR] =
            planning_input.q_soft_corridor() * 1.5;
        cost_config_vec.at(i)[W_HARD_CORRIDOR] =
            planning_input.q_hard_corridor() * 1.5;
        cost_config_vec.at(i)[W_ACC] = start_acc;
      } else if (i > planning_input.motion_plan_concerned_index()) {
        cost_config_vec.at(i)[W_REF_X] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_REF_X];
        cost_config_vec.at(i)[W_REF_Y] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_REF_Y];
        cost_config_vec.at(i)[W_REF_THETA] =
            end_ratio_for_qtheta * cost_config_vec.at(i - 1)[W_REF_THETA];
        cost_config_vec.at(i)[W_JERK] =
            end_ratio_for_qjerk * cost_config_vec.at(i - 1)[W_JERK];
        cost_config_vec.at(i)[W_SOFT_CORRIDOR] =
            cost_config_vec.at(i - 1)[W_SOFT_CORRIDOR] * 0.3;
        cost_config_vec.at(i)[W_HARD_CORRIDOR] =
            cost_config_vec.at(i - 1)[W_HARD_CORRIDOR] * 0.3;
        cost_config_vec.at(i)[W_ACC] = end_acc;
        cost_config_vec.at(i)[W_FRONT_REF_X] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_FRONT_REF_X];
        cost_config_vec.at(i)[W_FRONT_REF_Y] =
            end_ratio_for_qxy * cost_config_vec.at(i - 1)[W_FRONT_REF_Y];
      }
    }
    cost_config_vec.at(i)[W_CONTINUITY_X] =
        cost_config_vec.at(i)[W_REF_X] * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_Y] =
        cost_config_vec.at(i)[W_REF_Y] * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_THETA] =
        cost_config_vec.at(i)[W_REF_THETA] * planning_input.q_continuity();

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set cost config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().x(),
      planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().delta();

  ilqr_core_ptr_->Solve(init_state_);

  // fail protection
  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
  const auto solver_config = ilqr_core_ptr_->GetSolverConfigPtr();

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(solver_config->horizon + 1);

  for (size_t i = 0; i < u_vec.size(); ++i) {
    Control u;
    u.resize(1);
    u[0] = planning_input.control_vec(i);
    u_vec[i] = u;
  }

  // fail protection
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    ilqr_core_ptr_->Simulation(init_state_, u_vec);
  }

  return solver_condition;
}

}  // namespace lateral_planning
}  // namespace pnc