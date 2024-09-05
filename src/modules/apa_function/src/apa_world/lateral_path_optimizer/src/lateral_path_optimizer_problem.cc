#include "lateral_path_optimizer_problem.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "geometry_math.h"
#include "ilqr_define.h"
#include "lateral_path_optimizer_core.h"
#include "lateral_path_optimizer_cost.h"
#include "lateral_path_optimizer_model.h"
#include "math_lib.h"

using namespace pnc::mathlib;
using namespace ilqr_solver;
namespace planning {
namespace apa_planner {
void LateralPathOptimizerProblem::Init(const bool c_ilqr_enable) {
  // step 0 : set solver config parameters
  ilqr_solver::iLqrSolverConfig solver_config;
  solver_config.horizon = 29;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.max_al_iter = 10;
  solver_config.c_ilqr_enable = c_ilqr_enable;
  init_state_.resize(STATE_SIZE);
  // step1 : init core with solver config
  ilqr_core_ptr_ = std::make_shared<LateralPathOptimizerCore>();
  ilqr_core_ptr_->Init(std::make_shared<LateralPathOptimizerModel>(),
                       solver_config);

  // step 2: add cost
  ilqr_core_ptr_->AddCost(std::make_shared<ReferenceCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<TerminalCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<KCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<UCostTerm>());

  if (c_ilqr_enable) {
    ilqr_core_ptr_->AddCost(std::make_shared<KBoundCostTerm>());
    ilqr_core_ptr_->AddCost(std::make_shared<UBoundCostTerm>());
  } else {
    ilqr_core_ptr_->AddCost(std::make_shared<KSoftBoundCostTerm>());
    ilqr_core_ptr_->AddCost(std::make_shared<USoftBoundCostTerm>());
  }

  // step3 : init debug info
  ilqr_core_ptr_->InitAdvancedInfo();

  // init planning output
  const auto N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_s_vec()->Resize(N, 0.0);

  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_k_vec()->Resize(N, 0.0);

  planning_output_.mutable_u_vec()->Resize(N, 0.0);
}

uint8_t LateralPathOptimizerProblem::Update(
    planning::common::LateralPathOptimizerInput &planning_input,
    const uint8_t gear_cmd) {
  double model_sign_gain = 1.0;
  if (gear_cmd == pnc::geometry_lib::SEG_GEAR_INVALID) {
    return 0;
  } else if (gear_cmd == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    model_sign_gain = -1.0;
  }
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  std::vector<ilqr_solver::AliLqrConfig> alilqr_config_vec;
  alilqr_config_vec.resize(N);

  // transfer input to cost_config
  for (size_t i = 0; i < N; i++) {
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec(i);

    cost_config_vec.at(i)[TERMINAL_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[TERMINAL_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[TERMINAL_THETA] = planning_input.ref_theta_vec(i);

    cost_config_vec.at(i)[K_MAX] = planning_input.k_max_vec(i);
    cost_config_vec.at(i)[U_MAX] = planning_input.u_max_vec(i);

    cost_config_vec.at(i)[K_MIN] = planning_input.k_min_vec(i);
    cost_config_vec.at(i)[U_MIN] = planning_input.u_min_vec(i);

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }

    if (i >= N - 5) {
      cost_config_vec.at(i)[W_TERMINAL_X] = planning_input.q_terminal_x();
      cost_config_vec.at(i)[W_TERMINAL_Y] = planning_input.q_terminal_y();

      cost_config_vec.at(i)[W_TERMINAL_THETA] =
          planning_input.q_terminal_theta();
    } else {
      cost_config_vec.at(i)[W_TERMINAL_X] = 0.0;
      cost_config_vec.at(i)[W_TERMINAL_Y] = 0.0;
      cost_config_vec.at(i)[W_TERMINAL_THETA] = 0.0;
    }
    // weigth
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();
    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta();
    cost_config_vec.at(i)[W_K] = planning_input.q_k();
    cost_config_vec.at(i)[W_U] = planning_input.q_u();
    cost_config_vec.at(i)[MODEL_SIGN_GAIN] = model_sign_gain;

    // alilqr
    if (ilqr_core_ptr_->GetSolverConfigPtr()->c_ilqr_enable) {
      alilqr_config_vec.at(i)[W_K_HARDBOUND] = 1.0;
      alilqr_config_vec.at(i)[W_U_HARDBOUND] = 1.0;
      alilqr_config_vec.at(i)[L_K_HARDBOUND] = 0.0;
      alilqr_config_vec.at(i)[L_U_HARDBOUND] = 0.0;

      alilqr_config_vec.at(i)[PHI_SCALE] = 10.0;
      alilqr_config_vec.at(i)[CONSTRAINT_CONFIG_SIZE] = 2;
    }
  }

  // set const config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  if (ilqr_core_ptr_->GetSolverConfigPtr()->c_ilqr_enable) {
    ilqr_core_ptr_->SetAliLqrConfig(alilqr_config_vec);
  }

  // set solver config (ds)
  double ds =
      planning_input.ref_s() / ilqr_core_ptr_->GetSolverConfigPtr()->horizon;
  ilqr_solver::iLqrSolverConfig solver_config;
  solver_config.model_dt = ds;
  solver_config.horizon = 29;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.max_al_iter = 10;
  solver_config.c_ilqr_enable =
      ilqr_core_ptr_->GetSolverConfigPtr()->c_ilqr_enable;

  ilqr_core_ptr_->SetSolverConfig(solver_config);

  // solve ilqr problem
  init_state_ << planning_input.init_state().x(),
      planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().k();

  if (ilqr_core_ptr_->GetSolverConfigPtr()->c_ilqr_enable) {
    std::cout << "cilqr" << std::endl;
    ilqr_core_ptr_->SolveForAliLqr(init_state_);
  } else {
    std::cout << "ilqr" << std::endl;
    ilqr_core_ptr_->Solve(init_state_);
  }

  // fail protection
  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;

  const auto get_solver_config = ilqr_core_ptr_->GetSolverConfigPtr();

  // fail protection
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    double s = 0.0;
    for (size_t i = 0; i < N; i++) {
      planning_output_.mutable_s_vec()->Set(i, s);
      s += ds;
      planning_output_.mutable_x_vec()->Set(i, planning_input.ref_x_vec(i));
      planning_output_.mutable_y_vec()->Set(i, planning_input.ref_y_vec(i));

      planning_output_.mutable_theta_vec()->Set(
          i, planning_input.ref_theta_vec(i));

      planning_output_.mutable_k_vec()->Set(i, planning_input.ref_k_vec(i));

      planning_output_.mutable_u_vec()->Set(i, planning_input.control_vec(i));
    }
  } else {
    // assemble planning result
    const auto &state_result = ilqr_core_ptr_->GetStateResultPtr();
    const auto &control_result = ilqr_core_ptr_->GetControlResultPtr();

    double s = 0.0;
    for (size_t i = 0; i < N; i++) {
      planning_output_.mutable_s_vec()->Set(i, s);
      s += ds;
      planning_output_.mutable_x_vec()->Set(i, state_result->at(i)[StateId::X]);
      planning_output_.mutable_y_vec()->Set(i, state_result->at(i)[StateId::Y]);

      planning_output_.mutable_theta_vec()->Set(
          i, state_result->at(i)[StateId::THETA]);

      planning_output_.mutable_k_vec()->Set(i, state_result->at(i)[StateId::K]);

      if (i < N - 1) {
        planning_output_.mutable_u_vec()->Set(
            i, control_result->at(i)[ControlId::U]);
      } else {
        planning_output_.mutable_u_vec()->Set(i, planning_output_.u_vec(i - 1));
      }
    }
  }
  // load solver and iteration info
  planning_output_.clear_solver_info();

  const auto &soler_info_ptr = ilqr_core_ptr_->GetSolverInfoPtr();

  planning_output_.mutable_solver_info()->set_solver_condition(
      soler_info_ptr->solver_condition);

  planning_output_.mutable_solver_info()->set_cost_size(
      soler_info_ptr->cost_size);

  planning_output_.mutable_solver_info()->set_iter_count(
      soler_info_ptr->iter_count);

  planning_output_.mutable_solver_info()->set_init_cost(
      soler_info_ptr->init_cost);

  for (size_t i = 0; i < soler_info_ptr->iter_count; ++i) {
    const auto &iter_info =
        planning_output_.mutable_solver_info()->add_iter_info();

    iter_info->set_linesearch_success(
        soler_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        soler_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda(soler_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(soler_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(soler_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(soler_info_ptr->iteration_info_vec[i].du_norm);
  }

  return solver_condition;
}

}  // namespace apa_planner
}  // namespace planning
