#include "ilqr_core.h"
#include "math_lib.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cstddef>
#include <iostream>
#include <numeric>

using namespace pnc::mathlib;
namespace ilqr_solver {
void iLqr::Init(const std::shared_ptr<iLqrModel> ilqr_model,
                const iLqrSolverConfig &ilqr_sovler_config) {
  // init first to make core share config_ptr with model
  Init(ilqr_model);

  // set config
  *solver_config_ptr_ = ilqr_sovler_config;

  // init config
  InitSolverConfig();
}

void iLqr::Init(std::shared_ptr<iLqrModel> ilqr_model) {
  ilqr_model_ptr_ = ilqr_model;

  // solver config will also be used by model
  solver_config_ptr_ = ilqr_model->GetSolverConfigPtr();

  // do not init solver config without config input
  // InitSolverConfig();
}

void iLqr::InitSolverConfig() {
  const auto horizon = solver_config_ptr_->horizion;
  const auto input_size = solver_config_ptr_->input_size;
  const auto state_size = solver_config_ptr_->state_size;

  xk_vec_.resize(horizon + 1);
  ResetVectorElemFromEigen(xk_vec_);

  uk_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(uk_vec_, input_size);
  ResetVectorElemFromEigen(uk_vec_);

  lx_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(lx_vec_, state_size);
  ResetVectorElemFromEigen(lx_vec_);

  lu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(lu_vec_, input_size);
  ResetVectorElemFromEigen(lu_vec_);

  lxx_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(lxx_vec_, state_size, state_size);
  ResetVectorElemFromEigen(lxx_vec_);

  lxu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(lxu_vec_, state_size, input_size);
  ResetVectorElemFromEigen(lxu_vec_);

  luu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(luu_vec_, input_size, input_size);
  ResetVectorElemFromEigen(luu_vec_);

  fx_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(fx_vec_, state_size, state_size);
  ResetVectorElemFromEigen(fx_vec_);

  fu_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(fu_vec_, state_size, input_size);
  ResetVectorElemFromEigen(fu_vec_);

  k_vec_.resize(horizon);
  ResizeVectorElemFromEigenVec(k_vec_, input_size);
  ResetVectorElemFromEigen(k_vec_);

  K_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(K_vec_, state_size, state_size);
  ResetVectorElemFromEigen(K_vec_);

  Quu_eye_ = Eigen::MatrixXd::Identity(input_size, input_size);

  // du norm debug
  du_norm_vec_.resize(horizon);

  // solver info init
  const auto max_iter = solver_config_ptr_->max_iter;
  solver_info_.solver_condition = iLqrSolveCondition::INIT;
  solver_info_.iteration_info_vec.resize(max_iter + 1);

  // init advanced info
#ifdef __ILQR_DEBUG__
  // +1 means considering init cost
  solver_info_.cost_map_vec.resize(max_iter + 1);

  // +1 means considering init cost
  solver_info_.cost_iter_vec.resize(max_iter + 1);
#endif

  // init cost size
  solver_info_.cost_size = 0;

  // for ilqr_model
  ilqr_model_ptr_->InitControlVar();
}

void iLqr::AddCost(std::shared_ptr<BaseCostTerm> cost_term) {
  ilqr_model_ptr_->AddCost(cost_term);
  solver_info_.cost_size++;
}

void iLqr::Solve(const State &x0) {
// TODO: check feasibility of input and avoid coredump
// InputFeasibilityCheck(xk, uk);

// time tag of all start
#ifdef __ILQR_TIMER__
  // init time info
  time_info_.Reset();
  time_info_.UpdateAllStart();
#endif

  // set init x
  xk_vec_[0] = x0;

  // set first cost_map before init guess
#ifdef __ILQR_DEBUG__
  ilqr_model_ptr_->SetCostMapPtr(&(solver_info_.cost_map_vec[0]));
#endif

  // init guess
  ilqr_model_ptr_->InitGuess(xk_vec_, uk_vec_, cost_);

  // calculate t_init_guess_ms
#ifdef __ILQR_TIMER__
  time_info_.t_init_guess_ms +=
      time_info_.GetElapsed(time_info_.all_start, false);
#endif

  // update advanced info after init guess
#ifdef __ILQR_DEBUG__
  UpdateAdvancedInfo(0);
#endif

  // ilqr main iteration
  iLqrIteration();

  // t_one_step_ms
#ifdef __ILQR_TIMER__
  time_info_.t_one_step_ms = time_info_.GetElapsed(time_info_.all_start, false);
#endif

  // print info when debug
#ifdef __ILQR_PRINT__
  PrintSolverInfo();
  PrintCostInfo();
  PrintTimeInfo();
#endif
}

void iLqr::Reset() { ilqr_model_ptr_->Reset(); }

bool iLqr::PSDCheck(Eigen::MatrixXd &Q) {
  if (solver_config_ptr_->input_size == 1) {
    // for scalar input, to make Quu be positive definite is very easy
    // lambda_min must less than 1e-5
    if (Quu_(0, 0) <= std::min(solver_config_ptr_->lambda_min, 1e-5)) {
      lambda_ = solver_config_ptr_->lambda_fix;
      return false;
    } else {
      return true;
    }
  } else {
    Eigen::LLT<Eigen::MatrixXd> lltOfA(Q);
    if (lltOfA.info() == Eigen::NumericalIssue) {
      return false;
    } else {
      return true;
    }
  }
}
/*
   INPUTS
      cx: 2x(N+1)          cu: 2x(N+1)
      cuu: nxnx(N+1)        cxx: nxnx(N+1)  cuu: 2x2x(N+1)
      fx: nxnx(N+1)        fu: nx2x(N+1)    fxx: none
      fxu: None            fuu: none        u: 2xT
    OUTPUTS
      Vx: nx(N+1)      Vxx: nxnx(N+1)      k:mxT
      K: mxnxT         dV: 2x1
      diverge - returns 0 if it doesn't diverge, timestep where it diverges
   otherwise
*/
bool iLqr::BackwardPass() {
  // cost-to-go at end
  Eigen::MatrixXd Vx = lx_vec_[solver_config_ptr_->horizion];
  Eigen::MatrixXd Vxx = lxx_vec_[solver_config_ptr_->horizion];

  dV_.fill(0);

  for (int i = (static_cast<int>(solver_config_ptr_->horizion) - 1); i >= 0;
       i--) { // back up from end of trajectory

    const Eigen::MatrixXd fut = fu_vec_[i].transpose();
    const Eigen::MatrixXd fxt = fx_vec_[i].transpose();
    const Eigen::MatrixXd fu_Vxx = fut * Vxx;

    Qx_ = lx_vec_[i] + fxt * Vx;
    Qu_ = lu_vec_[i] + fut * Vx;
    Qxx_ = lxx_vec_[i] + fxt * Vxx * fx_vec_[i];
    Quu_ = luu_vec_[i] + fu_Vxx * fu_vec_[i];
    Qux_ = lxu_vec_[i].transpose() + fu_Vxx * fx_vec_[i];

    // Similar to equations 10a and 10b in [Tassa 2012]. Note that
    // regularization is different
    Quuf_ = Quu_ + lambda_ * Quu_eye_;
    if (!PSDCheck(Quuf_)) {
      std::cout << "wocao wocao = " << Quuf_ << std::endl;
      return false;
    }

    const Eigen::MatrixXd QuuF_inv = Quuf_.inverse();
    k_i_ = -QuuF_inv * Qu_;
    K_i_ = -QuuF_inv * Qux_;

    const Eigen::MatrixXd Kt = K_i_.transpose();
    const Eigen::MatrixXd Kt_Quu = Kt * Quu_;
    const Eigen::MatrixXd Quxt = Qux_.transpose();
    const Eigen::MatrixXd Kt_Quu_Quxt = Kt_Quu + Quxt;

    // Update cost-to-go approximation. Equations 11 in [Tassa 2012]
    dV_[0] += k_i_.transpose() * Qu_;
    dV_[1] += 0.5 * k_i_.transpose() * Quu_ * k_i_;

    Vx = Qx_ + Kt_Quu_Quxt * k_i_ + Kt * Qu_;
    Vxx = Qxx_ + Kt_Quu_Quxt * K_i_ + Kt * Qux_;

    // save controls/gains
    k_vec_[i] = k_i_;
    K_vec_[i] = K_i_;
  }
  return true;
}

// input x, u, dV, k
// output success_flag, new_cost
bool iLqr::ForwardPass(double &new_cost, double &expected, const size_t &iter) {
  double alpha;
  xk_new_vec_ = xk_vec_;
  uk_new_vec_ = uk_vec_;

  size_t linsearch_start_index = 0;
  if (iter > 0 &&
      solver_info_.iteration_info_vec[iter - 1].linesearch_success == false) {
    linsearch_start_index = solver_config_ptr_->alpha_vec.size() - 2;
  }

  solver_info_.iteration_info_vec[iter].linesearch_count = 0;
  // linesearch process
  for (size_t i = linsearch_start_index;
       i < solver_config_ptr_->alpha_vec.size(); ++i) {
    // set linesearch count for info
    solver_info_.iteration_info_vec[iter].linesearch_count++;

    alpha = solver_config_ptr_->alpha_vec[i];
    new_cost = 0.0;

    for (size_t i = 0; i < solver_config_ptr_->horizion; ++i) {
      const auto du =
          alpha * k_vec_[i] + K_vec_[i] * (xk_new_vec_[i] - xk_vec_[i]);

      uk_new_vec_[i] = uk_vec_[i] + du;
      du_norm_vec_[i] = du.norm();

      new_cost += ilqr_model_ptr_->GetCost(xk_new_vec_[i], uk_new_vec_[i], i);

      xk_new_vec_[i + 1] = ilqr_model_ptr_->UpdateDynamicsOneStep(
          xk_new_vec_[i], uk_new_vec_[i], i);
    }

    const double du_norm =
        std::accumulate(du_norm_vec_.begin(), du_norm_vec_.end(), 0.0) /
        du_norm_vec_.size();
    solver_info_.iteration_info_vec[iter].du_norm = du_norm;

    new_cost += ilqr_model_ptr_->GetTerminalCost(
        xk_new_vec_[solver_config_ptr_->horizion]);

    expected = -alpha * (dV_[0] + alpha * dV_[1]);

    // note that (expected >= 0.0) does not mean reduced cost
    if (expected >= 0.0) {
      if ((cost_ - new_cost) / expected > solver_config_ptr_->z_min) {
        // line search success
        xk_vec_ = xk_new_vec_;
        uk_vec_ = uk_new_vec_;
        return true;
      } else if (du_norm < solver_config_ptr_->du_tol) {
        return false;
      }
    } else {
      // expected < 0 should not occour
      solver_info_.solver_condition = NON_POSITIVE_EXPECT;
      return false;
    }
  }
  return false;
}

void iLqr::SetMaxIter(size_t max_iter) {
  solver_config_ptr_->max_iter = max_iter;

  solver_info_.iteration_info_vec.resize(max_iter + 1);

  InitAdvancedInfo();
}

void iLqr::InitAdvancedInfo() {
  // init advanced info
#ifdef __ILQR_DEBUG__
  // +1 means considering init cost
  solver_info_.cost_map_vec.resize(solver_config_ptr_->max_iter + 1);

  // +1 means considering init cost
  solver_info_.cost_iter_vec.resize(solver_config_ptr_->max_iter + 1);

  std::vector<double> tmp_vec;
  tmp_vec.resize(solver_config_ptr_->horizion + 1, 0.0);

  for (size_t iter = 0; iter < solver_config_ptr_->max_iter + 1; ++iter) {
    // init cost map after adding cost
    auto &cost_map = solver_info_.cost_map_vec[iter];
    cost_map.clear();
    for (auto &each_cost : *(ilqr_model_ptr_->GetCostStackPtr())) {
      cost_map.insert(std::make_pair(each_cost->GetCostId(), tmp_vec));
    }

    // init cost_iter_vec after adding cost
    solver_info_.cost_iter_vec[iter].resize(solver_info_.cost_size, 0);
  }
#endif
}

void iLqr::IncreaseLambda() {
  lambda_gain_ = std::max(solver_config_ptr_->lambda_factor,
                          lambda_gain_ * solver_config_ptr_->lambda_factor);
  lambda_ = std::max(solver_config_ptr_->lambda_start, lambda_ * lambda_gain_);
}

void iLqr::DecreaseLambda() {
  lambda_gain_ = std::min(1.0 / solver_config_ptr_->lambda_factor,
                          lambda_gain_ / solver_config_ptr_->lambda_factor);
  lambda_ = lambda_ * lambda_gain_ * (lambda_ > solver_config_ptr_->lambda_min);
}

void iLqr::UpdateDynamicsDerivatives() {
  for (size_t i = 0; i < solver_config_ptr_->horizion + 1; ++i) {
    // reset Derivatives before updating
    lx_vec_[i].setZero();
    lu_vec_[i].setZero();
    lxx_vec_[i].setZero();
    lxu_vec_[i].setZero();
    luu_vec_[i].setZero();

    if (i < solver_config_ptr_->horizion) {
      ilqr_model_ptr_->GetDynamicsDerivatives(xk_vec_[i], uk_vec_[i],
                                              fx_vec_[i], fu_vec_[i], i);

      ilqr_model_ptr_->GetGradientHessian(xk_vec_[i], uk_vec_[i], i, lx_vec_[i],
                                          lu_vec_[i], lxx_vec_[i], lxu_vec_[i],
                                          luu_vec_[i]);
    } else {
      ilqr_model_ptr_->GetTerminalGradientHessian(xk_vec_[i], lx_vec_[i],
                                                  lu_vec_[i], lxx_vec_[i],
                                                  lxu_vec_[i], luu_vec_[i]);
    }
  }
}

// This assumes that x0, xs, us are initialized
bool iLqr::iLqrIteration() {
  if (xk_vec_.empty() || uk_vec_.empty()) {
    solver_info_.solver_condition = iLqrSolveCondition::FAULT_INPUT_SIZE;
    return false;
  }

  solver_info_.init_cost = cost_;

  // init some states
  solver_info_.solver_condition = iLqrSolveCondition::INIT;
  lambda_ = 0.0;
  lambda_gain_ = 1.0;

  // iteration loop
  bool solver_success = false;
  bool update_success = true;
  for (size_t iter = 0; iter < solver_config_ptr_->max_iter; ++iter) {
    // timer tag for iteration start
#ifdef __ILQR_TIMER__
    time_info_.UpdateStart();
#endif

    // set iter count for info
    solver_info_.iter_count = iter + 1;

    // set cost_map before iteration
#ifdef __ILQR_DEBUG__
    ilqr_model_ptr_->SetCostMapPtr(&(solver_info_.cost_map_vec[iter + 1]));
#endif

    // update dynamics and derivatives when iter success
    if (update_success) {
      // Differentiate dynamics and cost along new trajectory before iteration
      UpdateDynamicsDerivatives();
      update_success = false;
    }

    // calculate t_compute_deriv_ms
#ifdef __ILQR_TIMER__
    time_info_.t_compute_deriv_ms +=
        time_info_.GetElapsed(time_info_.start, true);
#endif

    // STEP 1: backward pass
    size_t backward_pass_count = 0;
    while (true) {
      backward_pass_count++;

      // set backward pass count for info
      solver_info_.iteration_info_vec[iter].backward_pass_count =
          backward_pass_count;

      // Update Vx, Vxx, l, L, dV with back_pass
      const bool is_converged = BackwardPass();
      if (is_converged) {
        break;
      } else {
        if (lambda_ > solver_config_ptr_->lambda_max ||
            backward_pass_count >=
                solver_config_ptr_->max_backward_pass_count) {
          // backward pass failed, this should not happen when input_size = 1
          solver_info_.solver_condition =
              iLqrSolveCondition::BACKWARD_PASS_FAIL;
          return false;
        }
        IncreaseLambda();
      }
    }

    // calculate t_backward_pass_ms
#ifdef __ILQR_TIMER__
    time_info_.t_backward_pass_ms +=
        time_info_.GetElapsed(time_info_.start, true);
#endif

    // STEP 2: forward pass
    double expected = 0.0;
    double new_cost = 0.0;
    const bool forward_pass_success = ForwardPass(new_cost, expected, iter);
    const double dcost = cost_ - new_cost;

    // solver info recording
    solver_info_.iteration_info_vec[iter].linesearch_success =
        forward_pass_success;

    solver_info_.iteration_info_vec[iter].cost = new_cost;
    solver_info_.iteration_info_vec[iter].dcost = dcost;
    solver_info_.iteration_info_vec[iter].lambda = lambda_;
    solver_info_.iteration_info_vec[iter].expect = expected;

    // calculate t_forward_pass_ms
#ifdef __ILQR_TIMER__
    time_info_.t_forward_pass_ms +=
        time_info_.GetElapsed(time_info_.start, true);
#endif

    // update advanced info after forward pass
#ifdef __ILQR_DEBUG__
    // +1 means considering init
    UpdateAdvancedInfo(iter + 1);
#endif

    // STEP 3: check if terminate
    if (forward_pass_success) {
      // decrease lambda
      DecreaseLambda();

      // accept changes
      cost_ = new_cost;

      // when accept change, dynamics and derivatives should be updated
      update_success = true;

      // terminate check
      if (dcost < solver_config_ptr_->cost_tol) {
        solver_success = true;
        solver_info_.solver_condition = iLqrSolveCondition::NORMAL_TERMINATE;
        break;
      }
    } else {
      // linesearch failed, increase lambda
      IncreaseLambda();

      // terminate?
      if (lambda_ > solver_config_ptr_->lambda_max) {
        solver_success = true;
        if (iter > 0) {
          solver_info_.solver_condition =
              iLqrSolveCondition::LINESEARCH_TERMINATE;
        } else {
          solver_info_.solver_condition = iLqrSolveCondition::INIT_TERMINATE;
        }
        break;
      } else if (solver_info_.solver_condition ==
                 iLqrSolveCondition::NON_POSITIVE_EXPECT) {
        solver_success = false;
      } else {
        if (solver_info_.iteration_info_vec[iter].du_norm <
            solver_config_ptr_->du_tol) {
          // should be considered next
          // when linesearch cannot result in reduced cost, terminate
          solver_success = true;
          if (iter > 0) {
            solver_info_.solver_condition =
                iLqrSolveCondition::CONST_CONTROL_TERMINATE;
          } else {
            solver_info_.solver_condition = iLqrSolveCondition::INIT_TERMINATE;
          }
          break;
        }
      }
    }

    if (iter == solver_config_ptr_->max_iter - 1) {
      solver_info_.solver_condition = iLqrSolveCondition::MAX_ITER_TERMINATE;
      solver_success =
          true; // This situation should not belong to solver success;
    }
  } // end of iteration loop

  ilqr_model_ptr_->UpdateWarmStart(solver_success, uk_vec_);

  return solver_success;
} // generate_trajectory

void iLqr::UpdateAdvancedInfo(size_t iter) {
  // record x_vec and u_vec for each iteration
  solver_info_.iteration_info_vec[iter].x_vec = xk_vec_;
  solver_info_.iteration_info_vec[iter].u_vec = uk_vec_;

  // record every cost term for each iteration
  auto &cost_map = solver_info_.cost_map_vec[iter];

  size_t i = 0;
  for (auto &each_cost : *(ilqr_model_ptr_->GetCostStackPtr())) {
    auto const &vec = cost_map[each_cost->GetCostId()];
    solver_info_.cost_iter_vec[iter].at(i++) =
        std::accumulate(vec.begin(), vec.end(), 0.0);
  }
}

void iLqr::PrintSolverInfo() {
  std::cout
      << "--------------------------------------------------- iLqr solver info "
         "--------------------------------------------------- "
      << std::endl;

  // iteration info
  std::cout << "cost size = " << solver_info_.cost_size
            << ", init cost = " << solver_info_.init_cost
            << ", iteration count = " << solver_info_.iter_count
            << ", solver condition = "
            << static_cast<size_t>(solver_info_.solver_condition) << std::endl;

  std::cout << "iteration\tcost\t\treduction\texpect\t\tlambda\t\tLS_count\tLS_"
               "success\tdu_norm\n";

  for (size_t iter = 0; iter < solver_info_.iter_count; ++iter) {
    printf(
        "%-12ld\t%-12.5g\t%-12.5f\t%-12.5f\t%-12.5f\t%-12ld\t%-12d\t%-12.5f\n",
        iter, solver_info_.iteration_info_vec[iter].cost,
        solver_info_.iteration_info_vec[iter].dcost,
        solver_info_.iteration_info_vec[iter].expect,
        solver_info_.iteration_info_vec[iter].lambda,
        solver_info_.iteration_info_vec[iter].linesearch_count,
        solver_info_.iteration_info_vec[iter].linesearch_success,
        solver_info_.iteration_info_vec[iter].du_norm);
  }
}

void iLqr::PrintCostInfo() {
#ifdef __ILQR_DEBUG__
  // std::cout << "\ncost details:" << std::endl;
  // for (size_t iter = 0; iter < solver_info_.iter_count + 1; ++iter) {
  //   if (iter == 0) {
  //     printf("\ninit: \n");
  //   } else {
  //     printf("\niter: %ld\n", iter - 1);
  //   }
  //   // printf details
  //   auto vec = solver_info_.cost_map_vec[iter].at(
  //       ilqr_model_ptr_->GetCostStackPtr()->at(0)->GetCostId());
  //   for (size_t j = 0; j < solver_info_.cost_size; ++j) {
  //     printf("cost[%ld] = [ ", j);
  //     for (auto &x : vec) {
  //       printf("%.3f ", x);
  //     }
  //     printf("]\n");
  //     // double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  //     // printf("sum = %.5f\n", sum);
  //   }
  // }

  // cost info
  std::cout << "\n-----cost vec info:" << std::endl;
  std::cout << "cost list: [";
  for (size_t i = 0; i < ilqr_model_ptr_->GetCostStackPtr()->size(); ++i) {
    if (i < ilqr_model_ptr_->GetCostStackPtr()->size() - 1) {
      std::cout << ilqr_model_ptr_->GetCostStackPtr()->at(i)->GetCostString()
                << ", ";
    } else {
      std::cout << ilqr_model_ptr_->GetCostStackPtr()->at(i)->GetCostString()
                << "]" << std::endl;
    }
  }
  for (size_t iter = 0; iter < solver_info_.iter_count + 1; ++iter) {
    if (iter == 0) {
      printf("cost_vec[init] = [ ");
    } else {
      // printf("cost_vec[%ld]    = [ ", iter - 1);
      printf("cost_vec[%ld]    = [ ", iter - 1);
    }

    for (size_t i = 0; i < solver_info_.cost_size; ++i) {
      printf("%.5f ", solver_info_.cost_iter_vec[iter].at(i));
    }
    printf("]\n");
  }
#endif
}

void iLqr::PrintTimeInfo() {
#ifdef __ILQR_TIMER__
  // time info
  std::cout << "\n-----time info:" << std::endl;
  std::cout << "Total time: " << time_info_.t_one_step_ms << std::endl;
  std::cout << "compute_derivatives: " << time_info_.t_compute_deriv_ms
            << std::endl;
  std::cout << "backward pass: " << time_info_.t_backward_pass_ms << std::endl;
  std::cout << "forward pass: " << time_info_.t_forward_pass_ms << std::endl;
  std::cout << "other stuff: "
            << time_info_.t_one_step_ms - (time_info_.t_compute_deriv_ms +
                                           time_info_.t_backward_pass_ms +
                                           time_info_.t_forward_pass_ms)
            << std::endl;
#endif
}
} // namespace ilqr_solver
