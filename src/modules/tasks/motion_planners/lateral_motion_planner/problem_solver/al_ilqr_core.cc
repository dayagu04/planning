#include "al_ilqr_core.h"

#include <iostream>
#include <numeric>

#include "ifly_time.h"
#include "solver_define.h"
#include "math_lib.h"

using namespace pnc::mathlib;

namespace pnc {
namespace lateral_planning {

bool ALiLQR::ForwardPass(double &new_cost, double &expected,
                                           const size_t &iter) {
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

    for (auto &each_cost : *(ilqr_model_ptr_->GetCostStackPtr())) {
      each_cost->ResetCostValue();
    }
    for (size_t i = 0; i < solver_config_ptr_->horizon; ++i) {
      const auto du =
          alpha * k_vec_[i] + K_vec_[i] * (xk_new_vec_[i] - xk_vec_[i]);

      uk_new_vec_[i] = uk_vec_[i] + du;

      // double tmp_u = uk_new_vec_[i][U];
      // const auto u_com = pnc::mathlib::Limit(tmp_u, 0.3 * 400 / 57.3 / 15
      // / 1.688); uk_new_vec_[i][U] = u_com;

      du_norm_vec_[i] = du.norm();

      new_cost += ilqr_model_ptr_->GetCost(xk_new_vec_[i], uk_new_vec_[i], i);

      xk_new_vec_[i + 1] = ilqr_model_ptr_->UpdateDynamicsOneStep(
          xk_new_vec_[i], uk_new_vec_[i], i);

      // double tmp_k =  xk_new_vec_[i + 1][K];
      // const auto k_com = pnc::mathlib::Limit(tmp_k, 1.0 / 5.5);
      // xk_new_vec_[i + 1][K] = k_com;

      // if (std::isnan(xk_new_vec_[i + 1].norm()) || 1) {
      //   std::cout << "-------------------debug_info ------------" <<
      //   std::endl; std::cout << "alpha = " << alpha << std::endl; std::cout
      //   << "k_vec_[i] = " << k_vec_[i].transpose() << std::endl; std::cout <<
      //   "K_vec_[i] = " << K_vec_[i] << std::endl; std::cout << "xk_vec_[i] =
      //   " << xk_vec_[i].transpose() << std::endl; std::cout <<
      //   "xk_new_vec_[i] = " << xk_new_vec_[i].transpose()
      //             << std::endl;
      //   std::cout << "xk_new_vec_[i + 1] = " << xk_new_vec_[i +
      //   1].transpose()
      //             << std::endl;
      // }
    }

    const double du_norm =
        std::accumulate(du_norm_vec_.begin(), du_norm_vec_.end(), 0.0) /
        du_norm_vec_.size();
    solver_info_.iteration_info_vec[iter].du_norm = du_norm;

    new_cost += ilqr_model_ptr_->GetTerminalCost(
        xk_new_vec_[solver_config_ptr_->horizon]);

    expected = -alpha * (dV_[0] + alpha * dV_[1]);

    // note that (expected >= 0.0) does not mean reduced cost
    if (expected >= 0.0) {
      if ((cost_ - new_cost) / expected > solver_config_ptr_->z_min) {
        // line search success
        xk_vec_ = xk_new_vec_;
        uk_vec_ = uk_new_vec_;
        return true;
      } else if (du_norm < solver_config_ptr_->du_tol) {
        solver_info_.solver_condition = CONST_CONTROL_TERMINATE;
        return false;
      }
    } else {
      // expected < 0 should not occour
      solver_info_.solver_condition = NON_POSITIVE_EXPECT;
      return false;
    }
  }
  solver_info_.solver_condition = MAX_ITER_TERMINATE;
  return false;
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
bool ALiLQR::BackwardPass() {
  // cost-to-go at end
  Eigen::MatrixXd Vx = lx_vec_[solver_config_ptr_->horizon];
  Eigen::MatrixXd Vxx = lxx_vec_[solver_config_ptr_->horizon];

  dV_.fill(0);

  for (int i = (static_cast<int>(solver_config_ptr_->horizon) - 1); i >= 0;
       i--) {  // back up from end of trajectory
    // seperate matrix
    const Eigen::MatrixXd fu = fu_vec_[i];
    const Eigen::MatrixXd fx = fx_vec_[i];
    const Eigen::MatrixXd lx_vec = lx_vec_[i];
    const Eigen::MatrixXd lxx_vec = lxx_vec_[i];
    const Eigen::MatrixXd lu_vec = lu_vec_[i];
    const Eigen::MatrixXd luu_vec = luu_vec_[i];
    const Eigen::MatrixXd lxu_vec = lxu_vec_[i];
    // compute Q value
    Qu_.resize(1);
    Qx_.resize(4);
    Quu_.resize(1, 1);
    Qxx_.resize(4, 4);
    Qux_.resize(1, 4);
    Quuf_.resize(1, 1);
    K_i_.resize(1, 4);
    k_i_.resize(1);

    Qx_[0] = lx_vec(0, 0) + Vx(0, 0);
    Qx_[1] = lx_vec(1, 0) + Vx(1, 0);
    Qx_[2] =
        lx_vec(2, 0) + fx(0, 2) * Vx(0, 0) + fx(1, 2) * Vx(1, 0) + Vx(2, 0);

    Qx_[3] = lx_vec(3, 0) + fx(0, 3) * Vx(0, 0) + fx(1, 3) * Vx(1, 0) +
             fx(2, 3) * Vx(2, 0) + Vx(3, 0);

    Qu_[0] = lu_vec(0, 0) + fu(3, 0) * Vx(3, 0);

    const double tmp_q1 =
        Vxx(2, 0) + fx(0, 2) * Vxx(0, 0) + fx(1, 2) * Vxx(1, 0);

    const double tmp_q2 =
        Vxx(2, 1) + fx(0, 2) * Vxx(0, 1) + fx(1, 2) * Vxx(1, 1);

    const double tmp_q3 =
        Vxx(2, 2) + fx(0, 2) * Vxx(0, 2) + fx(1, 2) * Vxx(1, 2);

    const double tmp_q4 =
        Vxx(2, 3) + fx(0, 2) * Vxx(0, 3) + fx(1, 2) * Vxx(1, 3);

    const double tmp_p1 = Vxx(3, 0) + fx(0, 3) * Vxx(0, 0) +
                          fx(1, 3) * Vxx(1, 0) + fx(2, 3) * Vxx(2, 0);

    const double tmp_p2 = Vxx(3, 1) + fx(0, 3) * Vxx(0, 1) +
                          fx(1, 3) * Vxx(1, 1) + fx(2, 3) * Vxx(2, 1);

    const double tmp_p3 = Vxx(3, 2) + fx(0, 3) * Vxx(0, 2) +
                          fx(1, 3) * Vxx(1, 2) + fx(2, 3) * Vxx(2, 2);

    const double tmp_p4 = Vxx(3, 3) + fx(0, 3) * Vxx(0, 3) +
                          fx(1, 3) * Vxx(1, 3) + fx(2, 3) * Vxx(2, 3);

    Qxx_(0, 0) = lxx_vec(0, 0) + Vxx(0, 0);
    Qxx_(0, 1) = Vxx(0, 1);
    Qxx_(0, 2) = Vxx(0, 2) + fx(0, 2) * Vxx(0, 0) + fx(1, 2) * Vxx(0, 1);
    Qxx_(0, 3) = Vxx(0, 3) + fx(0, 3) * Vxx(0, 0) + fx(1, 3) * Vxx(0, 1) +
                 fx(2, 3) * Vxx(0, 2);

    Qxx_(1, 0) = Vxx(1, 0);
    Qxx_(1, 1) = Vxx(1, 1) + lxx_vec(1, 1);
    Qxx_(1, 2) = Vxx(1, 2) + fx(0, 2) * Vxx(1, 0) + fx(1, 2) * Vxx(1, 1);
    Qxx_(1, 3) = Vxx(1, 3) + fx(0, 3) * Vxx(1, 0) + fx(1, 3) * Vxx(1, 1) +
                 fx(2, 3) * Vxx(1, 2);

    Qxx_(2, 0) = tmp_q1;
    Qxx_(2, 1) = tmp_q2;
    Qxx_(2, 2) = lxx_vec(2, 2) + tmp_q3 + fx(0, 2) * tmp_q1 + fx(1, 2) * tmp_q2;
    Qxx_(2, 3) =
        tmp_q4 + fx(0, 3) * tmp_q1 + fx(1, 3) * tmp_q2 + fx(2, 3) * tmp_q3;

    Qxx_(3, 0) = tmp_p1;

    Qxx_(3, 1) = tmp_p2;

    Qxx_(3, 2) = tmp_p3 + fx(0, 2) * tmp_p1 + fx(1, 2) * tmp_p2;

    Qxx_(3, 3) = lxx_vec(3, 3) + tmp_p4 + fx(0, 3) * tmp_p1 +
                 fx(1, 3) * tmp_p2 + fx(2, 3) * tmp_p3;

    const double tmp_qu0 = Vxx(3, 0) * fu(3, 0);
    const double tmp_qu1 = Vxx(3, 1) * fu(3, 0);
    const double tmp_qu2 = Vxx(3, 2) * fu(3, 0);
    const double tmp_qu3 = Vxx(3, 3) * fu(3, 0);

    Quu_(0, 0) = luu_vec(0, 0) + fu(3, 0) * tmp_qu3;

    Qux_(0, 0) = lxu_vec(0, 0) + tmp_qu0;
    Qux_(0, 1) = lxu_vec(1, 0) + tmp_qu1;
    Qux_(0, 2) =
        lxu_vec(2, 0) + tmp_qu2 + fx(0, 2) * tmp_qu0 + fx(1, 2) * tmp_qu1;

    Qux_(0, 3) = lxu_vec(3, 0) + tmp_qu3 + fx(0, 3) * tmp_qu0 +
                 fx(1, 3) * tmp_qu1 + fx(2, 3) * tmp_qu2;

    // Similar to equations 10a and 10b in [Tassa 2012]. Note that
    // regularization is different
    Quuf_(0, 0) = Quu_(0, 0) + lambda_;
    if (!PSDCheck(Quuf_)) {
      std::cout << "wocao wocao = " << Quuf_ << std::endl;
      return false;
    }

    const auto QuuF_inv = 1.0 / Quuf_(0, 0);
    k_i_[0] = -QuuF_inv * Qu_[0];
    K_i_(0, 0) = -QuuF_inv * Qux_(0, 0);
    K_i_(0, 1) = -QuuF_inv * Qux_(0, 1);
    K_i_(0, 2) = -QuuF_inv * Qux_(0, 2);
    K_i_(0, 3) = -QuuF_inv * Qux_(0, 3);

    // Update cost-to-go approximation. Equations 11 in [Tassa 2012]
    dV_[0] += k_i_[0] * Qu_[0];
    dV_[1] += 0.5 * k_i_[0] * Quu_(0, 0) * k_i_[0];

    const double temp_v0 = K_i_(0, 0) * Quu_(0, 0) + Qux_(0, 0);
    const double temp_v1 = K_i_(0, 1) * Quu_(0, 0) + Qux_(0, 1);
    const double temp_v2 = K_i_(0, 2) * Quu_(0, 0) + Qux_(0, 2);
    const double temp_v3 = K_i_(0, 3) * Quu_(0, 0) + Qux_(0, 3);

    Vx(0, 0) = Qx_[0] + temp_v0 * k_i_[0] + K_i_(0, 0) * Qu_[0];
    Vx(1, 0) = Qx_[1] + temp_v1 * k_i_[0] + K_i_(0, 1) * Qu_[0];
    Vx(2, 0) = Qx_[2] + temp_v2 * k_i_[0] + K_i_(0, 2) * Qu_[0];
    Vx(3, 0) = Qx_[3] + temp_v3 * k_i_[0] + K_i_(0, 3) * Qu_[0];

    Vxx(0, 0) = Qxx_(0, 0) + temp_v0 * K_i_(0, 0) + K_i_(0, 0) * Qux_(0, 0);
    Vxx(0, 1) = Qxx_(0, 1) + temp_v0 * K_i_(0, 1) + K_i_(0, 0) * Qux_(0, 1);
    Vxx(0, 2) = Qxx_(0, 2) + temp_v0 * K_i_(0, 2) + K_i_(0, 0) * Qux_(0, 2);
    Vxx(0, 3) = Qxx_(0, 3) + temp_v0 * K_i_(0, 3) + K_i_(0, 0) * Qux_(0, 3);

    Vxx(1, 0) = Qxx_(1, 0) + temp_v1 * K_i_(0, 0) + K_i_(0, 1) * Qux_(0, 0);
    Vxx(1, 1) = Qxx_(1, 1) + temp_v1 * K_i_(0, 1) + K_i_(0, 1) * Qux_(0, 1);
    Vxx(1, 2) = Qxx_(1, 2) + temp_v1 * K_i_(0, 2) + K_i_(0, 1) * Qux_(0, 2);
    Vxx(1, 3) = Qxx_(1, 3) + temp_v1 * K_i_(0, 3) + K_i_(0, 1) * Qux_(0, 3);

    Vxx(2, 0) = Qxx_(2, 0) + temp_v2 * K_i_(0, 0) + K_i_(0, 2) * Qux_(0, 0);
    Vxx(2, 1) = Qxx_(2, 1) + temp_v2 * K_i_(0, 1) + K_i_(0, 2) * Qux_(0, 1);
    Vxx(2, 2) = Qxx_(2, 2) + temp_v2 * K_i_(0, 2) + K_i_(0, 2) * Qux_(0, 2);
    Vxx(2, 3) = Qxx_(2, 3) + temp_v2 * K_i_(0, 3) + K_i_(0, 2) * Qux_(0, 3);

    Vxx(3, 0) = Qxx_(3, 0) + temp_v3 * K_i_(0, 0) + K_i_(0, 3) * Qux_(0, 0);
    Vxx(3, 1) = Qxx_(3, 1) + temp_v3 * K_i_(0, 1) + K_i_(0, 3) * Qux_(0, 1);
    Vxx(3, 2) = Qxx_(3, 2) + temp_v3 * K_i_(0, 2) + K_i_(0, 3) * Qux_(0, 2);
    Vxx(3, 3) = Qxx_(3, 3) + temp_v3 * K_i_(0, 3) + K_i_(0, 3) * Qux_(0, 3);

    // save controls/gains
    k_vec_[i] = k_i_;
    K_vec_[i] = K_i_;
  }
  return true;
}

void ALiLQR::AliLqrIteration() {
  auto start_time = IflyTime::Now_ms();
  // outer iteration
  for (size_t al_iter = 0; al_iter < solver_config_ptr_->max_al_iter;
       ++al_iter) {
    // record al param
    for (int i = 0; i < static_cast<int>(solver_config_ptr_->horizon) + 1;
         i++) {
      solver_info_.lat_al_iteration_info.mu_hard_acc_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[L_ACC_HARDBOUND];
      solver_info_.lat_al_iteration_info.rho_hard_acc_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[W_ACC_HARDBOUND];
      solver_info_.lat_al_iteration_info.mu_hard_jerk_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[L_JERK_HARDBOUND];
      solver_info_.lat_al_iteration_info.rho_hard_jerk_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[W_JERK_HARDBOUND];
      solver_info_.lat_al_iteration_info.mu_hard_lower_pos_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[L_POS_LOWER_HARDBOUND];
      solver_info_.lat_al_iteration_info.rho_hard_lower_pos_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[W_POS_LOWER_HARDBOUND];
      solver_info_.lat_al_iteration_info.mu_hard_upper_pos_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[L_POS_UPPER_HARDBOUND];
      solver_info_.lat_al_iteration_info.rho_hard_upper_pos_iteration(
          al_iter, i) = alilqr_config_vec_ptr_->at(i)[W_POS_UPPER_HARDBOUND];
    }

    // constraint_violation
    solver_info_.constraint_violation = MaxConstraintViolation();

    // alilqr
    solver_info_.outer_iter_count = al_iter;

#ifdef __ILQR_PRINT__
    std::cout << "iteration = " << al_iter << std::endl;
    PrintAlParamInfo();
#endif
    iLqrIteration();

    // constraint_violation
    solver_info_.constraint_violation = MaxConstraintViolation();
    // derivation_error
    solver_info_.dcost_outer = MaxDerivationValue();

    //  print info when debug
#ifdef __ILQR_PRINT__
    PrintCostInfo();
    PrintAlSolverInfo();
    PrintAlParamInfoAfter();
    // PrintTimeInfo();
#endif
    // normal terminate
    auto end_time = IflyTime::Now_ms();
    double time_cost = end_time - start_time;
    // fail protection / can not optimization anymore
    if (solver_info_.solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
      break;
    } else if (solver_info_.constraint_violation <
               solver_config_ptr_->constraint_tolerance_tol) {
      solver_info_.solver_condition = iLqrSolveCondition::KKT_TERMINATE;
      break;
    } else if ((al_iter == solver_config_ptr_->max_al_iter - 1) || (time_cost > solver_config_ptr_->max_al_solve_time)){
      solver_info_.solver_condition =
          iLqrSolveCondition::MAX_OUTERITER_TERMINATE;
      break;
    }

    UpdateAugmentedLagragian();
    //solver_config_ptr_->cost_tol *= solver_config_ptr_->cost_scale;

    // init solver config  set to zero
    InitAliLqrSolverConfig();
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
  }
}

double ALiLQR::MaxConstraintViolation() {
  double max_violation = 0.0;
  for (size_t j = 0; j < solver_config_ptr_->horizon + 1; j++) {
    // k2v4
    const double k2v4 = Square(cost_config_vec_ptr_->at(j)[CURV_FACTOR]) *
                      SSquare(cost_config_vec_ptr_->at(j)[REF_VEL]);

    // acc bound
    solver_info_.lat_al_iteration_info.constraint_data_iteration(0, j) =
        std::max(k2v4 * (Square(xk_vec_[j][DELTA]) -
                                 Square(cost_config_vec_ptr_->at(j)[DELTA_UPPER_BOUND])),
                 -alilqr_config_vec_ptr_->at(j)[L_ACC_HARDBOUND] /
                     alilqr_config_vec_ptr_->at(j)[W_ACC_HARDBOUND]);

    // jerk bound
    solver_info_.lat_al_iteration_info.constraint_data_iteration(1, j) =
        std::max(k2v4 * (Square(uk_vec_[j][OMEGA]) -
                                 Square(cost_config_vec_ptr_->at(j)[OMEGA_UPPER_BOUND])),
                 -alilqr_config_vec_ptr_->at(j)[L_JERK_HARDBOUND] /
                     alilqr_config_vec_ptr_->at(j)[W_JERK_HARDBOUND]);

    // upper pos hard bound
    const double a1 = cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_Y1] -
                      cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_Y0];
    const double b1 = cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_X0] -
                      cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_X1];
    const double c1 = cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_Y0] *
                          cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_X1] -
                      cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_X0] *
                          cost_config_vec_ptr_->at(j)[HARD_UPPER_BOUND_Y1];
    // const double d1 = Square(a1) + Square(b1);

    solver_info_.lat_al_iteration_info.constraint_data_iteration(2, j) =
        std::max(-(a1 * xk_vec_[j][X] + b1 * xk_vec_[j][Y] + c1),
                 -alilqr_config_vec_ptr_->at(j)[L_POS_UPPER_HARDBOUND] /
                     alilqr_config_vec_ptr_->at(j)[W_POS_UPPER_HARDBOUND]);

    // lower pos hard bound
    const double a2 = cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_Y1] -
                      cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_Y0];
    const double b2 = cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_X0] -
                      cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_X1];
    const double c2 = cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_Y0] *
                          cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_X1] -
                      cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_X0] *
                          cost_config_vec_ptr_->at(j)[HARD_LOWER_BOUND_Y1];
    // const double d2 = Square(a2) + Square(b2); //@cailiu: reserve

    solver_info_.lat_al_iteration_info.constraint_data_iteration(3, j) =
        std::max(a2 * xk_vec_[j][X] + b2 * xk_vec_[j][Y] + c2,
                 -alilqr_config_vec_ptr_->at(j)[L_POS_LOWER_HARDBOUND] /
                     alilqr_config_vec_ptr_->at(j)[W_POS_LOWER_HARDBOUND]);
  }
  const auto constraint_data =
      solver_info_.lat_al_iteration_info.constraint_data_iteration.array().abs();
  const auto row_sum = constraint_data.rowwise().sum();
  max_violation =
      row_sum.maxCoeff();  // @cailiu: the max violation of all hard bound
  return max_violation;
}

void ALiLQR::UpdateAugmentedLagragian() {
  // update alilqr augmented parameters
  for (int i = 0; i < static_cast<int>(solver_config_ptr_->horizon) + 1; i++) {
    // k2v4
    const double k2v4 = Square(cost_config_vec_ptr_->at(i)[CURV_FACTOR]) *
                      SSquare(cost_config_vec_ptr_->at(i)[REF_VEL]);
    // acc
    const double l_acc = alilqr_config_vec_ptr_->at(i)[L_ACC_HARDBOUND];
    alilqr_config_vec_ptr_->at(i)[L_ACC_HARDBOUND] = std::max(
        0.0,
        l_acc + alilqr_config_vec_ptr_->at(i)[W_ACC_HARDBOUND] * k2v4 * (Square(xk_vec_[i][DELTA]) -
                                 Square(cost_config_vec_ptr_->at(i)[DELTA_UPPER_BOUND])));

    alilqr_config_vec_ptr_->at(i)[W_ACC_HARDBOUND] =
        Limit(alilqr_config_vec_ptr_->at(i)[W_ACC_HARDBOUND] *
                  alilqr_config_vec_ptr_->at(i)[PHI_SCALE],
              1e8);

    // jerk
    const double l_jerk = alilqr_config_vec_ptr_->at(i)[L_JERK_HARDBOUND];
    alilqr_config_vec_ptr_->at(i)[L_JERK_HARDBOUND] = std::max(
        0.0,
        l_jerk + alilqr_config_vec_ptr_->at(i)[W_JERK_HARDBOUND] * k2v4 * (Square(uk_vec_[i][OMEGA]) -
                                 Square(cost_config_vec_ptr_->at(i)[OMEGA_UPPER_BOUND])));

    alilqr_config_vec_ptr_->at(i)[W_JERK_HARDBOUND] =
        Limit(alilqr_config_vec_ptr_->at(i)[W_JERK_HARDBOUND] *
                  alilqr_config_vec_ptr_->at(i)[PHI_SCALE],
              1e8);

    // upper pos hard bound
    const double a1 = cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_Y1] -
                      cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_Y0];
    const double b1 = cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_X0] -
                      cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_X1];
    const double c1 = cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_Y0] *
                          cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_X1] -
                      cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_X0] *
                          cost_config_vec_ptr_->at(i)[HARD_UPPER_BOUND_Y1];
    const double l_hard_upper =
        alilqr_config_vec_ptr_->at(i)[L_POS_UPPER_HARDBOUND];

    alilqr_config_vec_ptr_->at(i)[L_POS_UPPER_HARDBOUND] =
        std::max(0.0, l_hard_upper +
                          alilqr_config_vec_ptr_->at(i)[W_POS_UPPER_HARDBOUND] *
                              (-(a1 * xk_vec_[i][X] + b1 * xk_vec_[i][Y] + c1)));

    alilqr_config_vec_ptr_->at(i)[W_POS_UPPER_HARDBOUND] =
        Limit(alilqr_config_vec_ptr_->at(i)[W_POS_UPPER_HARDBOUND] *
                  alilqr_config_vec_ptr_->at(i)[PHI_SCALE],
              1e6);

    // lower pos hard bound
    const double a2 = cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_Y1] -
                      cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_Y0];
    const double b2 = cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_X0] -
                      cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_X1];
    const double c2 = cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_Y0] *
                          cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_X1] -
                      cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_X0] *
                          cost_config_vec_ptr_->at(i)[HARD_LOWER_BOUND_Y1];
    const double l_hard_lower =
        alilqr_config_vec_ptr_->at(i)[L_POS_LOWER_HARDBOUND];
    alilqr_config_vec_ptr_->at(i)[L_POS_LOWER_HARDBOUND] = std::max(
        0.0,
        l_hard_lower + alilqr_config_vec_ptr_->at(i)[W_POS_LOWER_HARDBOUND] *
                           (a2 * xk_vec_[i][X] + b2 * xk_vec_[i][Y] + c2));

    alilqr_config_vec_ptr_->at(i)[W_POS_LOWER_HARDBOUND] =
        Limit(alilqr_config_vec_ptr_->at(i)[W_POS_LOWER_HARDBOUND] *
                  alilqr_config_vec_ptr_->at(i)[PHI_SCALE],
              1e6);
  }
}

double ALiLQR::MaxDerivationValue() {
  UpdateDynamicsDerivatives();
  Eigen::VectorXd state_derivation_sum(
      static_cast<int>(solver_config_ptr_->state_size));
  state_derivation_sum.setZero();
  Eigen::VectorXd input_derivation_sum(
      static_cast<int>(solver_config_ptr_->input_size));
  input_derivation_sum.setZero();
  for (size_t i = 0; i < solver_config_ptr_->horizon + 1; i++) {
    state_derivation_sum =
        state_derivation_sum.array().abs() + lx_vec_[i].array().abs();
    input_derivation_sum =
        input_derivation_sum.array().abs() + lu_vec_[i].array().abs();
  }
  double derivation_error = std::max(state_derivation_sum.maxCoeff(),
                                     input_derivation_sum.maxCoeff());
  return derivation_error;
}

void ALiLQR::PrintAlParamInfo() {
  std::cout << "-------------------------------- 0. AliLqr "
               "augumented lagrangian params info "
               "-------------------------------- "
            << std::endl;
  std::cout << "mu acc bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.mu_hard_acc_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "rho acc bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.rho_hard_acc_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "mu jerk bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.mu_hard_jerk_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "rho jerk bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.rho_hard_jerk_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "mu upper bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.mu_hard_upper_pos_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "rho upper bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.rho_hard_upper_pos_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "mu lower bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.mu_hard_lower_pos_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "rho lower bound : " << std::endl;
  std::cout << solver_info_.lat_al_iteration_info.rho_hard_lower_pos_iteration.row(
                   solver_info_.outer_iter_count)
            << std::endl;

  std::cout << "constraint error data :" << std::endl;
  std::cout << "error acc bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(0)
            << std::endl;
  std::cout << "error jerk bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(1)
            << std::endl;
  std::cout << "error upper bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(2)
            << std::endl;
  std::cout << "error lower bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(3)
            << std::endl;
}

void ALiLQR::PrintAlParamInfoAfter() {
  std::cout << "\n-------------------------------- 3. constraint error data "
               "after ilqr "
               "--------------------------------"
            << std::endl;
  std::cout << "error acc bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(0)
            << std::endl;
  std::cout << "error jerk bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(1)
            << std::endl;
  std::cout << "error upper bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(2)
            << std::endl;
  std::cout << "error lower bound : \n"
            << solver_info_.lat_al_iteration_info.constraint_data_iteration.row(3)
            << std::endl;
}
}  // namespace lateral_planning
}  // namespace pnc