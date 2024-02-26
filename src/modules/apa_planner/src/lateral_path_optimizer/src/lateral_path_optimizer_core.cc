#include "lateral_path_optimizer_core.h"

#include <iostream>
#include <numeric>

#include "ilqr_core.h"
#include "math_lib.h"
#include "src/lateral_path_optimizer_cost.h"

namespace planning {
namespace apa_planner {

bool LateralPathOptimizerCore::ForwardPass(double &new_cost, double &expected,
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
bool LateralPathOptimizerCore::BackwardPass() {
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
}  // namespace apa_planner
}  // namespace planning