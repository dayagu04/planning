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
      du_norm_vec_[i] = du.norm();

      new_cost += ilqr_model_ptr_->GetCost(xk_new_vec_[i], uk_new_vec_[i], i);

      xk_new_vec_[i + 1] = ilqr_model_ptr_->UpdateDynamicsOneStep(
          xk_new_vec_[i], uk_new_vec_[i], i);

      double tmp_k = 0.0;
      const auto k_com = xk_new_vec_[i + 1][StateId::K];
      tmp_k = pnc::mathlib::Limit(k_com, 1.0 / 5.5);
      xk_new_vec_[i + 1][StateId::K] = tmp_k;

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

}  // namespace apa_planner
}  // namespace planning