

#pragma once

#include <array>
#include <tuple>
#include <utility>
#include <vector>

#include "log_glog.h"
#include "osqp.h"

namespace planning {
/*
 * @brief:
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

// point size must bigger than 1.
class PiecewiseJerkProblem {
 public:
  PiecewiseJerkProblem(const size_t num_of_knots, const c_float delta_s,
                       const std::array<c_float, 3>& x_init);

  void Init(const size_t num_of_knots, const c_float delta_s,
            const std::array<c_float, 3>& x_init);

  virtual ~PiecewiseJerkProblem() = default;

  void set_x_bounds(std::vector<std::pair<c_float, c_float>> x_bounds);

  void set_x_bounds(const c_float x_lower_bound, const c_float x_upper_bound);

  void set_dx_bounds(std::vector<std::pair<c_float, c_float>> dx_bounds);

  void set_dx_bounds(const c_float dx_lower_bound,
                     const c_float dx_upper_bound);

  void set_ddx_bounds(std::vector<std::pair<c_float, c_float>> ddx_bounds);

  // deceleration:(-inf, 0]
  // acceleration:[0, inf)
  void set_ddx_bounds(const c_float ddx_lower_bound,
                      const c_float ddx_upper_bound);

  void set_dddx_bound(const c_float dddx_bound) {
    set_dddx_bound(-dddx_bound, dddx_bound);
  }

  void set_dddx_bound(const c_float dddx_lower_bound,
                      const c_float dddx_upper_bound) {
    dddx_bound_.first = dddx_lower_bound;
    dddx_bound_.second = dddx_upper_bound;
  }

  void set_weight_x(const c_float weight_x) { weight_x_ = weight_x; }

  void set_weight_dx(const c_float weight_dx) { weight_dx_ = weight_dx; }

  void set_weight_ddx(const c_float weight_ddx) { weight_ddx_ = weight_ddx; }

  void set_weight_dddx(const c_float weight_dddx) {
    weight_dddx_ = weight_dddx;
  }

  void set_scale_factor(const std::array<c_float, 3>& scale_factor) {
    scale_factor_ = scale_factor;
  }

  /**
   * @brief Set the x ref object and the uniform x_ref weighting
   *
   * @param weight_x_ref: uniform weighting for x_ref
   * @param x_ref: objective value of x
   */
  void set_x_ref(const c_float weight_x_ref, std::vector<c_float> x_ref);

  void set_end_state_ref(const std::array<c_float, 3>& weight_end_state,
                         const std::array<c_float, 3>& end_state_ref);

  virtual bool Optimize(const int max_iter, const c_float max_time);

  const std::vector<c_float>& opt_x() const { return x_; }

  const std::vector<c_float>& opt_dx() const { return dx_; }

  const std::vector<c_float>& opt_ddx() const { return ddx_; }

  void DebugString();

  void set_end_state_constriants(const std::array<c_float, 3>& end_state_ref);

 protected:
  // naming convention follows osqp solver.
  // square cost, hessian matrix
  virtual void CalculateKernel(std::vector<c_float>* P_data,
                               std::vector<c_int>* P_indices,
                               std::vector<c_int>* P_indptr) = 0;

  // linear cost
  virtual void CalculateOffset(std::vector<c_float>* q) = 0;

  virtual void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                         std::vector<c_int>* A_indices,
                                         std::vector<c_int>* A_indptr,
                                         std::vector<c_float>* lower_bounds,
                                         std::vector<c_float>* upper_bounds);

  virtual OSQPSettings* SolverDefaultSettings();

  OSQPData* FormulateProblem();

  void FreeData(OSQPData* data);

  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

 protected:
  size_t num_of_knots_ = 0;

  // output
  std::vector<c_float> x_;
  std::vector<c_float> dx_;
  std::vector<c_float> ddx_;

  std::array<c_float, 3> x_init_;
  std::array<c_float, 3> scale_factor_ = {{1.0, 1.0, 1.0}};

  std::vector<std::pair<c_float, c_float>> x_bounds_;
  std::vector<std::pair<c_float, c_float>> dx_bounds_;
  std::vector<std::pair<c_float, c_float>> ddx_bounds_;
  std::pair<c_float, c_float> dddx_bound_;

  c_float weight_x_ = 0.0;
  c_float weight_dx_ = 0.0;
  c_float weight_ddx_ = 0.0;
  c_float weight_dddx_ = 0.0;

  // For lon optimization, delta_s is time;
  // For lateral optimization, delta_s is distance;
  c_float delta_s_ = 1.0;

  bool has_x_ref_ = false;
  c_float weight_x_ref_ = 0.0;
  std::vector<c_float> x_ref_;

  bool has_end_state_ref_ = false;
  std::array<c_float, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
  std::array<c_float, 3> end_state_;

  bool has_end_state_constriants_ = false;
};

}  // namespace planning
