
#pragma once

#include <utility>
#include <vector>

#include "piecewise_jerk_problem.h"

namespace planning {

// point size must bigger than 1.
class PiecewiseJerkSpeedProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkSpeedProblem(const size_t num_of_knots, const c_float delta_s,
                            const std::array<c_float, 3>& x_init);

  void Init(const size_t num_of_knots, const c_float delta_s,
            const std::array<c_float, 3>& x_init);

  void set_init_state(const std::array<c_float, 3>& x_init);

  virtual ~PiecewiseJerkSpeedProblem() = default;

  void set_dx_ref(const c_float weight_dx_ref, std::vector<c_float>& dx_ref);

  void set_penalty_dx(std::vector<c_float> penalty_dx);

 protected:
  // naming convention follows osqp solver.
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  OSQPSettings* SolverDefaultSettings() override;

  bool has_dx_ref_ = false;
  // 速度系数为0
  c_float weight_dx_ref_ = 0.0;
  std::vector<c_float> dx_ref_;

  // v cost, not (v-v_ref) cost. Normally, these weights are zero.
  std::vector<c_float> penalty_dx_;
};

}  // namespace planning
