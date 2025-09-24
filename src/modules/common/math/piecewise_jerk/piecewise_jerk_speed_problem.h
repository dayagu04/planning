
#pragma once

#include <utility>
#include <vector>

#include "piecewise_jerk_problem.h"

namespace planning {

// matrix is sparse, so store it's key elements.
struct SparseMatrixElement {
  c_int rows = -1;
  c_float value = 0.0f;

  const bool IsValid() const { return rows > -1 ? true : false; }
};

// matrix is sparse, max number in a column is two.
struct SparseMatrixColumn {
  SparseMatrixElement diagonal_element;
  // this element is next row for diagonal_element
  SparseMatrixElement next_row_element;

  SparseMatrixColumn() = default;
  SparseMatrixColumn(const c_int diagonal_rows, const c_float diagonal_v,
                     const c_int next_rows, const c_float next_v) {
    diagonal_element.rows = diagonal_rows;
    diagonal_element.value = diagonal_v;
    next_row_element.rows = next_rows;
    next_row_element.value = next_v;
  }
};

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

  void CalculateKernel2(std::vector<c_float>* P_data,
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
