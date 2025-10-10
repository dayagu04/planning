
#include "piecewise_jerk_speed_problem.h"

#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "src/common/planning_gflags.h"

namespace planning {
PiecewiseJerkSpeedProblem::PiecewiseJerkSpeedProblem(
    const size_t num_of_knots, const c_float delta_s,
    const std::array<c_float, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {
  penalty_dx_.resize(num_of_knots_, 0.0);
}

void PiecewiseJerkSpeedProblem::Init(const size_t num_of_knots,
                                     const c_float delta_s,
                                     const std::array<c_float, 3>& x_init) {
  Init(num_of_knots, delta_s, x_init);

  penalty_dx_.resize(num_of_knots_, 0.0);

  return;
}

void PiecewiseJerkSpeedProblem::set_init_state(
    const std::array<c_float, 3>& x_init) {
  x_init_ = x_init;
  return;
}

void PiecewiseJerkSpeedProblem::set_dx_ref(const c_float weight_dx_ref,
                                           const std::vector<c_float>& dx_ref) {
  weight_dx_ref_ = weight_dx_ref;
  dx_ref_ = dx_ref;
  has_dx_ref_ = true;

  return;
}

void PiecewiseJerkSpeedProblem::set_penalty_dx(
    std::vector<c_float> penalty_dx) {
  CHECK_EQ(penalty_dx.size(), num_of_knots_);
  penalty_dx_ = std::move(penalty_dx);

  return;
}

void PiecewiseJerkSpeedProblem::CalculateKernel(std::vector<c_float>* P_data,
                                                std::vector<c_int>* P_indices,
                                                std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  // N*3, for x =[x, x', x'']
  const int kNumParam = 3 * n;

  // P matrix is a sparse matrix, and it's max valid rows is 2.
  // std::pair<c_int, c_float>: rows, weight
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // x(i)^2 * w_x_ref
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, weight_x_ref_);
    ++value_index;
  }

  // x(n-1)^2 * (w_x_ref + w_end_x)
  columns[n - 1].emplace_back(n - 1, (weight_x_ref_ + weight_end_state_[0]));
  ++value_index;

  // x(i)'^2 * (w_dx_ref + penalty_dx)
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(n + i, (weight_dx_ref_ + penalty_dx_[i]));
    ++value_index;
  }

  // x(n-1)'^2 * (w_dx_ref + penalty_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(
      2 * n - 1, (weight_dx_ref_ + penalty_dx_[n - 1] + weight_end_state_[1]));
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  c_float scale_square = 1.0f;

  // x(i)''^2 * (w_ddx + w_dddx / delta_s^2), n = 0
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square));
  ++value_index;

  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square));
    ++value_index;
  }

  // acc, point n-1
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]));
  ++value_index;

  // - 2 w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // 对角线的下一行
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    -2.0 * weight_dddx_ / delta_s_square);
    ++value_index;
  }

  // // - w_dddx / delta_s^2 * x(i+1)'' * x(i)''
  // for (int i = 0; i < n - 1; ++i) {
  //   columns[2 * n + i + 1].emplace_back(
  //       2 * n + i, -weight_dddx_ / delta_s_square / scale_square);
  //   ++value_index;
  // }

  P_data->reserve(value_index);
  P_indptr->reserve(kNumParam + 1);
  P_indices->reserve(value_index);
  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    P_indptr->emplace_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->emplace_back(row_data_pair.second * 2.0f);
      P_indices->emplace_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);

  return;
}

void PiecewiseJerkSpeedProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam);
  for (int i = 0; i < n; ++i) {
    if (has_x_ref_) {
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i];
    }
    if (has_dx_ref_) {
      q->at(n + i) += -2.0 * weight_dx_ref_ * dx_ref_[i];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) += -2.0 * weight_end_state_[0] * end_state_[0];
    q->at(2 * n - 1) += -2.0 * weight_end_state_[1] * end_state_[1];
    q->at(3 * n - 1) += -2.0 * weight_end_state_[2] * end_state_[2];
  }

  return;
}

OSQPSettings* PiecewiseJerkSpeedProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->eps_abs = 1e-4;
  settings->eps_rel = 1e-4;
  settings->eps_prim_inf = 1e-5;
  settings->eps_dual_inf = 1e-5;
  settings->polish = true;
  settings->verbose = false;
  settings->scaled_termination = true;

  return settings;
}

void PiecewiseJerkSpeedProblem::CalculateKernel2(std::vector<c_float>* P_data,
                                                 std::vector<c_int>* P_indices,
                                                 std::vector<c_int>* P_indptr) {
  const c_int n = static_cast<c_int>(num_of_knots_);
  // N*3, for x =[x, x', x'']
  const c_int kNumParam = 3 * n;

  // P matrix is a sparse matrix, and it's max valid rows is 2.
  // std::pair<c_int, c_float>: rows, weight
  std::vector<SparseMatrixColumn> columns;
  columns.resize(kNumParam, SparseMatrixColumn(-1, 0.0f, -1, 0.0f));
  c_int value_index = 0;

  // ILOG_INFO << "value_index " << static_cast<int>(value_index);

  // x(i)^2 * w_x_ref
  for (c_int i = 0; i < n - 1; ++i) {
    // columns[i].emplace_back(i, weight_x_ref_);
    columns[i].diagonal_element.rows = i;
    columns[i].diagonal_element.value = weight_x_ref_;
    ++value_index;
  }

  // x(n-1)^2 * (w_x_ref + w_end_x)
  columns[n - 1].diagonal_element.rows = n - 1;
  columns[n - 1].diagonal_element.value = weight_x_ref_ + weight_end_state_[0];
  ++value_index;

  // x(i)'^2 * (w_dx_ref + penalty_dx)
  for (c_int i = 0; i < n - 1; ++i) {
    // columns[n + i].emplace_back(n + i, (weight_dx_ref_ + penalty_dx_[i]));

    columns[n + i].diagonal_element.rows = n + i;
    columns[n + i].diagonal_element.value = weight_dx_ref_ + penalty_dx_[i];
    ++value_index;
  }

  // x(n-1)'^2 * (w_dx_ref + penalty_dx + w_end_dx)
  // columns[2 * n - 1].emplace_back(
  //     2 * n - 1, (weight_dx_ref_ + penalty_dx_[n - 1] +
  //     weight_end_state_[1]));
  columns[2 * n - 1].diagonal_element.rows = 2 * n - 1;
  columns[2 * n - 1].diagonal_element.value =
      weight_dx_ref_ + penalty_dx_[n - 1] + weight_end_state_[1];
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  c_float scale_square = 1.0f;

  // x(i)''^2 * (w_ddx + w_dddx / delta_s^2), n = 0
  // columns[2 * n].emplace_back(2 * n,
  //                             (weight_ddx_ + weight_dddx_ / delta_s_square));
  columns[2 * n].diagonal_element.rows = 2 * n;
  columns[2 * n].diagonal_element.value =
      weight_ddx_ + weight_dddx_ / delta_s_square;
  ++value_index;

  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  for (c_int i = 1; i < n - 1; ++i) {
    // columns[2 * n + i].emplace_back(
    //     2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square));
    columns[2 * n + i].diagonal_element.rows = 2 * n + i;
    columns[2 * n + i].diagonal_element.value =
        weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square;
    ++value_index;
  }

  // acc, point n-1
  // columns[3 * n - 1].emplace_back(
  //     3 * n - 1,
  //     (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]));
  columns[3 * n - 1].diagonal_element.rows = 3 * n - 1;
  columns[3 * n - 1].diagonal_element.value =
      weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2];
  ++value_index;

  // - 2 w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // 对角线的下一行
  for (c_int i = 0; i < n - 1; ++i) {
    // columns[2 * n + i].emplace_back(2 * n + i + 1,
    //                                 -2.0 * weight_dddx_ / delta_s_square);
    columns[2 * n + i].next_row_element.rows = 2 * n + i + 1;
    columns[2 * n + i].next_row_element.value =
        -2.0 * weight_dddx_ / delta_s_square;
    ++value_index;
  }

  // // - w_dddx / delta_s^2 * x(i+1)'' * x(i)''
  // for (int i = 0; i < n - 1; ++i) {
  //   columns[2 * n + i + 1].emplace_back(
  //       2 * n + i, -weight_dddx_ / delta_s_square / scale_square);
  //   ++value_index;
  // }

  // ILOG_INFO << "value_index " << static_cast<int>(value_index);

  P_data->reserve(value_index);
  P_indptr->reserve(kNumParam + 1);
  P_indices->reserve(value_index);
  c_int ind_p = 0;
  for (c_int i = 0; i < kNumParam; ++i) {
    P_indptr->emplace_back(ind_p);

    if (columns[i].diagonal_element.IsValid()) {
      P_data->emplace_back(columns[i].diagonal_element.value * 2.0f);
      P_indices->emplace_back(columns[i].diagonal_element.rows);
      ++ind_p;
    }

    if (columns[i].next_row_element.IsValid()) {
      P_data->emplace_back(columns[i].next_row_element.value * 2.0f);
      P_indices->emplace_back(columns[i].next_row_element.rows);
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);

  // ILOG_INFO << "ind_p " << static_cast<int>(ind_p);

  return;
}

}  // namespace planning
