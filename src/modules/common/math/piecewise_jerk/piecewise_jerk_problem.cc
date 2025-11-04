
#include "piecewise_jerk_problem.h"

#include <chrono>

#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "time_benchmark.h"

namespace planning {

constexpr c_float kMaxVariableRange = 10000.0;

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const c_float delta_s,
    const std::array<c_float, 3>& x_init) {
  num_of_knots_ = num_of_knots;
  x_init_ = x_init;
  delta_s_ = delta_s;

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));
}

void PiecewiseJerkProblem::Init(const size_t num_of_knots,
                                const c_float delta_s,
                                const std::array<c_float, 3>& x_init) {
  if (num_of_knots < 2) {
    return;
  }

  num_of_knots_ = num_of_knots;
  x_init_ = x_init;
  delta_s_ = delta_s;

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  return;
}

OSQPData* PiecewiseJerkProblem::FormulateProblem() {
  // calculate kernel
  static std::vector<c_float> P_data;
  static std::vector<c_int> P_indices;
  static std::vector<c_int> P_indptr;
  P_data.clear();
  P_indices.clear();
  P_indptr.clear();

  CalculateKernel2(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  static std::vector<c_float> A_data;
  static std::vector<c_int> A_indices;
  static std::vector<c_int> A_indptr;
  A_data.clear();
  A_indices.clear();
  A_indptr.clear();

  // ILOG_INFO << "A_indptr ";

  static std::vector<c_float> lower_bounds;
  static std::vector<c_float> upper_bounds;
  lower_bounds.clear();
  upper_bounds.clear();

  // ILOG_INFO << "upper_bounds ";
  CalculateAffineConstraint2(&A_data, &A_indices, &A_indptr, &lower_bounds,
                             &upper_bounds);

  // calculate offset
  static std::vector<c_float> q;
  q.clear();
  CalculateOffset(&q);
  // ILOG_INFO << "CalculateOffset ";

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  // CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  c_int kernel_dim = 3 * num_of_knots_;
  c_int num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P =
      csc_matrix(kernel_dim, kernel_dim, static_cast<c_int>(P_data.size()),
                 P_data.data(), P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(num_affine_constraint, kernel_dim,
                       static_cast<c_int>(A_data.size()), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  // ILOG_INFO << "OSQPData ";

  return data;
}

bool PiecewiseJerkProblem::Optimize(const int max_iter,
                                    const c_float max_time) {
  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  settings->max_iter = max_iter;
  settings->eps_abs = 1e-3;
  settings->eps_rel = 1e-3;
  // settings->eps_dual_inf = 1e-3;
  // settings->eps_prim_inf = 1e-3;
  // second
  settings->time_limit = max_time;

  // DebugString();
  OSQPWorkspace* osqp_work = nullptr;
  osqp_work = osqp_setup(data, settings);
  // osqp_setup(&osqp_work, data, settings);
  // ILOG_INFO << "osqp_setup ";

  double start_time = IflyTime::Now_us();
  osqp_solve(osqp_work);
  // ILOG_INFO << "osqp_solve ";

  double planning_cost_time = (IflyTime::Now_us() - start_time) / 1000;
  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_OSQP,
                                    planning_cost_time);

  // ILOG_INFO << "osqp iter = " << osqp_work->info->iter;

  auto status = osqp_work->info->status_val;

  if (status < 0 || (status != 1 && status != 2) ||
      osqp_work->solution == nullptr) {
    ILOG_ERROR << "failed optimization status: " << status << ", "
               << osqp_work->info->status;

    FreeData(data, osqp_work, settings);

    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_];
    ddx_.at(i) = osqp_work->solution->x[i + 2 * num_of_knots_];
  }

  // Cleanup
  FreeData(data, osqp_work, settings);
  // ILOG_INFO << "FreeData ";

  return true;
}

void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3N boundary constraints on x, x', x''
  // 3 init constraints on x_init_
  // 3(N-1) continous constraints on x, x', x''
  // 3 end state constraints
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;

  // check end state constraints
  int end_state_constraint_num = 0;
  if (has_end_state_constriants_) {
    end_state_constraint_num = 3;
  }

  int num_of_constraints =
      num_of_variables + 3 * (n - 1) + 3 + end_state_constraint_num;

  // L<Ax<U; lower bounds record L
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  // variables: record A matrix columns
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  // Eigen::MatrixXf matrix =
  //     Eigen::MatrixXf::Zero(num_of_constraints, num_of_variables);
  int csc_matrix_valid_num = 0;

  int matrix_row_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      // x
      variables[i].emplace_back(matrix_row_index, 1.0);
      lower_bounds->at(matrix_row_index) = x_bounds_[i].first;
      upper_bounds->at(matrix_row_index) = x_bounds_[i].second;
    } else if (i < 2 * n) {
      // x'
      variables[i].emplace_back(matrix_row_index, 1.0);

      lower_bounds->at(matrix_row_index) = dx_bounds_[i - n].first;
      upper_bounds->at(matrix_row_index) = dx_bounds_[i - n].second;
    } else {
      // x''
      variables[i].emplace_back(matrix_row_index, 1.0);

      lower_bounds->at(matrix_row_index) = ddx_bounds_[i - 2 * n].first;
      upper_bounds->at(matrix_row_index) = ddx_bounds_[i - 2 * n].second;
    }
    ++matrix_row_index;
    csc_matrix_valid_num++;
  }
  CHECK_EQ(matrix_row_index, num_of_variables);

  // continous constraints
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(matrix_row_index, -1.0);
    variables[2 * n + i + 1].emplace_back(matrix_row_index, 1.0);

    lower_bounds->at(matrix_row_index) = dddx_bound_.first * delta_s_;
    upper_bounds->at(matrix_row_index) = dddx_bound_.second * delta_s_;
    ++matrix_row_index;
    csc_matrix_valid_num += 2;
  }

  // continuous constraints
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(matrix_row_index, -1.0);
    variables[n + i + 1].emplace_back(matrix_row_index, 1.0);
    variables[2 * n + i].emplace_back(matrix_row_index, -0.5 * delta_s_);
    variables[2 * n + i + 1].emplace_back(matrix_row_index, -0.5 * delta_s_);

    lower_bounds->at(matrix_row_index) = 0.0;
    upper_bounds->at(matrix_row_index) = 0.0;
    ++matrix_row_index;
    csc_matrix_valid_num += 4;
  }

  // continous constraints
  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(matrix_row_index, -1.0);
    variables[i + 1].emplace_back(matrix_row_index, 1.0);
    variables[n + i].emplace_back(matrix_row_index, -delta_s_);
    variables[2 * n + i].emplace_back(matrix_row_index, -delta_s_sq_ / 3.0);
    variables[2 * n + i + 1].emplace_back(matrix_row_index, -delta_s_sq_ / 6.0);

    lower_bounds->at(matrix_row_index) = 0.0;
    upper_bounds->at(matrix_row_index) = 0.0;
    ++matrix_row_index;
    csc_matrix_valid_num += 5;
  }

  // constrain on x_init
  variables[0].emplace_back(matrix_row_index, 1.0);
  lower_bounds->at(matrix_row_index) = x_init_[0];
  upper_bounds->at(matrix_row_index) = x_init_[0];
  ++matrix_row_index;
  csc_matrix_valid_num++;

  variables[n].emplace_back(matrix_row_index, 1.0);
  lower_bounds->at(matrix_row_index) = x_init_[1];
  upper_bounds->at(matrix_row_index) = x_init_[1];
  ++matrix_row_index;
  csc_matrix_valid_num++;

  c_float ddx_slack_bound = 0.08;
  variables[2 * n].emplace_back(matrix_row_index, 1.0);
  lower_bounds->at(matrix_row_index) = x_init_[2] - ddx_slack_bound;
  upper_bounds->at(matrix_row_index) = x_init_[2] + ddx_slack_bound;
  ++matrix_row_index;
  csc_matrix_valid_num++;

  // constraints on end state
  if (has_end_state_constriants_) {
    c_float s_slack_bound = 0.03;
    variables[n - 1].emplace_back(matrix_row_index, 1.0);
    lower_bounds->at(matrix_row_index) = end_state_[0] - s_slack_bound;
    upper_bounds->at(matrix_row_index) = end_state_[0] + s_slack_bound;
    ++matrix_row_index;

    c_float dx_slack_bound = 0.03;
    variables[2 * n - 1].emplace_back(matrix_row_index, 1.0);
    lower_bounds->at(matrix_row_index) = end_state_[1] - dx_slack_bound;
    upper_bounds->at(matrix_row_index) = end_state_[1] + dx_slack_bound;
    ++matrix_row_index;

    c_float ddx_slack_bound = 0.15;
    variables[3 * n - 1].emplace_back(matrix_row_index, 1.0);
    lower_bounds->at(matrix_row_index) = end_state_[2] - ddx_slack_bound;
    upper_bounds->at(matrix_row_index) = end_state_[2] + ddx_slack_bound;
    ++matrix_row_index;
    csc_matrix_valid_num += 3;
  }

  CHECK_EQ(matrix_row_index, num_of_constraints);

  A_data->reserve(csc_matrix_valid_num);
  A_indices->reserve(csc_matrix_valid_num);
  A_indptr->reserve(num_of_variables + 1);
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->emplace_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->emplace_back(variable_nz.second);

      // constraint index
      A_indices->emplace_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->emplace_back(ind_p);

  return;
}

OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  if (settings == nullptr) {
    ILOG_ERROR << "failed to allocate memory for OSQPSettings!";
    return nullptr;
  }

  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = false;
  settings->scaled_termination = true;
  return settings;
}

void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<c_float, c_float>> x_bounds) {
  CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);

  return;
}

void PiecewiseJerkProblem::set_dx_bounds(
    const std::vector<std::pair<c_float, c_float>>& dx_bounds) {
  dx_bounds_ = dx_bounds;

  return;
}

void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<c_float, c_float>> ddx_bounds) {
  CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);

  return;
}

void PiecewiseJerkProblem::set_x_bounds(const c_float x_lower_bound,
                                        const c_float x_upper_bound) {
  for (auto& x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }

  return;
}

void PiecewiseJerkProblem::set_dx_bounds(const c_float dx_lower_bound,
                                         const c_float dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }

  return;
}

void PiecewiseJerkProblem::set_ddx_bounds(const c_float ddx_lower_bound,
                                          const c_float ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }

  return;
}

void PiecewiseJerkProblem::set_x_ref(const c_float weight_x_ref,
                                     const std::vector<c_float>& x_ref) {
  weight_x_ref_ = weight_x_ref;
  x_ref_ = x_ref;
  has_x_ref_ = true;

  return;
}

void PiecewiseJerkProblem::set_end_state_ref(
    const std::array<c_float, 3>& weight_end_state,
    const std::array<c_float, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ = end_state_ref;
  has_end_state_ref_ = true;

  return;
}

void PiecewiseJerkProblem::set_end_state_constriants(
    const std::array<c_float, 3>& end_state_ref) {
  end_state_ = end_state_ref;
  has_end_state_constriants_ = true;

  return;
}

void PiecewiseJerkProblem::FreeData(OSQPData* data, OSQPWorkspace* work,
                                    OSQPSettings* setting) {
  osqp_cleanup(work);
  c_free(data);
  c_free(setting);

  return;
}

void PiecewiseJerkProblem::DebugString() {
  ILOG_INFO << "size = " << num_of_knots_;
  ILOG_INFO << "init0, s = " << x_init_[0];
  ILOG_INFO << "init1, v = " << x_init_[1];
  ILOG_INFO << "init2, acc = " << x_init_[2];
  ILOG_INFO << "delta_x = " << delta_s_;

  ILOG_INFO << "x bound";
  for (size_t i = 0; i < x_bounds_.size(); i++) {
    ILOG_INFO << "t=" << delta_s_ * i
              << ", lower, upper = " << x_bounds_.at(i).first << ", "
              << x_bounds_.at(i).second;
  }

  ILOG_INFO << "dx bound";
  for (size_t i = 0; i < dx_bounds_.size(); i++) {
    ILOG_INFO << "lower, upper = " << dx_bounds_.at(i).first << ", "
              << dx_bounds_.at(i).second;
  }

  ILOG_INFO << "ddx bound";
  for (size_t i = 0; i < ddx_bounds_.size(); i++) {
    ILOG_INFO << "lower,upper = " << ddx_bounds_.at(i).first << ", "
              << ddx_bounds_.at(i).second;
  }

  ILOG_INFO << "dddx bound";
  ILOG_INFO << "lower, upper = " << dddx_bound_.first << ", "
            << dddx_bound_.second;

  return;
}

void PiecewiseJerkProblem::CalculateAffineConstraint2(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3N boundary constraints on x, x', x''
  // 3 init constraints on x_init_
  // 3(N-1) continous constraints on x, x', x''
  // 3 end state constraints
  const c_int n = static_cast<c_int>(num_of_knots_);
  const c_int num_of_variables = 3 * n;

  // check end state constraints
  c_int end_state_constraint_num = 0;
  if (has_end_state_constriants_) {
    end_state_constraint_num = 3;
  }

  c_int num_of_constraints =
      num_of_variables + 3 * (n - 1) + 3 + end_state_constraint_num;

  // L<Ax<U; lower bounds record L
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  // ILOG_INFO << "size " << static_cast<int>(num_of_constraints);

  // variables: record A matrix columns
  Eigen::MatrixXf matrix =
      Eigen::MatrixXf::Zero(num_of_constraints, num_of_variables);
  int csc_matrix_valid_num = 0;

  // ILOG_INFO << "num_of_variables " << static_cast<int>(num_of_variables);

  int matrix_row_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      // x
      matrix(matrix_row_index, i) = 1.0f;
      lower_bounds->at(matrix_row_index) = x_bounds_[i].first;
      upper_bounds->at(matrix_row_index) = x_bounds_[i].second;
    } else if (i < 2 * n) {
      // x'
      matrix(matrix_row_index, i) = 1.0f;

      lower_bounds->at(matrix_row_index) = dx_bounds_[i - n].first;
      upper_bounds->at(matrix_row_index) = dx_bounds_[i - n].second;
    } else {
      // x''
      matrix(matrix_row_index, i) = 1.0f;

      lower_bounds->at(matrix_row_index) = ddx_bounds_[i - 2 * n].first;
      upper_bounds->at(matrix_row_index) = ddx_bounds_[i - 2 * n].second;
    }
    ++matrix_row_index;
    csc_matrix_valid_num++;
  }
  // CHECK_EQ(matrix_row_index, num_of_variables);

  // continous constraints
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    matrix(matrix_row_index, 2 * n + i) = -1.0f;
    matrix(matrix_row_index, 2 * n + i + 1) = 1.0f;

    lower_bounds->at(matrix_row_index) = dddx_bound_.first * delta_s_;
    upper_bounds->at(matrix_row_index) = dddx_bound_.second * delta_s_;
    ++matrix_row_index;
    csc_matrix_valid_num += 2;
  }

  // continuous constraints
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    matrix(matrix_row_index, n + i) = -1.0f;
    matrix(matrix_row_index, n + i + 1) = 1.0f;
    matrix(matrix_row_index, 2 * n + i) = -0.5f * delta_s_;
    matrix(matrix_row_index, 2 * n + i + 1) = -0.5f * delta_s_;

    lower_bounds->at(matrix_row_index) = 0.0f;
    upper_bounds->at(matrix_row_index) = 0.0f;
    ++matrix_row_index;
    csc_matrix_valid_num += 4;
  }

  // continous constraints
  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)'' = 0
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    matrix(matrix_row_index, i) = -1.0f;
    matrix(matrix_row_index, i + 1) = 1.0f;
    matrix(matrix_row_index, n + i) = -delta_s_;
    matrix(matrix_row_index, 2 * n + i) = -delta_s_sq_ / 3.0f;
    matrix(matrix_row_index, 2 * n + i + 1) = -delta_s_sq_ / 6.0f;
    lower_bounds->at(matrix_row_index) = 0.0f;
    upper_bounds->at(matrix_row_index) = 0.0f;
    ++matrix_row_index;
    csc_matrix_valid_num += 5;
  }

  // constrain on x_init
  matrix(matrix_row_index, 0) = 1.0f;
  lower_bounds->at(matrix_row_index) = x_init_[0];
  upper_bounds->at(matrix_row_index) = x_init_[0];
  ++matrix_row_index;
  csc_matrix_valid_num++;

  matrix(matrix_row_index, n) = 1.0f;
  lower_bounds->at(matrix_row_index) = x_init_[1];
  upper_bounds->at(matrix_row_index) = x_init_[1];
  ++matrix_row_index;
  csc_matrix_valid_num++;

  c_float ddx_slack_bound = 0.08f;
  matrix(matrix_row_index, 2 * n) = 1.0f;
  lower_bounds->at(matrix_row_index) = x_init_[2] - ddx_slack_bound;
  upper_bounds->at(matrix_row_index) = x_init_[2] + ddx_slack_bound;
  ++matrix_row_index;
  csc_matrix_valid_num++;

  // constraints on end state
  if (has_end_state_constriants_) {
    c_float s_slack_bound = 0.03f;
    matrix(matrix_row_index, n - 1) = 1.0f;
    lower_bounds->at(matrix_row_index) = end_state_[0] - s_slack_bound;
    upper_bounds->at(matrix_row_index) = end_state_[0] + s_slack_bound;
    ++matrix_row_index;

    c_float dx_slack_bound = 0.03f;
    matrix(matrix_row_index, 2 * n - 1) = 1.0f;
    lower_bounds->at(matrix_row_index) = end_state_[1] - dx_slack_bound;
    upper_bounds->at(matrix_row_index) = end_state_[1] + dx_slack_bound;
    ++matrix_row_index;

    c_float ddx_slack_bound = 0.15f;
    matrix(matrix_row_index, 3 * n - 1) = 1.0f;
    lower_bounds->at(matrix_row_index) = end_state_[2] - ddx_slack_bound;
    upper_bounds->at(matrix_row_index) = end_state_[2] + ddx_slack_bound;
    ++matrix_row_index;
    csc_matrix_valid_num += 3;
  }

  // CHECK_EQ(matrix_row_index, num_of_constraints);

  // ILOG_INFO << "matrix_row_index " << static_cast<int>(matrix_row_index);

  A_data->reserve(csc_matrix_valid_num);
  A_indices->reserve(csc_matrix_valid_num);
  A_indptr->reserve(num_of_variables + 1);
  c_int ind_p = 0;

  // columns
  for (c_int i = 0; i < num_of_variables; ++i) {
    A_indptr->emplace_back(ind_p);
    for (c_int r = 0; r < matrix.rows(); ++r) {
      if (std::fabs(matrix(r, i)) < 0.0001f) {
        continue;
      }
      // coefficient
      A_data->emplace_back(matrix(r, i));

      // row
      A_indices->emplace_back(r);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->emplace_back(ind_p);

  // ILOG_INFO << "ind_p " << static_cast<int>(ind_p);

  return;
}

}  // namespace planning
