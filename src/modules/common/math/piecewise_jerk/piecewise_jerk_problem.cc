
#include "piecewise_jerk_problem.h"

#include <chrono>

#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "log_glog.h"

namespace planning {

constexpr double kMaxVariableRange = 1.0e10;

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) {
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

void PiecewiseJerkProblem::Init(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) {
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
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

bool PiecewiseJerkProblem::Optimize(const int max_iter, const double max_time) {
  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  settings->max_iter = max_iter;
  settings->eps_abs = 1e-5;
  settings->eps_rel = 1e-5;
  // settings->eps_dual_inf = 1e-3;
  // settings->eps_prim_inf = 1e-3;
  // second
  settings->time_limit = max_time;

  // DebugString();

  OSQPWorkspace* osqp_work = nullptr;
  osqp_work = osqp_setup(data, settings);
  // osqp_setup(&osqp_work, data, settings);

  osqp_solve(osqp_work);

  // ILOG_INFO << "osqp iter = " << osqp_work->info->iter;

  auto status = osqp_work->info->status_val;

  if (status < 0 || (status != 1 && status != 2) ||
      osqp_work->solution == nullptr) {
    ILOG_ERROR << "failed optimization status: " << status << ", "
               << osqp_work->info->status;

    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);

    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }

  // Cleanup
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
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

  // variables: record A matrix
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      // x
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      // x'
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      // x''
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }
  CHECK_EQ(constraint_index, num_of_variables);

  // continous constraints
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);

    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  // continuous constraints
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // continous constraints
  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // constrain on x_init
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  double ddx_slack_bound = 0.08;
  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) =
      x_init_[2] * scale_factor_[2] - ddx_slack_bound;
  upper_bounds->at(constraint_index) =
      x_init_[2] * scale_factor_[2] + ddx_slack_bound;
  ++constraint_index;

  // constraints on end state
  if (has_end_state_constriants_) {
    double s_slack_bound = 0.03;
    variables[n - 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        end_state_[0] * scale_factor_[0] - s_slack_bound;
    upper_bounds->at(constraint_index) =
        end_state_[0] * scale_factor_[0] + s_slack_bound;
    ++constraint_index;

    double dx_slack_bound = 0.03;
    variables[2 * n - 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        end_state_[1] * scale_factor_[1] - dx_slack_bound;
    upper_bounds->at(constraint_index) =
        end_state_[1] * scale_factor_[1] + dx_slack_bound;
    ++constraint_index;

    double ddx_slack_bound = 0.15;
    variables[3 * n - 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        end_state_[2] * scale_factor_[2] - ddx_slack_bound;
    upper_bounds->at(constraint_index) =
        end_state_[2] * scale_factor_[2] + ddx_slack_bound;
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, num_of_constraints);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);

      // constraint index
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->push_back(ind_p);

  return;
}

OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = false;
  settings->scaled_termination = true;
  return settings;
}

void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);

  return;
}

void PiecewiseJerkProblem::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);

  return;
}

void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);

  return;
}

void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,
                                        const double x_upper_bound) {
  for (auto& x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }

  return;
}

void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,
                                         const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }

  return;
}

void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,
                                          const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }

  return;
}

void PiecewiseJerkProblem::set_x_ref(const double weight_x_ref,
                                     std::vector<double> x_ref) {
  CHECK_EQ(x_ref.size(), num_of_knots_);
  weight_x_ref_ = weight_x_ref;
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;

  return;
}

void PiecewiseJerkProblem::set_end_state_ref(
    const std::array<double, 3>& weight_end_state,
    const std::array<double, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ = end_state_ref;
  has_end_state_ref_ = true;

  return;
}

void PiecewiseJerkProblem::set_end_state_constriants(
    const std::array<double, 3>& end_state_ref) {
  end_state_ = end_state_ref;
  has_end_state_constriants_ = true;

  return;
}

void PiecewiseJerkProblem::FreeData(OSQPData* data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;

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

}  // namespace planning
