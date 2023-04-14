#include "src/modules/common/math/piecewise_jerk/piecewise_problem.h"

#include "src/common/log.h"
#include "assert.h"
// #include "core/common/trace.h"

namespace planning {
namespace planning_math {

constexpr double MaxVariableRange = 1.0e4;
constexpr double MinDeltaSSquare = 1e-3;

PiecewiseProblem::PiecewiseProblem(const size_t num_of_knots,
                                   const std::vector<double> &s,
                                   const std::array<double, 3> &x_init,
                                   bool hard_constraint) {
  num_of_knots_ = num_of_knots;
  num_of_slack_bounds_ = 0;
  s_ = s;
  x_init_ = x_init;
  hard_constraint_ = hard_constraint;

  // init variables
  x_.resize(num_of_knots_, 0.0);
  dx_.resize(num_of_knots_, 0.0);
  ddx_.resize(num_of_knots_, 0.0);
  bounds_lower_slack_.clear();
  bounds_upper_slack_.clear();

  // bounds
  variable_bounds_cons_.clear();
  variable_bounds_slack_.clear();
  ddd_bounds_.clear();

  weight_x_ = 0.0;
  weight_dx_ = 0.0;
  weight_ddx_ = 0.0;
  weight_dddx_ = 0.0;
  weight_slack_ = 0.0;

  has_x_ref_ = false;
  has_dx_ref_ = false;
  x_ref_.resize(num_of_knots_, 0.0);
  dx_ref_.resize(num_of_knots_, 0.0);
  weight_x_ref_.resize(num_of_knots_, 0.0);

  weight_dx_ref_.resize(num_of_knots_, 0.0);
  has_end_state_ref_ = false;
  end_state_ref_ = {0.0, 0.0, 0.0};
  weight_end_state_ = {0.0, 0.0, 0.0};
}

void PiecewiseProblem::add_variable_bound(
    const IndexedVariableBound &variable_bound) {
  if (variable_bound.weight < 0) {
    if (hard_constraint_) {
      variable_bounds_cons_.push_back(variable_bound);
    } else {
      auto bound = variable_bound;
      bound.weight = 1e4 * pow(1e2, variable_bound.order);
      variable_bounds_slack_.push_back(bound);
      num_of_slack_bounds_++;
    }
  } else {
    variable_bounds_slack_.push_back(variable_bound);
    num_of_slack_bounds_++;
  }
}

void PiecewiseProblem::add_infer_bound(const IndexedInferBound &infer_bound) {
  assert(infer_bound.weight > 0);
  infer_bounds_slack_.push_back(infer_bound);
  num_of_slack_infer_bounds_++;
}

void PiecewiseProblem::set_x_bounds(
    const std::vector<WeightedBounds> &x_bounds) {
  assert(num_of_knots_ == x_bounds.size());
  for (size_t i = 0; i < x_bounds.size(); i++) {
    for (auto &bound : x_bounds[i]) {
      IndexedVariableBound variable_bound;
      variable_bound.order = 0;
      variable_bound.idx = i;
      variable_bound.lower = std::max(bound.lower, -MaxVariableRange);
      variable_bound.upper = std::min(bound.upper, MaxVariableRange);
      variable_bound.weight = bound.weight;
      add_variable_bound(variable_bound);
    }
  }
}

void PiecewiseProblem::set_x_infer_bounds(
    const std::vector<WeightedLonLeadBounds> &x_infer_bounds) {
  assert(num_of_knots_ == x_infer_bounds.size());
  for (size_t i = 0; i < x_infer_bounds.size(); i++) {
    for (auto &bound : x_infer_bounds[i]) {
      IndexedInferBound infer_bound;
      infer_bound.order = 0;
      infer_bound.idx = i;
      infer_bound.infer_s = bound.t_ego + bound.t_lead;
      infer_bound.lower = -MaxVariableRange;
      infer_bound.upper = bound.s_lead + bound.t_lead * bound.v_lead;
      infer_bound.weight = bound.weight;
      add_infer_bound(infer_bound);
    }
  }
}

void PiecewiseProblem::set_dx_bounds(const double dx_lower_bound,
                                     const double dx_upper_bound,
                                     const double weight) {
  for (size_t i = 0; i < num_of_knots_; i++) {
    IndexedVariableBound variable_bound;
    variable_bound.order = 1;
    variable_bound.idx = i;
    variable_bound.lower = std::max(dx_lower_bound, -MaxVariableRange);
    variable_bound.upper = std::min(dx_upper_bound, MaxVariableRange);
    variable_bound.weight = weight;
    add_variable_bound(variable_bound);
  }
}

void PiecewiseProblem::set_dx_bounds(const Bounds &dx_bounds,
                                     const double weight) {
  for (size_t i = 0; i < num_of_knots_; i++) {
    auto &bound = dx_bounds[i];
    IndexedVariableBound variable_bound;
    variable_bound.order = 1;
    variable_bound.idx = i;
    variable_bound.lower = std::max(bound.lower, -MaxVariableRange);
    variable_bound.upper = std::min(bound.upper, MaxVariableRange);
    variable_bound.weight = weight;
    add_variable_bound(variable_bound);
  }
}

void PiecewiseProblem::set_ddx_bounds(const Bounds &ddx_bounds) {
  assert(ddx_bounds.size() == num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; i++) {
    IndexedVariableBound variable_bound;
    variable_bound.order = 2;
    variable_bound.idx = i;
    variable_bound.lower = std::max(ddx_bounds[i].lower, -MaxVariableRange);
    variable_bound.upper = std::min(ddx_bounds[i].upper, MaxVariableRange);
    variable_bound.weight = -1;
    add_variable_bound(variable_bound);
  }
}

void PiecewiseProblem::set_dddx_bound(const double dddx_bound) {
  for (size_t i = 0; i < num_of_knots_ - 1; i++) {
    IndexedDDDBound ddd_bound;
    ddd_bound.idx = i;
    ddd_bound.lower = std::max(-dddx_bound, -MaxVariableRange);
    ddd_bound.upper = std::min(dddx_bound, MaxVariableRange);
    ddd_bounds_.push_back(ddd_bound);
  }
}

void PiecewiseProblem::set_dddx_bounds(const Bounds &dddx_bounds) {
  for (size_t i = 0; i < num_of_knots_ - 1; i++) {
    IndexedDDDBound ddd_bound;
    ddd_bound.idx = i;
    ddd_bound.lower = std::max(dddx_bounds[i].lower, -MaxVariableRange);
    ddd_bound.upper = std::min(dddx_bounds[i].upper, MaxVariableRange);
    ddd_bounds_.push_back(ddd_bound);
  }
}

void PiecewiseProblem::set_x_ref(const std::vector<double> &xref_basic,
                                 const std::vector<double> &x_reference,
                                 const std::vector<double> &obs_weight) {
  for (size_t i = 0; i < num_of_knots_; ++i) {
    weight_x_ref_[i] = xref_basic[i] * (1 + obs_weight[i]);
    x_ref_[i] = x_reference[i];
  }
  has_x_ref_ = true;
}

void PiecewiseProblem::set_x_ref(
    const std::vector<std::pair<double, double>> &x_ref,
    const double weight_xref_basic) {
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_ref_[i] = x_ref[i].first;
    weight_x_ref_[i] = weight_xref_basic * x_ref[i].second;
  }
  has_x_ref_ = true;
}

void PiecewiseProblem::set_dx_ref(
    const std::vector<std::pair<double, double>> &dx_ref,
    const double weight_dxref_basic) {
  for (size_t i = 0; i < num_of_knots_; ++i) {
    dx_ref_[i] = dx_ref[i].first;
    weight_dx_ref_[i] = weight_dxref_basic * dx_ref[i].second;
  }
  has_dx_ref_ = true;
}

void PiecewiseProblem::set_end_state_ref(
    const std::array<double, 3> &weight_end_state,
    const std::array<double, 3> &end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

bool PiecewiseProblem::optimize(const int max_iter, int &status) {
  // clear
  x_.clear();
  dx_.clear();
  ddx_.clear();
  dddx_.clear();
  bounds_lower_slack_.clear();
  bounds_upper_slack_.clear();

  std::vector<c_float> p_data;
  std::vector<c_int> p_indices;
  std::vector<c_int> p_indptr;
  calculate_kernel(&p_data, &p_indices, &p_indptr);
  p_data_.assign(p_data.begin(), p_data.end());
  p_indices_.assign(p_indices.begin(), p_indices.end());
  p_indptr_.assign(p_indptr.begin(), p_indptr.end());

  // calculate affine constraints
  std::vector<c_float> a_data;
  std::vector<c_int> a_indices;
  std::vector<c_int> a_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  calculate_affine_constraint(&a_data, &a_indices, &a_indptr, &lower_bounds,
                              &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  calculate_offset(&q);

  // solve
  size_t kernel_dim = 3 * num_of_knots_ + 2 * num_of_slack_bounds_ +
                      2 * num_of_slack_infer_bounds_;
  c_float *result = new c_float[kernel_dim];

  status = OptimizeWithOsqp(p_data.data(), p_data.size(), p_indices.data(),
                            p_indptr.data(), q.data(), a_data.data(),
                            a_data.size(), a_indices.data(), a_indptr.data(),
                            lower_bounds.data(), upper_bounds.data(),
                            kernel_dim, lower_bounds.size(), max_iter, result);

  if (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE &&
      status != OSQP_MAX_ITER_REACHED) {
    LOG_ERROR("Piecewise Problem Solver status : %d \n", status);
    delete[] result;
    return false;
  }

  auto success = true;
  if (status == OSQP_SOLVED_INACCURATE or status == OSQP_MAX_ITER_REACHED) {
    for (auto &bound : variable_bounds_cons_) {
      auto var = result[bound.order * num_of_knots_ + bound.idx];
      auto epsilon = 0.1;

      if (var < bound.lower - epsilon or var > bound.upper + epsilon) {
        LOG_DEBUG(
            "solver constraint failure on x: %d, order: %d, lower: %f, upper: "
            "%f, value: %f \n",
            bound.idx, bound.order, bound.lower, bound.upper, var);
        success = false;
      }
    }
  }

  if (success) {
    x_.resize(num_of_knots_);
    dx_.resize(num_of_knots_);
    ddx_.resize(num_of_knots_);
    dddx_.resize(num_of_knots_ - 1);
    for (size_t i = 0; i < num_of_knots_; ++i) {
      x_.at(i) = result[i];
      dx_.at(i) = result[i + num_of_knots_];
      ddx_.at(i) = result[i + 2 * num_of_knots_];
    }
    for (size_t i = 0; i < num_of_knots_ - 1; ++i) {
      dddx_.at(i) = (ddx_[i + 1] - ddx_[i]) / (s_[i + 1] - s_[i]);
    }
  } else {
    LOG_ERROR("PiecewiseProblem::optimize failed \n");
  }
  delete[] result;

  return success;
}

void PiecewiseProblem::calculate_kernel(std::vector<c_float> *p_data,
                                        std::vector<c_int> *p_indices,
                                        std::vector<c_int> *p_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam =
      3 * n + num_of_slack_bounds_ * 2 + num_of_slack_infer_bounds_ * 2;

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, weight_x_ + weight_x_ref_[i]);
    ++value_index;
  }

  // x(n-1)^2 * (w_x + w_x_ref + w_end_x)
  columns[n - 1].emplace_back(
      n - 1, weight_x_ + weight_x_ref_[n - 1] + weight_end_state_[0]);
  ++value_index;

  // x(i)'^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(n + i, weight_dx_ + weight_dx_ref_[i]);
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(
      2 * n - 1, weight_dx_ + weight_dx_ref_[n - 1] + weight_end_state_[1]);
  ++value_index;

  // i = 0 or n-1: x(i)''^2 * (w_ddx + w_dddx / delta_s^2)
  // 0 < i < n- 1: x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  double delta_s{s_[1] - s_[0]};
  double delta_s_square{std::max(MinDeltaSSquare, delta_s * delta_s)};
  columns[2 * n].emplace_back(2 * n,
                              weight_ddx_ + weight_dddx_ / delta_s_square);
  ++value_index;

  // - w_dddx / delta_s^2 * x(i + 1)'' * x(i)''
  for (int i = 0; i < n - 1; ++i) {
    delta_s = s_[i + 1] - s_[i];
    delta_s_square = std::max(MinDeltaSSquare, delta_s * delta_s);
    columns[2 * n + i + 1].emplace_back(2 * n + i,
                                        -1.0 * weight_dddx_ / delta_s_square);
    ++value_index;
  }

  for (int i = 1; i < n - 1; ++i) {
    delta_s = s_[i + 1] - s_[i];
    delta_s_square = std::max(MinDeltaSSquare, delta_s * delta_s);
    double delta_s_last = s_[i] - s_[i - 1];
    double delta_s_last_square =
        std::max(MinDeltaSSquare, delta_s_last * delta_s_last);
    columns[2 * n + i].emplace_back(
        2 * n + i, weight_ddx_ + weight_dddx_ / delta_s_square +
                       weight_dddx_ / delta_s_last_square);
    ++value_index;
  }

  size_t cnt = s_.size();
  delta_s = s_[cnt - 1] - s_[cnt - 2];
  delta_s_square = std::max(MinDeltaSSquare, delta_s * delta_s);
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]);
  ++value_index;

  // - w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < n - 1; ++i) {
    delta_s = s_[i + 1] - s_[i];
    delta_s_square = std::max(MinDeltaSSquare, delta_s * delta_s);
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    -1.0 * weight_dddx_ / delta_s_square);
    ++value_index;
  }

  // slack_lower^2, slack_upper^2
  auto slack_variable_pos = 3 * n;
  for (size_t i = 0; i < num_of_slack_bounds_; i++) {
    columns[slack_variable_pos + 2 * i].emplace_back(
        slack_variable_pos + 2 * i,
        weight_slack_ * variable_bounds_slack_[i].weight);
    columns[slack_variable_pos + 2 * i + 1].emplace_back(
        slack_variable_pos + 2 * i + 1,
        weight_slack_ * variable_bounds_slack_[i].weight);
  }

  // infer slack bound, slack_lower^2, slack_upper^2
  auto slack_infer_variable_pos = 3 * n + 2 * num_of_slack_bounds_;
  for (size_t i = 0; i < num_of_slack_infer_bounds_; i++) {
    columns[slack_infer_variable_pos + 2 * i].emplace_back(
        slack_infer_variable_pos + 2 * i,
        weight_slack_ * infer_bounds_slack_[i].weight);
    columns[slack_infer_variable_pos + 2 * i + 1].emplace_back(
        slack_infer_variable_pos + 2 * i + 1,
        weight_slack_ * infer_bounds_slack_[i].weight);
  }

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    p_indptr->push_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      p_data->push_back(row_data_pair.second * 2.0);
      p_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  p_indptr->push_back(ind_p);
}

void PiecewiseProblem::calculate_offset(std::vector<c_float> *q) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam =
      3 * n + num_of_slack_bounds_ * 2 + num_of_slack_infer_bounds_ * 2;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * (weight_x_ref_[i] + weight_x_) * x_ref_[i];
    }
  }

  if (has_dx_ref_) {
    for (int i = n; i < 2 * n; ++i) {
      q->at(i) += -2.0 * (weight_dx_ + weight_dx_ref_[i - n]) * dx_ref_[i - n];
      LOG_DEBUG("HHLDEBUG has_dx_ref weight_dx_ref_: %.2f, dx_ref_: %.2f \n",
            weight_dx_ref_[i - n], dx_ref_[i - n] * 3.6);
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) += -2.0 * weight_end_state_[0] * end_state_ref_[0];
    q->at(2 * n - 1) += -2.0 * weight_end_state_[1] * end_state_ref_[1];
    q->at(3 * n - 1) += -2.0 * weight_end_state_[2] * end_state_ref_[2];
  }
}

void PiecewiseProblem::calculate_affine_constraint(
    std::vector<c_float> *a_data, std::vector<c_int> *a_indices,
    std::vector<c_int> *a_indptr, std::vector<c_float> *lower_bounds,
    std::vector<c_float> *upper_bounds) {
  // 2(n-1) constraints on continuity with x', x', x''
  // 3 constraints on x_init
  // ddd_bounds.size() ddd constraints
  // variable_bounds_cons_.size() variable_bounds_cons
  // num_of_slack_bounds * 2 variable_bounds_slack
  // num_of_slack_infer_bounds * 2 infer_bounds_slack
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam =
      3 * n + 2 * num_of_slack_bounds_ + 2 * num_of_slack_infer_bounds_;
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);

  const int kNumConstraint =
      2 * (n - 1) + 3 + ddd_bounds_.size() + variable_bounds_cons_.size() +
      num_of_slack_bounds_ * 2 + num_of_slack_infer_bounds_ * 2;
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  int constraint_index = 0;
  // continuity
  // x(i+1)' - x(i)' - 0.5 * delta_s * (x(i+1)'' + x(i)'') = 0
  for (int i = 0; i + 1 < n; ++i) {
    auto delta_s = s_[i + 1] - s_[i];
    columns[n + i].emplace_back(constraint_index, -1.0);
    columns[n + i + 1].emplace_back(constraint_index, 1.0);
    columns[2 * n + i].emplace_back(constraint_index, -0.5 * delta_s);
    columns[2 * n + i + 1].emplace_back(constraint_index, -0.5 * delta_s);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - x(i)'*delta_s - 1/3*x(i)''*delta_s^2 -
  // 1/6*x(i+1)''*delta_s^2
  for (int i = 0; i + 1 < n; ++i) {
    auto delta_s = s_[i + 1] - s_[i];
    auto delta_s_sq = std::pow(delta_s, 2.0);
    columns[i].emplace_back(constraint_index, -1.0);
    columns[i + 1].emplace_back(constraint_index, 1.0);
    columns[n + i].emplace_back(constraint_index, -delta_s);
    columns[2 * n + i].emplace_back(constraint_index, -delta_s_sq / 3.0);
    columns[2 * n + i + 1].emplace_back(constraint_index, -delta_s_sq / 6.0);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // constrain on x_init
  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0];
  upper_bounds->at(constraint_index) = x_init_[0];
  ++constraint_index;

  columns[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1];
  upper_bounds->at(constraint_index) = x_init_[1];
  ++constraint_index;

  columns[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2];
  upper_bounds->at(constraint_index) = x_init_[2];
  ++constraint_index;

  // ddd constraints
  // dddx_lower_bound * delta_s <= x(i+1)'' - x(i)''  <= dddx_upper_bound *
  // delta_s
  for (auto &bound : ddd_bounds_) {
    auto idx = bound.idx;
    auto delta_s = s_[idx + 1] - s_[idx];
    columns[2 * n + idx].emplace_back(constraint_index, -1.0);
    columns[2 * n + idx + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = bound.lower * delta_s;
    upper_bounds->at(constraint_index) = bound.upper * delta_s;
    ++constraint_index;
  }

  // variable constraints
  for (auto &bound : variable_bounds_cons_) {
    columns[bound.order * n + bound.idx].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = bound.lower;
    upper_bounds->at(constraint_index) = bound.upper;
    ++constraint_index;
  }

  // variable slack bounds
  auto slack_variable_pos = 3 * n;
  for (size_t i = 0; i < variable_bounds_slack_.size(); ++i) {
    auto &bound = variable_bounds_slack_[i];
    // x_slack_bound_down <= x + x_slack_down
    columns[bound.order * n + bound.idx].emplace_back(constraint_index, 1.0);
    columns[slack_variable_pos + 2 * i].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = bound.lower;
    upper_bounds->at(constraint_index) = MaxVariableRange;
    ++constraint_index;

    // x - x_slack_up <= x_slack_bound_up
    columns[bound.order * n + bound.idx].emplace_back(constraint_index, 1.0);
    columns[slack_variable_pos + 2 * i + 1].emplace_back(constraint_index,
                                                         -1.0);
    lower_bounds->at(constraint_index) = -MaxVariableRange;
    upper_bounds->at(constraint_index) = bound.upper;
    ++constraint_index;
  }

  // infer slack bounds
  auto slack_infer_variable_pos = 3 * n + 2 * num_of_slack_bounds_;
  for (size_t i = 0; i < infer_bounds_slack_.size(); ++i) {
    auto &bound = infer_bounds_slack_[i];
    assert(bound.order <= 1);
    // x_slack_bound_down <= x + x' * infer_s + x_slack_down
    columns[bound.order * n + bound.idx].emplace_back(constraint_index, 1.0);
    columns[(bound.order + 1) * n + bound.idx].emplace_back(constraint_index,
                                                            bound.infer_s);
    columns[slack_infer_variable_pos + 2 * i].emplace_back(constraint_index,
                                                           1.0);
    lower_bounds->at(constraint_index) = bound.lower;
    upper_bounds->at(constraint_index) = MaxVariableRange;
    ++constraint_index;

    // x + x' * infer_s - x_slack_up <= x_slack_bound_up
    columns[bound.order * n + bound.idx].emplace_back(constraint_index, 1.0);
    columns[(bound.order + 1) * n + bound.idx].emplace_back(constraint_index,
                                                            bound.infer_s);
    columns[slack_infer_variable_pos + 2 * i + 1].emplace_back(constraint_index,
                                                               -1.0);
    lower_bounds->at(constraint_index) = -MaxVariableRange;
    upper_bounds->at(constraint_index) = bound.upper;
    ++constraint_index;
  }

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    a_indptr->push_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      a_data->push_back(row_data_pair.second);
      a_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  a_indptr->push_back(ind_p);
}

}  // namespace planning_math
}  // namespace planning
