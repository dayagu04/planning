#pragma once

#include <array>
#include <cassert>
#include <vector>

#include "tasks/task_basic_types.h"

typedef long long c_int;
typedef double c_float;

extern "C" {
#include "common/math/piecewise_jerk/osqp_interface.h"
}

typedef int (*Osqp_run_para)(c_float *, c_int, c_int *, c_int *, c_float *,
                             c_float *, c_int, c_int *, c_int *, c_float *,
                             c_float *, c_int, c_int, c_float *);

namespace planning {
namespace planning_math {

struct IndexedVariableBound {
  int order;
  int idx;
  double lower;
  double upper;
  double weight;
};
using IndexedVariableBounds = std::vector<IndexedVariableBound>;

struct IndexedDDDBound {
  int idx;
  double lower;
  double upper;
};
using IndexedDDDBounds = std::vector<IndexedDDDBound>;

struct IndexedInferBound {
  int order;
  int idx;
  double infer_s;
  double lower;
  double upper;
  double weight;
};
using IndexedInferBounds = std::vector<IndexedInferBound>;

class PiecewiseProblem {
 public:
  PiecewiseProblem(const size_t num_of_knots, const std::vector<double> &s,
                   const std::array<double, 3> &x_init,
                   bool hard_constraint = true);
  virtual ~PiecewiseProblem() = default;

  // bounds
  void set_x_bounds(const std::vector<WeightedBounds> &x_bounds);
  void set_dx_bounds(const double dx_lower_bound, const double dx_upper_bound,
                     const double weight = -1);
  void set_dx_bounds(const Bounds &dx_bounds, const double weight = -1);
  void set_ddx_bounds(const Bounds &ddx_bounds);
  void set_dddx_bound(const double dddx_bound);
  void set_dddx_bounds(const Bounds &dddx_bounds);
  void set_x_infer_bounds(
      const std::vector<WeightedLonLeadBounds> &x_infer_bounds);

  void set_weight_x(const double weight_x) { weight_x_ = weight_x; }
  void set_weight_dx(const double weight_dx) { weight_dx_ = weight_dx; }
  void set_weight_ddx(const double weight_ddx) { weight_ddx_ = weight_ddx; }
  void set_weight_dddx(const double weight_dddx) { weight_dddx_ = weight_dddx; }
  void set_weight_slack(const double weight_slack) {
    weight_slack_ = weight_slack;
  }

  void set_x_ref(const std::vector<double> &xref_basic,
                 const std::vector<double> &x_reference,
                 const std::vector<double> &obs_weight);

  void set_dx_ref(const std::vector<std::pair<double, double>> &dx_ref,
                  const double weight_dxref_basic = 1.0);

  /* set reference and weight.
   *
   * @param[in] x_ref
   *     x_ref[i].first : optimization reference value at i point
   *     x_ref[i].second : optimization reference cost weight at i point
   * @param[in] weight_xref_basic : optimization weight for reference cost term,
   *     trade-off among reference cost, acc cost, jerk cost ....
   */
  void set_x_ref(const std::vector<std::pair<double, double>> &x_ref,
                 const double weight_xref_basic = 1.0);
  const std::vector<double> &weight_x_ref() { return weight_x_ref_; }
  void set_end_state_ref(const std::array<double, 3> &weight_end_state,
                         const std::array<double, 3> &end_state_ref);

  const std::vector<double> &x() const { return x_; }
  const std::vector<double> &dx() const { return dx_; }
  const std::vector<double> &ddx() const { return ddx_; }
  const std::vector<double> &dddx() const { return dddx_; }

  const std::vector<c_float> &p_data() const { return p_data_; }
  const std::vector<c_int> &p_indices() const { return p_indices_; }
  const std::vector<c_int> &p_indptr() const { return p_indptr_; }

  bool optimize(const int max_iter, int &status);

 private:
  // bounds
  void add_variable_bound(const IndexedVariableBound &variable_bound);
  void add_ddd_bound(const IndexedDDDBound &ddd_bound);
  void add_infer_bound(const IndexedInferBound &infer_bound);

  void calculate_kernel(std::vector<c_float> *p_data,
                        std::vector<c_int> *p_indices,
                        std::vector<c_int> *p_indptr);

  virtual void calculate_affine_constraint(std::vector<c_float> *a_data,
                                           std::vector<c_int> *a_indices,
                                           std::vector<c_int> *a_indptr,
                                           std::vector<c_float> *lower_bounds,
                                           std::vector<c_float> *upper_bounds);

  void calculate_offset(std::vector<c_float> *q);

 private:
  bool hard_constraint_ = true;
  size_t num_of_knots_ = 0;
  size_t num_of_slack_bounds_ = 0;
  size_t num_of_slack_infer_bounds_ = 0;
  std::array<double, 3> x_init_;
  std::vector<double> s_;

  // optimize variables
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;
  std::vector<double> dddx_;
  std::vector<double> bounds_lower_slack_;
  std::vector<double> bounds_upper_slack_;

  // p matrix
  std::vector<c_float> p_data_;
  std::vector<c_int> p_indices_;
  std::vector<c_int> p_indptr_;

  // bounds
  IndexedVariableBounds variable_bounds_cons_;
  IndexedVariableBounds variable_bounds_slack_;
  IndexedDDDBounds ddd_bounds_;
  IndexedInferBounds infer_bounds_slack_;

  // weights
  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;
  //  std::vector<double> weight_dddx_;
  double weight_slack_ = 0.0;

  // x_ref
  bool has_x_ref_ = false;
  std::vector<double> x_ref_;
  std::vector<double> weight_x_ref_;

  // dx_ref
  bool has_dx_ref_ = false;
  std::vector<double> dx_ref_;
  std::vector<double> weight_dx_ref_;

  // end state ref
  bool has_end_state_ref_ = false;
  std::array<double, 3> end_state_ref_;
  std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
};

}  // namespace planning_math
}  // namespace planning
