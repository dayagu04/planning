#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>

#include "fem_pos_deviation_smoother_config.pb.h"
#include "src/modules/common/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

namespace py = pybind11;
using namespace planning::planning_math;

static FemPosDeviationSmoother *pBase = nullptr;

std::vector<double> smooth_points_x_;
std::vector<double> smooth_points_y_;

int Init() {
  pBase = new FemPosDeviationSmoother();
  return 0;
}

int Update(std::vector<double> raw_points_x, std::vector<double> raw_points_y,
           bool is_enable_local, bool apply_curvature_constraint, bool use_sqp,
           bool verbose, bool scaled_termination, bool warm_start,
           int sqp_pen_max_iter, int sqp_sub_max_iter, int max_iter,
           double weight_fem_pos_deviation, double weight_ref_deviation,
           double weight_path_length, double weight_curvature_constraint_slack_var,
           double curvature_constraint, double sqp_ftol,
           double sqp_ctol, double time_limit, double bound_val) {
  planning::FemPosDeviationSmootherConfig smoother_param;
  // tune param
  smoother_param.set_apply_curvature_constraint(apply_curvature_constraint);
  smoother_param.set_use_sqp(use_sqp);
  smoother_param.set_verbose(verbose);
  smoother_param.set_scaled_termination(scaled_termination);
  smoother_param.set_warm_start(warm_start);
  smoother_param.set_sqp_pen_max_iter(sqp_pen_max_iter);
  smoother_param.set_sqp_sub_max_iter(sqp_sub_max_iter);
  smoother_param.set_max_iter(max_iter);
  smoother_param.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
  smoother_param.set_weight_ref_deviation(weight_ref_deviation);
  smoother_param.set_weight_path_length(weight_path_length);
  smoother_param.set_weight_curvature_constraint_slack_var(weight_curvature_constraint_slack_var);
  smoother_param.set_curvature_constraint(curvature_constraint);
  smoother_param.set_sqp_ftol(sqp_ftol);
  smoother_param.set_sqp_ctol(sqp_ctol);
  smoother_param.set_time_limit(time_limit);
  pBase->SetFemPosDeviationSmootherConfig(smoother_param);
  smooth_points_x_.clear();
  smooth_points_y_.clear();
  std::pair<double, double> raw_init_point{0, 0};
  if (raw_points_x.size() == raw_points_y.size() &&
      raw_points_x.size() >= 5) {
    // raw point
    if (is_enable_local) {
      raw_init_point.first = raw_points_x[0];
      raw_init_point.second = raw_points_y[0];
    }
    std::vector<std::pair<double, double>> raw_points;
    raw_points.reserve(raw_points_x.size());
    for (size_t i = 0; i < raw_points_x.size(); ++i) {
      raw_points.emplace_back(raw_points_x[i] - raw_init_point.first, raw_points_y[i] - raw_init_point.second);
    }
    // bounds
    std::vector<double> bounds;
    bounds.resize(raw_points_x.size(), bound_val);
    bounds.front() = 1e-6;
    bounds.back() = 1e-6;
    // solve
    pBase->Solve(raw_points, bounds, &smooth_points_x_, &smooth_points_y_);
  }
  return 0;
}

std::vector<double> GetSmoothPointsX() {
  return smooth_points_x_;
}

std::vector<double> GetSmoothPointsY() {
  return smooth_points_y_;
}

PYBIND11_MODULE(reference_path_smooth_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
   .def("Update", &Update)
   .def("GetSmoothPointsX", &GetSmoothPointsX)
   .def("GetSmoothPointsY", &GetSmoothPointsY);
}
