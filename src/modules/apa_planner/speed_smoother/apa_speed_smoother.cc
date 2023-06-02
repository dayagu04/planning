
#include "apa_planner/speed_smoother/apa_speed_smoother.h"

#include <array>
#include <vector>
#include <utility>

#include "apa_planner/common/planning_log_helper.h"
#include "apa_planner/speed_smoother/apa_speed_limit_decider.h"
#include "common/math/linear_interpolation.h"
#include "common/math/piecewise_jerk/piecewise_problem.h"
#include "common/utils/file.h"

namespace planning {

using ::PlanningOutput::PlanningOutput;
using planning_math::LineSegment2d;

namespace {
constexpr double kMinSpd = 0.3;
}

ApaSpeedSmoother::ApaSpeedSmoother() {
  const std::string config_file =
      "/asw/planning/res/conf/apa_speed_smoother_config.pb.txt";
  common::util::GetProtoFromFile(config_file, &config_);
}

bool ApaSpeedSmoother::Smooth(
    const std::vector<LineSegment2d>& obstacles,
    PlanningOutput *const planning_output) {
  const int traj_point_num =
      planning_output->trajectory().trajectory_points_size();
  if (traj_point_num < 2) {
    return false;
  }

  // resample path by same delta_s
  const double path_length =
      planning_output->trajectory().trajectory_points().rbegin()->distance();
  delta_s_ = config_.delta_s();
  int resample_point_num = static_cast<int>(path_length / delta_s_) + 1;
  resample_point_num = std::max(resample_point_num, 5);
  delta_s_ = path_length / (resample_point_num - 1);
  PLANNING_LOG << "path_length:" << path_length
      << ", delta_s_:" << delta_s_ << std::endl;

  std::vector<double> s_vev;
  s_vev.reserve(resample_point_num);
  int traj_point_index = 0;
  double path_s = 0.0;
  const auto& trajectory_points =
      planning_output->trajectory().trajectory_points();
  PlanningOutput smoothed_planning_output = *planning_output;
  auto smooothed_trajectory =
      smoothed_planning_output.mutable_trajectory();
  smooothed_trajectory->mutable_trajectory_points()->Clear();
  while (s_vev.size() < resample_point_num) {
    while (traj_point_index < traj_point_num
        && trajectory_points[traj_point_index].distance() < path_s) {
      ++traj_point_index;
    }
    traj_point_index = std::min(traj_point_index, traj_point_num - 1);

    const int pre_traj_point_index = std::max(traj_point_index - 1, 0);
    ::PlanningOutput::TrajectoryPoint traj_point =
        planning_math::InterpolateUsingLinearApproximation(
            trajectory_points[pre_traj_point_index],
            trajectory_points[traj_point_index], path_s);
    smooothed_trajectory->add_trajectory_points()->CopyFrom(traj_point);

    s_vev.emplace_back(path_s);
    path_s += delta_s_;
  }
  PLANNING_LOG << "traj_point_num:" << traj_point_num
      << ", resample_point_num:" << resample_point_num << std::endl;

  // get speed limit data
  ApaSpeedLimit speed_limit_data;
  speed_limit_decider_.GetSpeedLimits(
      smoothed_planning_output, obstacles, &speed_limit_data);

  // construct piecewise jerk problem
  const std::array<double, 3> init_state = {kMinSpd, 0.0, 0.0};
  planning_math::PiecewiseProblem
      piecewise_jerk_problem(resample_point_num, s_vev, init_state, true);
  piecewise_jerk_problem.set_weight_x(config_.weight_v());
  piecewise_jerk_problem.set_weight_dx(config_.weight_dv());
  piecewise_jerk_problem.set_weight_ddx(config_.weight_ddv());
  piecewise_jerk_problem.set_weight_dddx(config_.weight_dddv());
  piecewise_jerk_problem.set_weight_slack(config_.weight_slack());

  PLANNING_LOG << "weight_v:" << config_.weight_v()
      << ", weight_dv:" << config_.weight_dv()
      << ", set_weight_ddx:" << config_.weight_ddv()
      << ", weight_dddv:" << config_.weight_dddv()
      << ", weight_slack:" << config_.weight_slack()
      << ", weight_ref_v:" << config_.weight_ref_v() << std::endl;

  // calculate bounds
  std::vector<WeightedBounds> v_bounds(resample_point_num,
      std::vector<WeightedBound>(1, WeightedBound()));
  std::vector<std::pair<double, double>> v_refs;
  v_refs.reserve(resample_point_num);
  for (int i = 0; i < resample_point_num; ++i) {
    WeightedBound v_bound;
    v_bound.lower = 0.0;
    v_bound.upper = speed_limit_data.speed_limit_points()[i].second;
    v_bound.weight = -1.0; // weight < 0 means hard constraint
    v_bounds[i][0] = v_bound;
    v_refs.emplace_back(v_bound.upper, config_.weight_ref_v());
  }
  v_bounds.back().back().lower = kMinSpd;
  v_bounds.back().back().upper = kMinSpd;

  const int end_index = resample_point_num - 1;
  piecewise_jerk_problem.set_x_bounds(v_bounds);
  Bound end_dv_bound;
  end_dv_bound.lower = 0.0;
  end_dv_bound.upper = 0.0;
  piecewise_jerk_problem.set_dx_bounds(end_dv_bound, end_index);
  Bound end_ddv_bound;
  end_ddv_bound.lower = 0.0;
  end_ddv_bound.upper = 0.0;
  piecewise_jerk_problem.set_ddx_bounds(end_ddv_bound, end_index);
  piecewise_jerk_problem.set_x_ref(v_refs);
  // solve problem
  int status = 0;
  bool success =
      piecewise_jerk_problem.optimize(config_.max_iter(), status);
  if (!success) {
    PLANNING_LOG << "apa piecewise problem solve failed, starus:"
        << status << std::endl;
    return false;
  }

  // update smooothed trajectory
  const double motion_sign = trajectory_points[0].v() > 0.0 ? 1.0 : -1.0;
  const auto& v_vec = piecewise_jerk_problem.x();
  for (int i = 0; i < resample_point_num; ++i) {
    const double v = v_vec[i] * motion_sign;
    const double v_limit =
        speed_limit_data.speed_limit_points()[i].second * motion_sign;
    smooothed_trajectory->mutable_trajectory_points(i)->set_v(v);
    const auto& pt = smooothed_trajectory->trajectory_points()[i];
    PLANNING_LOG << "smoothed traj pt [" << i << "], x:" << pt.x()
        << ", y:" << pt.y() << ", theta:" << pt.heading_yaw()
        << ", kappa:" << pt.curvature() << ", v:" << pt.v()
        << ", v limit:" << v_limit << ", s:" << pt.distance() << std::endl;
  }
  smooothed_trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);
  *planning_output = smoothed_planning_output;

  return true;
}

}  // namespace planning
