#include "lateral_path_optimizer.h"

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "apa_world.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.pb.h"
#include "log.h"
#include "math_lib.h"
#include "planning_plan_c.h"
#include "src/lateral_path_optimizer_cost.h"
#include "src/lateral_path_optimizer_problem.h"

static const double kPiConst = 3.141592654;
static const double kMinRadius = 6.0;
static const double KappaMax = 1.0 / kMinRadius;
static const double kCurvFactor = 0.3;
static const double kMaxDelta = 400 / 57.3 / 15;
static const double kRefVel = 0.6;
static const size_t PlanningTraPointsNum =
    PLANNING_TRAJ_POINTS_MAX_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_MAX_NUM;

namespace planning {
namespace apa_planner {
void LateralPathOptimizer::Init(const bool c_ilqr_enable) {
  // init planning problem
  planning_problem_ptr_ =
      std::make_shared<apa_planner::LateralPathOptimizerProblem>();
  planning_problem_ptr_->Init(c_ilqr_enable);

  // init planning input and ouput
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  // init planning input
  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_k_vec()->Resize(N, 0.0);

  planning_input_.mutable_k_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_k_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_u_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_u_min_vec()->Resize(N, 0.0);

  planning_input_.mutable_control_vec()->Resize(N, 0.0);

  // init output
  optimizer_planning_output_.mutable_s_vec()->Resize(N, 0.0);

  optimizer_planning_output_.mutable_x_vec()->Resize(N, 0.0);
  optimizer_planning_output_.mutable_y_vec()->Resize(N, 0.0);
  optimizer_planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  optimizer_planning_output_.mutable_k_vec()->Resize(N, 0.0);

  optimizer_planning_output_.mutable_u_vec()->Resize(N, 0.0);
}

void LateralPathOptimizer::AssembleInput(
    const std::vector<pnc::geometry_lib::PathPoint> &path_vec) {
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> s_vec;

  const auto N_path_vec = path_vec.size();
  x_vec.reserve(N_path_vec);
  y_vec.reserve(N_path_vec);
  theta_vec.reserve(N_path_vec);
  s_vec.reserve(N_path_vec);

  double s = 0.0;
  for (size_t i = 0; i < N_path_vec; ++i) {
    x_vec.emplace_back(path_vec[i].pos.x());
    y_vec.emplace_back(path_vec[i].pos.y());
    s_vec.emplace_back(s);

    if (i < N_path_vec - 1) {
      const auto ds =
          std::max((path_vec[i].pos - path_vec[i + 1].pos).norm(), 0.001);
      s += ds;
    }
  }

  // theta preprocess
  double angle_offset = 0.0;
  theta_vec.emplace_back(path_vec.front().heading);
  for (size_t i = 1; i < N_path_vec; i++) {
    const auto delta_theta = path_vec[i].heading - path_vec[i - 1].heading;
    if (delta_theta > 1.5 * kPiConst) {
      angle_offset -= 2.0 * kPiConst;
    } else if (delta_theta < -1.5 * kPiConst) {
      angle_offset += 2.0 * kPiConst;
    }
    theta_vec.emplace_back(path_vec[i].heading + angle_offset);
  }

  pnc::mathlib::spline x_s_spline_;
  pnc::mathlib::spline y_s_spline_;
  pnc::mathlib::spline theta_s_spline_;

  x_s_spline_.set_points(s_vec, x_vec);
  y_s_spline_.set_points(s_vec, y_vec);
  theta_s_spline_.set_points(s_vec, theta_vec);

  const auto ilqr_horizon =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon;

  const auto ds_ilqr = s_vec.back() / static_cast<double>(ilqr_horizon);
  double s_ilqr = 0.0;

  // set reference
  const auto k_min = -KappaMax;
  const auto u_max = kCurvFactor * kMaxDelta / kRefVel;
  const auto u_min = -u_max;
  for (size_t i = 0; i < ilqr_horizon + 1; ++i) {
    // set init state
    if (i == 0) {
      planning_input_.mutable_init_state()->set_x(x_vec.front());

      planning_input_.mutable_init_state()->set_y(y_vec.front());

      planning_input_.mutable_init_state()->set_theta(theta_s_spline_(s_ilqr));

      planning_input_.mutable_init_state()->set_k(pnc::mathlib::Limit(
          theta_s_spline_.deriv(1, s_ilqr + 0.5 * ds_ilqr), KappaMax));
    }
    planning_input_.mutable_ref_theta_vec()->Set(i, theta_s_spline_(s_ilqr));
    planning_input_.mutable_ref_x_vec()->Set(i, x_s_spline_(s_ilqr));
    planning_input_.mutable_ref_y_vec()->Set(i, y_s_spline_(s_ilqr));
    planning_input_.mutable_ref_k_vec()->Set(
        i, pnc::mathlib::Limit(theta_s_spline_.deriv(1, s_ilqr), KappaMax));

    planning_input_.mutable_k_max_vec()->Set(i, KappaMax);
    planning_input_.mutable_k_min_vec()->Set(i, k_min);
    planning_input_.mutable_u_max_vec()->Set(i, u_max);
    planning_input_.mutable_u_min_vec()->Set(i, u_min);
    s_ilqr += ds_ilqr;
  }
  planning_input_.set_ref_s(s_ilqr - ds_ilqr);

  // terminal x terminal y terminal theta
  planning_input_.set_last_x(planning_input_.ref_x_vec(ilqr_horizon));
  planning_input_.set_last_y(planning_input_.ref_y_vec(ilqr_horizon));
  planning_input_.set_last_theta(planning_input_.ref_theta_vec(ilqr_horizon));

  // simu_ weight
  planning_input_.set_q_ref_x(param_.q_ref_xy);
  planning_input_.set_q_ref_y(param_.q_ref_xy);
  planning_input_.set_q_ref_theta(param_.q_ref_theta);
  planning_input_.set_q_terminal_x(param_.q_terminal_xy);
  planning_input_.set_q_terminal_y(param_.q_terminal_xy);
  planning_input_.set_q_terminal_theta(param_.q_terminal_theta);
  planning_input_.set_q_k(param_.q_k);
  planning_input_.set_q_u(param_.q_u);
  planning_input_.set_q_k_bound(param_.q_k_bound);
  planning_input_.set_q_u_bound(param_.q_u_bound);
  // curv param etc.
  planning_input_.set_r_min(kMinRadius);
  planning_input_.set_curv_factor(kCurvFactor);
  planning_input_.set_delta_max(kMaxDelta);
  planning_input_.set_ref_vel(kRefVel);
}

void LateralPathOptimizer::AssembleOutput() {
  const auto &planning_output = planning_problem_ptr_->GetOutput();
  const size_t N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  for (size_t i = 0; i < N; i++) {
    optimizer_planning_output_.mutable_s_vec()->Set(i,
                                                    planning_output.s_vec(i));
    optimizer_planning_output_.mutable_x_vec()->Set(i,
                                                    planning_output.x_vec(i));
    optimizer_planning_output_.mutable_y_vec()->Set(i,
                                                    planning_output.y_vec(i));
    optimizer_planning_output_.mutable_theta_vec()->Set(
        i, planning_output.theta_vec(i));
    optimizer_planning_output_.mutable_k_vec()->Set(i,
                                                    planning_output.k_vec(i));

    if (i < N - 1) {
      optimizer_planning_output_.mutable_u_vec()->Set(i,
                                                      planning_output.u_vec(i));
    } else {
      optimizer_planning_output_.mutable_u_vec()->Set(
          i, optimizer_planning_output_.u_vec(i - 1));
    }
  }

  const Eigen::Vector2d delta_terminal_pose(
      optimizer_planning_output_.x_vec(N - 1) -
          planning_input_.ref_x_vec(N - 1),
      optimizer_planning_output_.y_vec(N - 1) -
          planning_input_.ref_y_vec(N - 1));

  const double terminal_pose_error = delta_terminal_pose.norm();
  const double terminal_heading_error =
      (optimizer_planning_output_.theta_vec(N - 1) -
       planning_input_.ref_theta_vec(N - 1)) *
      57.3;

  optimizer_planning_output_.set_terminal_pos_error(terminal_pose_error);
  optimizer_planning_output_.set_terminal_heading_error(terminal_heading_error);
}

void LateralPathOptimizer::PostProcessOutput() {
  const auto &planning_output = planning_problem_ptr_->GetOutput();
  const size_t N = planning_output.x_vec_size();

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> k_vec;
  std::vector<double> s_vec;
  x_vec.reserve(N);
  y_vec.reserve(N);
  theta_vec.reserve(N);
  k_vec.reserve(N);
  s_vec.reserve(N);

  // double s_sum = 0.0;
  pnc::geometry_lib::PathPoint pt;
  origin_output_path_vec_.clear();
  origin_output_path_vec_.reserve(planning_output.x_vec().size());
  for (size_t i = 0; i < N; i++) {
    x_vec.emplace_back(planning_output.x_vec(i));
    y_vec.emplace_back(planning_output.y_vec(i));
    theta_vec.emplace_back(planning_output.theta_vec(i));
    k_vec.emplace_back(planning_output.k_vec(i));
    s_vec.emplace_back(planning_output.s_vec(i));
    pt.pos << planning_output.x_vec(i), planning_output.y_vec(i);
    pt.heading =
        pnc::geometry_lib::NormalizeAngle(planning_output.theta_vec(i));
    pt.kappa = pnc::mathlib::Limit(planning_output.k_vec(i), KappaMax);
    pt.s = planning_output.s_vec(i);
    origin_output_path_vec_.emplace_back(pt);
  }

  size_t resampled_point_num = std::ceil(s_vec.back() / param_.sample_ds);
  double adjust_sample_ds = param_.sample_ds;
  size_t adjust_sample_point_num = resampled_point_num;
  if (resampled_point_num > PlanningTraPointsNum) {
    adjust_sample_ds = s_vec.back() / PlanningTraPointsNum;
    adjust_sample_point_num = std::ceil(s_vec.back() / adjust_sample_ds);
  }

  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline theta_s_spline;
  pnc::mathlib::spline k_s_spline;

  x_s_spline.set_points(s_vec, x_vec);
  y_s_spline.set_points(s_vec, y_vec);
  theta_s_spline.set_points(s_vec, theta_vec);
  k_s_spline.set_points(s_vec, k_vec);

  output_path_vec_.clear();
  output_path_vec_.reserve(adjust_sample_point_num);
  double resampled_ds = 0.0;
  Eigen::Vector2d point;
  double heading;
  pnc::geometry_lib::PathPoint tmp_output;
  for (size_t i = 0; i < adjust_sample_point_num; i++) {
    point << x_s_spline(resampled_ds), y_s_spline(resampled_ds);
    heading = pnc::geometry_lib::NormalizeAngle(theta_s_spline(resampled_ds));
    tmp_output.Set(point, heading);
    tmp_output.s = resampled_ds;
    tmp_output.kappa = pnc::mathlib::Limit(k_s_spline(resampled_ds), KappaMax);
    output_path_vec_.emplace_back(tmp_output);
    resampled_ds += adjust_sample_ds;
  }
}

void LateralPathOptimizer::Update(
    const std::vector<pnc::geometry_lib::PathPoint> &path_vec,
    const uint8_t gear_cmd) {
  // fail protection if path_vec is empty
  if (path_vec.size() < 3) {
    // std::cout << "optimizer input size is too small" << std::endl;
    return;
  }
  // assemble input
  AssembleInput(path_vec);

  // run solver
  auto solver_condition =
      planning_problem_ptr_->Update(planning_input_, gear_cmd);
  // std::cout << "lateral path optimizer: solver_condition::"
  //           << static_cast<size_t>(solver_condition) << "\n";
  // postprocess planning_output
  PostProcessOutput();
  // assemble output to plan debug info
  AssembleOutput();
}
}  // namespace apa_planner
}  // namespace planning
