#include "lateral_path_optimizer.h"

#include <cmath>
#include <cstddef>
#include <memory>

#include "Eigen/src/Core/Matrix.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.pb.h"
#include "log.h"
#include "math_lib.h"
#include "src/lateral_path_optimizer_cost.h"
#include "src/lateral_path_optimizer_problem.h"

static const double kPiConst = 3.141592654;
static const double kMinRadius = 5.5;
static const double kCurvFactor = 0.3;
static const double kMaxDelta = 400 / 57.3 / 15;
static const double kRefVel = 1.688;

namespace planning {
namespace apa_planner {
void LateralPathOptimizer::Init() {
  // init planning problem
  planning_problem_ptr_ =
      std::make_shared<apa_planner::LateralPathOptimizerProblem>();
  planning_problem_ptr_->Init();

  // init planning input and ouput
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  // init planning input
  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);

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
  const auto k_max = 1 / kMinRadius;
  const auto k_min = -k_max;
  const auto u_max = kCurvFactor * kMaxDelta / kRefVel;
  const auto u_min = -u_max;
  for (size_t i = 0; i < ilqr_horizon + 1; ++i) {
    // set init state
    if (i == 0) {
      planning_input_.mutable_init_state()->set_x(x_vec.front());

      planning_input_.mutable_init_state()->set_y(y_vec.front());

      planning_input_.mutable_init_state()->set_theta(theta_s_spline_(s_ilqr));

      planning_input_.mutable_init_state()->set_k(
          pnc::mathlib::Limit(theta_s_spline_.deriv(1, s_ilqr), k_max));
    }
    planning_input_.mutable_ref_theta_vec()->Set(i, theta_s_spline_(s_ilqr));
    planning_input_.mutable_ref_x_vec()->Set(i, x_s_spline_(s_ilqr));
    planning_input_.mutable_ref_y_vec()->Set(i, y_s_spline_(s_ilqr));
    planning_input_.mutable_k_max_vec()->Set(i, k_max);
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
  // return 0;
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

  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();

  planning_debug_data->clear_lateral_path_optimizer_input();
  planning_debug_data->clear_lateral_motion_planning_output();
  planning_debug_data->mutable_lateral_path_optimizer_input()->CopyFrom(
      planning_input_);

  planning_debug_data->mutable_lateral_path_optimizer_output()->CopyFrom(
      optimizer_planning_output_);
}

void LateralPathOptimizer::PostProcessOutput() {
  const auto &planning_output = planning_problem_ptr_->GetOutput();
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> s_vec;
  x_vec.reserve(N);
  y_vec.reserve(N);
  theta_vec.reserve(N);

  double s_sum = 0.0;
  for (size_t i = 0; i < N; i++) {
    x_vec.emplace_back(planning_output.x_vec(i));
    y_vec.emplace_back(planning_output.y_vec(i));
    theta_vec.emplace_back(planning_output.theta_vec(i));
    if (i < N - 1) {
      s_vec.emplace_back(s_sum);
      s_sum += std::sqrt(pnc::mathlib::Square(planning_output.x_vec(i + 1) -
                                              planning_output.x_vec(i)) +
                         pnc::mathlib::Square(planning_output.y_vec(i + 1) -
                                              planning_output.y_vec(i)));
    }
  }
  s_vec.emplace_back(s_sum);
  const size_t resampled_point_num = std::floor(s_sum / param_.sample_ds) + 1;

  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline theta_s_spline;
  x_s_spline.set_points(s_vec, x_vec);
  y_s_spline.set_points(s_vec, y_vec);
  theta_s_spline.set_points(s_vec, theta_vec);

  double resampled_ds = 0.0;
  output_path_vec_.clear();
  output_path_vec_.reserve(resampled_point_num);
  Eigen::Vector2d point;
  double heading;
  pnc::geometry_lib::PathPoint tmp_output;
  for (size_t i = 0; i < resampled_point_num; i++) {
    point << x_s_spline(resampled_ds), y_s_spline(resampled_ds);
    heading = pnc::geometry_lib::NormalizeAngle(theta_s_spline(resampled_ds));
    tmp_output.Set(point, heading);
    output_path_vec_.emplace_back(tmp_output);
    resampled_ds += param_.sample_ds;
  }
}

void LateralPathOptimizer::Update(
    const std::vector<pnc::geometry_lib::PathPoint> &path_vec,
    const uint8_t gear_cmd) {
  // assemble input
  AssembleInput(path_vec);

  // run solver
  auto solver_condition =
      planning_problem_ptr_->Update(planning_input_, gear_cmd);
  std::cout << "lateral path optimizer: solver_condition::"
            << static_cast<size_t>(solver_condition) << "\n";

  // postprocess planning_output
  PostProcessOutput();

  // assemble output
  AssembleOutput();
}
}  // namespace apa_planner
}  // namespace planning
