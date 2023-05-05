#include "acado_lat_mpc.h"
#include "lateral_motion_planning_problem.h"
#include "spline.h"
#include "transform.h"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>
struct PybindFuncParam {
  // vehicle states
  double pos_x = 0.0;
  double pos_y = 0.0;
  double theta = 0.0;
  double delta = 0.0;

  // set_speed
  double set_vel_gain = 1.0;

  // reference offset
  double ref_offset = 0.0; // > 0 means towards upper (left)
  // last traj offset
  double last_offset = 0.0; // > 0 means towards upper (left)

  // corridor offset
  double corridor_upper_offset = 2.0;
  double corridor_lower_offset = 2.0;

  // weights
  // lateral acc and omega bound
  double acc_bound = 100.0;
  double jerk_bound = 100.0;

  // weights
  double q_ref_x = 1.0;
  double q_ref_y = 1.0;
  double q_ref_theta = 1.0;

  double q_continuity = 0.0;
  double q_acc = 1.0;
  double q_jerk = 1.0;

  double q_acc_bound = 1000.0;
  double q_jerk_bound = 1000.0;

  double q_soft_corridor = 0.0;
  double q_hard_corridor = 0.0;

  double q_snap = 2.0;
};

class LateralMotionPlanningPybindFunc {
public:
  // hack planning input by mpc_input
  void Update(pnc::control::LatMpcInput &mpc_input, PybindFuncParam &param) {
    // curv_factor
    planning_input_.curv_factor = mpc_input.curv_factor_;

    // mpc input is in vehicle coord, tansform into world coord
    planning_input_.ref_theta_vec = mpc_input.dphi_ref_mpc_vec_;
    ThetaTransformBody2Inertia(planning_input_.ref_theta_vec, param.theta);

    // mpc input is in vehicle coord, tansform into world coord
    planning_input_.ref_x_vec = mpc_input.dx_ref_mpc_vec_;
    planning_input_.ref_y_vec = mpc_input.dy_ref_mpc_vec_;
    Point2dTransformBody2Inertia(planning_input_.ref_x_vec,
                                 planning_input_.ref_y_vec, param.pos_x,
                                 param.pos_y, param.theta);

    // magify the trajectory by param set_vel_gain
    double mean_vel =
        std::accumulate(mpc_input.vel_lat_ctrl_mpc_vec_.begin(),
                        mpc_input.vel_lat_ctrl_mpc_vec_.end(), 0.0) /
        mpc_input.vel_lat_ctrl_mpc_vec_.size();

    planning_input_.ref_vel = mean_vel * param.set_vel_gain;
    GenerateReference(planning_input_.ref_x_vec, planning_input_.ref_y_vec,
                      planning_input_.ref_theta_vec, param.set_vel_gain,
                      mean_vel);

    // last traj without offset
    planning_input_.last_x_vec = planning_input_.ref_x_vec;
    planning_input_.last_y_vec = planning_input_.ref_y_vec;
    planning_input_.last_theta_vec = planning_input_.ref_theta_vec;

    // init corridor
    auto upper_corridor_x_vec = planning_input_.ref_x_vec;
    auto upper_corridor_y_vec = planning_input_.ref_y_vec;
    auto lower_corridor_x_vec = planning_input_.ref_x_vec;
    auto lower_corridor_y_vec = planning_input_.ref_y_vec;

    // record init point before adding offset
    double init_pos_x = planning_input_.ref_x_vec[0];
    double init_pos_y = planning_input_.ref_y_vec[0];
    double init_theta = planning_input_.ref_theta_vec[0];

    // for lane change, add offset to reference
    AddCurvOffset(planning_input_.ref_x_vec, planning_input_.ref_y_vec,
                  planning_input_.ref_theta_vec, param.ref_offset);

    // for path corridor generation, can be more complicated
    AddCurvOffset(upper_corridor_x_vec, upper_corridor_y_vec,
                  planning_input_.ref_theta_vec, param.corridor_upper_offset);
    AddCurvOffset(lower_corridor_x_vec, lower_corridor_y_vec,
                  planning_input_.ref_theta_vec, -param.corridor_lower_offset);

    // get (x0, y0) and (x1, y1) of corridor
    planning_input_.soft_upper_bound_x0_vec = upper_corridor_x_vec;
    planning_input_.soft_upper_bound_y0_vec = upper_corridor_y_vec;
    planning_input_.soft_upper_bound_x1_vec = upper_corridor_x_vec;
    planning_input_.soft_upper_bound_y1_vec = upper_corridor_y_vec;
    PropagateCurv(planning_input_.soft_upper_bound_x1_vec,
                  planning_input_.soft_upper_bound_y1_vec, 1);

    planning_input_.soft_lower_bound_x0_vec = lower_corridor_x_vec;
    planning_input_.soft_lower_bound_y0_vec = lower_corridor_y_vec;
    planning_input_.soft_lower_bound_x1_vec = lower_corridor_x_vec;
    planning_input_.soft_lower_bound_y1_vec = lower_corridor_y_vec;
    PropagateCurv(planning_input_.soft_lower_bound_x1_vec,
                  planning_input_.soft_lower_bound_y1_vec, 1);

    // weights
    planning_input_.acc_bound = param.acc_bound;
    planning_input_.jerk_bound = param.jerk_bound;
    planning_input_.q_ref_x = param.q_ref_x;
    planning_input_.q_ref_y = param.q_ref_y;
    planning_input_.q_ref_theta = param.q_ref_theta;
    planning_input_.q_continuity = param.q_continuity;
    planning_input_.q_acc = param.q_acc;
    planning_input_.q_jerk = param.q_jerk;
    planning_input_.q_acc_bound = param.q_acc_bound;
    planning_input_.q_jerk_bound = param.q_jerk_bound;
    planning_input_.q_soft_corridor = param.q_soft_corridor;
    planning_input_.q_hard_corridor = param.q_hard_corridor;
    planning_input_.q_snap = param.q_snap;

    // init state
    Eigen::VectorXd x0;
    x0.resize(5);
    x0 << init_pos_x, init_pos_y, init_theta, param.delta, 0.0;

    planning_input_.init_state = x0;
  }

  pnc::lateral_planning::LateralMotionPlanningInput GetOutput() {
    return planning_input_;
  }

private:
  pnc::lateral_planning::LateralMotionPlanningInput planning_input_;

  void AddCurvOffset(std::vector<double> &x_vec, std::vector<double> &y_vec,
                     std::vector<double> &theta_vec, double offset) {
    int N = x_vec.size();
    for (int i = 0; i < N; ++i) {
      x_vec[i] += -sin(theta_vec[i]) * offset;
      y_vec[i] += cos(theta_vec[i]) * offset;
    }
  }

  void Point2dTransformBody2Inertia(std::vector<double> &x_vec,
                                    std::vector<double> &y_vec, double pos_x,
                                    double pos_y, double theta) {
    Eigen::Vector2d pos_b;
    Eigen::Vector2d pos_i;
    for (int i = 0; i < static_cast<int>(x_vec.size()); ++i) {
      pos_b << x_vec[i], y_vec[i];
      pos_i = pnc::transform::Angle2Rotm2d(theta) * pos_b +
              Eigen::Vector2d(pos_x, pos_y);

      x_vec[i] = pos_i.x();
      y_vec[i] = pos_i.y();
    }
  }

  void ThetaTransformBody2Inertia(std::vector<double> &theta_vec,
                                  double theta) {
    for (size_t i = 0; i < theta_vec.size(); ++i) {
      theta_vec[i] += theta;
    }
  }

  void GenerateReference(std::vector<double> &x_vec, std::vector<double> &y_vec,
                         std::vector<double> &theta_vec, double set_vel_gain,
                         double mean_vel) {
    int N = x_vec.size();
    std::vector<double> dx_vec;
    std::vector<double> dy_vec;
    // N - 1 dx, dy
    for (int i = 0; i < N - 1; ++i) {
      dx_vec.emplace_back(x_vec[i + 1] - x_vec[i]);
      dy_vec.emplace_back(y_vec[i + 1] - y_vec[i]);
    }

    // first point fixed
    for (int i = 1; i < N; ++i) {
      double dx = dx_vec[i - 1] * set_vel_gain * 1.02;
      double dy = dy_vec[i - 1] * set_vel_gain * 1.02;
      x_vec[i] = x_vec[i - 1] + dx;
      y_vec[i] = y_vec[i - 1] + dy;
    }

    double s = 0.0;
    std::vector<double> s_vec;
    s_vec.emplace_back(0.0);
    for (int i = 1; i < N; ++i) {
      s += std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s_vec.emplace_back(s);
    }

    // spline x_vec, y_vec, theta_vec
    pnc::mathlib::spline x_vec_spline_s;
    pnc::mathlib::spline y_vec_spline_s;
    pnc::mathlib::spline theta_vec_spline_s;

    x_vec_spline_s.set_points(s_vec, x_vec);
    y_vec_spline_s.set_points(s_vec, y_vec);
    theta_vec_spline_s.set_points(s_vec, theta_vec);

    s = 0.0;
    double dt = 0.1;
    for (int i = 0; i < N; ++i) {
      x_vec[i] = x_vec_spline_s(s);
      y_vec[i] = y_vec_spline_s(s);
      theta_vec[i] = theta_vec_spline_s(s);
      s += set_vel_gain * mean_vel * dt;
    }
  }

  void PropagateCurv(std::vector<double> &x_vec, std::vector<double> &y_vec,
                     const size_t &step) {
    int N = x_vec.size();
    double dx = x_vec[N - 1] - x_vec[N - 2];
    double dy = y_vec[N - 1] - y_vec[N - 2];

    auto const tmp_x_vec = x_vec;
    auto const tmp_y_vec = y_vec;

    for (int i = 0; i < N; ++i) {
      if (i < N - static_cast<int>(step)) {
        x_vec[i] = tmp_x_vec[i + step];
        y_vec[i] = tmp_y_vec[i + step];
      } else {
        x_vec[i] = x_vec[i - 1] + dx;
        y_vec[i] = y_vec[i - 1] + dy;
      }
    }
  }
};
