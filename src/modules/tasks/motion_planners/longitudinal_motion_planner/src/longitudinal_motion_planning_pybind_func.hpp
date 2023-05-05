#include "longitudinal_motion_planning_problem.h"
#include "math_lib.h"
#include "spline.h"
#include "transform.h"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>

namespace pnc {
namespace longitudinal_planning {
struct PybindFuncParam {
  // vehicle states
  double pos = 0.0;
  double vel = 0.0;
  double acc = 0.0;
  double jerk = 0.0;

  // parameters
  double set_vel = 10.0;
  double cipv_vel = 10.0;
  double cipv_dist = 10.0;
  double stop_s = 20.0;
  double vel_max = 35.0;
  double acc_max = 2.0;
  double acc_min = -5.0;
  double jerk_max = 2.0;
  double jerk_min = -4.0;

  bool stop_enable = false;

  double ref_acc_inc = 0.5;
  double ref_acc_dec = -0.5;

  // weights
  double q_ref_pos = 1.0;
  double q_ref_vel = 1.0;
  double q_acc = 1.0;
  double q_jerk = 1.0;
  double q_snap = 1.0;
  double q_pos_bound = 100.0;
  double q_vel_bound = 100.0;
  double q_acc_bound = 100.0;
  double q_jerk_bound = 100.0;
  double q_stop_s = 100.0;
};

class LongitudinalMotionPlanningPybindFunc {
public:
  // hack planning input
  void Update(PybindFuncParam &param) {
    const size_t N = 41;
    const double dt = 0.1;
    planning_input_.ref_vel_vec.clear();
    planning_input_.ref_pos_vec.clear();

    planning_input_.pos_max_vec.clear();
    planning_input_.pos_min_vec.clear();

    planning_input_.pos_max_vec.clear();
    planning_input_.pos_min_vec.clear();

    planning_input_.vel_max_vec.clear();
    planning_input_.vel_min_vec.clear();

    planning_input_.acc_max_vec.clear();
    planning_input_.acc_min_vec.clear();

    planning_input_.jerk_max_vec.clear();
    planning_input_.jerk_min_vec.clear();

    const double back_to_rear = 1.083;
    for (size_t i = 0; i < N; ++i) {
      if (i == 0) {
        planning_input_.ref_vel_vec.emplace_back(param.vel);
        planning_input_.ref_pos_vec.emplace_back(param.pos);
        planning_input_.pos_max_vec.emplace_back(param.cipv_dist -
                                                 back_to_rear);
        planning_input_.pos_min_vec.emplace_back(0.0);
      } else {
        planning_input_.ref_vel_vec.emplace_back(mathlib::Clamp(
            param.set_vel,
            planning_input_.ref_vel_vec.at(i - 1) + param.ref_acc_dec * dt,
            planning_input_.ref_vel_vec.at(i - 1) + param.ref_acc_inc * dt));

        planning_input_.ref_pos_vec.emplace_back(
            planning_input_.ref_pos_vec.at(i - 1) +
            planning_input_.ref_vel_vec.at(i) * dt);

        planning_input_.pos_max_vec.emplace_back(
            planning_input_.pos_max_vec.at(i - 1) + param.cipv_vel * dt);
        planning_input_.pos_min_vec.emplace_back(0.0);
      }

      planning_input_.vel_max_vec.emplace_back(param.vel_max);
      planning_input_.vel_min_vec.emplace_back(0.05);

      planning_input_.acc_max_vec.emplace_back(param.acc_max);
      planning_input_.acc_min_vec.emplace_back(param.acc_min);

      planning_input_.jerk_max_vec.emplace_back(param.jerk_max);
      planning_input_.jerk_min_vec.emplace_back(param.jerk_min);

      if (param.stop_enable) {
        planning_input_.s_stop = param.stop_s;
      } else {
        planning_input_.s_stop = 1000000.0;
      }
    }

    // weights
    planning_input_.q_ref_pos = param.q_ref_pos;
    planning_input_.q_ref_vel = param.q_ref_vel;
    planning_input_.q_acc = param.q_acc;
    planning_input_.q_jerk = param.q_jerk;
    planning_input_.q_snap = param.q_snap;
    planning_input_.q_pos_bound = param.q_pos_bound;
    planning_input_.q_vel_bound = param.q_vel_bound;
    planning_input_.q_acc_bound = param.q_acc_bound;
    planning_input_.q_jerk_bound = param.q_jerk_bound;
    planning_input_.q_stop_s = param.q_stop_s;

    // init state
    Eigen::VectorXd x0;
    x0.resize(4);
    x0 << param.pos, param.vel, param.acc, param.jerk;

    planning_input_.init_state = x0;
  }

  pnc::longitudinal_planning::LongitudinalMotionPlanningInput GetOutput() {
    return planning_input_;
  }

private:
  pnc::longitudinal_planning::LongitudinalMotionPlanningInput planning_input_;
};
} // namespace longitudinal_planning
} // namespace pnc
