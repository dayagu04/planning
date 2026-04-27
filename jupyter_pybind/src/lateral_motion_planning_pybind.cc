#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>

#include "lateral_motion_planner.pb.h"
#include "motion_planners/lateral_motion_planner/problem_solver/ilqr_solver.h"
#include "motion_planners/lateral_motion_planner/problem_solver/solver_define.h"
#include "planning_debug_info.pb.h"

#include "serialize_utils.h"

namespace py = pybind11;
using namespace pnc::lateral_planning;

static iLQRSolver *pBase = nullptr;
constexpr size_t N = 26;

planning::common::LateralPlanningOutput planning_output;

int Init() {
  pBase = new iLQRSolver();
  pBase->SimInit();
  planning_output.mutable_time_vec()->Resize(N, 0.0);
  planning_output.mutable_x_vec()->Resize(N, 0.0);
  planning_output.mutable_y_vec()->Resize(N, 0.0);
  planning_output.mutable_theta_vec()->Resize(N, 0.0);
  planning_output.mutable_delta_vec()->Resize(N, 0.0);
  planning_output.mutable_omega_vec()->Resize(N, 0.0);
  planning_output.mutable_acc_vec()->Resize(N, 0.0);
  planning_output.mutable_jerk_vec()->Resize(N, 0.0);
  return 0;
}

int UpdateBytes(py::bytes &planning_input_bytes) {
  planning::common::LateralPlanningInput planning_input =
      BytesToProto<planning::common::LateralPlanningInput>(
          planning_input_bytes);

  // pBase->Update(0,0,0,planning_input);

  return 0;
}

py::bytes GetOutputBytes() {
  std::string serialized_message;
  planning_output.SerializeToString(&serialized_message);
  return serialized_message;
}

int UpdateByParams(py::bytes &planning_input_bytes, double q_ref_xy,
                   double q_ref_theta, double q_acc, double q_jerk, double q_continuity,
                   double q_acc_bound, double q_jerk_bound, double acc_bound,
                   double jerk_bound, double q_soft_bound, double q_hard_bound,
                   double ref_xy,  double first_upper_soft_bound, double first_lower_soft_bound,
                   double second_upper_soft_bound, double second_lower_soft_bound,
                   double upper_hard_bound, double lower_hard_bound,
                   int first_soft_ub_start_idx, int first_soft_ub_end_idx,
                   int first_soft_lb_start_idx, int first_soft_lb_end_idx,
                   int second_soft_ub_start_idx, int second_soft_ub_end_idx,
                   int second_soft_lb_start_idx, int second_soft_lb_end_idx,
                   int hard_ub_start_idx, int hard_ub_end_idx,
                   int hard_lb_start_idx, int hard_lb_end_idx,
                   bool complete_follow,
                   int motion_plan_concerned_start_index, int motion_plan_concerned_end_index,
                   double curv_factor,
                   double start_w_jerk, double ego_v, double expected_acc, double start_acc, double end_acc,
                   double end_ratio1, double end_ratio2, double end_ratio3, double max_iter,
                   double wheel_base, double q_front_ref_xy,
                   double q_virtual_ref_xy, double q_virtual_ref_theta, double max_delta, double max_omega,
                   std::vector<double> virtual_ref_x, std::vector<double> virtual_ref_y, std::vector<double> virtual_ref_theta) {
  planning::common::LateralPlanningInput planning_input =
      BytesToProto<planning::common::LateralPlanningInput>(
          planning_input_bytes);
  planning::common::LateralPlanningInput origin_planning_input = planning_input;
  // tune param
  planning_input.set_acc_bound(acc_bound);
  planning_input.set_jerk_bound(jerk_bound);
  planning_input.set_q_ref_x(q_ref_xy);
  planning_input.set_q_ref_y(q_ref_xy);
  planning_input.set_q_ref_theta(q_ref_theta);
  planning_input.set_q_acc(q_acc);
  planning_input.set_q_jerk(q_jerk);
  planning_input.set_q_continuity(q_continuity);
  planning_input.set_q_acc_bound(q_acc_bound);
  planning_input.set_q_jerk_bound(q_jerk_bound);
  planning_input.set_q_soft_corridor(q_soft_bound);
  planning_input.set_q_hard_corridor(q_hard_bound);
  planning_input.set_curv_factor(curv_factor);

  planning_input.set_complete_follow(complete_follow);
  if (motion_plan_concerned_start_index < 0) {
    motion_plan_concerned_start_index = 0;
  } else if (motion_plan_concerned_start_index > 26) {
    motion_plan_concerned_start_index = 26;
  }
  if (motion_plan_concerned_end_index < 0) {
    motion_plan_concerned_end_index = 0;
  } else if (motion_plan_concerned_end_index > 26) {
    motion_plan_concerned_end_index = 26;
  }
  planning_input.set_motion_plan_concerned_index(motion_plan_concerned_end_index);
  // ref
  // const auto N = planning_input.ref_x_vec_size();
  for (size_t i = 0; i < N; i++) {
    if (i < N - 1) {
      Eigen::Vector2d ref_unit_vector(
          origin_planning_input.hard_upper_bound_x0_vec(i) -
              origin_planning_input.hard_lower_bound_x0_vec(i),
          origin_planning_input.hard_upper_bound_y0_vec(i) -
              origin_planning_input.hard_lower_bound_y0_vec(i));
      ref_unit_vector.normalize();
      planning_input.mutable_ref_x_vec()->Set(i, origin_planning_input.ref_x_vec(i) + ref_unit_vector.x() * ref_xy);
      planning_input.mutable_ref_y_vec()->Set(i, origin_planning_input.ref_y_vec(i) + ref_unit_vector.y() * ref_xy);
    } else {
      Eigen::Vector2d ref_unit_vector(
          origin_planning_input.hard_upper_bound_x1_vec(i) -
              origin_planning_input.hard_lower_bound_x1_vec(i),
          origin_planning_input.hard_upper_bound_y1_vec(i) -
              origin_planning_input.hard_lower_bound_y1_vec(i));
      ref_unit_vector.normalize();
      planning_input.mutable_ref_x_vec()->Set(i, origin_planning_input.ref_x_vec(i) + ref_unit_vector.x() * ref_xy);
      planning_input.mutable_ref_y_vec()->Set(i, origin_planning_input.ref_y_vec(i) + ref_unit_vector.y() * ref_xy);
    }
  }
  planning_input.mutable_ref_vel_vec()->Resize(N, origin_planning_input.ref_vel());
  if (origin_planning_input.ref_vel_vec_size() == N) {
    for (size_t i = 0; i < N; i++) {
      planning_input.mutable_ref_vel_vec()->Set(i, origin_planning_input.ref_vel_vec(i));
    }
  } else {
    //
  }
  // virtual ref
  // bool is_virtual_empty =
  //     virtual_ref_x.size() != N ||
  //     virtual_ref_y.size() != N ||
  //     virtual_ref_theta.size() != N;
  // if (is_virtual_empty) {
  //   virtual_ref_x.clear();
  //   virtual_ref_y.clear();
  //   virtual_ref_theta.clear();
  //   for (size_t i = 0; i < N; i++) {
  //     virtual_ref_x.emplace_back(planning_input.ref_x_vec(i));
  //     virtual_ref_y.emplace_back(planning_input.ref_y_vec(i));
  //     virtual_ref_theta.emplace_back(planning_input.ref_theta_vec(i));
  //   }
  //   q_virtual_ref_xy = 0;
  //   q_virtual_ref_theta = 0;
  // }
  // tune the soft bound and hard bound
  bool is_enable_first_and_second_soft_bound =
      origin_planning_input.soft_upper_bound_x0_vec_size() != N ||
      origin_planning_input.soft_upper_bound_y0_vec_size() != N ||
      origin_planning_input.soft_lower_bound_x0_vec_size() != N ||
      origin_planning_input.soft_lower_bound_y0_vec_size() != N;
  if (is_enable_first_and_second_soft_bound) {
    // first bound
    if (first_soft_lb_start_idx < 0) {
      first_soft_lb_start_idx = 0;
    }
    if (first_soft_lb_end_idx > 26) {
      first_soft_lb_end_idx = 26;
    }
    if (first_soft_ub_start_idx < 0) {
      first_soft_ub_start_idx = 0;
    }
    if (first_soft_ub_end_idx > 26) {
      first_soft_ub_end_idx = 26;
    }
    for (size_t i = first_soft_lb_start_idx; i < first_soft_lb_end_idx; i++) {
      if (i < N - 1) {
        Eigen::Vector2d lower_unit_vector(
            origin_planning_input.first_soft_lower_bound_x0_vec(i) -
                origin_planning_input.first_soft_upper_bound_x0_vec(i),
            origin_planning_input.first_soft_lower_bound_y0_vec(i) -
                origin_planning_input.first_soft_upper_bound_y0_vec(i));
        lower_unit_vector.normalize();
        planning_input.mutable_first_soft_lower_bound_x0_vec()->Set(
            i, origin_planning_input.first_soft_lower_bound_x0_vec(i) +
                  lower_unit_vector.x() * first_lower_soft_bound);
        planning_input.mutable_first_soft_lower_bound_y0_vec()->Set(
            i, origin_planning_input.first_soft_lower_bound_y0_vec(i) +
                  lower_unit_vector.y() * first_lower_soft_bound);
      }
      if (i > 0) {
        Eigen::Vector2d lower_unit_vector1(
            origin_planning_input.first_soft_lower_bound_x1_vec(i - 1) -
                origin_planning_input.first_soft_upper_bound_x1_vec(i - 1),
            origin_planning_input.first_soft_lower_bound_y1_vec(i - 1) -
                origin_planning_input.first_soft_upper_bound_y1_vec(i - 1));
        lower_unit_vector1.normalize();
        planning_input.mutable_first_soft_lower_bound_x1_vec()->Set(
            i - 1, origin_planning_input.first_soft_lower_bound_x1_vec(i - 1) +
                  lower_unit_vector1.x() * first_lower_soft_bound);
        planning_input.mutable_first_soft_lower_bound_y1_vec()->Set(
            i - 1, origin_planning_input.first_soft_lower_bound_y1_vec(i - 1) +
                  lower_unit_vector1.y() * first_lower_soft_bound);
      }
    }
    planning_input.mutable_first_soft_lower_bound_x0_vec()->Set(N - 1, planning_input.first_soft_lower_bound_x0_vec(N - 2));
    planning_input.mutable_first_soft_lower_bound_y0_vec()->Set(N - 1, planning_input.first_soft_lower_bound_y0_vec(N - 2));
    planning_input.mutable_first_soft_lower_bound_x1_vec()->Set(N - 1, planning_input.first_soft_lower_bound_x1_vec(N - 2));
    planning_input.mutable_first_soft_lower_bound_y1_vec()->Set(N - 1, planning_input.first_soft_lower_bound_y1_vec(N - 2));

    for (size_t i = first_soft_ub_start_idx; i < first_soft_ub_end_idx; i++) {
      if (i < N - 1) {
        Eigen::Vector2d upper_unit_vector(
            origin_planning_input.first_soft_upper_bound_x0_vec(i) -
                origin_planning_input.first_soft_lower_bound_x0_vec(i),
            origin_planning_input.first_soft_upper_bound_y0_vec(i) -
                origin_planning_input.first_soft_lower_bound_y0_vec(i));
        upper_unit_vector.normalize();
        planning_input.mutable_first_soft_upper_bound_x0_vec()->Set(
            i, origin_planning_input.first_soft_upper_bound_x0_vec(i) +
                  upper_unit_vector.x() * first_upper_soft_bound);
        planning_input.mutable_first_soft_upper_bound_y0_vec()->Set(
            i, origin_planning_input.first_soft_upper_bound_y0_vec(i) +
                  upper_unit_vector.y() * first_upper_soft_bound);
      }
      if (i > 0) {
        Eigen::Vector2d upper_unit_vector1(
            origin_planning_input.first_soft_upper_bound_x1_vec(i - 1) -
                origin_planning_input.first_soft_lower_bound_x1_vec(i - 1),
            origin_planning_input.first_soft_upper_bound_y1_vec(i - 1) -
                origin_planning_input.first_soft_lower_bound_y1_vec(i - 1));
        upper_unit_vector1.normalize();
        planning_input.mutable_first_soft_upper_bound_x1_vec()->Set(
          i - 1, origin_planning_input.first_soft_upper_bound_x1_vec(i - 1) +
                upper_unit_vector1.x() * first_upper_soft_bound);
        planning_input.mutable_first_soft_upper_bound_y1_vec()->Set(
          i - 1, origin_planning_input.first_soft_upper_bound_y1_vec(i - 1) +
                upper_unit_vector1.y() * first_upper_soft_bound);
      }
    }
    planning_input.mutable_first_soft_upper_bound_x0_vec()->Set(N - 1, planning_input.first_soft_upper_bound_x0_vec(N - 2));
    planning_input.mutable_first_soft_upper_bound_y0_vec()->Set(N - 1, planning_input.first_soft_upper_bound_y0_vec(N - 2));
    planning_input.mutable_first_soft_upper_bound_x1_vec()->Set(N - 1, planning_input.first_soft_upper_bound_x1_vec(N - 2));
    planning_input.mutable_first_soft_upper_bound_y1_vec()->Set(N - 1, planning_input.first_soft_upper_bound_y1_vec(N - 2));
    // second bound
    if (second_soft_lb_start_idx < 0) {
      second_soft_lb_start_idx = 0;
    }
    if (second_soft_lb_end_idx > 26) {
      second_soft_lb_end_idx = 26;
    }
    if (second_soft_ub_start_idx < 0) {
      second_soft_ub_start_idx = 0;
    }
    if (second_soft_ub_end_idx > 26) {
      second_soft_ub_end_idx = 26;
    }
    for (size_t i = second_soft_lb_start_idx; i < second_soft_lb_end_idx; i++) {
      if (i < N - 1) {
        Eigen::Vector2d lower_unit_vector(
            origin_planning_input.second_soft_lower_bound_x0_vec(i) -
                origin_planning_input.second_soft_upper_bound_x0_vec(i),
            origin_planning_input.second_soft_lower_bound_y0_vec(i) -
                origin_planning_input.second_soft_upper_bound_y0_vec(i));
        lower_unit_vector.normalize();
        planning_input.mutable_second_soft_lower_bound_x0_vec()->Set(
            i, origin_planning_input.second_soft_lower_bound_x0_vec(i) +
                  lower_unit_vector.x() * second_lower_soft_bound);
        planning_input.mutable_second_soft_lower_bound_y0_vec()->Set(
            i, origin_planning_input.second_soft_lower_bound_y0_vec(i) +
                  lower_unit_vector.y() * second_lower_soft_bound);
      }
      if (i > 0) {
        Eigen::Vector2d lower_unit_vector1(
            origin_planning_input.second_soft_lower_bound_x1_vec(i - 1) -
                origin_planning_input.second_soft_upper_bound_x1_vec(i - 1),
            origin_planning_input.second_soft_lower_bound_y1_vec(i - 1) -
                origin_planning_input.second_soft_upper_bound_y1_vec(i - 1));
        lower_unit_vector1.normalize();
        planning_input.mutable_second_soft_lower_bound_x1_vec()->Set(
            i - 1, origin_planning_input.second_soft_lower_bound_x1_vec(i - 1) +
                  lower_unit_vector1.x() * second_lower_soft_bound);
        planning_input.mutable_second_soft_lower_bound_y1_vec()->Set(
            i - 1, origin_planning_input.second_soft_lower_bound_y1_vec(i - 1) +
                  lower_unit_vector1.y() * second_lower_soft_bound);
      }
    }
    planning_input.mutable_second_soft_lower_bound_x0_vec()->Set(N - 1, planning_input.second_soft_lower_bound_x0_vec(N - 2));
    planning_input.mutable_second_soft_lower_bound_y0_vec()->Set(N - 1, planning_input.second_soft_lower_bound_y0_vec(N - 2));
    planning_input.mutable_second_soft_lower_bound_x1_vec()->Set(N - 1, planning_input.second_soft_lower_bound_x1_vec(N - 2));
    planning_input.mutable_second_soft_lower_bound_y1_vec()->Set(N - 1, planning_input.second_soft_lower_bound_y1_vec(N - 2));

    for (size_t i = second_soft_ub_start_idx; i < second_soft_ub_end_idx; i++) {
      if (i < N - 1) {
        Eigen::Vector2d upper_unit_vector(
            origin_planning_input.second_soft_upper_bound_x0_vec(i) -
                origin_planning_input.second_soft_lower_bound_x0_vec(i),
            origin_planning_input.second_soft_upper_bound_y0_vec(i) -
                origin_planning_input.second_soft_lower_bound_y0_vec(i));
        upper_unit_vector.normalize();
        planning_input.mutable_second_soft_upper_bound_x0_vec()->Set(
            i, origin_planning_input.second_soft_upper_bound_x0_vec(i) +
                  upper_unit_vector.x() * second_upper_soft_bound);
        planning_input.mutable_second_soft_upper_bound_y0_vec()->Set(
            i, origin_planning_input.second_soft_upper_bound_y0_vec(i) +
                  upper_unit_vector.y() * second_upper_soft_bound);
      }
      if (i > 0) {
        Eigen::Vector2d upper_unit_vector1(
            origin_planning_input.second_soft_upper_bound_x1_vec(i - 1) -
                origin_planning_input.second_soft_lower_bound_x1_vec(i - 1),
            origin_planning_input.second_soft_upper_bound_y1_vec(i - 1) -
                origin_planning_input.second_soft_lower_bound_y1_vec(i - 1));
        upper_unit_vector1.normalize();
        planning_input.mutable_second_soft_upper_bound_x1_vec()->Set(
          i - 1, origin_planning_input.second_soft_upper_bound_x1_vec(i - 1) +
                upper_unit_vector1.x() * second_upper_soft_bound);
        planning_input.mutable_second_soft_upper_bound_y1_vec()->Set(
          i - 1, origin_planning_input.second_soft_upper_bound_y1_vec(i - 1) +
                upper_unit_vector1.y() * second_upper_soft_bound);
      }
    }
    planning_input.mutable_second_soft_upper_bound_x0_vec()->Set(N - 1, planning_input.second_soft_upper_bound_x0_vec(N - 2));
    planning_input.mutable_second_soft_upper_bound_y0_vec()->Set(N - 1, planning_input.second_soft_upper_bound_y0_vec(N - 2));
    planning_input.mutable_second_soft_upper_bound_x1_vec()->Set(N - 1, planning_input.second_soft_upper_bound_x1_vec(N - 2));
    planning_input.mutable_second_soft_upper_bound_y1_vec()->Set(N - 1, planning_input.second_soft_upper_bound_y1_vec(N - 2));
  } else {
    // second bound
    if (second_soft_lb_start_idx < 0) {
      second_soft_lb_start_idx = 0;
    }
    if (second_soft_lb_end_idx > 26) {
      second_soft_lb_end_idx = 26;
    }
    if (second_soft_ub_start_idx < 0) {
      second_soft_ub_start_idx = 0;
    }
    if (second_soft_ub_end_idx > 26) {
      second_soft_ub_end_idx = 26;
    }
    for (size_t i = second_soft_lb_start_idx; i < second_soft_lb_end_idx; i++) {
      if (i < N - 1) {
        Eigen::Vector2d lower_unit_vector(
            origin_planning_input.soft_lower_bound_x0_vec(i) -
                origin_planning_input.soft_upper_bound_x0_vec(i),
            origin_planning_input.soft_lower_bound_y0_vec(i) -
                origin_planning_input.soft_upper_bound_y0_vec(i));
        lower_unit_vector.normalize();
        planning_input.mutable_soft_lower_bound_x0_vec()->Set(
            i, origin_planning_input.soft_lower_bound_x0_vec(i) +
                  lower_unit_vector.x() * second_lower_soft_bound);
        planning_input.mutable_soft_lower_bound_y0_vec()->Set(
            i, origin_planning_input.soft_lower_bound_y0_vec(i) +
                  lower_unit_vector.y() * second_lower_soft_bound);
      }
      if (i > 0) {
        Eigen::Vector2d lower_unit_vector1(
            origin_planning_input.soft_lower_bound_x1_vec(i - 1) -
                origin_planning_input.soft_upper_bound_x1_vec(i - 1),
            origin_planning_input.soft_lower_bound_y1_vec(i - 1) -
                origin_planning_input.soft_upper_bound_y1_vec(i - 1));
        lower_unit_vector1.normalize();
        planning_input.mutable_soft_lower_bound_x1_vec()->Set(
            i - 1, origin_planning_input.soft_lower_bound_x1_vec(i - 1) +
                  lower_unit_vector1.x() * second_lower_soft_bound);
        planning_input.mutable_soft_lower_bound_y1_vec()->Set(
            i - 1, origin_planning_input.soft_lower_bound_y1_vec(i - 1) +
                  lower_unit_vector1.y() * second_lower_soft_bound);
      }
    }
    planning_input.mutable_soft_lower_bound_x0_vec()->Set(N - 1, planning_input.soft_lower_bound_x0_vec(N - 2));
    planning_input.mutable_soft_lower_bound_y0_vec()->Set(N - 1, planning_input.soft_lower_bound_y0_vec(N - 2));
    planning_input.mutable_soft_lower_bound_x1_vec()->Set(N - 1, planning_input.soft_lower_bound_x1_vec(N - 2));
    planning_input.mutable_soft_lower_bound_y1_vec()->Set(N - 1, planning_input.soft_lower_bound_y1_vec(N - 2));

    for (size_t i = second_soft_ub_start_idx; i < second_soft_ub_end_idx; i++) {
      if (i < N - 1) {
        Eigen::Vector2d upper_unit_vector(
            origin_planning_input.soft_upper_bound_x0_vec(i) -
                origin_planning_input.soft_lower_bound_x0_vec(i),
            origin_planning_input.soft_upper_bound_y0_vec(i) -
                origin_planning_input.soft_lower_bound_y0_vec(i));
        upper_unit_vector.normalize();
        planning_input.mutable_soft_upper_bound_x0_vec()->Set(
            i, origin_planning_input.soft_upper_bound_x0_vec(i) +
                  upper_unit_vector.x() * second_upper_soft_bound);
        planning_input.mutable_soft_upper_bound_y0_vec()->Set(
            i, origin_planning_input.soft_upper_bound_y0_vec(i) +
                  upper_unit_vector.y() * second_upper_soft_bound);
      }
      if (i > 0) {
        Eigen::Vector2d upper_unit_vector1(
            origin_planning_input.soft_upper_bound_x1_vec(i - 1) -
                origin_planning_input.soft_lower_bound_x1_vec(i - 1),
            origin_planning_input.soft_upper_bound_y1_vec(i - 1) -
                origin_planning_input.soft_lower_bound_y1_vec(i - 1));
        upper_unit_vector1.normalize();
        planning_input.mutable_soft_upper_bound_x1_vec()->Set(
          i - 1, origin_planning_input.soft_upper_bound_x1_vec(i - 1) +
                upper_unit_vector1.x() * second_upper_soft_bound);
        planning_input.mutable_soft_upper_bound_y1_vec()->Set(
          i - 1, origin_planning_input.soft_upper_bound_y1_vec(i - 1) +
                upper_unit_vector1.y() * second_upper_soft_bound);
      }
    }
    planning_input.mutable_soft_upper_bound_x0_vec()->Set(N - 1, planning_input.soft_upper_bound_x0_vec(N - 2));
    planning_input.mutable_soft_upper_bound_y0_vec()->Set(N - 1, planning_input.soft_upper_bound_y0_vec(N - 2));
    planning_input.mutable_soft_upper_bound_x1_vec()->Set(N - 1, planning_input.soft_upper_bound_x1_vec(N - 2));
    planning_input.mutable_soft_upper_bound_y1_vec()->Set(N - 1, planning_input.soft_upper_bound_y1_vec(N - 2));
  }
  // hard bound
  if (hard_lb_start_idx < 0) {
    hard_lb_start_idx = 0;
  }
  if (hard_lb_end_idx > 26) {
    hard_lb_end_idx = 26;
  }
  if (hard_ub_start_idx < 0) {
    hard_ub_start_idx = 0;
  }
  if (hard_ub_end_idx > 26) {
    hard_ub_end_idx = 26;
  }
  for (size_t i = hard_lb_start_idx; i < hard_lb_end_idx; i++) {
    if (i < N - 1) {
      Eigen::Vector2d lower_unit_vector(
          origin_planning_input.hard_lower_bound_x0_vec(i) -
              origin_planning_input.hard_upper_bound_x0_vec(i),
          origin_planning_input.hard_lower_bound_y0_vec(i) -
              origin_planning_input.hard_upper_bound_y0_vec(i));
      lower_unit_vector.normalize();
      planning_input.mutable_hard_lower_bound_x0_vec()->Set(
          i, origin_planning_input.hard_lower_bound_x0_vec(i) +
                lower_unit_vector.x() * lower_hard_bound);
      planning_input.mutable_hard_lower_bound_y0_vec()->Set(
          i, origin_planning_input.hard_lower_bound_y0_vec(i) +
                lower_unit_vector.y() * lower_hard_bound);
    }
    if (i > 0) {
      Eigen::Vector2d lower_unit_vector1(
          origin_planning_input.hard_lower_bound_x1_vec(i - 1) -
              origin_planning_input.hard_upper_bound_x1_vec(i - 1),
          origin_planning_input.hard_lower_bound_y1_vec(i - 1) -
              origin_planning_input.hard_upper_bound_y1_vec(i - 1));
      lower_unit_vector1.normalize();
      planning_input.mutable_hard_lower_bound_x1_vec()->Set(
        i - 1, origin_planning_input.hard_lower_bound_x1_vec(i - 1) +
               lower_unit_vector1.x() * lower_hard_bound);
      planning_input.mutable_hard_lower_bound_y1_vec()->Set(
        i - 1, origin_planning_input.hard_lower_bound_y1_vec(i - 1) +
               lower_unit_vector1.y() * lower_hard_bound);
    }
  }
  planning_input.mutable_hard_lower_bound_x0_vec()->Set(N - 1, planning_input.hard_lower_bound_x0_vec(N - 2));
  planning_input.mutable_hard_lower_bound_y0_vec()->Set(N - 1, planning_input.hard_lower_bound_y0_vec(N - 2));
  planning_input.mutable_hard_lower_bound_x1_vec()->Set(N - 1, planning_input.hard_lower_bound_x1_vec(N - 2));
  planning_input.mutable_hard_lower_bound_y1_vec()->Set(N - 1, planning_input.hard_lower_bound_y1_vec(N - 2));

  for (size_t i = hard_ub_start_idx; i < hard_ub_end_idx; i++) {
    if (i < N - 1) {
      Eigen::Vector2d upper_unit_vector(
          origin_planning_input.hard_upper_bound_x0_vec(i) -
              origin_planning_input.hard_lower_bound_x0_vec(i),
          origin_planning_input.hard_upper_bound_y0_vec(i) -
              origin_planning_input.hard_lower_bound_y0_vec(i));
      upper_unit_vector.normalize();
      planning_input.mutable_hard_upper_bound_x0_vec()->Set(
          i, origin_planning_input.hard_upper_bound_x0_vec(i) +
                upper_unit_vector.x() * upper_hard_bound);
      planning_input.mutable_hard_upper_bound_y0_vec()->Set(
          i, origin_planning_input.hard_upper_bound_y0_vec(i) +
                upper_unit_vector.y() * upper_hard_bound);
    }
    if (i > 0) {
      Eigen::Vector2d upper_unit_vector1(
          origin_planning_input.hard_upper_bound_x1_vec(i - 1) -
              origin_planning_input.hard_lower_bound_x1_vec(i - 1),
          origin_planning_input.hard_upper_bound_y1_vec(i - 1) -
              origin_planning_input.hard_lower_bound_y1_vec(i - 1));
      upper_unit_vector1.normalize();
      planning_input.mutable_hard_upper_bound_x1_vec()->Set(
        i - 1, origin_planning_input.hard_upper_bound_x1_vec(i - 1) +
               upper_unit_vector1.x() * upper_hard_bound);
      planning_input.mutable_hard_upper_bound_y1_vec()->Set(
        i - 1, origin_planning_input.hard_upper_bound_y1_vec(i - 1) +
               upper_unit_vector1.y() * upper_hard_bound);
    }
  }
  planning_input.mutable_hard_upper_bound_x0_vec()->Set(N - 1, planning_input.hard_upper_bound_x0_vec(N - 2));
  planning_input.mutable_hard_upper_bound_y0_vec()->Set(N - 1, planning_input.hard_upper_bound_y0_vec(N - 2));
  planning_input.mutable_hard_upper_bound_x1_vec()->Set(N - 1, planning_input.hard_upper_bound_x1_vec(N - 2));
  planning_input.mutable_hard_upper_bound_y1_vec()->Set(N - 1, planning_input.hard_upper_bound_y1_vec(N - 2));

  // const double ref_s = planning_input.ref_vel() * 5.0;
  // std::vector<double> ref_vel_vec;
  // std::vector<double> model_dt_vec;
  // ref_vel_vec.resize(26, 0.0);
  // model_dt_vec.resize(26, 0.0);
  // double new_ref_s = 0.0;
  // for (size_t i = 0; i < 26; ++i) {
  //   ref_vel_vec[i] = std::min(planning_input.ref_vel(), ego_v + ref_acc * (0.2 * i));
  //   double new_dt = 0.0;
  //   if (i > 0) {
  //     double ref_ds = planning_input.ref_vel() * 0.2 * i;
  //     double new_ref_ds = 0.5 * (ref_vel_vec[i] + ref_vel_vec[i - 1]) * 0.2;
  //     new_ref_s += new_ref_ds;
  //     new_dt = 0.2 * i * (new_ref_s / ref_ds);
  //   }
  //   model_dt_vec[i] = new_dt;
  // }

  pBase->SimUpdate(expected_acc, start_acc, end_acc,
                   end_ratio1, end_ratio2, end_ratio3,
                   max_iter, motion_plan_concerned_start_index,
                   start_w_jerk, ego_v, max_delta, max_omega,
                   wheel_base, q_front_ref_xy,
                   q_virtual_ref_xy, q_virtual_ref_theta,
                   virtual_ref_x, virtual_ref_y, virtual_ref_theta,
                   planning_input);
  // output
  const auto& ilqr_core_ptr = pBase->GetiLqrCorePtr();
  const auto& state_result = ilqr_core_ptr->GetStateResultPtr();
  const auto& control_result = ilqr_core_ptr->GetControlResultPtr();
  const auto& dt = ilqr_core_ptr->GetSolverConfigPtr()->model_dt;
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    double ref_vel = planning_input.ref_vel_vec(i);
    if (i < N - 1) {
      ref_vel = (ref_vel + planning_input.ref_vel_vec(i + 1)) * 0.5;
    }
    double kv2 = planning_input.curv_factor() * ref_vel * ref_vel;
    planning_output.mutable_time_vec()->Set(i, t);
    t += dt;

    planning_output.mutable_x_vec()->Set(
        i, state_result->at(i)[StateID::X]);
    planning_output.mutable_y_vec()->Set(
        i, state_result->at(i)[StateID::Y]);
    planning_output.mutable_theta_vec()->Set(
        i, state_result->at(i)[StateID::THETA]);
    planning_output.mutable_delta_vec()->Set(
        i, state_result->at(i)[StateID::DELTA]);
    planning_output.mutable_acc_vec()->Set(
        i, kv2 * state_result->at(i)[StateID::DELTA]);

    if (i < N - 1) {
      planning_output.mutable_omega_vec()->Set(
          i, control_result->at(i)[ControlID::OMEGA]);
      planning_output.mutable_jerk_vec()->Set(
          i,
          kv2 * control_result->at(i)[ControlID::OMEGA]);
    } else {
      planning_output.mutable_omega_vec()->Set(
          i, planning_output.omega_vec(i - 1));
      planning_output.mutable_jerk_vec()->Set(
          i, planning_output.jerk_vec(i - 1));
    }
  }

  // load solver and iteration info
  planning_output.clear_solver_info();

  const auto &soler_info_ptr = ilqr_core_ptr->GetSolverInfoPtr();

  planning_output.mutable_solver_info()->set_solver_condition(
      soler_info_ptr->solver_condition);
  planning_output.mutable_solver_info()->set_cost_size(
      soler_info_ptr->cost_size);
  planning_output.mutable_solver_info()->set_iter_count(
      soler_info_ptr->iter_count);
  planning_output.mutable_solver_info()->set_init_cost(
      soler_info_ptr->init_cost);

  for (size_t i = 0; i < soler_info_ptr->iter_count; ++i) {
    const auto& iter_info =
        planning_output.mutable_solver_info()->add_iter_info();
    iter_info->set_linesearch_success(
        soler_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        soler_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda(soler_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(soler_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(soler_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(soler_info_ptr->iteration_info_vec[i].du_norm);
    for (size_t j = 0; j < soler_info_ptr->iteration_info_vec[i].cost_vec.size(); ++j) {
      const auto& cost_info = iter_info->add_cost_info();
      cost_info->set_id(soler_info_ptr->iteration_info_vec[i].cost_vec[j].id);
      cost_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost_vec[j].cost);
    }
  }

  return 0;
}

// planning::common::LateralPlanningOutput GetOutput() {
//   auto res = pBase->GetOutput();

//   return res;
// }

PYBIND11_MODULE(lateral_motion_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes);
}
