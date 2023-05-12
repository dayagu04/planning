#include <stdio.h>

#include <cstddef>

#include "gtest/gtest.h"
// #include "ilqr_define.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_problem.h"

using namespace std;
using namespace pnc::lateral_planning;

class LateralMotionPlanningTest : public ::testing::Test {
 protected:
  void init() {
    // planning_input_.init_state.resize(STATE_SIZE);
    planning_input_.add_init_state(0.0);
    planning_input_.add_init_state(0.0);
    planning_input_.add_init_state(45 / 57.3);
    planning_input_.add_init_state(0.0);
    planning_input_.add_init_state(0.0);

    // planning_input_.ref_x_vec.reserve(26);

    for (size_t i = 0; i < 26; ++i) {
      planning_input_.add_ref_x_vec(i * 0.1 * 5 * cos(45 / 57.3));
      planning_input_.add_ref_theta_vec(45 / 57.3);
    }
    for (size_t i = 0; i < 26; ++i) {
      planning_input_.add_ref_y_vec(i * 0.1 * 5 * sin(45 / 57.3));
    }
    std::cout << planning_input_.ref_y_vec_size() << std::endl;
    planning_input_.mutable_last_x_vec()->CopyFrom(planning_input_.ref_x_vec());
    planning_input_.mutable_last_y_vec()->CopyFrom(planning_input_.ref_y_vec());
    planning_input_.mutable_last_theta_vec()->CopyFrom(
        planning_input_.ref_theta_vec());

    planning_input_.set_ref_vel(5.0);
    planning_input_.set_curv_factor(0.276);

    // set corridor
    // set upper x0
    planning_input_.mutable_soft_upper_bound_x0_vec()->CopyFrom(
        planning_input_.ref_x_vec());

    // set lower x0
    planning_input_.mutable_soft_lower_bound_x0_vec()->CopyFrom(
        planning_input_.ref_x_vec());

    // set upper x1
    planning_input_.mutable_soft_upper_bound_x1_vec()->CopyFrom(
        planning_input_.soft_upper_bound_x0_vec());
    double tmp_upper_bound_x1;
    for (int i = 0; i < 26; i++) {
      tmp_upper_bound_x1 = planning_input_.soft_upper_bound_x1_vec(i);
      planning_input_.set_soft_upper_bound_x1_vec(i, tmp_upper_bound_x1 + 0.5);
    }

    // set lower x1
    planning_input_.mutable_soft_lower_bound_x1_vec()->CopyFrom(
        planning_input_.soft_upper_bound_x1_vec());

    // set upper y0
    planning_input_.mutable_soft_upper_bound_y0_vec()->CopyFrom(
        planning_input_.ref_y_vec());
    for (size_t i = 0; i < 26; i++) {
      planning_input_.set_soft_upper_bound_y0_vec(
          i, planning_input_.soft_upper_bound_y0_vec(i) + 0.5);
    }

    // set upper y1
    planning_input_.mutable_soft_upper_bound_y1_vec()->CopyFrom(
        planning_input_.soft_upper_bound_y0_vec());

    // set lower y0
    planning_input_.mutable_soft_lower_bound_y0_vec()->CopyFrom(
        planning_input_.ref_y_vec());

    for (size_t i = 0; i < 26; i++) {
      planning_input_.set_soft_upper_bound_y0_vec(
          i, planning_input_.soft_upper_bound_y0_vec(i) - 0.5);
    }
    // set lower y1
    planning_input_.mutable_soft_lower_bound_y1_vec()->CopyFrom(
        planning_input_.soft_lower_bound_y0_vec());

    // set acc jerk bound
    planning_input_.set_acc_bound(5.0);
    planning_input_.set_jerk_bound(3.0);

    // set weights: use default
  }

 public:
  planning::common::LateralMotionPlanningInput planning_input_;
  LateralMotionPlanningProblem lat_planning_;
};

TEST_F(LateralMotionPlanningTest, OneFrameTest) {
  this->init();
  this->lat_planning_.Init();
  auto solver_flag = this->lat_planning_.Update(this->planning_input_);

  // auto planning_output = this->lat_planning_.GetOutput();

  // for (size_t i = 0; i < planning_output.x_vec.size(); i++) {
  //   printf("x: %f, y: %f\n", planning_output.x_vec[i],
  //          planning_output.y_vec[i]);
  // }

  // for (size_t i = 0; i < planning_output.x_vec.size(); i++) {
  //   printf("theta: %f, delta: %f, omega: %f, u: %f \n",
  //          planning_output.theta_vec[i] * 57.3, planning_output.delta_vec[i],
  //          planning_output.omega_vec[i], planning_output.omega_dot_vec[i]);
  // }

  // EXPECT_TRUE(solver_flag > ilqr_solver::iLqr::INIT);

  // printf("solver_flag = %d\n", solver_flag);

  // EXPECT_TRUE(solver_flag < ilqr_solver::iLqr::BACKWARD_PASS_FAIL);
}
