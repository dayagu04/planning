#include "ilqr_define.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_problem.h"
#include "gtest/gtest.h"
#include <cstddef>
#include <stdio.h>

using namespace std;
using namespace pnc::lateral_planning;

class LateralMotionPlanningTest : public ::testing::Test {
protected:
  void init() {
    planning_input_.init_state.resize(STATE_SIZE);
    planning_input_.init_state << 0.0, 0.0, 45 / 57.3, 0.0, 0.0;

    planning_input_.ref_x_vec.reserve(26);

    for (size_t i = 0; i < 26; ++i) {
      planning_input_.ref_x_vec.emplace_back(i * 0.1 * 5 * cos(45 / 57.3));
    }
    for (size_t i = 0; i < 26; ++i) {
      planning_input_.ref_y_vec.emplace_back(i * 0.1 * 5 * sin(45 / 57.3));
    }

    planning_input_.ref_y_vec.resize(26, 0.0);
    planning_input_.ref_theta_vec.resize(26, 0.0);

    planning_input_.last_x_vec = planning_input_.ref_x_vec;
    planning_input_.last_y_vec = planning_input_.ref_y_vec;
    planning_input_.last_theta_vec = planning_input_.ref_theta_vec;

    planning_input_.ref_vel = 5.0;
    planning_input_.curv_factor = 0.276;

    // set corridor
    // set upper x0
    planning_input_.soft_upper_bound_x0_vec = planning_input_.ref_x_vec;

    // set lower x0
    planning_input_.soft_lower_bound_x0_vec = planning_input_.ref_x_vec;

    // set upper x1
    planning_input_.soft_upper_bound_x1_vec =
        planning_input_.soft_upper_bound_x0_vec;
    for (auto &x : planning_input_.soft_upper_bound_x1_vec) {
      x += 0.5;
    }

    // set lower x1
    planning_input_.soft_lower_bound_x1_vec =
        planning_input_.soft_upper_bound_x1_vec;

    // set upper y0
    planning_input_.soft_upper_bound_y0_vec = planning_input_.ref_y_vec;
    for (auto &y : planning_input_.soft_upper_bound_y0_vec) {
      y += 5.0;
    }

    // set upper y1
    planning_input_.soft_upper_bound_y1_vec =
        planning_input_.soft_upper_bound_y0_vec;

    // set lower y0
    planning_input_.soft_lower_bound_y0_vec = planning_input_.ref_y_vec;
    for (auto &y : planning_input_.soft_upper_bound_y0_vec) {
      y -= 5.0;
    }
    // set lower y1
    planning_input_.soft_lower_bound_y1_vec =
        planning_input_.soft_lower_bound_y0_vec;

    // set acc jerk bound
    planning_input_.acc_bound = 5.0;
    planning_input_.jerk_bound = 3.0;

    // set weights: use default
  }

public:
  LateralMotionPlanningInput planning_input_;
  LateralMotionPlanningProblem lat_planning_;
};

TEST_F(LateralMotionPlanningTest, OneFrameTest) {
  this->init();
  this->lat_planning_.Init();
  auto solver_flag = this->lat_planning_.Update(this->planning_input_);

  auto planning_output = this->lat_planning_.GetOutput();

  for (size_t i = 0; i < planning_output.x_vec.size(); i++) {
    printf("x: %f, y: %f\n", planning_output.x_vec[i],
           planning_output.y_vec[i]);
  }

  for (size_t i = 0; i < planning_output.x_vec.size(); i++) {
    printf("theta: %f, delta: %f, omega: %f, u: %f \n",
           planning_output.theta_vec[i] * 57.3, planning_output.delta_vec[i],
           planning_output.omega_vec[i], planning_output.omega_dot_vec[i]);
  }

  EXPECT_TRUE(solver_flag > ilqr_solver::iLqr::INIT);

  printf("solver_flag = %d\n", solver_flag);

  EXPECT_TRUE(solver_flag < ilqr_solver::iLqr::BACKWARD_PASS_FAIL);
}

int main(int argc, char **argv) {
  printf("Testing LateralMotionPlanning!");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
