#include <stdio.h>

#include <cstddef>

#include "gtest/gtest.h"
#include "ilqr_define.h"
#include "motion_planners/longitudinal_motion_planner/src/longitudinal_motion_planning_cost.h"
#include "motion_planners/longitudinal_motion_planner/src/longitudinal_motion_planning_problem.h"

using namespace std;
using namespace pnc::longitudinal_planning;

class LongitudinalMotionPlanningTest : public ::testing::Test {
 protected:
  void init() {
    planning_input_.init_state.resize(STATE_SIZE);
    planning_input_.init_state << 0.0, 13.0, -0.0, 0.0;

    planning_input_.ref_pos_vec.reserve(26);
    planning_input_.ref_vel_vec.reserve(26);

    for (size_t i = 0; i < 26; ++i) {
      planning_input_.ref_vel_vec.emplace_back(0.0);
      planning_input_.ref_pos_vec.emplace_back(15.0);
    }

    planning_input_.pos_max_vec.reserve(26);
    planning_input_.pos_min_vec.reserve(26);
    planning_input_.vel_max_vec.reserve(26);
    planning_input_.vel_min_vec.reserve(26);
    planning_input_.acc_max_vec.reserve(26);
    planning_input_.acc_min_vec.reserve(26);
    planning_input_.jerk_max_vec.reserve(26);
    planning_input_.jerk_min_vec.reserve(26);

    for (size_t i = 0; i < 26; ++i) {
      planning_input_.pos_max_vec.emplace_back(12.0);
      planning_input_.pos_min_vec.emplace_back(0.0);
      planning_input_.vel_max_vec.emplace_back(11.0);
      planning_input_.vel_min_vec.emplace_back(6.0);

      if (i < 24) {
        planning_input_.acc_max_vec.emplace_back(4.0);
        planning_input_.acc_min_vec.emplace_back(-5.0);
      } else {
        planning_input_.acc_max_vec.emplace_back(2.0);
        planning_input_.acc_min_vec.emplace_back(-0.3);
      }

      planning_input_.jerk_max_vec.emplace_back(2.0);
      planning_input_.jerk_min_vec.emplace_back(-5.0);
    }

    planning_input_.s_stop = 15.0;

    // set weights: use default
    planning_input_.q_ref_pos = 0.01;
    planning_input_.q_ref_vel = 0.1;

    planning_input_.q_acc = 1.5;
    planning_input_.q_jerk = 1.5;
    planning_input_.q_snap = 2000.0;
    planning_input_.q_pos_bound = 100.0;
    planning_input_.q_vel_bound = 100.0;
    planning_input_.q_acc_bound = 100.0;
    planning_input_.q_jerk_bound = 100.0;

    planning_input_.q_stop_s = 500.0;
  }

 public:
  LongitudinalMotionPlanningInput planning_input_;
  LongitudinalMotionPlanningProblem lon_planning_;
};

// TEST_F(LongitudinalMotionPlanningTest, OneFrameTest) {
//   this->init();
//   this->lon_planning_.Init();
//   auto solver_flag = this->lon_planning_.Update(this->planning_input_);

//   auto planning_output = this->lon_planning_.GetOutput();

//   for (size_t i = 0; i < planning_output.pos_vec.size(); i++) {
//     printf("pos = %.4f, vel = %.4f, acc = %.4f, jerk = %.4f\n",
//            planning_output.pos_vec[i], planning_output.vel_vec[i],
//            planning_output.acc_vec[i], planning_output.jerk_vec[i]);
//   }

//   EXPECT_TRUE(solver_flag > ilqr_solver::iLqr::INIT);

//   printf("solver_flag = %d\n", solver_flag);

//   EXPECT_TRUE(solver_flag < ilqr_solver::iLqr::BACKWARD_PASS_FAIL);
// }

// int main(int argc, char **argv) {
//   printf("Testing LongitudinalMotionPlanning!");
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
