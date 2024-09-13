
#include <gtest/gtest.h>

#include "pose2d.h"
#include "reeds_shepp.h"
#include "reeds_shepp_interface.h"
#include "rs_path_interpolate.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_common.h"

using namespace planning;

TEST(test_reeds_shepp, test_GetShortestRSPathParam) {
  int ret;
  double error, min_radius;
  Pose2D start_pose, goal_pose;
  RSPathParam path;

  error = 1e-5;
  start_pose.x = 0.0;
  start_pose.y = 0.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 0.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  min_radius = 3.179429;

  ret = GetShortestRSPathParam(NULL, &start_pose, &goal_pose, min_radius,
                               1.0 / min_radius);

  ret = GetShortestRSPathParam(&path, NULL, &goal_pose, min_radius,
                               1.0 / min_radius);

  ret = GetShortestRSPathParam(&path, &start_pose, NULL, min_radius,
                               1.0 / min_radius);

  ret = GetShortestRSPathParam(&path, &start_pose, &goal_pose, 0.0, 0.0);

  ret = GetShortestRSPathParam(&path, &start_pose, &goal_pose, min_radius,
                               1.0 / min_radius);

  EXPECT_EQ(ret, 0);
  EXPECT_NEAR(path.t, 0.0, error);
  EXPECT_NEAR(path.u, 0.62904, error);
  EXPECT_NEAR(path.v, 0.0, error);
  EXPECT_NEAR(path.w, 0.0, error);
  EXPECT_NEAR(path.x, 0.0, error);
  EXPECT_NEAR(path.length[0], 0.0, error);
  EXPECT_NEAR(path.length[1], 2.0, error);
  EXPECT_NEAR(path.length[2], 0.0, error);
  EXPECT_NEAR(path.length[3], 0.0, error);
  EXPECT_NEAR(path.length[4], 0.0, error);
  EXPECT_NEAR(path.total_length, 2.0, error);
  EXPECT_EQ(path.type[0], 1);
  EXPECT_EQ(path.type[1], 2);
  EXPECT_EQ(path.type[2], 1);
  EXPECT_EQ(path.type[3], 0);
  EXPECT_EQ(path.type[4], 0);

  start_pose.x = 1.0;
  start_pose.y = 1.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 1.0;
  goal_pose.y = 3.0;
  goal_pose.theta = M_PI / 2.0;
  ret = GetShortestRSPathParam(&path, &start_pose, &goal_pose, min_radius,
                               1.0 / min_radius);
  EXPECT_EQ(ret, 0);

  EXPECT_NEAR(path.t, 0.0, error);
  EXPECT_NEAR(path.u, 0.62904, error);
  EXPECT_NEAR(path.v, 0.0, error);
  EXPECT_NEAR(path.w, 0.0, error);
  EXPECT_NEAR(path.x, 0.0, error);
  EXPECT_NEAR(path.length[0], 0.0, error);
  EXPECT_NEAR(path.length[1], 2.0, error);
  EXPECT_NEAR(path.length[2], 0.0, error);
  EXPECT_NEAR(path.length[3], 0.0, error);
  EXPECT_NEAR(path.length[4], 0.0, error);
  EXPECT_NEAR(path.total_length, 2.0, error);
  EXPECT_EQ(path.type[0], 1);
  EXPECT_EQ(path.type[1], 2);
  EXPECT_EQ(path.type[2], 1);
  EXPECT_EQ(path.type[3], 0);
  EXPECT_EQ(path.type[4], 0);

  start_pose.x = 1.0;
  start_pose.y = 1.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 1.0;
  goal_pose.y = 1.0;
  goal_pose.theta = M_PI / 2.0;
  ret = GetShortestRSPathParam(&path, &start_pose, &goal_pose, min_radius,
                               1.0 / min_radius);

  EXPECT_EQ(ret, 0);
  EXPECT_NEAR(path.t, 0.0, error);
  EXPECT_NEAR(path.u, 0.0, error);
  EXPECT_NEAR(path.v, 0.0, error);
  EXPECT_NEAR(path.w, 0.0, error);
  EXPECT_NEAR(path.x, 0.0, error);
  EXPECT_NEAR(path.length[0], 0.0, error);
  EXPECT_NEAR(path.length[1], 0.0, error);
  EXPECT_NEAR(path.length[2], 0.0, error);
  EXPECT_NEAR(path.length[3], 0.0, error);
  EXPECT_NEAR(path.length[4], 0.0, error);
  EXPECT_NEAR(path.total_length, 0.0, error);
  EXPECT_EQ(path.type[0], 1);
  EXPECT_EQ(path.type[1], 3);
  EXPECT_EQ(path.type[2], 1);
  EXPECT_EQ(path.type[3], 0);
  EXPECT_EQ(path.type[4], 0);
}

TEST(test_reeds_shepp, test_CalcRSPathKappa) {
  int ret;
  RSPathParam path;
  Pose2D start_pose, goal_pose;
  double min_radius, error;
  RSPathKappaParam control_list;

  error = 1e-5;
  start_pose.x = 0.0;
  start_pose.y = 0.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 0.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  min_radius = 3.179429;

  ret = GetShortestRSPathParam(&path, &start_pose, &goal_pose, min_radius,
                               1.0 / min_radius);
  EXPECT_EQ(ret, 0);

  RSPathInterpolator interpolator;

  ret = interpolator.CalcRSPathKappa(&control_list, NULL, min_radius);

  ret = interpolator.CalcRSPathKappa(NULL, &path, min_radius);

  ret = interpolator.CalcRSPathKappa(&control_list, &path, 0.0);

  ret = interpolator.CalcRSPathKappa(&control_list, &path, min_radius);
  EXPECT_EQ(ret, 0);

  EXPECT_EQ(control_list.size, 1);
  EXPECT_NEAR(control_list.path_kappa[0].length, 2, error);
  EXPECT_NEAR(control_list.path_kappa[0].kappa, 0.0, error);

  start_pose.x = 1.0;
  start_pose.y = 2.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 1.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  ret = GetShortestRSPathParam(&path, &start_pose, &goal_pose, min_radius,
                               1.0 / min_radius);
  EXPECT_EQ(ret, 0);

  ret = interpolator.CalcRSPathKappa(&control_list, &path, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 0);
}

TEST(test_reeds_shepp, test_GetRSPathDist) {
  int ret;
  double error, distance, min_radius;
  Pose2D start_pose, goal_pose;

  error = 1e-5;
  start_pose.x = 0.0;
  start_pose.y = 0.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 0.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  min_radius = 3.179429;

  ret = GetRSPathDist(NULL, &start_pose, &goal_pose, min_radius);

  ret = GetRSPathDist(&distance, NULL, &goal_pose, min_radius);

  ret = GetRSPathDist(&distance, &start_pose, NULL, min_radius);

  ret = GetRSPathDist(&distance, &start_pose, &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_NEAR(distance, 2.0, error);

  /* use path_info, test right */
  ret = GetRSPathDist(&distance, &start_pose, &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_NEAR(distance, 2.0, error);

  start_pose.x = 1.0;
  start_pose.y = 2.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 1.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;

  ret = GetRSPathDist(&distance, &start_pose, &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_NEAR(distance, 0.0, error);
}

TEST(test_reeds_shepp, test_CalcShortestRSPathKappa) {
  int ret;
  double error, min_radius;
  Pose2D start_pose, goal_pose;
  RSPathKappaParam control_list;

  error = 1e-5;
  start_pose.x = 0.0;
  start_pose.y = 0.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 0.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  min_radius = 3.179429;

  RSPathInterpolator interpolator;

  ret = interpolator.CalcShortestRSPathKappa(NULL, &start_pose, &goal_pose,
                                             min_radius);

  ret = interpolator.CalcShortestRSPathKappa(&control_list, NULL, &goal_pose,
                                             min_radius);

  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose, NULL,
                                             min_radius);

  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                             &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 1);
  EXPECT_NEAR(control_list.path_kappa[0].length, 2.0, error);
  EXPECT_NEAR(control_list.path_kappa[0].kappa, 0.0, error);

  /* use path_info, test right */
  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                             &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 1);
  EXPECT_NEAR(control_list.path_kappa[0].length, 2, error);
  EXPECT_NEAR(control_list.path_kappa[0].kappa, 0, error);

  start_pose.x = 1.0;
  start_pose.y = 2.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 1.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;

  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                             &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 0);
}

TEST(test_reeds_shepp, test_GetRSPathGearSwitchNum) {
  int ret;
  int gear_switch_num;
  Pose2D start_pose, goal_pose;
  double min_radius;
  AstarPathGear initial_pose_dir;

  start_pose.x = 1.0;
  start_pose.y = 2.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 1.0;
  goal_pose.y = 3.0;
  goal_pose.theta = M_PI / 2.0;
  initial_pose_dir = AstarPathGear::reverse;
  min_radius = 3.179429;

  ret = GetRSPathGearSwitchNum(NULL, &start_pose, &goal_pose, min_radius,
                               initial_pose_dir);

  ret = GetRSPathGearSwitchNum(&gear_switch_num, NULL, &goal_pose, min_radius,
                               initial_pose_dir);

  ret = GetRSPathGearSwitchNum(&gear_switch_num, &start_pose, NULL, min_radius,
                               initial_pose_dir);

  ret = GetRSPathGearSwitchNum(&gear_switch_num, &start_pose, &goal_pose,
                               min_radius, initial_pose_dir);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(gear_switch_num, 1);

  /* use path_info, test right */
  ret = GetRSPathGearSwitchNum(&gear_switch_num, &start_pose, &goal_pose,
                               min_radius, initial_pose_dir);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(gear_switch_num, 1);

  goal_pose.x = 1.0;
  goal_pose.y = 1.0;
  goal_pose.theta = M_PI / 2.0;
  ret = GetRSPathGearSwitchNum(&gear_switch_num, &start_pose, &goal_pose,
                               min_radius, initial_pose_dir);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(gear_switch_num, 0);

  goal_pose.x = 1.0;
  goal_pose.y = 3.0;
  goal_pose.theta = M_PI / 2.0;
  initial_pose_dir = AstarPathGear::none;
  ret = GetRSPathGearSwitchNum(&gear_switch_num, &start_pose, &goal_pose,
                               min_radius, initial_pose_dir);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(gear_switch_num, 0);

  goal_pose.x = 1.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  initial_pose_dir = AstarPathGear::reverse;
  min_radius = 3.179429;

  ret = GetRSPathGearSwitchNum(&gear_switch_num, &start_pose, &goal_pose,
                               min_radius, initial_pose_dir);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(gear_switch_num, 0);
}

TEST(test_reeds_shepp, test_reeds_shepp_use_path_info_1) {
  int ret;
  int i;
  double error, min_radius, distance;
  Pose2D start_pose, goal_pose;
  RSPathKappaParam control_list;

  error = 1e-5;
  start_pose.x = 0.0;
  start_pose.y = 0.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 0.0;
  goal_pose.y = 2.0;
  goal_pose.theta = M_PI / 2.0;
  min_radius = 3.179429;

  RSPathInterpolator interpolator;

  /* use path_info, test right */
  for (i = 0; i < 3; i++) {
    ret = GetRSPathDist(&distance, &start_pose, &goal_pose, min_radius);
    EXPECT_EQ(ret, 0);
    EXPECT_NEAR(distance, 2.0, error);

    ret = GetRSPathDist(&distance, &start_pose, &goal_pose, min_radius);
    EXPECT_EQ(ret, 0);
    EXPECT_NEAR(distance, 2.0, error);

    ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                               &goal_pose, min_radius);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(control_list.size, 1);
    EXPECT_NEAR(control_list.path_kappa[0].length, 2.0, error);
    EXPECT_NEAR(control_list.path_kappa[0].kappa, 0.0, error);
  }
}

TEST(test_reeds_shepp, test_reeds_shepp_use_path_info_2) {
  int ret;
  int i;
  double error, min_radius, distance;
  Pose2D start_pose, goal_pose;
  RSPathKappaParam control_list;

  /* not_use path_info, test right */
  error = 1e-5;
  start_pose.x = 0.0;
  start_pose.y = 0.0;
  start_pose.theta = M_PI / 2.0;
  goal_pose.x = 0.0;
  goal_pose.y = 1.0;
  goal_pose.theta = M_PI / 2.0;
  min_radius = 3.179429;

  RSPathInterpolator interpolator;

  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                             &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 1);
  EXPECT_NEAR(control_list.path_kappa[0].length, 1.0, error);
  EXPECT_NEAR(control_list.path_kappa[0].kappa, 0.0, error);

  goal_pose.x = 0.0;
  goal_pose.y = 3.0;
  goal_pose.theta = M_PI / 2.0;
  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                             &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 1);
  EXPECT_NEAR(control_list.path_kappa[0].length, 3.0, error);
  EXPECT_NEAR(control_list.path_kappa[0].kappa, 0.0, error);

  goal_pose.x = 0.0;
  goal_pose.y = 1.0;
  goal_pose.theta = M_PI / 2.0;
  ret = interpolator.CalcShortestRSPathKappa(&control_list, &start_pose,
                                             &goal_pose, min_radius);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(control_list.size, 1);
  EXPECT_NEAR(control_list.path_kappa[0].length, 1.0, error);
  EXPECT_NEAR(control_list.path_kappa[0].kappa, 0.0, error);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
