#include <gtest/gtest.h>

#include <iostream>

#include "cdl_math.h"
#include "cdl_perf.h"
#include "collision_detect_types.h"
#include "gjk2d.h"

TEST(test_collision_detection_cdl_math, test_GetClockwiseNormal) {
  cdl::Vector2r v;
  cdl::Vector2r v_result;
  v[0] = 0.0;
  v[1] = 0.0;
  v_result = cdl::GetClockwiseNormal(v);
  EXPECT_DOUBLE_EQ(v_result[0], 0.0);
  EXPECT_DOUBLE_EQ(v_result[1], 0.0);

  v[0] = 1.0;
  v[1] = 0.0;
  v_result = cdl::GetClockwiseNormal(v);
  EXPECT_DOUBLE_EQ(v_result[0], 0.0);
  EXPECT_DOUBLE_EQ(v_result[1], -1.0);

  v[0] = 0.0;
  v[1] = 1.0;
  v_result = cdl::GetClockwiseNormal(v);
  EXPECT_DOUBLE_EQ(v_result[0], 1.0);
  EXPECT_DOUBLE_EQ(v_result[1], 0.0);
}

TEST(test_collision_detection_cdl_math, test_GetCounterClockwiseNormal) {
  cdl::Vector2r v;
  cdl::Vector2r v_result;
  v[0] = 0.0;
  v[1] = 0.0;
  v_result = cdl::GetCounterClockwiseNormal(v);
  EXPECT_DOUBLE_EQ(v_result[0], 0.0);
  EXPECT_DOUBLE_EQ(v_result[1], 0.0);

  v[0] = 1.0;
  v[1] = 0.0;
  v_result = cdl::GetCounterClockwiseNormal(v);
  EXPECT_DOUBLE_EQ(v_result[0], 0.0);
  EXPECT_DOUBLE_EQ(v_result[1], 1.0);

  v[0] = 0.0;
  v[1] = 1.0;
  v_result = cdl::GetCounterClockwiseNormal(v);
  EXPECT_DOUBLE_EQ(v_result[0], -1.0);
  EXPECT_DOUBLE_EQ(v_result[1], 0.0);
}

TEST(test_collision_detection_cdl_math, test_GetVectorCross) {
  cdl::Vector2r p1, p2;
  cdl::real p_result;
  p1[0] = 1.0;
  p1[1] = 1.0;
  p2[0] = 1.0;
  p2[1] = 1.0;
  p_result = cdl::GetVectorCross(p1, p2);
  EXPECT_DOUBLE_EQ(p_result, 0.0);

  p1[0] = 1;
  p1[1] = 2;
  p2[0] = 3;
  p2[1] = 4;
  p_result = cdl::GetVectorCross(p1, p2);
  EXPECT_DOUBLE_EQ(p_result, -2.0);
}

TEST(test_collision_detection_cdl_math, test_dot2D) {
  cdl::Vector2r a, b;
  cdl::real p_result;
  a[0] = 1.0;
  a[1] = 1.0;
  b[0] = 1.0;
  b[1] = 1.0;
  p_result = cdl::GetVectorDot(a, b);
  EXPECT_DOUBLE_EQ(p_result, 2.0);

  a[0] = 1.0;
  a[1] = 2.0;
  b[0] = 3.0;
  b[1] = 4.0;
  p_result = cdl::GetVectorDot(a, b);
  EXPECT_DOUBLE_EQ(p_result, 11.0);
}