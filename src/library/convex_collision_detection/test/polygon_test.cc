#include <gtest/gtest.h>
#include <cmath>

#include "polygon_base.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/convex_collision_detection/cdl_math.h"

using namespace planning;

/* Determine whether two points are the same */
static bool is_point_same(Position2D &point_a, Position2D &point_b) {
  if (!ifly_fequal(point_a.x, point_b.x)) {
    return false;
  }
  if (!ifly_fequal(point_a.y, point_b.y)) {
    return false;
  }
  return true;
}

/* Determine whether two polygons are the same */
static bool is_polygon_same(Polygon2D *p1, Polygon2D *p2) {
  uint32_t i;
  bool same = true;

  if (p1->vertex_num != p2->vertex_num) {
    printf("polygon vertex num diff %d %d \n", p1->vertex_num, p2->vertex_num);
  }
  for (i = 0; i < p1->vertex_num; i++) {
    same = same && (ifly_fequal(p1->vertexes[i].x, p2->vertexes[i].x) &&
                    ifly_fequal(p1->vertexes[i].y, p2->vertexes[i].y));
    if (!same) {
      printf(
          "polygon vertex [%d]  pos diff (%.2f %.2f),"
          "(%.2f %.2f),\n",
          i, p1->vertexes[i].x, p1->vertexes[i].x, p2->vertexes[i].x,
          p2->vertexes[i].y);
    }
  }

  return same;
}

TEST(InitPolygon, test01) {
  /* Wrong input,input is a null pointer. */
  int ret;
  ret = InitPolygon(NULL);
  EXPECT_EQ(ret, -1);
}

TEST(InitPolygon, test02) {
  /* function test: initialize the polygon */
  int32_t i;
  Polygon2D polygon;
  int ret;

  polygon.radius = 1.0;
  polygon.vertex_num = 2;
  polygon.vertexes[0].x = 2.0;
  polygon.shape = PolygonShape::line_segment;

  /* Test each parameter of initialization in the struct of polygon */
  ret = InitPolygon(&polygon);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(static_cast<int>(polygon.shape),
            static_cast<int>(PolygonShape::multi_edge));
  EXPECT_EQ(polygon.vertex_num, 0);
  EXPECT_DOUBLE_EQ(polygon.radius, 0.0);

  Position2D point = {0.0, 0.0};
  EXPECT_EQ(is_point_same(polygon.center_pt, point), true);

  for (i = 0; i < MAX_POLYGON_VERTEX_NUM; i++) {
    EXPECT_EQ(is_point_same(polygon.vertexes[i], point), true);
  }
  // InitPolygon() and memset() have the same function
  // Polygon2D polygon_B;
  // memset(&polygon_B, 0, sizeof(polygon_t));
  // EXPECT_TRUE(is_polygon_same(&polygon, &polygon_B));
}

TEST(PolygonCopy, test01) {
  /* Wrong input,input is a null pointer. */
  int ret;
  Polygon2D des_poly;
  Polygon2D src_poly;

  ret = PolygonCopy(&des_poly, NULL);
  EXPECT_EQ(ret, -1);

  ret = PolygonCopy(NULL, &src_poly);
  EXPECT_EQ(ret, -1);
}

TEST(PolygonCopy, test02) {
  /* function test */
  int ret;
  Polygon2D des_poly;
  Polygon2D src_poly;

  /* use the function of InitPolygon() to ensure all parameters are
   * initialized in the struct of des_poly and src_poly.
   * */
  ret = InitPolygon(&des_poly);
  EXPECT_EQ(ret, 0);
  ret = InitPolygon(&src_poly);
  EXPECT_EQ(ret, 0);

  src_poly.shape = PolygonShape::box;
  src_poly.vertex_num = 6;
  src_poly.vertexes[0].x = 1.0;
  src_poly.vertexes[0].y = 2.0;
  src_poly.radius = 3.0;

  ret = PolygonCopy(&des_poly, &src_poly);
  EXPECT_TRUE(is_polygon_same(&des_poly, &src_poly));
}

TEST(UpdatePolygonValue, test01) {
  /* Wrong input,input is a null pointer. */
  int ret;
  Polygon2D polygon;
  Pose2D center_pose;
  bool use_center_pose;
  bool radius_known;
  double radius;

  radius = 0.0;
  use_center_pose = false;
  radius_known = false;

  ret = UpdatePolygonValue(NULL, &center_pose, use_center_pose, radius_known,
                           radius);
  EXPECT_EQ(ret, -1);

  /* Illegal input data */
  ret = InitPolygon(&polygon);
  EXPECT_EQ(ret, 0);

  polygon.vertex_num = 13;
  ret = UpdatePolygonValue(&polygon, &center_pose, use_center_pose,
                           radius_known, radius);
  EXPECT_EQ(ret, -1);

  polygon.vertex_num = 0;
  ret = UpdatePolygonValue(&polygon, &center_pose, use_center_pose,
                           radius_known, radius);
  EXPECT_EQ(ret, -1);
}

TEST(UpdatePolygonValue, test02) {
  int ret;
  Polygon2D polygon;
  bool use_center_pose;
  bool radius_known;
  double radius;

  radius = 0.0;
  ret = InitPolygon(&polygon);
  EXPECT_EQ(ret, 0);

  /* Input rectangle coordinates */
  polygon.shape = PolygonShape::box;
  polygon.vertex_num = 4;
  polygon.vertexes[0].x = 3.0;
  polygon.vertexes[0].y = 0.0;
  polygon.vertexes[1].x = 3.0;
  polygon.vertexes[1].y = 2.0;
  polygon.vertexes[2].x = 0;
  polygon.vertexes[2].y = 2.0;
  polygon.vertexes[3].x = 0;
  polygon.vertexes[3].y = 0;

  /* Radius is unknown */
  use_center_pose = false;
  radius_known = false;
  ret =
      UpdatePolygonValue(&polygon, NULL, use_center_pose, radius_known, radius);
  EXPECT_EQ(ret, 1);

  Position2D center_point = {1.5, 1.0};
  printf("xy %.2f, %.2f \n", polygon.center_pt.x, polygon.center_pt.y);

  EXPECT_TRUE(is_point_same(polygon.center_pt, center_point));

  EXPECT_DOUBLE_EQ(polygon.radius, ifly_sqrt(3.25));

  /* Radius is known */
  radius_known = true;
  ret = UpdatePolygonValue(&polygon, NULL, use_center_pose, radius_known, 3);
  EXPECT_EQ(ret, 1);
  EXPECT_DOUBLE_EQ(polygon.radius, 3.0);
}

TEST(UpdatePolygonValue, test03) {
  int ret;
  Polygon2D polygon;
  Pose2D center_pose;

  center_pose.x = 1;
  center_pose.y = 1;
  center_pose.theta = 0;
  ret = InitPolygon(&polygon);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(static_cast<int>(polygon.shape),
            static_cast<int>(PolygonShape::multi_edge));

  /* Input polyhedron coordinates. */
  polygon.vertex_num = 5;
  polygon.vertexes[0].x = 3.0;
  polygon.vertexes[1].x = 3.0;
  polygon.vertexes[1].y = 2.0;
  polygon.vertexes[2].x = 1.5;
  polygon.vertexes[2].y = 3.0;
  polygon.vertexes[3].y = 2.0;

  /* There are center point and radius with information. */
  ret = UpdatePolygonValue(&polygon, &center_pose, true, true, 3);
  EXPECT_EQ(ret, 1);

  Position2D center_point = {1.0, 1.0};
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, 3);

  /* There is a center point with information without radius. */
  ret = UpdatePolygonValue(&polygon, &center_pose, true, false, 3);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, ifly_sqrt(5));

  /* There are radius and center point without information. */
  ret = UpdatePolygonValue(&polygon, &center_pose, false, true, 3);
  EXPECT_EQ(ret, 1);

  center_point = {3.0, 0.0};
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, 3);

  /* There is a center point without information and radius. */
  ret = UpdatePolygonValue(&polygon, NULL, true, false, 3);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, ifly_sqrt(13));
}

TEST(UpdatePolygonValue, test04) {
  /* corner cases: Point overlap and Abnormal polygon. */
  int ret;
  Polygon2D polygon;
  Pose2D center_pose;
  uint32_t i;

  /* Declare ret as the opposite of the expected result. */
  center_pose.x = 2;
  center_pose.y = 1;
  center_pose.theta = 0;
  ret = InitPolygon(&polygon);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(polygon.shape, PolygonShape::multi_edge);

  /* Input polyhedron information: 4 points overlap. */
  polygon.vertex_num = 4;
  polygon.shape = PolygonShape::box;
  for (i = 0; i < polygon.vertex_num; i++) {
    polygon.vertexes[i].x = 1.0;
    polygon.vertexes[i].y = 1.0;
  }
  ret = UpdatePolygonValue(&polygon, &center_pose, true, false, 3);
  EXPECT_EQ(ret, 1);

  Position2D center_point = {1.0, 1.0};
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, 0);

  /* Input polyhedron information: 3 points overlap. */
  polygon.vertex_num = 4;
  polygon.shape = PolygonShape::box;
  for (i = 0; i < polygon.vertex_num - 1; i++) {
    polygon.vertexes[i].x = 1.0;
    polygon.vertexes[i].y = 1.0;
  }
  polygon.vertexes[polygon.vertex_num - 1].x = 0.0;
  polygon.vertexes[polygon.vertex_num - 1].y = 0.0;

  ret = UpdatePolygonValue(&polygon, &center_pose, true, false, 3);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, 0.0);

  /* Input polyhedron information: 2 points overlap. */
  ret = InitPolygon(&polygon);
  polygon.shape = PolygonShape::box;
  polygon.vertex_num = 4;
  polygon.vertexes[0].x = 3.0;
  polygon.vertexes[1].x = 3.0;
  polygon.vertexes[1].y = 2.0;
  polygon.vertexes[2].x = 0.0;
  polygon.vertexes[2].y = 0.0;
  polygon.vertexes[3].x = 0.0;
  polygon.vertexes[3].y = 0.0;
  ret = UpdatePolygonValue(&polygon, &center_pose, true, false, 3);
  EXPECT_EQ(ret, 1);

  center_point = {1.5, 0.0};
  EXPECT_EQ(is_point_same(polygon.center_pt, center_point), true);

  EXPECT_DOUBLE_EQ(polygon.radius, 1.5);
}

TEST(GenerateRectPolygon, test01) {
  /* Wrong input,input is a null pointer. */
  int ret;
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;

  ret = GenerateRectPolygon(NULL, min_x, min_y, max_x, max_y);
  EXPECT_EQ(ret, 0);
}

TEST(GenerateRectPolygon, test02) {
  int ret;
  double min_x = 4.0;
  double min_y = 4.0;
  double max_x = 6.0;
  double max_y = 6.0;
  Polygon2D polygon;

  ret = InitPolygon(&polygon);
  EXPECT_EQ(ret, 0);

  ret = GenerateRectPolygon(&polygon, min_x, min_y, max_x, max_y);
  EXPECT_EQ(ret, 1);

  Position2D vertexes1 = {6.0, 6.0};
  EXPECT_EQ(is_point_same(polygon.vertexes[0], vertexes1), true);

  Position2D vertexes2 = {4.0, 6.0};
  EXPECT_EQ(is_point_same(polygon.vertexes[1], vertexes2), true);

  Position2D vertexes3 = {4.0, 4.0};
  EXPECT_EQ(is_point_same(polygon.vertexes[2], vertexes3), true);

  Position2D vertexes4 = {6.0, 4.0};
  EXPECT_EQ(is_point_same(polygon.vertexes[3], vertexes4), true);
}

TEST(test_local_global_poly, transform) {
  int ret;
  double min_x = -1.0;
  double min_y = -2.0;
  double max_x = 1.0;
  double max_y = 4.0;
  Polygon2D local_polygon;

  ret = InitPolygon(&local_polygon);
  EXPECT_EQ(ret, 0);

  // right-up coordinate
  ret = GenerateRectPolygon(&local_polygon, min_x, min_y, max_x, max_y);
  EXPECT_EQ(ret, 1);

  Polygon2D global_polygon;
  Pose2D global_pose;

  global_pose.x = 0;
  global_pose.y = -10;
  global_pose.theta = -M_PI_2;

  RULocalPolygonToGlobal(&global_polygon, &local_polygon, &global_pose);

  EXPECT_DOUBLE_EQ(global_polygon.vertexes[0].x, -1.0);
  EXPECT_DOUBLE_EQ(global_polygon.vertexes[0].y, -14.0);

  EXPECT_DOUBLE_EQ(global_polygon.vertexes[2].x, 1.0);
  EXPECT_DOUBLE_EQ(global_polygon.vertexes[2].y, -8.0);

  EXPECT_DOUBLE_EQ(global_polygon.vertexes[3].x, -1.0);
  EXPECT_DOUBLE_EQ(global_polygon.vertexes[3].y, -8.0);
}

TEST(test_local_global_poly, transform2) {
  int ret;
  double back = -2.0;
  double half = 1.0;
  double front = 4.0;
  Polygon2D local_polygon;

  ret = InitPolygon(&local_polygon);
  EXPECT_EQ(ret, 0);

  local_polygon.vertex_num = 4;
  local_polygon.shape = PolygonShape::box;

  // up-left coordinate
  // right up
  local_polygon.vertexes[0].x = front;
  local_polygon.vertexes[0].y = -half;

  // left up
  local_polygon.vertexes[1].x = front;
  local_polygon.vertexes[1].y = half;

  // left down
  local_polygon.vertexes[2].x = back;
  local_polygon.vertexes[2].y = half;

  // right down
  local_polygon.vertexes[3].x = back;
  local_polygon.vertexes[3].y = -half;

  UpdatePolygonValue(&local_polygon, nullptr, false, false, POLYGON_MAX_RADIUS);

  Polygon2D global_polygon;
  // east- north coordinate
  Pose2D global_pose;

  global_pose.x = 0;
  global_pose.y = -10;
  global_pose.theta = -M_PI_2;

  RULocalPolygonToGlobal(&global_polygon, &local_polygon, &global_pose);

  EXPECT_DOUBLE_EQ(global_polygon.vertexes[0].x, -1.0);
  EXPECT_DOUBLE_EQ(global_polygon.vertexes[0].y, -14.0);

  EXPECT_DOUBLE_EQ(global_polygon.vertexes[2].x, 1.0);
  EXPECT_DOUBLE_EQ(global_polygon.vertexes[2].y, -8.0);

  EXPECT_DOUBLE_EQ(global_polygon.vertexes[3].x, -1.0);
  EXPECT_DOUBLE_EQ(global_polygon.vertexes[3].y, -8.0);
}