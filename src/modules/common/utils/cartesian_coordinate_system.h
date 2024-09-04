
#ifndef __CARTESIAN_COORDINATE_SYSTEM_H__
#define __CARTESIAN_COORDINATE_SYSTEM_H__

#include <cmath>

#include "define/geometry.h"
#include "pose2d.h"

const double PI = 3.1415926;

struct Segment2D {
  Point2D point[2];
};

double CrossProduct(const Segment2D& s1, const Segment2D& s2);
double DotProduct(const Segment2D& s1, const Segment2D& s2);

double Norm(const Segment2D& s1);

double PointsSquareDistance(const Point2D& p1, const Point2D& p2);

// Cartesian state of a point
struct CartesianState {
  double x;
  double y;
  double speed;
  double yaw;
  double curvature;
  double acceleration;
};

class CartesianCoordinateSystem {
 private:
  planning::Pose2D m_origin;

 public:
  CartesianCoordinateSystem(planning::Pose2D origin);
  Point2D CartCoord2CurCartCoord(const planning::Pose2D& prev_origin,
                                 const Point2D& prev_rel_coord);
  planning::Pose2D CartPose2CurCartPose(const planning::Pose2D& prev_origin,
                              const planning::Pose2D& prev_rel_pos);
  Point2D CurCartCoord2CartCoord(const planning::Pose2D& future_origin,
                                 const Point2D& cur_rel_coord);
  planning::Pose2D CurCartPose2CartPose(const planning::Pose2D& future_origin,
                              const planning::Pose2D& cur_rel_pos);
};
#endif
