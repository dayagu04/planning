#pragma once

#include <cmath>
#include "log_glog.h"
#include "math_utils.h"

namespace planning {
struct Position2D {
  double x;
  double y;

  Position2D() = default;
  Position2D(const double x_, const double y_) : x(x_), y(y_) {}
};

struct Position3D {
  double x;
  double y;
  double z;

  Position3D() = default;
  Position3D(const double x_, const double y_, const double z_)
      : x(x_), y(y_), z(z_) {}
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  /* theta of head against east (unit: r) [-pi,pi)*/
  double theta = 0.0;

  Pose2D() = default;
  Pose2D(const double x_, const double y_, const double theta_)
      : x(x_), y(y_), theta(theta_) {}
  void SetPose(const double x_, const double y_, const double theta_) {
    x = x_;
    y = y_;
    theta = theta_;
  }

  const double GetX() const { return x; }
  const double GetY() const { return y; }
  const double GetPhi() const { return theta; }

  double DistanceTo(const Pose2D &p) const {
    return std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
  }

  double DistanceTo(const Pose2D *p) const {
    return std::sqrt((x - p->x) * (x - p->x) + (y - p->y) * (y - p->y));
  }

  double DistanceSquareTo(const Pose2D *p) const {
    return (x - p->x) * (x - p->x) + (y - p->y) * (y - p->y);
  }

  double DistanceToOrigin() const { return std::sqrt(x * x + y * y); }

  bool IsSame(const Pose2D &p) const {
    if (ifly_fequal(x, p.x) && ifly_fequal(y, p.y)) {
      return true;
    }

    return false;
  }

  bool IsSame(const Pose2D *p) const {
    if (ifly_fequal(x, p->x) && ifly_fequal(y, p->y)) {
      return true;
    }

    return false;
  }

  void DebugString() const {
    ILOG_INFO << "x = " << x << " ,y = " << y << " ,theta = " << theta;
    return;
  }

  double DistanceSquareTo(const Eigen::Vector2d &p) const {
    return (x - p[0]) * (x - p[0]) + (y - p[1]) * (y - p[1]);
  }
};

/* Calculate distance from two points */
template <typename T>
inline double CalcPointDist(T *p1, T *p2) {
  return std::sqrt((p1->x - p2->x) * (p1->x - p2->x) +
                   (p1->y - p2->y) * (p1->y - p2->y));
}

#define CalcPointOriginDist(p) std::sqrt((p)->x *(p)->x + (p)->y * (p)->y)

#define CalcPointOriginDistSq(p) ((p)->x * (p)->x + (p)->y * (p)->y)

/* Check if two points are same */
#define TwoPointsAreSame(pt1, pt2) \
  (ifly_fequal((pt1)->x, (pt2)->x) && (ifly_fequal((pt1)->y, (pt2)->y)))

/* Check if two points are same */
#define IsPointEqual(pt1, pt2) \
  (ifly_fequal((pt1)->x, (pt2)->x) && (ifly_fequal((pt1)->y, (pt2)->y)))

/* Unify theta to [-base, -base + 2*M_PI) */
double IflyUnifyTheta(double theta, double base);

/* Get difference of two theta, unify it to [-PI, PI) */
#define GetThetaDiff(theta1, theta2) IflyUnifyTheta((theta1) - (theta2), M_PI)

/* Compare two angles (radian) are equal (unified to [0, 2*PI) */
#define IsThetaEqual(theta1, theta2) \
  (ifly_fequal(IflyUnifyTheta((theta1) - (theta2), M_PI), 0.0))

#define IflyThetaEqualWithinThres(theta1, theta2, thres) \
  (!ifly_fgreater(std::fabs(GetThetaDiff((theta1), (theta2))), (thres)))

/*
 * Get theta orientation (angle from east) of a line
 * specified by (curr_pos, next_pos). clockwise is negative.
 * result range: (-PI, PI]
 */
#define CalcThetaByPosition(curr_pos, next_pos) \
  ifly_atan2((next_pos)->y - (curr_pos)->y, (next_pos)->x - (curr_pos)->x)

/*
 * Get theta orientation (angle from east) of a line
 * specified by (x1, y1, x2, y2). clockwise is negative.
 * result range: (-PI, PI]
 */
#define GetThetaByPoint(x1, y1, x2, y2) ifly_atan2((y2) - (y1), (x2) - (x1))

/*
 * Vector cross product of ab and ac
 */
#define CrossProductForVector(a, b) (((a)->x * (b)->y) - (a)->y * (b)->x)

/*
 * Vector cross product of a and b
 */
#define CrossProductForPoint(a, b, c)      \
  (((b)->x - (a)->x) * ((c)->y - (a)->y) - \
   ((c)->x - (a)->x) * ((b)->y - (a)->y))

void CvtPosGlobalToLocal(Position2D *local_pos, const Position2D *global_pos,
                         const Pose2D *base_pose);

void CvtPosLocalToGlobal(Position2D *global_pos, const Position2D *local_pos,
                         const Pose2D *base_pose);

/**
 * Position2D *local_pos: right-up coordinate
 */
void CvtPosLocalToGlobalFast(Position2D *global_pos,
                             const Position2D *local_pos,
                             const Pose2D *base_pose, const double sin_theta,
                             const double cos_theta);

void CvtThetaGlobalToLocal(double *local_theta, const double global_theta,
                           const double base_theta);

void CvtThetaLocalToGlobal(double *global_theta, const double local_theta,
                           const double base_theta);

void CvtPoseGlobalToLocal(Pose2D *local_pose, const Pose2D *global_pose,
                          const Pose2D *base_pose);

void IsLineSegmentIntersection(bool *is, const Position2D *p1,
                               const Position2D *p2, const Position2D *p3,
                               const Position2D *p4);

inline double GetDotProduct(const Pose2D &a, const Pose2D &b) {
  return a.x * b.x + a.y * b.y;
}

inline double HorizonProjectionLength(const Pose2D &base,
                                      const Pose2D &vector) {
  double base_length = std::sqrt(base.x * base.x + base.y * base.y);
  if (base_length < 1e-7) {
    return 0.0;
  }

  return (base.x * vector.x + base.y * vector.y) / base_length;
}

/*
 * Vector cross product
 */
const double inline CrossProduct(const Pose2D &base, const Pose2D &vector) {
  return base.x * vector.y - base.y * vector.x;
}

}  // namespace planning