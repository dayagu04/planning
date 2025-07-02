#include "pose2d.h"

namespace planning {

void CvtPosGlobalToLocal(Position2D *local_pos, const Position2D *global_pos,
                         const Pose2D *base_pose) {
  double dx, dy, theta;

  dx = global_pos->x - base_pose->x;
  dy = global_pos->y - base_pose->y;
  theta = base_pose->theta;

  local_pos->x = ifly_sin(theta) * dx - ifly_cos(theta) * dy;
  local_pos->y = ifly_cos(theta) * dx + ifly_sin(theta) * dy;

  return;
}

void CvtPosLocalToGlobal(Position2D *global_pos, const Position2D *local_pos,
                         const Pose2D *base_pose) {
  double lx, ly, theta;

  lx = local_pos->x;
  ly = local_pos->y;
  theta = base_pose->theta;

  global_pos->x = base_pose->x;
  global_pos->x += ifly_sin(theta) * lx + ifly_cos(theta) * ly;
  global_pos->y = base_pose->y;
  global_pos->y += ifly_sin(theta) * ly - ifly_cos(theta) * lx;

  return;
}

void CvtPosLocalToGlobalFast(Position2D *global_pos,
                             const Position2D *local_pos,
                             const Pose2D *base_pose, const double sin_theta,
                             const double cos_theta) {
  double lx, ly;

  lx = local_pos->x;
  ly = local_pos->y;

  global_pos->x = base_pose->x;
  global_pos->x += sin_theta * lx + cos_theta * ly;
  global_pos->y = base_pose->y;
  global_pos->y += sin_theta * ly - cos_theta * lx;

  return;
}

void CvtThetaGlobalToLocal(double *local_theta, const double global_theta,
                           const double base_theta) {
  *local_theta = IflyUnifyTheta(M_PI / 2.0 + global_theta - base_theta, 0.0);

  return;
}

void CvtThetaLocalToGlobal(double *global_theta, const double local_theta,
                           const double base_theta) {
  *global_theta = IflyUnifyTheta(local_theta + base_theta - M_PI / 2.0, 0.0);

  return;
}

void CvtPoseGlobalToLocal(Pose2D *local_pose, const Pose2D *global_pose,
                          const Pose2D *base_pose) {
  double dx, dy, theta;

  dx = global_pose->x - base_pose->x;
  dy = global_pose->y - base_pose->y;
  theta = base_pose->theta;

  local_pose->x = ifly_sin(theta) * dx - ifly_cos(theta) * dy;
  local_pose->y = ifly_cos(theta) * dx + ifly_sin(theta) * dy;

  local_pose->theta = M_PI / 2.0 + global_pose->theta - theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, 0.0);

  return;
}

void IsLineSegmentIntersection(bool *is, const Position2D *p1,
                               const Position2D *p2, const Position2D *p3,
                               const Position2D *p4) {
  Position2D p3p1, p3p4, p3p2, p1p3, p1p2, p1p4;
  double cross_result1, cross_result2;

  p3p1.x = p1->x - p3->x;
  p3p1.y = p1->y - p3->y;

  p3p4.x = p4->x - p3->x;
  p3p4.y = p4->y - p3->y;

  p3p2.x = p2->x - p3->x;
  p3p2.y = p2->y - p3->y;

  p1p3.x = -p3p1.x;
  p1p3.y = -p3p1.y;

  p1p2.x = p2->x - p1->x;
  p1p2.y = p2->y - p1->y;

  p1p4.x = p4->x - p1->x;
  p1p4.y = p4->y - p1->y;

  if (!ifly_fgreater(ifly_min(p1->x, p2->x), ifly_max(p3->x, p4->x)) &&
      !ifly_fgreater(ifly_min(p3->x, p4->x), ifly_max(p1->x, p2->x)) &&
      !ifly_fgreater(ifly_min(p1->y, p2->y), ifly_max(p3->y, p4->y)) &&
      !ifly_fgreater(ifly_min(p3->y, p4->y), ifly_max(p1->y, p2->y))) {
    cross_result1 = CrossProductForVector(&p3p1, &p3p4);
    cross_result1 *= CrossProductForVector(&p3p2, &p3p4);
    cross_result2 = CrossProductForVector(&p1p3, &p1p2);
    cross_result2 *= CrossProductForVector(&p1p4, &p1p2);

    if (!ifly_fgreater(cross_result1, 0) && !ifly_fgreater(cross_result2, 0)) {
      *is = true;
      return;
    }
  }

  *is = false;

  return;
}

}  // namespace planning
