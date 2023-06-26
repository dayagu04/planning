#include "transform.h"
#include "math_lib.h"

namespace pnc {
namespace transform {

Eigen::AngleAxisd Quat2AxisAngle(Eigen::Quaterniond q) {
  Eigen::AngleAxisd rotation_vector(q);
  return rotation_vector;
}

Eigen::Matrix3d Quat2Rotm(Eigen::Quaterniond q) { return q.toRotationMatrix(); }

// There is a bug for Eigen to transform quaternion to euler angle
Eigen::Vector3d Quat2EulerZYX(Eigen::Quaterniond q) {
  Eigen::Vector3d euler_zyx;
  euler_zyx << std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())),
      std::asin(mathlib::Limit(2.0 * (q.w() * q.y() - q.x() * q.z()),
                               1.0)),  // limit to avoid input of asin over 1
      std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  return euler_zyx;
}

Eigen::Quaterniond Rotm2Quat(Eigen::Matrix3d &R) {
  Eigen::Quaterniond quaternion(R);
  return quaternion;
}

Eigen::AngleAxisd Rotm2AxisAngle(const Eigen::Matrix3d &R) {
  Eigen::AngleAxisd rotation_vector;
  return rotation_vector.fromRotationMatrix(R);
}

Eigen::Vector3d Rotm2EulerZYX(Eigen::Matrix3d &R) {
  return Quat2EulerZYX(Rotm2Quat(R));
}

// There is a bug for Eigen to transform euler angle to quaternion, never
// EulerZYX2Quat by Eigen
Eigen::Quaterniond EulerZYX2Quat(Eigen::Vector3d &euler_zyx) {
  double c1 = std::cos(euler_zyx.x() / 2.0);
  double c2 = std::cos(euler_zyx.y() / 2.0);
  double c3 = std::cos(euler_zyx.z() / 2.0);
  double s1 = std::sin(euler_zyx.x() / 2.0);
  double s2 = std::sin(euler_zyx.y() / 2.0);
  double s3 = std::sin(euler_zyx.z() / 2.0);
  Eigen::Quaterniond q(c1 * c2 * c3 + s1 * s2 * s3, c1 * c2 * s3 - c3 * s1 * s2,
                       c1 * c3 * s2 + c2 * s1 * s3,
                       c2 * c3 * s1 - c1 * s2 * s3);
  return q;
}

Eigen::AngleAxisd EulerZYX2AxisAngle(Eigen::Vector3d &euler_zyx) {
  Eigen::Quaterniond q = EulerZYX2Quat(euler_zyx);
  return Quat2AxisAngle(q);
}

Eigen::Matrix3d EulerZYX2Rotm(Eigen::Vector3d &euler_zyx) {
  Eigen::Quaterniond q = EulerZYX2Quat(euler_zyx);
  return q.toRotationMatrix();
}

Eigen::Matrix2d Angle2Rotm2d(const double &angle) {
  Eigen::Vector3d euler_zyx(angle, 0.0, 0.0);
  Eigen::Matrix3d R = EulerZYX2Rotm(euler_zyx);
  return R.block(0, 0, 2, 2);
}

Eigen::Vector2d Vector2dFrom3d(const Eigen::Vector3d v3d) {
  Eigen::Vector2d v2d(v3d.x(), v3d.y());
  return v2d;
}

double GetAngleFromTwoVec(Eigen::Vector2d a, Eigen::Vector2d b) {
  if (a.norm() < 0.00001 || b.norm() < 0.00001) {
    return 0.0;
  } else {
    Eigen::Vector3d a_3d(a(0), a(1), 0.0);
    Eigen::Vector3d b_3d(b(0), b(1), 0.0);

    a_3d.normalize();
    b_3d.normalize();

    double cos_theta = a_3d.dot(b_3d);
    Eigen::Vector3d cross_a_b = a_3d.cross(b_3d);
    double sign = cross_a_b.z() >= 0.0 ? 1.0 : -1.0;
    double sin_theta = cross_a_b.norm() * sign;
    double theta = std::atan2(sin_theta, cos_theta);
    return std::fabs(theta);
  }
}

}  // namespace transform
}  // namespace pnc
