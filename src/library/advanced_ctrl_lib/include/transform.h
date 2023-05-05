#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "Eigen/Geometry"

namespace pnc {
namespace transform {

// note that roation matrix is the rotation matrix from inertial frame to body
// frame, and transformation matrix from body frame to inertial frame
Eigen::AngleAxisd Quat2AxisAngle(Eigen::Quaterniond q);
Eigen::Matrix3d Quat2Rotm(Eigen::Quaterniond q);
Eigen::Vector3d Quat2EulerZYX(Eigen::Quaterniond q);

Eigen::Quaterniond Rotm2Quat(Eigen::Matrix3d &R);
Eigen::AngleAxisd Rotm2AxisAngle(const Eigen::Matrix3d &R);
Eigen::Vector3d Rotm2EulerZYX(Eigen::Matrix3d &R);

Eigen::Quaterniond EulerZYX2Quat(Eigen::Vector3d &euler_zyx);
Eigen::AngleAxisd EulerZYX2AxisAngle(Eigen::Vector3d &euler_zyx);
Eigen::Matrix3d EulerZYX2Rotm(Eigen::Vector3d &euler_zyx);

Eigen::Matrix2d Angle2Rotm2d(const double &angle);

Eigen::Vector2d Vector2dFrom3d(const Eigen::Vector3d v3d);
double GetAngleFromTwoVec(Eigen::Vector2d a, Eigen::Vector2d b);

} // namespace transform
} // namespace pnc

#endif
