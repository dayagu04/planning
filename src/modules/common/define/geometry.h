#ifndef __UTILS__GEOMETRY_H__
#define __UTILS__GEOMETRY_H__

struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double xx, double yy) : x(xx), y(yy) {}
};

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Point3D() = default;
  Point3D(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;

  Quaternion() = default;
  Quaternion(double xx, double yy, double zz, double ww)
      : x(xx), y(yy), z(zz), w(ww) {}
};

struct Pose {
  Point3D position{};
  Quaternion orientation{};
};

#endif
