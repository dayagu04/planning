#include <assert.h>

#include <cmath>

#include "vecf32.h"

namespace planning {

Vec2df32 Vec2df32::CreateUnitVec2df32(const float angle) {
  return Vec2df32(cos(angle), sin(angle));
}

float Vec2df32::Length() const { return std::hypot(x_, y_); }

float Vec2df32::LengthSquare() const { return x_ * x_ + y_ * y_; }

float Vec2df32::Angle() const { return std::atan2(y_, x_); }

void Vec2df32::Normalize() {
  const float l = Length();
  if (l > kMathEpsilon32) {
    x_ /= l;
    y_ /= l;
  }
}

float Vec2df32::DistanceTo(const Vec2df32 &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

float Vec2df32::DistanceSquareTo(const Vec2df32 &other) const {
  const float dx = x_ - other.x_;
  const float dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

float Vec2df32::CrossProd(const Vec2df32 &other) const {
  return x_ * other.y() - y_ * other.x();
}

float Vec2df32::InnerProd(const Vec2df32 &other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec2df32 Vec2df32::rotate(const float angle) const {
  return Vec2df32(x_ * cos(angle) - y_ * sin(angle),
                      x_ * sin(angle) + y_ * cos(angle));
}

void Vec2df32::SelfRotate(const float angle) {
  float tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2df32 Vec2df32::operator+(const Vec2df32 &other) const {
  return Vec2df32(x_ + other.x(), y_ + other.y());
}

Vec2df32 Vec2df32::operator-(const Vec2df32 &other) const {
  return Vec2df32(x_ - other.x(), y_ - other.y());
}

Vec2df32 Vec2df32::operator*(const float ratio) const {
  return Vec2df32(x_ * ratio, y_ * ratio);
}

Vec2df32 Vec2df32::operator/(const float ratio) const {
  assert(std::abs(ratio) > kMathEpsilon);
  return Vec2df32(x_ / ratio, y_ / ratio);
}

Vec2df32 &Vec2df32::operator+=(const Vec2df32 &other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2df32 &Vec2df32::operator-=(const Vec2df32 &other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2df32 &Vec2df32::operator*=(const float ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2df32 &Vec2df32::operator/=(const float ratio) {
  assert(std::abs(ratio) > kMathEpsilon32);
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2df32::operator==(const Vec2df32 &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon32 &&
          std::abs(y_ - other.y()) < kMathEpsilon32);
}

Vec2df32 operator*(const float ratio, const Vec2df32 &vec) {
  return vec * ratio;
}

float CrossProdf32(const Vec2df32 &start_point, const Vec2df32 &end_point_1,
                   const Vec2df32 &end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

}  // namespace planning
