#include <assert.h>

#include <cmath>

#include "vecf32.h"

namespace planning {

Vec2f Vec2f::CreateUnitVec(const float angle) {
  return Vec2f(cos(angle), sin(angle));
}

float Vec2f::Length() const { return std::hypot(x_, y_); }

float Vec2f::LengthSquare() const { return x_ * x_ + y_ * y_; }

float Vec2f::Angle() const { return std::atan2(y_, x_); }

void Vec2f::Normalize() {
  const float l = Length();
  if (l > kMathEpsilon32) {
    x_ /= l;
    y_ /= l;
  }
}

float Vec2f::DistanceTo(const Vec2f &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

float Vec2f::DistanceSquareTo(const Vec2f &other) const {
  const float dx = x_ - other.x_;
  const float dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

float Vec2f::CrossProd(const Vec2f &other) const {
  return x_ * other.y() - y_ * other.x();
}

float Vec2f::InnerProd(const Vec2f &other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec2f Vec2f::rotate(const float angle) const {
  return Vec2f(x_ * cos(angle) - y_ * sin(angle),
                      x_ * sin(angle) + y_ * cos(angle));
}

void Vec2f::SelfRotate(const float angle) {
  float tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2f Vec2f::operator+(const Vec2f &other) const {
  return Vec2f(x_ + other.x(), y_ + other.y());
}

Vec2f Vec2f::operator-(const Vec2f &other) const {
  return Vec2f(x_ - other.x(), y_ - other.y());
}

Vec2f Vec2f::operator*(const float ratio) const {
  return Vec2f(x_ * ratio, y_ * ratio);
}

Vec2f Vec2f::operator/(const float ratio) const {
  assert(std::abs(ratio) > kMathEpsilon32);
  return Vec2f(x_ / ratio, y_ / ratio);
}

Vec2f &Vec2f::operator+=(const Vec2f &other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2f &Vec2f::operator-=(const Vec2f &other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2f &Vec2f::operator*=(const float ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2f &Vec2f::operator/=(const float ratio) {
  assert(std::abs(ratio) > kMathEpsilon32);
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2f::operator==(const Vec2f &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon32 &&
          std::abs(y_ - other.y()) < kMathEpsilon32);
}

Vec2f operator*(const float ratio, const Vec2f &vec) {
  return vec * ratio;
}

float CrossProdf32(const Vec2f &start_point, const Vec2f &end_point_1,
                   const Vec2f &end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

}  // namespace planning
