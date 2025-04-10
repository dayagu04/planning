#pragma once

#include <cmath>
#include <string>

namespace planning {

constexpr float kMathEpsilon32 = 1e-10;

/**
 * @class Vec2df32
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec2df32 {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2df32(const float x, const float y) noexcept : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  constexpr Vec2df32() noexcept : Vec2df32(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2df32 CreateUnitVec2df32(const float angle);

  //! Getter for x component
  float x() const { return x_; }

  //! Getter for y component
  float y() const { return y_; }

  //! Setter for x component
  void set_x(const float x) { x_ = x; }

  //! Setter for y component
  void set_y(const float y) { y_ = y; }

  //! Gets the length of the vector
  float Length() const;

  //! Gets the squared length of the vector
  float LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  float Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  float DistanceTo(const Vec2df32 &other) const;

  //! Returns the squared distance to the given vector
  float DistanceSquareTo(const Vec2df32 &other) const;

  //! Returns the "cross" product between these two Vec2df32 (non-standard).
  float CrossProd(const Vec2df32 &other) const;

  //! Returns the inner product between these two Vec2df32.
  float InnerProd(const Vec2df32 &other) const;

  //! rotate the vector by angle.
  Vec2df32 rotate(const float angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const float angle);

  //! Sums two Vec2df32
  Vec2df32 operator+(const Vec2df32 &other) const;

  //! Subtracts two Vec2df32
  Vec2df32 operator-(const Vec2df32 &other) const;

  //! Multiplies Vec2df32 by a scalar
  Vec2df32 operator*(const float ratio) const;

  //! Divides Vec2df32 by a scalar
  Vec2df32 operator/(const float ratio) const;

  //! Sums another Vec2df32 to the current one
  Vec2df32 &operator+=(const Vec2df32 &other);

  //! Subtracts another Vec2df32 to the current one
  Vec2df32 &operator-=(const Vec2df32 &other);

  //! Multiplies this Vec2df32 by a scalar
  Vec2df32 &operator*=(const float ratio);

  //! Divides this Vec2df32 by a scalar
  Vec2df32 &operator/=(const float ratio);

  //! Compares two Vec2df32
  bool operator==(const Vec2df32 &other) const;

 protected:
  float x_ = 0.0;
  float y_ = 0.0;
};

//! Multiplies the given Vec2df32 by a given scalar
Vec2df32 operator*(const float ratio, const Vec2df32 &vec);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
float CrossProdf32(const Vec2df32 &start_point, const Vec2df32 &end_point_1,
                   const Vec2df32 &end_point_2);

}  // namespace planning
