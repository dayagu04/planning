#pragma once

#include <cmath>
#include <string>

namespace planning {

constexpr float kMathEpsilon32 = 1e-10;

/**
 * @class Vec2f
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec2f {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2f(const float x, const float y) noexcept : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  constexpr Vec2f() noexcept : Vec2f(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2f CreateUnitVec2f(const float angle);

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
  float DistanceTo(const Vec2f &other) const;

  //! Returns the squared distance to the given vector
  float DistanceSquareTo(const Vec2f &other) const;

  //! Returns the "cross" product between these two Vec2f (non-standard).
  float CrossProd(const Vec2f &other) const;

  //! Returns the inner product between these two Vec2f.
  float InnerProd(const Vec2f &other) const;

  //! rotate the vector by angle.
  Vec2f rotate(const float angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const float angle);

  //! Sums two Vec2f
  Vec2f operator+(const Vec2f &other) const;

  //! Subtracts two Vec2f
  Vec2f operator-(const Vec2f &other) const;

  //! Multiplies Vec2f by a scalar
  Vec2f operator*(const float ratio) const;

  //! Divides Vec2f by a scalar
  Vec2f operator/(const float ratio) const;

  //! Sums another Vec2f to the current one
  Vec2f &operator+=(const Vec2f &other);

  //! Subtracts another Vec2f to the current one
  Vec2f &operator-=(const Vec2f &other);

  //! Multiplies this Vec2f by a scalar
  Vec2f &operator*=(const float ratio);

  //! Divides this Vec2f by a scalar
  Vec2f &operator/=(const float ratio);

  //! Compares two Vec2f
  bool operator==(const Vec2f &other) const;

 protected:
  float x_ = 0.0;
  float y_ = 0.0;
};

//! Multiplies the given Vec2f by a given scalar
Vec2f operator*(const float ratio, const Vec2f &vec);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
float CrossProdf32(const Vec2f &start_point, const Vec2f &end_point_1,
                   const Vec2f &end_point_2);

}  // namespace planning
