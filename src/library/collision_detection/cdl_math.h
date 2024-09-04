#pragma once
#include <iostream>

#include "types.h"

namespace cdl {
/** Returns the clockwise normal of a vector.(x,y) ---> (y,-x) */
inline Vector2r GetClockwiseNormal(const Vector2r &v) {
  return Vector2r(v[1], -v[0]);
}

/** Returns the counter-clockwise normal of a vector. (x,y) ---> (-y,x) */
inline Vector2r GetCounterClockwiseNormal(const Vector2r &v) {
  return Vector2r(-v[1], v[0]);
}

inline real GetVectorCross(const Vector2r &p1, const Vector2r &p2) {
  return p1[0] * p2[1] - p1[1] * p2[0];
}

inline real GetVectorDot(const Vector2r &a, const Vector2r &b) {
  return (a[0] * b[0] + a[1] * b[1]);
}

inline Vector2r GetMinVector(const Vector2r &a, const Vector2r &b) {
  return Vector2r(std::min(a[0], b[0]), std::min(a[1], b[1]));
}

inline void GetVectorMin(Vector2r *min, const Vector2r &a, const Vector2r &b) {
  (*min)[0] = std::min(a[0], b[0]);
  (*min)[1] = std::min(a[1], b[1]);
}

inline real GetMinFloat(const real a, const real b) {
  if (a < b) {
    return a;
  }
  return b;
}

inline Vector2r GetMaxVector(const Vector2r &a, const Vector2r &b) {
  return Vector2r(std::max(a[0], b[0]), std::max(a[1], b[1]));
}

inline void GetVectorMax(Vector2r *max_, const Vector2r &a, const Vector2r &b) {
  (*max_)[0] = std::max(a[0], b[0]);
  (*max_)[1] = std::max(a[1], b[1]);
}

inline real GetMaxFloat(const real a, const real b) {
  if (a < b) {
    return b;
  }
  return a;
}

}  // namespace cdl
