#pragma once
#include <iostream>

#include "collision_geometry2d.h"
#include "gjk2d.h"
#include "collision_detect_types.h"

namespace cdl {
/**
 * \brief Center at zero point, axis aligned box
 */
class CDL_EXPORT Box : public CollisionGeometry {
 public:
  Box(const Vector2r *vertices);
  Box();
  Box(const Vector2r &side);
  Box(const real &x, const real &y);

  Vector2r vertices_[4];

  /** Compute local AABB */
  void computeLocalAABB() override;

  void updateBox(const Transform2r &tf);

  void setBox(const real &x, const real &y, const Transform2r &tf);

  void setBox(const Vector2r *vertices);

  /** Get node type: a box */
  NODE_TYPE getNodeType() const override;

  void getBoundVertices(cdl::ShapeProxy2D *proxy) const;

  /** get side length: width, hight */
  Vector2r getSide() const;

  /** get side length using side id: i, 0 is width, 1 is hight  */
  real side(const int32 &i);

};  // class Box
}  // namespace cdl
