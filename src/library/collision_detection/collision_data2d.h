#pragma once
#include "collision_request.h"
#include "collision_result2d.h"

namespace cdl {
struct CollisionData {
  CollisionData() : done(false) {}
  CollisionRequest request;
  CollisionResult result;

  /** Whether the collision iteration can stop */
  bool done;
};  // class CollisionData

}  // namespace cdl