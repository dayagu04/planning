#pragma once

#include "ad_common/math/aabox2d.h"
#include "ad_common/math/vec2d.h"

namespace ad_common {
namespace hdmap {

template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const ad_common::math::AABox2d &aabox, const Object *object,
                  const GeoObject *geo_object, const uint64_t id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const ad_common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const ad_common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const ad_common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  uint64_t id() const { return id_; }

 private:
  ad_common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  uint64_t id_;
};

}  // namespace hdmap
}  // namespace ad_common