#pragma once

#include <memory>
#include <variant>

#include "map_data.pb.h"
#include "math/aabox2d.h"
#include "math/aaboxkdtree2d.h"
#include "math/box2d.h"
#include "math/vec2d.h"

namespace ad_common {
namespace sdpromap {
using ad_common::math::Vec2d;
template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const ad_common::math::AABox2d &aabox, const Object *object,
                  const GeoObject &geo_object, const uint64_t id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const ad_common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const ad_common::math::Vec2d &point) const {
    return geo_object_.DistanceTo(point);
  }
  double DistanceSquareTo(const ad_common::math::Vec2d &point) const {
    return geo_object_.DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject geo_object() const { return geo_object_; }
  uint64_t id() const { return id_; }

 private:
  ad_common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject geo_object_;
  uint64_t id_;
};

enum MapInfoType { LINK_INFO = 0, LANE_INFO = 1, ROUTE_INFO = 2, MAP_NUM };

class MapInfoBase {
 public:
  MapInfoBase() = default;
  virtual ~MapInfoBase() {}
  virtual void BuildSegmentKdTree(
      const ad_common::math::AABoxKDTreeParams &params) = 0;
  virtual std::vector<std::pair<uint64_t, uint64_t>> GetObjects(
      const Vec2d &center, double &radius) = 0;
};

template <typename T>
class MyOptional {
  T value_;
  bool has_value_ = false;

 public:
  MyOptional() = default;
  MyOptional(const T &value) : value_(value), has_value_(true) {}
  bool has_value() const { return has_value_; }
  T value() const { return value_; }
};

}  // namespace sdpromap
}  // namespace ad_common
