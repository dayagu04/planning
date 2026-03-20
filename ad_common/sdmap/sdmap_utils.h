#pragma once

#include "ehr_sdmap.pb.h"
#include "math/aabox2d.h"
#include "math/aaboxkdtree2d.h"
#include "math/box2d.h"
#include "math/vec2d.h"

namespace ad_common {
namespace sdmap {
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

class RoadSegInfo {
 public:
  RoadSegInfo(const ::SdMapSwtx::Segment &seg) : seg_(seg) { Init(); }
  const ::SdMapSwtx::Segment &SegInfo() const { return seg_; }
  const std::vector<ad_common::math::LineSegment2d> &LineSegments() const {
    return segments_;
  }
  const std::vector<double> &AccumulatedS() const { return accumulated_s_; }
  const std::vector<double> &Headings() const { return headings_; }

 private:
  void Init();

 private:
  SdMapSwtx::Segment seg_;  // EHR原始输出信息
  std::vector<ad_common::math::LineSegment2d>
      segments_;                  // 道路内，形点组成的线段集合
  std::vector<double> headings_;  // 形点线段的角度值
  std::vector<double> accumulated_s_;  // 道路起点到各个形点的累积长度
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
}  // namespace sdmap
}  // namespace ad_common