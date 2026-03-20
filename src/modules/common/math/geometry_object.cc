#include "geometry_object.h"

#include <iostream>
#include <memory>

#include "aabox2d.h"
#include "box2d.h"
#include "line_segment2d.h"
#include "polygon2d.h"
#include "vec2d.h"

namespace planning {
namespace planning_math {

GeometryObject::GeometryObject(const LineSegment2d& line_seg, const int index)
    : ptr_line_segment_(std::make_shared<LineSegment2d>(line_seg)),
      index_(index),
      type_(Type::LINE_SEGMENT) {}

GeometryObject::GeometryObject(const LineSegment2d& line_seg, const int index,
                               const int32_t track_id)
    : ptr_line_segment_(std::make_shared<LineSegment2d>(line_seg)),
      index_(index),
      type_(Type::LINE_SEGMENT) {}

GeometryObject::GeometryObject(const Box2d& box, const int index)
    : ptr_box_(std::make_shared<Box2d>(box)), index_(index), type_(Type::BOX) {}

GeometryObject::GeometryObject(const Box2d& box, const int index,
                               const int32_t track_id)
    : ptr_box_(std::make_shared<Box2d>(box)),
      index_(index),
      track_id_(track_id),
      type_(Type::BOX) {}

GeometryObject::GeometryObject(const Polygon2d& polygon, const int index,
                               const int32_t track_id)
    : ptr_polygon_(std::make_shared<Polygon2d>(polygon)),
      index_(index),
      track_id_(track_id),
      type_(Type::POLYGON) {}

GeometryObject::GeometryObject(const AABox2d& aabox, const int index,
                               const int32_t track_id)
    : ptr_aabox_(std::make_shared<AABox2d>(aabox)),
      index_(index),
      track_id_(track_id),
      type_(Type::AABB) {}

GeometryObject::GeometryObject(const Vec2d& point, const int index,
                               const int32_t track_id)
    : ptr_point_(std::make_shared<Vec2d>(point)),
      index_(index),
      track_id_(track_id),
      type_(Type::POINT) {}

const AABox2d& GeometryObject::aabox() const { return *ptr_aabox_; }

const LineSegment2d* GeometryObject::line_segment() const {
  return ptr_line_segment_.get();
}

int GeometryObject::index() const { return index_; }

int32_t GeometryObject::track_id() const { return track_id_; }

GeometryObject::Type GeometryObject::type() const { return type_; }

const Box2d* GeometryObject::box() const { return ptr_box_.get(); }

const Polygon2d* GeometryObject::polygon() const { return ptr_polygon_.get(); }

const Vec2d* GeometryObject::point() const { return ptr_point_.get(); }

double GeometryObject::max_x() const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->max_x();
    case Type::AABB:
      return ptr_aabox_->max_x();
    case Type::BOX:
      return ptr_box_->max_x();
    case Type::POLYGON:
      return ptr_polygon_->max_x();
    case Type::POINT:
      return ptr_point_->x();
    default:
      break;
  }
  return 0.0;
}
double GeometryObject::min_x() const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->min_x();
    case Type::AABB:
      return ptr_aabox_->min_x();
    case Type::BOX:
      return ptr_box_->min_x();
    case Type::POLYGON:
      return ptr_polygon_->min_x();
    case Type::POINT:
      return ptr_point_->x();
    default:
      break;
  }
  return 0.0;
}

double GeometryObject::max_y() const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->max_y();
    case Type::AABB:
      return ptr_aabox_->max_y();
    case Type::BOX:
      return ptr_box_->max_y();
    case Type::POLYGON:
      return ptr_polygon_->max_y();
    case Type::POINT:
      return ptr_point_->y();
    default:
      break;
  }
  return 0.0;
}
double GeometryObject::min_y() const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->min_y();
    case Type::AABB:
      return ptr_aabox_->min_y();
    case Type::BOX:
      return ptr_box_->min_y();
    case Type::POLYGON:
      return ptr_polygon_->min_y();
    case Type::POINT:
      return ptr_point_->y();
    default:
      break;
  }
  return 0.0;
}

bool GeometryObject::IsPointIn(const Vec2d& point) const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->IsPointIn(point);
    case Type::AABB:
      return ptr_aabox_->IsPointIn(point);
    case Type::BOX:
      return ptr_box_->IsPointIn(point);
    case Type::POLYGON:
      return ptr_polygon_->IsPointIn(point);
    case Type::POINT:
      return ptr_point_->DistanceSquareTo(point) < kMathEpsilon;
    default:
      break;
  }
  return false;
}

double GeometryObject::DistanceTo(const Vec2d& point) const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->DistanceTo(point);
    case Type::AABB:
      return ptr_aabox_->DistanceTo(point);
    case Type::BOX:
      return ptr_box_->DistanceTo(point);
    case Type::POLYGON:
      return ptr_polygon_->DistanceTo(point);
    case Type::POINT:
      return ptr_point_->DistanceTo(point);
    default:
      break;
  }
  return 0.0;
}

double GeometryObject::DistanceSquareTo(const Vec2d& point) const {
  switch (type_) {
    case Type::LINE_SEGMENT:
      return ptr_line_segment_->DistanceSquareTo(point);
    case Type::AABB:
      return ptr_aabox_->DistanceSquareTo(point);
    case Type::BOX:
      return ptr_box_->DistanceSquareTo(point);
    case Type::POLYGON:
      return ptr_polygon_->DistanceSquareTo(point);
    case Type::POINT:
      return ptr_point_->DistanceSquareTo(point);
    default:
      break;
  }
  return 0.0;
}

}  // namespace planning_math
}  // namespace planning
