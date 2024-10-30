#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "aabox2d.h"
#include "box2d.h"
#include "line_segment2d.h"
#include "obstacle.h"
#include "polygon2d.h"
#include "vec2d.h"

namespace planning {
namespace planning_math {

class GeometryObject {
 public:
  enum class Type {
    UNKNOWN = 0,
    AABB = 1,
    BOX = 2,
    LINE_SEGMENT = 3,
    POLYGON = 4,
    POINT = 5,
    LANE = 6,
    BOUNDARY = 7,
  };
  GeometryObject(const LineSegment2d& line_seg, const int index);

  GeometryObject(const LineSegment2d& line_seg, const int index,
                 const int32_t track_id);

  GeometryObject(const Box2d& box, const int index);

  GeometryObject(const Box2d& box, const int index, const int32_t track_id);

  GeometryObject(const Box2d& box, const int index, const int32_t track_id,
                 const Obstacle* obstacle_ptr);

  GeometryObject(const Polygon2d& box, const int index, const int32_t track_id);

  GeometryObject(const AABox2d& aabox, const int index, const int32_t track_id);

  GeometryObject(const Vec2d& point, const int index, const int32_t track_id);

  ~GeometryObject() = default;

  double max_x() const;
  double min_x() const;
  double max_y() const;
  double min_y() const;

  bool IsPointIn(const Vec2d& point) const;

  double DistanceTo(const Vec2d& point) const;

  double DistanceSquareTo(const Vec2d& point) const;

  const AABox2d& aabox() const;

  const Box2d* box() const;

  const Polygon2d* polygon() const;

  const Vec2d* point() const;

  const LineSegment2d* line_segment() const;

  int index() const;

  int32_t track_id() const;

  Type type() const;

  const Obstacle* obstacle_ptr() const;

 private:
  // AABox2d aabox_;
  std::shared_ptr<LineSegment2d> ptr_line_segment_;
  std::shared_ptr<Box2d> ptr_box_;
  std::shared_ptr<Polygon2d> ptr_polygon_;
  std::shared_ptr<AABox2d> ptr_aabox_;
  std::shared_ptr<Vec2d> ptr_point_;

  int index_ = -1;
  int32_t track_id_ = -1;
  Type type_ = Type::UNKNOWN;
  const Obstacle* obstacle_ptr_ = nullptr;
};

}  // namespace planning_math
}  // namespace planning
