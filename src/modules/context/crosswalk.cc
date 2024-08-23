#include "crosswalk.h"

namespace planning {

Crosswalk::Crosswalk(int32_t track_id) : track_id_(track_id) {}

int Crosswalk::track_id() const { return track_id_; }

void Crosswalk::set_id(const int32_t id) { track_id_ = id; }

double Crosswalk::width() const { return width_; }

void Crosswalk::set_width(const double width) { width_ = width; }

const std::vector<planning::planning_math::Vec2d>&
Crosswalk::center_line_points() const {
  return center_line_points_;
}

void Crosswalk::set_center_line_points(
    std::vector<planning::planning_math::Vec2d>&& center_line_points) {
  center_line_points_ = std::move(center_line_points);
}

const std::vector<planning::planning_math::Vec2d>& Crosswalk::polygon_points()
    const {
  return polygon_points_;
}

void Crosswalk::set_polygon_points(
    std::vector<planning::planning_math::Vec2d>&& polygon_points) {
  polygon_points_ = std::move(polygon_points);
}

}  // namespace planning
