#include "utils/general_lateral_decider_utils.h"

#include <algorithm>

#include "log.h"
#include "utils/kd_path.h"

namespace planning {
bool ConstructLinePolygons(const std::vector<planning_math::Vec2d> &line,
                           double y_width,
                           std::vector<planning_math::Polygon2d> &polygons) {
  for (size_t i = 0; i < line.size() - 1; i++) {
    auto x_start = line[i].x();
    auto y_start = line[i].y();
    auto x_end = line[i + 1].x();
    auto y_end = line[i + 1].y();

    auto y_edge = y_width;
    if (y_width > 0) {
      y_edge += fmax(y_start, y_end);
    } else if (y_width < 0) {
      y_edge += fmin(y_start, y_end);
    }

    std::vector<planning_math::Vec2d> points{
        planning_math::Vec2d{x_start, y_start},
        planning_math::Vec2d{x_end, y_end}, planning_math::Vec2d{x_end, y_edge},
        planning_math::Vec2d{x_start, y_edge}};

    planning_math::Polygon2d polygon;
    if (not planning_math::Polygon2d::ComputeConvexHull(points, &polygon)) {
      continue;
    }
    polygons.emplace_back(polygon);
  }

  return true;
}

bool OnLeftSide(const std::vector<planning_math::Vec2d> &vec2ds) {
  for (auto &vec2d : vec2ds) {
    if (vec2d.y() <= 0) {
      return false;
    }
  }
  return true;
}

bool Vec2dsToFrenet2ds(const std::shared_ptr<KDPath> &frenet_coord,
                       const std::vector<planning_math::Vec2d> &pts,
                       std::vector<planning_math::Vec2d> &frenet_pts) {
  const double kDistanceToRefThreshold = 1.0;
  if (pts.size() == 0) {
    LOG_DEBUG("vec2ds_to_frenet2ds empty input");
    return false;
  }
  Point2D carte_point, frenet_point;
  for (size_t i = 0; i < pts.size(); i++) {
    carte_point.x = pts[i].x();
    carte_point.y = pts[i].y();
    if (!frenet_coord->XYToSL(carte_point, frenet_point) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
      return false;
    }
    frenet_pts.emplace_back(
        planning_math::Vec2d{frenet_point.x, frenet_point.y});
  }

  // [hack]: just effective for groundline close to road center
  for (size_t i = 0; i < frenet_pts.size(); i++) {
    if (std::fabs(frenet_pts.at(i).y()) < kDistanceToRefThreshold) {
      return false;
    }
  }

  bool is_same_lat_direction = true;
  for (size_t i = 0; i + 1 < frenet_pts.size(); i++) {
    if (frenet_pts.at(i).y() * frenet_pts.at(i + 1).y() < 0) {
      is_same_lat_direction = false;
      break;
    }
  }
  if (!is_same_lat_direction) {
    LOG_DEBUG("stupid ground line obstacle!!!");
    for (size_t i = 0; i < frenet_pts.size(); i++) {
      LOG_DEBUG("Point number [%zu], x is %f, y is %f", i, frenet_pts.at(i).x(),
                frenet_pts.at(i).y());
    }
  }
  return is_same_lat_direction;
}

void MakeLinePolygons(
    const int obstacle_id, const std::shared_ptr<KDPath> &frenet_coord,
    const std::vector<planning_math::Vec2d> &points,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons) {
  std::vector<planning_math::Polygon2d> temp_polygons;
  std::vector<planning_math::Vec2d> frenet_points;
  frenet_points.reserve(points.size());
  bool trans_success = Vec2dsToFrenet2ds(frenet_coord, points, frenet_points);
  if (!trans_success) {
    return;
  }

  float offset = OnLeftSide(frenet_points) ? 1.0 : -1.0;
  ConstructLinePolygons(frenet_points, offset, temp_polygons);

  for (auto &polygon : temp_polygons) {
    if (offset > 0) {
      left_polygons.emplace_back(obstacle_id, polygon);
    } else {
      right_polygons.emplace_back(obstacle_id, polygon);
    }
  }
}

void MakePolygon(
    const int obstacle_id, const std::shared_ptr<KDPath> &frenet_coord,
    const planning_math::Polygon2d &polygon,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons) {
  const std::vector<planning_math::Vec2d> &points = polygon.points();
  std::vector<planning_math::Vec2d> frenet_points;
  frenet_points.reserve(points.size());
  bool trans_success = Vec2dsToFrenet2ds(frenet_coord, points, frenet_points);
  if (!trans_success) {
    return;
  }
  planning_math::Polygon2d tmp_polygon(frenet_points);
  if (OnLeftSide(frenet_points) > 0) {
    left_polygons.emplace_back(obstacle_id, tmp_polygon);
  } else {
    right_polygons.emplace_back(obstacle_id, tmp_polygon);
  }
}

}  // namespace planning
