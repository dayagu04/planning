#ifndef GENERAL_LATERAL_DECIDER_UTILS_H
#define GENERAL_LATERAL_DECIDER_UTILS_H
#include <cmath>
// #include <vector>

#include "config/basic_type.h"
#include "math/polygon2d.h"
#include "utils/kd_path.h"
// #include "vec2d.h"

namespace planning {

bool ConstructLinePolygons(const std::vector<planning_math::Vec2d> &line,
                           double y_width,
                           std::vector<planning_math::Polygon2d> &polygons);

bool OnLeftSide(const std::vector<planning_math::Vec2d> &vec2ds);
bool Vec2dsToFrenet2ds(
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const std::vector<planning_math::Vec2d> &pts,
    std::vector<planning_math::Vec2d> &frenet_pts);
void MakeLinePolygons(
    const int obstacle_id,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const std::vector<planning_math::Vec2d> &points,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons);
void MakePolygon(
    const int obstacle_id,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const planning_math::Polygon2d &polygon,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons);

}  // namespace planning

#endif
