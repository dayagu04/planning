#pragma once

#include "common/math/polygon2d.h"
#include "apa_planner/common/geometry_planning_io.h"

namespace planning {

planning::planning_math::Polygon2d ConstructVehiclePolygon(
    const PlanningPoint &rear_center, const double half_width,
    const double front_edge_to_rear_center,
    const double rear_edge_to_rear_center,
    const double front_shrink_dis, const double front_side_shrink_dis,
    const double rear_shrink_dis, const double rear_side_shrink_dis);

} // namespace  planning