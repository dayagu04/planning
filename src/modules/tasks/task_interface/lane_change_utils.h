#pragma once
#include <array>

namespace planning {

double CalcGapObjSafeDistance(const double ego_v,
                              const double edge_distance_rel,
                              const double obj_v, const double obj_a,
                              bool is_large_car, bool is_front_car);
}  // namespace planning
