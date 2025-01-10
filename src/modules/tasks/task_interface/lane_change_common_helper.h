#ifndef LANE_CHANGE_COMMON_HELPER_H
#define LANE_CHANGE_COMMON_HELPER_H

#include <array>
namespace planning {

// 声明函数，外部文件可以通过这个声明来调用函数
double CalcGapObjSafeDistance(const double ego_v,
                              const double edge_distance_rel,
                              const double obj_v, const double obj_a,
                              bool is_large_car, bool is_front_car);
}  // namespace planning

#endif  // LANE_CHANGE_COMMON_HELPER_H