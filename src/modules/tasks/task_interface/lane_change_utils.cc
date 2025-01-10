#include "lane_change_utils.h"
#include "refline.h"

namespace planning {
namespace {
constexpr double kEgoReachBoundaryTime = 4.0;
constexpr std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
constexpr std::array<double, 3> fp{3.0, 8.0, 20.0};
constexpr std::array<double, 3> buffer{1.0, 3.0, 10.0};
constexpr std::array<double, 3> xp_for_large_car{6.0, 12.0, 30.0};
constexpr std::array<double, 3> buffer_for_large_car{3.0, 6, 20.0};
}  // namespace

double CalcGapObjSafeDistance(const double ego_v,
                              const double edge_distance_rel,
                              const double obj_v, const double obj_a,
                              bool is_large_car, bool is_front_car) {
  double safety_dist = ego_v * ego_v * 0.02 + 2.0;
  const double distance_obj =
      obj_v * kEgoReachBoundaryTime +
      0.5 * obj_a * kEgoReachBoundaryTime * kEgoReachBoundaryTime;
  const double distance_ego = ego_v * kEgoReachBoundaryTime;
  const double gap_front_obj_safety_dist =
      is_large_car ? interp(ego_v, xp_for_large_car, buffer_for_large_car)
                   : interp(ego_v, xp, fp);
  const double buffer_dist = interp(ego_v, xp, buffer);
  double dynamic_compensate_distance = 0.0;
  if (is_front_car) {
    dynamic_compensate_distance = distance_ego - distance_obj;
  } else {
    dynamic_compensate_distance = distance_obj - distance_ego;
  }

  const double target_lane_need_safety_dist = std::max(
      dynamic_compensate_distance + buffer_dist, gap_front_obj_safety_dist);
  return target_lane_need_safety_dist;
}
}  // namespace planning