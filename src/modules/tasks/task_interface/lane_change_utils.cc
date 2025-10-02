#include "lane_change_utils.h"
#include <cmath>

#include "refline.h"

namespace planning {
namespace {
constexpr double kEps = 1e-6;
constexpr double kEgoReachBoundaryTime = 4.0;
constexpr std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
constexpr std::array<double, 3> fp{3.0, 8.0, 20.0};
constexpr std::array<double, 3> buffer{1.0, 3.0, 10.0};
constexpr std::array<double, 3> fp_for_large_car{6.0, 12.0, 30.0};
constexpr std::array<double, 3> buffer_for_large_car{3.0, 6, 20.0};
}  // namespace

double CalcGapObjSafeDistance(const double ego_v,
                              const double obj_v, const double obj_a,
                              bool is_large_car, bool is_front_car) {
  // 比较自车与障碍物车辆的4s内行驶的距离，检查在这个过程中是否会有碰撞风险。
  // rel_dis = distance_ego - distance_obj
  // rel_dis = -0.5 * obj_a * t * t + (ego_v - obj_v) * t;
  //根据二次函数的特性，obj_a、ego_v、obj_v的不同取值分别计算安全距离
  //(1)当obj_a > 0，ego_v < obj_v; obj_a < 0，ego_v > obj_v时，rel_dis的值单调变化，需考虑如果是减速情况，在4s范围内是否会出现倒车情况
  //(2)当obj_a > 0，ego_v > obj_v; obj_a < 0，ego_v < obj_v时, rel_dis的值非单调变化，在二者速度相等的地方，rel_dis达到极值点，需使用极值点的时间计算相对距离
  double rel_dis = 0;
  const double dec_time = std::abs(obj_v / obj_a);
  const double dec_time_v_equal = std::abs((ego_v - obj_v) / obj_a);
  double calculate_collision_time = kEgoReachBoundaryTime;
  if ((obj_a >= 0 && ego_v <= obj_v) || (obj_a <= 0 && ego_v >= obj_v)) {
    if (obj_a < kEps && dec_time < kEgoReachBoundaryTime) {
      calculate_collision_time = dec_time;
    }
  } else if ((obj_a > 0 && ego_v > obj_v && is_front_car) ||
             (obj_a < 0 && ego_v < obj_v && !is_front_car)) {
    double calculate_collision_time = kEgoReachBoundaryTime;
    if (dec_time_v_equal < kEgoReachBoundaryTime) {
      calculate_collision_time = dec_time_v_equal;
    }
  }
  rel_dis = -0.5 * obj_a * calculate_collision_time * calculate_collision_time +
            (ego_v - obj_v) * calculate_collision_time;

  const double gap_front_obj_safety_dist =
      is_large_car ? interp(ego_v, xp, fp_for_large_car)
                   : interp(ego_v, xp, fp);                
  const double buffer_dist = 
      is_large_car ? interp(ego_v, xp, buffer_for_large_car)
                   : interp(ego_v, xp, buffer);

  double dynamic_compensate_distance = 0.0;
  if (is_front_car) {
    dynamic_compensate_distance = rel_dis;
  } else {
    dynamic_compensate_distance = -rel_dis;
  }

  const double target_lane_need_safety_dist = std::max(
      dynamic_compensate_distance + buffer_dist, gap_front_obj_safety_dist);
  return target_lane_need_safety_dist;
}
}  // namespace planning