#ifndef __APA_PARAM_SETTING_H__
#define __APA_PARAM_SETTING_H__

#include <cstdint>
#include <memory>

#ifndef apa_param
#define apa_param planning::apa_planner::ApaParametersSetting::GetInstance()
#endif

namespace planning {
namespace apa_planner {

struct ApaParameters {
  // park planner params
  double normal_slot_length = 5.2;
  double max_finish_lat_offset = 0.08;
  double max_finish_lon_offset = 0.2;
  double max_finish_heading_offset_deg = 2.8;
  double max_velocity = 0.6;
  double safe_uss_remain_dist = 0.35;
  // replan params
  double stuck_failed_time = 9.0;
  double stuck_replan_time = 4.0;
  double max_replan_remain_dist = 0.2;
  // T-lane expending or shrinking distance
  double vacant_p0_x_diff = 0.5;
  double vacant_p0_y_diff = 0.3;
  double vacant_p1_x_diff = 0.8;
  double vacant_p1_y_diff = 0.6;
  double occupied_p0_x_diff = 0.2;
  double occupied_p0_y_diff = 0.0;
  double occupied_p1_x_diff = 0.3;
  double occupied_p1_y_diff = 0.0;
  double nearby_slot_corner_dist = 0.6;
  double channel_width = 6.5;
  double terminal_target_x = 1.35;
  double terminal_target_y = 0.0;
  double terminal_target_y_bias = 0.0;
  double terminal_target_x_to_limiter = 0.15;
  double uss_stuck_replan_wait_time = 2.5;
  // path planner params
  double min_turn_radius = 5.5;
  double max_one_step_arc_radius = 8.5;
  double max_radius_in_slot = 12.66;
  double radius_eps = 0.01;
  double min_line_length = 0.3;
  double min_one_step_path_length = 0.6;
  double prepare_line_x_offset_slot = 7.2;
  double prepare_line_heading_offset_slot_deg = 8.8;
  double static_pos_eps = 0.01;
  // apa world params
  double max_standstill_speed = 0.01;
  double min_standstill_time_by_pos = 1.5;
  double min_standstill_time_by_vel = 0.5;
  // vehicle params
  double front_overhanging = 0.924;
  double rear_overhanging = 0.94;
  double wheel_base = 2.7;
  double vehicle_width = 1.89;
};

class ApaParametersSetting {
 public:
  static ApaParametersSetting &GetInstance() {
    static ApaParametersSetting instance;
    return instance;
  }

  // planning parameters
  const ApaParameters &GetParam() const { return param_; }
  ApaParameters &SetPram() { return param_; }

 private:
  ApaParametersSetting(){};

  // apa parameters
  ApaParameters param_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
