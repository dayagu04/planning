#ifndef COMMON_VISION_LONGITUDINAL_BEHAVIOR_PLANNER_OUTPUT_
#define COMMON_VISION_LONGITUDINAL_BEHAVIOR_PLANNER_OUTPUT_

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace planning {

struct CutinMsg {
  std::string cutin_dir;
  double a_limit_cutin;
  int track_id;
};

struct GapInfo {
  int rear_id;
  int front_id;
  int base_car_id;
  bool acc_valid; // acc valid if front car speed is much larger than rear car
                  // speed
  bool valid;
  double cost;
  double acc_time;

  double base_car_drel;
  double base_car_vrel;
};

struct TargetObstacle {
  int id;
  double d_rel;
  double start_s{0.0};
  double end_s{0.0};
  double v_rel;
};

struct VisionLongitudinalBehaviorPlannerOutput {
  double timestamp;

  // planning result
  double velocity_target = 40.0;
  double a_target_min = -0.5;
  double a_target_max = 0.5;

  CutinMsg cutin_msg{"none", 0.0, 1};

  // debug info
  double v_limit_in_turns = 40.0;
  std::array<double, 3> v_limit_cutin{};
  double v_target_lead_one = 40.0;
  double v_target_lead_two = 40.0;
  double v_target_temp_lead_one = 40.0;
  double v_target_temp_lead_two = 40.0;
  double v_target_terminus = 40.0;
  double v_target_ramp = 40.0;
  double v_target_cutin_front = 40.0;
  double v_target_pre_brake = 40.0;
  double v_target_merge = 40.0;
  double v_target_lane_change = 40.0;

  std::vector<double> vel_sequence{};

  std::pair<double, double> a_limit_in_turns{0.0, 0.0};
  std::array<double, 3> a_limit_cutin{};
  std::map<int, double> a_limit_cutin_history{};
  std::pair<double, double> a_target_lead{0.0, 0.0};
  std::pair<double, double> a_target_temp_lead_one{0.0, 0.0};
  std::pair<double, double> a_target_temp_lead_two{0.0, 0.0};
  double a_target_ramp = 0.0;
  double a_target_cutin_front = 0.0;
  double a_target_pre_brake = 0.0;
  double a_target_merge = 0.0;
  double a_target_lane_change = 0.0;
  double decel_base = 0.0;

  std::array<std::string, 3> cutin_condition{};

  std::array<int, 3> nearest_car_track_id{};
  std::vector<int> front_cut_in_track_id{};
  std::vector<int> prebrk_cut_in_track_id{};

  std::vector<TargetObstacle> merging_cars;
  std::vector<TargetObstacle> lane_changing_cars;

  std::vector<GapInfo> merging_available_gap;
  std::vector<GapInfo> lane_changing_available_gap;

  int merging_nearest_rear_car_track_id;
  int lane_changing_nearest_rear_car_track_id;

  std::string extra_json;
  std::string plan_msg;
};

} // namespace planning

#endif
