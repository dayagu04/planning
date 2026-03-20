#ifndef COMMON_LATERAL_MOTION_PLANNER_OUTPUT_
#define COMMON_LATERAL_MOTION_PLANNER_OUTPUT_

#include <array>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning {

struct InitStateInfo {
  double v{};
  double s{};
  double l{};
  double dl{};
  double ddl{};
};

struct StaticObstacleInfo {
  int id{};
  int priority{};
  int way_right{};

  bool is_trunk{};
  bool lon_ignore{};

  std::string side_pass{};
  std::string lon_decider{};

  double obstacle_type_buffer{};
};

struct DynamicObstacleInfo {
  int id{};
  int priority{};
  int way_right{};

  bool is_trunk{};
  bool is_cutin{};

  std::string side_pass{};
  std::string lon_decider{};
  std::string lat_decider_source{};

  double start_time{};
  double time_buffer{};
  double way_right_buffer{};
  double obstacle_type_buffer{};
};

struct ObstacleDebugInfo {
  std::vector<StaticObstacleInfo> staticObstacleInfo{};
  std::vector<DynamicObstacleInfo> dynamicObstacleInfo{};
};

struct LaneBorrowDebugInfo {
  bool lane_borrow{};
  int lane_borrow_range{};
  double lane_change_length{10000.0};
  double lane_borrow_width{10.0};
};

struct PathBoundsDeciderDebugInfo {
  std::vector<std::pair<double, double>>
      lateral_motion_offset;  // s, obs_weight
  std::vector<
      std::tuple<double, double, std::string, int, double, double, double>>
      lateral_l_constrain;  // s, constraint_l_left, type, id,
                            // constraint_start_buffer, constraint_end_buffer,
                            // constraint_lat_buffer;   map_id = -1
  std::vector<
      std::tuple<double, double, std::string, int, double, double, double>>
      lateral_r_constrain;  // s, constraint_l_right, type, id,
                            // constraint_start_buffer, constraint_end_buffer,
                            // constraint_lat_buffer;   map_id = -1
  std::vector<
      std::tuple<double, double, std::string, int, double, double, double>>
      lateral_l_offset;  // s, offset_l_left, type, id, offset_start_buffer,
                         // offset_end_buffer, offset_lat_buffer; map_id = -1
  std::vector<
      std::tuple<double, double, std::string, int, double, double, double>>
      lateral_r_offset;  // s, offset_l_right, type, id, offset_start_buffer,
                         // offset_end_buffer, offset_lat_buffer; map_id = -1
  std::vector<std::tuple<double, double, double, double>>
      obs_constrain_expand_lists;  // rel_t, front_expand, back_expand,
                                   // lat_expand
  std::vector<std::tuple<double, double, double, double>>
      obs_expecation_expand_lists;  // rel_t, front_expand, back_expand,
                                    // lat_expand
};

struct PiecewiseJerkPathOptimizerDebugInfo {
  std::string lat_plan_bound;
  PathBoundsDeciderDebugInfo path_bound_decider_debug_info{};
};

struct LateralMotionPlannerOutput {
  int scenario{};
  double offset{};
  bool is_lane_change{};
  int lat_plan_status{};
  bool default_valid{};
  std::vector<double> lat_jerk{};

  double premove_or_premoving{};
  std::string lc_request_source{};

  InitStateInfo initStateInfo{};
  LaneBorrowDebugInfo laneBorrowInfo{};
  ObstacleDebugInfo obstacleDebugInfo{};
  PiecewiseJerkPathOptimizerDebugInfo pwjDebugInfo{};
};

}  // namespace planning

#endif
