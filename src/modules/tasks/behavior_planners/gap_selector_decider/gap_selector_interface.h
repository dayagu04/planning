/**
 * @file gap_selector.h
 **/

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "gap_selector.pb.h"
#include "obstacle.h"
#include "obstacle_manager.h"
#include "reference_path_manager.h"
#include "session.h"
#include "speed/st_point.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "virtual_lane_manager.h"
#define DEBUG_GAP_SELECTOR FALSE
namespace planning {
using namespace planning_math;
struct LonInfo {
  double v = 0.;
  double a = 0.;
  double j = 0.;
  double l = 0.;
  double heading = 0.;
};

struct Transform2D {
  double cos_theta;
  double sin_theta;
  double x0;
  double y0;
  Transform2D() : cos_theta(0.), sin_theta(1.), x0(0.), y0(0.){};

  Transform2D(double theta, double x, double y) {
    cos_theta = std::cos(theta);
    sin_theta = std::sin(theta);
    x0 = x;
    y0 = y;
  };

  void SetEgoState(double theta, double x, double y) {
    cos_theta = std::cos(theta);
    sin_theta = std::sin(theta);
    x0 = x;
    y0 = y;
  };

  void Global2Local(const Point2D &global_point, Point2D &local_point) {
    double tmp_x = global_point.x - x0;
    double tmp_y = global_point.y - y0;

    local_point.x = tmp_x * cos_theta + tmp_y * sin_theta;
    local_point.y = -tmp_x * sin_theta + tmp_y * cos_theta;
  };

  void Local2Gobal(const Point2D &local_point, Point2D &global_point) {
    double tmp_x = local_point.x * cos_theta - local_point.y * sin_theta;
    double tmp_y = local_point.x * sin_theta + local_point.y * cos_theta;
    global_point.x = tmp_x + x0;
    global_point.y = tmp_y + y0;
  };
};

struct Gap {
  int64_t front_agent_id{-1};
  int64_t rear_agent_id{-1};

  double begin_gap_length = 0.;
  double end_gap_length = 0.;
  double gap_front_s = 0.;
};

/*
------------------------------------------
1. 通过单例存储gap selector复现所需要的数据

2. 作为实例，通过pybind调用及调试
------------------------------------------
*/
struct GapSelectorFeedInfo {
  GapSelectorInfo gap_selector_info;

  std::array<double, 3> ego_init_s;
  std::array<double, 2> ego_cart_point;
  TrajectoryPoints traj_points;
  int target_state;  // 0:none ; 1:left ; 2:right
  std::unordered_map<int, Obstacle> map_gs_care_obstacles;
  std::vector<int> map_target_lane_obstacles;
  std::vector<int> map_origin_lane_obstacles;
  std::vector<int> lc_request_buffer{0, 0, 0};
  std::vector<double> ego_l_buffer{0., 0., 0.};

  std::vector<std::pair<double, double>> origin_refline_points;
  std::vector<std::pair<double, double>> target_refline_points;
  std::vector<std::pair<double, double>> current_refline_points;

  double ego_l_cur_lane{0.};
  int origin_lane_id{0};
  int target_lane_id{0};
  int current_lane_id{0};

  double cruise_vel{0.};
  PlanningInitPoint planning_init_point_;

  std::vector<std::pair<double, double>> origin_lane_s_width_vec;
  std::vector<std::pair<double, double>> target_lane_s_width_vec;
};

struct GapSelectorStateMachineInfo {
  bool lane_cross{false};
  bool lc_triggered{false};
  bool lb_triggered{false};
  bool lc_in{false};
  bool lb_in{false};
  double lc_pass_time{0.};
  double lc_wait_time{0.};
  double lc_premove_time{0.};
  bool path_requintic{false};
  bool lc_cancel{false};
  bool gs_skip{false};
  std::vector<int> lc_request_buffer{0, 0, 0};
  std::vector<double> ego_l_buffer{0., 0., 0.};
  int origin_lane_id{0};
  int current_lane_id{0};
  int target_lane_id{0};
};

enum GapSelectorStatus {
  DEFAULT = 0,
  NO_LC_REQUSET = 1,
  EXCEED_MAX_LC_TIME = 2,
  HIGH_LEVEL_RETREAT = 3,
  GAP_EMPTY = 4,
  FREE_LC = 5,
  COLLISION_CHECK_OK_PATH = 6,
  COLLISION_CHECK_FAILED_PATH = 7,
  GS_SKIP = 8,
  LC_CANCEL = 9,
  BASE_FRENET_COORD_NULLPTR = 10,
  LC_FINISHED = 11,
  LC_PASS_TIME_EXCEED_THRESHOLD = 12,
  LC_LATERAL_PREMOVE = 13,
};

class GapSelectorInterface {
 public:
  GapSelectorInterface(){};
  virtual ~GapSelectorInterface() = default;

  // Store Function
  void Store(
      const framework::Session *session,
      const GapSelectorStateMachineInfo &gap_selector_state_machine_info);
  void Store(const GapSelectorPathSpline &gap_selector_path_spline,
             const TrajectoryPoints &traj_points);

  // Parse
  GapSelectorFeedInfo gap_selector_feed_info_;
  void Parse(planning::common::GapSelectorInput &input);

  // Replay Info
  void ReplayCollect(
      const TrajectoryPoints &traj_points,
      const GapSelectorStateMachineInfo &gap_selector_state_machine_info,
      const GapSelectorPathSpline &gap_selector_path_spline,
      const std::vector<Gap> &gap_list, const Gap &nearby_gap,
      const std::vector<std::pair<STPoint, STPoint>>
          &front_gap_car_st_boundaries,
      const std::vector<std::pair<STPoint, STPoint>>
          &rear_gap_car_st_boundaries,
      const std::vector<std::pair<STPoint, STPoint>>
          &current_lane_front_car_st_boundaries,
      const int ego_current_lane_id,
      const std::vector<STPoint> &st_time_optimal,
      const GapSelectorStatus gap_status, const double front_car_dynamic_dis,
      const double rear_car_dynamic_dis, const double ego_lane_car_dynamic_dis);

  // Simulate Function
  GapSelectorStatus Simulate(planning::common::GapSelectorInput,
                             bool closed_loop);

 private:
};

}  // namespace planning