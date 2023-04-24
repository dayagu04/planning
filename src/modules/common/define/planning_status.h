#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common/define/geometry.h"
// #include "modules/common/define/plan.h"
// #include "proto/generated_files/vehicle_status.pb.h"
#include "../res/include/proto/common.pb.h"
#include "../res/include/proto/planning_plan.pb.h"

namespace planning {
namespace common {

typedef struct {
  enum Status {
    CHANGE_LANE_PREPARATION = 0, // before change lane state
    IN_CHANGE_LANE = 1,          // during change lane state
    CHANGE_LANE_BACK = 2,        // lane change back
    CHANGE_LANE_FAILED = 3,      // change lane failed
    CHANGE_LANE_FINISHED = 4,    // change lane finished
  };
  Status status{CHANGE_LANE_PREPARATION};
  int path_id;
  bool is_active_lane_change{false};
  std::string direction{"none"};
  double start_timestamp;
  bool exist_lane_change_start_position{false};
  Point2D lane_change_start_position;
  double last_succeed_timestamp;
  bool is_current_opt_succeed{false};
  // obs to follow and obs to overtake in targetb lane
  // which together determine where the gap is
  std::pair<int, int> target_gap_obs;
  int origin_lane_leader_id;
  double left_dash_line_length;
  double lane_change_wait_time;
} ChangeLaneStatus;

typedef struct {
  enum Status {
    IN_BORROW_LANE = 1,       // during borrow lane state
    BORROW_LANE_KEEP = 2,     // stay in borrowed lane
    BORROW_LANE_FAILED = 3,   // borrow lane failed
    BORROW_LANE_FINISHED = 4, // borrow lane finished
  };
  Status status{IN_BORROW_LANE};
  int path_id;
  std::string direction;
  double start_timestamp;
  double last_succeed_timestamp;
  bool is_current_opt_succeed;
  double lat_offset{0.0};
} BorrowLaneStatus;

typedef struct {
  enum Status {
    LANE_KEEP = 1,   // during change lane state
    LANE_CHANGE = 2, // change lane failed
    LANE_BORROW = 3, // change lane finished
  };
  Status status{LANE_KEEP};
  ChangeLaneStatus change_lane;
  BorrowLaneStatus borrow_lane;
  int target_lane_id = 0;
  int target_lane_map_id;
  double target_lane_lat_offset;
} LaneStatus;

typedef struct {
  int id;
  double start_stopstamp;
} StopTime;

typedef struct {
  int crosswalk_id;
  std::unordered_map<int, double> stop_times;
  // std::vector<StopTime> stop_times;
} CrosswalkStatus;

typedef struct {
  double last_rerouting_time;
  bool need_rerouting;
  std::string routing_request;
} ReroutingStatus;

typedef struct {
  std::unordered_map<int, bool> junction;
} RightOfWayStatus;

typedef struct {
  std::string scenario_type;
  std::string stage_type;
} ScenarioStatus;

typedef struct {
  int front_blocking_obstacle_id;
} SidePassStatus;

typedef struct {
  std::string traffic_light_status;
} TrafficLightStatus;

typedef struct {
  std::unordered_map<int, int> broken_down_cars_map;
  std::vector<int> detected_broken_down_cars_vector;
} BrokenDownCarStatus;

typedef struct {
  double prebrake_acc;
  double prebrake_duration;
  double preacc_duration;
  bool enable_prebrake;
  bool enable_preacc;
  int n_prebrake_slip;
  int n_prebrake_slow;
  int n_preacc;
  double v_lim_curv;
  int n_prebrake_curv;
} PreActionResults;

struct AvdInfo
{
  int priority;
  int ignore_loop;
  bool lon_ignore;
  double time_buffer;
};

typedef struct {
  double timestamp;
  double next_timestamp;

  float lon_error = 0.0F;
  float lat_error = 0.0F;

  // decision
  std::vector<uint32_t> lon_follow_obstacles;
  std::vector<uint32_t> lon_overtake_obstacles;
  std::vector<uint32_t> lat_nudge_obstacles;
  std::unordered_map<int, AvdInfo> avd_info;
  std::unordered_map<int, int> yield_history;

  // planning_output
  PlanningOutput::PlanningOutput planning_output;

  std::vector<double> traj_vel_array;
  std::vector<double> traj_acceleration;

  //for control
  std::string extra_json;
} PlanningResult;

typedef enum {
  PRIMARY,
  SECONDARY,
  BACKUP,
} SchemeStage;

typedef struct {
  int64_t planning_loop = 0;
  bool planning_success = false;
  bool last_planning_success = false;
  double v_limit{40.0};
  double a_limit{1.0};
  double acc_smooth{5.0};
  double s_limit{400.0};
  // PreActionResults pre_action;
  LaneStatus lane_status;
  CrosswalkStatus crosswalk;
  ReroutingStatus rerouting;
  RightOfWayStatus right_of_way;
  ScenarioStatus scenario;
  SidePassStatus side_pass;
  TrafficLightStatus traffic_light;
  BrokenDownCarStatus broken_down_car;
  PlanningResult planning_result;
  PlanningResult pre_planning_result;
  SchemeStage scheme_stage;
  int backup_consecutive_loops = 0;
  std::string backup_reason = "none";
  double time_consumption = 0.0;
  std::string trigger_msg_id;
} PlanningStatus;


}  // namespace common
}  // namespace planning