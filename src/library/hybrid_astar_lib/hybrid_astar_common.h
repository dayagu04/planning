#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include "pose2d.h"

namespace planning {

enum class AstarFailType {
  none,
  start_collision,
  goal_collision,
  out_of_bound,
  time_out,
  success,
  allocate_node_fail,
  search_too_much_node,
  dp_cost_fail,
};

enum class SearchState {
  none,
  time_out,
  extend_node_too_much,
  success,
  overdue,
  failure,
  searching,
  search_state_max_number,
};

// use astar searching to get a path or just use rs path to link start and end.
enum class AstarPathGenerateType {
  none,
  reeds_shepp,
  astar_searching,
  max_number,
};

enum class ParkSpaceType {
  none,
  vertical,
  parallel,
  max_number,
};

enum class ParkingTask {
  none,
  parking_in,
  parking_out,
  max_number,
};

enum class ParkingVehDirectionRequest {
  none,
  tail_in_first,
  head_in_first,
  max_number,
};

enum class NodeCollisionType {
  none = 0,
  map_bound,
  slot_line,
  virtual_wall,
  ground_line,
  obs,
  max_num
};

enum class AstarPathType {
  none = 0,
  Reeds_Shepp,
  dubins,
  node_searching,
  point_interpolate,
  start_node,
  end_node,
  max_num
};

enum class PlanningReason {
  none,
  path_completed,
  path_stucked,
  slot_changed,
  first_plan,
  adjust_self_car_pose,
};

enum class AstarPathSteer {
  none = 0,
  left = 1,
  right = 2,
  straight = 3,
  max_num
};

enum class AstarPathGear {
  none = 0,
  drive = 1,
  reverse,
  normal,
  parking,
  max_num
};

enum class AstarNodeVisitedType {
  not_visited = 0,
  in_open = 1,
  in_close,
  max_num
};

// set first action in path
// 期望搜索节点的挡位。
// 1. node点按照期望的动作，有可能无法搜索出结果，所以不能将挡位限定死。
// 2. node点在该处依然可以不按照期望来，但是g cost会很大.
struct ParkFirstActionRequest {
  bool has_request;
  AstarPathGear gear_request;
  double dist_request;

  AstarPathSteer steer_request;
};

struct MapBound {
  double x_min;
  double x_max;
  double y_min;
  double y_max;

  MapBound() = default;
  MapBound(const double x_min_, const double x_max_, const double y_min_,
           const double y_max_)
      : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_) {}
};

struct AStarPathPoint {
  double x;
  double y;
  double phi;

  AstarPathGear gear;
  double accumulated_s;
  AstarPathType type;
  // left turn is postive
  double kappa;

  AStarPathPoint() = default;
  AStarPathPoint(const double x_, const double y_, const double phi_,
                 const AstarPathGear gear_, const double s,
                 const AstarPathType type_, const double kappa_)
      : x(x_),
        y(y_),
        phi(phi_),
        gear(gear_),
        accumulated_s(s),
        type(type_),
        kappa(kappa_) {}
};

struct HybridAStarSpeedPoint {
  double v;
  double acc;
  double t;
};

struct HybridAStarTrajPoint {
  AStarPathPoint path_point;
  HybridAStarSpeedPoint speed_point;
};

struct HybridAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<AstarPathGear> gear;
  std::vector<AstarPathType> type;

  // fill 0 for now
  std::vector<double> v;

  // not fill end point
  std::vector<double> a;

  // is positive
  std::vector<double> accumulated_s;
  // left turn is
  std::vector<double> kappa;

  int gear_change_num;
  bool is_nice_path;

  // slot pose
  Pose2D base_pose;

  double time_ms;

  AstarFailType fail_type;

  void Clear() {
    x.clear();
    y.clear();
    phi.clear();
    a.clear();
    accumulated_s.clear();
    v.clear();
    gear.clear();
    type.clear();
    kappa.clear();
    time_ms = 0;
    fail_type = AstarFailType::none;

    return;
  }
};

struct QueuePoint {
  size_t node_id;
  double f_cost;

  QueuePoint() = default;
  QueuePoint(const size_t id, const double f) : node_id(id), f_cost(f) {}
};

struct QueueCompare {
  bool operator()(const QueuePoint& left, const QueuePoint& right) const {
    return left.f_cost >= right.f_cost;
  }
};

enum CarSlotRelativePosition { none, car_is_right, car_is_left, car_is_middle };

#define astar_max_angle_number (16)
struct AstarSamplingAngle {
  size_t size;
  double angles[astar_max_angle_number];
  // left turn is positive
  double radius[astar_max_angle_number];
};

std::string PathGearDebugString(const AstarPathGear gear);

std::string GetPathSteerDebugString(const AstarPathSteer type);

}  // namespace planning