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

enum class AstarSearchState {
  NONE,
  TIME_OUT,
  EXTEND_NODE_TOO_MUCH,
  SUCCESS,
  OVERDUE,
  FAILURE,
  SEARCHING,
  SEARCH_STATE_MAX_NUMBER,
};

// use astar searching to get a path or just use rs path to link start and end.
// path type: node searching, rs, dubins, cubic polynomial curve, quntic
// polynomial curve, cubic spiral.
enum class AstarPathGenerateType {
  NONE,
  REEDS_SHEPP,
  ASTAR_SEARCHING,
  GEAR_REVERSE_DYNAMIC_PROGRAMMING,
  GEAR_DRIVE_DYNAMIC_PROGRAMMING,
  DUBINS_SAMPLING,
  SPIRAL_SAMPLING,
  CUBIC_POLYNOMIAL_SAMPLING,
  QUNTIC_POLYNOMIAL_SAMPLING,
  MAX_NUMBER,
};

enum class ParkSpaceType {
  NONE,
  VERTICAL,
  PARALLEL,
  SLANTING,
  MAX_NUMBER,
};

enum class ParkingTask {
  NONE,
  TAIL_PARKING_IN,
  HEAD_PARKING_IN,
  HEAD_PARKING_OUT,
  TAIL_PARKING_OUT,
  MAX_NUMBER,
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
  NONE = 0,
  REEDS_SHEPP,
  DUBINS,
  NODE_SEARCHING,
  LINE_SEGMENT,
  START_NODE,
  END_NODE,
  CUBIC_POLYNOMIAL,
  QUNTIC_POLYNOMIAL,
  SPIRAL,
  MAX_NUM
};

enum class PlanningReason {
  NONE,
  PATH_COMPLETED,
  PATH_STUCKED,
  SLOT_CHANGED,
  FIRST_PLAN,
  ADJUST_SELF_CAR_POSE,
  SIMULATION_TRIGGER,
  GEOMETRY_CURVE_FAIL,
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

enum class PolynomialPathErrorCode {
  NONE = 0,
  OUT_OF_X_BOUNDARY = 1,
  COLLISION = 2,
  BACK_TO_START_NODE = 3,
  BACK_TO_PARENT_NODE = 4,
  OUT_OF_HEADING_BOUNDARY = 5,
  UNEXPECTED_GEAR = 6,
  UNEXPECTED_STEERING_WHEEL = 7,
  UNEXPECTED_DRIVE_DIST = 8,
  FAIL_TO_ALLOCATE_NODE = 9,
  OUT_OF_Y_BOUNDARY,
  OUT_OF_KAPPA_BOUNDARY,
  LINK_POINT_INVALID_KAPPA,
  OUT_OF_SEARCH_BOUNDARY,
  MAX_NUMBER,
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

struct AStarSTPoint {
  double v;
  double acc;
  double jerk;
  double t;
  double s;
};

struct HybridAStarTrajPoint {
  AStarPathPoint path_point;
  AStarSTPoint speed_point;
};

// need refact this data
struct HybridAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<AstarPathGear> gear;
  std::vector<AstarPathType> type;

  std::vector<double> accumulated_s;
  // left turn is is positive
  std::vector<double> kappa;

  int gear_change_num;

  // slot pose
  Pose2D base_pose;

  double time_ms;

  AstarFailType fail_type;

  void Clear() {
    x.clear();
    y.clear();
    phi.clear();
    accumulated_s.clear();
    gear.clear();
    type.clear();
    kappa.clear();
    time_ms = 0;
    fail_type = AstarFailType::none;
    gear_change_num = 0;

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

#define astar_max_angle_number (16)
struct AstarSamplingAngle {
  size_t size;
  double angles[astar_max_angle_number];
  // left turn is positive
  double radius[astar_max_angle_number];
};

struct DebugAstarSearchPoint {
  Position2D pos;
  bool safe;

  DebugAstarSearchPoint() = default;

  DebugAstarSearchPoint(const double x, const double y, const bool is_safe) {
    pos.x = x;
    pos.y = y;
    safe = is_safe;
  }
};

enum class SlotRelativePosition {
  NONE,
  RIGHT,
  LEFT,
  MAX_NUMBER
};

enum class VehRelativePosition {
  NONE,
  RIGHT,
  LEFT,
  MIDDLE,
  MAX_NUMBER
};

struct Boundary2D {
  double min;
  double max;
};

struct PolynomialPathCost {
  double offset_to_center;
  double accumulated_s;
  double tail_heading;

  size_t point_size;

  void Clear() {
    offset_to_center = 100.0;
    tail_heading = 100.0;
    point_size = 0;
    return;
  }
};

std::string PathGearDebugString(const AstarPathGear gear);

std::string GetPathSteerDebugString(const AstarPathSteer type);

bool IsGearDifferent(const AstarPathGear left, const AstarPathGear right);

std::string PlanReasonDebugString(const PlanningReason reason);

const bool PolynomialPathBetter(const PolynomialPathCost& path,
                                const PolynomialPathCost& base);

}  // namespace planning