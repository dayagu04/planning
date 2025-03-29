#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include "pose2d.h"

namespace planning {

enum class AstarFailType {
  NONE,
  START_COLLISION,
  GOAL_COLLISION,
  OUT_OF_BOUND,
  TIME_OUT,
  SUCCESS,
  ALLOCATE_NODE_FAIL,
  SEARCH_TOO_MUCH_NODE,
  DP_COST_FAIL,
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
  REEDS_SHEPP_SAMPLING,
  ASTAR_SEARCHING,
  GEAR_REVERSE_SEARCHING,
  GEAR_DRIVE_SEARCHING,
  SPIRAL_SAMPLING,
  CUBIC_POLYNOMIAL_SAMPLING,
  QUNTIC_POLYNOMIAL_SAMPLING,
  // 点击车位之后，尝试搜索
  TRY_SEARCHING,
  MAX_NUMBER,
};

enum class ParkSpaceType {
  NONE,
  VERTICAL,
  PARALLEL,
  SLANTING,
  MAX_NUMBER,
};

enum class ParkingVehDirection {
  NONE,
  TAIL_IN,
  TAIL_OUT_TO_LEFT,
  TAIL_OUT_TO_RIGHT,
  TAIL_OUT_TO_MIDDLE,
  HEAD_IN,
  HEAD_OUT_TO_LEFT,
  HEAD_OUT_TO_RIGHT,
  HEAD_OUT_TO_MIDDLE,
  MAX_NUMBER,
};

enum class NodeCollisionType {
  NONE = 0,
  MAP_BOUND,
  SLOT_LINE,
  VIRTUAL_WALL,
  GROUND_LINE,
  FUSION_OCC_OBS,
  SLOT_LIMITER,
  MAX_NUM
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
  NONE = 0,
  LEFT = 1,
  RIGHT = 2,
  STRAIGHT = 3,
  MAX_NUM
};

enum class AstarPathGear {
  NONE = 0,
  DRIVE = 1,
  REVERSE,
  NORMAL,
  PARKING,
  MAX_NUM
};

enum class AstarNodeVisitedType {
  NOT_VISITED = 0,
  IN_OPEN = 1,
  IN_CLOSE,
  MAX_NUM
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
  float dist_request;

  AstarPathSteer steer_request;

  void Clear() {
    has_request = false;
    gear_request = AstarPathGear::NONE;
    dist_request = 0.0;

    return;
  }
};

struct MapBound {
  float x_min;
  float x_max;
  float y_min;
  float y_max;

  MapBound() = default;
  MapBound(const float x_min_, const float x_max_, const float y_min_,
           const float y_max_)
      : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_) {}
};

struct AStarPathPoint {
  float x;
  float y;
  float phi;

  AstarPathGear gear;
  float accumulated_s;
  AstarPathType type;
  // left turn is postive
  float kappa;

  AStarPathPoint() = default;
  AStarPathPoint(const float x_, const float y_, const float phi_,
                 const AstarPathGear gear_, const float s,
                 const AstarPathType type_, const float kappa_)
      : x(x_),
        y(y_),
        phi(phi_),
        gear(gear_),
        accumulated_s(s),
        type(type_),
        kappa(kappa_) {}
};

struct AStarSTPoint {
  float v;
  float acc;
  float jerk;
  float t;
  float s;
};

struct HybridAStarTrajPoint {
  AStarPathPoint path_point;
  AStarSTPoint speed_point;
};

// need refact this data
struct HybridAStarResult {
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> phi;
  std::vector<AstarPathGear> gear;
  std::vector<AstarPathType> type;

  std::vector<float> accumulated_s;
  // left turn is is positive
  std::vector<float> kappa;

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
    fail_type = AstarFailType::NONE;
    gear_change_num = 0;

    return;
  }
};

struct QueuePoint {
  size_t node_id;
  float f_cost;

  QueuePoint() = default;
  QueuePoint(const size_t id, const float f) : node_id(id), f_cost(f) {}
};

struct QueueCompare {
  bool operator()(const QueuePoint& left, const QueuePoint& right) const {
    return left.f_cost >= right.f_cost;
  }
};

#define astar_max_angle_number (16)
struct AstarSamplingAngle {
  size_t size;
  float angles[astar_max_angle_number];
  // left turn is positive
  float radius[astar_max_angle_number];
};

struct DebugAstarSearchPoint {
  Position2D pos;
  bool safe;
  // If this point is first path end point, and is a geat switch point without
  // collision, true.
  bool gear_switch_point;

  DebugAstarSearchPoint() = default;

  DebugAstarSearchPoint(const float x, const float y, const bool is_safe) {
    pos.x = x;
    pos.y = y;
    safe = is_safe;
    gear_switch_point = false;
  }

  DebugAstarSearchPoint(const float x, const float y,
                        const bool is_gear_switch_point, const bool is_safe) {
    pos.x = x;
    pos.y = y;
    gear_switch_point = is_gear_switch_point;
    safe = is_safe;
  }
};

enum class SlotRelativePosition { NONE, RIGHT, LEFT, MAX_NUMBER };

enum class VehRelativePosition { NONE, RIGHT, LEFT, MIDDLE, MAX_NUMBER };

struct Boundary2D {
  float min;
  float max;
};

std::string PathGearDebugString(const AstarPathGear gear);

std::string GetPathSteerDebugString(const AstarPathSteer type);

bool IsGearDifferent(const AstarPathGear left, const AstarPathGear right);

std::string PlanReasonDebugString(const PlanningReason reason);

}  // namespace planning