#pragma once
#include <cstdint>
#include <string>

namespace planning {
namespace apa_planner {

enum class ParkingScenarioType : uint8_t {
  SCENARIO_UNKNOWN = 0,
  SCENARIO_PERPENDICULAR_TAIL_IN = 1,
  SCENARIO_PERPENDICULAR_HEAD_IN = 2,
  SCENARIO_PERPENDICULAR_TAIL_OUT = 3,
  SCENARIO_PERPENDICULAR_HEAD_OUT = 4,
  SCENARIO_PARALLEL_IN = 5,
  SCENARIO_PARALLEL_OUT = 6,
  SCENARIO_SLANT_TAIL_IN = 7,
  SCENARIO_SLANT_HEAD_IN = 8,
  SCENARIO_SLANT_TAIL_OUT = 9,
  SCENARIO_SLANT_HEAD_OUT = 10,
  // todo, remove narrow space scenario
  SCENARIO_NARROW_SPACE = 11,
};

void PrintApaScenarioType(const ParkingScenarioType scenario_type);

const std::string GetApaScenarioTypeString(
    const ParkingScenarioType scenario_type);

enum class TaskType : uint8_t {
  CENTER_LINE_DECIDER,
  SLOT_RELEASE_DECIDER,
  TARGET_POSE_DECIDER,
  VIRTUAL_WALL_DECIDER,
  SLOT_MANAGER,
  OBSTACLE_MANAGER,
  PATH_OPTIMIZER,
  SPEED_LIMIT_DECIDER,
  STOP_DECIDER,
  SPEED_OPTIMIZER,
  ASTAR_PATH_GENERATOR,
  CURVE_BASED_PATH_GENERATOR,
};

enum class ParkingScenarioStatus {
  STATUS_UNKNOWN = 0,
  // 表示点击泊车，这个场景正在运行
  STATUS_RUNNING = 1,
  STATUS_DONE = 2,
  // 表示点击车位，尝试计算这个场景
  STATUS_TRY = 3,
  STATUS_FAIL = 4,
};

enum PathPlannerResult {
  PLAN_FAILED,  // path plan failed
  PLAN_HOLD,    // follow last
  PLAN_UPDATE,
  WAIT_PATH,
};

enum class ProcessObsMethod : uint8_t {
  DO_NOTHING = 0,
  MOVE_OBS_OUT_SLOT = 1,
  MOVE_OBS_OUT_CAR_SAFE_POS = 2,
  COUNT,
};

enum class RealTimeBrakeType : uint8_t {
  STOP,
  HEAVY_BRAKE,
  MODERATE_BRAKE,
  SLIGHT_BRAKE,
  COUNT,
};

struct RealTimeBrakeInfo {
  RealTimeBrakeType brake_type = RealTimeBrakeType::STOP;
  double body_lat_buffer = 0.0;
  double mirror_lat_buffer = 0.0;
  double min_lon_dist = 0.0;

  RealTimeBrakeInfo() = default;
  RealTimeBrakeInfo(const RealTimeBrakeType _brake_type,
                    const double _body_lat_buffer,
                    const double _mirror_lat_buffer, const double _min_lon_dist)
      : brake_type(_brake_type),
        body_lat_buffer(_body_lat_buffer),
        mirror_lat_buffer(_mirror_lat_buffer),
        min_lon_dist(_min_lon_dist) {}
  void Set(const RealTimeBrakeType _brake_type, const double _body_lat_buffer,
           const double _mirror_lat_buffer, const double _min_lon_dist) {
    this->brake_type = _brake_type;
    this->body_lat_buffer = _body_lat_buffer;
    this->mirror_lat_buffer = _mirror_lat_buffer;
    this->min_lon_dist = _min_lon_dist;
  }
  ~RealTimeBrakeInfo() = default;
};

enum class TaskExcuteState {
  NONE = 0,
  FAIL = 1,
  TIME_OUT = 2,
  MAX_ITERATION = 3,
  SUCCESS = 4,
};

void PrintApaScenarioStatus(const ParkingScenarioStatus scenario_status);

const std::string GetApaScenarioStatusString(
    const ParkingScenarioStatus scenario_status);

const std::string GetRePlanReasonString(const uint8_t type);

}  // namespace apa_planner
}  // namespace planning