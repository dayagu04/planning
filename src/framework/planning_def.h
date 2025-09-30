#ifndef ZNQC_DEFS_H
#define ZNQC_DEFS_H

#include <functional>
#include <string>

#include "macro.h"
#include "scene_type_config.pb.h"

namespace planning {
namespace framework {

enum FaultType {
  PERCEPTION_TIME_OUT_EXCEPTION = 0,
  PERCEPTION_COUNTER_EXCEPTION,
  LOCALIZATION_TIME_OUT_EXCEPTION,
  LOCALIZATION_COUNTER_EXCEPTION,
  PREDICTION_TIME_OUT_EXCEPTION,
  PREDICTION_COUNTER_EXCEPTION,
  VEHICLE_SERVICE_TIME_OUT_EXCEPTION,
  VEHICLE_SERVICE_COUNTER_EXCEPTION,
  FSM_TIME_OUT_EXCEPTION,
  FSM_COUNTER_EXCEPTION,
  TRAJ_LENGTH_EXCEPTION,
  TRAJ_CURVATURE_EXCEPTION,
  TRAJ_LON_POSITION_EXCEPTION,
  TRAJ_LON_VELOCITY_EXCEPTION,
  TRAJ_LON_ACC_EXCEPTION,
  TRAJ_LON_DECEL_EXCEPTION,
  TRAJ_LAT_POSITION_EXCEPTION,
  TRAJ_LAT_VELOCITY_EXCEPTION,
  TRAJ_LAT_ACC_EXCEPTION,
  TRAJ_YAW_EXCEPTION,
  TRAJ_ROLL_EXCEPTION,
  FAULT_TYPE_NUM
};

struct FaultCounter {
  int fault_trigger_counter;
  int fault_recovery_counter;
};

struct PlanningInitConfig {
  planning::common::SceneType scene_type{planning::common::SceneType::HIGHWAY};
  double cruise_velocity{120.0};
  double max_prediction_delay_time{1.0};
  int driving_style{1};
  std::string config_file_dir{""};
};

}  // namespace framework
}  // namespace planning

#endif  // ZNQC_DEFS_H
