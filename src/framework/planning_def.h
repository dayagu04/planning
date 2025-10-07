#ifndef ZNQC_DEFS_H
#define ZNQC_DEFS_H

#include <functional>
#include <string>

#include "macro.h"
#include "scene_type_config.pb.h"

namespace planning {
namespace framework {

enum class FaultType {
  PERCEPTION_TIME_OUT = 0,
  PERCEPTION_COUNTER,
  LOCALIZATION_TIME_OUT,
  LOCALIZATION_COUNTER,
  PREDICTION_TIME_OUT,
  PREDICTION_COUNTER,
  VEHICLE_SERVICE_TIME_OUT,
  VEHICLE_SERVICE_COUNTER,
  FSM_TIME_OUT,
  FSM_COUNTER,
  TRAJ_LENGTH_RANGE,
  TRAJ_LENGTH_CONTINUITY,
  TRAJ_CURVATURE_RANGE,
  TRAJ_CURVATURE_CONTINUITY,
  TRAJ_LON_POS_CONSISTENCY,
  TRAJ_LON_VEL_CONSISTENCY,
  TRAJ_LON_ACC_CONSISTENCY,
  TRAJ_LON_DEC_CONSISTENCY,
  TRAJ_LAT_POS_CONSISTENCY,
  TRAJ_LAT_VEL_CONSISTENCY,
  TRAJ_LAT_ACC_CONSISTENCY,
  TRAJ_YAW_CONSISTENCY,
  TRAJ_ROLL_CONSISTENCY,
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
