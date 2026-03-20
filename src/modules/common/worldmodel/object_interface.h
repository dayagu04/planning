#pragma once

#include <string>
#include <vector>
#include "../common/common.h"
#include "object_status.h"
#include "perception_fusion_result.h"
// #include "object_type.h"

namespace planning {

struct MSDObjectWorldModelData {
  bool filtered;
  std::vector<MSDPoint3f> occlusion_area;
  std::string extra_json;
};

struct MSDObjectWorldModelResult {
  uint8_t available;
  MSDObjectWorldModelData object_world_model_data;
};

enum MSDIntentionType {
  MSD_INTENTION_TYPE_INVALID = -1,
  MSD_INTENTION_TYPE_FREE_MOVE = 0,
  MSD_INTENTION_TYPE_KEEP_LANE = 1,
  MSD_INTENTION_TYPE_LEFT_CHANGE_LANE_OUT = 2,
  MSD_INTENTION_TYPE_LEFT_CHANGE_LANE_IN = 3,
  MSD_INTENTION_TYPE_RIGHT_CHANGE_LANE_OUT = 4,
  MSD_INTENTION_TYPE_RIGHT_CHANGE_LANE_IN = 5,
  MSD_INTENTION_TYPE_GO_STRAIGHT = 11,
  MSD_INTENTION_TYPE_TURN_LEFT = 12,
  MSD_INTENTION_TYPE_TURN_RIGHT = 13,
  MSD_INTENTION_TYPE_U_TURN = 14,
};

struct MSDPredictionTrajectoryPoint {
  MSDPoint2f position;
  float yaw;
  float theta;
  float velocity;
  float confidence;

  MSDMatrix2f covariance_xy;
  float std_dev_yaw;
  float std_dev_velocity;

  MSDPoint2f relative_position;
  float relative_velocity;
  float relative_yaw;

  MSDPoint2f std_dev_relative_position;
  float std_dev_relative_yaw;
  float std_dev_relative_velocity;
};

struct MSDPredictionTrajectory {
  float confidence;
  float prediction_interval;

  MSDObjectStatus status;

  MSDIntentionType intention;

  std::vector<MSDPredictionTrajectoryPoint> trajectory_points;

  std::string extra_json;
};

struct MSDObjectPredictionData {
  std::vector<MSDPredictionTrajectory> trajectories;
  std::string extra_json;
};

struct MSDObjectPredictionResult {
  uint8_t available;
  MSDObjectPredictionData object_prediction_data;
};

struct MSDObjectFusionResult {
  uint8_t available;
  MSDPerceptionFusionObjectData object_fusion_data;
};

struct MSDObjectInterface {
  MSDObjectFusionResult object_fusion_result;
  MSDObjectPredictionResult object_prediction_result;
  MSDObjectWorldModelResult object_world_model_result;
  std::string extra_json;
};

struct MSDObjectsResult {
  uint8_t available;
  std::vector<MSDObjectInterface> object_interface;
};

struct MSDObjectsInterfaceMeta {
  uint64_t timestamp_us;
  std::string frame_id;
};

struct MSDObjectsInterface {
  MSDObjectsInterfaceMeta meta;
  MSDObjectsResult objects_result;
};

}  // namespace planning
