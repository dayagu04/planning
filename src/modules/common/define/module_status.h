#pragma once
#include <cstdint>
#include <string>

namespace planning {

struct MSDModuleStatus {
  enum ModuleType {
    PREDICTION = 0,
    ROUTING = 1,
    PLANNING = 2,
    CONTROLLER = 3,
    ENDPOINT = 4,
    VEHICLE_REPORTER = 5,
    WORLDMODEL = 6,
    RADAR_PERCEPTION = 7,
    VISION_PERCEPTION = 8,
    LIDAR_PERCEPTION = 9,
    FUSION_PERCEPTION = 10,
    LOCATION = 11,
  };

  enum Status {
    STOP = 0,
    STARTING = 1,
    RUNNING = 2,
    STOPPING = 3,
    RUNNING_WARNING = 4,
    RUNNING_ERROR = 5,
  };

  enum DetailStatus {
    NONE = 0,
    RUNNING_UNKNOWN_ERROR = 1,
    RUNNING_OUT_OF_ROUTE = 2,
    RUNNING_OUT_OF_MAP = 3,
    RUNNING_INVALID_LANE_SCENE = 4,
    RUNNING_TAKING_OVER = 5,
    RUNNING_UNEXPECTED_ERROR = 6,
    RUNNING_IMAGE_BLACK = 7,
    RUNNING_IMAGE_HIGH_LATENCY = 8,
    RUNNING_IMAGE_LOST = 9,
    RUNNING_POINT_CLOUD_EMPTY = 10,
    RUNNING_LOCATION_LOW_ACCURACY = 11,
    RUNNING_MISSING_INPUTS = 12,
  };

  uint64_t timestamp_us;
  ModuleType module_type;
  Status status;
  DetailStatus detail_status;
  double latency_ms;
  std::string message;
};

}  // namespace planning
