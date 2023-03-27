#pragma once

#include <vector>

#include "common/common.h"
#include "object_interface.h"
#include "perception_fusion_result.h"

namespace planning {

enum MSDPerceptionObjectsAvailableFlag {
  MSD_PERCEPTION_OBJECTS_PERCEPTION_FUSION = 1 << 0,
  MSD_PERCEPTION_OBJECTS_PREDICTION = 1 << 1,
  MSD_PERCEPTION_OBJECTS_PERCEPTION_VISION = 1 << 2,
  MSD_PERCEPTION_OBJECTS_WORLDMODEL = 1 << 3
};

struct MSDPerceptionVisionObject {
  uint64_t track_id;
  MSDPoint2f relative_position;
  MSDPoint2f relative_velocity;
  MSDPolygon3f relative_polygon;
  float relative_theta;
  MSDObjectType object_type;
};

struct MSDPerceptionObjects {
  size_t available;
  MSDPerceptionFusionResult perception_fusion_result;
  MSDObjectsInterface prediction_result;
  MSDObjectsInterface world_model_result;
  std::vector<MSDPerceptionVisionObject> perception_vision_objects;
};
} // namespace planning
