#pragma once

#include "common.h"
#include "object_source_data.h"
#include "object_status.h"
#include "object_type.h"
#include "object_type_info.h"

namespace planning {

enum MSDPerceptionFusionObjectsAvailableFlag {
  MSD_PERCEPTION_FUSION_OBJECTS_PERCEPTION_FUSION_OBJECTS_DATA = 1 << 0,
};

struct MSDPerceptionFusionObjectData {
  u_int64_t track_id;

  MSDObjectTypeInfo type_info;

  MSDObjectType type;

  MSDObjectStatus object_status;
  std::vector<MSDObjectSourceData> object_source_data;

  MSDShape3d shape;
  MSDShape3d shape_sigma;
  MSDPoint3d position;
  MSDPoint3d position_sigma;
  MSDPoint3d velocity;
  MSDPoint3d velocity_sigma;
  MSDPoint3d acceleration;
  MSDPoint3d acceleration_sigma;
  float heading_yaw;
  float heading_yaw_sigma;
  float angular_velocity;
  float angular_velocity_sigma;

  MSDPoint2d relative_position;
  MSDPoint2d relative_position_sigma;
  MSDPoint2d relative_velocity;
  MSDPoint2d relative_velocity_sigma;
  MSDPoint2d relative_acceleration;
  MSDPoint2d relative_acceleration_sigma;
  float relative_heading_yaw;
  float relative_heading_yaw_sigma;
  float relative_angular_velocity;
  float relative_angular_velocity_sigma;

  MSDPoint2d velocity_relative_to_ground;
  MSDPoint2d velocity_relative_to_ground_sigma;
  MSDPoint2d acceleration_relative_to_ground;
  MSDPoint2d acceleration_relative_to_ground_sigma;

  float score;

  MSDPolygon3f polygon_bottom;
  MSDPolygon3f polygon_top;

  std::string reserved_info;
};

struct MSDPerceptionFusionObjects {
  size_t available;
  std::vector<MSDPerceptionFusionObjectData> perception_fusion_objects_data;
};

struct MSDPerceptionFusionResultMeta {
  uint64_t timestamp_us;
  std::string version_tag;
};

struct MSDPerceptionFusionResult {
  MSDPerceptionFusionResultMeta meta;
  MSDPerceptionFusionObjects perception_fusion_objects;
};

}  // namespace planning
