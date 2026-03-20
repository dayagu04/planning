#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "lane.h"
#include "lane_merging_splitting_point.h"
#include "lane_stratery.h"
#include "map_poi_info.h"
#include "perception_fusion_lanes.h"
#include "perception_objects.h"
#include "scene_objects.h"
#include "self_position.h"
#include "target_position.h"
#include "worldmodel_extra.h"

namespace planning {

struct MSDWorldModelMeta {
  uint64_t timestamp_us;
  uint64_t self_state_timestamp_us;
  uint64_t perception_fusion_timestamp_us;
  uint64_t prediction_timestamp_us;
  uint64_t perception_traffic_light_timestamp_us;
  uint64_t map_planning_timestamp_us;
};

struct MSDWorldModel {
  MSDWorldModelMeta meta;
  MSDSceneObjects scene_objects;
  // MSDFreespace freespace; 暂时不用
  MSDTargetPosition target_position;
  MSDSelfPosition self_position;
  MSDLane lane;
  MSDPerceptionFusionLanes perception_fusion_lanes;
  // MSDMapPOIInfo map_poi_info;
  // MSDIntersection intersection;
  MSDLaneMergingSplittingPoint lane_merging_splitting_point;
  MSDPerceptionObjects perception_objects;
  // MSDMPATrafficLightPerception perception_traffic_light;
  MSDLaneStratery lane_stratery;
  MSDWorldModelExtra extra;
};
}  // namespace planning
