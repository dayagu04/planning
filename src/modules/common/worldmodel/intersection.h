#pragma once
#include "../common/common.h"
#include "lane.h"
#include "perception_traffic_light.h"

namespace planning {

enum MSDIntersectionAvailableFlag {
  MSD_INTERSECTION_INTERSECTION_DATA = 1 << 0,
};

struct MSDMapTrafficLight {
  std::vector<MSDPoint3d> boundary;
  MSDMPATrafficLightPatternEnum pattern;
};

struct MSDMapTrafficLightGroup {
  uint64_t track_id;
  std::vector<MSDMapTrafficLight> traffic_lights;
};

struct MSDLeftWaitingArea {
  bool existence;
  double length;
  double distance;
};

struct MSDIntersectionData {
  MSDDirection direction;
  double length;
  std::vector<MSDMapTrafficLightGroup> traffic_light_groups;
  MSDLeftWaitingArea left_waiting_area;
  double distance_to_stop_line;
  std::vector<MSDLaneData> lanes_out;
};

struct MSDIntersection {
  size_t available;
  std::vector<MSDIntersectionData> intersections;
};
} // namespace planning
