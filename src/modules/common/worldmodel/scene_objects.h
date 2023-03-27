#pragma once

#include <vector>
#include "../common/common.h"

namespace planning {

enum MSDSceneObjectsAvailableFlag {
  MSD_SCENE_OBJECTS_POLYGON_OBJECTS = 1 << 0,
};

enum MSDSceneObjectType {
  MSD_SCENE_OBJECT_TYPE_FIRE_EXIT_DOOR = 0,
  MSD_SCENE_OBJECT_TYPE_PILLAR = 1,
};

struct MSDScenePolygonObject {
  uint64_t object_id;
  MSDSceneObjectType object_type;
  MSDPolygon3f object_polygon;
};

struct MSDSceneObjects {
  size_t available;
  std::vector<MSDScenePolygonObject> polygon_objects;
};
} // namespace planning
