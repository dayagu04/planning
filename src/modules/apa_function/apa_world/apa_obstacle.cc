#include "apa_obstacle.h"

namespace planning {
void ParkObstacle::Init() {
  acc_ = 0.0;
  height_bottom_ = 0.0;
  height_top_ = 0.0;
  semantic_type_ = ObjectType::NOT_KNOW;

  obs_slot_polygon_.vertex_num = 0;
  perception_polygon_.vertex_num = 0;
  is_static_ = true;
  is_virtual_ = true;
  perception_source_type_ = ParkObstacleType::FUSION_OBJECT_POINT_CLOUD;

  return;
}
}  // namespace planning