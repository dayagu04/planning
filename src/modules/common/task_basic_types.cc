#include "task_basic_types.h"

namespace planning {

const std::string BoundType2String(BoundType in) {
  switch (in) {
    case BoundType::DEFAULT:
      return "DEFAULT";
    case BoundType::LANE:
      return "LANE";
    case BoundType::AGENT:
      return "AGENT";
    case BoundType::DYNAMIC_AGENT:
      return "DYNAMIC_AGENT";
    case BoundType::ADJACENT_AGENT:
      return "ADJACENT_AGENT";
    case BoundType::ROAD_BORDER:
      return "ROAD_BORDER";
    case BoundType::EGO_POSITION:
      return "EGO_POSITION";

    case BoundType::GROUNDLINE:
      return "GROUNDLINE";
    case BoundType::PARKING_SPACE:
      return "PARKING_SPACE";

    case BoundType::TRAFFIC_LIGHT:
      return "TRAFFIC_LIGHT";
    case BoundType::DESTINATION:
      return "DESTINATION";
    default:
      return "ERROR";
  }
}

}  // namespace planning
