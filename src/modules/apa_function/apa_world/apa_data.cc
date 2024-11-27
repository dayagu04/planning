
#include "apa_data.h"

#include <string>

#include "debug_info_log.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {
void PrintApaPlannerType(const ApaPlannerType planner_type) {
  switch (planner_type) {
    case ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER:
      ILOG_INFO << "planner_type = PERPENDICULAR_PARK_IN_PLANNER";
      break;
    case ApaPlannerType::PERPENDICULAR_PARK_HEADING_IN_PLANNER:
      ILOG_INFO << "planner_type = PERPENDICULAR_PARK_HEADING_IN_PLANNER";
      break;
    case ApaPlannerType::SLANT_PARK_IN_PLANNER:
      ILOG_INFO << "planner_type = SLANT_PARK_IN_PLANNER";
      break;
    case ApaPlannerType::PERPENDICULAR_PARK_OUT_PLANNER:
      ILOG_INFO << "planner_type = PERPENDICULAR_PARK_OUT_PLANNER";
      break;
    case ApaPlannerType::PARALLEL_PARK_IN_PLANNER:
      ILOG_INFO << "planner_type = PARALLEL_PARK_IN_PLANNER";
      break;
    case ApaPlannerType::PARALLER_PARK_OUT_PLANNER:
      ILOG_INFO << "planner_type = PARALLER_PARK_OUT_PLANNER";
      break;
    case ApaPlannerType::HYBRID_ASTAR_PLANNER:
      ILOG_INFO << "planner_type = HYBRID_ASTAR_PLANNER";
      break;
    default:
      ILOG_INFO << "planner_type = INVALID_planner";
      break;
  }
}

const std::string GetApaPlannerTypeString(const ApaPlannerType planner_type) {
  std::string type;
  switch (planner_type) {
    case ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER:
      type = "PERPENDICULAR_PARK_IN_PLANNER";
      break;
    case ApaPlannerType::PERPENDICULAR_PARK_HEADING_IN_PLANNER:
      type = "PERPENDICULAR_PARK_HEADING_IN_PLANNER";
      break;
    case ApaPlannerType::SLANT_PARK_IN_PLANNER:
      type = "SLANT_PARK_IN_PLANNER";
      break;
    case ApaPlannerType::PERPENDICULAR_PARK_OUT_PLANNER:
      type = "PERPENDICULAR_PARK_OUT_PLANNER";
      break;
    case ApaPlannerType::PARALLEL_PARK_IN_PLANNER:
      type = "PARALLEL_PARK_IN_PLANNER";
      break;
    case ApaPlannerType::PARALLER_PARK_OUT_PLANNER:
      type = "PARALLER_PARK_OUT_PLANNER";
      break;
    default:
      type = "INVALID_planner";
      break;
  }
  return type;
}
}  // namespace apa_planner
}  // namespace planning