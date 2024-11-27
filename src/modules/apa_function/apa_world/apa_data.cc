
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

void PrintApaStateMachine(const ApaStateMachine apa_state) {
  switch (apa_state) {
    case ApaStateMachine::ACTIVE_IN:
      ILOG_INFO << "apa_state = ACTIVE_IN";
      break;
    case ApaStateMachine::ACTIVE_WAIT_IN:
      ILOG_INFO << "apa_state = ACTIVE_WAIT_IN";
      break;
    case ApaStateMachine::ACTIVE_OUT:
      ILOG_INFO << "apa_state = ACTIVE_OUT";
      break;
    case ApaStateMachine::ACTIVE_WAIT_OUT:
      ILOG_INFO << "apa_state = ACTIVE_WAIT_OUT";
      break;
    case ApaStateMachine::SEARCH_IN:
      ILOG_INFO << "apa_state = SEARCH_IN";
      break;
    case ApaStateMachine::SEARCH_OUT:
      ILOG_INFO << "apa_state = SEARCH_OUT";
      break;
    case ApaStateMachine::SUSPEND:
      ILOG_INFO << "apa_state = SUSPEND";
      break;
    case ApaStateMachine::SECURE:
      ILOG_INFO << "apa_state = SECURE";
      break;
    case ApaStateMachine::COMPLETE:
      ILOG_INFO << "apa_state = COMPLETE";
      break;
    default:
      ILOG_INFO << "apa_state = INVALID";
      break;
  }
}

void PrintApaParkingOutDirection(
    const ApaParkingOutDirection apa_park_out_direction) {
  switch (apa_park_out_direction) {
    case ApaParkingOutDirection::RIGHT_FRONT:
      DEBUG_PRINT("apa_park_out_direction = RIGHT_FRONT");
      break;
    case ApaParkingOutDirection::RIGHT_REAR:
      DEBUG_PRINT("apa_park_out_direction = RIGHT_REAR");
      break;
    case ApaParkingOutDirection::LEFT_FRONT:
      DEBUG_PRINT("apa_park_out_direction = ACTILEFT_FRONTVE_OUT");
      break;
    case ApaParkingOutDirection::LEFT_REAR:
      DEBUG_PRINT("apa_park_out_direction = LEFT_REAR");
      break;
    case ApaParkingOutDirection::FRONT:
      DEBUG_PRINT("apa_park_out_direction = FRONT");
      break;
    case ApaParkingOutDirection::REAR:
      DEBUG_PRINT("apa_park_out_direction = REAR");
      break;
    case ApaParkingOutDirection::INVALID:
      break;
  }
}

const std::string GetApaStateMachine(const ApaStateMachine apa_state) {
  std::string state;
  switch (apa_state) {
    case ApaStateMachine::ACTIVE_IN:
      state = "ACTIVE_IN";
      break;
    case ApaStateMachine::ACTIVE_OUT:
      state = "ACTIVE_OUT";
      break;
    case ApaStateMachine::SEARCH_IN:
      state = "SEARCH_IN";
      break;
    case ApaStateMachine::SEARCH_OUT:
      state = "SEARCH_OUT";
      break;
    case ApaStateMachine::SUSPEND:
      state = "SUSPEND";
      break;
    case ApaStateMachine::SECURE:
      state = "SECURE";
      break;
    case ApaStateMachine::COMPLETE:
      state = "COMPLETE";
      break;
    default:
      state = "INVALID";
      break;
  }
  return state;
}
}  // namespace apa_planner
}  // namespace planning