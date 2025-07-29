#include "apa_context.h"

#include "log_glog.h"

namespace planning {
namespace apa_planner {
void PrintApaScenarioType(const ParkingScenarioType scenario_type) {
  ILOG_INFO << "scenario_type = " << GetApaScenarioTypeString(scenario_type);
}

const std::string GetApaScenarioTypeString(
    const ParkingScenarioType scenario_type) {
  std::string type;
  switch (scenario_type) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      type = "SCENARIO_PERPENDICULAR_TAIL_IN";
      break;
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_OUT:
      type = "SCENARIO_PERPENDICULAR_TAIL_OUT";
      break;
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN:
      type = "SCENARIO_PERPENDICULAR_HEAD_IN";
      break;
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT:
      type = "SCENARIO_PERPENDICULAR_HEAD_OUT";
      break;
    case ParkingScenarioType::SCENARIO_SLANT_TAIL_IN:
      type = "SCENARIO_SLANT_TAIL_IN";
      break;
    case ParkingScenarioType::SCENARIO_SLANT_TAIL_OUT:
      type = "SCENARIO_SLANT_TAIL_OUT";
      break;
    case ParkingScenarioType::SCENARIO_SLANT_HEAD_IN:
      type = "SCENARIO_SLANT_HEAD_IN";
      break;
    case ParkingScenarioType::SCENARIO_SLANT_HEAD_OUT:
      type = "SCENARIO_SLANT_HEAD_OUT";
      break;
    case ParkingScenarioType::SCENARIO_PARALLEL_IN:
      type = "SCENARIO_PARALLEL_IN";
      break;
    case ParkingScenarioType::SCENARIO_PARALLEL_OUT:
      type = "SCENARIO_PARALLEL_OUT";
      break;
    case ParkingScenarioType::SCENARIO_NARROW_SPACE:
      type = "SCENARIO_NARROW_SPACE";
      break;
    default:
      type = "SCENARIO_UNKNOWN";
      break;
  }
  return type;
}

void PrintApaScenarioStatus(const ParkingScenarioStatus scenario_status) {
  ILOG_INFO << "scenario_status = "
            << GetApaScenarioStatusString(scenario_status);
}

const std::string GetApaScenarioStatusString(
    const ParkingScenarioStatus scenario_status) {
  std::string status;
  switch (scenario_status) {
    case ParkingScenarioStatus::STATUS_TRY:
      status = "STATUS_TRY";
      break;
    case ParkingScenarioStatus::STATUS_RUNNING:
      status = "STATUS_RUNNING";
      break;
    case ParkingScenarioStatus::STATUS_DONE:
      status = "STATUS_DONE";
      break;
    case ParkingScenarioStatus::STATUS_FAIL:
      status = "STATUS_FAIL";
      break;
    default:
      status = "STATUS_UNKNOWN";
      break;
  }
  return status;
}

const std::string GetRePlanReasonString(const uint8_t type) {
  switch (type) {
    case 1:
      return "FIRST_PLAN";
    case 2:
      return "SEG_COMPLETED_PATH";
    case 3:
      return "SEG_COMPLETED_OBS";
    case 4:
      return "STUCKED";
    case 5:
      return "SLOT CHANGE";
    case 6:
      return "SEG_COMPLETED_COL_DET";
    case 7:
      return "FORCE_PLAN";
    case 8:
      return "SEG_COMPLETED_SLOT_JUMP";
    default:
      return "NOT_REPLAN";
  }

  return "NOT_REPLAN";
}

}  // namespace apa_planner
}  // namespace planning