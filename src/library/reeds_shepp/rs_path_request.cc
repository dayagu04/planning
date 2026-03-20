#include "rs_path_request.h"

namespace planning {

std::string GetRSRequestType(const RSPathRequestType type) {
  switch (type) {
    case RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE:
      return "gear_switch_less_than_twice";
    case RSPathRequestType::ALL_PATH_FORBID_FORWARD:
      return "all_path_forbid_forward";
    case RSPathRequestType::ALL_PATH_FORBID_REVERSE:
      return "all_path_forbid_reverse";
    case RSPathRequestType::FIRST_PATH_FORBID_FORWARD:
      return "first_path_forbid_forward";
    case RSPathRequestType::FIRST_PATH_FORBID_REVERSE:
      return "first_path_forbid_reverse";
    case RSPathRequestType::FORBID_GEAR_CHANGE:
      return "forbid_gear_change";
    case RSPathRequestType::LAST_PATH_FORBID_FORWARD:
      return "last_path_forbid_forward";
    case RSPathRequestType::LAST_PATH_FORBID_REVERSE:
      return "last_path_forbid_reverse";
    default:
      break;
  }

  return "none";
}

}  // namespace planning