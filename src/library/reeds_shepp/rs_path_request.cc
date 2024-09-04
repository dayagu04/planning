#include "rs_path_request.h"

namespace planning {

std::string GetRSRequestType(const RSPathRequestType type) {
  switch (type) {
    case RSPathRequestType::gear_switch_less_than_twice:
      return "gear_switch_less_than_twice";
    case RSPathRequestType::all_path_forbid_forward:
      return "all_path_forbid_forward";
    case RSPathRequestType::all_path_forbid_reverse:
      return "all_path_forbid_reverse";
    case RSPathRequestType::first_path_forbid_forward:
      return "first_path_forbid_forward";
    case RSPathRequestType::first_path_forbid_reverse:
      return "first_path_forbid_reverse";
    case RSPathRequestType::forbid_gear_change:
      return "forbid_gear_change";
    case RSPathRequestType::last_path_forbid_forward:
      return "last_path_forbid_forward";
    case RSPathRequestType::last_path_forbid_reverse:
      return "last_path_forbid_reverse";
    default:
      break;
  }

  return "none";
}

}  // namespace planning