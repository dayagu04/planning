#include "hybrid_astar_context.h"

namespace planning {
namespace apa_planner {
const float PathCompareCost::GetTotalCost() {
  total_cost = length_cost + gear_change_cost + kappa_change_cost +
               unsuitable_last_line_length_cost + obs_dist_cost +
               cur_gear_switch_pose_cost + next_gear_switch_pose_cost;
  return total_cost;
}

const std::string GetSearchModeString(const SearchMode& search_mode) {
  switch (search_mode) {
    case SearchMode::FORMAL:
      return "FORMAL";
    case SearchMode::PRE_SEARCH:
      return "PRE_SEARCH";
    case SearchMode::DECIDE_CUL_DE_SAC:
      return "DECIDE_CUL_DE_SAC";
    default:
      return "UNKNOWN";
  }
}

}  // namespace apa_planner
}  // namespace planning
