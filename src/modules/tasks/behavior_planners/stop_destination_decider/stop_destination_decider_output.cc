#include "stop_destination_decider_output.h"

namespace planning {
bool StopDestinationDeciderOutput::update_rads_bound_s_by_collision_check(const std::vector<double>& bound_s) {
  rads_bound_s_by_collision_check_.assign(bound_s.begin(), bound_s.end());
  return true;
};
}  // namespace planning
