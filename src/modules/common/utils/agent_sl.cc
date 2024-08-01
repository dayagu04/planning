#include "agent_sl.h"

namespace planning {
bool GetLaneAgentSLInfo(const std::shared_ptr<planning_math::KDPath>& path,
                        const planning::planning_math::Box2d& box,
                        AgentSLInfo* agent_sl_info) {
  assert(path != nullptr);

  double s = 0.0;
  double l = 0.0;

  const auto& corners = box.GetAllCorners();
  if (!path->XYToSL(box.center_x(), box.center_y(), &s, &l)) {
    return false;
  }
  agent_sl_info->center_s = s;
  agent_sl_info->center_l = l;

  for (const auto& corner : corners) {
    if (!path->XYToSL(corner.x(), corner.y(), &s, &l)) {
      return false;
    }
    agent_sl_info->min_s = std::min(agent_sl_info->min_s, s);
    agent_sl_info->max_s = std::max(agent_sl_info->max_s, s);
    if (l < agent_sl_info->min_l) {
      agent_sl_info->min_l = l;
      agent_sl_info->min_l_s = s;
    }
    if (l > agent_sl_info->max_l) {
      agent_sl_info->max_l = l;
      agent_sl_info->max_l_s = s;
    }
  }
  return true;
}
}  // namespace planning