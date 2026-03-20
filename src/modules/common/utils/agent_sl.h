#ifndef _AGENT_SL_H
#define _AGENT_SL_H
#include <cassert>
#include <cstddef>
#include "math/box2d.h"
#include "utils/kd_path.h"

namespace planning {

struct AgentSLInfo {
  double max_s;
  double min_s;
  double max_l;
  double max_l_s;
  double min_l;
  double min_l_s;
  double center_s;
  double center_l;

  AgentSLInfo()
      : max_s(-1000.0),
        min_s(1000.0),
        max_l(-1000.0),
        max_l_s(-1000.0),
        min_l(1000.0),
        min_l_s(1000.0),
        center_s(0.0),
        center_l(0.0){};

  void reset() {
    max_s = -1000.0;
    min_s = 1000.0;
    max_l = -1000.0;
    max_l_s = -1000.0;
    min_l = 1000.0;
    min_l_s = 1000.0;
    center_s = 0.0;
    center_l = 0.0;
  }
};

bool GetLaneAgentSLInfo(const std::shared_ptr<planning_math::KDPath>& path,
                        const planning::planning_math::Box2d& box,
                        AgentSLInfo* agent_sl_info);
}  // namespace planning

#endif