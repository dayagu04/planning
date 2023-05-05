#ifndef ZNQC_DEFS_H
#define ZNQC_DEFS_H

#include <functional>
#include <string>

#include "macro.h"
#include "planning_config.pb.h"

namespace planning {
namespace framework {

struct PlanningInitConfig {
  planning::common::SceneType scene_type{planning::common::SceneType::HIGHWAY};
  double cruise_velocity{120.0};
  double max_prediction_delay_time{1.0};
  int driving_style{1};
  std::string config_file_dir{""};
};

}  // namespace framework
}  // namespace planning

#endif  // ZNQC_DEFS_H
