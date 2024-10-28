#include "virtual_obstacle_decider.h"

#include "log.h"
#include "task.h"

namespace planning {

VirtualObstacleDecider::VirtualObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "VirtualObstacleDecider";
}

bool VirtualObstacleDecider::Execute() {
  if (!PreCheck()) {
    LOG_ERROR("PreCheck failed\n");
    return false;
  }
  return true;
}

}  // namespace planning
