#pragma once

#include <cstddef>

#include "geometry_math.h"
#include "pose2d.h"
#include "speed/speed_data.h"
#include "trajectory/trajectory.h"
#include "apa_obstacle_manager.h"

namespace planning {
namespace apa_planner {

// phase1: rule based;
// phase2: data based;
class RuleBasedPredictor {
 public:
  RuleBasedPredictor() = default;

  void Execute(std::shared_ptr<ApaObstacleManager> obs_manager);

 private:
  void Predict(ApaObstacle& obs);

  void RecordDebugInfo(std::shared_ptr<ApaObstacleManager> obs_manager);

 private:
};
}  // namespace apa_planner
}  // namespace planning