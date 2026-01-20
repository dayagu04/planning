#pragma once

#include <cstddef>

#include "apa_obstacle_manager.h"
#include "geometry_math.h"
#include "pose2d.h"
#include "speed/speed_data.h"
#include "trajectory/trajectory.h"

namespace planning {
namespace apa_planner {

// phase1: rule based;
// phase2: data based;
class RuleBasedPredictor {
 public:
  RuleBasedPredictor() = default;

  void Execute(std::shared_ptr<ApaObstacleManager>& obs_manager);

 private:
  void PredictByCV(ApaObstacle& obs);
  void PredictByCTRV(ApaObstacle& obs);

  void RecordDebugInfo(std::shared_ptr<ApaObstacleManager>& obs_manager);

  const double PredictApaObstacleOmega(const std::vector<Eigen::Vector2d>& history,
                                    double history_dt, double speed,
                                    ApaObstacle& obs);

  const double EstimateOmegaByCurvature(const std::vector<Eigen::Vector2d>& history,
                                  double history_dt, double speed);

 private:
};
}  // namespace apa_planner
}  // namespace planning