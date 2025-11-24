#pragma once

namespace planning {

struct RuleBasedPredictorConfig {
  double predict_time = 4.0;

  // TODO: prediction traj need space continous.
  double time_resolution = 0.2;
};

}  // namespace planning