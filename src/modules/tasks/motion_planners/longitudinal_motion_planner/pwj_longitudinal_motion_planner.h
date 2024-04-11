#pragma once

#include <dlfcn.h>

#include "config/basic_type.h"
#include "tasks/task.h"

namespace planning {

struct LongitudinalSolverOption {
  bool enable_log{false};
  bool loose_obstacle_bound{false};
  bool use_raw_model_traj{false};
};

class LongitudinalOptimizerV3 : public Task {
 public:
  explicit LongitudinalOptimizerV3(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  virtual ~LongitudinalOptimizerV3() = default;

  bool Execute() override;

  bool optimize(const LongitudinalSolverOption &option);

  void interpolate_frenet_lon(const std::vector<TrajectoryPoint> &traj_points,
                              const std::vector<double> &s,
                              std::vector<double> &l,
                              std::vector<double> &heading_angle,
                              std::vector<double> &curvature);

 private:
  LongitudinalOptimizerV3Config config_;
  AdaptiveCruiseControlConfig config_acc_;
  StartStopEnableConfig config_start_stop_;
};

}  // namespace planning
