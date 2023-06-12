#pragma once

#include <dlfcn.h>

#include "config/basic_type.h"
#include "task.h"

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
      const std::shared_ptr<TaskPipelineContext>
          &pipeline_context);

  virtual ~LongitudinalOptimizerV3() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool optimize(const LongitudinalSolverOption &option,
                const LonRefPath &lon_ref_path,
                std::vector<TrajectoryPoint> &res_traj_points);

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
