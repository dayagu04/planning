#include <memory>

#include "debug_info_log.h"
#include "filters.h"
#include "lateral_obstacle.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "st_graph.h"
#include "sv_graph.h"
#include "task.h"
#include "task_basic_types.pb.h"
#include "virtual_lane_manager.h"

namespace planning {

class RealTimeLonBehaviorPlanner : public Task {
 public:
  explicit RealTimeLonBehaviorPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  explicit RealTimeLonBehaviorPlanner(
      const EgoPlanningConfigBuilder *config_builder);

  virtual ~RealTimeLonBehaviorPlanner() = default;

  bool Execute(framework::Frame *frame);

  bool Update();
  void SetConfig(
      planning::common::RealTimeLonBehaviorTunedParams &tuned_params);
  planning::common::LonRefPath GetOutput() { return lon_behav_output_pb_; }

  // This func is for pybind to set input
  void SetInput(
      planning::common::RealTimeLonBehaviorInput &lon_behav_plan_input);

 private:
  void Init();

  virtual bool Calculate();

  void UpdateLonRefPath(const std::vector<double> &s_refs,
                        const std::vector<double> &v_refs,
                        const real_time::STboundaries &st_boundaries,
                        const SVBoundaries &sv_boundaries);

  void ConstructLonBehavInput();

  void GenerateLonRefPathPB();

  void UpdateHMI();

  void SaveToSession();

  void SaveToDebugInfo();

  void ClearOutput();

 private:
  RealTimeLonBehaviorPlannerConfig config_;
  std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_plan_input_;
  std::shared_ptr<StGraphGenerator> st_graph_;
  std::shared_ptr<SvGraphGenerator> sv_graph_;

  LonRefPath lon_behav_output_;
  planning::common::LonRefPath lon_behav_output_pb_;
};

}  // namespace planning