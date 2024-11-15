#pragma once

#include <memory>
#include <vector>

#include "debug_info_log.h"
#include "filters.h"
#include "lateral_obstacle.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "scc_st_graph.h"
#include "scc_sv_graph.h"
#include "task_basic_types.pb.h"
#include "tasks/task.h"
#include "tasks/task_interface/longitudinal_decider_output.h"
#include "virtual_lane_manager.h"

namespace planning {

class SccLonBehaviorPlanner : public Task {
 public:
  explicit SccLonBehaviorPlanner(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session);

  virtual ~SccLonBehaviorPlanner() = default;

  bool Execute();

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
                        const scc::STboundaries &st_boundaries,
                        const SVBoundaries &sv_boundaries,
                        const std::pair<double, double> &a_bounds,
                        const std::pair<double, double> &j_bounds);

  void ConstructLonBehavInput();

  void GenerateLonRefPathPB();

  void UpdateHMI();

  void GetHardBounds();

  void SaveToSession();

  void SaveToDebugInfo();

  void ClearOutput();

  bool GenerateFarSlowCarFollowCurve(std::vector<double> &s_refs);

  bool GenerateStableFollowSlowCurve(std::vector<double> &s_refs);

  bool IsLeadVehicle(const planning::common::TrackedObjectInfo &lead);

  bool JudgeCurvBySDMap();

  struct STBound {
    double lower = -1.0e4;
    double upper = 1.0e4;
    double vel;
    double acc;
    int id = 0;
  };

 private:
  SccLonBehaviorPlannerConfig config_;
  std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_plan_input_;
  std::shared_ptr<scc::StGraphGenerator> st_graph_;
  std::shared_ptr<scc::SvGraphGenerator> sv_graph_;

  std::vector<STBound> hard_bounds_;
  std::array<double, 3> lon_init_state_;

  LongitudinalDeciderOutput lon_behav_output_;
  planning::common::LonRefPath lon_behav_output_pb_;
};

}  // namespace planning