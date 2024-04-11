#include "debug_info_log.h"
#include "define/lateral_behavior_planner_output.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "scc_lon_behavior_types.h"
#include "session.h"
#include "task_basic_types.h"

namespace planning {
namespace scc {

class SvGraphGenerator {
 public:
  explicit SvGraphGenerator(const SccLonBehaviorPlannerConfig &plan_cfg);
  virtual ~SvGraphGenerator() = default;

  // 更新
  void Update(std::shared_ptr<common::RealTimeLonBehaviorInput>);
  const SVBoundaries &GetSVBoundaries() { return sv_boundaries_; }

  void SetConfig(
      planning::common::RealTimeLonBehaviorTunedParams &tuned_params);

 private:
  void CalculateCurvSvs(const double v_ego, const double v_cruise,
                        const double angle_steers,
                        const std::vector<double> &d_poly,
                        SVBoundary &sv_boundary);

  void UpdateSVGraphs(const SVBoundaries &sv_boundaries);

 private:
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{125, 250, 500, 680};
  const std::vector<double> _AY_MAX_CURV_V{1.6, 1.2, 0.8, 0.6};

 private:
  SccLonBehaviorPlannerConfig config_;
  std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input_;
  SVBoundaries sv_boundaries_;
};

}  // namespace scc
}  // namespace planning
