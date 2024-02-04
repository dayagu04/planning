#ifndef __LATERAL_PATH_OPTIMIZER__
#define __LATERAL_PATH_OPTIMIZER__

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "geometry_math.h"
#include "lateral_path_optimizer.pb.h"
#include "spline.h"
#include "src/lateral_path_optimizer_cost.h"
#include "src/lateral_path_optimizer_model.h"
#include "src/lateral_path_optimizer_problem.h"

namespace planning {
namespace apa_planner {
class LateralPathOptimizer {
 public:
  struct Parameter {
    double sample_ds = 0.02;
    double q_ref_xy = 100.0;
    double q_ref_theta = 100.0;
    double q_terminal_xy = 1000.0;
    double q_terminal_theta = 9000.0;
    double q_k = 10.0;
    double q_u = 10.0;
    double q_k_bound = 100.0;
    double q_u_bound = 50.0;
  };
  void Init();
  void Update(const std::vector<pnc::geometry_lib::PathPoint> &path_vec,
              const uint8_t gear_cmd,
              const bool is_cilqr_path_optimization_enable);
  void SetParam(const Parameter &param) { param_ = param; }

  const std::vector<pnc::geometry_lib::PathPoint> &GetOutputPathVec() {
    return output_path_vec_;
  }

  const planning::common::LateralPathOptimizerOutput &GetOutputDebugInfo()
      const {
    return optimizer_planning_output_;
  }

 private:
  void AssembleInput(const std::vector<pnc::geometry_lib::PathPoint> &path_vec);
  void AssembleOutput();
  void PostProcessOutput();

  std::shared_ptr<apa_planner::LateralPathOptimizerProblem>
      planning_problem_ptr_;
  planning::common::LateralPathOptimizerInput planning_input_;
  planning::common::LateralPathOptimizerOutput optimizer_planning_output_;

  std::vector<pnc::geometry_lib::PathPoint> output_path_vec_;
  Parameter param_;
};
}  // namespace apa_planner
}  // namespace planning

#endif