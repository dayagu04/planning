#pragma once

#include "apa_state_machine_manager.h"
#include "dp_speed_common.h"
#include "parking_task.h"
#include "piecewise_jerk_speed_config.h"
#include "speed/st_point.h"
#include "src/modules/apa_function/parking_task/deciders/speed_limit_decider/speed_limit_profile.h"
#include "src/modules/common/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "src/modules/common/speed/speed_data.h"

namespace planning {
namespace apa_planner {
// model
// x = [x0, x1, x2,...,x_{n-1}, x0',x1',...,x0'',x1'',...x_{n-1}'']
// f = 1/2*x^T*H*x + Q*x
// L<=Ax<=U
class PiecewiseJerkSpeedQPOptimizer : public ParkingTask {
 public:
  PiecewiseJerkSpeedQPOptimizer();

  virtual ~PiecewiseJerkSpeedQPOptimizer() = default;

  void Execute(const SVPoint& init_point,
               const SpeedLimitProfile* speed_limit_profile,
               const SpeedData& dp_speed_data,
               const SpeedDecisions* speed_decisions,
               const ParkingSpeedMode& park_speed_mode);

  const SpeedData& GetSpeedData() const { return qp_speed_data_; }

  bool Init() override;

 private:
  void DebugPiecewiseJerkProblem(const PiecewiseJerkSpeedProblem& pwj);

  void DebugRef(const std::vector<double>& t_ref,
                const std::vector<double>& x_ref,
                const std::vector<double>& v_ref);

  void DebugLinearConstraints(
      const std::vector<double>& x_ref,
      const std::vector<std::pair<double, double>>& s_dot_bounds);

  void RecordDebugInfo(
      const std::vector<double>& x_ref,
      const std::vector<std::pair<double, double>>& s_dot_bounds);

  void ClearDebugInfo();

  // 限速不能超过制动能力
  double ComputeFastBrakeProfile(const std::array<double, 3>& init,
                                 const double time);

  void GenerateInitState(const SVPoint& init_point,
                         std::array<double, 3>& init_state);

 private:
  PiecewiseJerkSpeedQPConfig qp_config_;
  const SpeedLimitProfile* speed_limit_profile_;
  int num_of_knots_;
  double delta_time_;

  SpeedData qp_speed_data_;

  ParkingSpeedMode park_speed_mode_ = ParkingSpeedMode::FAST;
};
}  // namespace apa_planner
}  // namespace planning
