#pragma once

#include "dp_speed_common.h"
#include "parking_task.h"
#include "piecewise_jerk_speed_config.h"
#include "src/modules/apa_function/parking_task/deciders/speed_limit_decider/speed_limit_profile.h"
#include "src/modules/common/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "src/modules/common/speed/speed_data.h"

namespace planning {

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
               const SpeedData& dp_speed_data);

  const SpeedData& GetSpeedData() const { return qp_speed_data_; }

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

 private:
  PiecewiseJerkSpeedQPConfig qp_config_;
  const SpeedLimitProfile* speed_limit_profile_;
  int num_of_knots_;

  SpeedData qp_speed_data_;
};

}  // namespace planning
