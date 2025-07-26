#pragma once

#include <limits.h>

#include <map>

// #include "core/modules/common/config/basic_types.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "tasks/task_interface/lane_change_decider_output.h"
namespace planning {
class ResultTrajectoryGenerator : public Task {
 public:
  explicit ResultTrajectoryGenerator(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);
  virtual ~ResultTrajectoryGenerator() = default;

  bool Execute() override;

  bool TrajectoryGenerator();
  bool RealtimeTrajectoryGenerator();

  void Init();
  void UpdateHMIInfo();
  void UpdateTurnSignal();

  void UpdateTurnSignal();

  inline bool is_abnormal_number(double number) {
    return (std::isnan(number) == 1) || (std::isinf(number) != 0);
  }

 private:
  std::vector<double> t_vec_;
  std::vector<double> s_vec_;
  std::vector<double> l_vec_;
  std::vector<double> curvature_vec_;
  std::vector<double> dkappa_vec_;
  std::vector<double> ddkappa_vec_;
  std::vector<double> lat_acc_vec_;
  std::vector<double> lat_jerk_vec_;

  iflyauto::LandingPoint CalculateLandingPoint(
      bool is_lane_keeping,
      const LaneChangeDeciderOutput& lane_change_decider_output);

 private:
  ResultTrajectoryGeneratorConfig config_;
};

}  // namespace planning
