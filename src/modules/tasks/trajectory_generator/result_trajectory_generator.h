#pragma once

#include <limits.h>

#include <map>

// #include "core/modules/common/config/basic_types.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "tasks/task_interface/lane_change_decider_output.h"
namespace planning {
enum class FaultType : int {
  TRAJ_LENGTH_RANGE,
  TRAJ_LENGTH_CONTINUITY,
  TRAJ_CURVATURE_RANGE,
  TRAJ_CURVATURE_CONTINUITY,
  TRAJ_LON_POS_CONSISTENCY,
  TRAJ_LON_VEL_CONSISTENCY,
  TRAJ_LON_ACC_CONSISTENCY,
  TRAJ_LON_DEC_CONSISTENCY,
  TRAJ_LAT_POS_CONSISTENCY,
  TRAJ_LAT_VEL_CONSISTENCY,
  TRAJ_LAT_ACC_CONSISTENCY,
  TRAJ_YAW_CONSISTENCY,
  TRAJ_ROLL_CONSISTENCY,
  NUM
};

struct FaultCounter {
  int fault_trigger_counter;
  int fault_recovery_counter;
};
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

  inline bool is_abnormal_number(double number) {
    return (std::isnan(number) == 1) || (std::isinf(number) != 0);
  }

  void CheckTrajectory();
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
  int lc_state_complete_frame_nums_ = 31;
  RampDirection last_frame_dir_turn_signal_road_to_ramp_ = RAMP_NONE;

 private:
  ResultTrajectoryGeneratorConfig config_;

  std::vector<FaultCounter> fault_counter_vec_;
  uint64_t faultcode_ = 666;
};

}  // namespace planning
