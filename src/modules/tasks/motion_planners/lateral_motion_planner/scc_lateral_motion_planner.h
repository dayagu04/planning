/**
 * @file lateral_motion_planner.h
 **/

#pragma once

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "base_lateral_motion_planner.h"
#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_motion_planner.pb.h"
#include "math_lib.h"
#include "obstacle_manager.h"

#include "task_basic_types.h"
#include "tasks/task.h"

namespace planning {

class SCCLateralMotionPlanner : public BaseLateralMotionPlanner {
 public:
  SCCLateralMotionPlanner(const EgoPlanningConfigBuilder* config_builder,
                          framework::Session* session);

  virtual ~SCCLateralMotionPlanner() = default;

  bool Execute() override;

 private:
  void Init();

  bool HandleInputData();

  void ResetInput();

  bool AssembleInput();

  bool Update();

  bool IsLocatedInSplitArea();

  NudgeDirection CalculateDrivingDirectionForLeavingLane();

  double CalculateRemainingDrivingTimeToSolidLine(bool is_check_left, bool is_check_right);

 private:
  std::shared_ptr<pnc::lateral_planning::iLQRSolver> ilqr_solver_ptr_;

  bool is_divide_lane_into_two_;
  bool is_last_low_speed_lane_change_;
  int low_speed_lane_change_cd_timer_;
  double avoid_back_time_;
  double enter_split_time_;
  double enter_lccnoa_time_;
  double driving_away_lane_time_;

  std::vector<double> history_steer_vec_;
  std::vector<double> expected_steer_vec_;
};

}  // namespace planning