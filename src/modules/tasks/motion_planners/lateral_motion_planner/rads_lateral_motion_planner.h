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

class RADSLateralMotionPlanner : public BaseLateralMotionPlanner {
 public:
  RADSLateralMotionPlanner(const EgoPlanningConfigBuilder* config_builder,
                           framework::Session* session);

  virtual ~RADSLateralMotionPlanner() = default;

  bool Execute() override;

 private:
  void Init();

  bool HandleInputData();

  bool AssembleInput();

  bool Update();

 private:
  std::shared_ptr<pnc::lateral_planning::iLQRSolver> ilqr_solver_ptr_;
};

}  // namespace planning