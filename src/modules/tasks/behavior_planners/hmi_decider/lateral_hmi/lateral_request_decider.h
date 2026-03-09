#pragma once

#include "planning_context.h"
#include "session.h"

namespace planning {
class LateralRequestDecider {
 public:
  LateralRequestDecider(framework::Session* session);
  ~LateralRequestDecider() = default;

  bool Execute();

 private:
  void InitInfo();

  bool GenerateTakeoverRequest();

  bool CheckLateralACC();

  bool CheckPositionInLane();

  bool CheckLateralCollision();

  bool GenerateOutline();

 private:
  framework::Session* session_;
  pnc::mathlib::spline lbound_s_spline_;
  pnc::mathlib::spline rbound_s_spline_;
  bool is_out_lane_abnormal_;
};
}  // namespace planning