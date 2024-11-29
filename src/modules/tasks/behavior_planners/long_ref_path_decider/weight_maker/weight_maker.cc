#include "weight_maker.h"

#include "environmental_model.h"
#include "status/status.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"

namespace planning {

WeightMaker::WeightMaker(const SpeedPlannerConfig& speed_planning_config,
                         framework::Session* session)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status WeightMaker::Run(const TargetMaker& target_maker) {
  LOG_DEBUG("=======LongRefPathDecider: WeightMaker======= \n");
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_manager->planning_init_point();
  init_lon_state_ = {0, init_point.v, init_point.a};

  MakeSWeight(target_maker);

  MakeVWeight(target_maker);

  MakeAccWeight();

  MakeJerkWeight();

  return common::Status::OK();
}

void WeightMaker::MakeSWeight(const TargetMaker& target_maker) {}

void WeightMaker::MakeVWeight(const TargetMaker& target_maker) {}

void WeightMaker::MakeAccWeight() {}

void WeightMaker::MakeJerkWeight() {}

std::unique_ptr<Trajectory1d> WeightMaker::MakeVirtualZeroAccCurve() {
  auto virtual_zero_acc_curve =
      std::make_unique<PiecewiseJerkAccelerationTrajectory1d>(
          init_lon_state_[0], init_lon_state_[1]);
  virtual_zero_acc_curve->AppendSegment(init_lon_state_[2], dt_);

  const double zero_acc_jerk_max = speed_planning_config_.zero_acc_jerk_max;
  const double zero_acc_jerk_min = speed_planning_config_.zero_acc_jerk_min;
  for (double t = dt_; t <= plan_time_; t += dt_) {
    const double acc = virtual_zero_acc_curve->Evaluate(2, t);
    const double vel = virtual_zero_acc_curve->Evaluate(1, t);

    double a_next = 0.0;
    // if init acc < 0.0, move a to zero with jerk max
    // if init acc > 0.0, move a to zero with jerk min
    if (init_lon_state_[2] < 0.0) {
      a_next = acc + dt_ * zero_acc_jerk_max;
    } else {
      a_next = acc + dt_ * zero_acc_jerk_min;
    }

    if (init_lon_state_[2] * acc <= 0.0) {
      a_next = 0.0;
    }

    if (vel <= 0.0) {
      a_next = 0.0;
    }
    virtual_zero_acc_curve->AppendSegment(a_next, dt_);
  }
  return virtual_zero_acc_curve;
}

double WeightMaker::s_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_weight_[index];
}

double WeightMaker::v_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return v_weight_[index];
}

double WeightMaker::a_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return acc_weight_[index];
}

double WeightMaker::jerk_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return jerk_weight_[index];
}

}  // namespace planning