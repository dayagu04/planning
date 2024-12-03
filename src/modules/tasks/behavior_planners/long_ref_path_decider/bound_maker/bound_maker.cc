#include "bound_maker.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"

namespace planning {

BoundMaker::BoundMaker(const SpeedPlannerConfig& speed_planning_config,
                       framework::Session* session)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status BoundMaker::Run() {
  LOG_DEBUG("=======LongRefPathDecider: BoundMaker======= \n");
  // 1. acc bound
  MakeAccBound();

  // 2. s bound
  MakeSBound();

  // 3. v bound
  MakeVBound();

  // 4. jerk bound
  MakeJerkBound();

  return common::Status::OK();
}

void BoundMaker::MakeAccBound() {
  // @gpxu 待补充
}

void BoundMaker::MakeSBound() {
  // const double path_length =
  //     planning_data_.planner_output().planned_path().Length();
  // const double default_upper_bound =
  //     std::fmax(path_length, plan_time_ * kPerSecondPlanLenth);
  // s_lower_bound_ = std::vector<double>(plan_num_, -0.1);
  // s_upper_bound_ = std::vector<double>(plan_num_, default_upper_bound);
  // const auto& st_graph = planning_data_.st_graph_helper();
  // constexpr double kUpperBoundBuffer = 1.0;
  // constexpr double kLowSpeedBuffer = 0.2;

  // @gpxu 待补充
  // auto max_acceration_curve = GenerateMaxAccelerationCurve();
  // auto max_deceleration_curve = GenerateMaxDecelerationCurve();
}

void BoundMaker::MakeVBound() {
  // @gpxu 待补充
}

void BoundMaker::MakeJerkBound() {
  // @gpxu 待补充
}


double BoundMaker::s_lower_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_lower_bound_[index];
}

}  // namespace planning