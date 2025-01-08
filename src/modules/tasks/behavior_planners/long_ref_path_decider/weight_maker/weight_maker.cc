#include "weight_maker.h"

#include "environmental_model.h"
#include "planning_context.h"
#include "status/status.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"

namespace planning {

namespace {
constexpr double kUrgentWeightStartTime = 0.0;
constexpr double kUrgentWeightEndTime = 1.5;
constexpr double kAddUrgentWeightEndTime = 2.0;
constexpr double kUrgentSpeedThres = 4.0;
constexpr double lower_urgent_distance = 0.5;
constexpr double upper_urgent_distance = 1.5;
constexpr double lower_urgent_speed = 20.0 / 3.6;
constexpr double upper_urgent_speed = 80.0 / 3.6;
constexpr double upper_urgent_scale = 7.0;
constexpr double lower_urgent_scale = 5.0;
}  // namespace

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

  CollectDataToProto(target_maker);

  return common::Status::OK();
}

void WeightMaker::MakeSWeight(const TargetMaker& target_maker) {
  const double default_s_weight =
      speed_planning_config_.weight_maker_config.s_weight;
  const double follow_s_weight =
      speed_planning_config_.weight_maker_config.follow_s_weight;
  const double overtake_s_weight =
      speed_planning_config_.weight_maker_config.overtake_s_weight;
  const double neighbor_s_weight =
      speed_planning_config_.weight_maker_config.neighbor_s_weight;
  const double end_s_weight =
      speed_planning_config_.weight_maker_config.end_s_weight;
  const double max_s_weight_time =
      speed_planning_config_.weight_maker_config.max_s_weight_time;
  const double front_lower_weight =
      speed_planning_config_.weight_maker_config.front_lower_weight;
  const double back_upper_weight =
      speed_planning_config_.weight_maker_config.back_upper_weight;
  const double max_s_weight =
      speed_planning_config_.weight_maker_config.max_s_weight;
  const double s_speed_upper_weight_v =
      speed_planning_config_.weight_maker_config.s_speed_upper_weight_v;
  const double s_speed_lower_weight_v =
      speed_planning_config_.weight_maker_config.s_speed_lower_weight_v;
  const double s_speed_upper_weight =
      speed_planning_config_.weight_maker_config.s_speed_upper_weight;
  const double s_speed_lower_weight =
      speed_planning_config_.weight_maker_config.s_speed_lower_weight;

  auto virtual_acc_curve = MakeVirtualZeroAccCurve();
  const auto& st_graph = session_->planning_context().st_graph_helper();
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();

  const double speed_scale = planning_math::LerpWithLimit(
      s_speed_lower_weight, s_speed_lower_weight_v, s_speed_upper_weight,
      s_speed_upper_weight_v, init_lon_state_[1]);

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  int closest_index = -1;
  double min_urgent_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    if (!st_graph) {
      break;
    }
    const auto corridor_upper_point =
        st_graph->GetPassCorridorUpperBound(relative_t);
    if (corridor_upper_point.valid() && corridor_upper_point.agent_id() != -1) {
      // const auto* agent =
      //     agent_manager->GetAgent(corridor_upper_point.agent_id());
      // if (!agent) {
      //   break;
      // }
      const double agent_speed = corridor_upper_point.velocity();
      const double ego_s = virtual_acc_curve->Evaluate(0, relative_t);
      const double ego_s_to_front =
          ego_s + vehicle_param.front_edge_to_rear_axle;
      const double agent_s = corridor_upper_point.s();
      if (relative_t >= kUrgentWeightStartTime &&
          relative_t <= kUrgentWeightEndTime) {
        const double curr_dist = agent_s - ego_s_to_front;
        if (curr_dist < min_urgent_dist) {
          min_urgent_dist = curr_dist;
          closest_index = i;
        }
      }
    }
  }

  is_urgent_ = false;
  double urgent_scale = 1.0;
  if (closest_index >= 0) {
    double relative_t = closest_index * dt_;
    const auto corridor_upper_point =
        st_graph->GetPassCorridorUpperBound(relative_t);
    const double ego_speed = virtual_acc_curve->Evaluate(1, relative_t);
    const double agent_speed = corridor_upper_point.velocity();
    const double ego_s = virtual_acc_curve->Evaluate(0, relative_t);
    const double ego_s_to_front = ego_s + vehicle_param.front_edge_to_rear_axle;
    const double agent_s = corridor_upper_point.s();
    const double urgent_distance = planning_math::LerpWithLimit(
        lower_urgent_distance, lower_urgent_speed, upper_urgent_distance,
        upper_urgent_speed, ego_speed);
    if (agent_speed - ego_speed < kUrgentSpeedThres &&
        agent_s - ego_s_to_front < urgent_distance) {
      is_urgent_ = true;
      urgent_scale = planning_math::LerpWithLimit(
          upper_urgent_scale, 0.0, lower_urgent_scale, urgent_distance,
          agent_s - ego_s_to_front);
    }
  }

  s_weight_ = std::vector<double>(plan_points_num_, default_s_weight);
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto target_value = target_maker.target_value(relative_t);
    // std::cout << "t:" << relative_t
    //           << ",target value type:" <<
    //           Target::TargetValueType(target_value.target_type())
    //           << std::endl;
    if (target_value.target_type() == TargetType::kCruiseSpeed) {
      s_weight_[i] = default_s_weight;
    } else if (target_value.target_type() == TargetType::kFollow) {
      s_weight_[i] = follow_s_weight;
    } else if (target_value.target_type() == TargetType::kOvertake) {
      s_weight_[i] = overtake_s_weight;
    } else if (target_value.target_type() == TargetType::kNeighbor ||
               target_value.target_type() == TargetType::kNeighborYeild ||
               target_value.target_type() == TargetType::kNeighborOvertake) {
      s_weight_[i] = neighbor_s_weight;
    }
    const double mid_time =
        std::fmin(std::fmax(0.0, max_s_weight_time), plan_time_);
    if (relative_t <= mid_time) {
      const double front_time_scale = planning_math::LerpWithLimit(
          front_lower_weight, 0.0, max_s_weight, mid_time, relative_t);
      s_weight_[i] = s_weight_[i] * front_time_scale;
    } else {
      const double back_time_scale = planning_math::LerpWithLimit(
          max_s_weight, mid_time, back_upper_weight, plan_time_, relative_t);
      s_weight_[i] = s_weight_[i] * back_time_scale;
    }
    s_weight_[i] = s_weight_[i] * speed_scale;
    if (is_urgent_ && relative_t >= kUrgentWeightStartTime &&
        relative_t <= kAddUrgentWeightEndTime) {
      s_weight_[i] = s_weight_[i] * urgent_scale;
    }
  }
}

void WeightMaker::MakeVWeight(const TargetMaker& target_maker) {
  const double default_v_weight =
      speed_planning_config_.weight_maker_config.v_weight;
  const double cruise_v_weight =
      speed_planning_config_.weight_maker_config.cruise_v_weight;
  v_weight_ = std::vector<double>(plan_points_num_, default_v_weight);
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto target_value = target_maker.target_value(relative_t);
    if (target_value.target_type() == TargetType::kCruiseSpeed) {
      v_weight_[i] = cruise_v_weight;
    }
  }
}

void WeightMaker::MakeAccWeight() {
  const double default_a_weight =
      speed_planning_config_.weight_maker_config.a_weight;
  acc_weight_ = std::vector<double>(plan_points_num_, default_a_weight);
}

void WeightMaker::MakeJerkWeight() {
  const double default_jerk_weight =
      speed_planning_config_.weight_maker_config.jerk_weight;
  jerk_weight_ = std::vector<double>(plan_points_num_, default_jerk_weight);
}

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

void WeightMaker::Reset() {
  s_weight_.clear();
  v_weight_.clear();
  acc_weight_.clear();
  jerk_weight_.clear();
  weight_maker_replay_info_.Clear();
}

void WeightMaker::CollectDataToProto(const TargetMaker& target_maker) {
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_weight_data =
      debug_info_pb->mutable_weight_maker()->mutable_weight_maker_replay_info();
  mutable_weight_data->set_is_urgent(is_urgent_);
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto target_value = target_maker.target_value(relative_t);
    auto* ptr = weight_maker_replay_info_.add_target_point();
    ptr->set_s(target_value.s_target_val());
    ptr->set_t(target_value.relative_t());
    ptr->set_target_type(static_cast<int32_t>(target_value.target_type()));
    ptr->set_s_weight(s_weight_[i]);
  }
  mutable_weight_data->CopyFrom(weight_maker_replay_info_);
}

}  // namespace planning