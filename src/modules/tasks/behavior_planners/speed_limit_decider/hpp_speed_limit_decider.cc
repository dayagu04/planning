#include "hpp_speed_limit_decider.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "environmental_model.h"
#include "planning_context.h"

namespace planning {
HPPSpeedLimitDecider::HPPSpeedLimitDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  hpp_speed_limit_config_ = config_builder->cast<LongitudinalDeciderV3Config>();
  name_ = "HPPSpeedLimitDecider";
}

bool HPPSpeedLimitDecider::Execute() {
  ILOG_INFO << "=======HPPSpeedLimitDecider=======";
  const auto& environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto plan_init_point = ego_state_mgr->planning_init_point();
  double v_cruise = ego_state_mgr->ego_v_cruise();
  v_target_ = v_cruise;
  v_target_type_ = SpeedLimitType::CRUISE;

  CalculateUserSpeedLimit();

  CalculateMapSpeedLimit();

  CalculateCurveSpeedLimit();

  CalculateNarrowAreaSpeedLimit();

  CalculateAvoidLimit();

  auto hpp_speed_limit_output = session_->mutable_planning_context()
                                    ->mutable_speed_limit_decider_output();
  hpp_speed_limit_output->SetSpeedLimit(v_target_, v_target_type_);
  JSON_DEBUG_VALUE("v_target_decider", v_target_);
  JSON_DEBUG_VALUE("v_target_type_code",
                   std::underlying_type<SpeedLimitType>::type(v_target_type_));

  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);

  if (SpeedLimitType::CURVATURE == v_target_type_) {
    ad_info->is_curva = true;
  } else {
    ad_info->is_curva = false;
  }

  return true;
}

void HPPSpeedLimitDecider::CalculateUserSpeedLimit() {
  const double user_velocity_limit =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  if (user_velocity_limit < v_target_) {
    v_target_ = user_velocity_limit;
    v_target_type_ = SpeedLimitType::USER;
  }
}

void HPPSpeedLimitDecider::CalculateMapSpeedLimit() { return; }
void HPPSpeedLimitDecider::CalculateCurveSpeedLimit() {
  const auto& traj_points = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_points;

  const double max_lat_acceleration = ComputeMaxLatAcceleration();
  double vlimit_jerk = 0.0;
  double time_to_brake = 1e-2;
  double out_max_curvature = 0.0;

  double v_limit_curv =
      ComputeCurvatureSpeedLimit(traj_points, max_lat_acceleration,
                                 vlimit_jerk, time_to_brake, out_max_curvature);

  if (v_limit_curv < v_target_) {
    v_target_ = v_limit_curv;
    v_target_type_ = SpeedLimitType::CURVATURE;
  }

  max_curvature_ = out_max_curvature;
}
void HPPSpeedLimitDecider::CalculateNarrowAreaSpeedLimit() { return; }

void HPPSpeedLimitDecider::CalculateAvoidLimit() {
  double v_limit_avoid = hpp_speed_limit_config_.velocity_upper_bound;
  if (!session_->is_hpp_scene()) {
    return;
  }

  const auto& hpp_general_lateral_decider_output =
      session_->planning_context().hpp_general_lateral_decider_output();
  if (hpp_general_lateral_decider_output.avoid_ids.empty()) {
    return;
  }

  // 触发避让限速，使用可配置的 config.hpp_avoid_velocity_limit_kph
  v_limit_avoid =
      hpp_speed_limit_config_.hpp_avoid_velocity_limit_kph / 3.6;  // 转换为 m/s
  LOG_DEBUG("[get_velocity_limit] HPP avoid speed limit triggered: %f kph",
            hpp_speed_limit_config_.hpp_avoid_velocity_limit_kph);

  if (v_limit_avoid < v_target_) {
    v_target_ = v_limit_avoid;
    v_target_type_ = SpeedLimitType::AVOID;
  }
}

const double HPPSpeedLimitDecider::ComputeMaxLatAcceleration() {
  constexpr double kUrbanVelocityThld = 3.0;
  constexpr double kHighwayVelocityThld = 20.0;
  constexpr double kUrbanLatAccBuffMax = 0.7;
  constexpr double kHighwayLatAccBuffMax = 0.5;
  constexpr double kUrbanLatAccRate = 0.2;
  constexpr double kHighwayLatAccRate = 0.0;
  constexpr double kLowSpeedLaneChangeAccBuff = 0.3;
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const FrenetEgoState& frenet_ego_state =
      reference_path_ptr->get_frenet_ego_state();
  double ego_velocity = frenet_ego_state.planning_init_point().v;
  const CoarsePlanningInfo& coarse_planning_info =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const LaneChangeDeciderOutput& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const StateMachineLaneChangeStatus state = coarse_planning_info.target_state;
  const int lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  auto lc_acc_decay =
      ((is_LC_LCHANGE || is_LC_RCHANGE) && ego_velocity < kUrbanVelocityThld)
          ? kLowSpeedLaneChangeAccBuff
          : 0.0;

  const double max_lat_acceleration =
      hpp_speed_limit_config_.urban_max_lat_acceleration - lc_acc_decay +
      std::min(
          kUrbanLatAccBuffMax,
          kUrbanLatAccRate * std::max(ego_velocity - kUrbanVelocityThld, 0.0));
  return max_lat_acceleration;
}

const double HPPSpeedLimitDecider::ComputeCurvatureSpeedLimit(
    const TrajectoryPoints& traj_points, double max_lat_acceleration,
    double& vlimit_jerk, double& time_to_brake, double& out_max_curvature) {
  const double ego_velocity =
      session_->environmental_model().get_ego_state_manager()->planning_init_point().v;
  const double trigger_distance = interp(
      ego_velocity, hpp_speed_limit_config_.curv_trigger_velocity_breakpoints,
      hpp_speed_limit_config_.curv_trigger_distances);

  // a. 计算轨迹上的最大曲率及其距离
  double max_curvature = 1e-4;
  double max_curv_s = -1.0;
  double s_accumulate = 0.0;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (std::fabs(traj_points[i].curvature) > max_curvature) {
      max_curvature = std::fabs(traj_points[i].curvature);
      max_curv_s = s_accumulate;
    }
    if (i > 0) {
      s_accumulate +=
          planning_math::fast_hypot(traj_points[i].x - traj_points[i - 1].x,
                                    traj_points[i].y - traj_points[i - 1].y);
    }
  }

  // 考虑方向盘转角对应的曲率
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double steer_angle = ego_state->ego_steer_angle();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_curvature = std::tan(steer_angle / vehicle_param.steer_ratio) /
                           vehicle_param.wheel_base;
  max_curvature = std::max(max_curvature, std::fabs(steer_curvature));

  // 输出最大曲率
  out_max_curvature = max_curvature;

  // b. 计算弯道限速
  double v_limit_curv;

  // HPP模式下使用查表方式计算限速
  if (session_->is_hpp_scene()) {
    // 将曲率转换为半径 (半径 = 1/曲率)
    double radius = 1.0 / max_curvature;
    // 查表：半径(m) -> 限速(m/s). 5m→5kph, 10m→6kph, 15m→8kph, 20m→10kph,

    v_limit_curv =
        interp(radius, hpp_speed_limit_config_.curv_radius_breakpoints,
               hpp_speed_limit_config_.curv_speed_limits_ms);

  } else {
    // 非HPP模式下使用原有的基于横向加速度的计算方式
    // v = sqrt(a_lat / kappa)
    v_limit_curv = std::sqrt(max_lat_acceleration / max_curvature);
  }

  v_limit_curv =
      std::max(v_limit_curv, hpp_speed_limit_config_.velocity_lower_bound);
  v_limit_curv =
      std::min(v_limit_curv, hpp_speed_limit_config_.velocity_upper_bound);

  // c. 判断是否触发限速
  // 条件：曲率限速低于当前速度 && 最大曲率距离满足触发条件
  vlimit_jerk = 0.0;
  time_to_brake = 1e-2;

  if (max_curv_s <
      trigger_distance + max_curvature_slow_down_triger_buffer_) {
    // 需要减速以满足弯道限速
    const double ego_a = -2.0;  // 假设减速加速度 -2 m/s²
    const double b_square_minus_4ac =
        std::pow((2 * ego_velocity + v_limit_curv), 2) + 6 * ego_a * max_curv_s;

    if (b_square_minus_4ac > 0.0) {
      double time_to_brake =
          (-(2 * ego_velocity + v_limit_curv) + std::sqrt(b_square_minus_4ac)) /
          ego_a;
      time_to_brake = std::max(time_to_brake, 0.01);
      vlimit_jerk = 2 * (v_limit_curv - ego_velocity - ego_a * time_to_brake) /
                    std::pow(time_to_brake, 2);
    }

    max_curvature_slow_down_triger_buffer_ = 1.0;
    return v_limit_curv;
  }
  max_curvature_slow_down_triger_buffer_ = -1.0;
  return hpp_speed_limit_config_.velocity_upper_bound;
}

const double HPPSpeedLimitDecider::ComputeAvoidVelocityLimit(
    const framework::Session& session) {
  double v_limit_avoid = hpp_speed_limit_config_.velocity_upper_bound;
  if (!session.is_hpp_scene()) {
    return v_limit_avoid;
  }

  const auto& hpp_general_lateral_decider_output =
      session.planning_context().hpp_general_lateral_decider_output();
  if (hpp_general_lateral_decider_output.avoid_ids.empty()) {
    return v_limit_avoid;
  }

  // 触发避让限速，使用可配置的 config.hpp_avoid_velocity_limit_kph
  v_limit_avoid =
      hpp_speed_limit_config_.hpp_avoid_velocity_limit_kph / 3.6;  // 转换为 m/s
  LOG_DEBUG("[get_velocity_limit] HPP avoid speed limit triggered: %f kph",
            hpp_speed_limit_config_.hpp_avoid_velocity_limit_kph);

  return v_limit_avoid;
}
}  // namespace planning