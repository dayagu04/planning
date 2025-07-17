#include "ego_state_manager.h"

// #include <fastrtps/config.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "func_state_machine_c.h"
#include "ifly_time.h"
#include "log.h"
#include "math_lib.h"
#include "planning_context.h"
#include "refline.h"
#include "spline_projection.h"
#include "trajectory/trajectory_stitcher.h"
#include "utils/pose2d_utils.h"
#include "vehicle_config_context.h"
#include "vehicle_model/vehicle_model.h"
#include "vehicle_status.pb.h"

namespace planning {

static double planning_loop_dt = 0.1;
static const double curve_factor = 0.30;

EgoStateManager::EgoStateManager(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  SetConfig(config_builder);
  // init v_cruise_filter: -1.5m/s2, 1.5m/s2, 0-150km/h, 10hz
  v_cruise_filter_.Init(-1.5, 1.5, 0.0, 42.0, planning_loop_dt);
}

void EgoStateManager::SetConfig(
    const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<EgoPlanningEgoStateManagerConfig>();
  cruise_routing_speed_ = config_.cruise_routing_speed;
  cruise_searching_speed_ = config_.cruise_searching_speed;
  rads_cruise_speed_ = config_.rads_cruise_speed;
  max_replan_lat_err_ = config_.max_replan_lat_err;
  max_replan_theta_err_ = config_.max_replan_theta_err;
  max_replan_dist_err_ = config_.max_replan_dist_err;
  hpp_max_replan_lat_err_ = config_.hpp_max_replan_lat_err;
  hpp_max_replan_theta_err_ = config_.hpp_max_replan_theta_err;
  hpp_max_replan_lon_err_ = config_.hpp_max_replan_lon_err;
  hpp_max_replan_dist_err_ = config_.hpp_max_replan_dist_err;
  enable_delta_stitch_in_replan_ = config_.enable_delta_stitch_in_replan;
  enable_ego_state_compensation_ = config_.enable_ego_state_compensation;
#ifdef X86
  if (SimulationContext::Instance()->is_close_loop()) {
    enable_ego_state_compensation_ = false;
  }
#endif
}

void EgoStateManager::set_ego_carte(const Point2D &ego_carte) {
  ego_carte_.x = ego_carte.x;
  ego_carte_.y = ego_carte.y;
}

void EgoStateManager::set_ego_pose_and_vel(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_pose_.x = vehicle_status.location().location_enu().x();
  ego_pose_.y = vehicle_status.location().location_enu().y();
  ego_pose_.theta = vehicle_status.heading_yaw().heading_yaw_data().value_rad();
  ego_pose_raw_ = ego_pose_;

  ego_v_ = vehicle_status.velocity().heading_velocity().value_mps();
  ego_v_angle_ = vehicle_status.heading_yaw().heading_yaw_data().value_rad();
  ego_hmi_v_ = vehicle_status.velocity().hmi_speed();
  ego_yaw_rate_ =
      vehicle_status.angular_velocity().heading_yaw_rate().value_rps();
}

void EgoStateManager::set_ego_position_llh(
    const planning::common::VehicleStatus &vehicle_status) {
  const auto &location_geographic =
      vehicle_status.location().location_geographic();
  position_llh_.Longitude = location_geographic.longitude_degree();
  position_llh_.Latitude = location_geographic.latitude_degree();
  position_llh_.height = location_geographic.altitude_meter();
}

void EgoStateManager::set_ego_enu(
    const planning::common::VehicleStatus &vehicle_status) {
  const auto &location_enu = vehicle_status.location().location_enu();
  location_enu_.position.x = location_enu.x();
  location_enu_.position.y = location_enu.y();
  location_enu_.position.z = location_enu.z();
  location_enu_.orientation.x = location_enu.orientation().x();
  location_enu_.orientation.y = location_enu.orientation().y();
  location_enu_.orientation.z = location_enu.orientation().z();
  location_enu_.orientation.w = location_enu.orientation().w();
}

void EgoStateManager::set_ego_steer_angle(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_steer_angle_ = vehicle_status.steering_wheel()
                         .steering_wheel_data()
                         .steering_wheel_rad();
}

void EgoStateManager::set_ego_acc(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_acc_last_ = ego_acc_;
  ego_acc_ = vehicle_status.brake_info()
                 .brake_info_data()
                 .acceleration_on_vehicle_wheel();
}

void EgoStateManager::set_ego_jerk() {
  if (timestamp_us_ == timestamp_us_last_) {
    jerk_ = 0;
  } else {
    jerk_ = (ego_acc_ - ego_acc_last_) /
            ((timestamp_us_ - timestamp_us_last_) / 1000000.0);
  }
}

void EgoStateManager::set_ego_v_cruise(
    const planning::common::VehicleStatus &vehicle_status) {
  if (!session_->environmental_model().GetVehicleDbwStatus()) {
    v_cruise_filter_.SetState(
        vehicle_status.velocity().heading_velocity().value_mps());
  } else {
    v_cruise_filter_.Update(
        vehicle_status.velocity().cruise_velocity().value_mps());
  }
  ego_v_cruise_ = v_cruise_filter_.GetOutput();
  if (session_->is_hpp_scene()) {
    if (session_->environmental_model()
            .get_local_view()
            .function_state_machine_info.current_state ==
        iflyauto::FunctionalState_HPP_CRUISE_SEARCHING) {
      const double kMaxSearchingSpeed = 15 / 3.6;
      ego_v_cruise_ = std::min(cruise_searching_speed_, kMaxSearchingSpeed);
    } else {
      ego_v_cruise_ = cruise_routing_speed_;
    }
  }
  if (session_->is_rads_scene()) {
    ego_v_cruise_ = config_.rads_cruise_speed;
  }
}

void EgoStateManager::set_ego_t_distance(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_t_distance_ = vehicle_status.navi_time_distance();
}

void EgoStateManager::set_ego_start_stop(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_start_stop_ = vehicle_status.start_stop_info();
}

void EgoStateManager::set_throttle_override(
    const planning::common::VehicleStatus &vehicle_status) {
  throttle_override_ = vehicle_status.throttle().throttle_data().override();
}

void EgoStateManager::set_ego_blinker(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_blinker_ =
      vehicle_status.vehicle_light().vehicle_light_data().turn_signal().value();
}

void EgoStateManager::set_ego_blinker(
    const planning::common::VehicleLight &vehicle_light) {
  ego_blinker_ = vehicle_light.vehicle_light_data().turn_signal().value();
}

void EgoStateManager::set_ego_auto_light_state(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_auto_light_state_ =
      vehicle_status.vehicle_light().vehicle_light_data().auto_light_state();
}

void EgoStateManager::set_driver_hand_state(
    const planning::common::VehicleStatus &vehicle_status) {
  driver_hand_torque_ = vehicle_status.driver_hand_state().driver_hand_torque();
  driver_hands_off_state_ =
      vehicle_status.driver_hand_state().driver_hands_off_state();
}

void EgoStateManager::set_ego_gear(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_gear_ = vehicle_status.gear().gear_data().gear_status().value();
}

void EgoStateManager::set_time_headway_level(
    const planning::common::VehicleStatus &vehicle_status) {
  time_headway_level_ =
      std::round(vehicle_status.time_headway_level().value_num());
}

void EgoStateManager::update_transform() {
  Eigen::Vector4d q;
  q.x() = location_enu_.orientation.x;
  q.y() = location_enu_.orientation.y;
  q.z() = location_enu_.orientation.z;
  q.w() = location_enu_.orientation.w;

  Eigen::Vector3d v;
  v.x() = location_enu_.position.x;
  v.y() = location_enu_.position.y;
  v.z() = location_enu_.position.z;

  car2enu_ = define::Transform(q, v);
  enu2car_ = define::Transform(q, v).inverse();
}

void EgoStateManager::set_timestamp_us(
    const planning::common::VehicleStatus &vehicle_status) {
  timestamp_us_last_ = timestamp_us_;
  timestamp_us_ = vehicle_status.header().timestamp_us();
}
bool EgoStateManager::update(
    const planning::common::VehicleStatus &vehicle_status) {
  set_timestamp_us(vehicle_status);
  set_ego_position_llh(vehicle_status);
  set_ego_enu(vehicle_status);
  set_ego_pose_and_vel(vehicle_status);
  set_ego_carte(convert_pose2point(ego_pose_));
  // set_ego_prediction_info(vehicle_status);
  set_ego_steer_angle(vehicle_status);
  set_ego_acc(vehicle_status);
  set_ego_v_cruise(vehicle_status);
  set_ego_t_distance(vehicle_status);
  set_ego_start_stop(vehicle_status);
  set_throttle_override(vehicle_status);
  set_ego_blinker(vehicle_status);
  set_ego_auto_light_state(vehicle_status);
  set_driver_hand_state(vehicle_status);
  set_ego_gear(vehicle_status);
  set_ego_jerk();
  set_time_headway_level(vehicle_status);
  const auto &planning_result = session_->planning_context().planning_result();
  const auto &last_planning_result =
      session_->planning_context().last_planning_result();
  if (last_planning_result.timestamp > 0) {
    planning_loop_dt =
        (planning_result.timestamp - last_planning_result.timestamp) / 1000.0;
  }
  printf("planning_loop_dt:%f\n", planning_loop_dt);
  JSON_DEBUG_VALUE("planning_loop_dt", planning_loop_dt);
#ifdef X86
  planning_loop_dt = SimulationContext::Instance()->planning_loop_dt();
#endif
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  steer_ratio_ = vehicle_param.steer_ratio;
  max_steer_angle_ = vehicle_param.max_steer_angle;
  max_delta_ = vehicle_param.max_front_wheel_angle;
  planning_math::Vec2d center(
      ego_pose_.x +
          std::cos(ego_pose_.theta) * vehicle_param.rear_axle_to_center,
      ego_pose_.y +
          std::sin(ego_pose_.theta) * vehicle_param.rear_axle_to_center);
  planning_math::Box2d ego_box(center, ego_pose_.theta, vehicle_param.length,
                               vehicle_param.width);
  polygon_ = planning_math::Polygon2d(ego_box);
  LOG_DEBUG("ego center's x: [%f], y: [%f] \n", center.x(), center.y());

  update_transform();

  // planning init state for realtime & longtime decided by location_valid
  if (session_->environmental_model().location_valid()) {
    UpdatePlanningInitState();
  } else {
    RealtimeUpdatePlanningInitState();
  }
  return true;
}

uint8_t EgoStateManager::ReplanProcess(const bool &set_lat_replan,
                                       const bool &set_lon_replan) {
  // note that lon_reset_flag and lat_reset_flag reserved for acc and override

  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  auto &lat_init_state = planning_init_point_.lat_init_state;

  JSON_DEBUG_VALUE("enable_ego_state_compensation",
                   enable_ego_state_compensation_);
  // Eigen::Vector2d cur_pos(ego_state->ego_pose_raw().x,
  //                         ego_state->ego_pose_raw().y);
  Eigen::Vector2d cur_pos(cur_vehicle_state_process_.x,
                          cur_vehicle_state_process_.y);
  JSON_DEBUG_VALUE("predicted_ego_x", cur_vehicle_state_process_.x);
  JSON_DEBUG_VALUE("predicted_ego_y", cur_vehicle_state_process_.y);
  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(
      motion_planner_output.x_s_spline, motion_planner_output.y_s_spline,
      motion_planner_output.s_lat_vec.front(),
      motion_planner_output.s_lat_vec.back(), cur_pos);

  const auto &lat_err = projection_spline.GetOutput().dist_proj;
  const auto &proj_point = projection_spline.GetOutput().point_proj;
  Eigen::Vector2d init_point(lat_init_state.x(), lat_init_state.y());
  const double lat_init_theta = lat_init_state.theta();
  // double theta_err = lat_init_theta - ego_state->ego_pose_raw().theta;
  double theta_err = lat_init_theta - cur_vehicle_state_process_.heading;
  const double pi2 = 2.0 * M_PI;
  if (theta_err > M_PI) {
    lat_init_state.set_theta(lat_init_theta - pi2);
    theta_err -= pi2;
  } else if (theta_err < -M_PI) {
    lat_init_state.set_theta(lat_init_theta + pi2);
    theta_err += pi2;
  }
  const auto lon_err = std::hypot(init_point.x() - proj_point.x(),
                                  init_point.y() - proj_point.y());
  const double dist_err =
      std::hypot(lat_init_state.x() - ego_state->ego_pose_raw().x,
                 lat_init_state.y() - ego_state->ego_pose_raw().y);

  JSON_DEBUG_VALUE("lat_err", lat_err)
  JSON_DEBUG_VALUE("theta_err", theta_err)
  JSON_DEBUG_VALUE("lon_err", lon_err)
  JSON_DEBUG_VALUE("dist_err", dist_err)

  double max_replan_lat_err = max_replan_lat_err_;
  double max_replan_theta_err = max_replan_theta_err_ / 57.3;
  max_replan_lon_err_ =
      interp(ego_v_, config_.replan_longitudinal_distance_threshold_speed,
             config_.replan_longitudinal_distance_threshold_value);
  double max_replan_lon_err = max_replan_lon_err_;
  double max_replan_dist_err = max_replan_dist_err_;
  if (session_->is_hpp_scene()) {
    hpp_max_replan_lat_err_ =
        interp(ego_v_, config_.hpp_replan_threshold_speed,
               config_.hpp_replan_lat_err_threshold_value);
    hpp_max_replan_lon_err_ =
        interp(ego_v_, config_.hpp_replan_threshold_speed,
               config_.hpp_replan_lon_err_threshold_value);
    max_replan_lat_err = hpp_max_replan_lat_err_;
    max_replan_theta_err = hpp_max_replan_theta_err_ / 57.3;
    max_replan_lon_err = hpp_max_replan_lon_err_;
    max_replan_dist_err = hpp_max_replan_dist_err_;
  }

  double ego_acc_replan = session_->is_rads_scene() ? -(ego_state->ego_acc())
                                                    : ego_state->ego_acc();
  double ego_vel_replan =
      session_->is_rads_scene() ? -(ego_state->ego_v()) : ego_state->ego_v();
  const auto start_stop_state =
      session_->planning_context().start_stop_result().state();
  if (start_stop_state == common::StartStopInfo::START) {
    ego_acc_replan = std::max(0.0, ego_acc_replan);
  } else if (start_stop_state == common::StartStopInfo::STOP) {
    ego_acc_replan = std::min(0.0, ego_acc_replan);
  }
  bool low_speed_replan = (ego_vel_replan < config_.kEpsilon_v) &&
                          (start_stop_state == common::StartStopInfo::START);
  // avoid dramatic acc in ACC mode
  const bool is_acc_mode =
      session_->environmental_model().function_info().function_mode() ==
      common::DrivingFunctionInfo::ACC;
  if (is_acc_mode && ego_acc_replan > 1.0) {
    ego_acc_replan = 1.0;
  }
  cur_vehicle_state_process_.linear_acceleration = ego_acc_replan;
  cur_vehicle_state_process_.linear_velocity = ego_vel_replan;
  VehicleState cur_vehicle_state = cur_vehicle_state_process_;

  // replan type judge
  int replan_code = 0;
  if (fabs(lat_err) > max_replan_lat_err) {
    replan_type_.insert(LAT_POSITION_REPLAN);
    replan_code += LAT_POSITION_REPLAN;
  }
  if (fabs(theta_err) > max_replan_theta_err) {
    replan_type_.insert(LAT_ANGLE_REPLAN);
    replan_code += LAT_ANGLE_REPLAN;
  }
  if (set_lat_replan && set_lon_replan) {
    replan_type_.insert(LAT_LON_REPLAN);
    replan_code += LAT_LON_REPLAN;
  }
  if (set_lat_replan && !set_lon_replan) {
    replan_type_.insert(LAT_REPLAN);
    replan_code += LAT_REPLAN;
  }
  if (set_lon_replan && !set_lat_replan) {
    replan_type_.insert(LON_REPLAN);
    replan_code += LON_REPLAN;
  }
  if (fabs(lon_err) > max_replan_lon_err) {
    replan_type_.insert(LON_POSITION_REPLAN);
    replan_code += LON_POSITION_REPLAN;
  }
  if (low_speed_replan) {
    replan_type_.insert(LON_TINY_SPEED_REPLAN);
    replan_code += LON_TINY_SPEED_REPLAN;
  }

  PncTrajectoryPoint reinit_point;
  if (!replan_type_.empty()) {
    if (replan_type_.find(LON_TINY_SPEED_REPLAN) != replan_type_.end()) {
      enable_delta_stitch_in_replan_ = true;
      reinit_point = TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
          cur_vehicle_state);
      LateralInitStateReset(reinit_point);
      LongitudinalInitStateReset(reinit_point);
    } else if (replan_type_.find(LAT_LON_REPLAN) != replan_type_.end()) {
      enable_delta_stitch_in_replan_ = false;
      reinit_point = TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
          cur_vehicle_state);
      LateralInitStateReset(reinit_point);
      LongitudinalInitStateReset(reinit_point);
    } else if (replan_type_.find(LON_REPLAN) != replan_type_.end()) {
      reinit_point = TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
          cur_vehicle_state);
      if (replan_type_.find(LAT_POSITION_REPLAN) == replan_type_.end() &&
          replan_type_.find(LAT_ANGLE_REPLAN) == replan_type_.end()) {
        LongitudinalInitStateReset(reinit_point);
      } else {
        enable_delta_stitch_in_replan_ = false;
        LateralInitStateReset(reinit_point);
        LongitudinalInitStateReset(reinit_point);
      }
    } else if (replan_type_.find(LAT_REPLAN) != replan_type_.end()) {
      enable_delta_stitch_in_replan_ = false;
      reinit_point = TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
          cur_vehicle_state);
      if (replan_type_.find(LON_POSITION_REPLAN) != replan_type_.end()) {
        LateralInitStateReset(reinit_point);
        LongitudinalInitStateReset(reinit_point);
      } else {
        LateralInitStateReset(reinit_point);
      }
    } else {
      reinit_point = TrajectoryStitcher::ComputeReinitStitchingTrajectory(
          planning_loop_dt, cur_vehicle_state);
      LateralInitStateReset(reinit_point);
      LongitudinalInitStateReset(reinit_point);
    }
  }
  enable_delta_stitch_in_replan_ = config_.enable_delta_stitch_in_replan;
  return replan_code;
}

void EgoStateManager::CompensateEgoStateForLocalizationLatency() {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  cur_vehicle_state_process_.angular_velocity = ego_state->ego_yaw_rate();
  cur_vehicle_state_process_.linear_velocity =
      std::max(ego_state->ego_v(), 0.0);
  if (session_->is_rads_scene()) {
    cur_vehicle_state_process_.linear_velocity =
        std::min(ego_state->ego_v(), 0.0);
  }
  cur_vehicle_state_process_.jerk = ego_state->ego_jerk();
  cur_vehicle_state_process_.linear_acceleration = ego_state->ego_acc();
  cur_vehicle_state_process_.delta =
      ego_state->ego_steer_angle() / steer_ratio_;
  cur_vehicle_state_process_.heading = ego_state->heading_angle();
  cur_vehicle_state_process_.kappa =
      curve_factor * cur_vehicle_state_process_.delta;
  cur_vehicle_state_process_.x = ego_state->ego_pose().x;
  cur_vehicle_state_process_.y = ego_state->ego_pose().y;
  if (!enable_ego_state_compensation_) {
    return;
  }
  constexpr double US_PER_MS = 1000.0;
  auto cur_time_us = IflyTime::Now_us();
  const auto &local_view = session_->environmental_model().get_local_view();
  const auto &localization_timestamp_us =
      local_view.localization.meta.timestamp;

  uint64_t localization_latency_ms =
      localization_timestamp_us == 0
          ? 0
          : (cur_time_us - localization_timestamp_us) / US_PER_MS;  // ms
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  double localization_latency_s = localization_latency_ms / 1000.0;

#ifdef X86
  localization_latency_s =
      SimulationContext::Instance()->localizatoin_latency() / 1000.0;
#endif

  JSON_DEBUG_VALUE("localizatoin_latency_inEgoStateManager",
                   localization_latency_ms);
  JSON_DEBUG_VALUE("new_localization_latency",
                   planning_debug_data->input_topic_latency().localization());
  const auto cur_vehi_acc = cur_vehicle_state_process_.linear_acceleration;
  if (config_.enable_constanct_velocity_in_predicted_vehicle_state) {
    cur_vehicle_state_process_.linear_acceleration = 0.0;
  }

  VehicleState predicted_vehicle_state;
  predicted_vehicle_state = common::VehicleModel::Predict(
      localization_latency_s, cur_vehicle_state_process_);
  cur_vehicle_state_process_.linear_acceleration = cur_vehi_acc;
  cur_vehicle_state_process_ = predicted_vehicle_state;
}

void EgoStateManager::LateralInitStateReset(const PncTrajectoryPoint &point) {
  auto &lat_init_state = planning_init_point_.lat_init_state;
  lat_init_state.set_x(point.path_point.x());
  lat_init_state.set_y(point.path_point.y());
  lat_init_state.set_theta(point.path_point.theta());
  // TODO: need estimated delta and omega for large curv condition
  if (!enable_delta_stitch_in_replan_) {
    lat_init_state.set_delta(point.delta);
  }
  lat_init_state.set_curv(point.path_point.kappa());
  lat_init_state.set_d_curv(0.0);
}

void EgoStateManager::LongitudinalInitStateReset(
    const PncTrajectoryPoint &point) {
  auto &lon_init_state = planning_init_point_.lon_init_state;
  lon_init_state.set_s(0.0);
  lon_init_state.set_v(point.v);
  lon_init_state.set_a(point.a);
  lon_init_state.set_j(point.jerk);
}

// void EgoStateManager::LateralInitStateResetToEgoState() {
//   const auto &ego_state =
//       session_->environmental_model().get_ego_state_manager();
//   const auto &vehicle_param =
//       VehicleConfigurationContext::Instance()->get_vehicle_param();
//   // double steer_ratio = vehicle_param.steer_ratio;

//   auto &lat_init_state = planning_init_point_.lat_init_state;

//   lat_init_state.set_x(ego_state->ego_pose_raw().x);
//   lat_init_state.set_y(ego_state->ego_pose_raw().y);
//   lat_init_state.set_theta(ego_state->ego_pose_raw().theta);

//   // TODO: need estimated delta and omega for large curv condition
//   lat_init_state.set_delta(ego_state->ego_steer_angle() / steer_ratio_);
//   lat_init_state.set_curv(curve_factor * lat_init_state.delta());
//   lat_init_state.set_d_curv(0.0);
// }

// void EgoStateManager::LongitudinalInitStateResetToEgoState() {
//   const auto &ego_state =
//       session_->environmental_model().get_ego_state_manager();
//   auto &lon_init_state = planning_init_point_.lon_init_state;

//   // s is fakely frenet, cannot be obtained
//   lon_init_state.set_s(0.0);
//   lon_init_state.set_v(ego_state->ego_v());
//   auto start_stop_state =
//       session_->planning_context().start_stop_result().state();
//   // deal with ego_acc which has noise
//   if (start_stop_state == common::StartStopInfo::CRUISE) {
//     lon_init_state.set_a(ego_state->ego_acc());
//   } else if (start_stop_state == common::StartStopInfo::START) {
//     lon_init_state.set_a(std::max(0.0, ego_state->ego_acc()));
//   } else if (start_stop_state == common::StartStopInfo::STOP) {
//     lon_init_state.set_a(std::min(0.0, ego_state->ego_acc()));
//   }
// }

// uint8_t EgoStateManager::ReplanProcess(const bool &lat_reset_flag,
//                                        const bool &lon_reset_flag) {
//   // note that lon_reset_flag and lat_reset_flag reserved for acc and
//   override

//   const auto &vehicle_param =
//       VehicleConfigurationContext::Instance()->get_vehicle_param();
//   const auto &ego_state =
//       session_->environmental_model().get_ego_state_manager();
//   auto &motion_planner_output =
//       session_->mutable_planning_context()->mutable_motion_planner_output();
//   double steer_ratio = vehicle_param.steer_ratio;
//   // const auto &traj_points =
//   //
//   session_->mutable_planning_context()->mutable_planning_result().traj_points;

//   auto &lat_init_state = planning_init_point_.lat_init_state;
//   auto &lon_init_state = planning_init_point_.lon_init_state;
//   bool lat_replan = false;
//   bool lon_replan = false;
//   bool dist_replan = false;

//   Eigen::Vector2d cur_pos(ego_state->ego_pose_raw().x,
//                           ego_state->ego_pose_raw().y);
//   pnc::spline::Projection projection_spline;
//   projection_spline.CalProjectionPoint(
//       motion_planner_output.x_s_spline, motion_planner_output.y_s_spline,
//       motion_planner_output.s_lat_vec.front(),
//       motion_planner_output.s_lat_vec.back(), cur_pos);

//   const auto &lat_err = projection_spline.GetOutput().dist_proj;
//   const auto &s_proj = projection_spline.GetOutput().s_proj;
//   const auto &proj_point = projection_spline.GetOutput().point_proj;
//   Eigen::Vector2d init_point(lat_init_state.x(), lat_init_state.y());
//   // TODO: maybe more solid
//   // projection_spline.CalProjectionPoint(motion_planner_output.x_s_spline,
//   // motion_planner_output.y_s_spline,
//   // motion_planner_output.s_lat_vec.front(),
//   // motion_planner_output.s_lat_vec.back(),
//   //                                      init_point);
//   // const auto s_init = projection_spline.GetOutput().s_proj;
//   // const double &lon_err = s_init - s_proj;
//   const double lat_init_theta = lat_init_state.theta();
//   double theta_err = lat_init_theta - ego_state->ego_pose_raw().theta;
//   const double pi2 = 2.0 * M_PI;
//   if (theta_err > M_PI) {
//     lat_init_state.set_theta(lat_init_theta - pi2);
//     theta_err -= pi2;
//   } else if (theta_err < -M_PI) {
//     lat_init_state.set_theta(lat_init_theta + pi2);
//     theta_err += pi2;
//   }
//   const auto lon_err = std::hypot(init_point.x() - proj_point.x(),
//                                   init_point.y() - proj_point.y());
//   const double dist_err =
//       std::hypot(lat_init_state.x() - ego_state->ego_pose_raw().x,
//                  lat_init_state.y() - ego_state->ego_pose_raw().y);

//   JSON_DEBUG_VALUE("lat_err", lat_err)
//   JSON_DEBUG_VALUE("theta_err", theta_err)
//   JSON_DEBUG_VALUE("lon_err", lon_err)
//   JSON_DEBUG_VALUE("dist_err", dist_err)

//   double max_replan_lat_err = max_replan_lat_err_;
//   double max_replan_theta_err = max_replan_theta_err_ / 57.3;
//   double max_replan_lon_err = max_replan_lon_err_;
//   double max_replan_dist_err = max_replan_dist_err_;
//   if (session_->is_hpp_scene()) {
//     max_replan_lat_err = hpp_max_replan_lat_err_;
//     max_replan_theta_err = hpp_max_replan_theta_err_ / 57.3;
//     max_replan_lon_err = hpp_max_replan_lon_err_;
//     max_replan_dist_err = hpp_max_replan_dist_err_;
//   }

//   if (fabs(lat_err) > max_replan_lat_err || lat_reset_flag ||
//       fabs(theta_err) > max_replan_theta_err) {
//     lat_replan = true;
//   }

//   bool low_speed_replan = (ego_state->ego_v() < config_.kEpsilon_v); /*&&
//                                (ego_state->ego_acc() < config_.kEpsilon_a);*/
//   if (fabs(lon_err) > max_replan_lon_err || lon_reset_flag ||
//       low_speed_replan) {
//     lon_replan = true;
//   }

//   // if (fabs(dist_err) > max_replan_dist_err) {
//   //   dist_replan = true;
//   // }

//   uint8_t out = 0;

//   if (lat_replan) {
//     // update lat init state
//     lat_init_state.set_x(ego_state->ego_pose_raw().x);
//     lat_init_state.set_y(ego_state->ego_pose_raw().y);
//     lat_init_state.set_theta(ego_state->ego_pose().theta);
//     lat_init_state.set_delta(ego_state->ego_steer_angle() / steer_ratio_);

//     // lon use stitch result when lat replan
//     out = ReplanStatus::LAT_REPLAN;
//     motion_planner_output.lat_init_flag = false;
//   }

//   auto start_stop_state =
//       session_->planning_context().start_stop_result().state();
//   if (lon_replan) {
//     // update lat init state
//     lat_init_state.set_x(motion_planner_output.x_s_spline(s_proj));
//     lat_init_state.set_y(motion_planner_output.y_s_spline(s_proj));

//     // update lon init state
//     lon_init_state.set_s(s_proj);
//     // deal with ego_acc which has noise
//     if (start_stop_state == common::StartStopInfo::CRUISE) {
//       lon_init_state.set_a(ego_state->ego_acc());
//     } else if (start_stop_state == common::StartStopInfo::START) {
//       lon_init_state.set_a(std::max(0.0, ego_state->ego_acc()));
//     } else if (start_stop_state == common::StartStopInfo::STOP) {
//       lon_init_state.set_a(std::min(0.0, ego_state->ego_acc()));
//     }

//     if (!session_->is_hpp_scene()) {
//       if (lon_init_state.v() - ego_state->ego_v() > 1.0) {
//         lon_init_state.set_v(ego_state->ego_v());
//       }
//     } else {
//       if (lon_init_state.v() - ego_state->ego_v() > 0.5) {
//         lon_init_state.set_v(ego_state->ego_v());
//       }
//     }

//     // lon_init_state.set_a(ego_state->ego_acc());
//     out = ReplanStatus::LON_REPLAN;
//   }

//   if (dist_replan || (lon_replan && lat_replan)) {
//     LateralReset();
//     LongitudinalReset();

//     motion_planner_output.lat_init_flag = false;
//     out = ReplanStatus::LAT_REPLAN + ReplanStatus::LON_REPLAN;
//     // a and j use stitch result
//   }

//   return out;
// }

void EgoStateManager::MotionPlanningInfoReset() {
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();

  if (!session_->environmental_model().location_valid()) {
    motion_planner_output.lat_enable_flag = false;
    motion_planner_output.lon_enable_flag = false;
  }
}

bool EgoStateManager::LateralStitch() {
  auto &lat_init_state = planning_init_point_.lat_init_state;
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();

  if (motion_planner_output.lat_enable_flag) {
    // note that s is only for lateral path rather than frenet
    // const double s = motion_planner_output.s_t_spline(planning_loop_dt);

    // max delta as equivalent steer angle = 120 deg
    double max_delta = 120 / 57.3 / steer_ratio_;
    if (ego_v_ < 10.0) {
      max_delta = max_delta_;
    }

    lat_init_state.set_x(motion_planner_output.x_t_spline(planning_loop_dt));
    lat_init_state.set_y(motion_planner_output.y_t_spline(planning_loop_dt));
    lat_init_state.set_theta(
        motion_planner_output.theta_t_spline(planning_loop_dt));
    lat_init_state.set_delta(pnc::mathlib::Limit(
        motion_planner_output.delta_t_spline(planning_loop_dt), max_delta));
    lat_init_state.set_curv(curve_factor * lat_init_state.delta());
    lat_init_state.set_d_curv(0.0);

    return true;
  } else {
    return false;
  }
}

bool EgoStateManager::LongitudinalStitch() {
  auto &lon_init_state = planning_init_point_.lon_init_state;
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();

  if (motion_planner_output.lon_enable_flag) {
    // note that longtitudinal s is in frenet, but lateral s is not in frenet
    // so set zero here and set right value in longitudinal planning
    lon_init_state.set_s(0.0);
    lon_init_state.set_v(
        std::max(motion_planner_output.v_t_spline(planning_loop_dt), 0.0));
    lon_init_state.set_a(motion_planner_output.a_t_spline(planning_loop_dt));
    lon_init_state.set_j(motion_planner_output.j_t_spline(planning_loop_dt));
    return true;
  } else {
    return false;
  }
}

void EgoStateManager::RealtimeUpdatePlanningInitState() {
  // lateral motion never replans, lateral stitch obeys that (x,y,theta) always
  // apply current pose, delta uses current delta

  auto &lat_init_state = planning_init_point_.lat_init_state;
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  lat_init_state.set_x(0.0);
  lat_init_state.set_y(0.0);
  lat_init_state.set_theta(0.0);
  lat_init_state.set_delta(ego_state->ego_steer_angle() / steer_ratio_);

  // longitudinal stitch: ignore s, but v & a really stitch
  auto &lon_init_state = planning_init_point_.lon_init_state;
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();
  const double vel_ego = ego_state->ego_v();

  if (motion_planner_output.lon_enable_flag) {
    double vel_stitch =
        std::max(motion_planner_output.v_t_spline(planning_loop_dt), 0.0);
    double acc_stitch = motion_planner_output.a_t_spline(planning_loop_dt);

    const double vel_err = vel_stitch - vel_ego;

    planning_init_point_.lon_pos_err += vel_err * planning_loop_dt;

    // longitudinal motion replans due to large lon_pos_err
    if (fabs(planning_init_point_.lon_pos_err) > 1.5) {
      planning_init_point_.lon_pos_err = 0.0;

      vel_stitch = vel_ego;
    }

    lon_init_state.set_s(0.0);
    lon_init_state.set_v(vel_stitch);
    lon_init_state.set_a(acc_stitch);
    lon_init_state.set_j(0.0);
  } else {
    lon_init_state.set_s(0.0);
    lon_init_state.set_v(vel_ego);
    lon_init_state.set_a(0.0);
    lon_init_state.set_j(0.0);
  }

  planning_init_point_.x = lat_init_state.x();
  planning_init_point_.y = lat_init_state.y();
  planning_init_point_.heading_angle = lat_init_state.theta();
  planning_init_point_.curvature = lat_init_state.curv();
  planning_init_point_.dkappa = lat_init_state.d_curv();

  planning_init_point_.v = lon_init_state.v();
  planning_init_point_.a = lon_init_state.a();
  planning_init_point_.jerk = lon_init_state.j();

  planning_init_point_.relative_time = 0.0;

  auto &mutable_motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  mutable_motion_planner_output.lat_init_flag = false;
}

void EgoStateManager::UpdatePlanningInitState() {
  bool stitch_success = false;
  uint8_t replan_status = 0;

  // reset trajectory spline
  MotionPlanningInfoReset();

  // compensate ego state because of localization latency
  CompensateEgoStateForLocalizationLatency();

  // stitch process
  if (LateralStitch() && LongitudinalStitch()) {
    stitch_success = true;

    // replan process
    bool set_lat_replan = false;
    bool set_lon_replan = false;

    auto cur_fsm_state = session_->environmental_model()
                             .get_local_view()
                             .function_state_machine_info.current_state;
    if (!session_->environmental_model().GetVehicleDbwStatus()) {
      set_lat_replan = true;
      set_lon_replan = true;
    } else if (cur_fsm_state == iflyauto::FunctionalState_ACC_ACTIVATE) {
      set_lat_replan = true;
    } else if (cur_fsm_state == iflyauto::FunctionalState_ACC_OVERRIDE ||
               cur_fsm_state == iflyauto::FunctionalState_RADS_SUSPEND) {
      set_lat_replan = true;
      set_lon_replan = true;
    } else if (cur_fsm_state == iflyauto::FunctionalState_SCC_OVERRIDE ||
               cur_fsm_state == iflyauto::FunctionalState_NOA_OVERRIDE) {
      set_lon_replan = true;
    }
    replan_type_.clear();
    replan_status = ReplanProcess(set_lat_replan, set_lon_replan);
  } else {
    stitch_success = false;
    enable_delta_stitch_in_replan_ = false;
    PncTrajectoryPoint reinit_point;
    reinit_point = TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
        cur_vehicle_state_process_);
    LateralInitStateReset(reinit_point);
    LongitudinalInitStateReset(reinit_point);
    enable_delta_stitch_in_replan_ = config_.enable_delta_stitch_in_replan;
    replan_status = LAT_lON_REST;
  }

  JSON_DEBUG_VALUE("replan_status", replan_status)
  JSON_DEBUG_VALUE("stitch_success", stitch_success)

  // assebling init state
  const auto &lat_init_state = planning_init_point_.lat_init_state;
  const auto &lon_init_state = planning_init_point_.lon_init_state;

  planning_init_point_.x = lat_init_state.x();
  planning_init_point_.y = lat_init_state.y();
  planning_init_point_.heading_angle = lat_init_state.theta();
  planning_init_point_.curvature = lat_init_state.curv();
  planning_init_point_.dkappa = lat_init_state.d_curv();
  planning_init_point_.v = lon_init_state.v();
  planning_init_point_.a = lon_init_state.a();
  planning_init_point_.jerk = lon_init_state.j();

  planning_init_point_.relative_time = 0.0;

  if (replan_status > 0) {
    auto &motion_planner_output =
        session_->mutable_planning_context()->mutable_motion_planner_output();
    motion_planner_output.lat_init_flag = false;
  }
}

}  // namespace planning
