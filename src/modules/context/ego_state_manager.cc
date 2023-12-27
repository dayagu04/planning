#include "ego_state_manager.h"

#include <algorithm>
#include <cmath>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "log.h"
#include "math_lib.h"
#include "planning_context.h"
#include "planning_output_context.h"
#include "spline_projection.h"
#include "trajectory/trajectory_stitcher.h"
#include "utils/pose2d_utils.h"

namespace planning {

static const double planning_loop_dt = 0.1;
static const double steer_ratio = 15.7;
static const double curve_factor = 0.30;

EgoStateManager::EgoStateManager(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  vehicle_param_ = session_->vehicle_config_context().get_vehicle_param();
  config_ = config_builder->cast<EgoPlanningEgoStateManagerConfig>();
  parking_cruise_speed_ = config_.parking_cruise_speed;
  // init v_cruise_filter: -1.5m/s2, 1.5m/s2, 0-150km/h, 10hz
  v_cruise_filter_.Init(-1.5, 1.5, 0.0, 42.0, planning_loop_dt);
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

  // ego_pose_.x = ego_pose_.x -
  //   std::cos(ego_pose_.theta) * vehicle_param_.length / 2.0;
  // ego_pose_.y = ego_pose_.y -
  //   std::sin(ego_pose_.theta) * vehicle_param_.length / 2.0;

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
    ego_v_cruise_ = parking_cruise_speed_;
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
  if (timestamp_us_ == timestamp_us_last_) {
    jerk_ = 0;
  } else {
    jerk_ = (ego_acc_ - ego_acc_last_) /
            ((timestamp_us_ - timestamp_us_last_) / 1000000.0);
  }

  planning_math::Vec2d center(
      ego_pose_.x +
          std::cos(ego_pose_.theta) * vehicle_param_.rear_axis_to_center,
      ego_pose_.y +
          std::sin(ego_pose_.theta) * vehicle_param_.rear_axis_to_center);
  planning_math::Box2d ego_box(center, ego_pose_.theta, vehicle_param_.length,
                               vehicle_param_.width);
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

uint8_t EgoStateManager::ReplanProcess(const bool &lat_reset_flag,
                                       const bool &lon_reset_flag) {
  // note that lon_reset_flag and lat_reset_flag reserved for acc and override

  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  auto &motion_planning_info = session_->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;

  // const auto &traj_points =
  // session_->mutable_planning_context()->mutable_planning_result().traj_points;

  auto &lat_init_state = planning_init_point_.lat_init_state;
  auto &lon_init_state = planning_init_point_.lon_init_state;
  bool lat_replan = false;
  bool lon_replan = false;
  bool dist_replan = false;

  Eigen::Vector2d cur_pos(ego_state->ego_pose_raw().x,
                          ego_state->ego_pose_raw().y);

  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(
      motion_planning_info.x_s_spline, motion_planning_info.y_s_spline,
      motion_planning_info.s_lat_vec.front(),
      motion_planning_info.s_lat_vec.back(), cur_pos);

  const auto &lat_err = projection_spline.GetOutput().dist_proj;
  const auto &s_proj = projection_spline.GetOutput().s_proj;
  const auto &proj_point = projection_spline.GetOutput().point_proj;
  Eigen::Vector2d init_point(lat_init_state.x(), lat_init_state.y());
  // TODO: maybe more solid
  // projection_spline.CalProjectionPoint(motion_planning_info.x_s_spline,
  // motion_planning_info.y_s_spline,
  //                                      motion_planning_info.s_lat_vec.front(),
  //                                      motion_planning_info.s_lat_vec.back(),
  //                                      init_point);
  // const auto s_init = projection_spline.GetOutput().s_proj;
  // const double &lon_err = s_init - s_proj;

  const auto lon_err = std::hypot(init_point.x() - proj_point.x(),
                                  init_point.y() - proj_point.y());
  const double dist_err =
      std::hypot(lat_init_state.x() - ego_state->ego_pose_raw().x,
                 lat_init_state.y() - ego_state->ego_pose_raw().y);

  JSON_DEBUG_VALUE("lat_err", lat_err)
  JSON_DEBUG_VALUE("lon_err", lon_err)
  JSON_DEBUG_VALUE("dist_err", dist_err)

  double max_replan_lat_err = 0.6;
  double max_replan_lon_err = 1.0;
  double max_replan_dist_err = 1.5;

  if (session_->is_hpp_scene()) {
    max_replan_lat_err = 0.4;
    max_replan_lon_err = 0.5;
    max_replan_dist_err = 0.75;
  }

  if (fabs(lat_err) > max_replan_lat_err || lat_reset_flag) {
    lat_replan = true;
  }

  if (fabs(lon_err) > max_replan_lon_err || lon_reset_flag) {
    lon_replan = true;
  }

  if (fabs(dist_replan) > max_replan_dist_err) {
    dist_replan = true;
  }

  uint8_t out = 0;

  if (lat_replan) {
    // update lat init state
    lat_init_state.set_x(ego_state->ego_pose_raw().x);
    lat_init_state.set_y(ego_state->ego_pose_raw().y);
    lat_init_state.set_theta(ego_state->ego_pose().theta);
    lat_init_state.set_delta(ego_state->ego_steer_angle() / steer_ratio);

    // lon use stitch result when lat replan
    out = ReplanStatus::LAT_REPLAN;
    motion_planning_info.lat_init_flag = false;
  }

  if (lon_replan) {
    // update lat init state
    lat_init_state.set_x(motion_planning_info.x_s_spline(s_proj));
    lat_init_state.set_y(motion_planning_info.y_s_spline(s_proj));

    // update lon init state
    lon_init_state.set_s(0.0);
    if (lon_init_state.v() - ego_state->ego_v() > 3.0) {
      lon_init_state.set_v(ego_state->ego_v());
    }

    // lon_init_state.set_a(ego_state->ego_acc());
    out = ReplanStatus::LON_REPLAN;
  }

  if (dist_replan || (lon_replan && lat_replan)) {
    LateralReset();
    LongitudinalReset();

    motion_planning_info.lat_init_flag = false;
    out = ReplanStatus::LAT_REPLAN + ReplanStatus::LON_REPLAN;
    // a and j use stitch result
  }

  return out;
}

void EgoStateManager::LateralReset() {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();

  auto &lat_init_state = planning_init_point_.lat_init_state;

  lat_init_state.set_x(ego_state->ego_pose_raw().x);
  lat_init_state.set_y(ego_state->ego_pose_raw().y);
  lat_init_state.set_theta(ego_state->ego_pose_raw().theta);

  // TODO: need estimated delta and omega for large curv condition
  lat_init_state.set_delta(ego_state->ego_steer_angle() / steer_ratio);
  lat_init_state.set_curv(curve_factor * lat_init_state.delta());
  lat_init_state.set_d_curv(0.0);
}

void EgoStateManager::LongitudinalReset() {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  auto &lon_init_state = planning_init_point_.lon_init_state;

  // s is fakely frenet, cannot be obtained
  lon_init_state.set_s(0.0);
  lon_init_state.set_v(ego_state->ego_v());
  lon_init_state.set_a(0.0);
}

void EgoStateManager::MotionPlanningInfoReset() {
  auto &motion_planning_info = session_->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;

  if (!session_->environmental_model().location_valid()) {
    motion_planning_info.lat_enable_flag = false;
    motion_planning_info.lon_enable_flag = false;
  }
}

bool EgoStateManager::LateralStitch() {
  auto &lat_init_state = planning_init_point_.lat_init_state;
  const auto &motion_planning_info = session_->mutable_planning_context()
                                         ->mutable_planning_result()
                                         .motion_planning_info;

  if (motion_planning_info.lat_enable_flag) {
    // note that s is only for lateral path rather than frenet
    // const double s = motion_planning_info.s_t_spline(planning_loop_dt);

    // max delta as equivalent steer angle = 120 deg
    double max_delta = 120.0 / 57.3 / 15.7;
    if (session_->is_hpp_scene()) {
      max_delta = 470.0 / 57.3 / 15.7; 
    }

    lat_init_state.set_x(motion_planning_info.x_t_spline(planning_loop_dt));
    lat_init_state.set_y(motion_planning_info.y_t_spline(planning_loop_dt));
    lat_init_state.set_theta(
        motion_planning_info.theta_t_spline(planning_loop_dt));
    lat_init_state.set_delta(pnc::mathlib::Limit(
        motion_planning_info.delta_t_spline(planning_loop_dt), max_delta));
    lat_init_state.set_curv(curve_factor * lat_init_state.delta());
    lat_init_state.set_d_curv(0.0);

    return true;
  } else {
    return false;
  }
}

bool EgoStateManager::LongitudinalStitch() {
  auto &lon_init_state = planning_init_point_.lon_init_state;
  const auto &motion_planning_info = session_->mutable_planning_context()
                                         ->mutable_planning_result()
                                         .motion_planning_info;

  if (motion_planning_info.lon_enable_flag) {
    // note that longtitudinal s is in frenet, but lateral s is not in frenet
    // so set zero here and set right value in longitudinal planning
    lon_init_state.set_s(0.0);
    lon_init_state.set_v(
        std::max(motion_planning_info.v_t_spline(planning_loop_dt), 0.0));
    lon_init_state.set_a(motion_planning_info.a_t_spline(planning_loop_dt));
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
  lat_init_state.set_delta(ego_state->ego_steer_angle() / steer_ratio);

  // longitudinal stitch: ignore s, but v & a really stitch
  auto &lon_init_state = planning_init_point_.lon_init_state;
  const auto &motion_planning_info = session_->mutable_planning_context()
                                         ->mutable_planning_result()
                                         .motion_planning_info;
  const double vel_ego = ego_state->ego_v();

  if (motion_planning_info.lon_enable_flag) {
    double vel_stitch =
        std::max(motion_planning_info.v_t_spline(planning_loop_dt), 0.0);
    double acc_stitch = motion_planning_info.a_t_spline(planning_loop_dt);

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
}

void EgoStateManager::UpdatePlanningInitState() {
  bool stitch_success = false;
  uint8_t replan_status = 0;

  // reset trajectory spline
  MotionPlanningInfoReset();

  // stitch process
  if (LateralStitch() && LongitudinalStitch()) {
    stitch_success = true;

    // replan process
    bool set_lat_replan = false;
    bool set_lon_replan = false;

    // TODO: acc should be considered here
    if (!session_->environmental_model().GetVehicleDbwStatus()) {
      set_lat_replan = true;
      set_lon_replan = true;
    }

    replan_status = ReplanProcess(set_lat_replan, set_lon_replan);
  } else {
    stitch_success = false;
    LateralReset();
    LongitudinalReset();

    replan_status = ReplanStatus::LAT_REPLAN + ReplanStatus::LON_REPLAN;
  }

  JSON_DEBUG_VALUE("replan_status", replan_status)
  JSON_DEBUG_VALUE("stitch_success", stitch_success)
  JSON_DEBUG_VALUE("dbw_status",
                   session_->environmental_model().GetVehicleDbwStatus())

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
}

}  // namespace planning
