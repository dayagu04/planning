#include "context/ego_state_manager.h"
#include "common/utils/pose2d_utils.h"
#include "common/trajectory/trajectory_stitcher.h"
#include "debug_info_log.h"
#include "math_lib.h"
#include "spline_projection.h"
#include <cmath>
#include <cstdint>

namespace planning {

static const double planning_loop_dt = 0.1;

EgoStateManager::EgoStateManager(planning::framework::Session *session)
    : session_(session) {
  vehicle_param_ = session_->vehicle_config_context().get_vehicle_param();
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

  ego_pose_.x = ego_pose_.x -
    std::cos(ego_pose_.theta) * vehicle_param_.length / 2.0;
  ego_pose_.y = ego_pose_.y -
    std::sin(ego_pose_.theta) * vehicle_param_.length / 2.0;

  ego_v_ = vehicle_status.velocity().heading_velocity().value_mps();
  ego_v_angle_ = vehicle_status.heading_yaw().heading_yaw_data().value_rad();
  ego_hmi_v_ = vehicle_status.velocity().hmi_speed();
}

void EgoStateManager::set_ego_position_llh(const planning::common::VehicleStatus &vehicle_status){
  const auto &location_geographic = vehicle_status.location().location_geographic();
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
  ego_steer_angle_ = vehicle_status.steering_wheel().steering_wheel_data()
                                    .steering_wheel_rad();
}

void EgoStateManager::set_ego_acc(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_acc_last_ = ego_acc_;
  ego_acc_ = vehicle_status.brake_info().brake_info_data()
                            .acceleration_on_vehicle_wheel();
}

void EgoStateManager::set_ego_v_cruise(
    const planning::common::VehicleStatus &vehicle_status) {
  ego_v_cruise_ = vehicle_status.velocity().cruise_velocity().value_mps();
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
  ego_blinker_ = vehicle_status.vehicle_light().vehicle_light_data()
                                .turn_signal().value();
}

void EgoStateManager::set_ego_blinker(
    const planning::common::VehicleLight &vehicle_light) {
  ego_blinker_ =  vehicle_light.vehicle_light_data().turn_signal().value();
}

void EgoStateManager::set_ego_auto_light_state(const planning::common::VehicleStatus &vehicle_status) {
  ego_auto_light_state_ = vehicle_status.vehicle_light().vehicle_light_data().auto_light_state();
}

void EgoStateManager::set_driver_hand_state(
    const planning::common::VehicleStatus &vehicle_status) {
  driver_hand_torque_ =  vehicle_status.driver_hand_state().driver_hand_torque();
  driver_hands_off_state_ = vehicle_status.driver_hand_state().driver_hands_off_state();
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

void EgoStateManager::set_timestamp_us(const planning::common::VehicleStatus &vehicle_status) {
  timestamp_us_last_ = timestamp_us_;
  timestamp_us_ = vehicle_status.header().timestamp_us();
}
bool EgoStateManager::update(const planning::common::VehicleStatus &vehicle_status) {

  set_timestamp_us(vehicle_status);
  set_ego_position_llh(vehicle_status);
  set_ego_enu(vehicle_status);
  set_ego_pose_and_vel(vehicle_status);
  set_ego_carte(convert_pose2point(ego_pose_));
  //set_ego_prediction_info(vehicle_status);
  set_ego_steer_angle(vehicle_status);
  set_ego_acc(vehicle_status);
  set_ego_v_cruise(vehicle_status);
  set_ego_t_distance(vehicle_status);
  set_ego_start_stop(vehicle_status);
  set_throttle_override(vehicle_status);
  set_ego_blinker(vehicle_status);
  set_ego_auto_light_state(vehicle_status);
  set_driver_hand_state(vehicle_status);
  if (timestamp_us_ == timestamp_us_last_) {
    jerk_ = 0;
  } else {
    jerk_ = (ego_acc_ - ego_acc_last_) / ((timestamp_us_ - timestamp_us_last_) / 1000000.0);
  }

  planning_math::Vec2d center(ego_pose_.x, ego_pose_.y);
  planning_math::Box2d ego_box(center, ego_pose_.theta, vehicle_param_.length,
                               vehicle_param_.width);
  polygon_ = planning_math::Polygon2d(ego_box);

  update_transform();

  // planning start point
  UpdatePlanningInitState();
  return true;
}

uint8_t EgoStateManager::ReplanProcess(bool lat_replan, bool lon_replan) {
  const auto &ego_state =
    session_->environmental_model().get_ego_state_manager();
  const auto &traj_spline = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_spline;

  const auto &traj_points = session_->mutable_planning_context()
      ->mutable_planning_result().traj_points;

  auto &lat_init_state = planning_init_point_.lat_init_state;
  auto &lon_init_state = planning_init_point_.lon_init_state;

  Eigen::Vector2d cur_pos(ego_state->ego_pose().x, ego_state->ego_pose().y);

  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(traj_spline.x_s_spline, traj_spline.y_s_spline,
  traj_spline.s_lat_vec.front(), traj_spline.s_lat_vec.back(), cur_pos);

  const double &lat_err = projection_spline.GetOutput().dist_proj;

  // FBI WARNING
  // const double ds = ego_state->ego_v() * 0.8 * planning_loop_dt;
  const double ds = traj_points.front().v * planning_loop_dt;

  const double &lon_err = projection_spline.GetOutput().s_proj - (traj_spline.s_lat_vec.front() + ds);

  uint8_t out = 0;

  // lateral replan
  if (lat_err > 0.6 || lat_replan) {
    // when lateral replan, delta and omega use stitch result
    lat_init_state.set_x(ego_state->ego_pose().x);
    lat_init_state.set_y(ego_state->ego_pose().y);
    lat_init_state.set_theta(traj_spline.theta_s_spline(projection_spline.GetOutput().s_proj));

    out += ReplanStatus::LAT_REPLAN;
  }

  // longitudinal replan
  if (lon_err > 1.0 || lon_replan) {
    lat_init_state.set_x(projection_spline.GetOutput().point_proj.x());
    lat_init_state.set_y(projection_spline.GetOutput().point_proj.y());
    lat_init_state.set_theta(traj_spline.theta_s_spline(projection_spline.GetOutput().s_proj));

    // s is fakely frenet, cannot be obtained
    lon_init_state.set_s(0.0);

    // when longitudinal replan, acc and jerk use stitch result
    lon_init_state.set_v(ego_state->ego_v());

    out += ReplanStatus::LON_REPLAN;
  }

  return out;
}

void EgoStateManager::LateralReset() {
  const auto &ego_state =
    session_->environmental_model().get_ego_state_manager();

  auto &lat_init_state = planning_init_point_.lat_init_state;

  lat_init_state.set_x(ego_state->ego_pose().x);
  lat_init_state.set_y(ego_state->ego_pose().y);
  lat_init_state.set_theta(ego_state->ego_pose().theta);

  // TODO: need estimated delta and omega for large curv condition
  lat_init_state.set_delta(0.0);
  lat_init_state.set_omega(0.0);
  lat_init_state.set_curv(0.0);
  lat_init_state.set_d_curv(0.0);
}

void EgoStateManager::LongitudinalReset() {
  const auto &ego_state =
    session_->environmental_model().get_ego_state_manager();
  auto &lon_init_state = planning_init_point_.lon_init_state;

  // s is fakely frenet, cannot be obtained
  lon_init_state.set_s(0.0);
  lon_init_state.set_v(ego_state->ego_v());
  lon_init_state.set_a(ego_state->ego_acc());
  lon_init_state.set_j(0.0);
}

bool EgoStateManager::LateralStitch() {
  auto &lat_init_state = planning_init_point_.lat_init_state;
  const auto &traj_spline = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_spline;
  const auto &traj_points = session_->mutable_planning_context()
      ->mutable_planning_result().traj_points;

  if (traj_spline.lat_enable_flag) {
    // note that s is only for lateral path rather than frenet
    // const double s = traj_spline.s_t_spline(planning_loop_dt);

    // FBI WARNING
    // const auto &ego_state =
    // session_->environmental_model().get_ego_state_manager();
    // auto const ds = ego_state->ego_v() * 0.8 * planning_loop_dt;
    const double ds = traj_points.front().v * planning_loop_dt;

    const double s = ds;

    if (pnc::mathlib::IsInBound(s, traj_spline.s_lat_vec.front(),
      traj_spline.s_lat_vec.back())) {
        return false;
      }

    lat_init_state.set_x(traj_spline.x_s_spline(s));
    lat_init_state.set_y(traj_spline.y_s_spline(s));
    lat_init_state.set_theta(traj_spline.theta_s_spline(s));
    lat_init_state.set_delta(traj_spline.delta_s_spline(s));
    lat_init_state.set_omega(traj_spline.omega_s_spline(s));
    lat_init_state.set_curv(traj_spline.curv_s_spline(s));
    lat_init_state.set_d_curv(traj_spline.d_curv_s_spline(s));

    return true;
  } else {
    return false;
  }
}

bool EgoStateManager::LongitudinalStitch() {
  auto &lon_init_state = planning_init_point_.lon_init_state;
  const auto &traj_spline = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_spline;

  if (traj_spline.lon_enable_flag) {
    // s is fakely frenet, cannot be obtained
    lon_init_state.set_s(0.0);
    lon_init_state.set_v(traj_spline.v_t_spline(planning_loop_dt));
    lon_init_state.set_a(traj_spline.a_t_spline(planning_loop_dt));
    lon_init_state.set_j(traj_spline.j_t_spline(planning_loop_dt));
    return true;
  } else {
    return false;
  }
}

void EgoStateManager::UpdatePlanningInitState() {
  bool lat_reset_flag = false;
  bool lon_reset_flag = false;
  uint8_t replan_status = 0;

  // stitch process
  if (!LateralStitch()) {
    LateralReset();
    lat_reset_flag = true;
  }

  if (!LongitudinalStitch()) {
    LongitudinalReset();
    lon_reset_flag = true;
  }

  // replan process
  if (!lat_reset_flag && !lon_reset_flag) {
    // leave a protocal for acc (lat_replan default true)
    replan_status = ReplanProcess(false, false);
  }

  // JSON_DEBUG_VALUE("reset_flag", reset_flag)
  JSON_DEBUG_VALUE("replan_status", replan_status)

  // assebling init state
  auto const &lat_init_state = planning_init_point_.lat_init_state;
  auto const &lon_init_state = planning_init_point_.lon_init_state;

  planning_init_point_.x = lat_init_state.x();
  planning_init_point_.y = lat_init_state.y();
  planning_init_point_.heading_angle = lat_init_state.theta();
  planning_init_point_.curvature = lat_init_state.curv();
  planning_init_point_.dkappa = lat_init_state.d_curv();
  planning_init_point_.v = lon_init_state.v();
  planning_init_point_.a = lon_init_state.a();
  planning_init_point_.jerk = lon_init_state.j();

  planning_init_point_.relative_time = planning_loop_dt;
}

std::vector<PncTrajectoryPoint>
EgoStateManager::compute_stitching_trajectory() {
  // pnc planning_status
  // TODO
  auto* pnc_planning_status =
      session_->mutable_planning_output_context()->mutable_planning_status();
  const auto& last_planning_result =
      session_->mutable_planning_context()->last_planning_result();
  bool dbw_status = session_->environmental_model().GetVehicleDbwStatus();
  bool last_planning_success =
      session_->mutable_planning_context()->last_planning_success();

  const double planning_loop_rate = 10.0;
  const double planning_cycle_time = 1.0 / planning_loop_rate;

  // make vehicle state
  VehicleState vehicle_state;
  vehicle_state.x = ego_pose_.x;
  vehicle_state.y = ego_pose_.y;
  vehicle_state.yaw = ego_pose_.theta;
  vehicle_state.linear_acceleration = ego_acc_;
  vehicle_state.linear_velocity = ego_v_;
  vehicle_state.kappa = tan(ego_steer_angle_ / vehicle_param_.steer_ratio) /
                        vehicle_param_.wheel_base;
  vehicle_state.heading = ego_pose_.theta; //todo
  vehicle_state.driving_mode =
      dbw_status ? DrivingMode::AUTO : DrivingMode::MANUAL;

  // hack
  auto stitch_trajectory =
        TrajectoryStitcher::ComputeReinitStitchingTrajectory(
            planning_cycle_time, vehicle_state);
  // reinit
  auto& replan_trajectory =
      session_->mutable_planning_context()->mutable_replan_trajectory();
  replan_trajectory = false;
  if (dbw_status == false || last_planning_success == false) {
    auto stitch_trajectory =
        TrajectoryStitcher::ComputeReinitStitchingTrajectory(
            planning_cycle_time, vehicle_state);
    replan_trajectory = true;
    jerk_ = 0.0;
    return stitch_trajectory;
  }

  // compute stitch trajectory
  // relative time to last planning trajectory, timeline starts at first point
  // of last planning trajectory
  double init_point_relative_time =
      (pnc_planning_status->planning_result.next_timestamp -
       pnc_planning_status->planning_result.timestamp);// todo .sec();

  vehicle_state.timestamp = init_point_relative_time;
  LOG_DEBUG("init_point_relative_time: %f", init_point_relative_time);

  // TODO
  // auto stitch_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory()

  return stitch_trajectory;
}

}   // planning
