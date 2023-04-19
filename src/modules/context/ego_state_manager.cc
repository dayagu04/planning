#include "modules/context/ego_state_manager.h"
#include "modules/common/utils/pose2d_utils.h"
#include "modules/common/trajectory/trajectory_stitcher.h"

namespace planning {

EgoStateManager::EgoStateManager(planning::framework::Session *session)
    : session_(session) {
  vehicle_param_ = session_->vehicel_config_context().get_vehicle_param();
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
  update_planning_init_point();
  return true;
}

void EgoStateManager::update_planning_init_point() {
  stitch_trajectory_ = compute_stitching_trajectory();
  PncTrajectoryPoint init_point = stitch_trajectory_.back();
  planning_init_point_.x = init_point.path_point.x;
  planning_init_point_.y = init_point.path_point.y;
  planning_init_point_.heading_angle = init_point.path_point.theta;
  planning_init_point_.curvature = init_point.path_point.kappa;
  planning_init_point_.dkappa = init_point.path_point.dkappa;
  // planning_init_point_.v = std::min(init_point.v, velocity_);
  // planning_init_point_.a = std::min(init_point.a, acc_);
  planning_init_point_.v = init_point.v;
  planning_init_point_.a = init_point.a;
  planning_init_point_.jerk = jerk_;
  planning_init_point_.relative_time = init_point.relative_time;
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
