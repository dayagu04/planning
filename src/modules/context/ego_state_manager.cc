#include "modules/context/ego_state_manager.h"
#include "modules/common/trajectory/trajectory_stitcher.h"
#include "src/framework/session.h"
// #include "src/modules/task/task.h"

namespace planning {

EgoStateManager::EgoStateManager(planning::framework::Session *session) {
  session_ = session;
  vehicle_param_ = session_->vehicel_config_context().get_vehicle_param();
  // environmental_model_ = environmental_model;
}

void EgoStateManager::update() {

  assert(session_ != nullptr);

  auto cart_ego_state =
      session_->mutable_environmental_model()->get_cart_ego_state_manager()->get_cart_ego_state();
  navi_timestamp_ = cart_ego_state.ego_navi_timestamp();
  x_ = cart_ego_state.ego_pose().x();
  y_ = cart_ego_state.ego_pose().y();
  velocity_ = cart_ego_state.ego_vel();
  heading_angle_ = cart_ego_state.ego_pose().theta();
  acc_ = cart_ego_state.ego_acc();
  ego_steer_angle_ = cart_ego_state.ego_steer_angle();
  ego_v_cruise_ = cart_ego_state.ego_v_cruise();
  ego_t_distance_ = cart_ego_state.ego_t_distance();
  ego_start_stop_ = cart_ego_state.ego_start_stop();
  ego_throttle_override_ = cart_ego_state.throttle_override();

  // polygon
  auto center = planning_math::Vec2d(x_, y_);
  auto box2d = planning_math::Box2d(
      center, heading_angle_, vehicle_param_.length, vehicle_param_.width);
  polygon_ = planning_math::Polygon2d(box2d);

  // planning start point
  // update_planning_init_point();

  // update transform
  Eigen::Vector4d q;
  q.x() = cart_ego_state.ego_enu().orientation().x();
  q.y() = cart_ego_state.ego_enu().orientation().y();
  q.z() = cart_ego_state.ego_enu().orientation().z();
  q.w() = cart_ego_state.ego_enu().orientation().w();
  Eigen::Vector3d v;
  v.x() = cart_ego_state.ego_enu().position().x();
  v.y() = cart_ego_state.ego_enu().position().y();
  v.z() = cart_ego_state.ego_enu().position().z();

  car2enu_ = define::Transform(q, v);
  enu2car_ = define::Transform(q, v).inverse();
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
  vehicle_state.x = x_;
  vehicle_state.y = y_;
  vehicle_state.yaw = heading_angle_;
  vehicle_state.linear_acceleration = acc_;
  vehicle_state.linear_velocity = velocity_;
  vehicle_state.kappa = tan(ego_steer_angle_ / vehicle_param_.steer_ratio) /
                        vehicle_param_.wheel_base;
  vehicle_state.heading = heading_angle_;
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
       pnc_planning_status->planning_result.timestamp);  //TODO :暂时删掉。sec()
  vehicle_state.timestamp = init_point_relative_time;
  LOG_DEBUG("init_point_relative_time: %f", init_point_relative_time);

  // TODO
  // auto stitch_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory()

  return stitch_trajectory;
}

}  // namespace planning
