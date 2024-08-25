#include "history_obstacle_manager.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "../../common/planning_gflags.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "utils/frenet_coordinate_system.h"
#include "vehicle_config_context.h"

namespace planning {

HistoryObstacleManager::HistoryObstacleManager(
    const EgoPlanningConfigBuilder *config_builder,
    planning::framework::Session *session) {
  config_ = config_builder->cast<HistoryObstacleConfig>();
  session_ = session;
  planning_loop_dt_ = 1.0 / FLAGS_planning_loop_rate;
  frenet_coord_ = nullptr;
}

HistoryObstacleManager::~HistoryObstacleManager() {
  history_obstacles_.clear();
  old_obstacles_.clear();
  new_obstacles_.clear();
}

bool HistoryObstacleManager::Update() {
  new_obstacles_.clear();
  if (!session_->environmental_model().GetVehicleDbwStatus()) {
    history_obstacles_.clear();
    old_obstacles_.clear();
    return false;
  }
  const auto &planning_result = session_->planning_context().planning_result();
  const auto &last_planning_result =
      session_->planning_context().last_planning_result();
  if (last_planning_result.timestamp > 0) {
    planning_loop_dt_ =
        (planning_result.timestamp - last_planning_result.timestamp) / 1000.0;
  }
  const std::shared_ptr<EgoStateManager> &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const std::shared_ptr<ReferencePathManager> &reference_path =
      session_->environmental_model().get_reference_path_manager();
  const std::shared_ptr<ObstacleManager> &obstacles =
      session_->environmental_model().get_obstacle_manager();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_rear_axle = vehicle_param.front_edge_to_rear_axle;
  frenet_coord_ =
      reference_path->get_reference_path_by_current_lane()->get_frenet_coord();
  if (frenet_coord_ != nullptr) {
    Point2D ego_frenet;
    if (frenet_coord_->XYToSL(
            Point2D(ego_state->ego_carte().x, ego_state->ego_carte().y),
            ego_frenet)) {
      double dt = 1.0 / FLAGS_planning_loop_rate;  // 0.1
      double ego_s = ego_frenet.x;
      double ego_l = ego_frenet.y;
      double curve_heading = frenet_coord_->GetPathCurveHeading(ego_s);
      double ego_frenet_relative_v_angle = planning_math::NormalizeAngle(
          ego_state->ego_v_angle() - curve_heading);
      double ego_v_s =
          ego_state->ego_v() * std::cos(ego_frenet_relative_v_angle);
      // double ego_v_l = ego_state->ego_v() *
      // std::sin(ego_frenet_relative_v_angle);
      const IndexedList<int, Obstacle> &current_obstacles =
          obstacles->get_obstacles();
      // If the obstacle disappears in the front view, predict
      for (size_t i = 0; i < history_obstacles_.size(); i++) {
        if (current_obstacles.Find(history_obstacles_[i].id) ==
            nullptr) {  // disappear
          double real_v_s = history_obstacles_[i].frenet_velocity_s;
          double real_v_l = history_obstacles_[i].frenet_velocity_l;
          double rel_v_s = real_v_s - ego_v_s;
          // double rel_v_l = real_v_l - ego_v_l;
          double rel_s =
              rel_v_s * planning_loop_dt_ + history_obstacles_[i].frenet_rel_s;
          double rel_l = history_obstacles_[i].frenet_rel_l;
          double real_s = ego_s + rel_s;
          double real_l = ego_l + rel_l;
          Point2D obstacle_frenet;
          if (history_obstacles_[i].is_static) {
            if (frenet_coord_->XYToSL(
                    Point2D(history_obstacles_[i].cart_x_center,
                            history_obstacles_[i].cart_y_center),
                    obstacle_frenet)) {
              real_s = obstacle_frenet.x;
              real_l = history_obstacles_[i].frenet_l;
              rel_s = real_s - ego_s;
              rel_l = real_l - ego_l;
              real_v_s = 0;
              real_v_l = 0;
            } else {
              rel_s = history_obstacles_[i].frenet_rel_s -
                      ego_v_s * planning_loop_dt_;
              real_s = ego_s + rel_s;
              real_l = history_obstacles_[i].frenet_l;
              rel_l = real_l - ego_l;
              real_v_s = 0;
              real_v_l = 0;
            }
          }
          // obstacle center to camera.  camera to car front edge:1.25m
          // maintain obstacle in blind spots for entering the field of vision
          double obs_to_camera_s =
              rel_s - vehicle_param.front_edge_to_rear_axle + 1.25;
          // TODO(bsniu): rel_camera_s to be modified (enu)
          if (CheckEgoNearBound(rel_s, rel_l) &&
              ((obs_to_camera_s < 0) ||
               (std::abs(rel_l) > (obs_to_camera_s * std::tan(45))))) {
            if (old_obstacles_[i].is_static()) {
              new_obstacles_.emplace_back(std::move(old_obstacles_[i]));
              continue;
            }
            // dynamic obstacle deduction
            history_obstacles_[i].frenet_rel_s = rel_s;
            history_obstacles_[i].frenet_rel_l = rel_l;
            history_obstacles_[i].frenet_velocity_s = real_v_s;
            history_obstacles_[i].frenet_velocity_l = real_v_l;
            PredictionObject new_prediction;
            UpdatePredictionTrajectory(ego_s, ego_l, ego_v_s, real_s, real_l,
                                       ego_state, history_obstacles_[i],
                                       new_prediction);
            Obstacle new_obstacle(history_obstacles_[i].id, new_prediction,
                                  history_obstacles_[i].is_static, 0.0);
            new_obstacles_.emplace_back(std::move(new_obstacle));
          }
        } else {
          if (old_obstacles_[i].is_static()) {
            new_obstacles_.emplace_back(std::move(old_obstacles_[i]));
          }
        }
      }
    }
  }
  return true;
}

void HistoryObstacleManager::UpdatePredictionTrajectory(
    double ego_s, double ego_l, double ego_v_s, double frenet_s,
    double frenet_l, const std::shared_ptr<EgoStateManager> &ego_state,
    const HistoryObstacle &history_object, PredictionObject &new_prediction) {
  new_prediction.id = history_object.id;
  new_prediction.type = history_object.type;
  new_prediction.length = history_object.length;
  new_prediction.width = history_object.width;
  new_prediction.yaw = history_object.cart_heading_angle;
  new_prediction.relative_theta =
      history_object.cart_heading_angle - ego_state->heading_angle();
  new_prediction.theta = history_object.cart_velocity_angle;
  new_prediction.speed = history_object.cart_velocity;
  double relative_speed =
      history_object.cart_velocity - ego_state->ego_v();  // 标量
  new_prediction.relative_speed_x =
      history_object.cart_velocity *
          std::cos(history_object.cart_velocity_angle) -
      ego_state->ego_v() * std::cos(ego_state->ego_v_angle());
  new_prediction.relative_speed_y =
      history_object.cart_velocity *
          std::sin(history_object.cart_velocity_angle) -
      ego_state->ego_v() * std::sin(ego_state->ego_v_angle());
  new_prediction.acc = history_object.cart_acc;
  new_prediction.fusion_source = history_object.fusion_source;
  new_prediction.position_x = history_object.cart_x_center;
  new_prediction.position_y = history_object.cart_y_center;
  new_prediction.relative_position_x =
      history_object.cart_x_center - ego_state->ego_carte().x;
  new_prediction.relative_position_y =
      history_object.cart_y_center - ego_state->ego_carte().y;

  Point2D cart_point_init;
  if (frenet_coord_->SLToXY(Point2D(frenet_s, frenet_l), cart_point_init)) {
    new_prediction.position_x = cart_point_init.x;
    new_prediction.position_y = cart_point_init.y;
    new_prediction.relative_position_x =
        cart_point_init.x - ego_state->ego_carte().x;
    new_prediction.relative_position_y =
        cart_point_init.y - ego_state->ego_carte().y;
  }

  double real_v_s = history_object.frenet_velocity_s;
  // double real_v_l = history_object.frenet_velocity_l;
  double rel_v_s = real_v_s - ego_v_s;
  // double rel_v_l = real_v_l - ego_v_l;
  double real_s = frenet_s;
  double real_l = frenet_l;
  double rel_s = frenet_s - ego_s;
  double rel_l = frenet_l - ego_l;
  PredictionTrajectory prediction_trajectory;
  for (size_t i = 0; i < history_object.trajectory_size; ++i) {
    PredictionTrajectoryPoint prediction_trajectory_point;
    Point2D cart_point;
    if (frenet_coord_->SLToXY(Point2D(real_s, real_l), cart_point)) {
      prediction_trajectory_point.x = cart_point.x;
      prediction_trajectory_point.y = cart_point.y;
      prediction_trajectory_point.relative_ego_x =
          cart_point.x - ego_state->ego_carte().x;
      prediction_trajectory_point.relative_ego_y =
          cart_point.y - ego_state->ego_carte().y;
      prediction_trajectory_point.speed = new_prediction.speed;
      // prediction_trajectory_point.prob = ;
      // prediction_trajectory_point.std_dev_x = ;
      // prediction_trajectory_point.std_dev_y = ;
      prediction_trajectory_point.theta = new_prediction.theta;
      prediction_trajectory_point.yaw = new_prediction.yaw;
      prediction_trajectory_point.relative_ego_yaw =
          new_prediction.relative_theta;
      prediction_trajectory_point.relative_ego_speed = relative_speed;
      // prediction_trajectory_point.relative_ego_std_dev_x = ;
      // prediction_trajectory_point.relative_ego_std_dev_y = ;
      // prediction_trajectory_point.relative_ego_std_dev_yaw = ;
      // prediction_trajectory_point.relative_ego_std_dev_speed = ;
      prediction_trajectory_point.relative_time = i * 0.2;
    }
    prediction_trajectory.trajectory.emplace_back(prediction_trajectory_point);
    rel_s = rel_v_s * 0.2 + rel_s;
    real_s = ego_s + rel_s;
    real_l = ego_l + rel_l;
  }
  new_prediction.trajectory_array.emplace_back(prediction_trajectory);
}

bool HistoryObstacleManager::CheckEgoNearBound(double rel_s, double rel_l) {
  if ((rel_s <= config_.ego_near_bound_s1) &&
      (rel_s >= config_.ego_near_bound_s0) &&
      (std::fabs(rel_l) <= config_.ego_near_bound_l)) {
    return true;
  }
  return false;
}

void HistoryObstacleManager::AddNewDeductionObstacles(
    const std::shared_ptr<ReferencePath> &reference_path,
    std::vector<std::shared_ptr<FrenetObstacle>> &current_obstacles) {
  // get new frenet obstacles
  for (const Obstacle &obstacle : new_obstacles_) {
    std::shared_ptr<FrenetObstacle> new_frenet_obstacle;
    new_frenet_obstacle = std::make_shared<FrenetObstacle>(
        &obstacle, *reference_path,
        session_->environmental_model().get_ego_state_manager(),
        session_->environmental_model().location_valid());
    current_obstacles.emplace_back(new_frenet_obstacle);
  }
  // check ego near, save obstacle info
  history_obstacles_.clear();
  old_obstacles_.clear();
  history_obstacles_.reserve(current_obstacles.size());
  old_obstacles_.reserve(current_obstacles.size());
  for (const std::shared_ptr<FrenetObstacle> &frenet_obstacle :
       current_obstacles) {
    double rel_s = frenet_obstacle->frenet_s() -
                   reference_path->get_frenet_ego_state().s();
    double rel_l = frenet_obstacle->frenet_l() -
                   reference_path->get_frenet_ego_state().l();
    if (CheckEgoNearBound(rel_s, rel_l)) {
      HistoryObstacle history_obstacle;
      history_obstacle.id = frenet_obstacle->obstacle()->id();
      history_obstacle.type = frenet_obstacle->obstacle()->type();
      history_obstacle.cart_x_center = frenet_obstacle->obstacle()->x_center();
      history_obstacle.cart_y_center = frenet_obstacle->obstacle()->y_center();
      history_obstacle.cart_heading_angle =
          frenet_obstacle->obstacle()->heading_angle();
      history_obstacle.cart_velocity = frenet_obstacle->obstacle()->velocity();
      history_obstacle.cart_velocity_angle =
          frenet_obstacle->obstacle()->velocity_angle();
      history_obstacle.cart_acc = frenet_obstacle->obstacle()->acceleration();
      history_obstacle.frenet_s = frenet_obstacle->frenet_s();
      history_obstacle.frenet_l = frenet_obstacle->frenet_l();
      history_obstacle.frenet_rel_s = rel_s;
      history_obstacle.frenet_rel_l = rel_l;
      history_obstacle.frenet_velocity_s = frenet_obstacle->frenet_velocity_s();
      history_obstacle.frenet_velocity_l = frenet_obstacle->frenet_velocity_l();
      history_obstacle.length = frenet_obstacle->obstacle()->length();
      history_obstacle.width = frenet_obstacle->obstacle()->width();
      history_obstacle.fusion_source =
          OBSTACLE_SOURCE_CAMERA;  // TODO(bsniu):To be modified
      history_obstacle.trajectory_size =
          frenet_obstacle->obstacle()->trajectory().size();
      history_obstacle.is_static = frenet_obstacle->obstacle()->is_static();
      history_obstacles_.emplace_back(std::move(history_obstacle));
      Obstacle old_obstacle(frenet_obstacle->obstacle());
      old_obstacles_.emplace_back(std::move(old_obstacle));
    }
  }
}
}  // namespace planning