#include "history_obstacle_manager.h"

#include <cstddef>
#include <vector>

#include "../../common/planning_gflags.h"
#include "obstacle_manager.h"
#include "utils/frenet_coordinate_system.h"

namespace planning {

HistoryObstacleManager::HistoryObstacleManager(
    planning::framework::Session *session) {
  session_ = session;
  Init();
}

HistoryObstacleManager::~HistoryObstacleManager() {
  history_obstacles_.clear();
  new_obstacles_.clear();
}

void HistoryObstacleManager::Init() { frenet_coord_ = nullptr; }

bool HistoryObstacleManager::Update() {
  new_obstacles_.clear();
  UpdateNearbyObstacles(
      session_->environmental_model().get_ego_state_manager(),
      session_->environmental_model().get_reference_path_manager(),
      session_->environmental_model().get_obstacle_manager());
  history_obstacles_.clear();
  return true;
}

void HistoryObstacleManager::UpdateNearbyObstacles(
    const std::shared_ptr<EgoStateManager> &ego_state,
    const std::shared_ptr<ReferencePathManager> &reference_path,
    const std::shared_ptr<ObstacleManager> &obstacles) {
  frenet_coord_ =
      reference_path->get_reference_path_by_current_lane()->get_frenet_coord();
  if (frenet_coord_ != nullptr) {
    Point2D ego_frenet;
    if (frenet_coord_->CartCoord2FrenetCoord(
            Point2D(ego_state->ego_carte().x, ego_state->ego_carte().y),
            ego_frenet) == TRANSFORM_SUCCESS) {
      double dt = 1.0 / FLAGS_planning_loop_rate;  // 0.1
      double ego_s = ego_frenet.x;
      double ego_l = ego_frenet.y;
      double curve_heading = frenet_coord_->GetRefCurveHeading(ego_s);
      double ego_frenet_relative_v_angle = planning_math::NormalizeAngle(
          ego_state->ego_v_angle() - curve_heading);
      double ego_v_s =
          ego_state->ego_v() * std::cos(ego_frenet_relative_v_angle);
      // double ego_v_l = ego_state->ego_v() *
      // std::sin(ego_frenet_relative_v_angle);
      const IndexedList<int, Obstacle> &current_obstacles =
          obstacles->get_obstacles();
      // If the obstacle disappears in the front view, predict
      new_obstacles_.reserve(history_obstacles_.size());
      for (HistoryObstacle &history_obstacle : history_obstacles_) {
        if (current_obstacles.Find(history_obstacle.id) == nullptr) {
          double real_v_s = history_obstacle.frenet_velocity_s;
          double real_v_l = history_obstacle.frenet_velocity_l;
          double rel_v_s = real_v_s - ego_v_s;
          // double rel_v_l = real_v_l - ego_v_l;
          double rel_s = rel_v_s * dt + history_obstacle.frenet_rel_s;
          double rel_l = history_obstacle.frenet_rel_l;
          double real_s = ego_s + rel_s;
          double real_l = ego_l + rel_l;
          if (CheckEgoNearBound(rel_s, rel_l)) {
            history_obstacle.frenet_rel_s = rel_s;
            history_obstacle.frenet_rel_l = rel_l;
            history_obstacle.frenet_velocity_s = real_v_s;
            history_obstacle.frenet_velocity_l = real_v_l;
            PredictionObject new_prediction;
            UpdatePredictionTrajectory(ego_s, ego_l, ego_v_s, real_s, real_l,
                                       ego_state, history_obstacle,
                                       new_prediction);
            Obstacle new_obstacle(history_obstacle.id, new_prediction,
                                  history_obstacle.is_static, 0.0);
            new_obstacles_.emplace_back(std::move(new_obstacle));
          }
        }
      }
    }
  }
}

bool HistoryObstacleManager::CheckEgoNearBound(double rel_s, double rel_l) {
  const double kEgoNearBoundS = 10.0;
  const double kEgoNearBoundL = 5.0;
  if ((std::fabs(rel_s) <= kEgoNearBoundS) &&
      (std::fabs(rel_l) <= kEgoNearBoundL)) {
    return true;
  }
  return false;
}

void HistoryObstacleManager::SelectObstacleNearEgo(
    const std::vector<std::shared_ptr<FrenetObstacle>> &near_frenet_obstacles,
    const FrenetEgoState &frenet_ego) {
  history_obstacles_.reserve(near_frenet_obstacles.size());
  for (const std::shared_ptr<FrenetObstacle> &frenet_obstacle :
       near_frenet_obstacles) {
    // if ((CheckEgoNearBound(frenet_obstacle->rel_s(),
    // frenet_obstacle->l_relative_to_ego())) &&
    // (!frenet_obstacle->obstacle()->is_static())) {
    if (CheckEgoNearBound(frenet_obstacle->rel_s(),
                          frenet_obstacle->l_relative_to_ego())) {
      HistoryObstacle cur_obstacles;
      cur_obstacles.id = frenet_obstacle->obstacle()->id();
      cur_obstacles.type = frenet_obstacle->obstacle()->type();
      cur_obstacles.cart_x_center = frenet_obstacle->obstacle()->x_center();
      cur_obstacles.cart_y_center = frenet_obstacle->obstacle()->y_center();
      cur_obstacles.cart_heading_angle =
          frenet_obstacle->obstacle()->heading_angle();
      cur_obstacles.cart_velocity = frenet_obstacle->obstacle()->velocity();
      cur_obstacles.cart_velocity_angle =
          frenet_obstacle->obstacle()->velocity_angle();
      cur_obstacles.cart_acc = frenet_obstacle->obstacle()->acceleration();
      cur_obstacles.frenet_rel_s = frenet_obstacle->frenet_s() - frenet_ego.s();
      cur_obstacles.frenet_rel_l = frenet_obstacle->frenet_l() - frenet_ego.l();
      cur_obstacles.frenet_velocity_s = frenet_obstacle->frenet_velocity_s();
      cur_obstacles.frenet_velocity_l = frenet_obstacle->frenet_velocity_l();
      cur_obstacles.length = frenet_obstacle->obstacle()->length();
      cur_obstacles.width = frenet_obstacle->obstacle()->width();
      cur_obstacles.fusion_source =
          OBSTACLE_SOURCE_CAMERA;  // TODO(bsniu):To be modified
      cur_obstacles.trajectory_size =
          frenet_obstacle->obstacle()->trajectory().size();
      cur_obstacles.is_static = frenet_obstacle->obstacle()->is_static();
      history_obstacles_.emplace_back(cur_obstacles);
    }
  }
}

void HistoryObstacleManager::UpdatePredictionTrajectory(
    double ego_s, double ego_l, double ego_v_s, double frenet_s,
    double frenet_l, const std::shared_ptr<EgoStateManager> &ego_state,
    const HistoryObstacle &history_object, PredictionObject &new_trajectory) {
  double dt = 1.0 / FLAGS_planning_loop_rate;  // 0.1
  new_trajectory.id = history_object.id;
  new_trajectory.type = history_object.type;
  new_trajectory.length = history_object.length;
  new_trajectory.width = history_object.width;
  new_trajectory.yaw = history_object.cart_heading_angle;
  new_trajectory.relative_theta =
      history_object.cart_heading_angle - ego_state->heading_angle();
  new_trajectory.theta = history_object.cart_velocity_angle;
  new_trajectory.speed = history_object.cart_velocity;
  double relative_speed =
      history_object.cart_velocity - ego_state->ego_v();  // 标量
  new_trajectory.relative_speed_x =
      history_object.cart_velocity *
          std::cos(history_object.cart_velocity_angle) -
      ego_state->ego_v() * std::cos(ego_state->ego_v_angle());
  new_trajectory.relative_speed_y =
      history_object.cart_velocity *
          std::sin(history_object.cart_velocity_angle) -
      ego_state->ego_v() * std::sin(ego_state->ego_v_angle());
  new_trajectory.acc = history_object.cart_acc;
  new_trajectory.fusion_source = history_object.fusion_source;
  new_trajectory.position_x = history_object.cart_x_center;
  new_trajectory.position_y = history_object.cart_y_center;
  new_trajectory.relative_position_x =
      history_object.cart_x_center - ego_state->ego_carte().x;
  new_trajectory.relative_position_y =
      history_object.cart_y_center - ego_state->ego_carte().y;

  Point2D cart_point_init;
  if (frenet_coord_->FrenetCoord2CartCoord(
          Point2D(frenet_s, frenet_l), cart_point_init) == TRANSFORM_SUCCESS) {
    new_trajectory.position_x = cart_point_init.x;
    new_trajectory.position_y = cart_point_init.y;
    new_trajectory.relative_position_x =
        cart_point_init.x - ego_state->ego_carte().x;
    new_trajectory.relative_position_y =
        cart_point_init.y - ego_state->ego_carte().y;
  }
  if (!history_object.is_static) {
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
      rel_s = rel_v_s * dt + rel_s;
      real_s = ego_s + rel_s;
      real_l = ego_l + rel_l;
      Point2D cart_point;
      if (frenet_coord_->FrenetCoord2CartCoord(
              Point2D(real_s, real_l), cart_point) == TRANSFORM_SUCCESS) {
        prediction_trajectory_point.x = cart_point.x;
        prediction_trajectory_point.y = cart_point.y;
        prediction_trajectory_point.relative_ego_x =
            cart_point.x - ego_state->ego_carte().x;
        prediction_trajectory_point.relative_ego_y =
            cart_point.y - ego_state->ego_carte().y;
        prediction_trajectory_point.speed = new_trajectory.speed;
        // new_trajectory_point.prob = ;
        // new_trajectory_point.std_dev_x = ;
        // new_trajectory_point.std_dev_y = ;
        prediction_trajectory_point.theta = new_trajectory.theta;
        prediction_trajectory_point.yaw = new_trajectory.yaw;
        prediction_trajectory_point.relative_ego_yaw =
            new_trajectory.relative_theta;
        prediction_trajectory_point.relative_ego_speed = relative_speed;
        // new_trajectory_point.relative_ego_std_dev_x = ;
        // new_trajectory_point.relative_ego_std_dev_y = ;
        // new_trajectory_point.relative_ego_std_dev_yaw = ;
        // new_trajectory_point.relative_ego_std_dev_speed = ;
        // new_trajectory_point.relative_time = ;
      }
      prediction_trajectory.trajectory.emplace_back(
          prediction_trajectory_point);
    }
    new_trajectory.trajectory_array.emplace_back(prediction_trajectory);
  }
}
}  // namespace planning