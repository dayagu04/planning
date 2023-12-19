#ifndef HISTORY_OBSTACLE_MANAGER_H
#define HISTORY_OBSTACLE_MANAGER_H

#include "frenet_obstacle.h"
#include "prediction_object.h"
#include "reference_path_manager.h"
#include "session.h"

namespace planning {

struct HistoryObstacle {
  int id{};
  Common::ObjectType type;
  double time = 0.0;
  double cart_x_center = 0.0;
  double cart_y_center = 0.0;
  double cart_velocity_x = 0.0;
  double cart_velocity_y = 0.0;
  double cart_heading_angle = 0.0;
  double cart_velocity = 0.0;
  double cart_velocity_angle = 0.0;
  double cart_acc = 0.0;
  double frenet_rel_s = 0.0;
  double frenet_rel_l = 0.0;
  double frenet_velocity_s = 0.0;
  double frenet_velocity_l = 0.0;
  double width = 0.0;
  double length = 0.0;
  unsigned int fusion_source = 1;
  unsigned int trajectory_size = 0;
  bool is_static = false;
};

class ObstacleManager;
class HistoryObstacleManager {
 public:
  HistoryObstacleManager(planning::framework::Session *session);
  virtual ~HistoryObstacleManager();

  bool Update();

  void SelectObstacleNearEgo(
      const std::vector<std::shared_ptr<FrenetObstacle>> &near_frenet_obstacles,
      const FrenetEgoState &frenet_ego);

  const std::vector<Obstacle> &GetNearPredictionObstacle() const {
    return new_obstacles_;
  }

 private:
  void Init();

  void UpdateNearbyObstacles(
      const std::shared_ptr<EgoStateManager> &ego_state,
      const std::shared_ptr<ReferencePathManager> &reference_path,
      const std::shared_ptr<ObstacleManager> &obstacles);

  bool CheckEgoNearBound(double rel_s, double rel_l);

  void UpdatePredictionTrajectory(
      double ego_s, double ego_l, double ego_v_s, double frenet_s,
      double frenet_l, const std::shared_ptr<EgoStateManager> &ego_state,
      const HistoryObstacle &history_object, PredictionObject &new_trajectory);

  planning::framework::Session *session_ = nullptr;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  std::vector<Obstacle> new_obstacles_;
  std::vector<HistoryObstacle> history_obstacles_;
};
}  // namespace planning

#endif