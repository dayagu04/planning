#ifndef HISTORY_OBSTACLE_MANAGER_H
#define HISTORY_OBSTACLE_MANAGER_H

#include "frenet_obstacle.h"
#include "prediction_object.h"
#include "reference_path_manager.h"
#include "session.h"
#include "utils/kd_path.h"

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
  double frenet_s = 0.0;
  double frenet_l = 0.0;
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
  HistoryObstacleManager(const EgoPlanningConfigBuilder *config_builder,
                         planning::framework::Session *session);
  virtual ~HistoryObstacleManager();

  bool Update();

  void AddNewDeductionObstacles(
      const std::shared_ptr<ReferencePath> &reference_path,
      std::vector<std::shared_ptr<FrenetObstacle>> &current_obstacles);

  const std::vector<Obstacle> &GetOldObstacles() const {
    return old_obstacles_;
  }

 private:
  bool CheckEgoNearBound(double rel_s, double rel_l);

  void UpdatePredictionTrajectory(
      double ego_s, double ego_l, double ego_v_s, double frenet_s,
      double frenet_l, const std::shared_ptr<EgoStateManager> &ego_state,
      const HistoryObstacle &history_object, PredictionObject &new_prediction);

  HistoryObstacleConfig config_;
  planning::framework::Session *session_ = nullptr;
  std::shared_ptr<KDPath> frenet_coord_;
  std::vector<Obstacle> old_obstacles_;  // hack: maintain static_obstacle
  std::vector<Obstacle> new_obstacles_;
  std::vector<HistoryObstacle> history_obstacles_;
  // VehicleParam vehicle_param_;
  double planning_loop_dt_;
};
}  // namespace planning

#endif