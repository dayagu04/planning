#pragma once

#include "ego_planning_config.h"
#include "environmental_model.h"
#include "euler_distance_transform.h"
#include "session.h"

using namespace planning::planning_math;
namespace planning {

class EdtManager {
 public:
  EdtManager(const EgoPlanningConfigBuilder *config_builder,
             planning::framework::Session *session);

  void SetConfig(const EgoPlanningConfigBuilder *config_builder);

  void update();

  EulerDistanceTransform *GetEulerDistanceTransform() { return &edt_; }

  const bool GetIsEulerDistanceTransformValid() const { return is_edt_valid_; }

  static bool FilterObstacleForAra(
      const planning::FrenetObstacle &frenet_obstacle);

 private:
  void InitEDT();
  OccupancyGridBound GenerateOGM(const Pose2D &base_pose);
  void AddPointClouds(const std::vector<planning_math::Vec2d> &point_clouds,
                      size_t step = 1);
  void AddODPoint(const Obstacle &obstalce);
  bool UpdateEDT(const OccupancyGridBound &grid_bound);

 private:
  planning::framework::Session *session_ = nullptr;
  EgoPlanningEdtManagerConfig config_;
  OccupancyGridMap ogm_;
  EulerDistanceTransform edt_;
  bool is_edt_valid_;
  double resolution_;
};

}  // namespace planning
