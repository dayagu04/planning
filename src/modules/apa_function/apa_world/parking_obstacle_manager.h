#pragma once

#include "parking_obstacle.h"
#include <limits>
#include <memory>
#include <string>
#include "session.h"

namespace planning {

class ParkObstacleManager {
 public:
  ParkObstacleManager(planning::framework::Session *session);

  ~ParkObstacleManager() = default;

  void Clear();

  ParkObstacle *AddObstacle(const ParkObstacle &obstacle);

  const ParkObstacle *FindObstacle(int object_id) const;

  ParkObstacle *FindObstacle(int object_id);

  const IndexedList<int, ParkObstacle> &GetObstacles() const;

  void Update();

private:
 void SampleInLineSegment(const Eigen::Vector2d &start,
                          const Eigen::Vector2d &end,
                          std::vector<Position2D> *points);

private:
 int obs_id_;
 IndexedList<int, ParkObstacle> obstacles_;
 framework::Session *session_ = nullptr;
};

} // namespace parking

