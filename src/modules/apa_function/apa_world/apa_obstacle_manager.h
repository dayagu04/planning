#pragma once

#include "apa_obstacle.h"
#include <limits>
#include <memory>
#include <string>
#include "session.h"

namespace planning {
namespace apa_planner {

class ApaObstacleManager final {
  ApaObstacleManager() {}
  ~ApaObstacleManager() {}
};
}  // namespace apa_planner
}  // namespace planning

namespace planning {

// obstacle manager: 管理所有障碍物，包括超声波、限位器、视觉、虚拟
// apa_obstacle: 是泊车障碍物的基本数据结构.
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

}  // namespace planning
