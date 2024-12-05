#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apa_obstacle.h"
#include "local_view.h"
#include "point_cloud_obstacle.h"
#include "session.h"

namespace planning {
namespace apa_planner {
// obstacle manager: 管理所有障碍物，包括超声波、限位器、视觉、虚拟
// apa_obstacle: 是泊车障碍物的基本数据结构.
class ApaObstacleManager final {
 public:
  ApaObstacleManager() {}
  ~ApaObstacleManager() {}

  void Update(const LocalView *local_view);

  void Reset() {
    obs_id_generate_ = 0;
    obstacles_.clear();
  }

  const std::unordered_map<size_t, ApaObstacle> &GetObstacles() const {
    return obstacles_;
  }

  const size_t GetObsIdGenerate() const { return obs_id_generate_; }

  void SetObstacle(const size_t id, const ApaObstacle &apa_obs);

  const bool GetObstacle(const size_t id, ApaObstacle *obs);

  const bool GetObstacle(const ApaObsAttributeType type,
                         std::vector<ApaObstacle *> &obs_vec);

  void TransformCoordFromGlobalToLocal(
      const pnc::geometry_lib::PathPoint &origin_pose);

 private:
  size_t obs_id_generate_{0};
  std::unordered_map<size_t, ApaObstacle> obstacles_;
};

typedef IndexedList<int, ApaObstacle> IndexedParkObstacles;
typedef ThreadSafeIndexedList<int, ApaObstacle> ThreadSafeParkObstacles;

}  // namespace apa_planner
}  // namespace planning
