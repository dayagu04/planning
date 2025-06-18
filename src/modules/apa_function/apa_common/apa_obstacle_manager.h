#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apa_obstacle.h"
#include "geometry_math.h"
#include "local_view.h"
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
    uss_dis_vec_.clear();
  }

  const std::unordered_map<size_t, ApaObstacle> &GetObstacles() const {
    return obstacles_;
  }

  std::unordered_map<size_t, ApaObstacle> &GetMutableObstacles() {
    return obstacles_;
  }

  const size_t GetObsIdGenerate() const { return obs_id_generate_; }

  void AddObstacle(const ApaObstacle &apa_obs) {
    obstacles_[obs_id_generate_] = apa_obs;
    obs_id_generate_++;
  }

  void SetObstacle(const size_t id, const ApaObstacle &apa_obs);

  const bool GetObstacle(const size_t id, ApaObstacle *obs);

  const bool GetObstacle(const ApaObsAttributeType type,
                         std::vector<ApaObstacle *> &obs_vec);

  const std::vector<double> &GetUssDisVec() { return uss_dis_vec_; }

  void TransformCoordFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf &g2l_tf);

  const bool IsConsideredODType(const iflyauto::ObjectType type);

  const bool IsDynamicObjectType(const iflyauto::ObjectType type);

 private:
  size_t obs_id_generate_{0};
  std::unordered_map<size_t, ApaObstacle> obstacles_;
  std::vector<double> uss_dis_vec_;
};

typedef IndexedList<int, ApaObstacle> IndexedParkObstacles;
typedef ThreadSafeIndexedList<int, ApaObstacle> ThreadSafeParkObstacles;

}  // namespace apa_planner
}  // namespace planning
