#pragma once

#include <memory>
#include <vector>

#include "aabb2d.h"
#include "apa_obstacle_manager.h"
namespace planning {
namespace apa_planner {
using namespace pnc;
class ObstacleClearZoneDecider {
 public:
  ObstacleClearZoneDecider() = default;
  ~ObstacleClearZoneDecider() = default;

  const bool GenerateBoundingBox(
      const std::vector<Eigen::Vector2d>& pt_vec,
      const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr);

  const bool IsInClearZone(const Eigen::Vector2d& pt);

  const bool IsInClearZone(const cdl::AABB& box);

  const bool IsInClearZone(const geometry_lib::RectangleBound& bound);

  const std::vector<cdl::AABB>& GetBoxVec() const { return box_vec_; }

 private:
  const cdl::AABB GenerateBoundingBox(const Eigen::Vector2d& pt);

  const bool IsCollisionForBox(const cdl::AABB& box);

 private:
  std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr_;

  // the box where the obstacle is not in
  std::vector<cdl::AABB> box_vec_;
};
}  // namespace apa_planner
}  // namespace planning