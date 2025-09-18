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

  const bool IsInClearZone(const cdl::AABB2f& box);

  const bool IsInClearZone(const geometry_lib::RectangleBound& bound);

  const std::vector<cdl::AABB>& GetBoxVec() const { return box_vec_; }

 private:
  const cdl::AABB GenerateBoundingBox(const Eigen::Vector2d& pt,
                                      const float x_gain = 1.0,
                                      const float y_gain = 1.0,
                                      const bool achieve_x_min = false,
                                      const bool achieve_x_max = false,
                                      const bool achieve_y_min = false,
                                      const bool achieve_y_max = false);

  const bool IsCollisionForBox(const cdl::AABB& box);

 private:
  std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr_;

  // the box where the obstacle is not in
  std::vector<cdl::AABB> box_vec_;

  std::vector<cdl::AABB2f> box_vecf_;
};
}  // namespace apa_planner
}  // namespace planning