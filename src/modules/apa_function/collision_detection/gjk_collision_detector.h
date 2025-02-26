#pragma once
#include "base_collision_detector.h"
#include "geometry_math.h"
#include "gjk2d_interface.h"

namespace planning {
namespace apa_planner {

class GJKCollisionDetector final : public BaseCollisionDetector {
 public:
  GJKCollisionDetector() {}
  GJKCollisionDetector(
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr) {
    SetObsManager(obs_manager_ptr);
  }
  ~GJKCollisionDetector() {}

  const ColResult Update(const geometry_lib::PathSegment& path_seg,
                         const double lat_buffer, const double lon_buffer,
                         const bool only_check_max_car_polygon = false,
                         const bool use_obs_base_slot = true,
                         const ApaObsMovementType movement_type_request =
                             ApaObsMovementType::ALL);

  const ColResult Update(const std::vector<geometry_lib::PathPoint>& pt_vec,
                         const double lat_buffer, const double lon_buffer,
                         const bool only_check_max_car_polygon = false,
                         const bool use_obs_base_slot = true,
                         const ApaObsMovementType movement_type_request =
                             ApaObsMovementType::ALL);

  const bool IsPolygonCollision(const Polygon2D& polygon,
                                const bool use_obs_base_slot = true);

  const bool IsPolygonCollision(const Polygon2D& polygon,
                                const ApaObstacle& obs,
                                const bool use_obs_base_slot = true);

  void Reset();

 private:
  void GenCarPolygon();
  void TransformPolygonFootPrintLocalToGlobal(
      const geometry_lib::PathPoint& pt);

 private:
  PolygonFootPrint polygon_foot_print_local_;   // base on car
  PolygonFootPrint polygon_foot_print_global_;  // base on slot or ground
  GJK2DInterface gjk_interface_;
};

}  // namespace apa_planner
}  // namespace planning