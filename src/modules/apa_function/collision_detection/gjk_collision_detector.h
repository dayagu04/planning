#pragma once
#include "base_collision_detector.h"
#include "geometry_math.h"
#include "gjk2d_interface.h"

namespace planning {
namespace apa_planner {

enum class CarBodyType : uint8_t {
  NORMAL,
  EXPAND_MIRROR_TO_FRONT,
  EXPAND_MIRROR_TO_END,
};

struct GJKColDetRequest {
  bool use_obs_base_slot = true;
  bool only_check_max_car_polygon = false;
  CarBodyType car_body_type = CarBodyType::NORMAL;
  ApaObsMovementType movement_type = ApaObsMovementType::ALL;
  ApaObsHeightType height_type = ApaObsHeightType::HIGH;
  bool use_uss_pt = false;

  GJKColDetRequest() = default;
  ~GJKColDetRequest() = default;
  GJKColDetRequest(
      const bool _use_obs_base_slot,
      const bool _only_check_max_car_polygon = false,
      const CarBodyType _car_body_type = CarBodyType::NORMAL,
      const ApaObsMovementType _movement_type = ApaObsMovementType::ALL,
      const ApaObsHeightType _height_type = ApaObsHeightType::HIGH) {
    Set(_use_obs_base_slot, _only_check_max_car_polygon, _car_body_type,
        _movement_type, _height_type);
  }

  void Set(const bool _use_obs_base_slot = true,
           const bool _only_check_max_car_polygon = false,
           const CarBodyType _car_body_type = CarBodyType::NORMAL,
           const ApaObsMovementType _movement_type = ApaObsMovementType::ALL,
           const ApaObsHeightType _height_type = ApaObsHeightType::HIGH) {
    use_obs_base_slot = _use_obs_base_slot;
    only_check_max_car_polygon = _only_check_max_car_polygon;
    car_body_type = _car_body_type;
    movement_type = _movement_type;
    height_type = _height_type;
  }
};

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
                         const GJKColDetRequest gjk_col_det_request);

  const ColResult Update(const std::vector<geometry_lib::PathPoint>& pt_vec,
                         const double lat_buffer, const double lon_buffer,
                         const GJKColDetRequest gjk_col_det_request);

  const bool IsPolygonCollision(const Polygon2D& polygon,
                                const GJKColDetRequest gjk_col_det_request);

  const bool IsPolygonCollision(const Polygon2D& polygon,
                                const ApaObstacle& obs,
                                const GJKColDetRequest gjk_col_det_request);

  const bool IsObsInCar(const geometry_lib::PathPoint& pose,
                        const double lat_buffer, const Eigen::Vector2d& obs);

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