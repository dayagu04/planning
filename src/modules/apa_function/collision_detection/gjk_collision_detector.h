#pragma once
#include "apa_param_config.h"
#include "base_collision_detector.h"
#include "geometry_math.h"
#include "gjk2d_interface.h"

namespace planning {
namespace apa_planner {

enum class CarBodyType : uint8_t {
  NORMAL,
  EXPAND_MIRROR_TO_FRONT,
  EXPAND_MIRROR_TO_END,
  ONLY_MIRROR,
  ONLY_MAX_POLYGAN,
};

struct GJKColDetRequest {
  bool use_obs_base_slot = true;
  bool use_uss_pt = apa_param.GetParam().uss_config.use_uss_pt_cloud;
  bool use_limiter = true;
  CarBodyType car_body_type = CarBodyType::NORMAL;
  ApaObsMovementType movement_type = ApaObsMovementType::ALL;
  UseObsHeightMethod use_obs_height_method = UseObsHeightMethod::HIGH;

  GJKColDetRequest() = default;
  ~GJKColDetRequest() = default;
  GJKColDetRequest(
      const bool _use_obs_base_slot,
      const bool _use_uss_pt = apa_param.GetParam().uss_config.use_uss_pt_cloud,
      const CarBodyType _car_body_type = CarBodyType::NORMAL,
      const ApaObsMovementType _movement_type = ApaObsMovementType::ALL,
      const UseObsHeightMethod _use_obs_height_method =
          UseObsHeightMethod::HIGH,
      const bool _use_limiter = true) {
    Set(_use_obs_base_slot, _use_uss_pt, _car_body_type, _movement_type,
        _use_obs_height_method, _use_limiter);
  }

  void Set(
      const bool _use_obs_base_slot = true,
      const bool _use_uss_pt = apa_param.GetParam().uss_config.use_uss_pt_cloud,
      const CarBodyType _car_body_type = CarBodyType::NORMAL,
      const ApaObsMovementType _movement_type = ApaObsMovementType::ALL,
      const UseObsHeightMethod _use_obs_height_method =
          UseObsHeightMethod::HIGH,
      const bool _use_limiter = true) {
    use_obs_base_slot = _use_obs_base_slot;
    use_uss_pt = _use_uss_pt;
    car_body_type = _car_body_type;
    movement_type = _movement_type;
    use_obs_height_method = _use_obs_height_method;
    use_limiter = _use_limiter;
  }
};

class GJKCollisionDetector final : public BaseCollisionDetector {
 public:
  GJKCollisionDetector() {}
  GJKCollisionDetector(
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr) {
    SetObsManagerPtr(obs_manager_ptr);
  }
  ~GJKCollisionDetector() {}

  const ColResult Update(const geometry_lib::PathSegment& path_seg,
                         const double body_lat_buffer, const double lon_buffer,
                         const GJKColDetRequest gjk_col_det_request,
                         const bool special_process_mirror = false,
                         const double mirror_lat_buffer = 0.08);

  const ColResult Update(const std::vector<geometry_lib::PathPoint>& pt_vec,
                         const double body_lat_buffer, const double lon_buffer,
                         const GJKColDetRequest gjk_col_det_request,
                         const bool special_process_mirror = false,
                         const double mirror_lat_buffer = 0.08);

  const bool IsPolygonCollision(const Polygon2D& polygon,
                                const GJKColDetRequest gjk_col_det_request);

  const bool IsPolygonCollision(const Polygon2D& polygon,
                                const ApaObstacle& obs,
                                const GJKColDetRequest gjk_col_det_request,
                                Eigen::Vector2d& dangerous_pt);

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