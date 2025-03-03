#pragma once
#include <cstdint>
#include <memory>

#include "apa_obstacle_manager.h"
#include "collision_detection/base_collision_detector.h"
#include "collision_detection/path_safe_checker.h"
#include "collision_detection/uss_obstacle_avoidance.h"
#include "edt_collision_detector.h"
#include "geometry_collision_detector.h"
#include "gjk_collision_detector.h"

namespace planning {
namespace apa_planner {

enum class CollisionDetectorType : uint8_t {
  GEOMETRY,
  GJK,
  EDT,
  COUNT,
};

class CollisionDetectorInterface {
 public:
  CollisionDetectorInterface(
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr,
      const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
      const std::shared_ptr<ApaPredictPathManager>& predict_path_ptr);
  ~CollisionDetectorInterface() {}

  const std::shared_ptr<GeometryCollisionDetector>&
  GetGeometryCollisionDetectorPtr() const {
    return geometry_collision_detector_ptr_;
  }

  const std::shared_ptr<GJKCollisionDetector>& GetGJKCollisionDetectorPtr()
      const {
    return gjk_collision_detector_ptr_;
  }

  const std::shared_ptr<EDTCollisionDetector>& GetEDTCollisionDetectorPtr()
      const {
    return edt_collision_detector_ptr_;
  }

  const std::shared_ptr<UssObstacleAvoidance>& GetUssObsAvoidancePtr() const {
    return uss_obstacle_avoider_ptr_;
  }

  const std::shared_ptr<PathSafeChecker>& GetPathSafeCheckPtr() const {
    return path_safe_check_ptr_;
  }

  void Init();
  void Reset();

 private:
  std::shared_ptr<GeometryCollisionDetector> geometry_collision_detector_ptr_;
  std::shared_ptr<GJKCollisionDetector> gjk_collision_detector_ptr_;
  std::shared_ptr<EDTCollisionDetector> edt_collision_detector_ptr_;
  std::shared_ptr<UssObstacleAvoidance> uss_obstacle_avoider_ptr_;
  std::shared_ptr<PathSafeChecker> path_safe_check_ptr_;

  std::shared_ptr<ApaObstacleManager> obs_manager_ptr_;
};
}  // namespace apa_planner
}  // namespace planning