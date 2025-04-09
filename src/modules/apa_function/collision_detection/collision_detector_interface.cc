#include "collision_detector_interface.h"

#include <memory>

#include "collision_detection/edt_collision_detector.h"
#include "collision_detection/geometry_collision_detector.h"
#include "collision_detection/gjk_collision_detector.h"
#include "collision_detection/path_safe_checker.h"
#include "collision_detection/uss_obstacle_avoidance.h"

namespace planning {
namespace apa_planner {

CollisionDetectorInterface::CollisionDetectorInterface(
    const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr,
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
    const std::shared_ptr<ApaPredictPathManager>& predict_path_ptr) {
  obs_manager_ptr_ = obs_manager_ptr;

  geometry_collision_detector_ptr_ =
      std::make_shared<GeometryCollisionDetector>(obs_manager_ptr);

  gjk_collision_detector_ptr_ =
      std::make_shared<GJKCollisionDetector>(obs_manager_ptr);

  edt_collision_detector_ptr_ =
      std::make_shared<EDTCollisionDetector>(obs_manager_ptr);

  uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>(
      obs_manager_ptr, measure_data_ptr, predict_path_ptr);

  path_safe_check_ptr_ = std::make_shared<PathSafeChecker>(obs_manager_ptr);

  Init();
}

void CollisionDetectorInterface::Init() {
  geometry_collision_detector_ptr_->Init();
  gjk_collision_detector_ptr_->Init();
  edt_collision_detector_ptr_->Init();
  uss_obstacle_avoider_ptr_->Init();
}

void CollisionDetectorInterface::Reset() {
  geometry_collision_detector_ptr_->Reset();
  gjk_collision_detector_ptr_->Reset();
  edt_collision_detector_ptr_->Reset();
  uss_obstacle_avoider_ptr_->Reset();
}

}  // namespace apa_planner
}  // namespace planning