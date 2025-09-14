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

  geometry_col_det_ptr_ =
      std::make_shared<GeometryCollisionDetector>(obs_manager_ptr);

  gjk_col_det_ptr_ = std::make_shared<GJKCollisionDetector>(obs_manager_ptr);

  edt_col_det_ptr_ = std::make_shared<EDTCollisionDetector>(obs_manager_ptr);

  uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>(
      obs_manager_ptr, measure_data_ptr, predict_path_ptr);

  path_safe_check_ptr_ = std::make_shared<PathSafeChecker>(obs_manager_ptr);

  has_constructed_flag_ = true;

  Init(fold_mirror_flag_);
}

void CollisionDetectorInterface::Init(const bool fold_mirror_flag) {
  if (init_flag_ && fold_mirror_flag == fold_mirror_flag_) {
    return;
  }
  if (!has_constructed_flag_) {
    return;
  }
  fold_mirror_flag_ = fold_mirror_flag;
  geometry_col_det_ptr_->Init(fold_mirror_flag_);
  gjk_col_det_ptr_->Init(fold_mirror_flag_);
  edt_col_det_ptr_->Init(fold_mirror_flag_);
  uss_obstacle_avoider_ptr_->Init();
  init_flag_ = true;
}

void CollisionDetectorInterface::SetObsManagerPtr(
    const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr) {
  obs_manager_ptr_ = obs_manager_ptr;

  if (!has_constructed_flag_) {
    geometry_col_det_ptr_ =
        std::make_shared<GeometryCollisionDetector>(obs_manager_ptr);

    gjk_col_det_ptr_ = std::make_shared<GJKCollisionDetector>(obs_manager_ptr);

    edt_col_det_ptr_ = std::make_shared<EDTCollisionDetector>(obs_manager_ptr);

    uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>(
        obs_manager_ptr, nullptr, nullptr);

    path_safe_check_ptr_ = std::make_shared<PathSafeChecker>(obs_manager_ptr);

    has_constructed_flag_ = true;
  }

  else {
    geometry_col_det_ptr_->SetObsManagerPtr(obs_manager_ptr);
    gjk_col_det_ptr_->SetObsManagerPtr(obs_manager_ptr);
    edt_col_det_ptr_->SetObsManagerPtr(obs_manager_ptr);
  }
}

void CollisionDetectorInterface::Reset() {
  geometry_col_det_ptr_->Reset();
  gjk_col_det_ptr_->Reset();
  edt_col_det_ptr_->Reset();
  uss_obstacle_avoider_ptr_->Reset();
  init_flag_ = false;
  fold_mirror_flag_ = false;
  // has_constructed_flag_ = false;
}

}  // namespace apa_planner
}  // namespace planning