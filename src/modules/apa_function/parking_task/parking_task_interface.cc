#include "parking_task_interface.h"

#include <memory>

#include "perpendicular_tail_in_path_generator.h"
#include "tail_in/hybrid_astar_perpendicular_tail_in_path_generator.h"
#include "target_pose_decider/target_pose_decider.h"

namespace planning {
namespace apa_planner {

ParkingTaskInterface::ParkingTaskInterface(
    const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr,
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr,
    const std::shared_ptr<apa_planner::ApaMeasureDataManager>&
        measure_data_ptr) {
  target_pose_decider_ptr_ =
      std::make_shared<TargetPoseDecider>(col_det_interface_ptr);

  generate_obstacle_decider_ptr_ = std::make_shared<GenerateObstacleDecider>(
      obs_manager_ptr, col_det_interface_ptr);

  parking_stop_decider_ptr_ = std::make_shared<ParkingStopDecider>(
      col_det_interface_ptr, measure_data_ptr, obs_manager_ptr);

  geometry_perpendicular_tail_in_path_generator_ptr_ =
      std::make_shared<PerpendicularTailInPathGenerator>(col_det_interface_ptr);

  hybrid_astar_path_generator_interface_ptr_ =
      std::make_shared<HybridAstarPathGeneratorInterface>(
          col_det_interface_ptr);

  path_generator_thread_ptr_ = std::make_shared<PathGeneratorThread>();

  Init();
}

void ParkingTaskInterface::Init() {
  target_pose_decider_ptr_->Init();
  generate_obstacle_decider_ptr_->Init();
  parking_stop_decider_ptr_->Init();
  geometry_perpendicular_tail_in_path_generator_ptr_->Init();
  hybrid_astar_path_generator_interface_ptr_->Init();
  path_generator_thread_ptr_->Init();

  path_generator_thread_ptr_->Start();
}

void ParkingTaskInterface::Reset() {
  target_pose_decider_ptr_->Reset();
  generate_obstacle_decider_ptr_->Reset();
  parking_stop_decider_ptr_->Reset();
  geometry_perpendicular_tail_in_path_generator_ptr_->Reset();
  hybrid_astar_path_generator_interface_ptr_->Reset();
  path_generator_thread_ptr_->Reset();
}

}  // namespace apa_planner
}  // namespace planning
