#pragma once

#include <memory>

#include "generate_obstacle_decider/generate_obstacle_decider.h"
#include "parking_stop_decider.h"
#include "perpendicular_tail_in_path_generator.h"
#include "target_pose_decider/target_pose_decider.h"
namespace planning {
namespace apa_planner {

class ParkingTaskInterface {
 public:
  ParkingTaskInterface(
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr,
      const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr,
      const std::shared_ptr<apa_planner::ApaMeasureDataManager>&
          measure_data_ptr);
  ~ParkingTaskInterface() {}
  void Init();
  void Reset();

  const std::shared_ptr<TargetPoseDecider>& GetTargetPoseDeciderPtr() const {
    return target_pose_decider_ptr_;
  }

  const std::shared_ptr<GenerateObstacleDecider>&
  GetGenerateObstacleDeciderPtr() const {
    return generate_obstacle_decider_ptr_;
  }

  const std::shared_ptr<ParkingStopDecider>& GetParkingStopDeciderPtr() const {
    return parking_stop_decider_ptr_;
  }

  const std::shared_ptr<PerpendicularTailInPathGenerator>&
  GetPerpendicularTailInPathGeneratorPtr() const {
    return geometry_perpendicular_tail_in_path_generator_ptr_;
  }

 private:
  std::shared_ptr<TargetPoseDecider> target_pose_decider_ptr_;
  std::shared_ptr<GenerateObstacleDecider> generate_obstacle_decider_ptr_;
  std::shared_ptr<ParkingStopDecider> parking_stop_decider_ptr_;
  std::shared_ptr<PerpendicularTailInPathGenerator>
      geometry_perpendicular_tail_in_path_generator_ptr_;
};

}  // namespace apa_planner
}  // namespace planning