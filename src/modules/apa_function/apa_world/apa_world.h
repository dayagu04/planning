#ifndef __APA_WORLD_H__
#define __APA_WORLD_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>

#include "apa_data.h"
#include "collision_detection/collision_detection.h"
#include "collision_detection/uss_obstacle_avoidance.h"
#include "common.pb.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "slot_manager.h"
#include "slot_management_info.pb.h"
#include "spline.h"
#include "spline_projection.h"
#include "collision_detection/uss_obstacle_avoidance.h"

#define APA_COMPARE_PLANNING_TRAJ_POINTS_NUM 26

namespace planning {
namespace apa_planner {

class ApaWorld {
 public:
  ApaWorld() { Init(); }
  ~ApaWorld() {}

  void Init();
  void Reset();
  const bool Update(const LocalView* const local_view);
  const bool Update();

  std::shared_ptr<ApaData> GetApaDataPtr() { return apa_data_ptr_; }

  std::shared_ptr<SlotManager> GetSlotManagerPtr() {
    return slot_manager_ptr_;
  }

  std::shared_ptr<UssObstacleAvoidance> GetUssObstacleAvoidancePtr() {
    return uss_obstacle_avoider_ptr_;
  }

  std::shared_ptr<CollisionDetector> GetCollisionDetectorPtr() {
    return collision_detector_ptr_;
  }

  std::shared_ptr<LateralPathOptimizer> GetLateralPathOptimizerPtr() {
    return lateral_path_optimizer_ptr_;
  }

  const LocalView* GetLocalViewPtr() { return local_view_ptr_; }

 private:
  void Preprocess();
  void UpdateEgoState();
  void UpdateStateMachine();
  void UpdateParkOutDirection();
  void UpdateSlots();
  void UpdateUssDistance();

  // todo: obstacle update can move to parking obstacle manager
  void UpdateObstacles();
  void UpdateFuisonObs();
  void UpdateGroundLineObs();
  void UpdateUssObs();

  std::shared_ptr<ApaData> apa_data_ptr_;

  std::shared_ptr<SlotManager> slot_manager_ptr_;
  std::shared_ptr<UssObstacleAvoidance> uss_obstacle_avoider_ptr_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_;
  std::shared_ptr<LateralPathOptimizer> lateral_path_optimizer_ptr_;

  const LocalView* local_view_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif
