#ifndef __APA_WORLD_H__
#define __APA_WORLD_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>

#include "apa_data.h"
#include "apa_measure_data_manager.h"
#include "apa_predict_path_manager.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/collision_detection.h"
#include "collision_detection/uss_obstacle_avoidance.h"
#include "common.pb.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "slot_management_info.pb.h"
#include "slot_manager.h"
#include "spline.h"
#include "spline_projection.h"

#define APA_COMPARE_PLANNING_TRAJ_POINTS_NUM 26

namespace planning {
namespace apa_planner {

class ApaWorld {
 public:
  ApaWorld() { Init(); }
  ~ApaWorld() {}

  void Init();
  void Reset();
  const bool Update(const LocalView* const local_view,
                    const iflyauto::PlanningOutput& planning_output);
  const bool Update();

  std::shared_ptr<ApaData> GetApaDataPtr() { return apa_data_ptr_; }

  std::shared_ptr<SlotManager> GetSlotManagerPtr() { return slot_manager_ptr_; }

  std::shared_ptr<UssObstacleAvoidance> GetUssObstacleAvoidancePtr() {
    return uss_obstacle_avoider_ptr_;
  }

  std::shared_ptr<CollisionDetector> GetCollisionDetectorPtr() {
    return collision_detector_ptr_;
  }

  std::shared_ptr<LateralPathOptimizer> GetLateralPathOptimizerPtr() {
    return lateral_path_optimizer_ptr_;
  }

  std::shared_ptr<ApaMeasureDataManager> GetMeasureDataManagerPtr() {
    return measure_data_ptr_;
  }

  std::shared_ptr<ApaStateMachineManager> GetStateMachineManagerPtr() {
    return state_machine_ptr_;
  }

  std::shared_ptr<ApaPredictPathManager> GetPredictPathManagerPtr() {
    return predict_path_ptr_;
  }

  const LocalView* GetLocalViewPtr() { return local_view_ptr_; }

 private:
  void Preprocess();
  void UpdateEgoState();
  void UpdateSlots();
  void UpdateUssDistance();

  // todo: obstacle update can move to parking obstacle manager
  void UpdateObstacles();
  void UpdateFuisonObs();
  void UpdateGroundLineObs();
  void UpdateUssObs();


  std::shared_ptr<ApaData> apa_data_ptr_;
  std::shared_ptr<ApaPredictPathManager> predict_path_ptr_;
  std::shared_ptr<ApaMeasureDataManager> measure_data_ptr_;
  std::shared_ptr<ApaStateMachineManager> state_machine_ptr_;
  std::shared_ptr<SlotManager> slot_manager_ptr_;
  std::shared_ptr<UssObstacleAvoidance> uss_obstacle_avoider_ptr_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_;
  std::shared_ptr<LateralPathOptimizer> lateral_path_optimizer_ptr_;

  const LocalView* local_view_ptr_ = nullptr;
  const iflyauto::PlanningOutput* planning_output_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif
