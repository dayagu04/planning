#ifndef __APA_WORLD_H__
#define __APA_WORLD_H__

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>

#include "apa_measure_data_manager.h"
#include "apa_obstacle_manager.h"
#include "apa_predict_path_manager.h"
#include "apa_slot_manager.h"
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

#define APA_COMPARE_PLANNING_TRAJ_POINTS_MAX_NUM 26

namespace planning {
namespace apa_planner {

struct SimulationParam {
  int force_mid_process_plan = 0;
  bool is_simulation = false;
  int plan_type = 0;
  bool is_complete_path = false;
  bool force_plan = false;
  bool sim_to_target = false;
  bool use_slot_in_bag = true;
  bool use_obs_in_bag = true;
  bool is_path_optimization = false;
  bool is_cilqr_optimization = false;
  bool is_reset = false;
  double sample_ds = 0.02;
  std::vector<double> target_managed_slot_x_vec;
  std::vector<double> target_managed_slot_y_vec;
  std::vector<double> target_managed_limiter_x_vec;
  std::vector<double> target_managed_limiter_y_vec;
  std::vector<double> obs_x_vec;
  std::vector<double> obs_y_vec;

  double q_ref_xy = 100.0;
  double q_ref_theta = 100.0;
  double q_terminal_xy = 1000.0;
  double q_terminal_theta = 9000.0;
  double q_k = 10.0;
  double q_u = 10.0;
  double q_k_bound = 100.0;
  double q_u_bound = 50.0;
};

class ApaWorld {
 public:
  ApaWorld() { Init(); }
  ~ApaWorld() {}

  void Init();
  void Reset();
  const bool Update(const LocalView* const local_view,
                    const iflyauto::PlanningOutput& planning_output);
  const bool Update();

  std::shared_ptr<SlotManager> GetRetiredSlotManagerPtr() {
    return retired_slot_manager_ptr_;
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

  std::shared_ptr<ApaMeasureDataManager> GetMeasureDataManagerPtr() {
    return measure_data_ptr_;
  }

  std::shared_ptr<ApaStateMachineManager> GetStateMachineManagerPtr() {
    return state_machine_ptr_;
  }

  std::shared_ptr<ApaPredictPathManager> GetPredictPathManagerPtr() {
    return predict_path_ptr_;
  }

  std::shared_ptr<ApaObstacleManager> GetObstacleManagerPtr() {
    return obstacle_manager_ptr_;
  }

  std::shared_ptr<ApaSlotManager> GetSlotManagerPtr() {
    return slot_manager_ptr_;
  }

  const LocalView* GetLocalViewPtr() { return local_view_ptr_; }

  const SimulationParam& GetSimuParam() { return simu_param_; }
  void SetSimuParam(const SimulationParam& simu_param) {
    simu_param_ = simu_param;
  }

 private:
  std::shared_ptr<ApaSlotManager> slot_manager_ptr_;
  std::shared_ptr<ApaPredictPathManager> predict_path_ptr_;
  std::shared_ptr<ApaMeasureDataManager> measure_data_ptr_;
  std::shared_ptr<ApaStateMachineManager> state_machine_ptr_;
  std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr_;
  // will be retired
  std::shared_ptr<SlotManager> retired_slot_manager_ptr_;
  std::shared_ptr<UssObstacleAvoidance> uss_obstacle_avoider_ptr_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_;
  std::shared_ptr<LateralPathOptimizer> lateral_path_optimizer_ptr_;

  SimulationParam simu_param_;
  const LocalView* local_view_ptr_ = nullptr;
  const iflyauto::PlanningOutput* planning_output_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif
