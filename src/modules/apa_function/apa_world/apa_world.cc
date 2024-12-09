#include "apa_world.h"

#include <cstdint>
#include <cstdio>
#include <vector>


#include "apa_obstacle_manager.h"
#include "apa_param_config.h"
#include "apa_predict_path_manager.h"
#include "apa_slot_manager.h"
#include "apa_state_machine_manager.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "slot_management_info.pb.h"

namespace planning {
namespace apa_planner {
void ApaWorld::Init() {
  new_slot_manager_ptr_ = std::make_shared<ApaSlotManager>();
  measure_data_ptr_ = std::make_shared<ApaMeasureDataManager>();
  predict_path_ptr_ = std::make_shared<ApaPredictPathManager>();
  state_machine_ptr_ = std::make_shared<ApaStateMachineManager>();
  obstacle_manager_ptr_ = std::make_shared<ApaObstacleManager>();
  slot_manager_ptr_ = std::make_shared<SlotManager>();
  uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>();
  collision_detector_ptr_ = std::make_shared<CollisionDetector>();
  lateral_path_optimizer_ptr_ = std::make_shared<LateralPathOptimizer>();
}

void ApaWorld::Reset() {
  new_slot_manager_ptr_->Reset();
  state_machine_ptr_->Reset();
  obstacle_manager_ptr_->Reset();
  measure_data_ptr_->Reset();
  predict_path_ptr_->Reset();
  slot_manager_ptr_->Reset();
  uss_obstacle_avoider_ptr_->Reset();
  collision_detector_ptr_->Reset();
  lateral_path_optimizer_ptr_->Reset();
  local_view_ptr_ = nullptr;

  slot_type_ = Common::PARKING_SLOT_TYPE_INVALID;
  slot_id_ = 0;
  is_slot_type_fixed_ = false;
}

const bool ApaWorld::Update() {
  if (local_view_ptr_ == nullptr) {
    ILOG_INFO << "-- apa_world: local view ptr is nullptr, err ---";
    return false;
  }
  ILOG_INFO << "---- apa_world: Update() ---";

  measure_data_ptr_->Update(local_view_ptr_);

  state_machine_ptr_->Update(local_view_ptr_);

  predict_path_ptr_->Update(local_view_ptr_, planning_output_ptr_,
                            measure_data_ptr_);

  obstacle_manager_ptr_->Update(local_view_ptr_);

  new_slot_manager_ptr_->Update(local_view_ptr_, state_machine_ptr_,
                                measure_data_ptr_, obstacle_manager_ptr_);

  // 旧的车位管理 需要等其他模块适配新车位管理后尽快删除
  slot_manager_ptr_->Update(local_view_ptr_, state_machine_ptr_,
                            measure_data_ptr_, obstacle_manager_ptr_);

  if (!is_slot_type_fixed_) {
    slot_type_ = slot_manager_ptr_->GetEgoSlotInfo().slot_type;
    slot_id_ = slot_manager_ptr_->GetEgoSlotInfo().select_slot_id;
  }

  if (state_machine_ptr_->IsSeachingStatus()) {
    is_slot_type_fixed_ = false;
  } else if (state_machine_ptr_->IsParkingStatus()) {
    is_slot_type_fixed_ = true;
  }

  ILOG_INFO << "current slot type in slm ="
            << static_cast<int>(slot_manager_ptr_->GetEgoSlotInfo().slot_type);
  ILOG_INFO << "fixed slot type =" << static_cast<int>(slot_type_);

  return true;
}

const bool ApaWorld::Update(const LocalView* local_view,
                            const iflyauto::PlanningOutput& planning_output) {
  local_view_ptr_ = local_view;
  planning_output_ptr_ = &planning_output;
  return Update();
}

}  // namespace apa_planner
}  // namespace planning
