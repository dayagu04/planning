
#include <memory>

#include "apa_state_machine_manager.h"
#include "slot_manager.h"

namespace planning {
namespace apa_planner {

// 1. 如果用户不点击车位，那么就是基于规则的释放逻辑.
// 基于规则的释放逻辑，可以宽松一些.
// 2. todo：如果用户点击了车位，那么对点击的车位，进行预规划，
// 如果预规划成功，那么车位释放，如果预规划失败，车位不释放.
// 这里预规划可以选用几何或者A星.
// 3. 用户点击了开始泊车，那么进入场景运行
class RuleBasedSlotRelease {
 public:
  RuleBasedSlotRelease() = default;

  void Process(
      const LocalView *local_view, const MeasurementData *measures_ptr,
      const std::shared_ptr<ApaStateMachineManager> state_machine_manger_ptr,
      std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map,
      apa_planner::SlotManager::Frame &frame);

 private:
  //  泊车巡航场景，更新车位释放决策
  void ParkingLotCruiseProcess(
      std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map);

  // 选中了车位，或者点击了泊车
  void ParkingActivateProcess(
      std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map);

  const bool IsEgoCloseToObs(const LocalView *local_view,
                             const MeasurementData *measures_ptr);

  bool IsPassageAreaEnough(const common::SlotInfo *slot);

  const bool IsSlotOccupied(const common::SlotInfo *slot);

  const bool AddUssPerceptObstacles(const common::SlotInfo &slot_info);

  const double CalLonDistSlot2Car(const common::SlotInfo &new_slot_info) const;

  // todo: remove this code
  const bool UpdateEgoSlotInfo(
      apa_planner::SlotManager::EgoSlotInfo &ego_slot_info,
      const common::SlotInfo *slot_info);

  // todo: remove this code
  const bool GenTLane(
      apa_planner::SlotManager::EgoSlotInfo &ego_slot_info,
      apa_planner::PerpendicularTailInPathGenerator::Tlane &slot_tlane,
      apa_planner::PerpendicularTailInPathGenerator::Tlane &obs_tlane);

  // todo: remove this code
  const bool GenObstacles(
      const apa_planner::PerpendicularTailInPathGenerator::Tlane &slot_tlane,
      apa_planner::PerpendicularTailInPathGenerator::Tlane &obs_tlane,
      const apa_planner::SlotManager::EgoSlotInfo &ego_slot_info);

  const bool UpdateEgoParallelSlotInfoInSearching(
      apa_planner::SlotManager::EgoSlotInfo &ego_slot_info,
      const common::SlotInfo *slot_info);

  const bool IsPerpendicularSlotCoarseRelease(
      common::SlotInfo *slot, apa_planner::SlotInfoWindow *slot_history);

  const bool IsParallelSlotCoarseRelease(
      common::SlotInfo *slot, apa_planner::SlotInfoWindow *slot_history);

  // todo: remove this code
  const bool GeometryPathForTailInScenario(
      common::SlotInfo *slot, apa_planner::SlotInfoWindow *slot_history);

 private:
  apa_planner::SlotManager::Frame *frame_;

  const ApaParameters *config_;

  const LocalView *local_view_;
  const MeasurementData *measures_ptr_;

  std::shared_ptr<ApaStateMachineManager> state_machine_manger_ptr_;

  // todo: move to collision detection
  ParkObstacleList obs_list_;
};

}  // namespace apa_planner
}  // namespace planning