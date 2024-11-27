#ifndef __PERPENDICULAR_PARK_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_IN_PLANNER_H__

#include <cstdint>
#include <memory>

#include "apa_data.h"
#include "geometry_math.h"
#include "perpendicular_park_scenario.h"
#include "perpendicular_tail_in_path_generator.h"

namespace planning {
namespace apa_planner {

class PerpendicularTailInScenario : public PerpendicularParkScenario {
 public:
  PerpendicularTailInScenario() = default;
  PerpendicularTailInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

  void ScenarioTry() override;

  enum class SlotObsType : uint8_t {
    INSIDE_OBS,
    OUTSIDE_OBS,
    IN_OBS,
    OTHER_OBS,
    DISCARD_OBS,
  };

 private:
  // virtual func
  virtual const uint8_t PathPlanOnce() override;
  const uint8_t NewPathPlanOnce();
  virtual const bool UpdateEgoSlotInfo() override;
  virtual void GenTlane() override;
  virtual void GenObstacles() override;
  virtual void PlanCore() override;
  virtual void Log() const override;
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;

  void PlanCore(ApaSlot& apa_slot);
  void PlanCoreSearching(ApaSlot& apa_slot);
  void PlanCoreParking(ApaSlot& apa_slot);
  void UpdateEgoSlotInfo(ApaSlot& apa_slot);
  void GenTLane(ApaSlot& apa_slot);
  void GenObstacles(ApaSlot& apa_slot);

  const SlotObsType CalSlotObsType(const Eigen::Vector2d& obs_slot);

  virtual const bool PostProcessPathAccordingLimiter() override;
  virtual const bool PostProcessPathAccordingObs(
      const double car_remain_dist) override;
  virtual const bool CheckSegCompleted() override;
  virtual const bool CheckUssStucked() override;
  virtual const bool CheckColDetStucked() override;
  virtual const bool CheckDynamicUpdate() override;

  PerpendicularTailInPathGenerator::Tlane slot_t_lane_;
  PerpendicularTailInPathGenerator::Tlane obstacle_t_lane_;
  PerpendicularTailInPathGenerator perpendicular_path_planner_;

  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;

  Eigen::Vector2d pt_center_replan_;
  double pt_center_heading_replan_;
  double pt_center_replan_jump_dist_ = 0.0;
  double pt_center_replan_jump_heading_ = 0.0;
};

}  // namespace apa_planner
}  // namespace planning

#endif
