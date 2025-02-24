#ifndef __PERPENDICULAR_PARK_OUT_PLANNER_H__
#define __PERPENDICULAR_PARK_OUT_PLANNER_H__

#include "perpendicular_head_out_path_generator.h"
#include "perpendicular_park_scenario.h"

namespace planning {
namespace apa_planner {

class PerpendicularHeadOutScenario : public PerpendicularParkScenario {
 public:
  PerpendicularHeadOutScenario() = default;
  PerpendicularHeadOutScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

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
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const bool GenTlane() override;
  virtual const bool GenObstacles() override;
  virtual void ExcutePathPlanningTask() override;
  virtual void Log() const override;
  virtual const bool CheckFinished() override;

  const SlotObsType CalSlotObsType(const Eigen::Vector2d& obs_slot);
  const bool CheckSecurityCurrentpath();
  const bool CheckRationalityEndpointPosition();
  const bool CurrentPathTrimmed();
 private:
  PerpendicularPathOutPlanner perpendicular_path_planner_;
  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;
  bool path_trim_flag_ = false;
  bool end_position_correction_flag_ = false;
};

}  // namespace apa_planner
}  // namespace planning

#endif
