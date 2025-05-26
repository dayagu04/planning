#ifndef __PERPENDICULAR_PARK_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_IN_PLANNER_H__

#include <cstdint>
#include <memory>

#include "geometry_math.h"
#include "perpendicular_park_scenario.h"
#include "perpendicular_tail_in_path_generator.h"
#include "target_pose_decider/target_pose_decider.h"

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
  virtual void ExcutePathPlanningTask() override;
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const bool GenTlane() override;
  virtual const bool GenObstacles() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual void Log() const override;
  virtual const bool CheckFinished() override;

  void PathPlanByGeometry();
  const bool PostProcessPathAccordingRemainDist(const double remain_dist);
  const bool CheckShouldStopWhenSlotJumpsMuch();
  const bool CheckDynamicPlanPathOptimal();
  const bool LateralPathOptimize(
      std::vector<geometry_lib::PathPoint>& optimal_path_vec);
  const SlotObsType CalSlotObsType(const Eigen::Vector2d& obs_slot);
  const double CalRealTimeBrakeDist();
  const bool CalcPtInside();
  const bool CheckCanDelObsInSlot();

  void CalcProjPtForDynamicPlan(
      geometry_lib::PathPoint& proj_pt,
      std::vector<geometry_lib::PathPoint>& splicing_pt_vec);

  void SwitchProcessObsMethod();

  void CalRemainDistBySlotJump();

  virtual const bool PostProcessPathAccordingLimiter() override;

  virtual const bool CheckDynamicUpdate() override;

  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;
  std::vector<pnc::geometry_lib::PathSegment> all_plan_path_vec_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
