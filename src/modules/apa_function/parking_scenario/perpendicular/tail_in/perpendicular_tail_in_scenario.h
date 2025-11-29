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
  virtual void Clear() override;
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

  void PathPlan();
  virtual void GenHybridAstarConfigAndRequest(
      PlannerOpenSpaceConfig& config, HybridAStarRequest& request) override;
  virtual const uint8_t PathPlanOnceHybridAstar() override;
  virtual const uint8_t PathPlanOnceHybridAstarThread() override;
  void PathPlanByHybridAstarThread() override;

  const bool PostProcessPathAccordingRemainDist(const double remain_dist);
  const bool CheckShouldStopWhenSlotJumpsMuch();
  const bool CheckDynamicPlanPathOptimal(
      const size_t old_path_gear_change_count,
      const size_t new_path_gear_change_count,
      const double new_path_final_line_length,
      const std::vector<geometry_lib::PathPoint>& s_turn_path,
      const geometry_lib::PathPoint& old_tar_pose,
      const geometry_lib::PathPoint& new_tar_pose,
      const geometry_lib::PathPoint& real_tar_pose);
  const bool CheckDynamicPlanPathOptimalByGeometryPath();
  const bool CheckDynamicPlanPathOptimalByHybridAstarPath(
      const HybridAstarResponse& response);
  const bool LateralPathOptimize(
      std::vector<geometry_lib::PathPoint>& optimal_path_vec);
  const SlotObsType CalSlotObsType(const Eigen::Vector2d& obs_slot);
  virtual const double CalRealTimeBrakeDist() override;
  const bool CalcPtInside();
  const bool CheckCanDelObsInSlot();

  void CalcProjPtForDynamicPlan(
      geometry_lib::PathPoint& proj_pt,
      std::vector<geometry_lib::PathPoint>& splicing_pt_vec);

  void SwitchProcessObsMethod();

  void CalSlotJumpErr();

  const double CalRemainDistBySlotJump();

  void FillPathPointGlobalFromHybridPath(const HybridAstarResponse& response);

  virtual const bool PostProcessPathAccordingLimiter() override;

  virtual const bool CheckDynamicUpdate() override;

  virtual const bool CheckPathDangerous() override;

  virtual const CarSlotRelationship CalCarSlotRelationship(
      const geometry_lib::PathPoint& cur_pose) override;

  void DecideFoldMirrorCommand();

  void DecideExpandMirrorCommand();

  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;
  std::vector<pnc::geometry_lib::PathSegment> all_plan_path_vec_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
