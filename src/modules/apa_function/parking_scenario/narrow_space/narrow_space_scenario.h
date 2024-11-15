#ifndef __HYBRID_ASTAR_PARK_H__
#define __HYBRID_ASTAR_PARK_H__

#include <cstddef>
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "hybrid_astar_interface.h"
#include "hybrid_astar_thread.h"
#include "src/modules/apa_function/parking_task/deciders/virtual_wall_decider.h"

namespace planning {
namespace apa_planner {

// TODO: 目前默认几何规划无解的场景，就是狭窄场景，调用hybrid astar即可.
// 后续需要在普通场景中，调用A star.
class NarrowSpaceScenario : public ParkingScenario {
 public:
  NarrowSpaceScenario() = default;

  NarrowSpaceScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr);

  ~NarrowSpaceScenario();

  void Init() override;

  void Reset() override;

  virtual std::string GetName() override { return typeid(this).name(); }

  HybridAStarThreadSolver* GetThread() { return &thread_; }

  const size_t GetPathCollisionID() const { return path_collision_id_; }

  const bool IsPathCollision() const { return is_path_collision_; }

 private:
  virtual const bool CheckReplan() override;

  virtual const bool CheckFinished() override;

  const bool CheckVerticalSlotFinished();

  const bool CheckParallelSlotFinished();

  virtual void PlanCore() override;

  virtual void Log() const override;

  virtual void GenTlane() override;

  virtual void GenObstacles() override;

  virtual const uint8_t PathPlanOnce() override;

  const bool CheckStuckFailed() override;

  void UpdateRemainDist(const double uss_safe_dist) override;

  const std::string GetPlanReason(const uint8_t type);

  /**
   *      upper-left system, system is from slot space
   *                   x
   *                     ^
   *                     |
   *                     |
   *                     |
   *                     |
   *                     |
   *                     |
   *          y  ________|
   *
   */
  PathPlannerResult PlanBySearchBasedMethod();

  const int PathOptimizationByCILRQ(
      const double paht_s,
      const std::vector<pnc::geometry_lib::PathPoint>& local_path,
      Transform2d* tf);

  const int GenerateFallBackPath();

  const int LocalPathToGlobal(
      const std::vector<pnc::geometry_lib::PathPoint>& local_path,
      Transform2d* tf);

  const int UpdatePreparePlanFlag(const bool prepare_success);

  const int PublishHybridAstarDebugInfo(const HybridAStarResult& result,
                                        HybridAStarThreadSolver* thread,
                                        Transform2d* tf);

  const int HybridAstarDebugInfoClear();

  const bool UpdateThreadPath();

  const bool UpdateEgoSlotInfo() override;

  const bool UpdateVerticalSlotInfo();

  const bool CheckSegCompleted();

  const bool CheckUssStucked();

  void ShrinkPathByFusionObj();

  void PathShrinkBySlotLimiter();

  void PathExpansionBySlotLimiter();

  const bool CheckEgoReplanNumber(const bool is_replan);

  const bool IsEgoNeedDriveForwardInSlot(const Pose2D& ego_pose,
                                         const double slot_width,
                                         const double slot_len);

  const double CalRemainDistFromPath() override;

  size_t GetNearestPathPoint(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& pose);

  void DebugPathString(const std::vector<pnc::geometry_lib::PathPoint>& path);

  const bool UpdateParallelSlotInfo();

  RequestResponseState thread_state_;
  HybridAStarThreadSolver thread_;

  bool is_ego_collision_;
  bool is_path_collision_;
  size_t path_collision_id_;

  AstarPathGear current_gear_;
  int in_slot_car_adjust_count_;
  bool is_path_single_shot_to_goal_;

  SlotRelativePosition slot_side_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
