#ifndef __HYBRID_ASTAR_PARK_H__
#define __HYBRID_ASTAR_PARK_H__

#include <cstddef>
#include "hybrid_astar_interface.h"
#include "hybrid_astar_thread.h"
#include "narrow_space_decider.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "virtual_wall_decider.h"

namespace planning {
namespace apa_planner {

// TODO: 默认几何规划无解的场景，就是狭窄场景，调用hybrid astar即可.
// 后续需要在普通场景中，调用A star.
// 后续需要普通场景优先调用A星时，再删除narrow space
// 场景. 直接在正常场景中调用混合A星算法.
class NarrowSpaceScenario : public ParkingScenario {
 public:
  NarrowSpaceScenario() = default;

  NarrowSpaceScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr);

  ~NarrowSpaceScenario();

  void Init() override;

  void Reset() override;

  void ThreadClear() override;

  virtual std::string GetName() override { return typeid(this).name(); }

  HybridAStarThreadSolver* GetThread() { return &thread_; }

  void ScenarioTry() override;

  VirtualWallDecider* MutableVirtualWallDecider() {
    return &virtual_wall_decider_;
  }

 private:
  virtual const bool CheckFinished() override;

  const bool CheckVerticalSlotFinished();

  const bool CheckParallelSlotFinished();

  virtual void ExcutePathPlanningTask() override;

  virtual void Log() const override;

  virtual const bool GenTlane() override;

  virtual const bool GenObstacles() override;

  virtual const uint8_t PathPlanOnce() override;

  void UpdateRemainDist(
      const double uss_safe_dist,
      const double lat_buffer = apa_param.GetParam().lat_inflation,
      const double extra_buffer_when_reversing = 0.068) override;

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
  PathPlannerResult PlanBySearchBasedMethod(const bool is_scenario_try);

  const int PathOptimizationByCILRQ(
      const std::vector<pnc::geometry_lib::PathPoint>& local_path,
      Transform2d* tf);

  const void GenerateFallBackPath();

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

  void PathShrinkBySlotLimiter();

  void PathExpansionBySlotLimiter();

  const bool CheckEgoReplanNumber(const bool is_replan);

  const bool IsEgoNeedAdjustInSlot(const Pose2D& ego_pose,
                                   const double slot_width,
                                   const double slot_len);

  const double CalRemainDistFromPath() override;

  size_t GetNearestPathPoint(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& pose);

  void DebugPathString(const std::vector<pnc::geometry_lib::PathPoint>& path);

  const bool UpdateParallelSlotInfo();

  // ego_start: slot coordination for ego
  // If ego is not cross with slot line, parking is finished.
  const bool IsVehicleOverlapWithSlotLine(const double slot_length,
                                          const double slot_width,
                                          const Pose2D& ego_start);

 private:
  RequestResponseState thread_state_;
  HybridAStarThreadSolver thread_;

  AstarPathGear current_gear_;
  // 库内揉库次数
  int in_slot_car_adjust_count_;
  // path connected with goal
  bool is_path_connected_to_goal_;

  // 一个车位泊车中，通道虚拟墙只能增长，不能缩减.
  // 如果根据车辆位置去缩减，导致2次规划之间路径差异太大.
  VirtualWallDecider virtual_wall_decider_;

  NarrowScenarioDecider narrow_space_decider_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
