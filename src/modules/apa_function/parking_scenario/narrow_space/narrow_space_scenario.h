#ifndef __HYBRID_ASTAR_PARK_H__
#define __HYBRID_ASTAR_PARK_H__

#include <cstddef>
#include <cstdint>

#include "hybrid_astar_interface.h"
#include "hybrid_astar_thread.h"
#include "narrow_space_decider.h"
#include "planning_hmi_c.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "virtual_wall_decider.h"

namespace planning {
namespace apa_planner {

// next scenario use search based method:
// geometry no solution scenario;
// head in scenario;
// head out scenario;
// tail out scenario;
// TODO:
// parallel in;
// parallel out;
class NarrowSpaceScenario : public ParkingScenario {
 public:
  NarrowSpaceScenario() = default;

  NarrowSpaceScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr);

  ~NarrowSpaceScenario();

  void Init() override;

  void Reset() override;

  void ThreadClearState() override;

  virtual std::string GetName() override { return typeid(this).name(); }

  HybridAStarThreadSolver* GetThread() { return &thread_; }

  void ScenarioTry() override;

 private:
  virtual const bool CheckFinished() override;

  const bool CheckVerticalSlotFinished();

  const bool CheckParallelSlotFinished();

  const bool CheckHeadOutFinished();

  virtual void ExcutePathPlanningTask() override;

  virtual void Log() const override;

  virtual const bool GenTlane() override;

  virtual const bool GenObstacles() override;

  virtual const uint8_t PathPlanOnce() override;

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

  const int PathOptimizationByCILQR(
      const std::vector<AStarPathPoint>& first_seg_path, Transform2d* tf);

  const void GenerateFallBackPath();

  const int LocalPathToGlobal(
      const std::vector<pnc::geometry_lib::PathPoint>& local_path,
      Transform2d* tf);

  const int UpdatePreparePlanFlag(const bool prepare_success);

  const int PublishHybridAstarDebugInfo(const HybridAStarResult& result,
                                        Transform2d* tf);

  const int HybridAstarDebugInfoClear();

  const bool UpdateThreadPath();

  const bool UpdateEgoSlotInfo() override;

  const bool UpdateVerticalSlotInfo();

  const bool UpdateVerticalOutSlotInfo();

  const bool CheckSegCompleted();

  const bool CheckUssStucked();

  void PathShrinkBySlotLimiter();

  void PathExpansionBySlotLimiter();

  const bool CheckEgoReplanNumber(const bool is_replan);

  const double CalRemainDistFromPath() override;

  size_t GetNearestPathPoint(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& pose);

  const bool UpdateParallelSlotInfo();

  const bool NeedBlindZonePlanning(const EgoInfoUnderSlot& ego_info);

  const cdl::AABB GenerateBlindZoneSlotBox(
      const EgoInfoUnderSlot& ego_info) const;

  // check path replan by slot pose change
  const bool CheckDynamicUpdate() override;

  const bool CheckDynamicHeadOut();

  const bool ReplanBySlotRefresh();

  void FillPlanningReason(AstarRequest& cur_request);

  void FillGearRequest(const bool is_scenario_try, AstarRequest& cur_request);

  void FillPlanningMethod(const bool is_scenario_try,
                          AstarRequest& cur_request);

  ParkingVehDirection GetDirection();

  ParkSpaceType GetSlotType(const SlotType slot_type);

  double GetStraightLength(const ParkingVehDirection dir,
                           const ParkSpaceType slot_type);

  Pose2D GenerateStitchPoint();

  void RecordSearchNode(Transform2d* tf);

  const PathPlannerResult PubResponseForScenarioRunning(
      const EgoInfoUnderSlot& ego_info, const AstarRequest& cur_request,
      const ParkObstacleList& obs);

  const PathPlannerResult PubResponseForScenarioTry(
      const EgoInfoUnderSlot& ego_info, const AstarRequest& cur_request,
      const ParkObstacleList& obs);

  const pnc::geometry_lib::PathSegGear GetGear(const AstarPathGear gear);

  const bool IsNeedClipping(const HybridAStarResult& result, const size_t i);
  iflyauto::APAHMIData PubDirectionForParkOutTry(
      const AstarRequest& cur_request);

  void SetRequestForParkOutTry(AstarRequest& cur_request,
                               const EgoInfoUnderSlot& ego_info);

  void UpdateRecommentRouteBox();

  void RecordSearchTime(const double time);

 private:
  RequestResponseState thread_state_;
  HybridAStarThreadSolver thread_;
  // do not clear it every frame in cruise state.
  AstarResponse response_;

  AstarPathGear current_gear_;
  int replan_number_inside_slot_;
  // If path connected with goal, and no gear switch, True.
  bool is_path_connected_to_goal_;

  // offset to slot center.
  double lateral_offset_;
  double lon_offset_;

  // used by park out
  double current_path_last_heading_;
  bool dynamic_flag_head_out_;

  // 一个车位泊车中，通道虚拟墙只能增长，不能缩减.
  // 如果根据车辆位置去缩减，导致2次规划之间路径差异太大.
  VirtualWallDecider virtual_wall_decider_;

  NarrowScenarioDecider narrow_space_decider_;

  // Allow path planning fail one time.
  // If fail in history planning, consider blind zone, delete obstacles in blind
  // zone.
  int path_planning_fail_num_;

  MapBound recommend_route_bound_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
