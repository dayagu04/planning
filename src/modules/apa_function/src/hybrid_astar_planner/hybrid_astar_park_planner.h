#ifndef __HYBRID_ASTAR_PARK_H__
#define __HYBRID_ASTAR_PARK_H__

#include <cstddef>
#include "apa_plan_base.h"
#include "hybrid_astar_interface.h"
#include "hybrid_astar_thread.h"

namespace planning {
namespace apa_planner {

class HybridAStarParkPlanner : public ApaPlannerBase {
 public:
  HybridAStarParkPlanner() = default;
  HybridAStarParkPlanner(const std::shared_ptr<ApaWorld>& apa_world_ptr);
  ~HybridAStarParkPlanner();

  void Init() override;

  void Reset() override;

  virtual std::string GetName() override { return typeid(this).name(); }

  HybridAStarThreadSolver* GetThread() { return &thread_; }

  const size_t GetPathCollisionID() const { return path_collision_id_; }

  const bool IsPathCollision() const { return is_path_collision_; }

 private:
  virtual const bool CheckReplan() override;

  virtual const bool CheckFinished() override;

  virtual void PlanCore() override;

  virtual void Log() const override;

  virtual void GenTlane() override;

  virtual void GenObstacles() override;

  virtual const uint8_t PathPlanOnce() override;

  void UpdateRemainDist() override;

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
  ApaPlannerBase::PathPlannerResult PlanBySearchBasedMethod();

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

  const bool CheckSegCompleted();

  const bool CheckUssStucked();

  void ShrinkPathByFusionObj();

  RequestResponseState thread_state_;
  HybridAStarThreadSolver thread_;

  bool is_ego_collision_;
  bool is_path_collision_;
  size_t path_collision_id_;

  AstarPathGear current_gear_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
