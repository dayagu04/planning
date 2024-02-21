#ifndef __PARALLEL_PARK_IN_PLANNER_H__
#define __PARALLEL_PARK_IN_PLANNER_H__

#include <cstdint>
#include <memory>

#include "apa_plan_base.h"
#include "apa_world.h"
#include "dubins_lib.h"
#include "parallel_path_planner.h"

namespace planning {
namespace apa_planner {

class ParallelParInPlanner : public ApaPlannerBase {
 public:
  ParallelParInPlanner() = default;
  ParallelParInPlanner(const std::shared_ptr<ApaWorld> &apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
    Init();
  }

  virtual void Init() override;
  virtual void Reset() override;
  virtual void Update() override;
  virtual std::string GetName() override { return typeid(this).name(); }

 private:
  void PlanCore();
  void GenTlane();
  void GenObstacles();
  void UpdateObstacles();
  void SetParkingStatus(uint8_t status);
  const bool UpdateEgoSlotInfo();
  const uint8_t PathPlanOnce();

  virtual void Log() const override;
  virtual void GenPlanningOutput() override;
  virtual void GenPlanningPath() override;
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;
  virtual const bool CheckStuckFailed() override;
  virtual void UpdateRemainDist() override;
  virtual const double CalRemainDistFromPath() override;
  virtual const double CalRemainDistFromUss() override;
  virtual const bool PostProcessPath() override;

  const bool CheckSegCompleted();
  const uint8_t CheckParkingStatus();

  void InitSimulation();
  void PrepareSimulation();
  const bool CheckPlanSkip() const;

  planning::apa_planner::ParallelPathPlanner::Tlane t_lane_;
  ParallelPathPlanner parallel_path_planner_;

  uint8_t gear_command_ = 0;

  std::vector<pnc::geometry_lib::PathPoint> current_path_point_global_vec_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
