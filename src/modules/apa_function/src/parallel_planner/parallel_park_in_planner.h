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
  ParallelParInPlanner(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
    const bool c_ilqr_enable = false;
    Init(c_ilqr_enable);
  }

  virtual void Init(const bool c_ilqr_enable) override;
  virtual void Reset() override;
  virtual void Update() override;
  virtual std::string GetName() override { return typeid(this).name(); };

  const double CalcSlotOccupiedRatio(const Eigen::Vector2d& terminal_err,
                                     const double slot_width,
                                     const bool is_right_side) const;

 private:
  void PlanCore();
  void GenTlane();
  void UpdateTlaneOnceInSlot();
  void GenTBoundaryObstacles();
  void GenObstacles();
  void SetParkingStatus(uint8_t status);
  const bool IsEgoInSlot() const;
  const bool IsEgoInSlot(const pnc::geometry_lib::PathPoint& pose) const;
  const bool UpdateEgoSlotInfo();
  void UpdateSlotRealtime();
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

  const bool CheckPaused();
  const bool CheckSegCompleted();
  const uint8_t CheckParkingStatus();

  void InitSimulation();
  void PrepareSimulation();
  const bool CheckPlanSkip() const;

  ParallelPathPlanner::Tlane t_lane_;
  ParallelPathPlanner parallel_path_planner_;
  uint8_t gear_command_ = 0;

  std::vector<pnc::geometry_lib::PathPoint> current_path_point_global_vec_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
