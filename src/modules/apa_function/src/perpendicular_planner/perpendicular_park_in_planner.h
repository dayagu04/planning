#ifndef __PERPENDICULAR_PARK_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_IN_PLANNER_H__

#include <cstdint>
#include <memory>

#include "apa_plan_base.h"
#include "apa_world.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "perpendicular_path_planner.h"

namespace planning {
namespace apa_planner {

class PerpendicularInPlanner : public ApaPlannerBase {
 public:
  PerpendicularInPlanner() = default;
  PerpendicularInPlanner(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
    const bool c_ilqr_enable = false;
    Init(c_ilqr_enable);
  }
  virtual void Reset() override;
  virtual void Update() override;
  virtual std::string GetName() override { return typeid(this).name(); }

 private:
  void PlanCore();
  void GenTlane();
  void GenObstacles();
  virtual void Log() const override;
  virtual void GenPlanningOutput() override;
  virtual void GenPlanningPath() override;
  void SetParkingStatus(uint8_t status);
  const bool UpdateEgoSlotInfo();
  const uint8_t PathPlanOnce();

  // virtual func
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;
  virtual const bool CheckStuckFailed() override;
  virtual void UpdateRemainDist() override;
  virtual const double CalRemainDistFromPath() override;
  virtual const double CalRemainDistFromUss() override;
  virtual const bool PostProcessPath() override;

  const bool PostProcessPathAccordingLimiter();

  const bool PostProcessPathAccordingObs(const double& car_remain_dist);

  const bool CheckPaused();

  const bool CheckSegCompleted();
  const bool CheckDynamicUpdate();
  const uint8_t CheckParkingStatus();

  void InitSimulation();
  void PrepareSimulation();
  const bool CheckPlanSkip() const;

  planning::apa_planner::PerpendicularPathPlanner::Tlane slot_t_lane_;
  planning::apa_planner::PerpendicularPathPlanner::Tlane obstacle_t_lane_;
  PerpendicularPathPlanner perpendicular_path_planner_;

  uint8_t gear_command_ = 0;
  std::vector<pnc::geometry_lib::PathPoint> current_path_point_global_vec_;

  std::vector<pnc::geometry_lib::PathSegment> first_reverse_path_vec_;
  Eigen::Vector2d pt_center_;
  Eigen::Vector2d pt_center_replan_;
  double pt_center_heading_replan_;
  double pt_center_replan_jump_dist_ = 0.0;
  double pt_center_replan_jump_heading_ = 0.0;

  bool trim_path_by_obs_ = false;

  double max_target_velocity_ = 0.0;
};

}  // namespace apa_planner
}  // namespace planning

#endif
