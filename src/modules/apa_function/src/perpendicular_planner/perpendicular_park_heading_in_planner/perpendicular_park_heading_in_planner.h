#ifndef __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__

#include "perpendicular_park_planner.h"
#include "perpendicular_path_heading_in_planner.h"

namespace planning {
namespace apa_planner {

class PerpendicularParkHeadingInPlanner : public PerpendicularParkPlanner {
 public:
  PerpendicularParkHeadingInPlanner() = default;
  PerpendicularParkHeadingInPlanner(
      const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

 private:
  void PlanCore() override;
  void GenTlane() override;
  void GenObstacles() override;
  virtual void Log() const override;
  virtual void GenPlanningOutput() override;
  virtual void GenPlanningPath() override;
  const bool UpdateEgoSlotInfo() override;
  const uint8_t PathPlanOnce() override;
  void RealTimeDynamicColDet(const EgoSlotInfo& ego_slot_info);

  // virtual func
  virtual const double CalRemainDistFromPath() override;
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;
  virtual const bool PostProcessPath() override;
  const bool PostProcessPathAccordingLimiter() override;

  const bool CheckSegCompleted() override;
  const bool CheckUssStucked() override;
  const bool CheckColDetStucked() override;

  const bool CheckDynamicUpdate() override;
  const uint8_t CheckParkingStatus();

 private:
  planning::apa_planner::PerpendicularPathPlanner::Tlane slot_t_lane_;
  planning::apa_planner::PerpendicularPathPlanner::Tlane obstacle_t_lane_;
  PerpendicularPathHeadingInPlanner perpendicular_path_planner_;

  uint8_t gear_command_ = 0;
  std::vector<pnc::geometry_lib::PathPoint> current_path_point_global_vec_;
  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;

  Eigen::Vector2d pt_center_replan_;
  double pt_center_heading_replan_;
  double pt_center_replan_jump_dist_ = 0.0;
  double pt_center_replan_jump_heading_ = 0.0;

  double max_target_velocity_ = 0.0;
};

}  // namespace apa_planner
}  // namespace planning

#endif
