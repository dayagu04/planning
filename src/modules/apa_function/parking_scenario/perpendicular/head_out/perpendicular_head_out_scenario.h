#ifndef __PERPENDICULAR_PARK_OUT_PLANNER_H__
#define __PERPENDICULAR_PARK_OUT_PLANNER_H__

#include "perpendicular_park_scenario.h"
#include "perpendicular_head_out_path_generator.h"

namespace planning {
namespace apa_planner {

class PerpendicularHeadOutScenario : public PerpendicularParkScenario {
 public:
  PerpendicularHeadOutScenario() = default;
  PerpendicularHeadOutScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

 private:
  // virtual func
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool UpdateEgoSlotInfo() override;
  virtual void GenTlane() override;
  virtual void GenObstacles() override;
  virtual void ExcutePathPlanningTask() override;
  virtual void Log() const override;
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;

  virtual const bool PostProcessPathAccordingObs(
      const double car_remain_dist) override;
  virtual const bool CheckSegCompleted() override;
  virtual const bool CheckUssStucked() override;
  virtual const bool CheckColDetStucked() override;
  // virtual const bool CheckDynamicUpdate() override;
 private:
  PerpendicularPathOutPlanner::Tlane slot_t_lane_;
  PerpendicularPathOutPlanner::Tlane obstacle_t_lane_;
  PerpendicularPathOutPlanner perpendicular_path_planner_;

  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;

  Eigen::Vector2d pt_center_replan_;
  double pt_center_heading_replan_;
  double pt_center_replan_jump_dist_ = 0.0;
  double pt_center_replan_jump_heading_ = 0.0;
};

}  // namespace apa_planner
}  // namespace planning

#endif
