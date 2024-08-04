#ifndef __LATERAL_MOTION_PLANNING_WEIGHT_H__
#define __LATERAL_MOTION_PLANNING_WEIGHT_H__

#include <cstddef>
#include "ego_planning_config.h"
#include "lateral_motion_planner.pb.h"
#include "virtual_lane.h"

namespace pnc {
namespace lateral_planning {

enum LateralMotionSceneEnum { LANE_KEEP = 0, AVOID, LANE_CHANGE, STATIC_AVOID };

class LateralMotionPlanningWeight {
 public:
  explicit LateralMotionPlanningWeight(
      const planning::LateralMotionPlannerConfig &config);
  ~LateralMotionPlanningWeight() = default;

  void CalculateInitInfo(
      const planning::common::LateralPlanningInput &planning_input);

  void SetLateralMotionWeight(
      const LateralMotionSceneEnum scene,
      planning::common::LateralPlanningInput &planning_input);

  void SetInitDisToRef(const double init_dis_to_ref) {
    init_dis_to_ref_ = init_dis_to_ref;
  }

  void SetInitRefThetaError(const double init_ref_theta_error) {
    init_ref_theta_error_ = init_ref_theta_error;
  }

  void SetEgoVel(const double ego_vel) { ego_vel_ = ego_vel; }

  void SetEgoL(const double ego_l) { ego_l_ = ego_l; }

  void SetLCBackFlag(const bool is_lane_change_back) {
    is_lane_change_back_ = is_lane_change_back;
  }

  double GetConcernedStartQJerk() const { return concerned_start_q_jerk_; }

 private:
  void SetAccJerkBoundByVelocity(
      planning::common::LateralPlanningInput &planning_input);

  void MakeDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

 private:
  planning::LateralMotionPlannerConfig config_;
  LateralMotionSceneEnum lateral_motion_scene_;
  double init_dis_to_ref_;
  double init_ref_theta_error_;
  double concerned_start_q_jerk_;
  double ego_vel_;
  double ego_l_;
  bool is_lane_change_back_;
};

}  // namespace lateral_planning
}  // namespace pnc
#endif