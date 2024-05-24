#ifndef __LATERAL_MOTION_PLANNING_WEIGHT_H__
#define __LATERAL_MOTION_PLANNING_WEIGHT_H__

#include <cstddef>
#include "ego_planning_config.h"
#include "lateral_motion_planner.pb.h"
#include "virtual_lane.h"

namespace pnc {
namespace lateral_planning {

enum LateralMotionSceneEnum { LANE_KEEP = 0, AVOID, LANE_CHANGE, BEND };

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

  void SetWeightByEnterAutoTime(const double auto_time, planning::common::LateralPlanningInput &planning_input);

  void SetPosBoundWeightByLane(bool direction, planning::common::LateralPlanningInput &planning_input);

  void SetInitDisToRef(const double init_dis_to_ref) {
    init_dis_to_ref_ = init_dis_to_ref;
  }

  void SetInitRefThetaError(const double init_ref_theta_error) {
    init_ref_theta_error_ = init_ref_theta_error;
  }

  void SetRealVel(const double real_vel) { real_vel_ = real_vel; }

  size_t CalculateConcernedStartIndex();

  double GetConcernedStartRatio() { return concerned_start_ratio_; }

 private:
  void SetAccJerkBoundByVelocity(
      planning::common::LateralPlanningInput &planning_input);

  void SetCloseRangeJerkWeight(
      planning::common::LateralPlanningInput &planning_input);

  void SetJerkWeightByBigBias(
      planning::common::LateralPlanningInput &planning_input);

  void CalculateConcernedStartRatio();

 private:
  planning::LateralMotionPlannerConfig config_;
  LateralMotionSceneEnum lateral_motion_scene_;
  double init_dis_to_ref_;
  double init_ref_theta_error_;
  double concerned_start_ratio_;
  double real_vel_;
};

}  // namespace lateral_planning
}  // namespace pnc
#endif