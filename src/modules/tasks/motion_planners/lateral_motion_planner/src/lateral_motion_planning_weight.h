#ifndef __LATERAL_MOTION_PLANNING_WEIGHT_H__
#define __LATERAL_MOTION_PLANNING_WEIGHT_H__

#include "ego_planning_config.h"
#include "lateral_motion_planner.pb.h"
#include "virtual_lane.h"

namespace pnc {
namespace lateral_planning {

enum LateralMotionSceneEnum {
  LANE_KEEP = 0,
  AVOID,
  LANE_CHANGE,
  BIG_CURVATURE
};

class LateralMotionPlanningWeight {
 public:
  explicit LateralMotionPlanningWeight(
      const planning::LateralMotionPlannerConfig &config);
  ~LateralMotionPlanningWeight() = default;

  void SetWeightByScene(LateralMotionSceneEnum scene,
                        planning::common::LateralPlanningInput &planning_input);

  void SetWeightByPosBoundAndLaneType(
      std::shared_ptr<VirtualLane> current_lane,
      planning::common::LateralPlanningInput &planning_input);

 private:
  planning::LateralMotionPlannerConfig config_;
};

}  // namespace lateral_planning
}  // namespace pnc
#endif