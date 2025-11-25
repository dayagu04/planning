#pragma once
#include <cstddef>
#include <vector>

#include "config/basic_type.h"
#include "ilqr_core.h"
#include "joint_motion_planner.pb.h"
#include "joint_motion_planning_cost.h"
#include "joint_motion_planning_model.h"
#include "math_lib.h"
#include "utils/kd_path.h"
namespace pnc {
namespace joint_motion_planning {
class JointMotionPlanningProblem {
 public:
  JointMotionPlanningProblem() = default;
  virtual ~JointMotionPlanningProblem() = default;
  void Init();
  uint8_t Update(planning::common::JointMotionPlanningInput& planning_input);
  const planning::common::JointMotionPlanningOutput& GetOutput() const {
    return planning_output_;
  }
  void SetBoundaryPaths(
      std::shared_ptr<planning::planning_math::KDPath> road_left,
      std::shared_ptr<planning::planning_math::KDPath> road_right);
  void SetMaxIter(size_t max_iter) { ilqr_core_ptr_->SetMaxIter(max_iter); }
  const std::shared_ptr<ilqr_solver::iLqr> GetiLqrCorePtr() const {
    return ilqr_core_ptr_;
  }

 private:
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  planning::common::JointMotionPlanningOutput planning_output_;
  ilqr_solver::State init_state_;
  ilqr_solver::ControlVec u_vec_;
  int32_t obs_num_ = 0;
  std::shared_ptr<EgoRoadBoundaryCostTerm> ego_road_boundary_cost_term_;
};
}  // namespace joint_motion_planning
}  // namespace pnc