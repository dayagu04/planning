#include "apa_planner/apa_planner_base.h"
#include "context/environmental_model.h"
#include "context/planning_output_context.h"

namespace planning {
namespace apa_planner {

using framework::Frame;

void ApaPlannerBase::SetFailedPlanningOutput(Frame* const frame) const {
  auto planning_output_context =
      frame->mutable_session()->mutable_planning_output_context();
  auto planning_output = &(planning_output_context->mutable_planning_status()\
      ->planning_result.planning_output);

  planning_output->mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::FAILED);
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  trajectory->mutable_trajectory_points()->Clear();
  auto gear_command = planning_output->mutable_gear_command();
  gear_command->set_available(true);
  gear_command->set_gear_command_value(
      Common::GearCommandValue::GEAR_COMMAND_VALUE_PARKING);
  const auto& pose = frame->session()->environmental_model().\
      get_local_view().localization_estimate.pose();
  PlanningOutput::TrajectoryPoint *trajectory_point =
      trajectory->add_trajectory_points();
  trajectory_point->set_x(pose.local_position().x());
  trajectory_point->set_y(pose.local_position().y());
  trajectory_point->set_heading_yaw(pose.euler_angles().yaw());
  trajectory_point->set_curvature(0.0);
  trajectory_point->set_t(0.0);
  trajectory_point->set_v(0.0);
  trajectory_point->set_a(0.0);
  trajectory_point->set_distance(0.0);
  trajectory_point->set_jerk(0.0);
}

} // namespace apa_planner
} // namespace planning
