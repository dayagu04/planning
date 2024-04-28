#include "apa_function.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "apa_world.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "plan_data.h"
#include "planning_context.h"
#include "planning_plan_c.h"

namespace planning {

ApaFunction::ApaFunction(framework::Session *session) : BaseFunction(session) {
  Init();
}

void ApaFunction::Init() {
  // init apa planner interface
  apa_plan_interface_ = std::make_unique<apa_planner::ApaPlanInterface>();

  apa_plan_interface_->Init();
}

bool ApaFunction::Reset() {
  // reset apa planner
  apa_plan_interface_->Reset();

  return true;
}

bool ApaFunction::Plan() {
  const double start_timestamp_ms = IflyTime::Now_ms();

  const bool sync_param_flag =
      (last_planner_interface_type_ != session_->get_scene_type());
  last_planner_interface_type_ = session_->get_scene_type();

  const bool success = apa_plan_interface_->Update(
      &(session_->environmental_model().get_local_view()));
  // set planning output
  session_->mutable_planning_context()->mutable_planning_output() =
      apa_plan_interface_->GetPlaningOutput();

  // sync param
  if (sync_param_flag) {
    apa_plan_interface_->SyncParameters();
  }

  const auto end_timestamp_ms = IflyTime::Now_ms();
  const auto frame_duration = end_timestamp_ms - start_timestamp_ms;

  DEBUG_PRINT("time_consumption = " << frame_duration << "ms");

  return true;
}

}  // namespace planning