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
  // reset apa planner and sync param
  apa_plan_interface_->Reset();
  apa_plan_interface_->SyncParameters();
  return true;
}

bool ApaFunction::Plan() {
  const double start_timestamp_ms = IflyTime::Now_ms();

  const bool success = apa_plan_interface_->Update(
      &(session_->environmental_model().get_local_view()));
  // set planning output
  session_->mutable_planning_context()->mutable_planning_output() =
      apa_plan_interface_->GetPlaningOutput();

  const auto end_timestamp_ms = IflyTime::Now_ms();
  const auto frame_duration = end_timestamp_ms - start_timestamp_ms;

  DEBUG_PRINT("time_consumption = " << frame_duration << "ms");
  JSON_DEBUG_VALUE("total_plan_consume_time", frame_duration)

  return true;
}

}  // namespace planning