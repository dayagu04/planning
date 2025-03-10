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
  return true;
}

bool ApaFunction::Plan() {
  (void)apa_plan_interface_->Update(
      &(session_->environmental_model().get_local_view()),
      &(session_->planning_context().last_planning_result()));
  // set planning output
  session_->mutable_planning_context()->mutable_planning_output() =
      apa_plan_interface_->GetPlaningOutput();

  // currently only remaining distance is sent to planning hmi!
  session_->mutable_planning_context()
      ->mutable_planning_hmi_info()
      ->apa_info.remain_dist = apa_plan_interface_->GetAPAHmi().remain_dist;
  return true;
}

}  // namespace planning