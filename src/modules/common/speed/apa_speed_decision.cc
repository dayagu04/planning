#include "apa_speed_decision.h"

#include "pose2d.h"

namespace planning {

#define DECIDER_DEBUG (0)

const ParkLonDecision* GetCloseStopDecision(
    const SpeedDecisions* speed_decisions) {
  if (speed_decisions == nullptr || speed_decisions->decisions.empty()) {
    return nullptr;
  }

  const ParkLonDecision* stop_decision = nullptr;
  for (size_t i = 0; i < speed_decisions->decisions.size(); i++) {
    const ParkLonDecision* tmp_decision = &speed_decisions->decisions[i];
    if (tmp_decision->decision_type != LonDecisionType::STOP) {
      continue;
    }

    if (stop_decision == nullptr) {
      stop_decision = tmp_decision;
    } else if (tmp_decision->path_s - tmp_decision->lon_decision_buffer <
               stop_decision->path_s - stop_decision->lon_decision_buffer) {
      stop_decision = &speed_decisions->decisions[i];
    }

#if DECIDER_DEBUG
    ILOG_INFO << "s = " << stop_decision->path_s
              << ",reason = " << static_cast<int>(stop_decision->reason_code)
              << ", buffer = " << stop_decision->lon_decision_buffer;
#endif
  }

  return stop_decision;
}
}  // namespace planning