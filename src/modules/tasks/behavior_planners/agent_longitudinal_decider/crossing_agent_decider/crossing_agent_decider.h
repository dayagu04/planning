#pragma once

#include <memory>

#include "agent/agent.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class CrossingAgentDecider : public Task {
 public:
  explicit CrossingAgentDecider(const EgoPlanningConfigBuilder* config_builder,
                                framework::Session* session);
  virtual ~CrossingAgentDecider() = default;

  bool Execute() override;

  bool Reset();

 private:
  bool MakeYieldToVRUDecision(const agent::Agent* const agent);

  bool ClearVRUIdReverseCrossingMap();

  bool MakeYieldToVehicleDecision(const agent::Agent* const agent);

  bool ConstructVirtualAgentByCrossing(const agent::Agent* const agent,
                                       const bool is_vru,
                                       const double is_vru_reverse_crossing);

  bool CalcDesiredVirtualObsS(const bool is_vru_reverse_crossing,
                              const agent::Agent* const agent,
                              double* desired_virtual_s);

  bool AddVirtualAgentIntoAgentManager();

  CrossingAgentDeciderConfig config_;
  std::unordered_map<int32_t, int32_t>
      vru_id_reverse_crossing_map_;  // VRU_id : position
  std::unordered_map<int32_t, int32_t>
      vehicle_id_reverse_crossing_map_;  // vehicle_id : position
  std::vector<std::unique_ptr<agent::Agent>> virtual_agents_vru_ptr_;
  std::vector<std::unique_ptr<agent::Agent>> virtual_agents_vehicle_ptr_;
};
}  // namespace planning