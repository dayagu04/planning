#pragma once

#include <cstdint>

#include "agent/agent.h"
namespace planning {

class StopDestinationDeciderOutput {
 public:
  StopDestinationDeciderOutput() = default;
  ~StopDestinationDeciderOutput() = default;

  // const std::map<std::string, cp_common::agent::Agent>&
  // get_virtual_obstacles()
  //     const;
  int32_t& mutable_stop_destination_virtual_agent_id() {
    return stop_destination_virtual_agent_id_;
  };

  const int32_t stop_destination_virtual_agent_id() const {
    return stop_destination_virtual_agent_id_;
  };

  double& mutable_stop_destination_virtual_agent_time_headway() {
    return stop_destination_virtual_agent_time_headway_;
  };

  const double stop_destination_virtual_agent_time_headway() const {
    return stop_destination_virtual_agent_time_headway_;
  };

 private:
  int32_t stop_destination_virtual_agent_id_ =
      agent::AgentDefaultInfo::kNoAgentId;
  double stop_destination_virtual_agent_time_headway_ = 1.0;
};

}  // namespace planning
