#pragma once
#include <cstdint>
#include <unordered_map>

namespace planning {

struct AgentHeadwayInfo {
  double current_headway = 1.2;
};

class AgentHeadwayDeciderOutput {
 public:
  AgentHeadwayDeciderOutput() = default;
  ~AgentHeadwayDeciderOutput() = default;

  void Reset();

  const std::unordered_map<int32_t, AgentHeadwayInfo>& agents_headway_Info()
      const;
  void set_agents_headway_Info(
      const std::unordered_map<int32_t, AgentHeadwayInfo>& agents_headway_map);

 private:
  std::unordered_map<int32_t, AgentHeadwayInfo> agents_headway_map_;
};

}  // namespace planning
