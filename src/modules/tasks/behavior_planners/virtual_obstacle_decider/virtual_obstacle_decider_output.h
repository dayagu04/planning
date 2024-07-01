#pragma once

namespace planning {

class VirtualObstacleDeciderOutput {
 public:
  VirtualObstacleDeciderOutput() = default;
  ~VirtualObstacleDeciderOutput() = default;

  // const std::map<std::string, cp_common::agent::Agent>& get_virtual_obstacles()
  //     const;
  void get_multable_virtual_obstacles();

  void set_virtual_obstacle();

 private:
  // std::map<std::string, cp_common::agent::Agent> virtual_obstacles_map_;
};

}  // namespace planning
