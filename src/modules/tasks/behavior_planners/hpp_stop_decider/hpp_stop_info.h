#pragma once

namespace planning {

struct HppStopInfo {
  // 是否到达终点并停车
  bool is_stopped_at_destination = false;
  
  // 是否到达目标停车位
  bool is_reached_target_slot = false;
  
  // 是否到达目标目的地
  bool is_reached_target_dest = false;
  
  // 是否满足停车条件（距离终点<2m，距离轨迹终点<0.5m，速度<0.1）
  bool is_stop_condition_met = false;
  
  // 停车帧数计数（用于判断是否满足3帧）
  int stop_frame_count = 0;

  void Clear() {
    is_stopped_at_destination = false;
    is_reached_target_slot = false;
    is_reached_target_dest = false;
    is_stop_condition_met = false;
    stop_frame_count = 0;
    return;
  }
};

}  // namespace planning
