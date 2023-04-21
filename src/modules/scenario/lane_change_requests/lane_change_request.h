#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "src/common/ifly_time.h"
#include "src/framework/session.h"
#include "src/modules/common/config/basic_type.h"
#include "src/modules/scc_function/display_state_types.h"
#include "src/modules/scenario/lane_change_lane_manager.h"

namespace planning {
/// @brief 换道请求的基类，生成、结束换道请求等
class LaneChangeRequest {
 public:
  LaneChangeRequest(
      planning::framework::Session* session,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~LaneChangeRequest() = default;

  void GenerateRequest(RequestType direction);
  void Finish();
  bool AggressiveChange() const;
  bool IsDashedLineEnough(RequestType direction, const double ego_vel,
                          std::shared_ptr<VirtualLaneManager> map_info_mgr);

  RequestType request() const { return request_; }
  int target_lane_virtual_id() { return target_lane_virtual_id_; }
  void set_target_lane_virtual_id(int target_lane_virtual_id) {
    target_lane_virtual_id_ = target_lane_virtual_id;
  }
  RequestType turn_signal() const { return turn_signal_; }
  double tstart() const { return tstart_; }
  double tfinish() const { return tfinish_; }

 protected:
  RequestType request_ = NO_CHANGE;
  int target_lane_virtual_id_ = -1;
  int origin_lane_virtual_id_ = -1;
  int origin_lane_order_id_ = -1;
  RequestType turn_signal_ = NO_CHANGE;
  double tstart_ = 0.0;
  double tfinish_ = 0.0;
  framework::Session* session_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr_;
  double aggressive_lane_change_distance_highway_{0.0};
  double aggressive_lane_change_distance_urban_{150.0};
};

}  // namespace planning
