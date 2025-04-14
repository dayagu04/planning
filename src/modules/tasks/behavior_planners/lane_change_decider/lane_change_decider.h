#pragma once

#include <type_traits>
#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "lane_change_request_manager.h"
#include "lane_change_state_machine_manager.h"
#include "lateral_behavior_object_selector.h"
#include "reference_path.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {
class LaneChangeDecider : public Task {
 public:
  LaneChangeDecider(const EgoPlanningConfigBuilder* config_builder,
                    framework::Session* session);

  void Init();

  virtual ~LaneChangeDecider() = default;

  bool Execute() override;

  std::shared_ptr<LaneChangeRequestManager> get_lane_change_request_manager() {
    return lc_req_mgr_;
  }
  std::shared_ptr<LaneChangeLaneManager> get_lane_change_lane_manager() {
    return lc_lane_mgr_;
  }

 private:
  void update_scenario();

 private:
  bool CheckEgoPosition() const;

  bool UpdateObjectSelector(bool active);

  void UpdateFixLaneVirtualId();

 private:
  ScenarioStateMachineConfig config_;
  std::shared_ptr<LaneChangeRequestManager> lc_req_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lc_lane_mgr_;
  std::shared_ptr<LaneChangeStateMachineManager> lc_sm_mgr_;
  int scenario_ = SCENARIO_CRUISE;
  RequestSource last_frame_lc_req_source_ = NO_REQUEST;
  RequestType last_frame_lc_req_dir_ = NO_CHANGE;
};

}  // namespace planning
