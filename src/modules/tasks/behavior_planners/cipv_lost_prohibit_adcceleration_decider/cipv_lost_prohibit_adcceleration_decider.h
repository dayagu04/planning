#pragma once

#include <memory>
#include "dynamic_world/dynamic_world.h"
#include "ego_state_manager.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "session.h"
#include "tasks/task.h"
#include "utils/kd_path.h"
#include "virtual_lane_manager.h"

namespace planning {
class CipvLostProhibitAccelerationDecider : public Task {
 public:
  explicit CipvLostProhibitAccelerationDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);

  virtual ~CipvLostProhibitAccelerationDecider() = default;

  void Reset();

  bool Execute();

  void Update();

  double CalculateRelativeDistance(const std::shared_ptr<KDPath>& planned_path,
                                   planning::agent::Agent cipv);

  const planning::agent::Agent* QuerryCipv(
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
      const planning::agent::Agent* cipv);

 private:
  planning::framework::Session* session_ = nullptr;
  GeneralPlanningConfig config_;

  bool cipv_has_ = false;
  bool last_cipv_fn_ = false;
  bool prohibit_acc_ = false;
  int32_t counter_ = 0;
  double speed_limit_ = std::numeric_limits<double>::max();
  double start_time_ = -1.0;
  double end_time_ = -1.0;
  double duration_ = 0.0;
  int32_t pre_cipv_id_ = -1;
  int32_t pre_cipv_lost_id_ = -1;
  int32_t cipv_id_ = -1;
  double pre_cipv_rel_s_ = std::numeric_limits<double>::max();
  double pre_cipv_ttc_ = std::numeric_limits<double>::max();
  std::vector<int32_t> history_cipv_ids_;
};
}  // namespace planning