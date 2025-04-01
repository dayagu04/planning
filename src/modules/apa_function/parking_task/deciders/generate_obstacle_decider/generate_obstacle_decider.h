#pragma once

#include <memory>

#include "apa_obstacle_manager.h"
#include "apa_slot_manager.h"
#include "parking_task.h"
namespace planning {
namespace apa_planner {
using namespace pnc;

struct GenerateObstacleRequest {
  ParkingScenarioType scenario_type = ParkingScenarioType::SCENARIO_UNKNOWN;
  ProcessObsMethod process_obs_method = ProcessObsMethod::DO_NOTHING;

  double min_channel_width = 0.0;
  double min_channel_length = 0.0;

  GenerateObstacleRequest() {}
  GenerateObstacleRequest(const ParkingScenarioType _scenario_type,
                          ProcessObsMethod _process_obs_method,
                          const double _min_channel_width = 0.0,
                          const double _min_channel_length = 0.0)
      : scenario_type(_scenario_type),
        process_obs_method(_process_obs_method),
        min_channel_width(_min_channel_width),
        min_channel_length(_min_channel_length) {}
  ~GenerateObstacleRequest() {}

  void Reset() {
    scenario_type = ParkingScenarioType::SCENARIO_UNKNOWN;
    process_obs_method = ProcessObsMethod::DO_NOTHING;
  }
};

class GenerateObstacleDecider final : public ParkingTask {
 public:
  GenerateObstacleDecider(
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr,
      const std::shared_ptr<CollisionDetectorInterface>&
          col_det_interface_ptr) {
    obs_manager_ptr_ = obs_manager_ptr;
    col_det_interface_ptr_ = col_det_interface_ptr;
  }
  ~GenerateObstacleDecider() {}

  const bool GenObs(const EgoInfoUnderSlot& ego_info_under_slot,
                    GenerateObstacleRequest request);

  const bool GenObsForPerpendicularTailIn();

  const bool CalcVirtualTLane();

  const bool GenObsForPerpendicularHeadingIn();

  virtual void Reset() override {
    ego_info_under_slot_.Reset();
    request_.Reset();
    virtual_tlane_.Reset();
  }

  const TLane& GetVirtualTLane() const { return virtual_tlane_; }

 private:
  std::shared_ptr<ApaObstacleManager> obs_manager_ptr_;
  std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr_;
  EgoInfoUnderSlot ego_info_under_slot_;
  GenerateObstacleRequest request_;
  TLane virtual_tlane_;
};

}  // namespace apa_planner
}  // namespace planning