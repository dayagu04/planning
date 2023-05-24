#pragma once

#include <mutex>
#include <atomic>

#include "apa_sim_config.pb.h"
#include "func_state_machine.pb.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "planning_plan.pb.h"
#include "vehicle_service.pb.h"

#include "autoplt/include/ADSComponent.h"
#include "autoplt/include/ADSNode.h"
#include "autoplt/include/ADSTime.h"
#include "ifly_time.h"
#include "common/utils/file.h"

namespace planning {

using ::FuncStateMachine::FuncStateMachine;
using ::FuncStateMachine::FunctionalState;
using autoplt::ADSNode;
using LocalizationOutput::LocalizationEstimate;
using ParkingFusion::ParkingFusionInfo;
using PlanningOutput::PlanningOutput;
using VehicleService::VehicleServiceOutputInfo;

class ApaSimulationComponent final : public autoplt::ADSTimerCoponent {
 public:
  ApaSimulationComponent() = default;

  ~ApaSimulationComponent();

  bool Init() override;
  bool Proc() override;

 private:
  void MockParkingFusionInfo();
  void MockLocalizationAndVehicleService();
  void MockFuncStateMachine();
  void GetKeyInput();

 private:
  std::mutex msg_mutex_;
  std::mutex func_state_mutex_;

  ApaSimConfig apa_sim_config_;

  std::shared_ptr<ADSNode> simulation_node_ = nullptr;

  // input signals
  PlanningOutput planning_output_msg_;

  ParkingFusionInfo parking_fusion_info_msg_;

  // output signals
  std::shared_ptr<Writer<LocalizationEstimate>>
      localization_estimate_writer_ = nullptr;
  std::shared_ptr<Writer<ParkingFusionInfo>>
      parking_fusion_info_writer_ = nullptr;
  std::shared_ptr<Writer<VehicleServiceOutputInfo>>
      vehicle_service_output_info_writer_ = nullptr;
  std::shared_ptr<Writer<FuncStateMachine>>
      func_state_machine_writer_ = nullptr;

  int traj_pt_index_ = 0;
  PlanningOutput cur_planning_output_;
  Common::GearCommandValue last_planning_gear_ =
      Common::GearCommandValue::GEAR_COMMAND_VALUE_NONE;

  double last_ego_x_ = 0.0;
  double last_ego_y_ = 0.0;
  double last_ego_theta_ = 0.0;
  double last_ego_spd_ = 0.0;

  FunctionalState func_state_ = FunctionalState::INIT;
  std::shared_ptr<std::thread> key_input_thread_;
};

// register planning component
AUTOPLT_REGISTER_COMPONENT(ApaSimulationComponent)

} // namespace planning