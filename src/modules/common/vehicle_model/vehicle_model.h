#pragma once

#include <cmath>
#include <string>

#include "config/message_type.h"

namespace planning {
namespace common {

struct VehicleModelConfig {
  enum ModelType {
    REAR_CENTERED_KINEMATIC_BICYCLE_MODEL = 0,
    COM_CENTERED_DYNAMIC_BICYCLE_MODEL = 1,
    MLP_MODEL = 2,
  };

  struct RearCenteredKinematicBicycleModelConfig {
    double dt{0.1};
  };

  struct ComCenteredDynamicBicycleModelConfig {
    double dt{0.1};
  };

  struct MlpModelConfig {
    double dt{0.1};
  };

  ModelType model_type = ModelType::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL;
  RearCenteredKinematicBicycleModelConfig rc_kinematic_bicycle_model;
  ComCenteredDynamicBicycleModelConfig comc_dynamic_bicycle_model;
  MlpModelConfig mlp_model;
};

class VehicleModel {
 public:
  VehicleModel() = delete;

  static bool LoadVehicleModelConfig(const std::string& config_file_dir);

  static VehicleState Predict(const double predicted_time_horizon,
                              const VehicleState& cur_vehicle_state);

 private:
  static void RearCenteredKinematicBicycleModel(
      const VehicleModelConfig& vehicle_model_config,
      const double predicted_time_horizon,
      const VehicleState& cur_vehicle_state,
      VehicleState* predicted_vehicle_state);

 private:
  static VehicleModelConfig vehicle_model_config_;
};

}  // namespace common
}  // namespace planning
