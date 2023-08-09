#include "common/vehicle_param_helper.h"

#include <string>

#include "../../common/planning_gflags.h"
#include "utils/file.h"

namespace planning {

CommonVehicleParam::CommonVehicleParam VehicleParamHelper::vehicle_param_;
bool VehicleParamHelper::is_init_ = false;

void VehicleParamHelper::Init() { Init(FLAGS_vehicle_param_config_file); }

void VehicleParamHelper::Init(const std::string& config_file) {
  common::util::GetProtoFromFile(config_file, &vehicle_param_);
  is_init_ = true;
  std::cout << vehicle_param_.ShortDebugString() << std::endl;
}

const CommonVehicleParam::CommonVehicleParam& VehicleParamHelper::GetParam() {
  if (!is_init_) {
    Init();
  }
  return vehicle_param_;
}

}  // namespace planning
