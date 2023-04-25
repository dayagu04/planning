#include "vehicle_param_helper.h"

#include <string>

#include "common/utils/file.h"

namespace planning {

using common::VehicleParam;

VehicleParam VehicleParamHelper::vehicle_param_;
bool VehicleParamHelper::is_init_ = false;

void VehicleParamHelper::Init() {
  Init("/asw/Planning/Res/config/vehicle_param.pb.txt");
}

void VehicleParamHelper::Init(const std::string& config_file) {
  common::util::GetProtoFromFile(config_file, &vehicle_param_);
  is_init_ = true;
}

const VehicleParam &VehicleParamHelper::GetParam() {
  if (!is_init_) {
    Init();
  }
  return vehicle_param_;
}

} // namespace planning
