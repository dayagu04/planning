#pragma once

#include <string>

#include "common_vehicle_param.pb.h"
#include "macro.h"

namespace planning {

class VehicleParamHelper {
 private:
  DECLARE_SINGLETON(VehicleParamHelper);

 public:
  static void Init();

  static void Init(const std::string& config_file);

  static const CommonVehicleParam::CommonVehicleParam& GetParam();

 private:
  static CommonVehicleParam::CommonVehicleParam vehicle_param_;
  static bool is_init_;
};

}  // namespace planning