#pragma once

#include <string>

#include "macro.h"
#include "vehicle_param.pb.h"

namespace planning {

class VehicleParamHelper {
 private:
  DECLARE_SINGLETON(VehicleParamHelper);

 public:
  static void Init();

  static void Init(const std::string& config_file);

  static const common::VehicleParam& GetParam();

 private:
  static common::VehicleParam vehicle_param_;
  static bool is_init_;
};

} // namespace planning