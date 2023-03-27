#pragma once

#include <unistd.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <unordered_map>

#include "common/log.h"
#include "common/macro.h"
#include "common/utils.h"
#include "modules/common/config/vehicle_param.h"

namespace planning {

// TODO move to session
class VehicleConfigurationContext {
 private:
  // this is a singleton class
  DECLARE_SINGLETON(VehicleConfigurationContext);

 public:
  std::string get_vehicle_param_dir() {
    auto car_lib_dir = planning::common::getenv("CALIB_DIR");
    if (!car_lib_dir.empty()) {
      return car_lib_dir;
    }
    if (which_car_.empty()) {
      which_car_ = planning::common::getenv("WHICH_CAR");
    }
    std::string carlib_dir = planning::common::getenv("CAM_CALIB_DIR");
    return carlib_dir + "/" + which_car_;
  }

  void load_vehicle_param() {
    std::string vehicle_param_path = get_vehicle_param_dir() + "/vehicle.yaml";
    LOG_DEBUG("load_vehicle_param: vehicle_param_path: %s",
              vehicle_param_path.c_str());
    if (access(vehicle_param_path.c_str(), F_OK) == -1) {
      LOG_DEBUG("ConfigContext: vehicle.yaml not exist!");
      return;
    }
  }

  void reset_which_car(const std::string which_car) {
    this->which_car_ = which_car;
  }

  const VehicleParam &get_vehicle_param() const { return vehicle_param_; }

  void set_vehicle_param(const VehicleParam &vehicle_param) {
    vehicle_param_ = vehicle_param;
  }

  void write_params(std::string path) {}

 private:
  VehicleParam vehicle_param_;
  std::string which_car_;
};

}  // namespace planning
