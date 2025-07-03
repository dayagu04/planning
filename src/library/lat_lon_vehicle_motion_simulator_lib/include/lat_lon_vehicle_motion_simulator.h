#pragma once
#include <cstddef>
#include <memory>
#include <vector>

#include "basic_intelligent_driver_model.h"
#include "basic_pure_pursuit_model.h"
#include "longitudinal_motion_simulator_intelligent_driver_model.h"
#include "src/modules/common/config/basic_type.h"
#include "utils/cartesian_coordinate_system.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"
#include "vehicle_model_simulation.h"

namespace planning {

namespace simulator {
using IDMModelParam = BasicIntelligentDriverModel::ModelParam;
using PPModelParam = control::BasicPurePursuitModel::ModelParam;
using IDMState = BasicIntelligentDriverModel::ModelState;
using PPState = control::BasicPurePursuitModel::ModelState;

class LatLonVehicleMotionSimulator {
 public:
  struct SimulationResult {
    std::vector<double> t_vec;
    std::vector<double> s_vec;
    std::vector<double> vel_vec;
    std::vector<double> acc_vec;
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> theta_vec;
    std::vector<double> delta_vec;
    std::vector<double> kappa_vec;
    std::vector<double> ld_actual_length_vec;
    pnc::mathlib::spline s_t_spline;
    pnc::mathlib::spline vel_t_spline;
    pnc::mathlib::spline acc_t_spline;
    pnc::mathlib::spline x_t_spline;
    pnc::mathlib::spline y_t_spline;
    pnc::mathlib::spline theta_t_spline;
    pnc::mathlib::spline delta_t_spline;
    pnc::mathlib::spline kappa_t_spline;
    std::shared_ptr<planning_math::KDPath> lat_lon_vehicle_motion_path_ptr;
  };

  LatLonVehicleMotionSimulator(const IDMModelParam& idm_model_param,
                               const PPModelParam& pp_model_param,
                               const IDMState& idm_state,
                               const PPState& pp_state);

  LatLonVehicleMotionSimulator(const IDMModelParam& idm_model_param,
                               const PPModelParam& pp_model_param);

  LatLonVehicleMotionSimulator()
      : is_model_param_set_(false),
        is_model_state_set_(false),
        is_model_input_set_(false),
        simulation_result_(std::make_shared<SimulationResult>()) {}

  void set_model_param(const IDMModelParam& idm_model_param,
                       const PPModelParam& pp_model_param);

  void set_model_state(const IDMState& idm_state, const PPState& pp_state,
                       const double delta = PI);

  void set_model_input(const std::shared_ptr<ReferencePath>& reference_path) {
    reference_path_ = reference_path;
    is_model_input_set_ = true;
  }

  void set_dt_resolution(const double dt);

  const double get_dt_resolution() const { return dt_resolution_; }

  const std::shared_ptr<SimulationResult> get_simulation_result() const {
    return simulation_result_;
  }

  ErrorType Simulate(const double t);

  void Reset_Simulation_Result(size_t n);

  ~LatLonVehicleMotionSimulator() = default;

  void Reset();

 private:
  void Step_Forward(const double dt);

 private:
  // model param
  IDMModelParam idm_model_param_;
  PPModelParam pure_pursuit_model_param_;

  // model state
  IDMState idm_state_;
  PPState pp_state_;
  double init_delta_{PI};

  // lat lon motion model
  BasicIntelligentDriverModel basic_idm_model_;
  LonMotionSimulatorIntelligentDriverModel lon_motion_idm_model_;
  control::BasicPurePursuitModel pure_pursuit_model_;

  // vehicle model
  pnc::steerModel::VehicleSimulation vehicle_model_;

  // model param set flag
  bool is_model_param_set_{false};

  // model state set flag
  bool is_model_state_set_{false};

  // model input set flag
  bool is_model_input_set_{false};

  // dt resolution
  double dt_resolution_{0.1};  // default time step

  // model input
  std::shared_ptr<ReferencePath> reference_path_;

  // output
  std::shared_ptr<SimulationResult> simulation_result_;
};
}  // namespace simulator

}  // namespace planning