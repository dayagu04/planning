#pragma once

#include <array>
#include <cstddef>
#include <iostream>
#include <vector>

#include "basic_intelligent_driver_model.h"
#include "src/library/advanced_ctrl_lib/include/spline.h"

namespace planning {

class LonMotionSimulatorIntelligentDriverModel
    : public BasicIntelligentDriverModel {
 public:
  using ModelParam = BasicIntelligentDriverModel::ModelParam;
  using ModelState = BasicIntelligentDriverModel::ModelState;
  using InternalState = std::array<double, 4>;

  struct SimulationResult {
    std::vector<double> t_vec;
    std::vector<double> s_vec;
    std::vector<double> vel_vec;
    std::vector<double> acc_vec;
    pnc::mathlib::spline t_s_spline;
    pnc::mathlib::spline t_v_spline;
    pnc::mathlib::spline t_a_spline;
  };

  LonMotionSimulatorIntelligentDriverModel();

  LonMotionSimulatorIntelligentDriverModel(const ModelParam &param);

  ~LonMotionSimulatorIntelligentDriverModel() = default;

  const ModelState &state() const { return state_; }

  void set_model_state(const ModelState &state);

  void set_model_param(const ModelParam &param);

  void set_dt_resolution(const double dt) { dt_resolution_ = dt; }

  const double acc() const { return acc_; }

  const SimulationResult &get_simulation_result() const {
    return simulation_result_;
  }

  ErrorType Simulate();

  ErrorType Simulate(const ModelState &init_state, const double t);

  void Reset_Simulation_Result(size_t n);

 private:
  void UpdateInternalState();

  void Linear(const InternalState &x, const double dt, InternalState *x_out);

  // get acc by t in iidm and state dot
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

  void Step_Forward(const double dt);

  void RK4Integrate(const InternalState &x, const double dt,
                    InternalState *x_out);
  // ego s, ego vel, cipv s, cipv vel
  InternalState internal_state_;

  ModelParam param_;
  ModelState state_;
  double acc_{0.0};
  double dt_resolution_{0.1};  // default time step

  bool already_set_state_{false};

  bool already_set_param_{false};

  // output info
  SimulationResult simulation_result_;
};


}  // namespace planning