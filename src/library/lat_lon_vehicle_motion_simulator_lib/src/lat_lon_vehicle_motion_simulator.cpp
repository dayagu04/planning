#include "../include/lat_lon_vehicle_motion_simulator.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "log.h"
#include "src/common/ifly_time.h"
#include "utils/path_point.h"
#include "vehicle_model_simulation.h"
namespace planning {
namespace simulator {
LatLonVehicleMotionSimulator::LatLonVehicleMotionSimulator(
    const IDMModelParam& idm_model_param, const PPModelParam& pp_model_param,
    const IDMState& idm_state, const PPState& pp_state)
    : idm_model_param_(idm_model_param),
      pure_pursuit_model_param_(pp_model_param),
      idm_state_(idm_state),
      pp_state_(pp_state) {
  lon_motion_idm_model_.set_model_state(idm_state_);
  lon_motion_idm_model_.set_model_param(idm_model_param);
  pure_pursuit_model_.set_model_param(pp_model_param);
  pure_pursuit_model_.set_model_state(pp_state_);
  is_model_param_set_ = true;
  is_model_state_set_ = true;
  is_model_input_set_ = false;
}
LatLonVehicleMotionSimulator::LatLonVehicleMotionSimulator(
    const IDMModelParam& idm_model_param, const PPModelParam& pp_model_param)
    : idm_model_param_(idm_model_param),
      pure_pursuit_model_param_(pp_model_param) {
  lon_motion_idm_model_.set_model_param(idm_model_param);
  pure_pursuit_model_.set_model_param(pp_model_param);
  is_model_param_set_ = true;
  is_model_state_set_ = false;
  is_model_input_set_ = false;
}

void LatLonVehicleMotionSimulator::set_model_param(
    const IDMModelParam& idm_model_param, const PPModelParam& pp_model_param) {
  idm_model_param_ = idm_model_param;
  pure_pursuit_model_param_ = pp_model_param;
  lon_motion_idm_model_.set_model_param(idm_model_param_);
  pure_pursuit_model_.set_model_param(pure_pursuit_model_param_);
  is_model_param_set_ = true;
}

void LatLonVehicleMotionSimulator::set_model_state(const IDMState& idm_state,
                                                   const PPState& pp_state,
                                                   const double delta) {
  idm_state_ = idm_state;
  pp_state_ = pp_state;
  init_delta_ = delta == PI ? 0.0 : delta;
  lon_motion_idm_model_.set_model_state(idm_state_);
  pure_pursuit_model_.set_model_state(pp_state_);
  is_model_state_set_ = true;
}

void LatLonVehicleMotionSimulator::set_dt_resolution(const double dt) {
  dt_resolution_ = dt;
  lon_motion_idm_model_.set_dt_resolution(dt_resolution_);
  vehicle_model_.set_dt_resolution(dt_resolution_);
}

ErrorType LatLonVehicleMotionSimulator::Simulate(const double t) {
  if (!is_model_param_set_) {
    LOG_ERROR("LatLonVehicleMotionSimulator::Simulate: model param not set!!!");
    return ErrorType::kWrongStatus;
  }
  if (!is_model_state_set_) {
    LOG_ERROR("LatLonVehicleMotionSimulator::Simulate: model state not set!!!");
    return ErrorType::kWrongStatus;
  }
  if (!is_model_input_set_) {
    LOG_ERROR("LatLonVehicleMotionSimulator::Simulate: model input not set!!!");
    return ErrorType::kWrongStatus;
  }
  const size_t N = std::round(t / dt_resolution_);
  if (N < 1) {
    LOG_ERROR(
        "LatLonVehicleMotionSimulator::Simulate: t %f is less than "
        "kStepForwarddt %f",
        t, dt_resolution_);
    return ErrorType::kIllegalInput;
  }
  Reset_Simulation_Result(N + 1);
  std::vector<planning_math::PathPoint> path_points_vec;
  path_points_vec.reserve(N + 1);
  // run idm model simulation to get vel_vec acc_vec and s_vec
  const double start_time_idm = IflyTime::Now_ms();
  lon_motion_idm_model_.Simulate(t);
  const double end_time_idm = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("idm_cost", end_time_idm - start_time_idm)
  const auto& idm_simulation_result =
      lon_motion_idm_model_.get_simulation_result();
  const auto& ref_points = reference_path_->get_points();
  pnc::steerModel::VehicleState vehi_init_state{
      .x_ = pp_state_.x, .y_ = pp_state_.y, .phi_ = pp_state_.theta};
  pp_state_.vel = idm_simulation_result.vel_vec.front();
  if (pure_pursuit_model_.ProcessReferencePath(reference_path_) !=
      ErrorType::kSuccess) {
    LOG_ERROR("LatLonVehicleMotionSimulator::Simulate: reference path is null");
    return ErrorType::kIllegalInput;
  }
  pnc::steerModel::VehicleParameter vehicle_param;
  simulation_result_->t_vec[0] = 0.0;
  simulation_result_->s_vec[0] = idm_simulation_result.t_s_spline(0.0);
  simulation_result_->vel_vec[0] = idm_simulation_result.t_v_spline(0.0);
  simulation_result_->acc_vec[0] = idm_simulation_result.t_a_spline(0.0);
  simulation_result_->x_vec[0] = vehi_init_state.x_;
  simulation_result_->y_vec[0] = vehi_init_state.y_;
  simulation_result_->theta_vec[0] = vehi_init_state.phi_;
  simulation_result_->delta_vec[0] = init_delta_;
  simulation_result_->kappa_vec[0] = 0.33 * sin(init_delta_);
  path_points_vec.emplace_back(vehi_init_state.x_, vehi_init_state.y_);
  std::vector<double> pp_cost_vec(N, 0.0);
  std::vector<double> vehicle_sim_forward_vec(N, 0.0);
  for (size_t i = 1; i <= N; ++i) {
    double delta_step_i, vel_step_i, ld_actual_length_i = 0.0;
    pnc::steerModel::VehicleControl lat_lon_control_u_i;
    vehicle_model_.Init(vehi_init_state);
    // run pp model get delta_i
    pure_pursuit_model_.set_model_state(pp_state_);
    pure_pursuit_model_.set_model_param(pure_pursuit_model_param_);
    double start_time_pp = IflyTime::Now_ms();
    pure_pursuit_model_.CalculateDesiredDelta();
    double end_time_pp = IflyTime::Now_ms();
    pp_cost_vec.emplace_back(end_time_pp - start_time_pp);
    delta_step_i = pure_pursuit_model_.get_delta();
    ld_actual_length_i = pure_pursuit_model_.get_ld_actual_lenghth();
    // get vel_i from idm model simulation
    vel_step_i = idm_simulation_result.t_v_spline(i * dt_resolution_);
    // assemble vehicle control input(delta_i, vel_i)
    lat_lon_control_u_i.delta_ = delta_step_i;
    lat_lon_control_u_i.v_ = vel_step_i;
    // run vehicle model simulation to get vehicle state_i(x, y, phi)
    double start_time_vehi_sim_forward = IflyTime::Now_ms();
    vehicle_model_.Update(lat_lon_control_u_i, vehicle_param);
    double end_time_vehi_sim_forward = IflyTime::Now_ms();
    vehicle_sim_forward_vec.emplace_back(end_time_vehi_sim_forward -
                                         start_time_vehi_sim_forward);
    const auto vehicle_state_i = vehicle_model_.GetState();
    // update pp state and vehicle state
    pp_state_ =
        PPState(vehicle_state_i.x_, vehicle_state_i.y_, vehicle_state_i.phi_,
                idm_simulation_result.t_v_spline(i * dt_resolution_));
    vehi_init_state = vehicle_state_i;

    simulation_result_->t_vec[i] = i * dt_resolution_;
    simulation_result_->s_vec[i] =
        idm_simulation_result.t_s_spline(i * dt_resolution_);
    simulation_result_->vel_vec[i] =
        idm_simulation_result.t_v_spline(i * dt_resolution_);
    simulation_result_->acc_vec[i] =
        idm_simulation_result.t_a_spline(i * dt_resolution_);
    simulation_result_->x_vec[i] = vehicle_state_i.x_;
    simulation_result_->y_vec[i] = vehicle_state_i.y_;
    simulation_result_->theta_vec[i] = vehicle_state_i.phi_;
    simulation_result_->delta_vec[i] = delta_step_i;
    simulation_result_->kappa_vec[i] = 0.33 * sin(delta_step_i);
    simulation_result_->ld_actual_length_vec[i] = ld_actual_length_i;
    path_points_vec.emplace_back(vehicle_state_i.x_, vehicle_state_i.y_);
  }
  simulation_result_->s_t_spline.set_points(simulation_result_->t_vec,
                                            simulation_result_->s_vec);
  simulation_result_->vel_t_spline.set_points(simulation_result_->t_vec,
                                              simulation_result_->vel_vec);
  simulation_result_->acc_t_spline.set_points(simulation_result_->t_vec,
                                              simulation_result_->acc_vec);
  simulation_result_->x_t_spline.set_points(simulation_result_->t_vec,
                                            simulation_result_->x_vec);
  simulation_result_->y_t_spline.set_points(simulation_result_->t_vec,
                                            simulation_result_->y_vec);
  simulation_result_->theta_t_spline.set_points(simulation_result_->t_vec,
                                                simulation_result_->theta_vec);
  simulation_result_->delta_t_spline.set_points(simulation_result_->t_vec,
                                                simulation_result_->delta_vec);
  simulation_result_->kappa_t_spline.set_points(simulation_result_->t_vec,
                                                simulation_result_->kappa_vec);
  simulation_result_->lat_lon_vehicle_motion_path_ptr =
      std::make_shared<planning_math::KDPath>(std::move(path_points_vec));
  const double pp_cost =
      std::accumulate(pp_cost_vec.begin(), pp_cost_vec.end(), 0.0);
  const double vehicle_sim_forward_cost = std::accumulate(
      vehicle_sim_forward_vec.begin(), vehicle_sim_forward_vec.end(), 0.0);
  JSON_DEBUG_VALUE("pp_cost", pp_cost)
  JSON_DEBUG_VALUE("vehicle_sim_forward_cost", vehicle_sim_forward_cost)
  Reset();
  return ErrorType::kSuccess;
}

void LatLonVehicleMotionSimulator::Reset_Simulation_Result(size_t n) {
  simulation_result_->t_vec.resize(n, 0);
  simulation_result_->s_vec.resize(n, 0);
  simulation_result_->vel_vec.resize(n, 0);
  simulation_result_->acc_vec.resize(n, 0);
  simulation_result_->x_vec.resize(n, 0);
  simulation_result_->y_vec.resize(n, 0);
  simulation_result_->theta_vec.resize(n, 0);
  simulation_result_->delta_vec.resize(n, 0);
  simulation_result_->kappa_vec.resize(n, 0);
  simulation_result_->ld_actual_length_vec.resize(n, 0);
  simulation_result_->lat_lon_vehicle_motion_path_ptr.reset();
}

void LatLonVehicleMotionSimulator::Reset() {
  idm_model_param_ = IDMModelParam();
  pure_pursuit_model_param_ = PPModelParam();
  idm_state_ = IDMState();
  pp_state_ = PPState();
  is_model_param_set_ = false;
  is_model_state_set_ = false;
  is_model_input_set_ = false;
}

}  // namespace simulator

}  // namespace planning