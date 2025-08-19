#include "closest_in_path_vehicle_decider_output.h"

namespace planning {
void ClosestInPathVehicleDeciderOutput::Reset() { cipv_id_ = -1; }

const int32_t ClosestInPathVehicleDeciderOutput::cipv_id() const {
  return cipv_id_;
}

void ClosestInPathVehicleDeciderOutput::set_cipv_id(const int32_t cipv_id) {
  cipv_id_ = cipv_id;
}

double ClosestInPathVehicleDeciderOutput::v_frenet() const { return v_frenet_; }
void ClosestInPathVehicleDeciderOutput::set_v_frenet(const double v_frenet) {
  v_frenet_ = v_frenet;
}

double ClosestInPathVehicleDeciderOutput::v_fusion_frenet() const {
  return v_fusion_frenet_;
}
void ClosestInPathVehicleDeciderOutput::set_v_fusion_frenet(
    const double v_fusion_frenet) {
  v_fusion_frenet_ = v_fusion_frenet;
}

double ClosestInPathVehicleDeciderOutput::acceleration() const {
  return acceleration_;
}
void ClosestInPathVehicleDeciderOutput::set_acceleration(
    const double acceleration) {
  acceleration_ = acceleration;
}

double ClosestInPathVehicleDeciderOutput::acceleration_fusion() const {
  return acceleration_fusion_;
}
void ClosestInPathVehicleDeciderOutput::set_acceleration_fusion(
    const double acceleration_fusion) {
  acceleration_fusion_ = acceleration_fusion;
}

double ClosestInPathVehicleDeciderOutput::relative_s() const {
  return relative_s_;
}
void ClosestInPathVehicleDeciderOutput::set_relative_s(const double s) {
  relative_s_ = s;
}

double ClosestInPathVehicleDeciderOutput::ttc() const { return ttc_; }
void ClosestInPathVehicleDeciderOutput::set_ttc(const double ttc) {
  ttc_ = ttc;
}

double ClosestInPathVehicleDeciderOutput::dangerous_level() const {
  return dangerous_level_;
}
void ClosestInPathVehicleDeciderOutput::set_dangerous_level(
    const double dangerous_level) {
  dangerous_level_ = dangerous_level;
}
bool ClosestInPathVehicleDeciderOutput::is_virtual() const {
  return is_virtual_;
}

void ClosestInPathVehicleDeciderOutput::set_is_virtual(const bool is_virtual) {
  is_virtual_ = is_virtual;
}
}  // namespace planning
