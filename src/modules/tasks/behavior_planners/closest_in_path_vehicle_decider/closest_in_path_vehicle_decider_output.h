#pragma once

#include <cstdint>
namespace planning {

class ClosestInPathVehicleDeciderOutput {
 public:
  ClosestInPathVehicleDeciderOutput() = default;
  ~ClosestInPathVehicleDeciderOutput() = default;

 void Reset();

  const int32_t cipv_id() const;
  void set_cipv_id(const int32_t cipv_id);

  double relative_s() const;
  void set_relative_s(const double s);

  double v_frenet() const;
  void set_v_frenet(const double v_frenet);

  double ttc() const;
  void set_ttc(const double ttc);

  double dangerous_level() const;
  void set_dangerous_level(const double dangerous_level);
  bool is_virtual() const;
  void set_is_virtual(const bool is_virtual);

private:
  // TODO:Enrich the cipv info, or create a cipv_class later.
  int32_t cipv_id_ = -1;
  double relative_s_ = 0.0;
  double v_frenet_ = 0.0;
  double ttc_ = 100.0;
  int32_t dangerous_level_ = -1;
  bool is_virtual_ = false;
};

}  // namespace planning
