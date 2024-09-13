#ifndef MODULES_SIMULATION_CONTEXT_
#define MODULES_SIMULATION_CONTEXT_

#include <Eigen/Dense>
#include "macro.h"

namespace planning {

class SimulationContext {
 public:
  const double planning_loop_dt() const { return planning_loop_dt_; }
  void set_planning_loop_dt(double planning_loop_dt) {
    planning_loop_dt_ = planning_loop_dt;
  }

  const double prediction_relative_time() const {
    return prediction_relative_time_;
  }
  void set_prediction_relative_time(double prediction_relative_time) {
    prediction_relative_time_ = prediction_relative_time;
  }

  const double localizatoin_latency() const { return localizatoin_latency_; }
  void set_localizatoin_latency(double localizatoin_latency) {
    localizatoin_latency_ = localizatoin_latency;
  }

  const Eigen::Vector3d ego_pose() const { return ego_pose_; }
  void set_ego_pose(Eigen::Vector3d ego_pose) { ego_pose_ = ego_pose; }

  const Eigen::Vector4d ego_orientation() const { return ego_orientation_; }
  void set_ego_orientation(Eigen::Vector4d ego_orientation) {
    ego_orientation_ = ego_orientation;
  }

  const bool is_close_loop() const { return is_close_loop_; }
  void set_is_close_loop(bool is_close_loop) { is_close_loop_ = is_close_loop; }

  const double ego_yaw() const { return ego_yaw_; }
  void set_ego_yaw(double ego_yaw) { ego_yaw_ = ego_yaw; }

  const double ego_speed() const { return ego_speed_; }
  void set_ego_speed(double ego_speed) { ego_speed_ = ego_speed; }

  const double ego_acc() const { return ego_acc_; }
  void set_ego_acc(double ego_acc) { ego_acc_ = ego_acc; }

 private:
  // this is a singleton class
  IFLY_DECLARE_SINGLETON(SimulationContext);

  double planning_loop_dt_ = 0.1;
  double prediction_relative_time_ = 0;
  double localizatoin_latency_ = 45;
  Eigen::Vector3d ego_pose_;
  Eigen::Vector4d ego_orientation_;
  double ego_yaw_ = 0;
  bool is_close_loop_ = false;
  double ego_speed_ = 0;
  double ego_acc_ = 0;
};

}  // namespace planning

#endif