#pragma once

#include <cstdint>
#include <vector>

#include "apa_debug_data.pb.h"
#include "trajectory_point.h"

namespace planning {
namespace trajectory {

class Trajectory : public std::vector<TrajectoryPoint> {
 public:
  Trajectory() = default;
  explicit Trajectory(std::vector<TrajectoryPoint>&& trajectory_points);
  virtual ~Trajectory() = default;

  TrajectoryPoint Evaluate(const double query_time) const;

  const bool is_elements_vec_ready() const {
    return traj_elements_vec_ready_flag_;
  }

  const double GetTimeLength() const;

  size_t QueryLowerBoundPoint(const double absolute_time) const;
  size_t QueryLowerBoundPointByS(const double s) const;
  size_t QueryNearestPoint(const planning_math::Vec2d& position) const;
  const bool QueryNearestPointWithBuffer(const planning_math::Vec2d& position,
                                         const double buffer,
                                         TrajectoryPoint* point) const;

  void SetSpeedProfileType(const common::SpeedProfileType type) {
    speed_type_ = type;
    return;
  }

  const common::SpeedProfileType GetSpeedProfileType() const {
    return speed_type_;
  }

  void SetGear(const int gear);

  const int GetGear() const { return gear_; }

  void Clear();

  void SetStopS(const double s) {
    stop_decision_s_ = s;
    return;
  }

  const double GetStopS() const { return stop_decision_s_; }

  bool traj_elements_vec_ready_flag_ = false;
  std::vector<double> x_vec_{};
  std::vector<double> y_vec_{};
  std::vector<double> theta_vec_{};

 private:
  common::SpeedProfileType speed_type_;
  // 0: normal;
  // 1: reverse;
  // 2: drive;
  // 3: parking;
  int gear_;

  double stop_decision_s_;
};

}  // namespace trajectory
}  // namespace planning