#pragma once

#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "math/quintic_poly_1d.h"
#include "trajectory/trajectory.h"
#include "virtual_lane.h"

namespace planning {

class CoordinateConverter {
 public:
  CoordinateConverter() : x_{0.}, y_{0.}, theta_{0.} {}
  CoordinateConverter(const double x, const double y, const double theta)
      : x_{x}, y_{y}, theta_{theta} {
    sin_theta_ = std::sin(theta);
    cos_theta_ = std::cos(theta);
  }
  std::pair<double, double> ConvertToVehicle(const double x,
                                             const double y) const {
    double dx = x - x_;
    double dy = y - y_;
    return {dx * cos_theta_ + dy * sin_theta_,
            -dx * sin_theta_ + dy * cos_theta_};
  }
  std::pair<double, double> ConvertToGlobal(const double x,
                                            const double y) const {
    return {x_ + x * cos_theta_ - y * sin_theta_,
            y_ + x * sin_theta_ + y * cos_theta_};
  }
  double Theta() const { return theta_; }

 private:
  double x_;
  double y_;
  double theta_;
  double sin_theta_;
  double cos_theta_;
};

class AgentTrajectoryCalculator {
  struct QuinticPolylinePointInfo {
    double front_axis_x{0.};
    double front_axis_y{0.};
    double front_axis_s{0.};
    double front_axis_l{0.};
    double rear_axis_x{0.};
    double rear_axis_y{0.};
    double rear_axis_s{0.};
    double rear_axis_l{0.};
    double theta{0.};
  };

 public:
  AgentTrajectoryCalculator(framework::Session* session);

  ~AgentTrajectoryCalculator() = default;

  bool Process();

 private:
  bool CalculateCutinAgentTrajectory(
      const bool is_in_lane_change_execution,
      const std::shared_ptr<agent::Agent>& ptr_agent);

  bool GenerateMaxJerkQuinticPolyPath(
      const bool is_in_lane_change_execution,
      const std::shared_ptr<planning_math::KDPath>& current_lane_coord,
      const std::shared_ptr<agent::Agent>& ptr_agent, const bool is_left,
      const double target_l,
      std::vector<QuinticPolylinePointInfo>& quintic_poly_path,
      bool* need_extend_st_boundary_backward);

  bool GenarateAgentTrajectoryWithPolyPath(
      const bool is_in_lane_change_execution,
      const std::shared_ptr<agent::Agent>& ptr_agent,
      const std::vector<QuinticPolylinePointInfo>& quintic_poly_path) const;

  bool MakeTargetLWithOriginTrajectory(
      const std::shared_ptr<VirtualLane> target_lane,
      const std::shared_ptr<planning_math::KDPath>& current_lane_coord,
      const std::shared_ptr<agent::Agent>& ptr_agent, bool is_left,
      double* const target_l) const;

  double GetEndMaxSEnd(const double ego_speed, const double ego_s_target_lane,
                       const double ego_l_target_lane) const;

  void GetStartEndPointInfo(const std::shared_ptr<agent::Agent>& ptr_agent,
                            const planning_math::PathPoint& target_point,
                            const CoordinateConverter& converter,
                            const double target_l,
                            std::array<double, 3>& start_point,
                            std::array<double, 3>& end_point,
                            double& end_x) const;

  double CalculateAgentAverageSpeed(
      const std::shared_ptr<agent::Agent>& ptr_agent) const;

  double CalculateRoadCurvature(const double v_ego);

 private:
  framework::Session* session_ = nullptr;
};

}  // namespace planning