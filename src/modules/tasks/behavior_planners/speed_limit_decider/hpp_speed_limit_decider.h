#pragma once

#include <cstdint>

#include "agent/agent.h"
#include "define/geometry.h"
#include "ego_planning_config.h"
#include "speed_limit_decider_output.h"
#include "static_analysis_storage/static_analysis_storage.h"
#include "tasks/task.h"
#include "tasks/task_interface/crossing_agent_decider_output.h"
#include "traffic_light_decision_manager.h"

namespace planning {
struct HPPSpeedLimitZoneInfo {
  bool in_speed_limit_zone = false;           // 是否在减速带区域内
  bool approaching_speed_limit_zone = false;  // 是否接近减速带区域
  double distance_to_zone = 1000.0;  // 距离减速带区域的最小距离
  // 与自车路径有碰撞关系的减速带在参考线上的 [s_min, s_max] 区间列表
  std::vector<std::pair<double, double>> s_segments;
};

class HPPSpeedLimitDecider : public Task {
 public:
  HPPSpeedLimitDecider(const EgoPlanningConfigBuilder* config_builder,
                       framework::Session* session);
  virtual ~HPPSpeedLimitDecider() = default;
  bool Execute() override;

 private:
  void CalculateUserSpeedLimit();
  void CalculateMapSpeedLimit();
  void CalculateCurveSpeedLimit();
  void CalculateNarrowAreaSpeedLimit();
  void CalculateAvoidLimit();
  void CalculateBumpLimit();
  void CalculateRampLimit();
  void CalculateIntersectionRoadLimit();

  bool BuildSpeedObjectiveZoneInfo(HPPSpeedLimitZoneInfo& zone_info,
                                   const CRoadType& road_type,
                                   const CPassageType& passage_type,
                                   const CElemType& elem_type);
  const double ComputeMaxLatAcceleration();
  const double ComputeCurvatureSpeedLimit(const TrajectoryPoints& traj_points,
                                          double max_lat_acceleration,
                                          double& vlimit_jerk,
                                          double& time_to_brake,
                                          double& out_max_curvature);

  void CheckSpeedBumpZone(const TrajectoryPoints& traj_points, double ego_s);
  double GetSpeedLimitInObjectiveZone(const HPPSpeedLimitZoneInfo& zone_info,
                                      const double target_v);

 private:
  LongitudinalDeciderV3Config
      hpp_speed_limit_config_;    // Temporarily use the original
                                  // version of config
  double v_target_;               // final v target
  SpeedLimitType v_target_type_;  // final v target type

  double max_curvature_;
  double max_curvature_slow_down_triger_buffer_ = -1.0;
};
}  // namespace planning