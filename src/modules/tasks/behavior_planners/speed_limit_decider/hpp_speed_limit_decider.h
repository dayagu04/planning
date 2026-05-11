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
  double distance_to_zone = 1000.0;           // 距离减速带区域的最小距离
  // 与自车路径有碰撞关系的减速带在参考线上的 [s_min, s_max] 区间列表
  std::vector<std::pair<double, double>> s_segments;
};

struct SpeedLimitSegment {
  double s_start;
  double s_end;
  double v_limit;
  SpeedLimitType type;
};

// 曲率 profile：一次轨迹扫描的结果，供各弯道限速子函数共享
struct CurvatureProfile {
  std::vector<double> s_values;          // 累积弧长
  std::vector<double> curvature_values;  // 对应的曲率值
  double scan_distance = 0.0;            // 扫描的总距离
  double max_curvature = 0.0;            // 最大曲率（绝对值）
  double max_curvature_s = 0.0;          // 最大曲率对应的 s 位置
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

  CurvatureProfile ScanTrajectory(const TrajectoryPoints& traj_points);
  double ComputeCurvatureSpeedLimit(const CurvatureProfile& profile);
  void CalculateSCurveLimit(double v_limit_curv);
  void CalculateUTurnLimit(double v_limit_curv);

  bool BuildSpeedObjectiveZoneInfo(HPPSpeedLimitZoneInfo& zone_info,
                                   const CRoadType& road_type,
                                   const CPassageType& passage_type,
                                   const CElemType& elem_type,
                                   const double approach_distance_threshold,
                                   bool use_half_vehicle_exit = false);
  const double ComputeMaxLatAcceleration();

  void CheckSpeedBumpZone(const TrajectoryPoints& traj_points, double ego_s);
  void BuildSmoothedSpeedProfile(double ego_s, double ego_v, double v_cruise);

  void CalculateNarrowAreaSpeedLimitFromBounds();
  void CalculateNarrowAreaSpeedLimitFromMap();
  bool IsDynamicBound(BoundType type) const;
  bool IsImmovableObstacle(agent::AgentType type) const;
  double ComputeObstacleSpeedCompensation(const BoundInfo& bound_info) const;
  double ComputeNarrowSpeedLimit(double width, const BoundInfo& left_bound_info,
                                 const BoundInfo& right_bound_info) const;

 private:
  LongitudinalDeciderV3Config
      hpp_speed_limit_config_;    // Temporarily use the original
                                  // version of config
  double v_target_;               // final v target
  SpeedLimitType v_target_type_;  // final v target type

  double max_curvature_;
  double max_curvature_slow_down_triger_buffer_ = -1.0;
  std::vector<SpeedLimitSegment> speed_limit_segments_;
};
}  // namespace planning