#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "config/basic_type.h"
#include "ego_state_manager.h"
#include "session.h"
#include "utils/kd_path.h"
#include "quintic_poly_path.h"

namespace planning {

class QuinticPathPlanner {
 public:
  QuinticPathPlanner(const GapSelectorConfig& config,
                     framework::Session* session);

  // 执行固定时间的五次多项式轨迹规划
  bool Plan(const double lat_avoid_offset, const double lc_end_s,
            const double remain_lc_duration,
            TrajectoryPoints& traj_points);

 private:
  // 初始化规划所需的状态
  bool InitializeState(const double remain_lc_duration,
      const TrajectoryPoints& traj_points);

  // 计算终点状态
  bool CalculateEndState(const double lat_avoid_offset,
                        const double lane_change_end_s);

  // 构建五次多项式轨迹
  bool ConstructQuinticPath(const double remain_lc_duration);

  // 采样轨迹点
  bool SampleTrajectoryPoints(const double remain_lc_duration,
                              const double lat_avoid_offset,
                              TrajectoryPoints& traj_points);

 private:
  GapSelectorConfig config_;
  framework::Session* session_;

  // 规划相关状态
  bool use_ego_v_ = false;
  double ego_v_ = 0.0;
  planning::common::LateralInitState lat_state_;
  Point2D frenet_init_point_;
  Point2D cart_init_point_;
  std::shared_ptr<planning_math::KDPath> coord_;

  // 终点状态
  Point2D frenet_end_point_;
  Point2D cart_end_point_;
  double lane_change_end_heading_angle_ = 0.0;
  double lane_change_end_curvature_ = 0.0;

  // 五次多项式轨迹
  pnc::spline::QuinticPolynominalPath lane_change_quintic_path_;

  // 环境模型相关
  std::shared_ptr<EgoStateManager> ego_state_mgr_;
  CoarsePlanningInfo coarse_planning_info_;
};

}  // namespace planning