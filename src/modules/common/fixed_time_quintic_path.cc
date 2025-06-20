#include "fixed_time_quintic_path.h"
#include "environmental_model.h"
#include "glog/logging.h"
#include "planning_context.h"
#include "session.h"
#include "spline_projection.h"

namespace planning {

QuinticPathPlanner::QuinticPathPlanner(
    const GapSelectorConfig& config,
    framework::Session* session)
    : config_(config), session_(session) {
  ego_state_mgr_ =
      session_->mutable_environmental_model()->get_ego_state_manager();
  coarse_planning_info_ = session_->planning_context()
                              .lane_change_decider_output()
                              .coarse_planning_info;
  coord_ = coarse_planning_info_.reference_path->get_frenet_coord();
}

bool QuinticPathPlanner::Plan(const double lat_avoid_offset,
                              const double lc_end_s,
                              const double remain_lc_duration,
                              TrajectoryPoints& traj_points) {
  // 检查剩余时间是否过短
  if (remain_lc_duration < 1.0) {
    LOG_WARNING("Remaining lane change duration is too short: %f",
                remain_lc_duration);
    return false;
  }

  // 初始化规划状态
  if (!InitializeState(remain_lc_duration, traj_points)) {
    LOG_ERROR("Failed to initialize planning state");
    return false;
  }

  // 计算终点状态
  double lane_change_end_s = frenet_init_point_.x + ego_v_ * remain_lc_duration;
  if (!CalculateEndState(lat_avoid_offset, lane_change_end_s)) {
    LOG_ERROR("Failed to calculate end state");
    return false;
  }

  // 构建五次多项式轨迹
  if (!ConstructQuinticPath(remain_lc_duration)) {
    LOG_ERROR("Failed to construct quintic path");
    return false;
  }

  // 采样轨迹点
  if (!SampleTrajectoryPoints(remain_lc_duration, lat_avoid_offset,
                              traj_points)) {
    LOG_ERROR("Failed to sample trajectory points");
    return false;
  }

  return true;
}

bool QuinticPathPlanner::InitializeState(const double remain_lc_duration,
                                         const TrajectoryPoints& traj_points) {
  // 确定使用的速度
  if (config_.use_ego_v) {
    use_ego_v_ = true;
  }

  auto v_cruise =
      use_ego_v_ ? ego_state_mgr_->ego_v() : ego_state_mgr_->ego_v_cruise();
  ego_v_ = std::fmax(v_cruise, config_.min_ego_v_cruise);

  // 获取初始状态
  lat_state_ = ego_state_mgr_->planning_init_point().lat_init_state;
  cart_init_point_ = {lat_state_.x(), lat_state_.y()};

  // 转换为Frenet坐标
  if (!coord_->XYToSL(cart_init_point_, frenet_init_point_)) {
    LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
    return false;
  }

  // 如果路径过短，调整速度
  if (frenet_init_point_.x + ego_v_ * remain_lc_duration >=
      coord_->Length() - 0.5) {
    ego_v_ = (coord_->Length() - frenet_init_point_.x - 0.5) /
             std::fmax(remain_lc_duration, (traj_points.size() - 1) * 0.2);
  }

  return true;
}

bool QuinticPathPlanner::CalculateEndState(const double lat_avoid_offset,
                                           const double lane_change_end_s) {
  // 设置终点状态
  frenet_end_point_ = {lane_change_end_s, lat_avoid_offset};

  // 转换为笛卡尔坐标
  if (!coord_->SLToXY(frenet_end_point_, cart_end_point_)) {
    LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
    return false;
  }

  // 获取终点处的朝向和曲率
  lane_change_end_heading_angle_ =
      coord_->GetPathCurveHeading(lane_change_end_s);
  coord_->GetKappaByS(lane_change_end_s, &lane_change_end_curvature_);

  return true;
}

bool QuinticPathPlanner::ConstructQuinticPath(const double remain_lc_duration) {
  // 计算初始状态的速度和加速度
  const auto heading_angle = lat_state_.theta();
  const auto curvature = lat_state_.curv();

  const auto v_x = ego_v_ * std::cos(heading_angle);
  const auto v_y = ego_v_ * std::sin(heading_angle);

  const auto normal_acc = ego_v_ * ego_v_ * curvature;
  auto normal_acc_x = normal_acc * std::sin(heading_angle);
  auto normal_acc_y = normal_acc * std::cos(heading_angle);

  // 计算终点状态的速度和加速度
  const auto v_x_end = ego_v_ * std::cos(lane_change_end_heading_angle_);
  const auto v_y_end = ego_v_ * std::sin(lane_change_end_heading_angle_);

  const auto normal_acc_end = ego_v_ * ego_v_ * lane_change_end_curvature_;
  auto normal_acc_x_end =
      normal_acc_end * std::sin(lane_change_end_heading_angle_);
  auto normal_acc_y_end =
      normal_acc_end * std::cos(lane_change_end_heading_angle_);

  // 构建五次多项式轨迹
  Eigen::Vector2d x0(ego_state_mgr_->planning_init_point().x,
                     ego_state_mgr_->planning_init_point().y);
  Eigen::Vector2d dx0(v_x, v_y);
  Eigen::Vector2d ddx0(normal_acc_x, normal_acc_y);
  Eigen::Vector2d xT(cart_end_point_.x, cart_end_point_.y);
  Eigen::Vector2d dxT(v_x_end, v_y_end);
  Eigen::Vector2d ddxT(normal_acc_x_end, normal_acc_y_end);

  lane_change_quintic_path_.SetPoints(x0, xT, dx0, dxT, ddx0, ddxT,
                                      remain_lc_duration);

  return true;
}

bool QuinticPathPlanner::SampleTrajectoryPoints(const double remain_lc_duration,
                                                const double lat_avoid_offset,
                                                TrajectoryPoints& traj_points) {
  TrajectoryPoint point;
  Eigen::Vector2d sample_point;
  Point2D frenet_point;
  size_t truncation_idx = 0;
  double s_truncation = 0.0;

  const auto& cart_ref_info = coarse_planning_info_.cart_ref_info;

  for (size_t i = 0; i < traj_points.size(); i++) {
    if (traj_points[i].t < remain_lc_duration) {
      // 在五次多项式轨迹上采样
      sample_point = lane_change_quintic_path_(traj_points[i].t);
      point.x = sample_point.x();
      point.y = sample_point.y();
      point.heading_angle = lane_change_quintic_path_.heading(traj_points[i].t);

      // 转换为Frenet坐标
      Point2D cart_point(point.x, point.y);
      if (!coord_->XYToSL(cart_point, frenet_point)) {
        LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
        return false;
      }

      point.s = frenet_point.x;
      point.l = frenet_point.y;
      point.t = traj_points[i].t;

      traj_points[i] = point;
      truncation_idx = i;
    } else {
      // 轨迹截断处理
      if (i == truncation_idx + 1) {
        Eigen::Vector2d truncation_point(traj_points[truncation_idx].x,
                                         traj_points[truncation_idx].y);

        pnc::spline::Projection projection_truncation_point;
        projection_truncation_point.CalProjectionPoint(
            cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
            cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(),
            truncation_point);

        s_truncation = projection_truncation_point.GetOutput().s_proj;
      }

      // 计算截断后的轨迹点
      point.s = std::fmin(s_truncation + ego_v_ * (i - truncation_idx) * 0.2,
                          coord_->Length());
      point.l = lat_avoid_offset;
      point.t = traj_points[i].t;

      frenet_point.x = point.s;
      frenet_point.y = point.l;

      // 转换回笛卡尔坐标
      Point2D cart_point;
      if (!coord_->SLToXY(frenet_point, cart_point)) {
        LOG_ERROR("ERROR! Cart Point -> Frenet Point Failed!!!");
        return false;
      }

      point.x = cart_point.x;
      point.y = cart_point.y;
      point.heading_angle = coord_->GetPathCurveHeading(point.s);

      traj_points[i] = point;
    }
  }

  return true;
}

}  // namespace planning