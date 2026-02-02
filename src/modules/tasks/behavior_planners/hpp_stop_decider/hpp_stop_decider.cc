#include "hpp_stop_decider.h"

#include <cmath>

#include "common/ifly_time.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "parking_slot_manager.h"
#include "planning_context.h"
#include "reference_path.h"
#include "route_info.h"
#include "session.h"

namespace planning {

HppStopDecider::HppStopDecider(const EgoPlanningConfigBuilder *config_builder,
                               framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<HppStopDeciderConfig>();
  name_ = "HppStopDecider";
  is_stopped_at_destination_ = false;
  is_reached_target_slot_ = false;
  is_reached_target_dest_ = false;
  is_stop_condition_met_ = false;
  last_frame_stop_condition_met_ = false;
  stop_frame_count_ = 0;
}

bool HppStopDecider::Execute() {
  constexpr int kMaxStopFrameCount = 1000;  // 防止数据溢出

  is_stopped_at_destination_ = false;
  is_reached_target_slot_ = false;
  is_reached_target_dest_ = false;
  is_stop_condition_met_ = false;

  const EnvironmentalModel &env = session_->environmental_model();
  const auto &current_reference_path =
      env.get_reference_path_manager()->get_reference_path_by_current_lane();

  if (current_reference_path == nullptr || !current_reference_path->valid()) {
    ILOG_WARN << "HppStopDecider: Invalid reference path";
    return false;
  }

  // 基于 reference path 更新到终点和目标车位的信息
  UpdateTargetInfoBasedOnReferencePath(current_reference_path);

  // 获取自车状态
  const auto &ego_state_manager = env.get_ego_state_manager();
  const double ego_velocity = ego_state_manager->ego_v();
  const double ego_s = current_reference_path->get_frenet_ego_state().s();

  // 获取到目标终点和目标停车位的距离（已更新）
  const auto &route_info_output = env.get_route_info()->get_route_info_output();
  const double dist_to_target_dest =
      route_info_output.hpp_route_info_output.distance_to_target_dest;
  const double dist_to_target_slot =
      route_info_output.hpp_route_info_output.distance_to_target_slot;

  // 判断是否到达目标停车位和目的地
  const auto &parking_slot_manager = env.get_parking_slot_manager();
  const bool is_exist_target_slot = parking_slot_manager->IsExistTargetSlot();

  is_reached_target_slot_ =
      (is_exist_target_slot &&
       dist_to_target_slot < config_.dist_to_target_slot_thr);
  is_reached_target_dest_ =
      (dist_to_target_dest < config_.dist_to_target_dest_thr);

  // 判断是否满足停车条件
  is_stop_condition_met_ = IsStopConditionMet(
      dist_to_target_dest, dist_to_target_slot, ego_velocity);

  // 更新停车帧数计数
  if (is_stop_condition_met_) {
    if (last_frame_stop_condition_met_) {
      stop_frame_count_ = std::min(stop_frame_count_ + 1, kMaxStopFrameCount);
    } else {
      stop_frame_count_ = 1;
    }

    is_stopped_at_destination_ = true;

  } else {
    stop_frame_count_ = 0;
    is_stopped_at_destination_ = false;
  }

  last_frame_stop_condition_met_ = is_stop_condition_met_;

  // 更新 parking_slot_manager 的状态
  session_->mutable_environmental_model()
      ->get_parking_slot_manager()
      ->SetIsReachedTarget(is_reached_target_slot_, is_reached_target_dest_);

  ILOG_DEBUG << "HppStopDecider: dist_to_target_dest=" << dist_to_target_dest
             << ", dist_to_target_slot=" << dist_to_target_slot
             << ", ego_velocity=" << ego_velocity << ", is_stop_condition_met="
             << is_stop_condition_met_ << ", stop_frame_count="
             << stop_frame_count_;

  // 保存输出
  auto &hpp_stop_decider_output =
      session_->mutable_planning_context()->mutable_hpp_stop_decider_output();
  hpp_stop_decider_output.is_stopped_at_destination =
      is_stopped_at_destination_;
  hpp_stop_decider_output.is_reached_target_slot =
      is_reached_target_slot_;
  hpp_stop_decider_output.is_reached_target_dest =
      is_reached_target_dest_;
  hpp_stop_decider_output.is_stop_condition_met =
      is_stop_condition_met_;
  hpp_stop_decider_output.stop_frame_count = stop_frame_count_;

  return true;
}

bool HppStopDecider::IsStopConditionMet(double dist_to_dest,
                                        double dist_to_slot,
                                        double ego_velocity) {
  // 条件1: 距离目标目的地的纵向距离 < 阈值（可调参数）
  bool condition1 = dist_to_dest < config_.dist_to_stop_dest_thr;

  // 条件2: 距离目标停车位的纵向距离 < 阈值（可调参数）
  bool condition2 = dist_to_slot < config_.dist_to_stop_slot_thr;

  // 条件3: 自车处于静止（速度 < 0.1）
  bool condition3 = ego_velocity < config_.ego_still_velocity_thr;

  return condition1 && condition2 && condition3;
}

bool HppStopDecider::UpdateTargetInfoBasedOnReferencePath(
    const std::shared_ptr<ReferencePath> &reference_path) {
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto &parking_slot_manager =
      session_->environmental_model().get_parking_slot_manager();
  double dist_to_target_slot =
      route_info_output.hpp_route_info_output.distance_to_target_slot;
  double dist_to_target_dest =
      route_info_output.hpp_route_info_output.distance_to_target_dest;

  if (reference_path != nullptr && parking_slot_manager->IsExistTargetSlot()) {
    const double ego_s = reference_path->get_frenet_ego_state().s();
    const auto &frenet_coord = reference_path->get_frenet_coord();

    const auto &target_slot_center =
        parking_slot_manager->GetTargetSlotCenter();
    Point2D target_slot_cart_pt(target_slot_center.x(), target_slot_center.y());
    Point2D target_slot_frenet_pt{0.0, 0.0};
    if (frenet_coord->XYToSL(target_slot_cart_pt, target_slot_frenet_pt)) {
      dist_to_target_slot = std::fabs(target_slot_frenet_pt.x - ego_s);
    }

    const auto &target_dest_point =
        route_info_output.hpp_route_info_output.target_dest_point;
    Point2D target_dest_cart_pt(target_dest_point.x(), target_dest_point.y());
    Point2D target_dest_frenet_pt{0.0, 0.0};
    if (frenet_coord->XYToSL(target_dest_cart_pt, target_dest_frenet_pt)) {
      dist_to_target_dest = std::fabs(target_dest_frenet_pt.x - ego_s);
    }
  }
  session_->mutable_environmental_model()->get_route_info()->UpdateTargetInfo(
      dist_to_target_slot, dist_to_target_dest);
  return true;
}

}  // namespace planning
