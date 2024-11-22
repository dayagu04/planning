#include "lane_borrow_deciderv1.h"

#include <limits>

#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "frenet_obstacle.h"
#include "lateral_obstacle.h"
#include "log.h"
#include "math/polygon2d.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tracked_object.h"
#include <cmath>

namespace {
constexpr double kMinDisToSolidLane = 50.0;// 编译求值
constexpr double kMinDisToStopLine = 50.0;
constexpr double kMinDisToCrossWalk = 50.0;
// constexpr double kMaxConcernObsDistance = 50.0; in config
constexpr double kSafeBackDistance = 3.0; // add in config
constexpr double kDefaultStopLineAreaDistance = 5.0;
constexpr double kFilterStopObsDistance = 25.0;
constexpr double kObsSpeedLimit = 3.0;
constexpr double kLatPassableBuffer =
    0.8;  // todo: same with lat decider and lon decider
constexpr double kObsLatBuffer = 0.3;
// constexpr int kObserveFrames = 15; in config
constexpr double kBackwardSafeDistance = 50.0;//# abort
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsSpeedBuffer = 1.0;
constexpr double kObsLatExpendBuffer = 0.4;
constexpr double kObsLonDisBuffer = 2.0;
constexpr double kObsFilterVel = 2.5;
// constexpr double kObsStaticVelThold=0.2; in config
};  // namespace

namespace planning {

bool LaneBorrowDecider::Execute() {
  Update();//1

  LogDebugInfo();//2
  return true;
}

bool LaneBorrowDecider::ProcessEnvInfos() {//1-1
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();
  current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();// lane manager -> cur lane -> reference_path
  left_lane_ptr_ = virtual_lane_manager->get_left_lane();
  right_lane_ptr_ = virtual_lane_manager->get_right_lane();

  if (current_lane_ptr_ == nullptr || current_reference_path_ptr_ == nullptr) {
    LOG_ERROR("No current_lane_ptr_ or current_reference_path_ptr!");
    return false;
  };

  ego_speed_ = session_->environmental_model().get_ego_state_manager()->ego_v();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
lane_change_state_ = session_->planning_context()
                           .lane_change_decider_output()
                           .coarse_planning_info.target_state;//source state  target  state
  if (lane_change_state_ != kLaneKeeping) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = LANE_CHANGE_STATE;
    LOG_ERROR("It has lane change state!");
    return false;
  }

  distance_to_stop_line_ = virtual_lane_manager->GetEgoDistanceToStopline();
  distance_to_cross_walk_ = virtual_lane_manager->GetEgoDistanceToCrosswalk();

  return true;
}

void LaneBorrowDecider::Update() {//1
  if (!ProcessEnvInfos()) {// 1-1
    return;
  }

  switch (lane_borrow_status_) {//1-2
    case LaneBorrowStatus::kNoLaneBorrow: {
      if (CheckIfNoLaneBorrowToLaneBorrowDriving()) {// 切换条件 触发条件 等价于 true CheckLaneBorrowCondition
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      }

      break;
    }
    case LaneBorrowStatus::kLaneBorrowDriving: {
      if (!CheckLaneBorrowCondition()) {// 切换到借道
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfLaneBorrowDrivingToLaneBorrowBackOriginLane()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackOriginLane;// 切换到返回
      }
      break;
    }
    case LaneBorrowStatus::kLaneBorrowBackOriginLane: {
      if (CheckIfLaneBorrowBackOriginLaneToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;//切换到
      } else if (CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;//切换到借道
      }// else
      break;
    }
  }
  // 输出结论
  if (lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = true;
    lane_borrow_decider_output_.lane_borrow_failed_reason = NONE_FAILED_REASON;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_vec_;// 什么时候更新的

  } else {
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_vec_.clear();// 清空？
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_vec_;//None
  }

  session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
      lane_borrow_decider_output_;//结论 左右 true 给出最终方向为左边

  return;
}

bool LaneBorrowDecider::CheckIfNoLaneBorrowToLaneBorrowDriving() {// 非借道 到 借道
  if (!CheckLaneBorrowCondition()) {
    return false;
  }
  return true;
}

void LaneBorrowDecider::ClearLaneBorrowStatus() {
  observe_frame_num_ = 0;
  left_borrow_ = false;
  right_borrow_ = false;
}

bool LaneBorrowDecider::CheckIfLaneBorrowDrivingToLaneBorrowBackOriginLane() {
  if (!IsSafeForBackOriginLane()) {
    return false;
  }
  return true;
}

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving() {//从借道返回 回到 借道[继续借道]
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;// 左右等宽

  for (const auto& obstacle : obstacles) {//遍历 筛选出一个障碍物 使得自车继续借道
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;// 行人不影响借道结束
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;// 非视觉障碍物不影响
    }
    auto it = std::find(static_blocked_obj_vec_.begin(),
                        static_blocked_obj_vec_.end(), id);// 找不到会返回end迭代
    if (it != static_blocked_obj_vec_.end()) {// 如果找到 == 在静态区域的障碍物 不影响借道结束 已经划定区域绕过
      continue;
    }
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start >// 障碍物尾部 在自车车头前方20 以外不影响借道返回
        ego_frenet_boundary_.s_end + kForwardOtherObsDistance) {// 距离 20m 以外
      continue;
    }
    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      continue;// 没有侵入原车道
    }

    const double obs_v = obstacle->obstacle()->velocity();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {//车头前方 动态的不影响借道结束
      if (!obstacle->obstacle()->is_static()) {
        continue;//
      }

    } else {//车头后方
      if (lane_borrow_decider_output_.borrow_direction == 1) {//左借道
        if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start) { // 在自车右侧
          continue;
        }
        if (frenet_obstacle_sl.l_end < -right_width &&
            obstacle->obstacle()->is_static()) {//在原车道右侧以外 并且是静止的 不影响借道结束
          continue;
        }
        if (frenet_obstacle_sl.l_end + kLatPassableBuffer < -right_width) {// 在车道以外不是 静止的 但是足够远离 不影响借道结束
          continue;
        }
        if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <// added 观测速度 * 3.5   仍然在车后方
          ego_frenet_boundary_.s_start) {
          continue;
        }

      } else {// 右侧借道
        if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end) {// 自车左侧不影响
          continue;
        }
        if (frenet_obstacle_sl.l_start > left_width &&
            obstacle->obstacle()->is_static()) {//车道左侧并且静止 的不影响
          continue;
        }
        if (frenet_obstacle_sl.l_start - kLatPassableBuffer > left_width) {//车道左侧够远
          continue;
        }
        if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <// 观测速度 * 3.5   仍然在车后方
            ego_frenet_boundary_.s_start) {
          continue;
        }
      }
    }

    return true;// 某个障碍物 所有条件 no continue 返回继续借道
  }

  return false;// 所有障碍物 在某处都continue  不返回借道
}

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginLaneToNoBorrow() {//从借道返回 到 不借道
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_start) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_start) * 0.5;//后方位置处的宽度为准

  if (ego_frenet_boundary_.l_end < left_width &&
      ego_frenet_boundary_.l_start > -right_width) {//车身边界进入车道内
    ClearLaneBorrowStatus();
    return true;
  } else {
    return false;
  }
}

bool LaneBorrowDecider::CheckLaneBorrowCondition() {//借道触发判断条件
  UpdateJunctionInfo();

  if (forward_solid_start_s_ < kMinDisToSolidLane ||
      distance_to_cross_walk_ < kMinDisToCrossWalk ||
      distance_to_stop_line_ < kMinDisToStopLine) {
    LOG_DEBUG("Ego car is near junction");// 50 m 以内都不借
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  if (!SelectStaticBlockingArea()) {
    return false;
  }

  if (!UpdateLaneBorrowDirection()) {// 什么条件下允许更新借道方向
    return false;
    };



  observe_frame_num_++;
  if (observe_frame_num_ < config_.kObserveFrames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }
  if (lane_borrow_status_ == LaneBorrowStatus::kNoLaneBorrow)
  {
    if (!IsSafeForLaneBorrow2()) {// 更新方向
      return false;
    }
  }

  last_ego_center_position_.first = ego_pose_.first;
  last_ego_center_position_.second = ego_pose_.second;
  return true;
}

bool LaneBorrowDecider::SelectStaticBlockingArea() {
  double xx = config_.kMaxConcernObsDistance;// debug

  const double forward_obs_s =
      std::fmin(current_reference_path_ptr_->get_frenet_coord()->Length(),// 有问题吗？
                ego_frenet_boundary_.s_end + config_.kMaxConcernObsDistance);
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  obs_left_l_ = -left_width;
  obs_right_l_ = right_width;
  obs_start_s_ = forward_obs_s;
  obs_end_s_ = 0.0;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  static_blocked_obj_vec_.clear();
  for (const auto& obstacle : obstacles) {// 遍历 构造静态区域
    int idx = obstacle->obstacle()->id();
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;//去除行人
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }//非行人 纯视觉障碍物
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > forward_obs_s || //障碍物 超过前方静态区域的最大距离
        frenet_obstacle_sl.s_end + kObsLonDisBuffer <// 障碍物头部落后自车尾部超过两米 忽略
            ego_frenet_boundary_.s_start) {  // lon concern area
      continue;
    }
    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      // obstacle is absolutly out ego current lane 障碍物全身在车道外的不考虑
      continue;
    }
    // TODO: concern more scene
    if (frenet_obstacle_sl.l_end < left_width &&// 整个都在该车道的障碍物：//有较大速度的不考虑
        frenet_obstacle_sl.l_start > -right_width) {
      if (obstacle->obstacle()->velocity() > config_.kObsStaticVelThold) {
        continue;
      }
    } else {//部分在该车道的障碍物：
      if (!obstacle->obstacle()->is_static()) {//非静态不考虑
        continue;
      }
    }
    obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
    obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
    obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
    obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);//取极值

    static_blocked_obj_vec_.emplace_back(obstacle->obstacle()->id());//加入这些障碍物id int
  }

  obs_start_s_ = std::max(ego_frenet_boundary_.s_end, obs_start_s_);//障碍物的尾部 自车的头部 靠前的
  if (obs_left_l_ <= obs_right_l_) {// inti -10 left 10 right
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;// 无区域
  }
  if (obs_left_l_ + vehicle_param_.width + kLatPassableBuffer < left_width ||
      obs_right_l_ - vehicle_param_.width - kLatPassableBuffer > -right_width) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = SELF_LANE_ENOUGH;
    return false;//足够宽
  }
  obs_left_l_ += kObsLatBuffer;
  obs_right_l_ -= kObsLatBuffer;//扩张0.8m
  return true;
}

bool LaneBorrowDecider::UpdateLaneBorrowDirection() {// 借道方向

  left_borrow_ = true;// 默认true
  right_borrow_ = true;



  double lane_line_length = 0.0;
  const auto& left_lane_boundarys = current_lane_ptr_->get_left_lane_boundary();
  const auto& right_lane_boundarys =
      current_lane_ptr_->get_right_lane_boundary();
  iflyauto::LaneBoundaryType left_lane_boundary_type;
  iflyauto::LaneBoundaryType right_lane_boundary_type;

  for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
    lane_line_length += left_lane_boundarys.type_segments[i].length;
    if (lane_line_length > vehicle_param_.front_edge_to_rear_axle) {
      left_lane_boundary_type = left_lane_boundarys.type_segments[i].type;
    }
  }
  lane_line_length = 0.0;
  for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
    lane_line_length += right_lane_boundarys.type_segments[i].length;
    if (lane_line_length > vehicle_param_.front_edge_to_rear_axle) {
      right_lane_boundary_type = right_lane_boundarys.type_segments[i].type;
    }
  }

  if (left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    left_borrow_ = false;
  }
  if (left_lane_ptr_ == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  if (right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    right_borrow_ = false;
  }
  if (right_lane_ptr_ == nullptr) {
    right_borrow_ = false;
  }

  // todo: consider ego car near/in stop line or crosswalk area
  if (!left_borrow_ && !right_borrow_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        LANE_TYPE_CHECK_FAILED;
    return false;
  }
  return true;
}

void LaneBorrowDecider::UpdateJunctionInfo() {
  forward_solid_start_s_ = std::numeric_limits<double>::infinity();
  forward_solid_end_s_ = std::numeric_limits<double>::infinity();

  double distance_from_lane_start_to_solid = 0.0;
  const int num_type_segements =
      std::min(current_lane_ptr_->get_left_lane_boundary().type_segments_size,
               current_lane_ptr_->get_right_lane_boundary().type_segments_size);

  for (size_t i = 0; i < num_type_segements; i++) {
    const auto& left_type_segment =
        current_lane_ptr_->get_left_lane_boundary().type_segments[i];
    const auto& right_type_segment =
        current_lane_ptr_->get_right_lane_boundary().type_segments[i];
    if (left_type_segment.begin <= 0 || right_type_segment.begin <= 0) {
      continue;
    }
    if (left_type_segment.type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
        left_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
        left_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED &&
        right_type_segment.type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
        right_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
      forward_solid_start_s_ =
          std::fmin(forward_solid_start_s_, left_type_segment.begin);
      forward_solid_end_s_ =
          std::fmin(left_type_segment.end, right_type_segment.end);
    }

    if (forward_solid_end_s_ >
        current_reference_path_ptr_->get_frenet_coord()->Length()) {
      break;
    }
  }
  return;
}

bool LaneBorrowDecider::IsSafeForLaneBorrow() {
  double right_bounds_l = 0.0;
  double left_bounds_l = 0.0;
  bool safe_to_left_lane_borrow = false;
  double target_l = 0.0;
  double neighbor_left_width = 1.75;  // defualt init
  double neighbor_right_width = 1.75;

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;

  if (left_borrow_) {
    right_bounds_l = obs_left_l_;
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param_.front_edge_to_rear_axle);
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    left_bounds_l =
        current_left_lane_width + neighbor_right_width + neighbor_left_width;
    safe_to_left_lane_borrow = IsSafeForPath(left_bounds_l, right_bounds_l);//key
    target_l = std::min(
        left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5,
        right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5);
    target_l = std::max(target_l, right_bounds_l + vehicle_param_.width * 0.5);
    target_l = std::min(target_l, left_bounds_l - vehicle_param_.width * 0.5);
  }
  bool safe_to_right_lane_borrow = false;
  if (!safe_to_left_lane_borrow && right_borrow_) {// 如果左侧不安全并且右侧车道可变道才会考虑右侧
    left_borrow_ = false;
    left_bounds_l = obs_right_l_;
    if (right_lane_ptr_ == nullptr) {
      std::cout << "right lane is nullptr!" << std::endl;
      return false;
    }
    const double neighbor_width =
        right_lane_ptr_->width(  // todo: add lane ptr protect
            vehicle_param_
                .front_edge_to_rear_axle);  // use ego front bump width
    right_bounds_l =
        -current_right_lane_width - neighbor_left_width - neighbor_right_width;
    safe_to_right_lane_borrow = IsSafeForPath(left_bounds_l, right_bounds_l);
    target_l = std::max(
        right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5,
        left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5);
    target_l = std::min(target_l, left_bounds_l - vehicle_param_.width * 0.5);
  }

  if (!safe_to_left_lane_borrow && !safe_to_right_lane_borrow) {
    return false;
  }
  lane_borrow_decider_output_.target_l = target_l;
  lane_borrow_decider_output_.left_bounds_l = left_bounds_l;
  lane_borrow_decider_output_.right_bounds_l = right_bounds_l;
  lane_borrow_decider_output_.borrow_direction = left_borrow_ ? 1 : 2;// 优先级


  front_pass_sl_point_.first = obs_start_s_;
  front_pass_sl_point_.second = 0.0;
  Point2D frenet_front_pass_point{obs_start_s_, 0.0};

  current_reference_path_ptr_->get_frenet_coord()->SLToXY(
      obs_start_s_, 0.0, &front_pass_point_.first, &front_pass_point_.second);
  return true;
}
// select direction according to delta L
bool LaneBorrowDecider::IsSafeForLaneBorrow2() {
  double right_bounds_l = 0.0;
  double left_bounds_l = 0.0;
  //左侧通行bound
  double left_right_bounds_l = 0.0;
  double left_left_bounds_l = 0.0;
  //右侧通行bound
  double right_right_bounds_l = 0.0;
  double right_left_bounds_l = 0.0;

  bool safe_to_left_lane_borrow = false;
  double target_l = 0.0;
  double target_left_l = 0.0;
  double target_right_l = 0.0;
  double neighbor_left_width = 1.75;  // defualt init
  double neighbor_right_width = 1.75;

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;

  if (left_borrow_) {
    left_right_bounds_l = obs_left_l_;
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param_.front_edge_to_rear_axle);
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    left_left_bounds_l =
        current_left_lane_width + neighbor_right_width + neighbor_left_width;
    safe_to_left_lane_borrow = IsSafeForPath(left_left_bounds_l, left_right_bounds_l);//左侧安全性
    // target_l = std::min(
    //     left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5,
    //     right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5);
    // target_l = std::max(target_l, right_bounds_l + vehicle_param_.width * 0.5);
    // target_l = std::min(target_l, left_bounds_l - vehicle_param_.width * 0.5);
        target_left_l = std::min(
        left_left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5,
        left_right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5);
    target_left_l = std::max(target_left_l, left_right_bounds_l + vehicle_param_.width * 0.5);
    target_left_l = std::min(target_left_l, left_left_bounds_l - vehicle_param_.width * 0.5);
  }
  bool safe_to_right_lane_borrow = false;
  // if (!safe_to_left_lane_borrow && right_borrow_) {// 如果左侧不安全并且右侧车道可变道才会考虑右侧
  if (right_borrow_) {
    left_borrow_ = false;// 现在这个标志只是假设作用 为了复用原来的 IsSafeForPath 逻辑
    right_left_bounds_l = obs_right_l_;
    if (right_lane_ptr_ == nullptr) {
      std::cout << "right lane is nullptr!" << std::endl;
      return false;
    }
    const double neighbor_width =
        right_lane_ptr_->width(  // todo: add lane ptr protect
            vehicle_param_
                .front_edge_to_rear_axle);  // use ego front bump width
    right_right_bounds_l =
        -current_right_lane_width - neighbor_left_width - neighbor_right_width;
    safe_to_right_lane_borrow = IsSafeForPath(right_left_bounds_l, right_right_bounds_l);
    target_right_l = std::max(
        right_right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5,
        right_left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5);
    target_right_l = std::min(target_right_l, right_left_bounds_l - vehicle_param_.width * 0.5);
  }

  //如果都是不安全
  if (!safe_to_left_lane_borrow && !safe_to_right_lane_borrow) {
    return false;
  }
  else if(safe_to_left_lane_borrow&&safe_to_right_lane_borrow)// 都安全
  {
    if(abs(target_left_l) < abs(target_right_l))//左侧
    {
      lane_borrow_decider_output_.target_l = target_left_l;
      lane_borrow_decider_output_.left_bounds_l = left_left_bounds_l;
      lane_borrow_decider_output_.right_bounds_l = left_right_bounds_l;
      lane_borrow_decider_output_.borrow_direction = 1;
    }else {
      lane_borrow_decider_output_.target_l = target_right_l;
      lane_borrow_decider_output_.left_bounds_l = right_left_bounds_l;
      lane_borrow_decider_output_.right_bounds_l = right_right_bounds_l;
      lane_borrow_decider_output_.borrow_direction = 2;
    }
  }
  else if(safe_to_left_lane_borrow)
  {
    lane_borrow_decider_output_.target_l = target_left_l;
    lane_borrow_decider_output_.left_bounds_l = left_left_bounds_l;
    lane_borrow_decider_output_.right_bounds_l = left_right_bounds_l;
    lane_borrow_decider_output_.borrow_direction = 1;
  }else {
    lane_borrow_decider_output_.target_l = target_right_l;
    lane_borrow_decider_output_.left_bounds_l = right_left_bounds_l;
    lane_borrow_decider_output_.right_bounds_l = right_right_bounds_l;
    lane_borrow_decider_output_.borrow_direction = 2;
  }

  front_pass_sl_point_.first = obs_start_s_;
  front_pass_sl_point_.second = 0.0;
  Point2D frenet_front_pass_point{obs_start_s_, 0.0};

  current_reference_path_ptr_->get_frenet_coord()->SLToXY(
      obs_start_s_, 0.0, &front_pass_point_.first, &front_pass_point_.second);
  return true;
}


bool LaneBorrowDecider::IsSafeForBackOriginLane() {
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  // overtake static area
  if(ego_frenet_boundary_.s_start - obs_end_s_ < kSafeBackDistance )
  {
    return false;
  }
  for (const auto& obstacle : obstacles) {// 筛选 障碍物 是否结束借道 开始返回原车道
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (lane_borrow_decider_output_.borrow_direction == 1) {//左借道状态
      if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start && // 障碍物在原车道左边外 并且？
          frenet_obstacle_sl.l_start > left_width) {
        continue;
      }
      if (frenet_obstacle_sl.l_end < -right_width) { // 障碍物在原车道右边外
        continue;
      }
    } else {// 右借道
      if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end &&
          frenet_obstacle_sl.l_end < -right_width) {
        continue;
      }
      if (frenet_obstacle_sl.l_start > left_width) {
        continue;
      }
    }

    if (frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
        kForwardOtherObsDistance) {// 自车前方20m以外不影响返回
      continue;
    }

    const double obs_v = obstacle->obstacle()->velocity();
    if (frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end > //在自车前方 比较近 但是速度很快
            kSafeBackDistance &&// [fixed]
        obs_v > ego_speed_ + kObsSpeedBuffer) { // 速度高于自车
      continue;
    }

    if (frenet_obstacle_sl.s_end >
        ego_frenet_boundary_.s_start - kSafeBackDistance) { // 障碍物在自车后方
      return false;
    }// ：大后方

    if (ego_speed_ - obs_v > kObsSpeedBuffer) {// 自车速度更高直接忽略
      continue;
    }
    if (frenet_obstacle_sl.l_start > ego_frenet_boundary_.l_end ||
        frenet_obstacle_sl.l_end < ego_frenet_boundary_.l_start) { // 不在自车正后方
      const double dist =
          std::max(kSafeBackDistance, obs_v * kObsSpeedRatio);//[fixed]
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
        return false;  // fast come near ego car
      }
    }
  }

  if (lane_borrow_decider_output_.borrow_direction == 1) {
    lane_borrow_decider_output_.right_bounds_l = -right_width;
  } else {
    lane_borrow_decider_output_.left_bounds_l = left_width;
  }

  lane_borrow_decider_output_.target_l = 0.0;
  return true;
}

bool LaneBorrowDecider::IsSafeForPath(const double& left_bounds_l,
                                      const double& right_bounds_l) {
  if (left_bounds_l - right_bounds_l <
      vehicle_param_.width + kObsLatExpendBuffer) {//不会发生？
    lane_borrow_decider_output_.lane_borrow_failed_reason = BOUNDS_TOO_NARROW;
    return false;
  }

  double left_l = left_bounds_l;
  double right_l = right_bounds_l;

  if (left_borrow_) {
    left_l =
        std::min(left_l, right_l + vehicle_param_.width + kObsLatExpendBuffer);
  } else {
    right_l =
        std::max(right_l, left_l - vehicle_param_.width - kObsLatExpendBuffer);
  }

  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;// 当前车道绑定不会发生变化
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();//遍历障碍物
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (!obstacle->b_frenet_valid()) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_start) {// 前方障碍物
      if (obstacle->obstacle()->velocity() > kObsFilterVel) {
        continue;
      }
      if (frenet_obstacle_sl.s_start > obs_end_s_) {
        continue;
      }
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) { // 筛选侵入借道车道的静态障碍物
          continue;
        }
        if (frenet_obstacle_sl.l_end + vehicle_param_.width +
                    kLatPassableBuffer >// 通过左侧所需左侧的bound
                left_bounds_l &&//最大的左侧边界了 一个半车道  借道车道右边被侵入过多才不可通行
            frenet_obstacle_sl.l_start - vehicle_param_.width -
                    kLatPassableBuffer <//借道车道左边被侵入过多才不可通行
                obs_left_l_) {//kObsLatBuffer = 0.3; obs_left_l_ 静态区域的
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      } else {// 右侧借道
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        }

        if (frenet_obstacle_sl.l_end + vehicle_param_.width +
                    kLatPassableBuffer >
                obs_right_l_ &&
            frenet_obstacle_sl.l_end - vehicle_param_.width -
                    kLatPassableBuffer <
                right_bounds_l) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
    } else {// 后方障碍物
      if (frenet_obstacle_sl.l_start < left_l ||
          frenet_obstacle_sl.l_end > right_l) {
        continue;
      }

      const double l_buffer = 0.5;
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start <
            ego_frenet_boundary_.l_end - l_buffer) {
          continue;
        }
      } else {
        if (frenet_obstacle_sl.l_end <
            ego_frenet_boundary_.l_start + l_buffer) {
          continue;
        }
      }

      double dist = std::max(kSafeBackDistance,
                             obstacle->obstacle()->velocity() * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {// 后
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            BACKWARD_OBSTACLE_TOO_CLOSE;
        lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
        return false;
      }
    }
  }
  return true;
}
bool LaneBorrowDecider::IsSafeForPath2(const double& left_bounds_l, const double& right_bounds_l)
{
// 输入左右通行信号时候的初始bound[基于车道线和静态区域] 然后开始遍历障碍物
// 输出综合的是否安全(左or右)

//输入 左右通行信号 遍历障碍物
//输出 左右所需移动横向距离
  if (left_bounds_l - right_bounds_l <
      vehicle_param_.width + kObsLatExpendBuffer) {//不会发生？
    lane_borrow_decider_output_.lane_borrow_failed_reason = BOUNDS_TOO_NARROW;
    return false;
  }

  double left_l = left_bounds_l;
  double right_l = right_bounds_l;

  if (left_borrow_) {
    left_l =
        std::min(left_l, right_l + vehicle_param_.width + kObsLatExpendBuffer);
  } else {
    right_l =
        std::max(right_l, left_l - vehicle_param_.width - kObsLatExpendBuffer);
  }

  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;// 当前车道绑定不会发生变化
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();//遍历障碍物
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (!obstacle->b_frenet_valid()) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_start) {// 前方障碍物
      if (obstacle->obstacle()->velocity() > kObsFilterVel) {
        continue;
      }
      if (frenet_obstacle_sl.s_start > obs_end_s_) {
        continue;
      }
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) { // 筛选侵入借道车道的静态障碍物
          continue;
        }
        if (frenet_obstacle_sl.l_end + vehicle_param_.width +
                    kLatPassableBuffer >// 通过左侧所需左侧的bound
                left_bounds_l &&//最大的左侧边界了 一个半车道  借道车道右边被侵入过多才不可通行
            frenet_obstacle_sl.l_start - vehicle_param_.width -
                    kLatPassableBuffer <//借道车道左边被侵入过多才不可通行
                obs_left_l_) {//kObsLatBuffer = 0.3; obs_left_l_ 静态区域的
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      } else {// 右侧借道
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        }

        if (frenet_obstacle_sl.l_end + vehicle_param_.width +
                    kLatPassableBuffer >
                obs_right_l_ &&
            frenet_obstacle_sl.l_end - vehicle_param_.width -
                    kLatPassableBuffer <
                right_bounds_l) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
    } else {// 后方障碍物
      if (frenet_obstacle_sl.l_start < left_l ||
          frenet_obstacle_sl.l_end > right_l) {
        continue;
      }

      const double l_buffer = 0.5;
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start <
            ego_frenet_boundary_.l_end - l_buffer) {
          continue;
        }
      } else {
        if (frenet_obstacle_sl.l_end <
            ego_frenet_boundary_.l_start + l_buffer) {
          continue;
        }
      }

      double dist = std::max(kSafeBackDistance,
                             obstacle->obstacle()->velocity() * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {// 后
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            BACKWARD_OBSTACLE_TOO_CLOSE;
        lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
        return false;
      }
    }
  }
  return true;

}
void LaneBorrowDecider::LogDebugInfo() {
  // debug info
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_lane_borrow_failed_reason(
      lane_borrow_decider_output_.lane_borrow_failed_reason);
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();

  const auto& current_frenet_coord = current_reference_path->get_frenet_coord();

  Point2D front_left_corner, front_right_corner, back_right_corner,
      back_left_corner;
  current_frenet_coord->SLToXY(obs_end_s_, obs_left_l_, &front_left_corner.x,
                               &front_left_corner.y);
  current_frenet_coord->SLToXY(obs_end_s_, obs_right_l_, &front_right_corner.x,
                               &front_right_corner.y);
  current_frenet_coord->SLToXY(obs_start_s_, obs_right_l_, &back_right_corner.x,
                               &back_right_corner.y);
  current_frenet_coord->SLToXY(obs_start_s_, obs_left_l_, &back_left_corner.x,
                               &back_left_corner.y);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_left_corner()
      ->set_x(front_left_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_left_corner()
      ->set_y(front_left_corner.y);

  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_right_corner()
      ->set_x(front_right_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_right_corner()
      ->set_y(front_right_corner.y);

  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_left_corner()
      ->set_x(back_left_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_left_corner()
      ->set_y(back_left_corner.y);

  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_right_corner()
      ->set_x(back_right_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_right_corner()
      ->set_y(back_right_corner.y);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_left_l(obs_left_l_);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_right_l(obs_right_l_);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_start_s(obs_start_s_);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_end_s(obs_end_s_);

  lane_borrow_pb_info->set_lane_borrow_decider_status(lane_borrow_status_);

  lane_borrow_pb_info->mutable_static_blocked_obj_vec()->Clear();
  for (auto static_obs_id : static_blocked_obj_vec_) {
    lane_borrow_pb_info->mutable_static_blocked_obj_vec()->Add(static_obs_id);
  }
}

}  // namespace planning