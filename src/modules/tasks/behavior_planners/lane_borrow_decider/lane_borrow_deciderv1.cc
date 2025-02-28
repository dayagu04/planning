#include "lane_borrow_deciderv1.h"

#include <Eigen/src/Core/Matrix.h>
#include <math.h>

#include <cmath>

#include "basic_types.pb.h"
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/traffic_light_decider/traffic_light_decider.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "frenet_obstacle.h"
#include "lane_borrow_decider.pb.h"
#include "lateral_obstacle.h"
#include "log.h"
#include "math/polygon2d.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "pose2d.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tracked_object.h"
#include "utils/cartesian_coordinate_system.h"
#include "utils/pose2d_utils.h"

namespace {
constexpr double kMinDisToSolidLane = 50.0;
constexpr double kMinDisToStopLine = 50.0;
constexpr double kMinDisToCrossWalk = 50.0;
constexpr double kMinDisToTrafficLight = 120.0;
constexpr double kInfDisToTrafficLight = 10000.0;
constexpr double kSafeBackDistance = 2.0;
constexpr double kDefaultStopLineAreaDistance = 5.0;
constexpr double kFilterStopObsDistance = 25.0;
constexpr double kObsSpeedLimit = 3.0;
constexpr double kLatPassableBuffer = 0.8;
constexpr double kObsLatBuffer = 0.3;
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsSpeedBuffer = 1.0;
constexpr double kObsLatExpendBuffer = 0.4;
constexpr double kObsLonDisBuffer = 2.0;
constexpr double kObsFilterVel = 2.5;
constexpr double kBlockHeading = 0.17;
constexpr double kCheckTurningDistance = 10.0;
constexpr double kMaxCentricOffset = 0.75;
};  // namespace

namespace planning {

bool LaneBorrowDecider::Execute() {
  Update();

  LogDebugInfo();
  return true;
}

bool LaneBorrowDecider::ProcessEnvInfos() {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();
  current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();
  left_lane_ptr_ = virtual_lane_manager->get_left_lane();
  right_lane_ptr_ = virtual_lane_manager->get_right_lane();

  lane_borrow_decider_output_.lane_borrow_failed_reason =
      planning::LaneBorrowFailedReason::NONE_FAILED_REASON;
  const auto traffic_light_manager =
      session_->environmental_model().get_traffic_light_decision_manager();
  const auto all_traffic_lights = traffic_light_manager->GetTrafficLightsInfo();
  dis_to_traffic_lights_ = kInfDisToTrafficLight;
  // 检测是否有红绿灯，并输出距离
  for (int i = 0; i < all_traffic_lights.size(); i++) {
    if (all_traffic_lights[i].traffic_light_x > 0 &&
        all_traffic_lights[i].traffic_light_x < dis_to_traffic_lights_) {
      dis_to_traffic_lights_ = all_traffic_lights[i].traffic_light_x;
    }
  }
  // 红绿灯距离赋值到指针下面的属性
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_dis_to_traffic_lights(dis_to_traffic_lights_);
  // 检测当前位置是否有车道线和参考线，没有输出log
  if (current_lane_ptr_ == nullptr || current_reference_path_ptr_ == nullptr) {
    LOG_ERROR("No current_lane_ptr_ or current_reference_path_ptr!");
    return false;
  };

  ego_speed_ = session_->environmental_model().get_ego_state_manager()->ego_v();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  heading_angle_ = session_->environmental_model()
                       .get_reference_path_manager()
                       ->get_reference_path_by_current_lane()
                       ->get_frenet_ego_state()
                       .heading_angle();
  lane_change_state_ = session_->planning_context()
                           .lane_change_decider_output()
                           .coarse_planning_info.target_state;
  if (lane_change_state_ != kLaneKeeping) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = LANE_CHANGE_STATE;
    LOG_ERROR("It has lane change state!");
    return false;
  }

  // 获取交叉口状态并判断，除了不是在交叉口，都返回true
  intersection_state_ = virtual_lane_manager->GetIntersectionState();
  if (intersection_state_ !=
      planning::common::IntersectionState::NO_INTERSECTION) {
    return false;
  }

  distance_to_stop_line_ = virtual_lane_manager->GetEgoDistanceToStopline();
  distance_to_cross_walk_ = virtual_lane_manager->GetEgoDistanceToCrosswalk();

  return true;
}

// processenvinfos 返回false
// update返回true直接返回，不执行后续代码，表示处理环境信息失败 processenvinfos
// 返回true  update返回false，执行后续，表示处理环境信息成功
void LaneBorrowDecider::Update() {
  if (!ProcessEnvInfos()) {
    return;
  }

  switch (lane_borrow_status_) {
    // case1 无车道借用case  检测是否可以从无车道借用状态转换为车道借用状态
    case LaneBorrowStatus::kNoLaneBorrow: {
      if (CheckIfNoLaneBorrowToLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      }

      break;
    }
    // case2 借道状态case 如果CheckLaneBorrowCondition返回false 执行切状态
    // 如果返回true并且CheckIfLaneBorrowDrivingToLaneBorrowBackOriginLane也是true，切状态
    // kLaneBorrowBackOriginLane 应该是借道后返回自车原始车道
    case LaneBorrowStatus::kLaneBorrowDriving: {
      if (!CheckLaneBorrowCondition()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (ChecekIfLaneBorrowToLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
        // } else if (CheckIfLaneBorrowDrivingToLaneBorrowBackOriginLane()) {
        //   lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackOriginLane;
      }
      break;
    }
    // case 3 跨线借道
    case LaneBorrowStatus::kLaneBorrowCrossing: {
      if (CheckIfkLaneBorrowCrossingToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (IsSafeForBackOriginLane()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackOriginLane;
      }
      break;
    }
    // case4 借道回自车道case 如果借道回自车道到不借道为true，切状态
    // 如果为false并且借道回自车道到借道为true 则切状态
    case LaneBorrowStatus::kLaneBorrowBackOriginLane: {
      if (CheckIfLaneBorrowBackOriginLaneToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      } else if (CheckIfLaneBorrowBackOriginToLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
      }
      break;
    }
  }
  last_static_blocked_obj_id_vec_ = static_blocked_obj_id_vec_;
  // 如果状态不为不借道，那么正在借道状态为true，换到失败原因为none，静态障碍物id赋给阻塞障碍物
  if (lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = true;
    lane_borrow_decider_output_.lane_borrow_failed_reason = NONE_FAILED_REASON;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;

  } else {
    // 如果状态为不借道，那么正在借道状态为false，清空静态阻塞障碍物id，在输给借道输出阻塞障碍物id
    // 借道方向为no
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_id_vec_.clear();
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
  }

  session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
      lane_borrow_decider_output_;

  return;
}

// 通过调用 CheckLaneBorrowCondition
// 函数来判断是否存在车道借用的条件，并根据条件的结果返回相应的布尔值。
// 如果条件不满足（即 CheckLaneBorrowCondition 返回 false），则函数返回
// false；否则返回 true。
bool LaneBorrowDecider::CheckIfNoLaneBorrowToLaneBorrowDriving() {
  if (!CheckLaneBorrowCondition()) {
    return false;
  }
  return true;
}

// 清空状态函数
void LaneBorrowDecider::ClearLaneBorrowStatus() {
  observe_frame_num_ = 0;
  left_borrow_ = false;
  right_borrow_ = false;
  obs_direction_map_.clear();
}

// 如果回自车道安全，则借道切到借道后回自车道为true
bool LaneBorrowDecider::CheckIfLaneBorrowDrivingToLaneBorrowBackOriginLane() {
  if (!IsSafeForBackOriginLane()) {
    return false;
  }
  return true;
}

// 借道后回自车道切借道状态，这个函数是判断回自车道不安全的逻辑
bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving() {
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  // 遍历障碍物每个元素，并检测类型
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    // 检测是行人 跳过该障碍物
    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;
    }
    // 如果不是像头检测出来的，过滤掉
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    // it为迭代器，存std::find返回的id结果，如果找到这个值，返回指向该值迭代器，否则返回指向末尾
    auto it = std::find(static_blocked_obj_id_vec_.begin(),
                        static_blocked_obj_id_vec_.end(), id);
    // 如果 it 不等于
    // static_blocked_obj_id_vec_.end()，说明找到了元素；否则，没有找到
    if (it != static_blocked_obj_id_vec_.end()) {
      continue;
    }
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    // 障碍物距离自车过远，滤除
    if (frenet_obstacle_sl.s_start >
        ego_frenet_boundary_.s_end + kForwardOtherObsDistance) {
      continue;
    }
    // 超过边界，滤除
    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      continue;
    }

    const double obs_v = obstacle->obstacle()->velocity();
    // 取出obs的速度放入obs_v中，如果障碍物在自车前方，且是动态的，则countinue
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      if (!obstacle->obstacle()->is_static()) {
        continue;
      }

      // obs在自车后方
    } else {
      if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
        // 首先检查是否正在向左借用车道。
        // 然后检查是否存在一个障碍物，该障碍物的左端位置超过了当前车辆的右端位置。
        // 满足，则跳过当前迭代，继续处理后续逻辑。
        if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start) {
          continue;
        }

        if (frenet_obstacle_sl.l_end < -right_width &&
            obstacle->obstacle()->is_static()) {
          continue;  // 如果障碍物是静态的并且其左端位置小于负的右宽度，跳过当前迭代。
        }
        // 检查障碍物的左端位置加上可通行缓冲区是否小于负的右宽度。
        if (frenet_obstacle_sl.l_end + kLatPassableBuffer < -right_width) {
          continue;  // 满足则跳过迭代
        }
        // 检测动态位置与自车起始位置关系
        if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <
            ego_frenet_boundary_.s_start) {
          continue;
        }

        // 右借道
      } else {
        if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end) {
          continue;
        }
        if (frenet_obstacle_sl.l_start > left_width &&
            obstacle->obstacle()->is_static()) {
          continue;
        }
        if (frenet_obstacle_sl.l_start - kLatPassableBuffer > left_width) {
          continue;
        }
        if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <
            ego_frenet_boundary_.s_start) {
          continue;
        }
      }
    }

    // 如果找到符合条件的障碍物，返回 true。
    // 如果遍历完所有障碍物后没有找到符合条件的，返回 false。
    return true;
  }

  return false;
}

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginLaneToNoBorrow() {
  // 借道回自车道状态切为不借道
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_start) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_start) * 0.5;

  if (ego_frenet_boundary_.l_end < left_width &&
      ego_frenet_boundary_.l_start > -right_width) {
    ClearLaneBorrowStatus();
    return true;
  } else {
    return false;
  }
}

bool LaneBorrowDecider::CheckLaneBorrowCondition() {
  UpdateJunctionInfo();

  // 判断是否在交叉路口区域，如果在改原因为交叉路口并返回false
  // 通过一系列的条件检查和函数调用，决定车辆是否可以借用车道。
  // 如果任何一个条件不满足或者相关函数返回 false，则最终返回 false，
  // 表示不能借用车道；否则，继续执行后续逻辑
  if ((forward_solid_start_dis_ < kMinDisToSolidLane &&
       forward_solid_start_dis_ > 0) ||
      (distance_to_cross_walk_ < kMinDisToCrossWalk &&
       distance_to_cross_walk_ > 0.0) ||
      (distance_to_stop_line_ < kMinDisToStopLine &&
       distance_to_stop_line_ > 0.0) ||
      (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
       dis_to_traffic_lights_ > 0.0)) {
    LOG_DEBUG("Ego car is near junction");
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  // 更新车道借用的方向，根据边界类型和车道线虚实类型判断左右借道的可行性。如果该函数返回
  // false，表示无法确定车道借用方向，因此直接返回 false
  if (!UpdateLaneBorrowDirection()) {
    return false;
  }

  // shaixuan可能阻碍车道借用的静态障碍物。如果该函数返回
  // false，表示无法找到合适的障碍物，因此直接返回 false
  if (!SelectStaticBlockingObstcales()) {
    return false;
  }

  // 进行障碍物决策返回false决策失败
  if (!ObstacleDecision()) {
    return false;
  }

  // 判断观测时间如果小于最小观测帧数，则false原因为观测时间不足
  observe_frame_num_++;
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  // 函数返回false，则返回false表示不允许借用
  if (!IsSafeForLaneBorrow()) {
    return false;
  }

  // 如果上述都不满足，则返回true，表示可以借用
  return true;
}

// 筛选阻塞静态障碍物函数
bool LaneBorrowDecider::SelectStaticBlockingObstcales() {
  // 取forward_obs_s为当前frenet路径长和自车boundray和配置文件中obs距离最小数之和两者之间取小值
  const double forward_obs_s =
      std::fmin(current_reference_path_ptr_->get_frenet_coord()->Length(),
                ego_frenet_boundary_.s_end + config_.max_concern_obs_distance);
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  obs_left_l_ = -left_width;
  obs_right_l_ = right_width;
  obs_start_s_ = forward_obs_s;
  obs_end_s_ = 0.0;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  static_blocked_obstacles_.clear();
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  for (const auto& obstacle : obstacles) {
    int idx = obstacle->obstacle()->id();
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    // 纵向考虑静态区域
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > forward_obs_s ||
        frenet_obstacle_sl.s_end + kObsLonDisBuffer <
            ego_frenet_boundary_.s_start) {  // lon concern area
      continue;
    }
    const auto& vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    //  no lon overlap
    // 检查障碍物的起始位置是否超出了车辆的有效范围//横向滤除
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end ||
        frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
      const auto lat_obs_iter = lat_obstacle_decision.find(id);
      if (lat_obs_iter != lat_obstacle_decision.end() &&
          lat_obs_iter->second != LatObstacleDecisionType::IGNORE) {
        continue;
      }
    } else {  // lon overlap
      auto it = std::find(last_static_blocked_obj_id_vec_.begin(),
                          last_static_blocked_obj_id_vec_.end(), id);
      if (it == last_static_blocked_obj_id_vec_.end()) {
        continue;
      }
    }
    // TODO: concern more scene
    // 检测障碍物在width内，如果在如果是动态的
    // 跳过循环，不在width内，动态的也跳出循环
    if (frenet_obstacle_sl.l_end < left_width &&
        frenet_obstacle_sl.l_start > -right_width) {
      if (!obstacle->obstacle()->is_static()) {
        continue;
      }
    } else {
      if (!obstacle->obstacle()->is_static()) {
        continue;
      }
    }
    // if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowCrossing) {
    //   const double distance_to_ego = frenet_obstacle_sl.s_start -
    //   ego_frenet_boundary_.s_end; if (distance_to_ego >
    //   far_obs_distance_threshold_ &&
    //       std::fabs(frenet_obstacle_sl.l_start + frenet_obstacle_sl.l_end )
    //       <= kMaxCentricOffset) {

    //         // 纵向距离判断：与已选障碍物末端距离小于合并阈值
    //     if (frenet_obstacle_sl.s_start - obs_end_s_ < merge_obs_distance_) {
    //         // 合并边界
    //       merged_obs_boundary_.s_start =
    //       std::min(merged_obs_boundary_.s_start, frenet_obstacle_sl.s_start);
    //       merged_obs_boundary_.s_end = std::max(merged_obs_boundary_.s_end,
    //       frenet_obstacle_sl.s_end); merged_obs_boundary_.l_start =
    //       std::min(merged_obs_boundary_.l_start, frenet_obstacle_sl.l_start);
    //       merged_obs_boundary_.l_end = std::max(merged_obs_boundary_.l_end,
    //       frenet_obstacle_sl.l_end);
    //     }
    //   }
    // }

    static_blocked_obstacles_.emplace_back(
        obstacle);  // 将障碍物添加到列表中  // really needed
    static_blocked_obj_id_vec_.emplace_back(id);  // tmperal used
    if (obs_direction_map_.empty() ||
        obs_direction_map_.find(id) == obs_direction_map_.end()) {  // add
      obs_direction_map_[id] = std::make_pair(BorrowDirection::NO_BORROW, 0);
    }
  }
  // delete disappear obs
  for (auto it = obs_direction_map_.begin(); it != obs_direction_map_.end();) {
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  it->first) == static_blocked_obj_id_vec_.end()) {
      it = obs_direction_map_.erase(it);
    } else {
      ++it;
    }
  }

  return true;
}
// obs决策函数
bool LaneBorrowDecider::ObstacleDecision() {
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  static_blocked_obj_id_vec_.clear();
  bypass_direction_ = NO_BORROW;
  // 没有阻塞车辆，设置false原因为无obs
  if (static_blocked_obstacles_.empty()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
    // 容器的大小大于1时，对其元素按照 frenet_s() 的值进行升序排序。
    // 排序后，static_blocked_obstacles_ 中的元素将按 frenet_s()
    // 从小到大排列（按照纵向位置排序）
  } else if (static_blocked_obstacles_.size() > 1) {
    std::sort(static_blocked_obstacles_.begin(),
              static_blocked_obstacles_.end(),
              [](const std::shared_ptr<FrenetObstacle>& a,
                 const std::shared_ptr<FrenetObstacle>& b) -> bool {
                return a->frenet_s() < b->frenet_s();
              });
  }

  if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowCrossing) {
    for (const auto& obstacle : static_blocked_obstacles_) {
      const auto& id = obstacle->obstacle()->id();
      const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
      BorrowDirection obs_bypass_direction =
          GetBypassDirection(frenet_obstacle_sl, id);

      //--------------------------------------------
      //       -------------
      //      |         ____|         将多个obs囊括到一起当成一个整体
      //      |____     |  ||
      //      ||  |     |  ||
      //      ||  |     ----|
      //      |----         |
      //      ---------------
      //----------------------------------------------
      // if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowCrossing) {
      //     obs_left_l_ = merged_obs_boundary_.l_end;
      //     obs_right_l_ = merged_obs_boundary_.l_start;
      //     obs_start_s_ = merged_obs_boundary_.s_start;
      //     obs_end_s_ = merged_obs_boundary_.s_end;
      // }
      // 不管obs位姿，加入距离判断
      if (id == static_blocked_obstacles_[0]->obstacle()->id() ||
          frenet_obstacle_sl.s_start - obs_end_s_ < merge_obs_distance_) {
        obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
        obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
        obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
        obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);
        static_blocked_obj_id_vec_.emplace_back(obstacle->obstacle()->id());
      }
    }
  } else {
    // 将第一个obs的boundary赋值给front_obs_sl
    const auto& front_obstacle_sl =
        static_blocked_obstacles_[0]->frenet_obstacle_boundary();
    const auto& id = static_blocked_obstacles_[0]->obstacle()->id();
    BorrowDirection front_obs_bypass_direction =
        GetBypassDirection(front_obstacle_sl, id);
    const double front_obs_center_l =
        0.5 * (front_obstacle_sl.l_start + front_obstacle_sl.l_end);
    lane_borrow_pb_info->set_front_obs_center(front_obs_center_l);

    // 根据当前自车和障碍物位置信息和借道状态来决定是否借道以及那边借道，如果不借道则false
    if (front_obs_bypass_direction == LEFT_BORROW && left_borrow_) {
      bypass_direction_ = LEFT_BORROW;
      right_borrow_ = false;
    } else if (front_obs_bypass_direction == RIGHT_BORROW && right_borrow_) {
      bypass_direction_ = RIGHT_BORROW;
      left_borrow_ = false;
    } else {
      bypass_direction_ = NO_BORROW;
      lane_borrow_decider_output_.lane_borrow_failed_reason = CENTER_OBSTACLE;
      return false;
    }

    for (const auto& obstacle : static_blocked_obstacles_) {
      const auto& id = obstacle->obstacle()->id();
      const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
      BorrowDirection obs_bypass_direction =
          GetBypassDirection(frenet_obstacle_sl, id);

      // 如果对障碍物绕行方向=绕行方向，则更新障碍物信息的最大最小边界信息
      // 如果绕行方向不匹配，退出循环，break

      if (obs_bypass_direction == bypass_direction_) {
        obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
        obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
        obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
        obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);
        static_blocked_obj_id_vec_.emplace_back(obstacle->obstacle()->id());
      } else {
        // too dense obstacles
        const double dist = frenet_obstacle_sl.s_start - obs_end_s_;
        if (dist < config_.dense_obstacle_dist) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              CENTER_OBSTACLE;
          return false;
        }
        break;
      }
    }
  }

  if (obs_left_l_ <= obs_right_l_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }

  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  // 自车道空间足够不需要借道返回false
  if (obs_left_l_ + vehicle_param.width + kLatPassableBuffer < left_width ||
      obs_right_l_ - vehicle_param.width - kLatPassableBuffer > -right_width) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = SELF_LANE_ENOUGH;
    return false;
  }
  // 如果空间符合，则将bufffer距离加到obs边界上，增加安全阈值
  obs_left_l_ += kObsLatBuffer;
  obs_right_l_ -= kObsLatBuffer;
  return true;
}

// 借道方向
BorrowDirection LaneBorrowDecider::GetBypassDirection(
    const FrenetObstacleBoundary& frenet_obstacle_sl, const int obs_id) {
  // 计算obs的中心位置
  const double obs_center_l =
      0.5 * (frenet_obstacle_sl.l_start + frenet_obstacle_sl.l_end);
  // center位置距离车道中心的距离在允许的范围内，不饶
  // 其次根据在左or右选择左绕或右绕
  if (std::fabs(obs_center_l) <= kMaxCentricOffset) {
    if (obs_direction_map_[obs_id].second < config_.centric_obs_frames) {
      obs_direction_map_[obs_id].second += 1;
      return obs_direction_map_[obs_id].first;
    } else {
      obs_direction_map_[obs_id].first = NO_BORROW;
      return NO_BORROW;
    }
  } else if (obs_center_l < -kMaxCentricOffset) {
    obs_direction_map_[obs_id].first = LEFT_BORROW;
    obs_direction_map_[obs_id].second = 0;
    return LEFT_BORROW;
  } else {
    obs_direction_map_[obs_id].first = RIGHT_BORROW;
    obs_direction_map_[obs_id].second = 0;
    return RIGHT_BORROW;
  }
}

// 更新借道方向 // 主要看车道边界类型和车道线类型
bool LaneBorrowDecider::UpdateLaneBorrowDirection() {
  left_borrow_ = true;
  right_borrow_ = true;

  // 初始化车道长度并提出车道边界类型
  double lane_line_length = 0.0;
  const auto& left_lane_boundarys = current_lane_ptr_->get_left_lane_boundary();
  const auto& right_lane_boundarys =
      current_lane_ptr_->get_right_lane_boundary();
  iflyauto::LaneBoundaryType left_lane_boundary_type;
  iflyauto::LaneBoundaryType right_lane_boundary_type;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  // 确定车辆左侧车道边界的类型，通过累加车道线段的长度并与车辆前轴到后轴的距离进行比较来实现。
  // 当累加长度超过车辆前轴到后轴的距离时，记录当前段的类型并终止循环
  for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
    lane_line_length += left_lane_boundarys.type_segments[i].length;
    if (lane_line_length > vehicle_param.front_edge_to_rear_axle) {
      left_lane_boundary_type = left_lane_boundarys.type_segments[i].type;
      break;
    }
  }
  // 确定车辆右侧边界类型
  lane_line_length = 0.0;
  for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
    lane_line_length += right_lane_boundarys.type_segments[i].length;
    if (lane_line_length > vehicle_param.front_edge_to_rear_axle) {
      right_lane_boundary_type = right_lane_boundarys.type_segments[i].type;
      break;
    }
  }

  // 如果车道线不是虚线，左虚右实，双虚线，左绕返回false
  if (left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    left_borrow_ = false;
  }
  // 如果做测车道线信息为空，也输出false
  if (left_lane_ptr_ == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  // 右同左一样，根据车道线类型来判别能不能借道
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

// 检查给定的车道边界类型是否是虚线、左实右虚或双虚线中的任何一种，
// 如果是则返回 true，否则返回 false
bool LaneBorrowDecider::IsLaneTypeDashedOrMixed(
    const iflyauto::LaneBoundaryType& type) {
  return type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED;
}

// 更新路口信息函数
void LaneBorrowDecider::UpdateJunctionInfo() {
  // 初始化到前方实现的距离和到前方实现的s为double的最大值
  forward_solid_start_dis_ = std::numeric_limits<double>::max();
  forward_solid_end_s_ = std::numeric_limits<double>::max();
  // 当前无车道，返回
  if (current_lane_ptr_->lane_points().empty()) {
    return;
  }

  const auto& current_lane_points = current_lane_ptr_->lane_points();
  bool found_start = false;

  for (size_t i = 0; i < current_lane_points.size(); ++i) {
    const auto& lane_point = current_lane_points[i];
    // 如果还没找到起点，车道类型不是虚线和混合类型，且前方s大于自车起点位置
    if (!found_start &&
        !IsLaneTypeDashedOrMixed(lane_point.left_lane_border_type) &&
        !IsLaneTypeDashedOrMixed(lane_point.right_lane_border_type) &&
        lane_point.s > ego_frenet_boundary_.s_start) {
      // 计算并存储从起始点到当前点的前向距离。
      // found_start: 将 found_start 设置为
      // true，表示已经找到了符合条件的起始点。
      forward_solid_start_dis_ = lane_point.s - ego_frenet_boundary_.s_start;
      found_start = true;
    }

    // 如果找到起点，并是虚线或者混合类型车道线，而且前方s大于自车起点位置
    // 找到第一个符合条件的点后，计算其位置与起始位置之间的距离，然后退出循环
    if (found_start &&
        (IsLaneTypeDashedOrMixed(lane_point.left_lane_border_type) ||
         IsLaneTypeDashedOrMixed(lane_point.right_lane_border_type)) &&
        lane_point.s > ego_frenet_boundary_.s_start) {
      forward_solid_end_s_ =
          lane_point.s - ego_frenet_boundary_.s_start;  // bounding
      break;
    }
  }

  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_start_solid_lane_dis(forward_solid_start_dis_);
  lane_borrow_pb_info->set_end_solid_lane_dis(forward_solid_end_s_);
}

bool LaneBorrowDecider::IsSafeForLaneBorrow() {
  double right_bounds_l = 0.0;
  double left_bounds_l = 0.0;

  double left_right_bounds_l = 0.0;
  double left_left_bounds_l = 0.0;

  double right_right_bounds_l = 0.0;
  double right_left_bounds_l = 0.0;

  bool safe_to_left_lane_borrow = false;
  double target_l = 0.0;
  double target_left_l = 0.0;
  double target_right_l = 0.0;
  double neighbor_left_width = 1.75;
  double neighbor_right_width = 1.75;

  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (left_borrow_) {
    left_right_bounds_l = obs_left_l_;
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    // 计算相邻车道的左右边界宽度
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    // 计算可以借用到左侧车道的总宽度
    left_left_bounds_l =
        current_left_lane_width + neighbor_right_width + neighbor_left_width;
    // 判断是否可以安全地借用到左侧车道
    safe_to_left_lane_borrow =
        IsSafeForPath(left_left_bounds_l, left_right_bounds_l);
    // 计算目标位置，确保车辆不会超出可通行区域
    target_left_l = std::min(
        left_left_bounds_l - kLatPassableBuffer - vehicle_param.width * 0.5,
        left_right_bounds_l + kLatPassableBuffer + vehicle_param.width * 0.5);
    target_left_l = std::max(target_left_l,
                             left_right_bounds_l + vehicle_param.width * 0.5);
    target_left_l =
        std::min(target_left_l, left_left_bounds_l - vehicle_param.width * 0.5);
  }
  bool safe_to_right_lane_borrow = false;
  // 右借道类比左借道
  if (right_borrow_) {
    left_borrow_ = false;
    right_left_bounds_l = obs_right_l_;
    if (right_lane_ptr_ == nullptr) {
      std::cout << "right lane is nullptr!" << std::endl;
      return false;
    }
    const double neighbor_width =
        right_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    right_right_bounds_l =
        -current_right_lane_width - neighbor_left_width - neighbor_right_width;
    safe_to_right_lane_borrow =
        IsSafeForPath(right_left_bounds_l, right_right_bounds_l);
    target_right_l = std::max(
        right_right_bounds_l + kLatPassableBuffer + vehicle_param.width * 0.5,
        right_left_bounds_l - kLatPassableBuffer - vehicle_param.width * 0.5);
    target_right_l = std::min(target_right_l,
                              right_left_bounds_l - vehicle_param.width * 0.5);
  }
  double target_borrow_left = target_left_l;
  double target_borrow_right = target_right_l;
  lane_borrow_pb_info->set_target_left_l(target_borrow_left);
  lane_borrow_pb_info->set_target_right_l(target_borrow_right);
  lane_borrow_pb_info->set_safe_left_borrow(safe_to_left_lane_borrow);
  lane_borrow_pb_info->set_safe_right_borrow(safe_to_right_lane_borrow);
  // 确定自车起始位置的l
  double ego_state_l =
      (ego_frenet_boundary_.l_end + ego_frenet_boundary_.l_start) * 0.5;
  lane_borrow_pb_info->set_ego_l(ego_state_l);
  // 如果左右借道都不安全，那么目标l设为0，既没有目标车道，不借道
  if (!safe_to_left_lane_borrow && !safe_to_right_lane_borrow) {
    lane_borrow_decider_output_.target_l = 0;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    return false;
    // 如果左侧安全，并且右侧不安全，将左借道信息赋值给output
  } else if (safe_to_left_lane_borrow && !safe_to_right_lane_borrow) {
    lane_borrow_decider_output_.target_l = target_left_l;
    lane_borrow_decider_output_.left_bounds_l = left_left_bounds_l;
    lane_borrow_decider_output_.right_bounds_l = left_right_bounds_l;
    lane_borrow_decider_output_.borrow_direction = LEFT_BORROW;
    // 右同左
  } else if (!safe_to_left_lane_borrow && safe_to_right_lane_borrow) {
    lane_borrow_decider_output_.target_l = target_right_l;
    lane_borrow_decider_output_.left_bounds_l = right_left_bounds_l;
    lane_borrow_decider_output_.right_bounds_l = right_right_bounds_l;
    lane_borrow_decider_output_.borrow_direction = RIGHT_BORROW;
    // 如果都安全，且当前状态为no borrow，则判断target left和right 与ego state
    // 距离， 选择距离小的一侧进行借道
  } else {
    if (lane_borrow_decider_output_.borrow_direction == NO_BORROW) {
      if (abs(target_left_l - ego_state_l) <
          abs(target_right_l - ego_state_l)) {
        lane_borrow_decider_output_.target_l = target_left_l;
        lane_borrow_decider_output_.left_bounds_l = left_left_bounds_l;
        lane_borrow_decider_output_.right_bounds_l = left_right_bounds_l;
        lane_borrow_decider_output_.borrow_direction = LEFT_BORROW;
      } else {
        lane_borrow_decider_output_.target_l = target_right_l;
        lane_borrow_decider_output_.left_bounds_l = right_left_bounds_l;
        lane_borrow_decider_output_.right_bounds_l = right_right_bounds_l;
        lane_borrow_decider_output_.borrow_direction = RIGHT_BORROW;
      }
    }
  }
  return true;
}

bool LaneBorrowDecider::IsSafeForBackOriginLane() {
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  //  going to overtake static area [the area is updating ]
  // ？
  if (obs_end_s_ - ego_frenet_boundary_.s_end > kSafeBackDistance) {
    return false;
  }
  for (const auto& obstacle : obstacles) {
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
      // 左借道过滤掉那些在车辆左侧和右侧之外的障碍物。如果障碍物在车辆的左侧或者右侧之外，则跳过当前的循环迭代
      if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start &&
          frenet_obstacle_sl.l_start > left_width) {
        continue;
      }
      if (frenet_obstacle_sl.l_end < -right_width) {
        continue;
      }
      // 右借道过滤掉那些不在车辆左右边界范围内的障碍物
    } else {
      if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end &&
          frenet_obstacle_sl.l_end < -right_width) {
        continue;
      }
      if (frenet_obstacle_sl.l_start > left_width) {
        continue;
      }
    }

    // 距离过远大于阈值滤除
    if (frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
        kForwardOtherObsDistance) {
      continue;
    }

    // 距离大于安全距离并且obs车速比自车大，滤除
    const double obs_v = obstacle->obstacle()->velocity();
    if ((frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
         kSafeBackDistance) &&
        (obs_v > ego_speed_ + kObsSpeedBuffer)) {
      continue;
    }
    //?
    if (frenet_obstacle_sl.s_end > ego_frenet_boundary_.s_start) {
      return false;
    }

    if (ego_speed_ - obs_v > kObsSpeedBuffer) {
      continue;
    }
    // 快速靠近自车 有危险，false
    if (frenet_obstacle_sl.l_start > ego_frenet_boundary_.l_end ||
        frenet_obstacle_sl.l_end < ego_frenet_boundary_.l_start) {
      const double dist = std::max(kSafeBackDistance, obs_v * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
        return false;  // fast come near ego car
      }
    }
  }

  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    lane_borrow_decider_output_.right_bounds_l = -right_width;
  } else {
    lane_borrow_decider_output_.left_bounds_l = left_width;
  }

  lane_borrow_decider_output_.target_l = 0.0;
  return true;
}

// 根据边界判定路径是否安全
bool LaneBorrowDecider::IsSafeForPath(const double& left_bounds_l,
                                      const double& right_bounds_l) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (left_bounds_l - right_bounds_l <
      vehicle_param.width + kObsLatExpendBuffer) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = BOUNDS_TOO_NARROW;
    return false;
  }

  double left_l = left_bounds_l;
  double right_l = right_bounds_l;

  // 内收边界，不需要完全占据隔壁车道，故而需要内收
  if (left_borrow_) {
    left_l =
        std::min(left_l, right_l + vehicle_param.width + kObsLatExpendBuffer);
  } else {
    right_l =
        std::max(right_l, left_l - vehicle_param.width - kObsLatExpendBuffer);
  }

  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  //    too close to area but no borrow enough
  if (lane_borrow_status_ != LaneBorrowStatus::kLaneBorrowCrossing) {
    bool is_safe_for_turn = IsSafeForTurn();
    if (obs_start_s_ - ego_frenet_boundary_.s_end > 0 &&
        obs_start_s_ - ego_frenet_boundary_.s_end < kCheckTurningDistance &&
        is_safe_for_turn == false) {
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          STATIC_AREA_TOO_CLOSE;
      return false;
    }
  }
  // turn radius calculate

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    // obs的frenet坐标无效，滤除
    if (!obstacle->b_frenet_valid()) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      // 如果obs距离远，且车速大于阈值，滤除
      if (obstacle->obstacle()->velocity() > kObsFilterVel) {
        continue;
      }
      // for遍历obs，当出现frenet_obstacle_sl.s_start 大于 obs_end_s_
      // 时，跳出滤除
      if (frenet_obstacle_sl.s_start > obs_end_s_) {
        continue;
      }
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) {
          continue;
        }
        // 目标车压左侧bound，静态阻塞
        if (frenet_obstacle_sl.l_end + vehicle_param.width +
                    kLatPassableBuffer >
                left_bounds_l &&
            frenet_obstacle_sl.l_start - vehicle_param.width -
                    kLatPassableBuffer <
                obs_left_l_) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
        // 右借道压右边界 滤除 同左借道
      } else {
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        }

        if (frenet_obstacle_sl.l_end + vehicle_param.width +
                    kLatPassableBuffer >
                obs_right_l_ &&
            frenet_obstacle_sl.l_end - vehicle_param.width -
                    kLatPassableBuffer <
                right_bounds_l) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
      // obs位于自车后方，同上，
    } else if (frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) {
          continue;
        }

      } else {
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        }
      }
      // 后方距离自车太近
      if (lane_borrow_status_ != LaneBorrowStatus::kLaneBorrowCrossing) {
        double dist =
            std::max(kSafeBackDistance,
                     obstacle->obstacle()->velocity() * kObsSpeedRatio);
        if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              BACKWARD_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }

      // 该情况自车路径在纵向和obs有重合区间，左右借道同上
    } else {
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < right_l) {
          continue;
        } else {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              NEARBY_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }

      } else {
        if (frenet_obstacle_sl.l_start > left_l ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        } else {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              NEARBY_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
    }
  }
  return true;
}
// 转弯安全距离
// 主要计算转向半径，如果到bound距离大于转向半径，则return true
bool LaneBorrowDecider::IsSafeForTurn() {
  // radius
  if (obs_left_l_ <= obs_right_l_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double wheel_base = vehicle_param.wheel_base;
  const double front_angle =
      vehicle_param.max_front_wheel_angle;  // relative to ego_speed_?
  const double max_current_radius = wheel_base / std::tan(front_angle);
  // turn center
  const Pose2D rear_center_pose =
      session_->environmental_model().get_ego_state_manager()->ego_pose();
  const Point2D rear_center_cart =
      session_->environmental_model().get_ego_state_manager()->ego_carte();
  const double heading_angle =
      session_->environmental_model().get_ego_state_manager()->ego_pose().theta;
  const Point2D turning_center =
      CalTurningCenter(rear_center_cart, heading_angle, max_current_radius);
  // sl static area overlap circle
  // sl to SLToXY
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  const auto& current_frenet_coord = current_reference_path->get_frenet_coord();

  Point2D back_right_corner, back_left_corner;
  current_frenet_coord->SLToXY(obs_start_s_, obs_right_l_, &back_right_corner.x,
                               &back_right_corner.y);
  current_frenet_coord->SLToXY(obs_start_s_, obs_left_l_, &back_left_corner.x,
                               &back_left_corner.y);
  const double corner_angle = std::tan((vehicle_param.max_width * 0.5) /
                                       vehicle_param.front_edge_to_rear_axle);
  const double corner_length =
      std::hypot(vehicle_param.max_width * 0.5,
                 vehicle_param.front_edge_to_rear_axle);  // 前保转弯半径
  const double corner_radius_square =
      corner_length * corner_length + max_current_radius * max_current_radius -
      2 * corner_length * max_current_radius * std::cos(corner_angle + M_PI_2);
  const double corner_radius = std::sqrt(corner_radius_square);

  const double distance_to_left =
      std::hypot(turning_center.x - back_left_corner.x,
                 turning_center.y - back_left_corner.y);
  const double distance_to_right =
      std::hypot(turning_center.x - back_right_corner.x,
                 turning_center.y - back_right_corner.y);
  // log
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->mutable_borrow_turn_circle()->mutable_center()->set_x(
      turning_center.x);
  lane_borrow_pb_info->mutable_borrow_turn_circle()->mutable_center()->set_y(
      turning_center.y);
  lane_borrow_pb_info->mutable_borrow_turn_circle()->set_corner_radius(
      corner_radius);
  if (distance_to_left > corner_radius && distance_to_right > corner_radius) {
    return true;
  } else {
    return false;
  }
}

// 转向中心点
const Point2D LaneBorrowDecider::CalTurningCenter(const Point2D& ego_pos,
                                                  const double& theta,
                                                  const double& radius) const {
  Eigen::Vector2d ego_heading_vec(std::cos(theta), std::sin(theta));
  Eigen::Vector2d rear_pos(ego_pos.x, ego_pos.y);
  Eigen::Vector2d ego_n_vec(-ego_heading_vec.y(), ego_heading_vec.x());

  if (right_borrow_ == true) {
    ego_n_vec *= -1.0;
  }
  Eigen::Vector2d center = rear_pos + ego_n_vec * radius;
  return Point2D(center.x(), center.y());
}

bool LaneBorrowDecider::ChecekIfLaneBorrowToLaneBorrowCrossing() {
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();

  const auto& current_frenet_coord = current_reference_path->get_frenet_coord();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double heading_angle =
      session_->environmental_model().get_ego_state_manager()->ego_pose().theta;

  double ego_x =
      session_->environmental_model().get_ego_state_manager()->ego_pose().x;
  double ego_y =
      session_->environmental_model().get_ego_state_manager()->ego_pose().y;

  // 取直行时笛卡尔系角点信息

  // double radian_heading_angle = heading_angle * M_PI / 180.0;
  Point2D corner_front_left_xy(vehicle_param.front_edge_to_rear_axle,
                               vehicle_param.width * 0.5);
  Point2D corner_front_right_xy(vehicle_param.front_edge_to_rear_axle,
                                -vehicle_param.width * 0.5);
  Point2D corner_rear_left_xy(-vehicle_param.rear_edge_to_rear_axle,
                              vehicle_param.width * 0.5);
  Point2D corner_rear_right_xy(-vehicle_param.rear_edge_to_rear_axle,
                               -vehicle_param.width * 0.5);

  SLPoint corner_front_left, corner_rear_left, corner_front_right,
      corner_rear_right;

  if (left_borrow_) {
    double original_x = corner_front_left_xy.x;  // 平移
    double original_y = corner_front_left_xy.y;  // 平移
    corner_front_left_xy.x = original_x * cos(heading_angle) -
                             original_y * sin(heading_angle) +
                             ego_x;  // x旋转矩阵
    corner_front_left_xy.y = original_x * sin(heading_angle) +
                             original_y * cos(heading_angle) + ego_y;
    // corner_front_left_xy.x += ego_x;//逆平移
    // corner_front_left_xy.y += ego_y;

    original_x = corner_rear_left_xy.x;  // 平移
    original_y = corner_rear_left_xy.y;  // 平移
    corner_rear_left_xy.x = original_x * cos(heading_angle) -
                            original_y * sin(heading_angle) +
                            ego_x;  // 应用旋转矩阵
    corner_rear_left_xy.y = original_x * sin(heading_angle) +
                            original_y * cos(heading_angle) + ego_y;
    // corner_rear_left_xy.x += ego_x;    // 逆平移
    // corner_rear_left_xy.y += ego_y;

    // 转回sl坐标系，和车道线比较
    current_frenet_coord->XYToSL(corner_front_left_xy.x, corner_front_left_xy.y,
                                 &corner_front_left.s, &corner_front_left.l);
    current_frenet_coord->XYToSL(corner_rear_left_xy.x, corner_rear_left_xy.y,
                                 &corner_rear_left.s, &corner_rear_left.l);

    // 判断车道线和自车左侧角点的位置
    //  double left_front_check =
    //  right_lane_ptr_->distance_to_line(corner_front_right.s,
    //  corner_front_right.l, LineDirection::LEFT); double left_rear_check =
    //  right_lane_ptr_->distance_to_line(corner_rear_right.s,
    //  corner_rear_right.l, LineDirection::LEFT);
    const double current_front_left_lane_l =
        current_lane_ptr_->width_by_s(corner_front_left.s) * 0.5;
    const double current_rear_left_lane_l =
        current_lane_ptr_->width_by_s(corner_rear_left.s) * 0.5;
    if (corner_front_left.l > current_front_left_lane_l &&
        corner_rear_left.l > current_rear_left_lane_l) {
      return true;
    }
    return false;

  } else if (right_borrow_) {
    double original_x = corner_front_right_xy.x;  // 平移
    double original_y = corner_front_right_xy.y;  // 平移
    corner_front_right_xy.x = original_x * cos(heading_angle) -
                              original_y * sin(heading_angle) +
                              ego_x;  // 应用旋转矩阵
    corner_front_right_xy.y = original_x * sin(heading_angle) +
                              original_y * cos(heading_angle) + ego_y;
    // corner_front_right_xy.x += ego_x;    // 逆平移
    // corner_front_right_xy.y += ego_y;
    original_x = corner_rear_right_xy.x;  // 平移
    original_y = corner_rear_right_xy.y;  // 平移
    corner_rear_right_xy.x = original_x * cos(heading_angle) -
                             original_y * sin(heading_angle) +
                             ego_x;  // 应用旋转矩阵
    corner_rear_right_xy.y = original_x * sin(heading_angle) +
                             original_y * cos(heading_angle) + ego_y;
    // corner_rear_right_xy.x += ego_x;    // 逆平移
    // corner_rear_right_xy.y += ego_y;

    current_frenet_coord->XYToSL(corner_front_right_xy.x,
                                 corner_front_right_xy.y, &corner_front_right.s,
                                 &corner_front_right.l);
    current_frenet_coord->XYToSL(corner_rear_right_xy.x, corner_rear_right_xy.y,
                                 &corner_rear_right.s, &corner_rear_right.l);
    // double right_front_check =
    // right_lane_ptr_->distance_to_line(corner_front_right.s,
    // corner_front_right.l, LineDirection::RIGHT); double right_rear_check =
    // right_lane_ptr_->distance_to_line(corner_rear_right.s,
    // corner_rear_right.l, LineDirection::RIGHT);

    const double current_front_right_lane_l =
        current_lane_ptr_->width_by_s(corner_front_right.s) * 0.5;
    const double current_rear_right_lane_l =
        current_lane_ptr_->width_by_s(corner_rear_right.s) * 0.5;

    if (corner_front_right.l < current_front_right_lane_l &&
        corner_rear_right.l < current_rear_right_lane_l) {
      return true;
    }
    return false;
  }
  return false;
}

bool LaneBorrowDecider::CheckIfkLaneBorrowCrossingToNoBorrow() {
  if (!CheckLaneBorrowCrossingCondition()) {
    // if (lane_borrow_decider_output_.lane_borrow_failed_reason !=
    // BACKWARD_OBSTACLE_TOO_CLOSE){
    //     return true;
    //   }
    return true;
  }
  return false;
}

bool LaneBorrowDecider::CheckIfLaneBorrowCrossingToBackDriving() {
  if (!IsSafeForBackOriginLane()) {
    return false;
  }
  return true;
}

// double VirtualLane::distance_to_line(double s, double l, LineDirection
// direction) {
//   //  // 检查 session_ 是否为空
// if (!session_) {
//       std::cerr << "Error: session_ is null." << std::endl;
//   }

//   // 获取 environmental_model
// auto& env_model = session_->environmental_model();

//   // 获取 reference_path_manager 并检查其有效性
// auto ref_path_mgr = env_model.get_reference_path_manager();
// if (!ref_path_mgr) {
//       std::cerr << "Error: Reference path manager is null." << std::endl;
// }

//   // 获取当前车道的参考路径
// auto current_reference_path =
// ref_path_mgr->get_reference_path_by_current_lane();
//   if (!current_reference_path) {
//       std::cerr << "Error: Current reference path is null." << std::endl;
// }

//   // 获取 frenet 坐标系统
// const auto& current_frenet_coord =
// current_reference_path->get_frenet_coord();
//   if (!current_frenet_coord) {
//       std::cerr << "Error: Frenet coordinate system is null." << std::endl;
// }
//   auto current_reference_path = session_->environmental_model()
//                                     .get_reference_path_manager()
//                                     ->get_reference_path_by_current_lane();
//   const auto& current_frenet_coord =
//   current_reference_path->get_frenet_coord(); assert(direction == LEFT ||
//   direction == RIGHT);

//   SLPoint frenet_point;
//   Point2D cart_point;
//   current_frenet_coord->SLToXY(frenet_point.s, frenet_point.l, &cart_point.x,
//   &cart_point.y);

//   double distance_ref, distance_line, distance_line_ref;
//   if (has_lines(direction)) {
//     std::vector<double> coefficient;
//     if (direction == LEFT) {
//       for (auto value:left_lane_boundary_.poly_coefficient) {
//         coefficient.emplace_back(value);
//       }
//     } else {
//       for (auto value : right_lane_boundary_.poly_coefficient) {
//         coefficient.emplace_back(value);
//       }
//     }
//     std::reverse(coefficient.begin(),coefficient.end());

//     distance_ref = l;
//     distance_line_ref = -get_dist(cart_point.x, cart_point.y, coefficient);
//     distance_line = distance_ref - distance_line_ref;
//   } else {
//     double distance_tr_ref = 0;
//     if (direction == LEFT) {
//       distance_line_ref = distance_tr_ref + 0.5 * DEFAULT_LANE_WIDTH;
//     } else {
//       distance_line_ref = distance_tr_ref - 0.5 * DEFAULT_LANE_WIDTH;
//     }

//     distance_ref = l;
//     distance_line = distance_ref - distance_line_ref;
//   }
//   return distance_line_ref;
// }

// 检测从跨线借道返回不借道状态的条件
bool LaneBorrowDecider::CheckLaneBorrowCrossingCondition() {
  UpdateJunctionInfo();

  // 判断是否在交叉路口区域，如果在改原因为交叉路口并返回false
  // 通过一系列的条件检查和函数调用，决定车辆是否可以借用车道。
  // 如果任何一个条件不满足或者相关函数返回 false，则最终返回 false，
  // 表示不能借用车道；否则，继续执行后续逻辑
  if ((forward_solid_start_dis_ < kMinDisToSolidLane &&
       forward_solid_start_dis_ > 0) ||
      (distance_to_cross_walk_ < kMinDisToCrossWalk &&
       distance_to_cross_walk_ > 0.0) ||
      (distance_to_stop_line_ < kMinDisToStopLine &&
       distance_to_stop_line_ > 0.0) ||
      (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
       dis_to_traffic_lights_ > 0.0)) {
    LOG_DEBUG("Ego car is near junction");
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  // 更新车道借用的方向，根据边界类型和车道线虚实类型判断左右借道的可行性。如果该函数返回
  // false，表示无法确定车道借用方向，因此直接返回 false
  if (!UpdateLaneBorrowDirection()) {
    return false;
  }

  // shaixuan可能阻碍车道借用的静态障碍物。如果该函数返回
  // false，表示无法找到合适的障碍物，因此直接返回 false
  if (!SelectStaticBlockingObstcales()) {
    return false;
  }

  // 进行障碍物决策返回false决策失败
  if (!ObstacleDecision()) {
    return false;
  }

  // 判断观测时间如果小于最小观测帧数，则false原因为观测时间不足
  observe_frame_num_++;
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  // 函数返回false，则返回false表示不允许借用
  if (!IsSafeForLaneBorrow()) {
    // if (lane_borrow_decider_output_.lane_borrow_failed_reason ==
    // BACKWARD_OBSTACLE_TOO_CLOSE){
    // }
    return false;
  }

  // 如果上述都不满足，则返回true，表示可以借用
  return true;
}

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginToLaneBorrowCrossing() {
  if (ChecekIfLaneBorrowToLaneBorrowCrossing()) {
    if (CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving()) {
      return true;
    }
  }
  return false;
}

void LaneBorrowDecider::LogDebugInfo() {
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_lane_borrow_failed_reason(
      static_cast<int>(lane_borrow_decider_output_.lane_borrow_failed_reason));
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

  lane_borrow_pb_info->set_lane_borrow_decider_status(
      static_cast<int>(lane_borrow_status_));

  lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Clear();
  for (auto static_obs_id : static_blocked_obj_id_vec_) {
    lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Add(
        static_obs_id);
  }

  lane_borrow_pb_info->set_intersection_state(intersection_state_);
}

}  // namespace planning