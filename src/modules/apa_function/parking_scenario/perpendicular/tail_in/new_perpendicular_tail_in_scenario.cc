#include <cstddef>
#include <vector>

#include "apa_data.h"
#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "math_lib.h"
#include "perpendicular_tail_in_scenario.h"

namespace planning {
namespace apa_planner {

void PerpendicularTailInScenario::PlanCore(ApaSlot& apa_slot) {}

void PerpendicularTailInScenario::PlanCoreSearching(ApaSlot& apa_slot) {}

void PerpendicularTailInScenario::PlanCoreParking(ApaSlot& apa_slot) {}

void PerpendicularTailInScenario::UpdateEgoSlotInfo(ApaSlot& apa_slot) {
  // 建立车位坐标系 根据23角点或者限位器角点确定规划终点位姿
  const auto measures_data = apa_world_ptr_->GetMeasureDataManagerPtr();
  const Eigen::Vector2d pM01 =
      0.5 * (apa_slot.processed_corner_coord_global.pt_0 +
             apa_slot.processed_corner_coord_global.pt_1);
  const Eigen::Vector2d pM23 =
      0.5 * (apa_slot.processed_corner_coord_global.pt_2 +
             apa_slot.processed_corner_coord_global.pt_3);

  apa_slot.origin_pose_global.heading_vec = (pM01 - pM23).normalized();
  apa_slot.origin_pose_global.heading =
      std::atan2(apa_slot.origin_pose_global.heading_vec.y(),
                 apa_slot.origin_pose_global.heading_vec.x());

  apa_slot.slot_length = (pM01 - pM23).norm();
  apa_slot.slot_width = (apa_slot.processed_corner_coord_global.pt_0 -
                         apa_slot.processed_corner_coord_global.pt_1)
                            .norm();

  apa_slot.origin_pose_global.pos =
      pM01 - apa_slot.slot_length * apa_slot.origin_pose_global.heading_vec;

  apa_slot.origin_pose_local.heading = 0.0;
  apa_slot.origin_pose_local.heading_vec << 1.0, 0.0;
  apa_slot.origin_pose_local.pos << 0.0, 0.0;

  apa_slot.g2l_tf.Init(apa_slot.origin_pose_global.pos,
                       apa_slot.origin_pose_global.heading);
  apa_slot.l2g_tf.Init(apa_slot.origin_pose_global.pos,
                       apa_slot.origin_pose_global.heading);

  // 车位坐标转换到 局部坐标系
  apa_slot.origin_corner_coord_local.pt_0 =
      apa_slot.g2l_tf.GetPos(apa_slot.origin_corner_coord_global.pt_0);
  apa_slot.origin_corner_coord_local.pt_1 =
      apa_slot.g2l_tf.GetPos(apa_slot.origin_corner_coord_global.pt_1);
  apa_slot.origin_corner_coord_local.pt_2 =
      apa_slot.g2l_tf.GetPos(apa_slot.origin_corner_coord_global.pt_2);
  apa_slot.origin_corner_coord_local.pt_3 =
      apa_slot.g2l_tf.GetPos(apa_slot.origin_corner_coord_global.pt_3);
  apa_slot.origin_corner_coord_local.CalCenter();

  apa_slot.processed_corner_coord_local.pt_0 =
      apa_slot.g2l_tf.GetPos(apa_slot.processed_corner_coord_global.pt_0);
  apa_slot.processed_corner_coord_local.pt_1 =
      apa_slot.g2l_tf.GetPos(apa_slot.processed_corner_coord_global.pt_1);
  apa_slot.processed_corner_coord_local.pt_2 =
      apa_slot.g2l_tf.GetPos(apa_slot.processed_corner_coord_global.pt_2);
  apa_slot.processed_corner_coord_local.pt_3 =
      apa_slot.g2l_tf.GetPos(apa_slot.processed_corner_coord_global.pt_3);
  apa_slot.processed_corner_coord_local.CalCenter();

  if (apa_slot.slot_type == SlotType::PERPENDICULAR) {
    apa_slot.angle = 90.0;
    apa_slot.sin_angle = 1.0;
  } else if (apa_slot.slot_type == SlotType::SLANT) {
    const Eigen::Vector2d pt_01_vec =
        apa_slot.processed_corner_coord_local.pt_1 -
        apa_slot.processed_corner_coord_local.pt_0;
    double angle = std::fabs(geometry_lib::GetAngleFromTwoVec(
                       Eigen::Vector2d(1.0, 0.0), pt_01_vec)) *
                   kRad2Deg;
    if (angle > 90.0) {
      angle = 180.0 - angle;
    }
    apa_slot.angle = mathlib::DoubleConstrain(angle, 10.0, 80.0);
    apa_slot.sin_angle = std::sin(apa_slot.angle * kDeg2Rad);
  }

  auto& ego_car_info_slot = frame_.ego_car_info_slot;
  if (frame_.is_replan_first) {
    // 计算车位方向
    const Eigen::Vector2d ego_to_slot_center_vec =
        apa_slot.origin_corner_coord_global.pt_center - measures_data->GetPos();
    const double cross_ego_to_slot_center = geometry_lib::GetCrossFromTwoVec2d(
        measures_data->GetHeadingVec(), ego_to_slot_center_vec);
    const double cross_ego_to_slot_heading = geometry_lib::GetCrossFromTwoVec2d(
        measures_data->GetHeadingVec(),
        apa_slot.origin_pose_global.heading_vec);
    // judge slot side via slot center and heading
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      ego_car_info_slot.slot_side = geometry_lib::SLOT_SIDE_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      ego_car_info_slot.slot_side = geometry_lib::SLOT_SIDE_LEFT;
    } else {
      ego_car_info_slot.slot_side = geometry_lib::SLOT_SIDE_INVALID;
    }
  }

  ego_car_info_slot.cur_pose = apa_slot.g2l_tf.GetPose(geometry_lib::PathPoint(
      measures_data->GetPos(), measures_data->GetHeading()));

  // 初步根据车位或限位器计算终点位置
  if (!apa_slot.limiter.valid) {
    // 根据后面两个角点来停车
    ego_car_info_slot.target_pose.pos.x() =
        (apa_slot.processed_corner_coord_local.pt_2 +
         apa_slot.processed_corner_coord_local.pt_3)
                .x() *
            0.5 +
        apa_param.GetParam().terminal_target_x;
  } else {
    // 根据限位器来停车
    const Eigen::Vector2d limit_0 =
        apa_slot.g2l_tf.GetPos(apa_slot.limiter.start_pt);
    const Eigen::Vector2d limit_1 =
        apa_slot.g2l_tf.GetPos(apa_slot.limiter.start_pt);
    ego_car_info_slot.target_pose.pos.x() =
        (limit_0 + limit_1).x() * 0.5 + apa_param.GetParam().limiter_move_dist;
  }
  ego_car_info_slot.target_pose.pos.y() =
      apa_param.GetParam().terminal_target_y;

  if (measures_data->GetStaticFlag() && !measures_data->GetBrakeFlag() &&
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachineT::ACTIVE_IN_CAR_REAR) {
    // 重规划成功会清0
    if (frame_.plan_stm.planning_status == PARKING_RUNNING) {
      frame_.stuck_uss_time += apa_param.GetParam().plan_time;
    } else {
      frame_.stuck_uss_time = 0.0;
    }
    // 不管是否重规划成功一直累加
    if (frame_.plan_stm.planning_status == PARKING_RUNNING ||
        frame_.plan_stm.planning_status == PARKING_PLANNING) {
      frame_.stuck_time += apa_param.GetParam().plan_time;
    } else {
      frame_.stuck_time = 0.0;
    }
  }
}

void PerpendicularTailInScenario::GenTLane(ApaSlot& apa_slot) {
  // construct tlane pq
  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      left_pq_for_y(geometry_lib::Compare(3));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      left_pq_for_x(geometry_lib::Compare(0));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      right_pq_for_y(geometry_lib::Compare(2));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      right_pq_for_x(geometry_lib::Compare(0));

  const auto& param = apa_param.GetParam();

  const double mir_width =
      (param.max_car_width - param.car_width) * 0.5 - 0.0168;

  auto& ego_car_info_slot = frame_.ego_car_info_slot;
  const double mir_x = ego_car_info_slot.target_pose.pos.x() +
                       param.lon_dist_mirror_to_rear_axle - 0.368;

  // 获取融合障碍物 并转化到车位坐标系下
  if (apa_world_ptr_->GetApaDataPtr()->apa_obs_map.count(
          ObstacleType::FUSION) != 0) {
    std::vector<Eigen::Vector2d> fusion_obs_slot_vec;
    Eigen::Vector2d obs_slot;
    for (const auto& obs :
         apa_world_ptr_->GetApaDataPtr()->apa_obs_map[ObstacleType::FUSION]) {
      obs_slot = apa_slot.g2l_tf.GetPos(obs);
      fusion_obs_slot_vec.emplace_back(obs_slot);
    }
    ego_car_info_slot.obs_map[ObstacleType::FUSION] = fusion_obs_slot_vec;
  }

  // 获取接地线障碍物 并转化到车位坐标系下
  if (apa_world_ptr_->GetApaDataPtr()->apa_obs_map.count(
          ObstacleType::GROUND_LINE) != 0) {
    std::vector<Eigen::Vector2d> gl_obs_slot_vec;
    Eigen::Vector2d obs_slot;
    for (const auto& obs : apa_world_ptr_->GetApaDataPtr()
                               ->apa_obs_map[ObstacleType::GROUND_LINE]) {
      obs_slot = apa_slot.g2l_tf.GetPos(obs);
      gl_obs_slot_vec.emplace_back(obs_slot);
    }
    ego_car_info_slot.obs_map[ObstacleType::GROUND_LINE] = gl_obs_slot_vec;
  }

  // 获取uss点云障碍物 并转化到车位坐标系下
  if (apa_world_ptr_->GetApaDataPtr()->apa_obs_map.count(ObstacleType::USS) !=
      0) {
    std::vector<Eigen::Vector2d> uss_obs_slot_vec;
    Eigen::Vector2d obs_slot;
    for (const auto& obs :
         apa_world_ptr_->GetApaDataPtr()->apa_obs_map[ObstacleType::USS]) {
      obs_slot = apa_slot.g2l_tf.GetPos(obs);
      uss_obs_slot_vec.emplace_back(obs_slot);
    }
    ego_car_info_slot.obs_map[ObstacleType::USS] = uss_obs_slot_vec;
  }
  size_t obs_count_in_slot = 0;
  for (auto pair : ego_car_info_slot.obs_map) {
    for (Eigen::Vector2d obs_pt_slot : pair.second) {
      SlotObsType obs_slot_type = CalSlotObsType(obs_pt_slot);
      if (obs_slot_type == SlotObsType::IN_OBS) {
        obs_count_in_slot++;
      }
      if (obs_slot_type != SlotObsType::INSIDE_OBS &&
          obs_slot_type != SlotObsType::OUTSIDE_OBS) {
        continue;
      }
      // the obs lower mir can relax lat requirements
      if (obs_pt_slot.x() < mir_x) {
        if (obs_pt_slot.y() > 1e-6) {
          obs_pt_slot.y() += mir_width;
        } else {
          obs_pt_slot.y() -= mir_width;
        }
      }
      // the obs far from slot can relax lon equirements
      if (std::fabs(obs_pt_slot.y()) >
          ego_car_info_slot.slot.slot_width * 0.5 + 0.468) {
        obs_pt_slot.x() -= 0.268;
      }

      if (obs_pt_slot.y() > 1e-6) {
        left_pq_for_y.emplace(std::move(obs_pt_slot));
        left_pq_for_x.emplace(std::move(obs_pt_slot));
      } else {
        right_pq_for_y.emplace(std::move(obs_pt_slot));
        right_pq_for_x.emplace(std::move(obs_pt_slot));
      }
    }
  }

  if (false) {
    // 如果是巡库状态下 判断库内是否有障碍物 使其不释放
    if (obs_count_in_slot > 3) {
      apa_slot.is_release = false;
      apa_slot.slot_occupied_reason = SlotOccupiedReason::SLOT_IN_OBS;
      return;
    }
  }

  apa_param.SetPram().actual_mono_plan_enable = param.mono_plan_enable;
  // 如果保守的话  两侧全空才开启一把进
  const bool left_empty = left_pq_for_x.empty();
  const bool right_empty = right_pq_for_x.empty();
  if (param.conservative_mono_enable && (!left_empty || !right_empty)) {
    apa_param.SetPram().actual_mono_plan_enable = false;
  }

  // 加入左右侧的虚拟障碍物
  const Eigen::Vector2d pt_01_unit_vec =
      (apa_slot.origin_corner_coord_local.pt_1 -
       apa_slot.origin_corner_coord_local.pt_0)
          .normalized();

  const Eigen::Vector2d pt_01_mid = (apa_slot.origin_corner_coord_local.pt_1 +
                                     apa_slot.origin_corner_coord_local.pt_0) *
                                    0.5;

  double half_origin_slot_width =
      0.5 * (apa_slot.origin_corner_coord_local.pt_1 -
             apa_slot.origin_corner_coord_local.pt_0)
                .norm();
  half_origin_slot_width =
      std::max(half_origin_slot_width, param.max_car_width * 0.5 + 0.168);

  const Eigen::Vector2d virtual_left_obs =
      pt_01_mid -
      param.virtual_obs_x_pos * apa_slot.origin_pose_local.heading_vec +
      (half_origin_slot_width + param.virtual_obs_y_pos) * pt_01_unit_vec;

  const Eigen::Vector2d virtual_right_obs =
      pt_01_mid -
      apa_param.GetParam().virtual_obs_x_pos *
          apa_slot.origin_pose_local.heading_vec -
      (half_origin_slot_width + param.virtual_obs_y_pos) * pt_01_unit_vec;

  left_pq_for_y.emplace(virtual_left_obs);
  left_pq_for_x.emplace(virtual_left_obs);
  right_pq_for_y.emplace(virtual_right_obs);
  right_pq_for_x.emplace(virtual_right_obs);

  // 找到左侧和右侧障碍物极限位置
  const Eigen::Vector2d left_obs(left_pq_for_x.top().x(),
                                 left_pq_for_y.top().y());
  const Eigen::Vector2d right_obs(right_pq_for_x.top().x(),
                                  right_pq_for_y.top().y());

  const double car_width_fold_mirror = apa_param.GetParam().car_width + 0.06;
  const double car_width_no_fold_mirror = apa_param.GetParam().max_car_width;
  double car_width = car_width_no_fold_mirror;
  // 如果后期折叠了后视镜
  if (false) {
    car_width = car_width_fold_mirror;
  }

  // 找到车子如果居中停的时候最左边位置和最右边位置
  Eigen::Vector2d left_car(2.168, car_width * 0.5);
  Eigen::Vector2d right_car(2.168, -car_width * 0.5);

  // 计算左右侧障碍物离自车的距离
  const double left_dis_obs_car = (left_obs - left_car).y();
  const double right_dis_obs_car = (right_car - right_obs).y();

  ILOG_INFO << "left_dis_obs_car = " << left_dis_obs_car
            << "  right_dis_obs_car = " << right_dis_obs_car;

  // 设置障碍物离自车的合适安全距离 以及能使规划成功的最小安全距离
  const double suitable_safe_threshold =
      apa_param.GetParam().car_lat_inflation_normal +
      apa_param.GetParam().safe_threshold;

  const double min_safe_threshold =
      apa_param.GetParam().car_lat_inflation_normal + 0.0168;

  // 如果是寻库阶段 需要判断车位是否能释放
  if (false) {
    if (left_dis_obs_car + right_dis_obs_car <
        2.0 * suitable_safe_threshold + 0.01) {
      apa_slot.is_release = false;
      apa_slot.slot_occupied_reason = SlotOccupiedReason::TWO_SIDE_OBS;
    } else {
    }
  }

  const bool left_obs_meet_safe_require =
      left_dis_obs_car > suitable_safe_threshold ? true : false;

  const bool right_obs_meet_safe_require =
      right_dis_obs_car > suitable_safe_threshold ? true : false;

  // 一旦重规划失败 用上次重规划的车位移动距离
  ego_car_info_slot.last_move_slot_dist = ego_car_info_slot.move_slot_dist;
  // 只有在重规划的时候才需要根据障碍物来改变是否移动车位或者移动车位的距离
  if (frame_.replan_flag) {
    if (!left_obs_meet_safe_require && right_obs_meet_safe_require) {
      // left side is dangerous, should move toward right
      ego_car_info_slot.move_slot_dist =
          -1.0 * std::min(right_dis_obs_car - min_safe_threshold,
                          suitable_safe_threshold - left_dis_obs_car);
    } else if (left_obs_meet_safe_require && !right_obs_meet_safe_require) {
      // right side is dangerous, should move toward left
      ego_car_info_slot.move_slot_dist =
          std::min(left_dis_obs_car - min_safe_threshold,
                   suitable_safe_threshold - right_dis_obs_car);
    } else if (left_obs_meet_safe_require && right_obs_meet_safe_require) {
      ego_car_info_slot.move_slot_dist = 0.0;
    } else if (!left_obs_meet_safe_require && !right_obs_meet_safe_require) {
      // 往两侧障碍物正中停
      const double mid_dist = (left_dis_obs_car + right_dis_obs_car) * 0.5;
      if (mid_dist < left_dis_obs_car) {
        // 往右移
        ego_car_info_slot.move_slot_dist = mid_dist - left_dis_obs_car;
      } else if (mid_dist < right_dis_obs_car) {
        // 往左移
        ego_car_info_slot.move_slot_dist = right_dis_obs_car - mid_dist;
      }
    }

    // 计算最大移动车位距离 因为不仅要考虑安全
    // 也要考虑到底自车车体能侵入旁边空间多少
    // 车体能压线的距离 正数表示不能越过线 负数表示可以越过
    const double max_move_slot_dist =
        apa_slot.slot_width * 0.5 - car_width_fold_mirror * 0.5 -
        apa_param.GetParam().car2line_dist_threshold;
    if (ego_car_info_slot.move_slot_dist > 0.0) {
      ego_car_info_slot.move_slot_dist =
          std::min(ego_car_info_slot.move_slot_dist, max_move_slot_dist);
    } else {
      ego_car_info_slot.move_slot_dist =
          std::max(ego_car_info_slot.move_slot_dist, -max_move_slot_dist);
    }
  }
  JSON_DEBUG_VALUE("move_slot_dist", ego_car_info_slot.move_slot_dist)

  // 根据车位移动距离更新终点位置
  ego_car_info_slot.target_pose.pos +=
      ego_car_info_slot.move_slot_dist * pt_01_unit_vec;

  ego_car_info_slot.terminal_err.Set(
      ego_car_info_slot.cur_pose.pos - ego_car_info_slot.target_pose.pos,
      geometry_lib::NormalizeAngle(ego_car_info_slot.cur_pose.heading -
                                   ego_car_info_slot.target_pose.heading));

  if (std::fabs(ego_car_info_slot.cur_pose.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_car_info_slot.cur_pose.heading) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_car_info_slot.target_pose.pos.x(),
        apa_slot.slot_length + apa_param.GetParam().rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    ego_car_info_slot.slot_occupied_ratio = pnc::mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_car_info_slot.cur_pose.pos.x());
  } else {
    ego_car_info_slot.slot_occupied_ratio = 0.0;
  }

  // 根据自车位置制定通道宽
  double channel_width = apa_world_ptr_->GetCollisionDetectorPtr()->GetCarMaxX(
                             ego_car_info_slot.cur_pose) +
                         3.168 -
                         std::max(apa_slot.origin_corner_coord_local.pt_0.x(),
                                  apa_slot.origin_corner_coord_local.pt_1.x());

  channel_width = std::max(channel_width, apa_param.GetParam().channel_width);
  if (ego_car_info_slot.slot_occupied_ratio > 0.768 &&
      channel_width > apa_param.GetParam().min_channel_width) {
    channel_width = apa_param.GetParam().min_channel_width;
  }

  // 根据实际障碍物计算一个宽泛的障碍物tlane 为了后续产生虚拟障碍物
  TLane& obs_tlane = apa_slot.obs_tlane;
  obs_tlane.B = virtual_left_obs;
  obs_tlane.E = virtual_right_obs;

  const double area_length = 12.0 / apa_slot.sin_angle;
  const double area_width = channel_width + param.virtual_obs_x_pos;

  obs_tlane.A = obs_tlane.B + pt_01_unit_vec * area_length;
  obs_tlane.H =
      obs_tlane.A + apa_slot.origin_pose_local.heading_vec * area_width;

  const double origin_slot_length = (apa_slot.origin_corner_coord_local.pt_0 -
                                     apa_slot.origin_corner_coord_local.pt_2)
                                        .norm();

  obs_tlane.C =
      obs_tlane.B - apa_slot.origin_pose_local.heading_vec * origin_slot_length;

  obs_tlane.D =
      obs_tlane.E - apa_slot.origin_pose_local.heading_vec * origin_slot_length;

  obs_tlane.F = obs_tlane.E - pt_01_unit_vec * area_length;
  obs_tlane.G =
      obs_tlane.F + apa_slot.origin_pose_local.heading_vec * area_width;

  // 计算内侧安全圆切点
  if (ego_car_info_slot.slot_side == geometry_lib::SLOT_SIDE_RIGHT) {
    ego_car_info_slot.pt_inside = right_obs;
    ego_car_info_slot.pt_inside.x() =
        mathlib::Constrain(ego_car_info_slot.pt_inside.x(),
                           apa_slot.origin_corner_coord_local.pt_0.x() - 2.168,
                           apa_slot.origin_corner_coord_local.pt_0.x() + 2.68);

    ego_car_info_slot.pt_inside.y() = mathlib::Constrain(
        ego_car_info_slot.pt_inside.y(), -0.5 * apa_slot.slot_width - 0.128,
        -0.5 * apa_slot.slot_width + 0.068);
  } else {
    ego_car_info_slot.pt_inside = left_obs;
    ego_car_info_slot.pt_inside.x() =
        mathlib::Constrain(ego_car_info_slot.pt_inside.x(),
                           apa_slot.origin_corner_coord_local.pt_1.x() - 2.168,
                           apa_slot.origin_corner_coord_local.pt_1.x() + 2.68);

    ego_car_info_slot.pt_inside.y() = mathlib::Constrain(
        ego_car_info_slot.pt_inside.y(), 0.5 * apa_slot.slot_width - 0.068,
        0.5 * apa_slot.slot_width + 0.128);
  }
}

void PerpendicularTailInScenario::GenObstacles(ApaSlot& apa_slot) {
  apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();
  auto& ego_car_info_slot = frame_.ego_car_info_slot;
  geometry_lib::LineSegment tlane_line;
  std::vector<geometry_lib::LineSegment> tlane_line_vec;
  tlane_line.SetPoints(apa_slot.obs_tlane.A, apa_slot.obs_tlane.B);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(apa_slot.obs_tlane.B, apa_slot.obs_tlane.C);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(apa_slot.obs_tlane.C, apa_slot.obs_tlane.D);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(apa_slot.obs_tlane.D, apa_slot.obs_tlane.E);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(apa_slot.obs_tlane.E, apa_slot.obs_tlane.F);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(apa_slot.obs_tlane.F, apa_slot.obs_tlane.G);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(apa_slot.obs_tlane.G, apa_slot.obs_tlane.H);
  tlane_line_vec.emplace_back(tlane_line);

  // tmp method, should modify
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(188);
  std::vector<Eigen::Vector2d> point_set;
  for (const auto& line : tlane_line_vec) {
    geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                          apa_param.GetParam().obstacle_ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
}

const PerpendicularTailInScenario::SlotObsType
PerpendicularTailInScenario::CalSlotObsType(const Eigen::Vector2d& obs_slot) {
  EgoSlotInfo ego_slot_info = frame_.ego_slot_info;
  // 2米2的车位重规划考虑的障碍物单侧最多入侵车位15厘米
  double dy1 = 0.15 / 1.1 * (ego_slot_info.slot_width * 0.5);

  // 内外侧障碍物往远离车位的一遍考虑1.68米就可以
  double dy2 = 1.68;

  // 最多高于车位3.468米的障碍物可以当做内外侧障碍物
  double dx1 = 3.468;
  // 但是如果自车位置本身较低 那么内外侧障碍物考虑的x值也应该降低
  dx1 = std::min(dx1, ego_slot_info.ego_pos_slot.x() -
                          apa_param.GetParam().car_width * 0.5 -
                          ego_slot_info.slot_length);
  // 也需要有个最低考虑位置
  dx1 = std::max(dx1, 0.368);

  // 对于5米长的车位 从车位线往内延长4.86米当做内外侧障碍物即可
  // 这个时候可以参考当做根据障碍物移动车位的标准， 再深就无需横向移动
  double dx2 = 4.86 / 5.0 * ego_slot_info.slot_length;

  // 对于5米长的车位 最多往后5.2米的障碍物可以在重规划的时候不考虑
  // 再往后就要考虑
  double dx3 = 5.2 / 5.0 * ego_slot_info.slot_length - dx2;

  Eigen::Vector2d slot_left_pt = ego_slot_info.pt_1;
  Eigen::Vector2d slot_right_pt = ego_slot_info.pt_0;
  if (slot_left_pt.y() < slot_right_pt.y()) {
    std::swap(slot_left_pt, slot_right_pt);
  }

  bool is_left_side = false;
  if (slot_t_lane_.slot_side == geometry_lib::SLOT_SIDE_LEFT) {
    is_left_side = true;
  }

  std::vector<Eigen::Vector2d> inside_area;
  std::vector<Eigen::Vector2d> outside_area;
  std::vector<Eigen::Vector2d> in_area;
  std::vector<Eigen::Vector2d> discard_area;
  inside_area.resize(4);
  outside_area.resize(4);
  in_area.resize(4);
  discard_area.resize(4);

  const Eigen::Vector2d unit_right2left_vec =
      (slot_left_pt - slot_right_pt).normalized();
  const Eigen::Vector2d unit_left2right_vec = -unit_right2left_vec;
  const Eigen::Vector2d unit_up2down_vec(-1.0, 0.0);
  const Eigen::Vector2d unit_down2up_vec = -unit_up2down_vec;

  // Firstly, the default right side is the inner side, and the left side is the
  // outer side
  Eigen::Vector2d pt;
  // cal inside area
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  inside_area[0] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx1 * unit_down2up_vec;
  inside_area[1] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;
  inside_area[2] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  inside_area[3] = pt;

  // cal outside area
  pt = slot_left_pt + dy2 * unit_left2right_vec + dx1 * unit_down2up_vec;
  outside_area[0] = pt;
  pt = slot_left_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  outside_area[1] = pt;
  pt = slot_left_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  outside_area[2] = pt;
  pt = slot_left_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;

  if (is_left_side) {
    std::swap(inside_area, outside_area);
  }

  // cal in_area
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx1 * unit_down2up_vec;
  in_area[0] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  in_area[1] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  in_area[2] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx2 * unit_up2down_vec;
  in_area[3] = pt;

  // cal discard area
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx2 * unit_up2down_vec;
  discard_area[0] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;
  discard_area[1] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec +
       (dx2 + dx3) * unit_up2down_vec;
  discard_area[2] = pt;
  pt =
      slot_left_pt + dy2 * unit_right2left_vec + (dx2 + dx3) * unit_up2down_vec;
  discard_area[3] = pt;

  if (geometry_lib::IsPointInPolygon(inside_area, obs_slot)) {
    return SlotObsType::INSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(outside_area, obs_slot)) {
    return SlotObsType::OUTSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(in_area, obs_slot)) {
    return SlotObsType::IN_OBS;
  } else if (geometry_lib::IsPointInPolygon(discard_area, obs_slot)) {
    return SlotObsType::DISCARD_OBS;
  } else {
    return SlotObsType::OTHER_OBS;
  }
}

}  // namespace apa_planner
}  // namespace planning