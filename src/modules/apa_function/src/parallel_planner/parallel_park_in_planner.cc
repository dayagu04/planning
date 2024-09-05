#include "parallel_park_in_planner.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

#include "apa_data.h"
#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "math_lib.h"
#include "obstacle.h"
#include "parallel_path_planner.h"

namespace planning {
namespace apa_planner {
static double kFrontDetaXMagWhenFrontVacant = 1.98;
static double kFrontMaxDetaXMagWhenFrontOccupied = 0.5;
static double kRearDetaXMagWhenFrontVacant = 0.4;
static double kRearDetaXMagWhenBothSidesVacant = 0.2;
static double kRearDetaXMagWhenFrontOccupiedRearVacant = 1.2;
static double kRearDetaXMagWhenFrontVacantRearOccupied = 0.2;
static double kRearMaxDetaXMagWhenRearOccupied = 0.5;

static double kFrontObsLineYMagIdentification = 0.6;
static double kRearObsLineYMagIdentification = 0.6;
static double kCurbInitialOffset = 0.36;
static double kCurbYMagIdentification = 0.0;

static double kMaxDistDeleteObsToEgoInSlot = 0.3;
static double kMaxDistDeleteObsToEgoOutSlot = 0.35;

static double kMinChannelYMagIdentification = 3.3;

static double kTBoundarySampleDist = 0.38;
static double kChannelSampleDist = 0.46;
static double kEnterMultiPlanSlotRatio = 0.3;
static double kEps = 1e-5;

void ParallelParkInPlanner::Reset() {
  frame_.Reset();
  t_lane_.Reset();
  parallel_path_planner_.Reset();
}

void ParallelParkInPlanner::PlanCore() {
  // init simulation
  InitSimulation();

  // check planning status
  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan &&
      CheckPlanSkip()) {
    return;
  }

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
    }
    return;
  }

  // update remain dist
  UpdateRemainDist();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    std::cout << "update ego slot info" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // check finish
  if (CheckFinished()) {
    std::cout << "check apa finished!" << std::endl;
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    std::cout << "check stuck failed!" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // check replan
  if (CheckReplan() || apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan) {
    std::cout << "replan is required!" << std::endl;

    // generate t-lane
    GenTlane();

    // update obstacles
    GenTBoundaryObstacles();
    // GenObstacles();

    // path plan
    const auto pathplan_result = PathPlanOnce();
    frame_.pathplan_result = pathplan_result;

    if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_GEARCHANGE);
        std::cout << "replan from PARKING_GEARCHANGE!" << std::endl;
      } else {
        SetParkingStatus(PARKING_FAILED);
        std::cout << "replan failed from PLAN_HOLD!" << std::endl;
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_PLANNING);
        std::cout << "replan from PARKING_PLANNING!" << std::endl;
      } else {
        SetParkingStatus(PARKING_FAILED);
        std::cout << "replan failed from PARKING_PLANNING!" << std::endl;
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
      SetParkingStatus(PARKING_FAILED);
    }

    std::cout << "pathplan_result = " << static_cast<int>(pathplan_result)
              << std::endl;
  } else {
    std::cout << "replan is not required!" << std::endl;
    SetParkingStatus(PARKING_RUNNING);
  }

  // print planning status
  // std::cout << "parking status = "
  //           << static_cast<int>(GetPlannerStates().planning_status)
  //           << std::endl;
}

const bool ParallelParkInPlanner::UpdateEgoSlotInfo() {
  const auto* measures_ptr = &apa_world_ptr_->GetApaDataPtr()->measurement_data;
  const auto slot_manager_ptr = apa_world_ptr_->GetSlotManagerPtr();

  const auto& select_slot_slm =
      slot_manager_ptr->GetEgoSlotInfo().select_slot_filter;

  if (!select_slot_slm.has_corner_points()) {
    DEBUG_PRINT("no selected corner pts in slm!");
    return false;
  }

  if (select_slot_slm.corner_points().corner_point_size() != 4) {
    DEBUG_PRINT("select slot in slm corner points size != 4!");
    return false;
  }

  auto& ego_slot_info = frame_.ego_slot_info;

  // notice: get slot from GetEgoSlotInfo.select_slot_filter in slot management
  ego_slot_info.target_managed_slot.CopyFrom(
      slot_manager_ptr->GetEgoSlotInfo().select_slot_filter);

  const auto& slot_points =
      ego_slot_info.target_managed_slot.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(slot_points.size());
  Eigen::Vector2d slot_center = Eigen::Vector2d::Zero();

  std::cout << "parallel slot points in slm :" << std::endl;
  for (int i = 0; i < slot_points.size(); i++) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
    slot_center += pt[i];
    std::cout << pt[i].transpose() << std::endl;
  }
  ego_slot_info.slot_corner = pt;
  slot_center *= 0.25;

  // calc slot side once at first
  if (frame_.is_replan_first) {
    t_lane_.slot_side = slot_manager_ptr->GetEgoSlotInfo().slot_side;
    if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      t_lane_.slot_side_sgn = 1.0;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      t_lane_.slot_side_sgn = -1.0;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      t_lane_.slot_side_sgn = 0.0;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      std::cout << "calculate parallel slot side error " << std::endl;
      return false;
    }
  }
  Eigen::Vector2d n = Eigen::Vector2d::Zero();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();

  ego_slot_info.slot_length = (pt[0] - pt[1]).norm();
  pnc::geometry_lib::LineSegment line_01(pt[0], pt[1]);

  // note: slot points' order is corrected in slot management
  if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    ego_slot_info.slot_width =
        std::min(pnc::geometry_lib::CalPoint2LineDist(pt[2], line_01),
                 pnc::geometry_lib::CalPoint2LineDist(pt[3], line_01));

    n = (pt[0] - pt[1]).normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = pt[0] - ego_slot_info.slot_length * n -
                                    0.5 * ego_slot_info.slot_width * t;
  } else {
    ego_slot_info.slot_width =
        pnc::geometry_lib::CalPoint2LineDist(pt[3], line_01);

    n = -(pt[0] - pt[1]).normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = pt[1] - ego_slot_info.slot_length * n +
                                    0.5 * ego_slot_info.slot_width * t;
  }

  std::cout << "slot width =" << ego_slot_info.slot_width << std::endl;

  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;

  std::cout << "origin heading =" << ego_slot_info.slot_origin_heading * 57.3
            << std::endl;

  std::cout << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;

  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot = ego_slot_info.g2l_tf.GetPos(measures_ptr->pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // calc terminal pos, try best to stop in the middle of slot
  const double terminal_x = 0.5 * ego_slot_info.slot_length -
                            0.5 * apa_param.GetParam().car_length +
                            apa_param.GetParam().rear_overhanging;

  const double slot_side_sgn =
      t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  const double terminal_y =
      slot_side_sgn * apa_param.GetParam().terminal_parallel_y_offset;

  ego_slot_info.target_ego_pos_slot << terminal_x, terminal_y;
  ego_slot_info.target_ego_heading_slot = 0.0;

  std::cout << "target ego pos in slot ="
            << ego_slot_info.target_ego_pos_slot.transpose()
            << " heading =" << ego_slot_info.target_ego_heading_slot * 57.3
            << std::endl;

  // calc terminal error once
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      pnc::geometry_lib::NormalizeAngle(ego_slot_info.ego_heading_slot -
                                        ego_slot_info.target_ego_heading_slot));

  std::cout << "-- ego_slot:" << std::endl;
  std::cout << "ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << std::endl;

  std::cout << "ego_heading_slot (deg)= "
            << ego_slot_info.ego_heading_slot * 57.3 << std::endl;

  std::cout << "slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;
  std::cout << "vel_ego = " << measures_ptr->vel << std::endl;

  // calc slot occupied ratio

  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(ego_slot_info.terminal_err.pos.x(), -3.0, 4.0)) {
    const double y_err_ratio =
        ego_slot_info.terminal_err.pos.y() / (0.5 * ego_slot_info.slot_width);

    if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  ego_slot_info.slot_occupied_ratio = slot_occupied_ratio;

  std::cout << "ego_slot_info.slot_occupied_ratio = "
            << ego_slot_info.slot_occupied_ratio << std::endl;

  ego_slot_info.obs_pt_vec_slot.clear();
  const pnc::geometry_lib::PathPoint ego_pose(ego_slot_info.ego_pos_slot,
                                              ego_slot_info.ego_heading_slot);

  DEBUG_PRINT("parallel slot_manager_ptr->GetRealTimeObsPtVec() = "
              << slot_manager_ptr->GetRealTimeObsPtVec().size())
  // size_t dist_fail_cnt = 0;
  // size_t total_box_x_fail_cnt = 0;
  // size_t total_box_y_fail_cnt = 0;
  // size_t front_box_fail_cnt = 0;
  // size_t rear_box_fail_cnt = 0;
  // size_t in_ego_cnt = 0;

  for (const auto& obs_pt_global : slot_manager_ptr->GetRealTimeObsPtVec()) {
    if ((obs_pt_global - slot_center).norm() > 13.3) {
      // dist_fail_cnt++;
      continue;
    }

    const auto obs_pt_local = frame_.ego_slot_info.g2l_tf.GetPos(obs_pt_global);
    // outof total box range
    if (!pnc::mathlib::IsInBound(obs_pt_local.x(), -1.6,
                                 apa_param.GetParam().parallel_channel_x_mag)) {
      // total_box_x_fail_cnt++;
      continue;
    }

    if (obs_pt_local.y() * slot_side_sgn >
            apa_param.GetParam().parallel_channel_y_mag ||
        obs_pt_local.y() * slot_side_sgn <
            (-0.5 * t_lane_.slot_width - apa_param.GetParam().curb_offset)) {
      // total_box_y_fail_cnt++;
      continue;
    }

    // remote front T-boundary obs
    if (obs_pt_local.x() >
            t_lane_.slot_length + kFrontDetaXMagWhenFrontVacant &&
        obs_pt_local.y() * slot_side_sgn < -0.3 * slot_side_sgn) {
      // front_box_fail_cnt++;
      continue;
    }

    // remote rear T-boundary obs
    if (obs_pt_local.x() < -1.6 &&
        obs_pt_local.y() * slot_side_sgn < -0.3 * slot_side_sgn) {
      // rear_box_fail_cnt++;
      continue;
    }

    if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs_pt_local, ego_pose, 0.3)) {
      // in_ego_cnt++;
      continue;
    }
    ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_local));
  }

  // DEBUG_PRINT("dist_fail_cnt = " << dist_fail_cnt);
  // DEBUG_PRINT("total_box_x_fail_cnt = " << total_box_x_fail_cnt);
  // DEBUG_PRINT("total_box_y_fail_cnt = " << total_box_y_fail_cnt);
  // DEBUG_PRINT("front_box_fail_cnt = " << front_box_fail_cnt);
  // DEBUG_PRINT("rear_box_fail_cnt = " << rear_box_fail_cnt);
  // DEBUG_PRINT("in_ego_cnt = " << in_ego_cnt);
  // DEBUG_PRINT("ego_slot_info.obs_pt_vec_slot size = "
  //             << ego_slot_info.obs_pt_vec_slot.size());

  // update stuck time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      measures_ptr->static_flag && !measures_ptr->brake_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_IN) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // update pause time
  if (frame_.plan_stm.planning_status == PARKING_PAUSED) {
    frame_.pause_time += apa_param.GetParam().plan_time;
  } else {
    frame_.pause_time = 0.0;
  }

  return true;
}

void ParallelParkInPlanner::GenTlane() {
  // Todo: generate t-lane according to nearby obstacles
  const auto& ego_slot_info = frame_.ego_slot_info;

  // y
  // ^_______________    left side
  // |               |
  // |->x            |
  // |               |
  //
  // y                  ego car------->>
  // ^
  // |               |
  // |->x            |
  // |_______________|   right side

  const double slot_length = ego_slot_info.slot_length;
  const double half_slot_width = 0.5 * ego_slot_info.slot_width;
  const double quarter_slot_width = 0.5 * half_slot_width;
  const double side_sgn =
      (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0);

  // set initial x coordination for front and rear tlane obs
  double front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
  double rear_max_x = -kRearDetaXMagWhenFrontOccupiedRearVacant;

  // set initial y coordination for front and rear y tlane obs
  double front_parallel_line_y_limit = side_sgn * quarter_slot_width;
  double rear_parallel_line_y_limit = front_parallel_line_y_limit;

  // channel
  double channel_x_limit = apa_param.GetParam().parallel_channel_x_mag;
  double channel_y_limit =
      side_sgn * apa_param.GetParam().parallel_channel_y_mag;

  // curb
  size_t curb_count = 0;
  double curb_y_limit = -side_sgn * (half_slot_width + kCurbInitialOffset);

  DEBUG_PRINT("ego_slot_info.obs_pt_vec_slot size ="
              << ego_slot_info.obs_pt_vec_slot.size());

  // for json debug
  std::vector<double> front_que_x;
  std::vector<double> front_que_y;
  std::vector<double> rear_que_x;
  std::vector<double> rear_que_y;
  // filter obstacles meeting requirements that are in slot system
  for (const auto& obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    const bool front_obs_condition =
        pnc::mathlib::IsInBound(
            obstacle_point_slot.x(),
            slot_length - kFrontMaxDetaXMagWhenFrontOccupied,
            slot_length + kFrontDetaXMagWhenFrontVacant) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(),
            -kFrontObsLineYMagIdentification * side_sgn,
            (half_slot_width + kFrontObsLineYMagIdentification) * side_sgn);

    if (front_obs_condition) {
      front_que_x.emplace_back(obstacle_point_slot.x());
      front_que_y.emplace_back(obstacle_point_slot.y());
      front_min_x = std::min(front_min_x, obstacle_point_slot.x());

      // DEBUG_PRINT("front_obs_condition!");
    }

    const bool rear_obs_condition =
        ((pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                  -kRearDetaXMagWhenFrontOccupiedRearVacant,
                                  kRearMaxDetaXMagWhenRearOccupied)) &&
         (pnc::mathlib::IsInBound(
             obstacle_point_slot.y(),
             -side_sgn * kRearObsLineYMagIdentification,
             (half_slot_width + kRearObsLineYMagIdentification) * side_sgn)));

    if (rear_obs_condition) {
      rear_que_x.emplace_back(obstacle_point_slot.x());
      rear_que_y.emplace_back(obstacle_point_slot.y());
      rear_max_x = std::max(rear_max_x, obstacle_point_slot.x());
      // DEBUG_PRINT("rear_obs_condition!");
    }

    const bool curb_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                kRearDetaXMagWhenFrontVacant,
                                slot_length - kRearDetaXMagWhenFrontVacant) &&
        (obstacle_point_slot.y() * side_sgn <= -kCurbYMagIdentification);

    if (curb_condition) {
      curb_count++;
      if (side_sgn > 0.0) {
        curb_y_limit = std::max(curb_y_limit, obstacle_point_slot.y());
        curb_y_limit = std::min(curb_y_limit, half_slot_width);
      } else {
        curb_y_limit = std::min(curb_y_limit, obstacle_point_slot.y());
        curb_y_limit = std::max(curb_y_limit, half_slot_width);
      }

      // DEBUG_PRINT("curb condition!");
    }

    // Todo: select obs in ROI instead of using obs deciding channel y
    // const bool channel_y_condition =
    //     (obstacle_point_slot.x() > -1.0) &&
    //     (obstacle_point_slot.x() < slot_length + 2.0) &&
    //     (obstacle_point_slot.y() * side_sgn >=
    //     kMinChannelYMagIdentification);

    // if (channel_y_condition) {
    //   channel_y_limit =
    //       side_sgn > 0.0 ? std::min(channel_y_limit, obstacle_point_slot.y())
    //                      : std::max(channel_y_limit,
    //                      obstacle_point_slot.y());
    //   // DEBUG_PRINT("channel_y_condition!");
    // }

    const bool front_parallel_line_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), slot_length - 0.2,
                                slot_length + 1.6) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y(), 0.0, 2.2 * side_sgn);

    if (front_parallel_line_condition) {
      front_parallel_line_y_limit =
          side_sgn > 0.0
              ? std::max(front_parallel_line_y_limit, obstacle_point_slot.y())
              : std::min(front_parallel_line_y_limit, obstacle_point_slot.y());
      // DEBUG_PRINT("front_parallel_line_y_limit condition!");
    }

    const bool rear_parallel_line_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), -2.5, 0.2) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y() * side_sgn, 0.0, 2.3);

    if (rear_parallel_line_condition) {
      rear_parallel_line_y_limit =
          side_sgn > 0.0
              ? std::max(rear_parallel_line_y_limit, obstacle_point_slot.y())
              : std::min(rear_parallel_line_y_limit, obstacle_point_slot.y());
      // DEBUG_PRINT("rear_parallel_line_y_limit condition!");
    }
  }
  bool front_vacant = false;
  bool rear_vacant = false;

  if (front_min_x >= slot_length + kFrontDetaXMagWhenFrontVacant - kEps) {
    front_vacant = true;
    DEBUG_PRINT("front space empty!");
  }

  if (rear_max_x <= -kRearDetaXMagWhenFrontOccupiedRearVacant + kEps) {
    rear_vacant = true;
    DEBUG_PRINT("rear space empty!");
  }

  JSON_DEBUG_VALUE("para_tlane_is_front_vacant", front_vacant)
  JSON_DEBUG_VALUE("para_tlane_is_rear_vacant", rear_vacant)
  JSON_DEBUG_VALUE("para_tlane_side_sgn", side_sgn)

  if (front_vacant && rear_vacant) {
    front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
    rear_max_x = -kRearDetaXMagWhenBothSidesVacant;

  } else if (front_vacant && !rear_vacant) {
    front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
    // protection for rear x due to the low accuracy of rear uss obstacle points
    rear_max_x =
        std::max(rear_max_x, -kRearDetaXMagWhenFrontVacantRearOccupied);

  } else if (!front_vacant && rear_vacant) {
    rear_max_x = -kRearDetaXMagWhenFrontOccupiedRearVacant;
  } else {
  }

  DEBUG_PRINT("para_tlane_front_min_x_before_clamp = " << front_min_x);
  JSON_DEBUG_VALUE("para_tlane_front_min_x_before_clamp", front_min_x)

  front_min_x = pnc::mathlib::Clamp(
      front_min_x, slot_length - kRearDetaXMagWhenFrontVacant,
      slot_length + kFrontDetaXMagWhenFrontVacant);

  DEBUG_PRINT("para_tlane_front_min_x_after_clamp =" << front_min_x);
  JSON_DEBUG_VALUE("para_tlane_front_min_x_after_clamp", front_min_x)

  JSON_DEBUG_VECTOR("tlane_front_que_x", front_que_x, 2)
  JSON_DEBUG_VECTOR("tlane_front_que_y", front_que_y, 2)

  const double front_y_limit = front_parallel_line_y_limit;
  DEBUG_PRINT("front parallel line y =" << front_y_limit);
  JSON_DEBUG_VALUE("para_tlane_front_y", front_y_limit)

  DEBUG_PRINT("para_tlane_rear_max_x_before_clamp = " << rear_max_x);
  JSON_DEBUG_VALUE("para_tlane_rear_max_x_before_clamp", rear_max_x)

  rear_max_x =
      pnc::mathlib::Clamp(rear_max_x, -kRearDetaXMagWhenFrontOccupiedRearVacant,
                          kRearMaxDetaXMagWhenRearOccupied);

  DEBUG_PRINT("para_tlane_rear_max_x_after_clamp = " << rear_max_x);
  JSON_DEBUG_VALUE("para_tlane_rear_max_x_after_clamp", rear_max_x)

  JSON_DEBUG_VECTOR("tlane_rear_que_x", rear_que_x, 2)
  JSON_DEBUG_VECTOR("tlane_rear_que_y", rear_que_y, 2)

  const double rear_y_limit = rear_parallel_line_y_limit;
  DEBUG_PRINT("rear parallel line y =" << rear_y_limit);
  JSON_DEBUG_VALUE("para_tlane_rear_y", rear_y_limit)

  t_lane_.obs_pt_inside << front_min_x, front_y_limit;
  t_lane_.obs_pt_outside << rear_max_x, rear_y_limit;

  curb_y_limit =
      pnc::mathlib::Clamp(curb_y_limit, -side_sgn * (half_slot_width + 0.4),
                          -side_sgn * half_slot_width);

  t_lane_.corner_inside_slot << slot_length, half_slot_width * side_sgn;
  t_lane_.corner_outside_slot << 0.0, half_slot_width * side_sgn;

  t_lane_.curb_y = curb_y_limit;
  t_lane_.channel_y = channel_y_limit;
  t_lane_.channel_x_limit = channel_x_limit;

  t_lane_.pt_outside = t_lane_.obs_pt_outside;
  t_lane_.pt_inside = t_lane_.obs_pt_inside;

  t_lane_.slot_length = slot_length;
  t_lane_.slot_width = ego_slot_info.slot_width;

  std::vector<double> tlane_obs_pt_vec;
  tlane_obs_pt_vec.emplace_back(t_lane_.obs_pt_inside.x());
  tlane_obs_pt_vec.emplace_back(t_lane_.obs_pt_inside.y());
  tlane_obs_pt_vec.emplace_back(t_lane_.obs_pt_outside.x());
  tlane_obs_pt_vec.emplace_back(t_lane_.obs_pt_outside.x());
  JSON_DEBUG_VECTOR("para_tlane_obs_pt_before_uss", tlane_obs_pt_vec, 2)

  // update tlane using USS dist
  if (frame_.ego_slot_info.slot_occupied_ratio >= kEnterMultiPlanSlotRatio &&
      (frame_.in_slot_plan_count == 0 ||
       std::fabs(frame_.ego_slot_info.terminal_err.heading) <= 15.0 / 57.3)) {
    UpdateTlaneOnceInSlot();
  }

  // if curb exist, combine curb and slot center line to decide target y
  if (curb_count > 3) {
    const double target_y_with_curb =
        curb_y_limit +
        side_sgn * (apa_param.GetParam().terminal_parallel_y_offset_with_curb +
                    0.5 * apa_param.GetParam().car_width);

    frame_.ego_slot_info.target_ego_pos_slot.y() =
        (side_sgn > 0.0
             ? std::max(t_lane_.pt_terminal_pos.y(), target_y_with_curb)
             : std::min(t_lane_.pt_terminal_pos.y(), target_y_with_curb));
  }

  // for terminal pose: get accurate target x combining with obs tlane
  // const double front_obs_x = t_lane_.obs_pt_inside.x();
  // const double rear_obs_x = t_lane_.obs_pt_outside.x();
  // const double front_corner_x = t_lane_.corner_inside_slot.x();
  // const double rear_corner_x = t_lane_.corner_outside_slot.x();

  const double rac_to_geom_center = 0.5 * apa_param.GetParam().car_length -
                                    apa_param.GetParam().rear_overhanging;

  const double target_x_in_slot_center =
      0.5 * frame_.ego_slot_info.slot_length - rac_to_geom_center;

  // const double target_x_in_slot_center =
  //     (front_corner_x + rear_corner_x) * 0.5 - rac_to_geom_center;

  // const double stop_buffer =
  //     apa_param.GetParam().finish_parallel_rear_stop_buffer;

  // const double ego_front_x = target_x_in_slot_center +
  //                            apa_param.GetParam().wheel_base +
  //                            apa_param.GetParam().front_overhanging;
  // const double ego_rear_x =
  //     target_x_in_slot_center - apa_param.GetParam().rear_overhanging;

  // DEBUG_PRINT("terminal debug --------------------------------");
  // DEBUG_PRINT("target_x_in_slot_center =" << target_x_in_slot_center);

  // const bool is_front_exceeded = ego_front_x > front_obs_x - stop_buffer;
  // const bool is_rear_exceeded = ego_rear_x < rear_obs_x + stop_buffer;

  // double target_x = target_x_in_slot_center;
  // if (is_front_exceeded && is_rear_exceeded) {
  //   DEBUG_PRINT("both sides exceed!");
  //   target_x = (front_obs_x + rear_obs_x) * 0.5 - rac_to_geom_center;
  // } else if (is_front_exceeded && !is_rear_exceeded) {
  //   DEBUG_PRINT("front exceeded, rear safe!");
  //   const double target_x_via_front_obs =
  //       front_obs_x - apa_param.GetParam().wheel_base -
  //       apa_param.GetParam().front_overhanging - stop_buffer;

  //   const double target_x_via_obs_corner =
  //       (front_obs_x + rear_corner_x) * 0.5 - rac_to_geom_center;
  //   target_x = std::min(target_x_via_front_obs, target_x_via_obs_corner);

  //   const double target_x_via_obs_center =
  //       (front_obs_x + rear_obs_x) * 0.5 - rac_to_geom_center;

  //   target_x = std::max(target_x, target_x_via_obs_center);

  // } else if (!is_front_exceeded && is_rear_exceeded) {
  //   DEBUG_PRINT("rear exceeded, front safe!");
  //   const double target_x_via_rear_obs =
  //       rear_obs_x + apa_param.GetParam().rear_overhanging + stop_buffer;
  //   const double target_x_via_corner_obs =
  //       (front_corner_x + rear_obs_x) * 0.5 - rac_to_geom_center;

  //   target_x = std::max(target_x_via_rear_obs, target_x_via_corner_obs);

  //   const double target_x_via_obs_center =
  //       (front_obs_x + rear_obs_x) * 0.5 - rac_to_geom_center;
  //   target_x = std::min(target_x, target_x_via_obs_center);
  // } else {
  //   DEBUG_PRINT("both sides safe");
  //   target_x = target_x_in_slot_center;
  // }
  // frame_.ego_slot_info.target_ego_pos_slot.x() = target_x;

  frame_.ego_slot_info.target_ego_pos_slot.x() = target_x_in_slot_center;

  t_lane_.pt_terminal_pos = frame_.ego_slot_info.target_ego_pos_slot;

  frame_.ego_slot_info.terminal_err.Set(
      frame_.ego_slot_info.ego_pos_slot -
          frame_.ego_slot_info.target_ego_pos_slot,
      pnc::geometry_lib::NormalizeAngle(
          frame_.ego_slot_info.ego_heading_slot -
          frame_.ego_slot_info.target_ego_heading_slot));

  DEBUG_PRINT("-- t_lane --------");
  if (pnc::mathlib::IsDoubleEqual(side_sgn, 1.0)) {
    DEBUG_PRINT("right slot");
  } else if (pnc::mathlib::IsDoubleEqual(side_sgn, -1.0)) {
    DEBUG_PRINT("left slot!");
  }
  DEBUG_PRINT("obs_pt_inside = " << t_lane_.obs_pt_inside.transpose());
  DEBUG_PRINT("obs_pt_outside = " << t_lane_.obs_pt_outside.transpose());
  DEBUG_PRINT(
      "corner_inside_slot = " << t_lane_.corner_inside_slot.transpose());
  DEBUG_PRINT(
      "corner_outside_slot = " << t_lane_.corner_outside_slot.transpose());

  DEBUG_PRINT("pt_outside = " << t_lane_.pt_outside.transpose());
  DEBUG_PRINT("pt_inside = " << t_lane_.pt_inside.transpose());
  DEBUG_PRINT("pt_terminal_pos = " << t_lane_.pt_terminal_pos.transpose());
  DEBUG_PRINT("slot length =" << t_lane_.slot_length);
  DEBUG_PRINT("half slot width =" << 0.5 * t_lane_.slot_width);
  DEBUG_PRINT("curb y =" << t_lane_.curb_y);
  DEBUG_PRINT("channel x =" << t_lane_.channel_x_limit);
  DEBUG_PRINT("channel y =" << t_lane_.channel_y);
  DEBUG_PRINT("------------------");

  JSON_DEBUG_VALUE("para_tlane_obs_in_x", t_lane_.obs_pt_inside.x())
  JSON_DEBUG_VALUE("para_tlane_obs_in_y", t_lane_.obs_pt_inside.y())
  JSON_DEBUG_VALUE("para_tlane_obs_out_x", t_lane_.obs_pt_outside.x())
  JSON_DEBUG_VALUE("para_tlane_obs_out_x", t_lane_.obs_pt_outside.y())

  std::vector<double> x_vec = {0.0};
  std::vector<double> y_vec = {0.0};
  std::vector<double> phi_vec = {0.0};

  JSON_DEBUG_VECTOR("col_det_path_x", x_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_y", y_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_phi", phi_vec, 2)
}

void ParallelParkInPlanner::UpdateTlaneOnceInSlot() {
  if (frame_.ego_slot_info.slot_occupied_ratio < kEnterMultiPlanSlotRatio) {
    DEBUG_PRINT("ego not in slot!");
    return;
  }

  DEBUG_PRINT("UpdateTlaneOnceInSlot work now!");

  // update tlane via 12 uss dist
  const pnc::geometry_lib::PathPoint ego_pose(
      frame_.ego_slot_info.ego_pos_slot, frame_.ego_slot_info.ego_heading_slot);

  // get front and rear uss dist
  const auto& uss_dist_vec =
      apa_world_ptr_->GetUssObstacleAvoidancePtr()->GetUssDistVec();

  if (uss_dist_vec.size() != 12) {
    DEBUG_PRINT("uss dist size != 12");
    return;
  }

  // get upa dist, first four parameters are the front side, the others are the
  // rear size.
  std::vector<size_t> upa_idx_vec = {1, 2, 3, 4, 7, 8, 9, 10};

  if (t_lane_.slot_side_sgn > 0.0) {
    upa_idx_vec = {1, 2, 3, 8, 9, 10};
  } else {
    upa_idx_vec = {2, 3, 4, 7, 8, 9};
  }

  pnc::geometry_lib::LocalToGlobalTf ego2slot(ego_pose.pos, ego_pose.heading);

  for (size_t i = 0; i < upa_idx_vec.size(); i++) {
    const auto idx = upa_idx_vec[i];
    const double upa_dist = uss_dist_vec[idx];

    DEBUG_PRINT("upa dist = " << upa_dist);

    const Eigen::Vector2d upa_pos_ego(
        apa_param.GetParam().uss_vertex_x_vec[idx],
        apa_param.GetParam().uss_vertex_y_vec[idx]);

    const Eigen::Vector2d upa_pos_slot = ego2slot.GetPos(upa_pos_ego);

    const auto half_upa_size =
        static_cast<size_t>(std::floor(0.5 * upa_idx_vec.size()));
    // front
    if (i < half_upa_size) {
      const double upa_obs_x = upa_pos_slot.x() + upa_dist;
      DEBUG_PRINT("front upa_obs_x = " << upa_obs_x
                                       << " upa_dist =" << upa_dist);
      if (upa_obs_x >= t_lane_.corner_inside_slot.x() - 0.5) {
        t_lane_.obs_pt_inside.x() =
            std::min(t_lane_.obs_pt_inside.x(), upa_obs_x);
      }
    } else {
      const double upa_obs_x = upa_pos_slot.x() - upa_dist;
      DEBUG_PRINT("rear upa_obs_x = " << upa_obs_x
                                      << " upa_dist =" << upa_dist);
      // rear
      if (upa_obs_x <= t_lane_.obs_pt_inside.x() + 0.5) {
        t_lane_.obs_pt_outside.x() =
            std::max(t_lane_.obs_pt_outside.x(), upa_obs_x);
      }
    }
  }
}

void ParallelParkInPlanner::GenTBoundaryObstacles() {
  //                                                  |
  //                         c-------------D
  //                         |             |  pin
  //            A -----------B pout        E---------F
  //                         |       ego-------->     |
  //  channel pt1 ----------------------------------- channel pt2
  //                                                  |
  //                         |       ego-------->     |
  //            A -----------B pout        E---------F
  //                         |             |  pin
  //                         c-------------D

  apa_world_ptr_->GetCollisionDetectorPtr()->Reset();

  // set T-Boundary obstacles
  const Eigen::Vector2d B(t_lane_.obs_pt_outside.x(),
                          0.5 * t_lane_.slot_side_sgn);

  const Eigen::Vector2d A(B.x() - 1.2, B.y());
  const Eigen::Vector2d E(
      t_lane_.obs_pt_inside.x(),
      t_lane_.obs_pt_inside.y() - t_lane_.slot_side_sgn * 0.5);

  const Eigen::Vector2d C(
      B.x(), (-0.5 * t_lane_.slot_width - apa_param.GetParam().curb_offset) *
                 t_lane_.slot_side_sgn);
  const Eigen::Vector2d D(E.x(), C.y());
  const Eigen::Vector2d F(t_lane_.channel_x_limit, E.y());

  const Eigen::Vector2d channel_point_1(
      A.x(),
      apa_param.GetParam().parallel_channel_y_mag * t_lane_.slot_side_sgn);

  const Eigen::Vector2d channel_point_2(
      F.x(),
      apa_param.GetParam().parallel_channel_y_mag * t_lane_.slot_side_sgn);

  const pnc::geometry_lib::LineSegment channel_line(channel_point_1,
                                                    channel_point_2);

  // sample channel
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  pnc::geometry_lib::SamplePointSetInLineSeg(channel_obstacle_vec, channel_line,
                                             kChannelSampleDist);

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      channel_obstacle_vec, CollisionDetector::CHANNEL_OBS);

  for (const auto& obstacle_point_slot : frame_.ego_slot_info.obs_pt_vec_slot) {
    const bool channel_y_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), A.x(), F.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn,
            channel_point_1.y());

    if (channel_y_condition) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
          obstacle_point_slot, CollisionDetector::CHANNEL_OBS);

      if (pnc::mathlib::IsInBound(obstacle_point_slot.x(), t_lane_.slot_length,
                                  t_lane_.slot_length + 3.0)) {
        if (t_lane_.slot_side_sgn > 0.0) {
          t_lane_.channel_y =
              std::min(obstacle_point_slot.y(), t_lane_.channel_y);
        } else {
          t_lane_.channel_y =
              std::max(obstacle_point_slot.y(), t_lane_.channel_y);
        }
      }
    }
  }

  // set tlane obs
  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  // currently channel and curb are set as obstacles
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(100);
  std::vector<Eigen::Vector2d> point_set;
  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }

  for (const auto& obs_pos : tlane_obstacle_vec) {
    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        obs_pos, CollisionDetector::TLANE_BOUNDARY_OBS);
  }

  for (const auto& obstacle_point_slot : frame_.ego_slot_info.obs_pt_vec_slot) {
    const bool is_rear_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), A.x(),
                                t_lane_.obs_pt_outside.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), B.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn);

    if (is_rear_tlane_line) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
          obstacle_point_slot, CollisionDetector::TLANE_BOUNDARY_OBS);
      continue;
    }

    const bool is_front_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                t_lane_.obs_pt_inside.x() - 0.3, F.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), E.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn);

    if (is_front_tlane_line) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
          obstacle_point_slot, CollisionDetector::TLANE_BOUNDARY_OBS);
    }
  }

  // tlane
  tlane_line_vec.clear();

  const Eigen::Vector2d C_curb(C.x(), t_lane_.curb_y);
  const Eigen::Vector2d D_curb(D.x(), t_lane_.curb_y);

  tlane_line.SetPoints(B, C_curb);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(D_curb, E);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(C_curb, D_curb);
  tlane_line_vec.emplace_back(tlane_line);

  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }

  for (const auto& obs_pos : tlane_obstacle_vec) {
    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        obs_pos, CollisionDetector::TLANE_OBS);
  }

  for (const auto& obs_pos : frame_.ego_slot_info.obs_pt_vec_slot) {
    const bool is_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), B.x(), E.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y(),
                                C_curb.y());
    if (!is_tlane_obs) {
      continue;
    }

    if (pnc::mathlib::IsInBound(obs_pos.x(), 0.5, t_lane_.slot_length - 0.5) &&
        pnc::mathlib::IsInBound(obs_pos.y(), -0.4 * t_lane_.slot_side_sgn,
                                1.2 * t_lane_.slot_side_sgn)) {
      // obs noise in slot
      continue;
    }

    const bool is_front_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), t_lane_.obs_pt_inside.x(),
                                E.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y(),
                                D_curb.y());
    const bool is_rear_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), B.x(),
                                t_lane_.obs_pt_outside.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), 1.5 * t_lane_.slot_side_sgn,
                                C_curb.y());

    if (is_front_tlane_obs || is_rear_tlane_obs) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
          obs_pos, CollisionDetector::TLANE_OBS);
    }
  }
}

void ParallelParkInPlanner::GenObstacles() {
  // set obstacles.

  //                         ^ y                      |
  //                         c-------------D
  //                         |-->x         |  pin
  //            A -----------B pout        E---------F
  //                         |       ego-------->     |

  //  channel pt1 ----------------------------------- channel pt2
  //                         ^ y                      |
  //                         |       ego-------->     |
  //            A -----------B pout        E---------F
  //                         |-->x         |  pin
  //                         c-------------D

  const Eigen::Vector2d B = t_lane_.obs_pt_outside;
  const Eigen::Vector2d A(t_lane_.obs_pt_outside.x() - 1.0, B.y());
  const Eigen::Vector2d E = t_lane_.obs_pt_inside;

  const Eigen::Vector2d C(B.x(), t_lane_.curb_y);
  const Eigen::Vector2d D(E.x(), t_lane_.curb_y);
  const Eigen::Vector2d F(t_lane_.channel_x_limit, E.y());

  // channel
  const double channel_y = t_lane_.channel_y;
  const Eigen::Vector2d channel_point_1(A.x(), channel_y);
  const Eigen::Vector2d channel_point_2(F.x(), channel_y);

  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_line.SetPoints(channel_point_1, channel_point_2);
  channel_line_vec.emplace_back(channel_line);

  channel_line.SetPoints(channel_point_2, F);
  channel_line_vec.emplace_back(channel_line);

  // sample channel
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  const double obs_sample_ds = apa_param.GetParam().obstacle_ds;

  for (const auto& line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, obs_sample_ds);
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(channel_obstacle_vec);

  double channel_obs_lim = channel_y;
  DEBUG_PRINT("frame_.ego_slot_info.obs_pt_vec_slot size = "
              << frame_.ego_slot_info.obs_pt_vec_slot.size());

  for (const auto& obstacle_point_slot : frame_.ego_slot_info.obs_pt_vec_slot) {
    // Todo: select obs in ROI instead of using obs deciding channel y
    // const bool channel_y_condition =
    //     (obstacle_point_slot.x() > -kRearDetaXMagWhenFrontOccupiedRearVacant)
    //     && (obstacle_point_slot.x() < t_lane_.channel_x_limit) &&
    //     (pnc::mathlib::IsInBound(
    //         obstacle_point_slot.y(),
    //         kMinChannelYMagIdentification * t_lane_.slot_side_sgn,
    //         t_lane_.channel_y));

    const bool channel_y_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                t_lane_.corner_outside_slot.x(),
                                t_lane_.slot_length + 12.0) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn,
            t_lane_.channel_y);

    if (channel_y_condition) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
          obstacle_point_slot);

      if (pnc::mathlib::IsInBound(obstacle_point_slot.x(), t_lane_.slot_length,
                                  t_lane_.slot_length + 3.0)) {
        if (t_lane_.slot_side_sgn > 0.0) {
          channel_obs_lim = std::min(obstacle_point_slot.y(), channel_obs_lim);
        } else {
          channel_obs_lim = std::max(obstacle_point_slot.y(), channel_obs_lim);
        }
      }
    }
  }
  DEBUG_PRINT("pre channel y = " << t_lane_.channel_y);
  t_lane_.channel_y = channel_obs_lim;
  DEBUG_PRINT("channel_obs_lim = " << channel_obs_lim);

  // set tlane obs
  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  // currently channel and curb are set as obstacles
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(B, C);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(C, D);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(D, E);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(100);
  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, obs_sample_ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(frame_.ego_slot_info.ego_pos_slot,
               frame_.ego_slot_info.ego_heading_slot);

  const double safe_dist =
      frame_.ego_slot_info.slot_occupied_ratio < kEnterMultiPlanSlotRatio
          ? kMaxDistDeleteObsToEgoInSlot
          : kMaxDistDeleteObsToEgoOutSlot;

  for (const auto& obs_pos : tlane_obstacle_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs_pos, ego_pose, safe_dist)) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(obs_pos);
      // std::cout << "obs_pos = " << obs_pos.transpose() << std::endl;
    }
  }
}

const bool ParallelParkInPlanner::IsEgoInSlot() const {
  return std::fabs(frame_.ego_slot_info.ego_pos_slot.y() -
                   t_lane_.pt_terminal_pos.y()) <=
         0.5 * (frame_.ego_slot_info.slot_width +
                apa_param.GetParam().car_width - 0.2);
}

const bool ParallelParkInPlanner::IsEgoInSlot(
    const pnc::geometry_lib::PathPoint& pose) const {
  return pose.pos.y() * t_lane_.slot_side_sgn <=
         t_lane_.slot_side_sgn * t_lane_.corner_inside_slot.y();
}

const uint8_t ParallelParkInPlanner::PathPlanOnce() {
  // construct input

  ParallelPathPlanner::Input path_planner_input;
  path_planner_input.tlane = t_lane_;
  path_planner_input.sample_ds =
      apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_complete_path;

  const auto& ego_slot_info = frame_.ego_slot_info;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);
  if (frame_.is_replan_first) {
    // temprarily give driving gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;

    frame_.current_arc_steer =
        t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT
            ? pnc::geometry_lib::SEG_STEER_LEFT
            : pnc::geometry_lib::SEG_STEER_RIGHT;

    path_planner_input.path_planner_state =
        ParallelPathPlanner::PrepareStepPlan;
  } else if (frame_.ego_slot_info.slot_occupied_ratio >
                 kEnterMultiPlanSlotRatio &&
             frame_.in_slot_plan_count == 0) {
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    frame_.current_arc_steer =
        t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT
            ? pnc::geometry_lib::SEG_STEER_RIGHT
            : pnc::geometry_lib::SEG_STEER_LEFT;
  }

  DEBUG_PRINT("frame_.ego_slot_info.slot_occupied_ratio = "
              << frame_.ego_slot_info.slot_occupied_ratio);
  DEBUG_PRINT("frame_.in_slot_plan_count = " << frame_.in_slot_plan_count);

  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;

  JSON_DEBUG_VALUE("ref_gear", path_planner_input.ref_gear);
  JSON_DEBUG_VALUE("ref_arc_steer", path_planner_input.ref_arc_steer);

  DEBUG_PRINT("ref gear to path planner input ="
              << static_cast<int>(path_planner_input.ref_gear));
  DEBUG_PRINT("ref steer to path planner input ="
              << static_cast<int>(path_planner_input.ref_arc_steer));

  parallel_path_planner_.SetInput(path_planner_input);

  const double path_plan_start_time = IflyTime::Now_ms();

  const bool path_plan_success =
      parallel_path_planner_.Update(apa_world_ptr_->GetCollisionDetectorPtr());

  DEBUG_PRINT("path planner cost time(ms) = " << IflyTime::Now_ms() -
                                                     path_plan_start_time);
  // const auto& path_planner_output = parallel_path_planner_.GetOutput();

  frame_.total_plan_count++;
  if (frame_.ego_slot_info.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    frame_.in_slot_plan_count++;
  }

  uint8_t plan_result = 0;
  if (path_plan_success) {
    plan_result = PathPlannerResult::PLAN_UPDATE;
    std::cout << "path plan success!" << std::endl;
  } else {
    plan_result = PathPlannerResult::PLAN_FAILED;
    std::cout << "path plan fail!" << std::endl;
    return PathPlannerResult::PLAN_FAILED;
  }

  parallel_path_planner_.SetCurrentPathSegIndex();
  // parallel_path_planner_.SetLineSegmentHeading();

  // if (frame_.ego_slot_info.slot_occupied_ratio < 0.05 &&
  //     frame_.is_replan_first &&
  //     frame_.ego_slot_info.ego_pos_slot.x() < t_lane_.obs_pt_inside.x()
  //     + 1.0) {
  //   const double extend_lenth = 0.25;
  //   parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
  //       extend_lenth);
  // }

  // if (ego_slot_info.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
  //   const double extend_lenth = 0.15;
  //   parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
  //       extend_lenth);
  // }
  parallel_path_planner_.SampleCurrentPathSeg();

  // print segment info
  // pnc::geometry_lib::PrintSegmentsVecInfo(
  //     parallel_path_planner_.GetOutput().path_segment_vec);

  // reverse info for next plan
  if (frame_.is_replan_first) {
    frame_.current_gear = pnc::geometry_lib::ReverseGear(
        parallel_path_planner_.GetOutput().gear_cmd_vec.front());

    DEBUG_PRINT("next gear =" << static_cast<int>(frame_.current_gear));
  } else {
    // set current gear
    frame_.current_gear = pnc::geometry_lib::ReverseGear(frame_.current_gear);
    DEBUG_PRINT("next gear =" << static_cast<int>(frame_.current_gear));

    if (!pnc::geometry_lib::IsValidGear(frame_.current_gear)) {
      DEBUG_PRINT("frame_.current_gear == invalid gear!");
      return PathPlannerResult::PLAN_FAILED;
    }

    if (frame_.ego_slot_info.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
      // set current arc steer
      frame_.current_arc_steer =
          pnc::geometry_lib::ReverseSteer(frame_.current_arc_steer);

      if (!pnc::geometry_lib::IsValidArcSteer(frame_.current_arc_steer)) {
        DEBUG_PRINT("frame_.current_arc == invalid arc!");
        return PathPlannerResult::PLAN_FAILED;
      }
    }
  }

  frame_.is_replan_first = false;

  const auto& planner_output = parallel_path_planner_.GetOutput();
  frame_.gear_command = planner_output.current_gear;

  std::cout << "start lat optimizer!" << std::endl;

  // lateral path optimization
  bool is_use_optimizer = true;

  // refuse optimizer
  if (planner_output.path_point_vec.size() < 3) {
    std::cout << " input size is too small" << std::endl;
    is_use_optimizer = false;
  } else {
    const auto path_length = (planner_output.path_point_vec.front().pos -
                              planner_output.path_point_vec.back().pos)
                                 .norm();
    if (path_length < apa_param.GetParam().min_opt_path_length) {
      std::cout << "path length is too short, optimizer is closed "
                << std::endl;

      is_use_optimizer = false;
    }
  }

  bool cilqr_optimization_enable = true;
  bool parallel_optimization_enable = true;
  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation) {
    parallel_optimization_enable = apa_param.GetParam().parallel_lat_opt_enable;
    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    parallel_optimization_enable =
        apa_world_ptr_->GetApaDataPtr()->simu_param.is_path_optimization;
    cilqr_optimization_enable =
        apa_world_ptr_->GetApaDataPtr()->simu_param.is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (parallel_optimization_enable && is_use_optimizer) {
    std::cout << "------------------------ lateral path optimization "
                 "------------------------"
              << std::endl;

    std::cout << "frame_.gear_command_= "
              << static_cast<int>(frame_.gear_command) << std::endl;

    std::cout << "origin path size= " << planner_output.path_point_vec.size()
              << std::endl;

    LateralPathOptimizer::Parameter param;
    param.sample_ds = apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
    param.q_ref_xy = apa_world_ptr_->GetApaDataPtr()->simu_param.q_ref_xy;
    param.q_ref_theta = apa_world_ptr_->GetApaDataPtr()->simu_param.q_ref_theta;
    param.q_terminal_xy =
        apa_world_ptr_->GetApaDataPtr()->simu_param.q_terminal_xy;
    param.q_terminal_theta =
        apa_world_ptr_->GetApaDataPtr()->simu_param.q_terminal_theta;
    param.q_k = apa_world_ptr_->GetApaDataPtr()->simu_param.q_k;
    param.q_u = apa_world_ptr_->GetApaDataPtr()->simu_param.q_u;
    param.q_k_bound = apa_world_ptr_->GetApaDataPtr()->simu_param.q_k_bound;
    param.q_u_bound = apa_world_ptr_->GetApaDataPtr()->simu_param.q_u_bound;

    apa_world_ptr_->GetLateralPathOptimizerPtr()->Init(
        cilqr_optimization_enable);

    apa_world_ptr_->GetLateralPathOptimizerPtr()->SetParam(param);

    auto time_start = IflyTime::Now_ms();
    apa_world_ptr_->GetLateralPathOptimizerPtr()->Update(
        planner_output.path_point_vec, frame_.gear_command);

    auto time_end = IflyTime::Now_ms();
    lat_path_opt_cost_time_ms = time_end - time_start;

    const auto& optimized_path_vec =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputPathVec();

    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(optimized_path_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : optimized_path_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }

    const auto plan_debug_info =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

    std::cout << "lat_path_opt_cost_time_ms = " << lat_path_opt_cost_time_ms
              << std::endl;

    std::cout << "terminal point error = "
              << plan_debug_info.terminal_pos_error() << std::endl;

    std::cout << "terminal heading error = "
              << plan_debug_info.terminal_heading_error() << std::endl;
  } else {
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(
        planner_output.path_point_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : planner_output.path_point_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
  }

  JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  std::cout << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size() << std::endl;

  return plan_result;
}

const bool ParallelParkInPlanner::CheckSegCompleted() {
  frame_.is_replan_by_uss = false;

  bool is_seg_complete = false;
  if (frame_.spline_success) {
    const auto min_remain_dist =
        std::min(frame_.remain_dist_uss, frame_.remain_dist);

    if (min_remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
      frame_.is_replan_by_uss = (frame_.remain_dist_uss < frame_.remain_dist);

      if (!frame_.is_replan_by_uss) {
        std::cout << "close to target!\n";
        is_seg_complete = true;
      } else {
        std::cout << "close to obstacle by uss!\n";
        // uss distance may not be accurate, so wait for a period of time
        if (frame_.stuck_time >
            apa_param.GetParam().uss_stuck_replan_wait_time) {
          is_seg_complete = true;
        }
      }
    }
  }

  return is_seg_complete;
}

const bool ParallelParkInPlanner::CheckReplan() {
  if (frame_.is_replan_first == true ||
      apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan) {
    DEBUG_PRINT("first plan");
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  if (CheckSegCompleted()) {
    DEBUG_PRINT("replan by current segment completed!");
    frame_.replan_reason = SEG_COMPLETED_PATH;
    if (frame_.is_replan_by_uss) {
      frame_.replan_reason = SEG_COMPLETED_USS;
    }
    return true;
  }

  if (frame_.stuck_time > apa_param.GetParam().stuck_replan_time) {
    DEBUG_PRINT("replan by stuck!");
    frame_.replan_reason = STUCKED;
    return true;
  }

  // Todo: maybe CheckDynamicUpdate
  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool ParallelParkInPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const Eigen::Vector2d rear_bumper_center =
      ego_slot_info.ego_pos_slot - apa_param.GetParam().rear_overhanging *
                                       ego_slot_info.ego_heading_slot_vec;

  const Eigen::Vector2d front_bumper_center =
      ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
                                    apa_param.GetParam().front_overhanging) *
                                       ego_slot_info.ego_heading_slot_vec;

  const bool lon_condition_1 = std::fabs(ego_slot_info.terminal_err.pos.x()) <
                               apa_param.GetParam().finish_parallel_lon_err;
  const bool lon_condition_2 =
      rear_bumper_center.x() >= apa_param.GetParam().finish_parallel_lon_err &&
      front_bumper_center.x() <=
          ego_slot_info.slot_length -
              apa_param.GetParam().finish_parallel_lon_err;

  DEBUG_PRINT("terminal x error = " << ego_slot_info.terminal_err.pos.x());

  const bool lon_condition = lon_condition_1 || lon_condition_2;
  DEBUG_PRINT("lon_condition = " << lon_condition);
  if (lon_condition) {
    DEBUG_PRINT("lon rac condition = " << lon_condition_1);
    DEBUG_PRINT("lon overhaing contidion =" << lon_condition_2);
  }

  const bool heading_condition =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_parallel_heading_err / 57.3;

  DEBUG_PRINT(
      "terminal heading error = " << ego_slot_info.terminal_err.heading * 57.3);
  DEBUG_PRINT("heading_condition = " << heading_condition);

  const bool lat_condition_1 = std::fabs(ego_slot_info.terminal_err.pos.y()) <=
                               apa_param.GetParam().finish_parallel_lat_rac_err;

  // lat condition 2, keep both outer wheels in slot
  const double side_sgn =
      t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(ego_slot_info.ego_pos_slot, ego_slot_info.ego_heading_slot);

  const auto& front_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(apa_param.GetParam().wheel_base,
                      0.5 * apa_param.GetParam().car_width * side_sgn));

  const auto& rear_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(0.0, 0.5 * apa_param.GetParam().car_width * side_sgn));

  const double slot_outer_pt_y = 0.5 * ego_slot_info.slot_width * side_sgn;
  bool lat_condition_2 = false;
  if (side_sgn > 0.0) {
    const double wheel_limit_y =
        slot_outer_pt_y - apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() <= wheel_limit_y) &&
                      (front_out_wheel.y() <= wheel_limit_y);
  } else {
    const double wheel_limit_y =
        slot_outer_pt_y + apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() >= wheel_limit_y) &&
                      (front_out_wheel.y() >= wheel_limit_y);
  }
  const bool lat_condition = lat_condition_1 || lat_condition_2;

  DEBUG_PRINT("terminal y error = " << ego_slot_info.terminal_err.pos.y());
  DEBUG_PRINT("lat condition  = " << lat_condition);
  if (lat_condition) {
    if (lat_condition_1) {
      DEBUG_PRINT("lat y err = "
                  << ego_slot_info.terminal_err.pos.y() << " < "
                  << apa_param.GetParam().finish_parallel_lat_rac_err);
    } else {
      DEBUG_PRINT("ego outer wheel are both in slot!");
    }
  }

  const bool static_condition =
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag;

  DEBUG_PRINT("static_condition = " << static_condition);

  return lon_condition && lat_condition && heading_condition &&
         static_condition;
}

const double ParallelParkInPlanner::CalcSlotOccupiedRatio(
    const Eigen::Vector2d& terminal_err, const double slot_width,
    const bool is_right_side) const {
  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(terminal_err.x(), -3.0, 4.0)) {
    const double y_err_ratio = terminal_err.y() / (0.5 * slot_width);

    if (is_right_side) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  return slot_occupied_ratio;
}

void ParallelParkInPlanner::Log() const {
  const auto& l2g_tf = frame_.ego_slot_info.l2g_tf;
  const auto p0_g = l2g_tf.GetPos(t_lane_.obs_pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.obs_pt_inside);
  const auto pt_g = l2g_tf.GetPos(t_lane_.pt_terminal_pos);

  std::cout << "p0_g = " << p0_g.transpose() << std::endl;

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("slot_side", t_lane_.slot_side)

  DEBUG_PRINT("obs p out = " << p0_g.transpose());
  DEBUG_PRINT("obs p in = " << p1_g.transpose());

  std::vector<double> obstaclesX;
  std::vector<double> obstaclesY;
  for (const auto& obs_pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    for (const auto& obstacle : obs_pair.second) {
      const auto tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }
  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

  std::vector<double> slot_corner_X;
  slot_corner_X.clear();
  slot_corner_X.reserve(frame_.ego_slot_info.slot_corner.size());
  std::vector<double> slot_corner_Y;
  slot_corner_Y.clear();
  slot_corner_Y.reserve(frame_.ego_slot_info.slot_corner.size());
  for (const auto& corner : frame_.ego_slot_info.slot_corner) {
    slot_corner_X.emplace_back(corner.x());
    slot_corner_Y.emplace_back(corner.y());
  }

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 2)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x",
                   frame_.ego_slot_info.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y",
                   frame_.ego_slot_info.terminal_err.pos.y())
  JSON_DEBUG_VALUE("terminal_error_heading",
                   frame_.ego_slot_info.terminal_err.heading)

  JSON_DEBUG_VALUE("is_replan", frame_.is_replan)
  JSON_DEBUG_VALUE("is_finished", frame_.is_finished)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("is_replan_by_uss", frame_.is_replan_by_uss)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("gear_change_count", frame_.gear_change_count)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist)
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_uss)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("ego_heading_slot", frame_.ego_slot_info.ego_heading_slot)

  JSON_DEBUG_VALUE("selected_slot_id", frame_.ego_slot_info.selected_slot_id)
  JSON_DEBUG_VALUE("slot_length", frame_.ego_slot_info.slot_length)
  JSON_DEBUG_VALUE("slot_width", frame_.ego_slot_info.slot_width)

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   frame_.ego_slot_info.slot_origin_pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   frame_.ego_slot_info.slot_origin_pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   frame_.ego_slot_info.slot_origin_heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   frame_.ego_slot_info.slot_occupied_ratio)

  std::vector<double> target_ego_pos_slot = {
      frame_.ego_slot_info.target_ego_pos_slot.x(),
      frame_.ego_slot_info.target_ego_pos_slot.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = parallel_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

  // lateral optimization
  const auto plan_debug_info =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

  if (plan_debug_info.has_terminal_pos_error()) {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error",
                     plan_debug_info.terminal_pos_error())
    JSON_DEBUG_VALUE("optimization_terminal_heading_error",
                     plan_debug_info.terminal_heading_error())
  } else {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error", 0.0)
    JSON_DEBUG_VALUE("optimization_terminal_heading_error", 0.0)
  }
}

}  // namespace apa_planner
}  // namespace planning
