#include "perpendicular_park_in_planner.h"

#include <bits/stdint-uintn.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "Eigen/src/Core/util/Constants.h"
#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "fem_pos_deviation_smoother_config.pb.h"
#include "func_state_machine.pb.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "perpendicular_path_planner.h"
#include "slot_management_info.pb.h"

namespace planning {
namespace apa_planner {
void PerpendicularInPlanner::Reset() {
  frame_.Reset();
  t_lane_.Reset();
}

void PerpendicularInPlanner::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED ||
             status == PARKING_PAUSED) {
    frame_.plan_stm.path_plan_success = true;
  }

  frame_.plan_stm.planning_status = status;
}

void PerpendicularInPlanner::Update() {
  // run plan core
  PlanCore();

  // generate planning output
  GenPlanningOutput();

  // log json debug
  Log();
}

void PerpendicularInPlanner::PlanCore() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (!simu_param_.force_plan && CheckPlanSkip()) {
    return;
  }

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
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
  if (simu_param_.force_plan || CheckReplan()) {
    std::cout << "replan is required!" << std::endl;

    // UpdateSlotRealtime();
    frame_.ego_slot_info.first_fix_limiter = true;

    GenTlane();

    GenObstacles();

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

  // check planning status
  std::cout << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status)
            << std::endl;
}

const bool PerpendicularInPlanner::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasurementsPtr();
  const auto slot_manager_ptr_ = apa_world_ptr_->GetSlotManagerPtr();

  auto& ego_slot_info = frame_.ego_slot_info;

  if (!frame_.is_fix_slot) {
    ego_slot_info.target_managed_slot =
        slot_manager_ptr_->GetEgoSlotInfo().select_slot_filter;

    const auto& slot_points =
        ego_slot_info.target_managed_slot.corner_points().corner_point();
    std::vector<Eigen::Vector2d> pt;
    pt.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      pt[i] << slot_points[i].x(), slot_points[i].y();
      if (is_simulation_ && simu_param_.target_managed_slot_x_vec.size() == 4) {
        pt[i] << simu_param_.target_managed_slot_x_vec[i],
            simu_param_.target_managed_slot_y_vec[i];
      }
    }
    ego_slot_info.slot_corner = pt;

    const auto pM01 = 0.5 * (pt[0] + pt[1]);
    const auto pM23 = 0.5 * (pt[2] + pt[3]);
    const auto n = (pM01 - pM23).normalized();
    ego_slot_info.slot_origin_pos =
        pM01 - apa_param.GetParam().normal_slot_length * n;

    ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
    ego_slot_info.slot_origin_heading_vec = n;
    ego_slot_info.slot_length = apa_param.GetParam().normal_slot_length;
    ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

    ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);
  }

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measures_ptr->pos_ego);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading_ego);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.limiter = slot_manager_ptr_->GetEgoSlotInfo().limiter;
  if (is_simulation_ && simu_param_.target_managed_limiter_x_vec.size() == 2) {
    ego_slot_info.limiter.first << simu_param_.target_managed_limiter_x_vec[0],
        simu_param_.target_managed_limiter_y_vec[0];
    ego_slot_info.limiter.first =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.first);
    ego_slot_info.limiter.second << simu_param_.target_managed_limiter_x_vec[1],
        simu_param_.target_managed_limiter_y_vec[1];
    ego_slot_info.limiter.second =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.second);
  }

  ego_slot_info.limiter_corner.clear();
  ego_slot_info.limiter_corner.reserve(2);
  ego_slot_info.limiter_corner.emplace_back(ego_slot_info.limiter.first);
  ego_slot_info.limiter_corner.emplace_back(ego_slot_info.limiter.second);

  // cal target pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) /
             2.0,
      apa_param.GetParam().terminal_target_y;
  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  // cal terminal error
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal slot occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err / 57.3) {
    ego_slot_info.slot_occupied_ratio =
        pnc::mathlib::Clamp(1.0 - (ego_slot_info.terminal_err.pos.x() /
                                   apa_param.GetParam().normal_slot_length),
                            0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  // trim path according to limiter
  if (!frame_.is_replan_first &&
      perpendicular_path_planner_.GetOutput().is_last_path &&
      gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE &&
      ego_slot_info.first_fix_limiter) {
    const pnc::geometry_lib::LineSegment limiter_line(
        ego_slot_info.limiter.first, ego_slot_info.limiter.second);

    const auto dist_ego_limiter = pnc::geometry_lib::CalPoint2LineDist(
        ego_slot_info.ego_pos_slot, limiter_line);

    std::cout << "dist_ego_limiter = " << dist_ego_limiter << std::endl;

    if (dist_ego_limiter < apa_param.GetParam().car_to_limiter_dis) {
      std::cout << "should trim path according limiter\n";
      ego_slot_info.first_fix_limiter = false;
      PostProcessPathAccordingLimiter();
    }
  }

  // update stuck time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // fix slot
  if (ego_slot_info.slot_occupied_ratio >
          apa_param.GetParam().fix_slot_occupied_ratio &&
      !frame_.is_fix_slot &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag) {
    // apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
    frame_.is_fix_slot = true;
  }

  // calc slot side and init gear and init steer at first
  if (frame_.is_replan_first == true) {
    const auto pM01 =
        0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);
    const auto pM23 =
        0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);
    Eigen::Vector2d ego_to_slot_center_vec =
        0.5 * (pM01 + pM23) - measures_ptr->pos_ego;

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->heading_ego_vec,
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->heading_ego_vec,
            ego_slot_info.slot_origin_heading_vec);

    // judge slot side via slot center and heading
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      t_lane_.slot_side = SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      t_lane_.slot_side = SLOT_SIDE_LEFT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      t_lane_.slot_side = SLOT_SIDE_INVALID;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      std::cout << "calculate slot side error " << std::endl;
      // return false;
    }
  }

  std::cout << "slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;
  std::cout << "frame_.current_gear = " << static_cast<int>(frame_.current_gear)
            << std::endl;
  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  std::cout << "ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << "  ego_heading_slot = " << ego_slot_info.ego_heading_slot * 57.3
            << std::endl;

  std::cout << "ego_slot_info.limiter.first = "
            << ego_slot_info.limiter.first.transpose()
            << "  ego_slot_info.limiter.second = "
            << ego_slot_info.limiter.second.transpose() << std::endl;

  std::cout << "target_ego_pos_slot = "
            << ego_slot_info.target_ego_pos_slot.transpose()
            << "  target_ego_heading_slot = "
            << ego_slot_info.target_ego_heading_slot * 57.3 << std::endl;

  std::cout << "terminal x error= " << ego_slot_info.terminal_err.pos.x()
            << std::endl;
  std::cout << "terminal y error= " << ego_slot_info.terminal_err.pos.y()
            << std::endl;
  std::cout << "terminal heading error= "
            << ego_slot_info.terminal_err.heading * 57.3 << std::endl;

  std::cout << "slot_occupied_ratio = " << ego_slot_info.slot_occupied_ratio
            << std::endl;

  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;
  std::cout << "stuck_time = " << frame_.stuck_time << " s\n";
  std::cout << "static_flag = "
            << apa_world_ptr_->GetMeasurementsPtr()->static_flag << "\n";

  return true;
}

void PerpendicularInPlanner::GenTlane() {
  const auto measure = apa_world_ptr_->GetMeasurementsPtr();
  const auto& ego_slot_info = frame_.ego_slot_info;
  const auto& target_corner_pts =
      ego_slot_info.target_managed_slot.corner_points();

  Eigen::Vector2d target_corner0(target_corner_pts.corner_point(0).x(),
                                 target_corner_pts.corner_point(0).y());

  Eigen::Vector2d target_corner1(target_corner_pts.corner_point(1).x(),
                                 target_corner_pts.corner_point(1).y());

  bool corner_0_side_occupied = true;
  bool corner_1_side_occupied = true;
  size_t nearby_slot_nums = 0;

  const auto& slot_info_vec =
      apa_world_ptr_->GetSlotManagerPtr()->GetOutput().slot_info_vec();

  // Check if there are slot on both sides of the selected slot
  // if so, determine if they are occupied
  for (const auto& managed_slot : slot_info_vec) {
    Eigen::Vector2d managed_corner0(
        managed_slot.corner_points().corner_point(0).x(),
        managed_slot.corner_points().corner_point(0).y());

    Eigen::Vector2d managed_corner1(
        managed_slot.corner_points().corner_point(1).x(),
        managed_slot.corner_points().corner_point(1).y());

    if ((target_corner0 - managed_corner1).norm() <
        apa_param.GetParam().nearby_slot_corner_dist) {
      nearby_slot_nums++;
      corner_0_side_occupied = managed_slot.is_occupied();
    }

    if ((target_corner1 - managed_corner0).norm() <
        apa_param.GetParam().nearby_slot_corner_dist) {
      nearby_slot_nums++;
      corner_1_side_occupied = managed_slot.is_occupied();
    }
    if (nearby_slot_nums == 2) {
      break;
    }
  }

  if (apa_param.GetParam().force_both_side_occupied) {
    corner_1_side_occupied = true;
    corner_0_side_occupied = true;
  }

  std::cout << "corner 0 side slot occupied = " << corner_0_side_occupied
            << std::endl;

  std::cout << "corner 1 side slot occupied = " << corner_1_side_occupied
            << std::endl;

  // order of parking slot on the left side is inconsistent with the right side
  // 0 2 is on the right side, 1 3 is on the left side, left +, right -
  // 0 1 is outter, 2 3 is inner
  const auto& slot_side = t_lane_.slot_side;

  Eigen::Vector2d corner0_slot(ego_slot_info.slot_length,
                               -0.5 * ego_slot_info.slot_width);

  Eigen::Vector2d corner1_slot(ego_slot_info.slot_length,
                               0.5 * ego_slot_info.slot_width);

  if (slot_side == SLOT_SIDE_RIGHT) {
    // right corner is inside, left corner is outside
    if (corner_0_side_occupied) {
      // right side slot is occupied
      t_lane_.pt_inside =
          corner0_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_pt_inside_dx,
                          apa_param.GetParam().occupied_pt_inside_dy);
    } else {
      // right side slot is free
      t_lane_.pt_inside =
          corner0_slot +
          Eigen::Vector2d(-apa_param.GetParam().vacant_pt_inside_dx,
                          -apa_param.GetParam().vacant_pt_inside_dy);
    }
    if (corner_1_side_occupied) {
      // left side slot is occupied
      t_lane_.pt_outside =
          corner1_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_pt_outside_dx,
                          -apa_param.GetParam().occupied_pt_outside_dy);
    } else {
      // left side slot is free
      t_lane_.pt_outside =
          corner1_slot +
          Eigen::Vector2d(-apa_param.GetParam().vacant_pt_outside_dx,
                          apa_param.GetParam().vacant_pt_outside_dy);
    }
  } else if (slot_side == SLOT_SIDE_LEFT) {
    // left corner is inside, right corner is outside
    if (corner_0_side_occupied) {
      // right side slot is occupied
      t_lane_.pt_outside =
          corner0_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_pt_outside_dx,
                          apa_param.GetParam().occupied_pt_outside_dy);
    } else {
      // right side slot is free
      t_lane_.pt_outside =
          corner0_slot +
          Eigen::Vector2d(-apa_param.GetParam().vacant_pt_outside_dx,
                          -apa_param.GetParam().vacant_pt_outside_dy);
    }
    if (corner_1_side_occupied) {
      // left side slot is occupied
      t_lane_.pt_inside =
          corner1_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_pt_inside_dx,
                          -apa_param.GetParam().occupied_pt_inside_dy);
    } else {
      // left side slot is free
      t_lane_.pt_inside =
          corner1_slot +
          Eigen::Vector2d(-apa_param.GetParam().vacant_pt_inside_dx,
                          apa_param.GetParam().vacant_pt_inside_dy);
    }
  }

  const double car_width =
      apa_param.GetParam().car_width + 2.0 * apa_param.GetParam().mirror_width;

  double tlane_width =
      std::fabs(t_lane_.pt_outside.y() - t_lane_.pt_inside.y());

  std::cout << "car_width = " << car_width << "  tlane_width = " << tlane_width
            << std::endl;
  const double threshold = apa_param.GetParam().width_threshold;
  if (tlane_width < car_width + threshold) {
    tlane_width = car_width + threshold;
    if (slot_side == SLOT_SIDE_RIGHT) {
      t_lane_.pt_outside.y() = tlane_width / 2.0;
      t_lane_.pt_inside.y() = -tlane_width / 2.0;
    } else if (slot_side == SLOT_SIDE_LEFT) {
      t_lane_.pt_outside.y() = -tlane_width / 2.0;
      t_lane_.pt_inside.y() = tlane_width / 2.0;
    }
    std::cout << "t_lane width should be extended, extended to " << tlane_width
              << std::endl;
  }

  t_lane_.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  t_lane_.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  std::cout << "-- t_lane --" << std::endl;
  std::cout << "pt_outside = " << t_lane_.pt_outside.transpose() << std::endl;
  std::cout << "pt_inside = " << t_lane_.pt_inside.transpose() << std::endl;
  std::cout << "pt_terminal_pos = " << t_lane_.pt_terminal_pos.transpose()
            << std::endl;
}

void PerpendicularInPlanner::GenObstacles() {
  // set obstacles
  double channel_width = apa_param.GetParam().channel_width;

  double channel_length = apa_param.GetParam().channel_length;
  // double slot_length = input_.tlane.slot_length;
  if (t_lane_.pt_inside.y() < t_lane_.pt_outside.y()) {
    // right side slot
    channel_length = -channel_length;
  }
  double p0_x = t_lane_.pt_outside.x();
  double p0_y = t_lane_.pt_outside.y();
  double p1_x = t_lane_.pt_inside.x();
  double p1_y = t_lane_.pt_inside.y();

  Eigen::Vector2d A(channel_width, p0_y);
  Eigen::Vector2d B(channel_width, p0_y + channel_length);

  Eigen::Vector2d C(p1_x, p0_y + channel_length);
  Eigen::Vector2d D(p1_x, p1_y);
  Eigen::Vector2d E(p0_x - apa_param.GetParam().normal_slot_length, p1_y);
  Eigen::Vector2d F(p0_x - apa_param.GetParam().normal_slot_length, p0_y);
  Eigen::Vector2d G(p0_x, p0_y);

  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_line.SetPoints(A, B);
  channel_line_vec.emplace_back(channel_line);

  channel_line.SetPoints(B, C);
  channel_line_vec.emplace_back(channel_line);

  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  // tlane_line.SetPoints(D, E);
  // tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(G, F);
  tlane_line_vec.emplace_back(tlane_line);

  // line.SetPoints(C, D);
  // line_vec.emplace_back(line);

  // line.SetPoints(D, E);
  // line_vec.emplace_back(line);

  // line.SetPoints(D, E);
  // line_vec.emplace_back(line);

  // line.SetPoints(F, G);
  // line_vec.emplace_back(line);

  double ds = apa_param.GetParam().obstacle_ds;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  channel_obstacle_vec.clear();
  channel_obstacle_vec.reserve(100);
  for (const auto& line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    channel_obstacle_vec.reserve(channel_obstacle_vec.size() +
                                 point_set.size());
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(channel_obstacle_vec);

  // tmp method, should modify
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(100);
  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    tlane_obstacle_vec.reserve(tlane_obstacle_vec.size() + point_set.size());
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(frame_.ego_slot_info.ego_pos_slot,
               frame_.ego_slot_info.ego_heading_slot);
  for (const auto& obs_pos : tlane_obstacle_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(obs_pos,
                                                                    ego_pose)) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(obs_pos);
    }
  }
  // if (apa_world_ptr_->GetCollisionDetectorPtr()->CalMinDistObs2Car(
  //         t_lane_.pt_outside, ego_pose) >
  //     apa_param.GetParam().max_obs2car_dist) {
  //   std::cout << "add pt_outside as obstacle\n";
  //   obstacle_vec.emplace_back(t_lane_.pt_outside);
  // }

  // if ((t_lane_.pt_outside - frame_.ego_slot_info.ego_pos_slot).norm() > 2.2)
  // {
  //   obstacle_vec.emplace_back(t_lane_.pt_outside);
  // }

  // if ((t_lane_.pt_inside - frame_.ego_slot_info.ego_pos_slot).norm() > 2.2) {
  //   obstacle_vec.emplace_back(t_lane_.pt_inside);
  // }
}

const uint8_t PerpendicularInPlanner::PathPlanOnce() {
  std::cout << "-------------- PathPlanOnce --------------" << std::endl;
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;
  PerpendicularPathPlanner::Input path_planner_input;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = t_lane_;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;
  path_planner_input.sample_ds = simu_param_.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_replan_second = frame_.is_replan_second;
  path_planner_input.is_replan_dynamic = frame_.is_replan_dynamic;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  if (frame_.replan_reason == DYNAMIC &&
      gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    std::cout << "dynamic replan, gear should be reverse\n";
    path_planner_input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  perpendicular_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success = perpendicular_path_planner_.Update(
      apa_world_ptr_->GetCollisionDetectorPtr());

  uint8_t plan_result = 0;
  if (!path_plan_success && !is_simulation_ && !frame_.is_replan_dynamic) {
    std::cout << "path plan fail\n";
    plan_result = PathPlannerResult::PLAN_FAILED;
    return plan_result;
  }
  plan_result = PathPlannerResult::PLAN_UPDATE;

  perpendicular_path_planner_.SetCurrentPathSegIndex();

  perpendicular_path_planner_.SetLineSegmentHeading();

  perpendicular_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
      apa_param.GetParam().path_extend_distance);

  perpendicular_path_planner_.SampleCurrentPathSeg();

  perpendicular_path_planner_.PrintOutputSegmentsInfo();

  const auto& planner_output = perpendicular_path_planner_.GetOutput();

  const bool gear_steer_shift =
      (frame_.is_replan_first && planner_output.gear_shift) ||
      (frame_.is_replan_second && planner_output.gear_shift) ||
      (!frame_.is_replan_first && !frame_.is_replan_second &&
       frame_.replan_reason != DYNAMIC);

  if (gear_steer_shift) {
    std::cout << "next plan should shift gear\n";
    // set current arc steer
    if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else {
      std::cout << "fault ref_arc_steer state!" << std::endl;
      return false;
    }

    // set current gear
    if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    } else {
      std::cout << "fault ref_gear state!" << std::endl;
      return false;
    }
  }

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    frame_.is_replan_second = true;
  } else {
    if (frame_.is_replan_second) {
      frame_.is_replan_second = false;
    }
  }

  if (!path_plan_success) {
    return plan_result;
  }

  gear_command_ = planner_output.current_gear;

  // lateral path optimization
  auto lateral_path_optimization_enable = false;

  if (!is_simulation_) {
    lateral_path_optimization_enable =
        apa_param.GetParam().lateral_path_optimization_enable;
  } else {
    lateral_path_optimization_enable = simu_param_.is_path_optimization;
  }

  if (lateral_path_optimization_enable) {
    std::cout << "------------------------ lateral path optimization "
                 "------------------------"
              << std::endl;
    std::cout << "gear_command_= " << static_cast<int>(gear_command_)
              << std::endl;
    std::cout << "origin path size= " << planner_output.path_point_vec.size()
              << std::endl;

    LateralPathOptimizer::Parameter param;
    param.sample_ds = simu_param_.sample_ds;
    param.q_ref_xy = simu_param_.q_ref_xy;
    param.q_ref_theta = simu_param_.q_ref_theta;
    param.q_terminal_xy = simu_param_.q_terminal_xy;
    param.q_terminal_theta = simu_param_.q_terminal_theta;
    param.q_k = simu_param_.q_k;
    param.q_u = simu_param_.q_u;
    param.q_k_bound = simu_param_.q_k_bound;
    param.q_u_bound = simu_param_.q_u_bound;

    lateral_path_optimizer_ptr_->SetParam(param);

    lateral_path_optimizer_ptr_->Update(planner_output.path_point_vec,
                                        gear_command_);

    const auto& optimized_path_vec =
        lateral_path_optimizer_ptr_->GetOutputPathVec();

    std::cout << "optimization path size = " << optimized_path_vec.size()
              << std::endl;
    std::cout << "terminal pos error = "
              << (optimized_path_vec.back().pos -
                  planner_output.path_point_vec.back().pos)
                     .norm()
              << std::endl;

    std::cout << "terminal heading error = "
              << (optimized_path_vec.back().heading -
                  planner_output.path_point_vec.back().heading) *
                     57.3
              << std::endl;

    // TODO: longitudinal path optimization
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(optimized_path_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : optimized_path_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
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

  std::cout << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size() << std::endl;

  return plan_result;
}

const bool PerpendicularInPlanner::CheckSegCompleted() {
  bool is_seg_complete = false;
  if (frame_.spline_success) {
    const auto min_remain_dist =
        std::min(frame_.remain_dist_uss, frame_.remain_dist);

    if (min_remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetMeasurementsPtr()->static_flag) {
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

void PerpendicularInPlanner::UpdateSlotRealtime() {
  bool update_slot = false;

  const size_t replan_reaon = frame_.replan_reason;
  if (replan_reaon == FIRST_PLAN || replan_reaon == DYNAMIC) {
    update_slot = true;
  }

  if (replan_reaon == SEG_COMPLETED_USS) {
    if (frame_.ego_slot_info.slot_occupied_ratio >
        apa_param.GetParam().uss_slot_occupied_ratio) {
      update_slot = true;
    }
  }

  if (replan_reaon == SEG_COMPLETED_PATH) {
    if (std::fabs(frame_.ego_slot_info.ego_heading_slot) <
            apa_param.GetParam().path_heading_slot / 57.3 &&
        frame_.ego_slot_info.terminal_err.pos.norm() <
            apa_param.GetParam().path_pos_err) {
      update_slot = true;
    }
  }

  if (update_slot && frame_.need_update_slot) {
    std::cout << " real time update slot\n";
    apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
    apa_world_ptr_->Update();
    UpdateEgoSlotInfo();
  }

  if (frame_.ego_slot_info.slot_occupied_ratio >
      apa_param.GetParam().last_update_slot_occupied_ratio) {
    frame_.need_update_slot = false;
  }
}

const bool PerpendicularInPlanner::CheckDynamicUpdate() {
  bool update_flag = false;
  // first update, conditions are relatively relaxed
  if (frame_.dynamic_replan_count == 0) {
    bool condition_1 = std::fabs(frame_.ego_slot_info.terminal_err.pos.y()) <
                           apa_param.GetParam().pose_y_err &&
                       std::fabs(frame_.ego_slot_info.terminal_err.heading) <
                           apa_param.GetParam().pose_heading_err / 57.3;

    bool condition_2 = frame_.ego_slot_info.slot_occupied_ratio >
                       apa_param.GetParam().pose_slot_occupied_ratio;

    if (condition_1 && condition_2) {
      frame_.dynamic_replan_count++;
      update_flag = true;
    }
  }

  // second update
  if (frame_.dynamic_replan_count == 1) {
    bool condition_1 = std::fabs(frame_.ego_slot_info.terminal_err.pos.y()) >
                           apa_param.GetParam().max_y_err_2 ||
                       std::fabs(frame_.ego_slot_info.terminal_err.heading) >
                           apa_param.GetParam().max_heading_err_2 / 57.3;

    bool condition_2 = frame_.ego_slot_info.slot_occupied_ratio >
                       apa_param.GetParam().pose_slot_occupied_ratio_2;

    bool condition_3 =
        frame_.remain_dist > apa_param.GetParam().pose_min_remain_dis;

    if (condition_1 && condition_2 && condition_3) {
      frame_.dynamic_replan_count++;
      update_flag = true;
    }
  }

  // third update
  if (frame_.dynamic_replan_count == 2) {
    bool condition_1 = std::fabs(frame_.ego_slot_info.terminal_err.pos.y()) >
                           apa_param.GetParam().max_y_err_3 ||
                       std::fabs(frame_.ego_slot_info.terminal_err.heading) >
                           apa_param.GetParam().max_heading_err_3 / 57.3;

    bool condition_2 = frame_.ego_slot_info.slot_occupied_ratio >
                       apa_param.GetParam().pose_slot_occupied_ratio_3;

    bool condition_3 =
        frame_.remain_dist > apa_param.GetParam().pose_min_remain_dis;

    if (condition_1 && condition_2 && condition_3) {
      frame_.dynamic_replan_count++;
      update_flag = true;
    }
  }

  update_flag =
      (update_flag && gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE &&
       !apa_world_ptr_->GetMeasurementsPtr()->static_flag);

  frame_.is_replan_dynamic = update_flag;

  return update_flag;
}

const bool PerpendicularInPlanner::CheckReplan() {
  if (frame_.is_replan_first == true) {
    std::cout << "first plan" << std::endl;
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  frame_.is_replan_by_uss = false;
  frame_.is_replan_dynamic = false;

  if (CheckSegCompleted()) {
    std::cout << "replan by current segment completed!" << std::endl;
    frame_.replan_reason = SEG_COMPLETED_PATH;
    if (frame_.is_replan_by_uss) {
      frame_.replan_reason = SEG_COMPLETED_USS;
    }
    return true;
  }

  if (frame_.stuck_time > apa_param.GetParam().stuck_replan_time) {
    std::cout << "replan by stuck!" << std::endl;
    frame_.replan_reason = STUCKED;
    return true;
  }

  if (CheckDynamicUpdate()) {
    std::cout << "replan by dynamic!" << std::endl;
    frame_.replan_reason = DYNAMIC;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool PerpendicularInPlanner::CheckStuckFailed() {
  return frame_.stuck_time > apa_param.GetParam().stuck_failed_time;
}

const bool PerpendicularInPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool lon_condition =
      ego_slot_info.terminal_err.pos.x() < apa_param.GetParam().finish_lon_err;

  const double y1 = ego_slot_info.ego_pos_slot.y();
  const double y2 =
      (ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
                                     apa_param.GetParam().front_overhanging) *
                                        ego_slot_info.ego_heading_slot_vec)
          .y();

  const bool lat_condition_1 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err;

  const bool lat_condition_2 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err_strict &&
      std::fabs(y2) <= apa_param.GetParam().finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_heading_err / 57.3;

  const bool heading_condition_2 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) / 57.3;

  const bool lat_condition = (lat_condition_1 && heading_condition_1) ||
                             (lat_condition_2 && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetMeasurementsPtr()->static_flag;

  bool parking_finish = lon_condition && lat_condition && static_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  if (uss_obstacle_avoider_ptr->CheckIsDirectlyBehindUss()) {
    parking_finish =
        lat_condition && static_condition &&
        (frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist);
  }

  return parking_finish;
}

void PerpendicularInPlanner::GenPlanningOutput() {
  pnc::geometry_lib::PathPoint current_ego_pose(
      apa_world_ptr_->GetMeasurementsPtr()->pos_ego,
      apa_world_ptr_->GetMeasurementsPtr()->heading_ego);

  if (frame_.plan_stm.planning_status == PARKING_FINISHED) {
    SetFinishedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_FAILED) {
    SetFailedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
             frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
             frame_.plan_stm.planning_status == PARKING_RUNNING ||
             frame_.plan_stm.planning_status == PARKING_PAUSED) {
    GenPlanningPath();
  } else if (frame_.plan_stm.planning_status == PARKING_IDLE) {
    SetIdlePlanningOutput(planning_output_, current_ego_pose);
  }

  // std::cout << "planning_output:" << planning_output_.DebugString()
  //           << std::endl;
}

void PerpendicularInPlanner::GenPlanningPath() {
  planning_output_.Clear();
  planning_output_.mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);

  auto trajectory = planning_output_.mutable_trajectory();
  trajectory->set_available(true);

  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);

  for (const auto& global_point : current_path_point_global_vec_) {
    auto trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(global_point.pos.x());
    trajectory_point->set_y(global_point.pos.y());
    trajectory_point->set_heading_yaw(global_point.heading);
    trajectory_point->set_v(0.5);
  }

  // set target velocity to control as a limit
  const std::vector<double> ratio_tab = {0.0, 0.4, 0.8, 1.0};
  const std::vector<double> vel_limit_tab = {apa_param.GetParam().max_velocity,
                                             apa_param.GetParam().max_velocity,
                                             0.45, 0.35};
  const double vel_limit = pnc::mathlib::Interp1(
      ratio_tab, vel_limit_tab, frame_.ego_slot_info.slot_occupied_ratio);

  planning_output_.mutable_trajectory()
      ->mutable_target_reference()
      ->set_target_velocity(vel_limit);

  // send uss remain dist to control
  planning_output_.mutable_trajectory()
      ->mutable_trajectory_points(0)
      ->set_distance(frame_.remain_dist_uss);

  // send slot occupation ratio to control
  planning_output_.mutable_trajectory()
      ->mutable_trajectory_points(1)
      ->set_distance(frame_.ego_slot_info.slot_occupied_ratio);

  // set plan gear cmd
  auto gear_command = planning_output_.mutable_gear_command();
  gear_command->set_available(true);

  if (gear_command_ == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE);
  } else {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);
  }
}

void PerpendicularInPlanner::InitSimulation() {
  if (is_simulation_ && simu_param_.is_reset) {
    Reset();
  }
}

const bool PerpendicularInPlanner::CheckPlanSkip() const {
  if (frame_.plan_stm.planning_status == PARKING_FINISHED ||
      frame_.plan_stm.planning_status == PARKING_FAILED) {
    std::cout << "plan has been finished or failed, need reset" << std::endl;

    if (!is_simulation_) {
      apa_world_ptr_->GetSlotManagerPtr()->Reset();
    }

    return true;
  } else {
    return false;
  }
}

const bool PerpendicularInPlanner::CheckPaused() {
  if (apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_SUSPEND_ACTIVATE ||
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_SUSPEND_CLOSE) {
    return true;
  } else {
    return false;
  }
}

const double PerpendicularInPlanner::CalRemainDistFromPath() {
  double remain_dist = 5.01;

  if (frame_.is_replan_first) {
    return remain_dist;
  }

  if (frame_.spline_success) {
    double s_proj = 0.0;
    bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
        0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
        apa_world_ptr_->GetMeasurementsPtr()->pos_ego, frame_.x_s_spline,
        frame_.y_s_spline);

    if (success == true) {
      remain_dist = frame_.current_path_length - s_proj;

      std::cout << "remain_dist = " << remain_dist << "  s_proj = " << s_proj
                << "  current_path_length = " << frame_.current_path_length
                << std::endl;
    } else {
      std::cout << "remain_dist calculation error:input is error" << std::endl;
    }
  } else {
    std::cout << "remain_dist calculation error: path spline failed!"
              << std::endl;
  }

  return remain_dist;
}

const double PerpendicularInPlanner::CalRemainDistFromUss() {
  double remain_dist = 5.01;

  // if (frame_.is_replan_first) {
  //   return remain_dist;
  // }
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(&planning_output_,
                                   apa_world_ptr_->GetLocalViewPtr());

  const double safe_uss_remain_dist =
      (frame_.ego_slot_info.slot_occupied_ratio < 0.05)
          ? apa_param.GetParam().safe_uss_remain_dist_out_slot
          : apa_param.GetParam().safe_uss_remain_dist_in_slot;

  remain_dist = uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist -
                safe_uss_remain_dist;

  std::cout << "uss remain dist = " << remain_dist << std::endl;

  // remain_dist = 5.01;

  return remain_dist;
}

void PerpendicularInPlanner::UpdateRemainDist() {
  // 1. calculate remain dist according to plan path
  frame_.remain_dist = CalRemainDistFromPath();

  // 2.calculate remain dist uss according to uss
  frame_.remain_dist_uss = CalRemainDistFromUss();

  return;
}

const bool PerpendicularInPlanner::PostProcessPathAccordingLimiter() {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    std::cout << "error: origin_traj_size = " << origin_traj_size << std::endl;
    return false;
  }

  // cal proj point, need extend, add a point
  // if need extend according limiter, need to add another point
  const size_t traj_size = origin_traj_size + 2;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  std::vector<double> heading_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();
  heading_vec.clear();

  x_vec.reserve(traj_size);
  y_vec.reserve(traj_size);
  s_vec.reserve(traj_size);
  heading_vec.reserve(traj_size);

  const Eigen::Vector2d limiter_mid =
      (frame_.ego_slot_info.l2g_tf.GetPos(frame_.ego_slot_info.limiter.first) +
       frame_.ego_slot_info.l2g_tf.GetPos(
           frame_.ego_slot_info.limiter.second)) /
      2.0;

  double s_proj = 0.0;
  bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
      limiter_mid, frame_.x_s_spline, frame_.y_s_spline);
  if (!success) {
    std::cout << "path is err\n";
    return false;
  }
  if (s_proj < simu_param_.sample_ds * 1.5) {
    std::cout << "limiter s_proj is too small\n";
    return false;
  }
  double ds = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < current_path_point_global_vec_.size(); ++i) {
    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].pos.x() -
                          current_path_point_global_vec_[i - 1].pos.x(),
                      current_path_point_global_vec_[i].pos.y() -
                          current_path_point_global_vec_[i - 1].pos.y());
      s += std::max(ds, 1e-3);
    }
    if (s > s_proj) {
      std::cout << "path shoule be shorten because of limiter\n";
      break;
    }
    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);
    s_vec.emplace_back(s);
  }
  if (s < s_proj) {
    std::cout << "path shoule be extended because of limiter\n";
    x_vec.emplace_back(frame_.x_s_spline(s_proj));
    y_vec.emplace_back(frame_.y_s_spline(s_proj));
    heading_vec.emplace_back(heading_vec.back());
    s_vec.emplace_back(s_proj);
  }
  frame_.current_path_length = s_vec.back();
  const size_t N = x_vec.size();
  if (N < 2) {
    frame_.spline_success = false;
    std::cout << "error: no enough point = " << x_vec.size() << std::endl;
    return false;
  }
  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(N);
  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = 0; i < N; ++i) {
    path_point.Set(Eigen::Vector2d(x_vec[i], y_vec[i]), heading_vec[i]);
    current_path_point_global_vec_.emplace_back(path_point);
  }

  // need extend by cal proj point
  Eigen::Vector2d extended_point;
  success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      Eigen::Vector2d(x_vec[N - 2], y_vec[N - 2]),
      Eigen::Vector2d(x_vec[N - 1], y_vec[N - 1]), extended_point,
      frame_.path_extended_dist);

  if (!success) {
    frame_.spline_success = false;
    std::cout << "limit need extend fit line by spline error!" << std::endl;
    return false;
  }

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  heading_vec.emplace_back(heading_vec.back());
  s_vec.emplace_back(frame_.current_path_length + frame_.path_extended_dist);

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

const bool PerpendicularInPlanner::PostProcessPath() {
  size_t origin_trajectory_size = current_path_point_global_vec_.size();

  if (origin_trajectory_size < 2) {
    frame_.spline_success = false;
    std::cout << "error: origin_trajectory_size = " << origin_trajectory_size
              << std::endl;
    return false;
  }

  const size_t trajectory_size = origin_trajectory_size + 1;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();

  x_vec.resize(trajectory_size);
  y_vec.resize(trajectory_size);
  s_vec.resize(trajectory_size);

  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < origin_trajectory_size; ++i) {
    x_vec[i] = current_path_point_global_vec_[i].pos.x();
    y_vec[i] = current_path_point_global_vec_[i].pos.y();
    if (i == 0) {
      s_vec[i] = s;
    } else {
      ds = std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s += std::max(ds, 1e-3);
      s_vec[i] = s;
    }
  }

  frame_.current_path_length = s;

  // calculate the extended point and insert
  Eigen::Vector2d start_point(x_vec[origin_trajectory_size - 2],
                              y_vec[origin_trajectory_size - 2]);

  Eigen::Vector2d end_point(x_vec[origin_trajectory_size - 1],
                            y_vec[origin_trajectory_size - 1]);

  Eigen::Vector2d extended_point;

  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      start_point, end_point, extended_point, frame_.path_extended_dist);

  if (success == false) {
    frame_.spline_success = false;
    std::cout << "fit line by spline error!" << std::endl;
    return false;
  }

  x_vec[origin_trajectory_size] = extended_point.x();
  y_vec[origin_trajectory_size] = extended_point.y();

  s_vec[origin_trajectory_size] =
      s_vec[origin_trajectory_size - 1] + frame_.path_extended_dist;

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

void PerpendicularInPlanner::Log() const {
  const auto& l2g_tf = frame_.ego_slot_info.l2g_tf;
  const auto p0_g = l2g_tf.GetPos(t_lane_.pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.pt_inside);
  const auto pt_g = l2g_tf.GetPos(t_lane_.pt_terminal_pos);

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("slot_side", t_lane_.slot_side)

  std::vector<Eigen::Vector2d> obstacles =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetObstacles();

  std::vector<double> obstaclesX;
  obstaclesX.clear();
  obstaclesX.reserve(obstacles.size());
  std::vector<double> obstaclesY;
  obstaclesY.clear();
  obstaclesY.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    const auto tmp_obstacle = l2g_tf.GetPos(obstacle);
    obstaclesX.emplace_back(tmp_obstacle.x());
    obstaclesY.emplace_back(tmp_obstacle.y());
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

  std::vector<double> limiter_corner_X;
  limiter_corner_X.clear();
  limiter_corner_X.reserve(frame_.ego_slot_info.limiter_corner.size());
  std::vector<double> limiter_corner_Y;
  limiter_corner_Y.clear();
  limiter_corner_Y.reserve(frame_.ego_slot_info.limiter_corner.size());
  for (const auto& corner : frame_.ego_slot_info.limiter_corner) {
    const auto tmp_corner = l2g_tf.GetPos(corner);
    limiter_corner_X.emplace_back(tmp_corner.x());
    limiter_corner_Y.emplace_back(tmp_corner.y());
  }
  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

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
  JSON_DEBUG_VALUE(
      "car_static_timer_by_pos",
      apa_world_ptr_->GetMeasurementsPtr()->car_static_timer_by_pos)
  JSON_DEBUG_VALUE(
      "car_static_timer_by_vel",
      apa_world_ptr_->GetMeasurementsPtr()->car_static_timer_by_vel)
  JSON_DEBUG_VALUE("static_flag",
                   apa_world_ptr_->GetMeasurementsPtr()->static_flag)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("dynamic_replan_count", frame_.dynamic_replan_count)
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

  const auto& path_plan_output = perpendicular_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)
}

}  // namespace apa_planner
}  // namespace planning
