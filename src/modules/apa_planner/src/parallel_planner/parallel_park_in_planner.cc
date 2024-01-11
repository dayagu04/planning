#include "parallel_park_in_planner.h"

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine.pb.h"
#include "geometry_math.h"
#include "local_view.h"
#include "math_lib.h"

namespace planning {
namespace apa_planner {

static const bool kForceBothOccupied = true;
static const double kRearStopBuffer = 0.55;

void ParallelParInPlanner::Init() {
  // reset
  Reset();
}

void ParallelParInPlanner::Reset() {
  frame_.Reset();
  t_lane_.Reset();
}

void ParallelParInPlanner::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED) {
    frame_.plan_stm.path_plan_success = true;
  }

  frame_.plan_stm.planning_status = status;
}

void ParallelParInPlanner::Update() {
  std::cout << "---- apa_planner: PARALLEL_PARK_IN: Update() ---" << std::endl;

  // run plan core
  PlanCore();

  // generate planning output
  GenPlanningOutput();

  // log json debug
  Log();
}

void ParallelParInPlanner::PlanCore() {
  // init simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip() && simu_param_.force_plan == false) {
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
  if (CheckReplan() || simu_param_.force_plan) {
    std::cout << "replan is required!" << std::endl;

    // UpdateSlotRealtime();
    // frame_.ego_slot_info.first_fix_limiter = true;

    // generate t-lane
    GenTlane();

    // update obstacles
    UpdateObstacles();

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

  // check planning status
  std::cout << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status)
            << std::endl;
}

const bool ParallelParInPlanner::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasurementsPtr();

  auto& ego_slot_info = frame_.ego_slot_info;

  ego_slot_info.target_managed_slot.CopyFrom(measures_ptr->target_managed_slot);

  const auto& slot_points =
      ego_slot_info.target_managed_slot.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(slot_points.size());

  std::cout << "parallel slot points in slm :" << std::endl;
  for (int i = 0; i < slot_points.size(); i++) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
    std::cout << pt[i].transpose() << std::endl;
  }

  // calc slot side once at first
  if (frame_.is_replan_first == true) {
    const Eigen::Vector2d v_ego_to_pt3 = pt[3] - measures_ptr->pos_ego;

    const double cross_ego_to_pt3 = pnc::geometry_lib::GetCrossFromTwoVec2d(
        measures_ptr->heading_ego_vec, v_ego_to_pt3);

    // judge slot side via slot pt3
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_pt3 < 0.0) {
      t_lane_.slot_side = SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_pt3 > 0.0) {
      t_lane_.slot_side = SLOT_SIDE_LEFT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      t_lane_.slot_side = SLOT_SIDE_INVALID;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      std::cout << "calculate parallel slot side error " << std::endl;
      return false;
    }
  }
  const Eigen::Vector2d pM02 = 0.5 * (pt[0] + pt[2]);
  const Eigen::Vector2d pM13 = 0.5 * (pt[1] + pt[3]);
  Eigen::Vector2d n = Eigen::Vector2d::Zero();

  ego_slot_info.slot_length = (pt[0] - pt[1]).norm();
  ego_slot_info.slot_width = (pt[0] - pt[2]).norm();

  // note: slot points' order is corrected in slot management
  if (t_lane_.slot_side == SLOT_SIDE_RIGHT) {
    n = (pt[0] - pt[1]).normalized();
    ego_slot_info.slot_origin_pos = pM02 - ego_slot_info.slot_length * n;
  } else {
    n = -(pt[0] - pt[1]).normalized();
    ego_slot_info.slot_origin_pos = pM13 - ego_slot_info.slot_length * n;
  }

  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;

  std::cout << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;

  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measures_ptr->pos_ego);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading_ego);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // Todo: use limiter to determine the practical target pose

  // calc terminal pos
  ego_slot_info.target_ego_pos_slot
      << apa_param.GetParam().rear_overhanging + kRearStopBuffer,
      0.0;

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
  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;

  // calc slot occupied ratio
  // ego_slot_info.slot_occupied_ratio =
  //     apa_world_ptr_->GetSlotManagerPtr()->GetOccupiedRatio();
  const double slot_side_sgn =
      t_lane_.slot_side == SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  if (pnc::mathlib::IsInBound(ego_slot_info.terminal_err.pos.x(), 0.0, 3.0) &&
      std::fabs(ego_slot_info.ego_heading_slot) < 30.0 / 57.3) {
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - slot_side_sgn * (ego_slot_info.terminal_err.pos.y() /
                               ego_slot_info.slot_width / 2),
        0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  std::cout << "ego_slot_info.slot_occupied_ratio = "
            << ego_slot_info.slot_occupied_ratio << std::endl;

  // update stuck time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  return true;
}

void ParallelParInPlanner::UpdateObstacles() {
  // // set obstacles
  // std::vector<Eigen::Vector2d> obstacle_vec = {t_lane_.p0};

  // apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
  //     std::move(obstacle_vec));
}

void ParallelParInPlanner::GenTlane() {
  const auto measure = apa_world_ptr_->GetMeasurementsPtr();
  const auto& ego_slot_info = frame_.ego_slot_info;

  const auto& target_corner_pt =
      ego_slot_info.target_managed_slot.corner_points().corner_point();

  const Eigen::Vector2d target_corner0(target_corner_pt[0].x(),
                                       target_corner_pt[0].y());

  const Eigen::Vector2d target_corner1(target_corner_pt[1].x(),
                                       target_corner_pt[1].y());

  bool corner_0_side_occupied = true;
  bool corner_1_side_occupied = true;
  size_t nearby_slot_nums = 0;

  const auto& slot_info_vec =
      apa_world_ptr_->GetSlotManagerPtr()->GetOutput().slot_info_vec();

  // check if slots are occupied near the target slot.
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

  if (kForceBothOccupied) {
    corner_1_side_occupied = true;
    corner_0_side_occupied = true;
  }

  std::cout << "corner 0 side slot occupied = " << corner_0_side_occupied
            << std::endl;

  std::cout << "corner 1 side slot occupied = " << corner_1_side_occupied
            << std::endl;

  const double slot_side_sgn =
      t_lane_.slot_side == SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  const Eigen::Vector2d corner0_slot(
      ego_slot_info.slot_length,
      0.5 * ego_slot_info.slot_width * slot_side_sgn);

  const Eigen::Vector2d corner1_slot(
      0.0, 0.5 * ego_slot_info.slot_width * slot_side_sgn);

  t_lane_.pt_inside = corner1_slot;
  t_lane_.pt_outside = corner0_slot;
  t_lane_.pt_terminal = ego_slot_info.target_ego_pos_slot;

  std::cout << "-- t_lane --" << std::endl;
  std::cout << "pt_outside = " << t_lane_.pt_outside.transpose() << std::endl;
  std::cout << "pt_inside = " << t_lane_.pt_inside.transpose() << std::endl;
  std::cout << "pt_terminal = " << t_lane_.pt_terminal.transpose() << std::endl;
}

void ParallelParInPlanner::GenObstacles() {
  const double slot_side_sgn =
      (t_lane_.slot_side == SLOT_SIDE_RIGHT ? 1.0 : -1.0);

  double channel_length = 16.0;
  double channel_y = slot_side_sgn * (0.5 * frame_.ego_slot_info.slot_width +
                                      apa_param.GetParam().channel_width);

  // set obstacles
  Eigen::Vector2d A(t_lane_.pt_outside.x(), channel_y);
  Eigen::Vector2d B(t_lane_.pt_outside.x() + channel_length, channel_y);
  // Eigen::Vector2d C(B.x(), t_lane_.pt_inside.y());
  // Eigen::Vector2d D = t_lane_.pt_inside;

  // Eigen::Vector2d E(D.x(),
  //                   D.y() - slot_side_sgn * frame_.ego_slot_info.slot_width);

  // Eigen::Vector2d G = t_lane_.pt_outside;
  // Eigen::Vector2d F(G.x(), E.y());

  std::vector<pnc::geometry_lib::LineSegment> line_vec;
  pnc::geometry_lib::LineSegment line;
  line.SetPoints(A, B);
  line_vec.emplace_back(line);

  // line.SetPoints(B, C);
  // line_vec.emplace_back(line);

  // line.SetPoints(C, D);
  // line_vec.emplace_back(line);

  // line.SetPoints(D, E);
  // line_vec.emplace_back(line);

  // line.SetPoints(D, E);
  // line_vec.emplace_back(line);

  // line.SetPoints(F, G);
  // line_vec.emplace_back(line);

  double ds = 0.5;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> obstacle_vec;
  for (const auto& line : line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    obstacle_vec.reserve(obstacle_vec.size() + point_set.size());
    obstacle_vec.insert(obstacle_vec.end(), point_set.begin(), point_set.end());
  }

  // if ((t_lane_.pt_outside - frame_.ego_slot_info.ego_pos_slot).norm() > 2.2)
  // {
  //   obstacle_vec.emplace_back(t_lane_.pt_outside);
  // }

  // if ((t_lane_.pt_inside - frame_.ego_slot_info.ego_pos_slot).norm() > 2.2) {
  //   obstacle_vec.emplace_back(t_lane_.pt_inside);
  // }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(obstacle_vec);
}

const uint8_t ParallelParInPlanner::PathPlanOnce() {
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;

  ParallelPathPlanner::Input path_planner_input;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = t_lane_;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;
  path_planner_input.sample_ds = simu_param_.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;

  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  parallel_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success =
      parallel_path_planner_.Update(apa_world_ptr_->GetCollisionDetectorPtr());

  // set current arc_steer
  if (!frame_.is_replan_first) {
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

  frame_.is_replan_first = false;

  // TODO: check if need update
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
  parallel_path_planner_.SetLineSegmentHeading();
  // parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(0.3);
  parallel_path_planner_.SampleCurrentPathSeg();
  // print segment info
  parallel_path_planner_.PrintOutputSegmentsInfo();

  const auto& planner_output = parallel_path_planner_.GetOutput();
  gear_command_ = planner_output.current_gear;

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(planner_output.path_point_vec.size());

  pnc::geometry_lib::PathPoint global_point;
  for (const auto& path_point : planner_output.path_point_vec) {
    global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                     ego_slot_info.l2g_tf.GetHeading(path_point.heading));

    current_path_point_global_vec_.emplace_back(global_point);
  }

  std::cout << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size() << std::endl;

  return plan_result;
}

const bool ParallelParInPlanner::CheckSegCompleted() {
  frame_.is_replan_by_uss = false;

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

const bool ParallelParInPlanner::CheckReplan() {
  if (frame_.is_replan_first == true || simu_param_.force_plan) {
    std::cout << "first plan" << std::endl;
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  if (CheckSegCompleted()) {
    std::cout << "replan by current segment completed!" << std::endl;
    frame_.replan_reason = SEG_COMPLETED_PATH;
    return true;
  }

  if (frame_.stuck_time > apa_param.GetParam().stuck_replan_time) {
    std::cout << "replan by stuck!" << std::endl;
    frame_.replan_reason = STUCKED;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool ParallelParInPlanner::CheckStuckFailed() {
  return frame_.stuck_time > apa_param.GetParam().stuck_failed_time;
}

const bool ParallelParInPlanner::CheckFinished() {
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

  const bool parking_finish =
      lon_condition && lat_condition && static_condition;

  std::cout << "terminal x error= " << ego_slot_info.terminal_err.pos.x()
            << std::endl;
  std::cout << "terminal y error= " << ego_slot_info.terminal_err.pos.y()
            << std::endl;
  std::cout << "terminal heading error= "
            << ego_slot_info.terminal_err.heading * 57.3 << std::endl;

  return parking_finish;
}

void ParallelParInPlanner::GenPlanningOutput() {
  pnc::geometry_lib::PathPoint current_ego_pose(
      apa_world_ptr_->GetMeasurementsPtr()->pos_ego,
      apa_world_ptr_->GetMeasurementsPtr()->heading_ego);

  if (frame_.plan_stm.planning_status == PARKING_FINISHED) {
    SetFinishedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_FAILED) {
    SetFailedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
             frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
             frame_.plan_stm.planning_status == PARKING_RUNNING) {
    GenPlanningPath();
  } else if (frame_.plan_stm.planning_status == PARKING_IDLE) {
    SetIdlePlanningOutput(planning_output_, current_ego_pose);
  }

  // std::cout << "planning_output:" << planning_output_.DebugString()
  //           << std::endl;
}

void ParallelParInPlanner::GenPlanningPath() {
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

void ParallelParInPlanner::InitSimulation() {
  if (is_simulation_ && simu_param_.force_plan && simu_param_.is_reset) {
    Reset();
  }
}

const bool ParallelParInPlanner::CheckPlanSkip() const {
  if (frame_.plan_stm.planning_status == PARKING_FINISHED ||
      frame_.plan_stm.planning_status == PARKING_FAILED) {
    std::cout << "plan has been finished or failed, need reset" << std::endl;
    return true;
  } else {
    return false;
  }
}

const double ParallelParInPlanner::CalRemainDistFromPath() {
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

const double ParallelParInPlanner::CalRemainDistFromUss() {
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

  return remain_dist;
}

void ParallelParInPlanner::UpdateRemainDist() {
  // 1. calculate remain dist according to plan path
  frame_.remain_dist = CalRemainDistFromPath();

  // 2.calculate remain dist uss according to uss
  frame_.remain_dist_uss = CalRemainDistFromUss();

  return;
}

const bool ParallelParInPlanner::PostProcessPath() {
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

void ParallelParInPlanner::Log() const {
  const auto& l2g_tf = frame_.ego_slot_info.l2g_tf;
  const auto p0_g = l2g_tf.GetPos(t_lane_.pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.pt_inside);
  const auto pt_g = l2g_tf.GetPos(t_lane_.pt_terminal);

  std::cout << "p0_g = " << p0_g.transpose() << std::endl;

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("slot_side", t_lane_.slot_side)

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
}

}  // namespace apa_planner
}  // namespace planning
