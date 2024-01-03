#include "parallel_park_in_planner.h"

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine.pb.h"
#include "geometry_math.h"
#include "local_view.h"

namespace planning {
namespace apa_planner {

// planning params
static const double kNormalSlotLength = 5.2;
static const double kMaxFinishLatOffset = 0.08;
static const double kMaxFinishLonOffset = 0.2;
static const double kMaxFinishHeadingOffset = 2.8 / 57.3;
static const double kMaxVelocity = 0.6;
static const double kSafeUssRemainDist = 0.35;

// replan params
static const double kStuckFailedTime = 9.0;
static const double kStuckReplanTime = 4.0;
static const double kMaxReplanRemainDist = 0.2;

// T-lane expending or shrinking distance
static const double kVacantXp0 = 0.5;
static const double kVacantYp0 = 0.3;
static const double kVacantXp1 = 0.8;
static const double kVacantYp1 = 0.6;

static const double kOccupiedXp0 = 0.2;
static const double kOccupiedYp0 = 0.0;
static const double kOccupiedXp1 = 0.2;
static const double kOccupiedYp1 = 0.0;

static const double kNearBySlotCornerDist = 0.6;
static const double kChannelWidth = 6.5;
static const double kTerminalTargetX = 1.35;
static const double kTerminalTargetY = 0.0;
static const double kTerminalTargetYBias = 0.0;
static const double kPlanTime = 0.1;
static const double kUssStuckReplanWaitTime = 2.5;
static const bool kForceBothOccupied = true;
static const double kRearOverhanging = 0.94;
static const double kRearStopBuffer = 0.45;
static const double kpie = 3.141592653589793;

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

  // check failed
  if (CheckStuckFailed()) {
    std::cout << "check stuck failed!" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // check finish
  if (CheckFinished()) {
    std::cout << "check apa finished!" << std::endl;
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check replan
  if (CheckReplan() || simu_param_.force_plan) {
    std::cout << "replan is required!" << std::endl;
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

  for (int i = 0; i < slot_points.size(); i++) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }

  const auto pM01 = 0.5 * (pt[0] + pt[1]);
  const auto pM23 = 0.5 * (pt[2] + pt[3]);

  // calc slot side once at first
  if (frame_.is_replan_first == true) {
    Eigen::Vector2d v_ego_to_slot_pt3 = pt[3] - measures_ptr->pos_ego;

    const double cross_ego_to_slot_pt3 =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->heading_ego_vec,
                                                v_ego_to_slot_pt3);

    // judge slot side via slot center and heading
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_pt3 < 0.0) {
      t_lane_.slot_side = SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_pt3 > 0.0) {
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

  ego_slot_info.slot_origin_pos = 0.5 * (pM01 + pM23);
  const auto v_10 = (pt[0] - pt[1]).normalized();
  ego_slot_info.slot_origin_heading_vec = v_10;

  ego_slot_info.slot_origin_heading =
      std::atan2(ego_slot_info.slot_origin_heading_vec.y(),
                 ego_slot_info.slot_origin_heading_vec.x());

  std::cout << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;

  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  ego_slot_info.slot_length = (pt[0] - pt[1]).norm();
  ego_slot_info.slot_width = (pt[0] - pt[2]).norm();

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

  const double slot_side_sgn =
      (t_lane_.slot_side == SLOT_SIDE_RIGHT ? 1.0 : -1.0);

  // calc terminal pos
  ego_slot_info.target_ego_pos_slot.x() =
      (-0.5 * ego_slot_info.slot_length + kRearOverhanging + kRearStopBuffer) *
      slot_side_sgn;

  ego_slot_info.target_ego_pos_slot.y() = 0.0;
  ego_slot_info.target_ego_heading_slot = (slot_side_sgn > 0.0 ? 0.0 : kpie);

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

  std::cout << "ego_heading_slot = " << ego_slot_info.ego_heading_slot
            << std::endl;

  std::cout << "slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;
  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;

  // update stuck time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
    frame_.stuck_time += kPlanTime;
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

  const auto& target_corner_pts =
      ego_slot_info.target_managed_slot.corner_points();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(target_corner_pts.corner_point_size());

  for (int i = 0; i < target_corner_pts.corner_point_size(); i++) {
    pt[i] << target_corner_pts.corner_point(i).x(),
        target_corner_pts.corner_point(i).y();
  }

  // order of parking slot on the left side is inconsistent with the right side
  const auto& slot_side = t_lane_.slot_side;

  Eigen::Vector2d corner0_slot(0.5 * ego_slot_info.slot_length,
                               0.5 * ego_slot_info.slot_width);

  Eigen::Vector2d corner1_slot(-0.5 * ego_slot_info.slot_length,
                               0.5 * ego_slot_info.slot_width);

  t_lane_.p1 = corner1_slot;
  t_lane_.p0 = corner0_slot;
  if (slot_side == SLOT_SIDE_RIGHT) {
    t_lane_.p1.swap(t_lane_.p0);
  }

  t_lane_.pt << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  std::cout << "-- t_lane:" << std::endl;
  std::cout << "p0 = " << t_lane_.p0.transpose() << std::endl;
  std::cout << "p1 = " << t_lane_.p1.transpose() << std::endl;
  std::cout << "pt = " << t_lane_.pt.transpose() << std::endl;
}

const std::vector<Eigen::Vector2d> ParallelParInPlanner::GenTlaneInGlobSystem()
    const {
  std::vector<Eigen::Vector2d> tlane_pts;

  tlane_pts.emplace_back(frame_.ego_slot_info.l2g_tf.GetPos(t_lane_.p0));

  tlane_pts.emplace_back(
      frame_.ego_slot_info.l2g_tf.GetPos(Eigen::Vector2d(0, t_lane_.p0.y())));

  tlane_pts.emplace_back(
      frame_.ego_slot_info.l2g_tf.GetPos(Eigen::Vector2d(0, t_lane_.p1.y())));

  tlane_pts.emplace_back(frame_.ego_slot_info.l2g_tf.GetPos(t_lane_.p1));
  return tlane_pts;
}

const uint8_t ParallelParInPlanner::PathPlanOnce() {
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;

  ParallelPathPlanner::Input path_planner_input;
  path_planner_input.tlane = t_lane_;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;
  path_planner_input.sample_ds = simu_param_.sample_ds;

  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;

  parallel_path_planner_.SetInput(path_planner_input);

  // obj need change from lineSegment to point
  // parallel_path_planner_.SetObstacle(collision_detector_ptr_->GetObstacles());

  // TODO: check if need update
  uint8_t plan_result = 0;

  // need replan all path
  if (parallel_path_planner_.Update()) {
    plan_result = PathPlannerResult::PLAN_UPDATE;
  } else {
    plan_result = PathPlannerResult::PLAN_FAILED;
    return PathPlannerResult::PLAN_FAILED;
  }

  // print segment info
  parallel_path_planner_.PrintOutputSegmentsInfo();

  parallel_path_planner_.SetCurrentPathSegIndex();

  parallel_path_planner_.SetLineSegmentHeading();

  // parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(0.3);

  parallel_path_planner_.SampleCurrentPathSeg();

  const auto& planner_output = parallel_path_planner_.GetOutput();
  gear_command_ = planner_output.current_gear;

  // set current arc_steer
  if (!frame_.is_replan_first) {
    // set current arc steer
    if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
      std::cout << "fault ref_arc_steer state!" << std::endl;
      return false;
    } else if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    }

    // set current gear
    if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_INVALID) {
      std::cout << "fault ref_gear state!" << std::endl;
      return false;
    } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    } else {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    }
  }

  frame_.is_replan_first = false;

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

    if (min_remain_dist < kMaxReplanRemainDist &&
        apa_world_ptr_->GetMeasurementsPtr()->static_flag) {
      frame_.is_replan_by_uss = (frame_.remain_dist_uss < frame_.remain_dist);

      if (!frame_.is_replan_by_uss) {
        std::cout << "close to target!" << std::endl;
        is_seg_complete = true;
      } else {
        std::cout << "close to obstacle by uss!" << std::endl;
        if (frame_.ego_slot_info.slot_occupied_ratio > 0.25) {
          apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
          is_seg_complete = true;
        } else {
          if (frame_.stuck_time > kUssStuckReplanWaitTime) {
            is_seg_complete = true;
          }
        }
      }

    } else {
      is_seg_complete = false;
    }
  } else {
    is_seg_complete = false;
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
    frame_.replan_reason = SEG_COMPLETED;
    return true;
  }

  if (frame_.stuck_time > kStuckReplanTime) {
    std::cout << "replan by stuck!" << std::endl;
    frame_.replan_reason = STUCKED;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool ParallelParInPlanner::CheckStuckFailed() {
  return frame_.stuck_time > kStuckFailedTime;
}

const bool ParallelParInPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool parking_success =
      std::fabs(ego_slot_info.terminal_err.pos.x()) < kMaxFinishLatOffset &&
      std::fabs(ego_slot_info.terminal_err.pos.y()) <= kMaxFinishLonOffset &&
      std::fabs(ego_slot_info.terminal_err.heading) <=
          kMaxFinishHeadingOffset &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag;

  std::cout << "terminal x error= " << ego_slot_info.terminal_err.pos.x()
            << std::endl;
  std::cout << "terminal y error= " << ego_slot_info.terminal_err.pos.y()
            << std::endl;
  std::cout << "terminal heading error= "
            << ego_slot_info.terminal_err.heading * 57.3 << std::endl;

  return parking_success;
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
  const std::vector<double> vel_limit_tab = {kMaxVelocity, kMaxVelocity, 0.45,
                                             0.35};
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

  if (frame_.is_replan_first) {
    return remain_dist;
  }

  const auto uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(&planning_output_,
                                   apa_world_ptr_->GetLocalViewPtr());

  remain_dist = uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist -
                kSafeUssRemainDist;

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
  const auto p0_g = l2g_tf.GetPos(t_lane_.p0);
  const auto p1_g = l2g_tf.GetPos(t_lane_.p1);
  const auto pt_g = l2g_tf.GetPos(t_lane_.pt);

  std::cout << "p0_g = " << p0_g.transpose() << std::endl;

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("channel_x", t_lane_.channel_x)
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
  JSON_DEBUG_VALUE("use_line_step", path_plan_output.use_line_step)
}

}  // namespace apa_planner
}  // namespace planning
