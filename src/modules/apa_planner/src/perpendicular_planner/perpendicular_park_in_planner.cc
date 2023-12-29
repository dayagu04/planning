#include "perpendicular_park_in_planner.h"

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>

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
#include "local_view.h"
#include "perpendicular_path_planner.h"
#include "slot_management_info.pb.h"

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
static const double kOccupiedXp1 = 0.3;
static const double kOccupiedYp1 = 0.0;

static const double kNearBySlotCornerDist = 0.6;
static const double kChannelWidth = 6.5;
static const double kTerminalTargetX = 1.35;
static const double kTerminalTargetY = 0.0;
static const double kTerminalTargetYBias = 0.0;
static const double kTerminalTargetX2Limiter = 0.15;
static const double kPlanTime = 0.1;
static const double kUssStuckReplanWaitTime = 2.5;
static const bool kForceBothOccupied = true;

void PerpendicularInPlanner::Reset() {
  frame_.Reset();
  t_lane_.Reset();
}

void PerpendicularInPlanner::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED) {
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
    frame_.ego_slot_info.first_fix_limiter = true;
    frame_.ego_slot_info.terminal_target_X = kTerminalTargetX;

    UpdateSlotRealtime();

    // generate t-lane
    GenTlane();

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

const bool PerpendicularInPlanner::UpdateEgoSlotInfo() {
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
  const auto n = (pM01 - pM23).normalized();

  ego_slot_info.slot_origin_pos = pM01 - kNormalSlotLength * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;

  // calc slot side once at first
  if (frame_.is_replan_first == true) {
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
    frame_.current_gear = ApaPlannerBase::REVERSE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      t_lane_.slot_side = SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = ApaPlannerBase::RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      t_lane_.slot_side = SLOT_SIDE_LEFT;
      frame_.current_arc_steer = ApaPlannerBase::LEFT;
    } else {
      t_lane_.slot_side = SLOT_SIDE_NONE;
      frame_.current_arc_steer = ApaPlannerBase::STEER_NONE;
      frame_.current_gear = ApaPlannerBase::GEAR_NONE;
      std::cout << "calculate slot side error " << std::endl;
      // return false;
    }
  }

  std::cout << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;

  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  ego_slot_info.slot_length = kNormalSlotLength;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

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

  std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_point;
  const auto is_have_limiter =
      apa_world_ptr_->GetSlotManagerPtr()->GetSelectedLimiter(limiter_point);

  // std::cout << "limiter1: " << limiter_point.first.transpose() << " \n";
  // std::cout << "limiter2: " << limiter_point.second.transpose() << " \n";

  const auto point_mid = ego_slot_info.g2l_tf.GetPos(
      (limiter_point.first + limiter_point.second) / 2.0);

  if ((point_mid - ego_slot_info.ego_pos_slot).norm() < 1.0 &&
      ego_slot_info.first_fix_limiter &&
      perpendicular_path_planner_.GetOutput().is_last_path && false) {
    PostProcessPathAccordingLimiter();
    ego_slot_info.terminal_target_X = point_mid.x() + kTerminalTargetX2Limiter;
    ego_slot_info.first_fix_limiter = false;
  }

  // calc terminal pos
  ego_slot_info.target_ego_pos_slot << kTerminalTargetX,
      kTerminalTargetY + kTerminalTargetYBias;
  ego_slot_info.target_ego_heading_slot = 0.0;

  // calc terminal error once
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  std::cout << "-- ego_slot:" << std::endl;
  std::cout << "ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << std::endl;

  std::cout << "ego_heading_slot = " << ego_slot_info.ego_heading_slot
            << std::endl;

  std::cout << "slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;
  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;

  // calc slot occupied ratio
  // ego_slot_info.slot_occupied_ratio =
  //     apa_world_ptr_->GetSlotManagerPtr()->GetOccupiedRatio();
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) < 0.9 &&
      std::fabs(ego_slot_info.ego_heading_slot) < 75.0 / 57.3) {
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (ego_slot_info.terminal_err.pos.x() / kNormalSlotLength), 0.0,
        1.0);
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
    frame_.stuck_time += kPlanTime;
  } else {
    frame_.stuck_time = 0.0;
  }

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

  for (const auto& managed_slot : slot_info_vec) {
    Eigen::Vector2d managed_corner0(
        managed_slot.corner_points().corner_point(0).x(),
        managed_slot.corner_points().corner_point(0).y());

    Eigen::Vector2d managed_corner1(
        managed_slot.corner_points().corner_point(1).x(),
        managed_slot.corner_points().corner_point(1).y());

    if ((target_corner0 - managed_corner1).norm() < kNearBySlotCornerDist) {
      nearby_slot_nums++;
      corner_0_side_occupied = managed_slot.is_occupied();
    }

    if ((target_corner1 - managed_corner0).norm() < kNearBySlotCornerDist) {
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

  // order of parking slot on the left side is inconsistent with the right side
  const auto& slot_side = t_lane_.slot_side;

  Eigen::Vector2d corner0_slot(ego_slot_info.slot_length,
                               -0.5 * ego_slot_info.slot_width);

  Eigen::Vector2d corner1_slot(ego_slot_info.slot_length,
                               0.5 * ego_slot_info.slot_width);

  if (slot_side == SLOT_SIDE_LEFT) {
    corner0_slot.swap(corner1_slot);
  }

  if (slot_side == SLOT_SIDE_RIGHT) {
    // if right slot is free
    if (!corner_0_side_occupied) {
      t_lane_.p1 = corner0_slot + Eigen::Vector2d(-kVacantXp1, -kVacantYp1);
    } else {
      t_lane_.p1 = corner0_slot + Eigen::Vector2d(kOccupiedXp1, kOccupiedYp1);
    }

    if (!corner_1_side_occupied) {
      t_lane_.p0 = corner1_slot + Eigen::Vector2d(-kVacantXp0, kVacantYp0);
    } else {
      t_lane_.p0 = corner1_slot + Eigen::Vector2d(kOccupiedXp0, -kOccupiedYp0);
    }
  } else if (slot_side == SLOT_SIDE_LEFT) {
    if (!corner_0_side_occupied) {
      t_lane_.p1 = corner0_slot + Eigen::Vector2d(-kVacantXp1, kVacantYp1);
    } else {
      t_lane_.p1 = corner0_slot + Eigen::Vector2d(kOccupiedXp1, -kOccupiedYp1);
    }

    if (!corner_1_side_occupied) {
      t_lane_.p0 = corner1_slot + Eigen::Vector2d(-kVacantXp0, -kVacantYp0);
    } else {
      t_lane_.p0 = corner1_slot + Eigen::Vector2d(kOccupiedXp0, kOccupiedYp0);
    }
  }

  t_lane_.channel_x = kNormalSlotLength + kChannelWidth;

  t_lane_.pt << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  std::cout << "-- t_lane:" << std::endl;
  std::cout << "p0 = " << t_lane_.p0.transpose() << std::endl;
  std::cout << "p1 = " << t_lane_.p1.transpose() << std::endl;
  std::cout << "pt = " << t_lane_.pt.transpose() << std::endl;
}

const std::vector<Eigen::Vector2d>
PerpendicularInPlanner::GenTlaneInGlobSystem() const {
  std::vector<Eigen::Vector2d> tlane_pts;

  tlane_pts.emplace_back(frame_.ego_slot_info.l2g_tf.GetPos(t_lane_.p0));

  tlane_pts.emplace_back(
      frame_.ego_slot_info.l2g_tf.GetPos(Eigen::Vector2d(0, t_lane_.p0.y())));

  tlane_pts.emplace_back(
      frame_.ego_slot_info.l2g_tf.GetPos(Eigen::Vector2d(0, t_lane_.p1.y())));

  tlane_pts.emplace_back(frame_.ego_slot_info.l2g_tf.GetPos(t_lane_.p1));
  return tlane_pts;
}

const uint8_t PerpendicularInPlanner::PathPlanOnce() {
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;

  PerpendicularPathPlanner::Input path_planner_input;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = t_lane_;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;
  path_planner_input.sample_ds = simu_param_.sample_ds;

  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;

  perpendicular_path_planner_.SetInput(path_planner_input);

  // obj need change from lineSegment to point
  // perpendicular_path_planner_.SetObstacle(collision_detector_ptr_->GetObstacles());

  // TODO: check if need update
  uint8_t plan_result = 0;

  // need replan all path
  if (perpendicular_path_planner_.Update()) {
    plan_result = PathPlannerResult::PLAN_UPDATE;
  } else {
    plan_result = PathPlannerResult::PLAN_FAILED;
    return PathPlannerResult::PLAN_FAILED;
  }

  // print segment info
  perpendicular_path_planner_.PrintOutputSegmentsInfo();

  perpendicular_path_planner_.SetCurrentPathSegIndex();

  perpendicular_path_planner_.SetLineSegmentHeading();

  perpendicular_path_planner_.InsertLineSegAfterCurrentFollowLastPath(0.3);

  perpendicular_path_planner_.SampleCurrentPathSeg();

  const auto& planner_output = perpendicular_path_planner_.GetOutput();
  gear_command_ = planner_output.current_gear;

  // set current arc_steer
  if (!frame_.is_replan_first) {
    // set current arc steer
    if (frame_.current_arc_steer == ApaPlannerBase::STRAIGHT) {
      std::cout << "fault ref_arc_steer state!" << std::endl;
      return false;
    } else if (frame_.current_arc_steer == ApaPlannerBase::RIGHT) {
      frame_.current_arc_steer = ApaPlannerBase::LEFT;
    } else {
      frame_.current_arc_steer = ApaPlannerBase::RIGHT;
    }

    // set current gear
    if (frame_.current_gear == ApaPlannerBase::GEAR_NONE) {
      std::cout << "fault ref_gear state!" << std::endl;
      return false;
    } else if (frame_.current_gear == ApaPlannerBase::DRIVE) {
      frame_.current_gear = ApaPlannerBase::REVERSE;
    } else {
      frame_.current_gear = ApaPlannerBase::DRIVE;
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

const bool PerpendicularInPlanner::CheckSegCompleted() {
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

void PerpendicularInPlanner::UpdateSlotRealtime() {
  bool update_slot = false;
  if (frame_.is_replan_by_uss &&
      frame_.ego_slot_info.slot_occupied_ratio > 0.25) {
    update_slot = true;
    std::cout << "update slot by uss!" << std::endl;
  }

  if (std::fabs(frame_.ego_slot_info.ego_heading_slot) < 55.0 / 57.3 &&
      frame_.ego_slot_info.ego_pos_slot.norm() < 7.5) {
    update_slot = true;
    std::cout << "update slot by pose!" << std::endl;
  }

  if (update_slot) {
    apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
    apa_world_ptr_->Update();
    UpdateEgoSlotInfo();
  }
}

const bool PerpendicularInPlanner::CheckReplan() {
  if (frame_.is_replan_first == true) {
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

const bool PerpendicularInPlanner::CheckStuckFailed() {
  return frame_.stuck_time > kStuckFailedTime;
}

const bool PerpendicularInPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool parking_success =
      ego_slot_info.terminal_err.pos.x() < kMaxFinishLonOffset &&
      std::fabs(ego_slot_info.terminal_err.pos.y()) <= kMaxFinishLatOffset &&
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
             frame_.plan_stm.planning_status == PARKING_RUNNING) {
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

  if (gear_command_ == DRIVE) {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE);
  } else {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);
  }
}

void PerpendicularInPlanner::InitSimulation() {
  if (is_simulation_ && simu_param_.force_plan && simu_param_.is_reset) {
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

  double safe_uss_remain_dist = kSafeUssRemainDist;
  if (frame_.ego_slot_info.slot_occupied_ratio < 0.05) {
    safe_uss_remain_dist = 0.6;
  }

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

  std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_point;
  const auto is_have_limiter =
      apa_world_ptr_->GetSlotManagerPtr()->GetSelectedLimiter(limiter_point);

  const auto point_mid = (limiter_point.first + limiter_point.second) / 2.0;

  double s = 0.0;
  double ds = 0.0;
  double dis = 0.0;
  size_t i = 0;
  for (i = 0; i < origin_traj_size; ++i) {
    auto& x = current_path_point_global_vec_[i].pos.x();
    auto& y = current_path_point_global_vec_[i].pos.y();
    double dis = (Eigen::Vector2d(x, y) - point_mid).norm();
    if (dis < kTerminalTargetX2Limiter) {
      // need shorten
      break;
    }
    x_vec.emplace_back(x);
    y_vec.emplace_back(y);
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);

    if (i == 0) {
      s_vec.emplace_back(s);
    } else {
      ds = std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s += std::max(ds, 1e-3);
      s_vec.emplace_back(s);
    }
  }

  if (i < 2) {
    frame_.spline_success = false;
    std::cout << "error: no enough point = " << i << std::endl;
    return false;
  }

  if (dis > kTerminalTargetX2Limiter) {
    // need extend by limiter
    double extend_dis = dis - kTerminalTargetX2Limiter;
    Eigen::Vector2d extended_point;
    bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
        Eigen::Vector2d(x_vec[i - 2], y_vec[i - 2]),
        Eigen::Vector2d(x_vec[i - 1], y_vec[i - 1]), extended_point,
        extend_dis);
    if (success == false) {
      frame_.spline_success = false;
      std::cout << "limit need extend fit line by spline error!" << std::endl;
      return false;
    }

    x_vec.emplace_back(extended_point.x());
    y_vec.emplace_back(extended_point.y());
    heading_vec.emplace_back(heading_vec.back());
    s += extend_dis;
    s_vec.emplace_back(s);
    i++;
  }

  frame_.current_path_length = s;

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(i);
  pnc::geometry_lib::PathPoint path_point;
  for (size_t j = 0; j < x_vec.size(); ++j) {
    path_point.Set(Eigen::Vector2d(x_vec[j], y_vec[j]), heading_vec[j]);
    current_path_point_global_vec_.emplace_back(path_point);
  }

  // need extend by cal proj point
  Eigen::Vector2d extended_point;
  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      Eigen::Vector2d(x_vec[i - 2], y_vec[i - 2]),
      Eigen::Vector2d(x_vec[i - 1], y_vec[i - 1]), extended_point,
      frame_.path_extended_dist);

  if (success == false) {
    frame_.spline_success = false;
    std::cout << "limit need extend fit line by spline error!" << std::endl;
    return false;
  }

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  heading_vec.emplace_back(heading_vec.back());
  s += frame_.path_extended_dist;
  s_vec.emplace_back(s);

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

  const auto& path_plan_output = perpendicular_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)
  JSON_DEBUG_VALUE("use_line_step", path_plan_output.use_line_step)
}

}  // namespace apa_planner
}  // namespace planning
