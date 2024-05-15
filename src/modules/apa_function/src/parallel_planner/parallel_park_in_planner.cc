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

#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine.pb.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "math_lib.h"
#include "parallel_path_planner.h"

namespace planning {
namespace apa_planner {
static double kFrontDetaXMagWhenFrontVacant = 1.66;
static double kRearDetaXMagWhenFrontVacant = 0.2;
static double kRearDetaXMagWhenBothSidesVacant = 0.2;
static double kRearDetaXMagWhenFrontOccupiedRearVacant = 1.5;

static double kFrontObsLineYMagIdentification = 0.8;
static double kRearObsLineYMagIdentification = 0.8;
static double kChannelLengthXMagIdentification = 1.0;
static double kChannelLengthMinYMagIdentification = 0.5;
static double kChannelLengthMaxYMagIdentification = 5.0;
static double kCurbInitialOffset = 0.2;
static double kCurbYMagIdentification = 0.0;

static double kEnterMultiPlanSlotRatio = 0.36;

void ParallelParInPlanner::Init(const bool c_ilqr_enable) {
  lateral_path_optimizer_ptr_ = std::make_shared<LateralPathOptimizer>();
  lateral_path_optimizer_ptr_->Init(c_ilqr_enable);
  // reset
  Reset();
}

void ParallelParInPlanner::Reset() {
  frame_.Reset();
  t_lane_.Reset();
  gear_command_ = 0;
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
  if (!simu_param_.force_plan && CheckPlanSkip()) {
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
  if (CheckReplan() || simu_param_.force_plan) {
    std::cout << "replan is required!" << std::endl;

    // generate t-lane
    GenTlane();

    // update obstacles
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
    std::cout << "check apa finished again!" << std::endl;
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    std::cout << "check stuck failed again!" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // print planning status
  std::cout << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status)
            << std::endl;
}

const bool ParallelParInPlanner::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasurementsPtr();
  const auto slot_manager_ptr = apa_world_ptr_->GetSlotManagerPtr();

  auto& ego_slot_info = frame_.ego_slot_info;

  // notice: get slot from GetEgoSlotInfo.select_slot_filter in slot management
  ego_slot_info.target_managed_slot.CopyFrom(
      slot_manager_ptr->GetEgoSlotInfo().select_slot_filter);

  const auto& slot_points =
      ego_slot_info.target_managed_slot.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(slot_points.size());

  std::cout << "parallel slot points in slm :" << std::endl;
  for (int i = 0; i < slot_points.size(); i++) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
    std::cout << pt[i].transpose() << std::endl;
  }
  ego_slot_info.slot_corner = pt;

  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.obs_pt_vec_slot =
      slot_manager_ptr->GetEgoSlotInfo().obs_pt_vec_slot;

  // calc slot side once at first
  if (frame_.is_replan_first == true) {
    const Eigen::Vector2d v_ego_to_pt3 = pt[3] - measures_ptr->pos_ego;

    const double cross_ego_to_pt3 = pnc::geometry_lib::GetCrossFromTwoVec2d(
        measures_ptr->heading_ego_vec, v_ego_to_pt3);

    // judge slot side via slot pt3
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_pt3 < 0.0) {
      t_lane_.slot_side_sgn = 1.0;
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_pt3 > 0.0) {
      t_lane_.slot_side_sgn = -1.0;
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      t_lane_.slot_side_sgn = 0.0;
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
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

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measures_ptr->pos_ego);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading_ego);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // Todo: use limiter to determine the practical target pose

  // calc terminal pos
  const double terminal_x =
      apa_param.GetParam().rear_overhanging +
      apa_param.GetParam().finish_parallel_rear_stop_buffer;

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
  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;

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

  // update stuck time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
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

void ParallelParInPlanner::UpdateSlotRealtime() {
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

void ParallelParInPlanner::GenTlane() {
  // Todo: generate t-lane according to nearby obstacles
  const auto& ego_slot_info = frame_.ego_slot_info;

  const auto x_ascending_func = [](const Eigen::Vector2d& a,
                                   const Eigen::Vector2d& b) {
    return a.x() > b.x();  // the top element is smallest
  };

  const auto x_descending_func = [](const Eigen::Vector2d& a,
                                    const Eigen::Vector2d& b) {
    return a.x() < b.x();  // the top element is largest
  };

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

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      decltype(x_ascending_func)>
      front_obs_que_ascending_x(x_ascending_func);

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      decltype(x_descending_func)>
      rear_obs_que_descending_x(x_descending_func);

  const double slot_length = ego_slot_info.slot_length;
  const double half_slot_width = 0.5 * ego_slot_info.slot_width;

  const double side_sgn =
      (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0);

  double channel_x_limit = apa_param.GetParam().parallel_channel_x_mag;
  double channel_y_limit =
      side_sgn * apa_param.GetParam().parallel_channel_y_mag;

  size_t curb_count = 0;
  double curb_y_limit = -side_sgn * (half_slot_width + kCurbInitialOffset);

  const double quarter_slot_width = 0.5 * half_slot_width;
  double front_parallel_line_y_limit = side_sgn * quarter_slot_width;
  double rear_parallel_line_y_limit = side_sgn * quarter_slot_width;

  DEBUG_PRINT("obs size =" << ego_slot_info.obs_pt_vec_slot.size()
                           << "-----------");
  // shift obstacles that meet requirement
  for (const auto& obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    // DEBUG_PRINT(obstacle_point_slot.transpose());

    const bool front_obs_condition =
        (std::fabs(obstacle_point_slot.x() - slot_length) <
         kFrontDetaXMagWhenFrontVacant) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y() * side_sgn,
            -kFrontObsLineYMagIdentification,
            half_slot_width + kFrontObsLineYMagIdentification);

    if (front_obs_condition) {
      // front obs line que
      front_obs_que_ascending_x.emplace(obstacle_point_slot);
      // DEBUG_PRINT("front_obs_condition!");
    }

    const bool rear_obs_condition =
        (std::fabs(obstacle_point_slot.x()) <
         kRearDetaXMagWhenFrontOccupiedRearVacant) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y() * side_sgn, -kRearObsLineYMagIdentification,
            half_slot_width + kRearObsLineYMagIdentification);

    if (rear_obs_condition) {
      // rear obs line que
      rear_obs_que_descending_x.emplace(obstacle_point_slot);
      // DEBUG_PRINT("rear_obs_condition!");
    }

    const bool curb_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                kRearDetaXMagWhenFrontVacant,
                                slot_length - kRearDetaXMagWhenFrontVacant) &&
        (obstacle_point_slot.y() * side_sgn <= -kCurbYMagIdentification);

    if (curb_condition) {
      curb_count++;
      curb_y_limit = side_sgn > 0.0
                         ? std::max(curb_y_limit, obstacle_point_slot.y())
                         : std::min(curb_y_limit, obstacle_point_slot.y());
      // DEBUG_PRINT("curb condition!");
    }

    // const bool channel_y_condition =
    //     (obstacle_point_slot.x() > -1.0) &&
    //     (obstacle_point_slot.x() < slot_length + 1.0) &&
    //     (obstacle_point_slot.y() * side_sgn >= 2.5);

    // if (channel_y_condition) {
    //   channel_y_limit =
    //       side_sgn > 0.0 ? std::min(channel_y_limit, obstacle_point_slot.y())
    //                      : std::max(channel_y_limit,
    //                      obstacle_point_slot.y());
    //   DEBUG_PRINT("channel_y_condition!");
    // }

    const bool channel_x_limit_condition =
        (obstacle_point_slot.x() >
         slot_length + kChannelLengthXMagIdentification) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y() * side_sgn,
            half_slot_width + kChannelLengthMinYMagIdentification,
            half_slot_width + kChannelLengthMaxYMagIdentification);

    if (channel_x_limit_condition) {
      channel_x_limit = std::min(channel_x_limit, obstacle_point_slot.x());
      // DEBUG_PRINT("channel_x_limit_condition!");
    }

    const bool front_parallel_line_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), slot_length - 0.2,
                                slot_length + 2.5) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y() * side_sgn, 0.0, 2.3);

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

  if (front_obs_que_ascending_x.empty()) {
    front_vacant = true;
    DEBUG_PRINT("front space empty!");
  }

  if (rear_obs_que_descending_x.empty()) {
    rear_vacant = true;
    DEBUG_PRINT("rear space empty!");
  }

  if (front_vacant && rear_vacant) {
    front_obs_que_ascending_x.emplace(
        Eigen::Vector2d(slot_length + kFrontDetaXMagWhenFrontVacant,
                        side_sgn * 0.5 * half_slot_width));

    rear_obs_que_descending_x.emplace(Eigen::Vector2d(
        -kRearDetaXMagWhenBothSidesVacant, side_sgn * 0.5 * half_slot_width));
  } else if (front_vacant && !rear_vacant) {
    front_obs_que_ascending_x.emplace(
        Eigen::Vector2d(slot_length + kFrontDetaXMagWhenFrontVacant,
                        side_sgn * 0.5 * half_slot_width));
    rear_obs_que_descending_x.emplace(Eigen::Vector2d(
        -kRearDetaXMagWhenFrontVacant, side_sgn * 0.5 * half_slot_width));

  } else if (!front_vacant && rear_vacant) {
    rear_obs_que_descending_x.emplace(
        Eigen::Vector2d(-kRearDetaXMagWhenFrontOccupiedRearVacant,
                        side_sgn * 0.5 * half_slot_width));
  }

  const double front_min_x =
      pnc::mathlib::Clamp(front_obs_que_ascending_x.top().x(),
                          slot_length - kRearDetaXMagWhenFrontVacant,
                          slot_length + kFrontDetaXMagWhenFrontVacant);

  DEBUG_PRINT("front quene min x = " << front_obs_que_ascending_x.top().x());
  DEBUG_PRINT("front min x = " << front_min_x);

  const double front_y_limit = front_parallel_line_y_limit;
  DEBUG_PRINT("front parallel line y =" << front_y_limit);

  const double rear_max_x = pnc::mathlib::Clamp(
      rear_obs_que_descending_x.top().x(),
      -kRearDetaXMagWhenFrontOccupiedRearVacant, -kRearDetaXMagWhenFrontVacant);
  DEBUG_PRINT("rear quene min x = " << rear_obs_que_descending_x.top().x());
  DEBUG_PRINT("rear max x = " << rear_max_x);

  const double rear_y_limit = rear_parallel_line_y_limit;
  DEBUG_PRINT("rear parallel line y =" << rear_y_limit);

  t_lane_.obs_pt_inside << front_min_x, front_y_limit;
  t_lane_.obs_pt_outside << rear_max_x, rear_y_limit;

  t_lane_.corner_inside_slot << slot_length, half_slot_width * side_sgn;
  t_lane_.corner_outside_slot << 0.0, half_slot_width * side_sgn;

  t_lane_.curb_y = curb_y_limit;
  t_lane_.channel_y = channel_y_limit;
  t_lane_.channel_x_limit = channel_x_limit;

  if (t_lane_.obs_pt_inside.x() - t_lane_.corner_inside_slot.x() > 1.0) {
    t_lane_.pt_inside.x() = t_lane_.corner_inside_slot.x() + 1.0;
  } else if (t_lane_.obs_pt_inside.x() - t_lane_.corner_inside_slot.x() > 0.2) {
    t_lane_.pt_inside.x() = std::max(
        t_lane_.corner_inside_slot.x() + 0.2,
        0.5 * (t_lane_.obs_pt_inside.x() + t_lane_.corner_inside_slot.x()));
  } else {
    t_lane_.pt_inside.x() = t_lane_.corner_inside_slot.x() + 0.2;
  }

  if (t_lane_.obs_pt_inside.y() * side_sgn > t_lane_.pt_inside.y() * side_sgn) {
    t_lane_.pt_inside.y() = t_lane_.obs_pt_inside.y() + 0.1;
  } else {
    t_lane_.pt_inside.y() =
        0.5 * (t_lane_.obs_pt_inside.y() + t_lane_.pt_inside.y());
  }

  t_lane_.pt_inside.x() =
      std::min(t_lane_.obs_pt_inside.x() - 0.2, t_lane_.corner_inside_slot.x());

  t_lane_.pt_inside.y() =
      std::min(t_lane_.obs_pt_inside.y(), t_lane_.corner_inside_slot.y());

  t_lane_.pt_outside = t_lane_.obs_pt_outside;

  t_lane_.slot_width = ego_slot_info.slot_width;
  t_lane_.slot_length = ego_slot_info.slot_length;

  // update tlane using USS dist
  if (frame_.ego_slot_info.slot_occupied_ratio >= kEnterMultiPlanSlotRatio &&
      frame_.in_slot_plan_count == 0) {
    UpdateTlaneOnceInSlot();
  }

  // if curb exist, combine curb and slot center line to decide target y
  if (curb_count > 1) {
    const double target_y_with_curb =
        curb_y_limit +
        side_sgn * (apa_param.GetParam().terminal_parallel_y_offset_with_curb +
                    0.5 * apa_param.GetParam().car_width);

    frame_.ego_slot_info.target_ego_pos_slot.y() =
        (side_sgn ? std::max(t_lane_.pt_terminal_pos.y(), target_y_with_curb)
                  : std::min(t_lane_.pt_terminal_pos.y(), target_y_with_curb));
  }
  // get accurate target x combining with obs tlane
  const double front_obs_x = t_lane_.obs_pt_inside.x();
  const double rear_obs_x = t_lane_.obs_pt_outside.x();

  const double front_corner_x = t_lane_.corner_inside_slot.x();
  const double rear_corner_x = t_lane_.corner_outside_slot.x();

  const double rac_to_geom_center =
      0.5 * apa_param.GetParam().car_length - apa_param.GetParam().rear_overhanging;

  const double target_x_in_slot_center =
      (front_corner_x + rear_corner_x) * 0.5 - rac_to_geom_center;

  const double stop_buffer =
      apa_param.GetParam().finish_parallel_rear_stop_buffer;

  const double ego_front_x = target_x_in_slot_center +
                             apa_param.GetParam().wheel_base +
                             apa_param.GetParam().front_overhanging;
  const double ego_rear_x =
      target_x_in_slot_center - apa_param.GetParam().rear_overhanging;

  DEBUG_PRINT("terminal debug --------------------------------");
  DEBUG_PRINT("target_x_in_slot_center =" << target_x_in_slot_center);

  const bool is_front_exceeded = ego_front_x > front_obs_x - stop_buffer;
  const bool is_rear_exceeded = ego_rear_x < rear_obs_x + stop_buffer;

  double target_x = target_x_in_slot_center;
  if (is_front_exceeded && is_rear_exceeded) {
    DEBUG_PRINT("both sides exceed!");
    target_x = (front_obs_x + rear_obs_x) * 0.5 - rac_to_geom_center;
  } else if (is_front_exceeded && !is_rear_exceeded) {
    DEBUG_PRINT("front exceeded, rear safe!");
    const double target_x_via_front_obs =
        front_obs_x - apa_param.GetParam().wheel_base -
        apa_param.GetParam().front_overhanging - stop_buffer;

    const double target_x_via_obs_corner =
        (front_obs_x + rear_corner_x) * 0.5 - rac_to_geom_center;
    target_x = std::min(target_x_via_front_obs, target_x_via_obs_corner);

    const double target_x_via_obs_center =
        (front_obs_x + rear_obs_x) * 0.5 - rac_to_geom_center;

    target_x = std::max(target_x, target_x_via_obs_center);

  } else if (!is_front_exceeded && is_rear_exceeded) {
    DEBUG_PRINT("rear exceeded, front safe!");
    const double target_x_via_rear_obs =
        rear_obs_x + apa_param.GetParam().rear_overhanging + stop_buffer;
    const double target_x_via_corner_obs =
        (front_corner_x + rear_obs_x) * 0.5 - rac_to_geom_center;

    target_x = std::max(target_x_via_rear_obs, target_x_via_corner_obs);

    const double target_x_via_obs_center =
        (front_obs_x + rear_obs_x) * 0.5 - rac_to_geom_center;
    target_x = std::min(target_x, target_x_via_obs_center);
  } else {
    DEBUG_PRINT("both sides safe");
    target_x = target_x_in_slot_center;
  }
  frame_.ego_slot_info.target_ego_pos_slot.x() = target_x;

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
}

void ParallelParInPlanner::UpdateTlaneOnceInSlot() {
  if (frame_.ego_slot_info.slot_occupied_ratio < kEnterMultiPlanSlotRatio) {
    DEBUG_PRINT("ego not in slot!");
    return;
  }

  if (frame_.in_slot_plan_count != 0) {
    DEBUG_PRINT("already planned twice in slot!");
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
  const std::vector<size_t> upa_idx_vec = {1, 2, 3, 4, 7, 8, 9, 10};
  pnc::geometry_lib::LocalToGlobalTf ego2slot(ego_pose.pos, ego_pose.heading);

  for (size_t i = 0; i < upa_idx_vec.size(); i++) {
    const auto idx = upa_idx_vec[i];
    const double upa_dist = uss_dist_vec[idx];

    const Eigen::Vector2d upa_pos_ego(
        apa_param.GetParam().uss_vertex_x_vec[idx],
        apa_param.GetParam().uss_vertex_y_vec[idx]);

    const Eigen::Vector2d upa_pos_slot = ego2slot.GetPos(upa_pos_ego);

    if (i < 4) {
      t_lane_.obs_pt_inside.x() =
          std::min(t_lane_.obs_pt_inside.x(), upa_pos_slot.x() + upa_dist);
    } else {
      t_lane_.obs_pt_outside.x() =
          std::max(t_lane_.obs_pt_outside.x(), upa_pos_slot.x() - upa_dist);
    }
  }
}

void ParallelParInPlanner::GenObstacles() {
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
  double safe_dist = apa_param.GetParam().max_obs2car_dist_in_slot;
  if (frame_.ego_slot_info.slot_occupied_ratio < 0.018) {
    safe_dist = apa_param.GetParam().max_obs2car_dist_out_slot;
  }

  for (const auto& obs_pos : tlane_obstacle_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs_pos, ego_pose, safe_dist)) {
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(obs_pos);
      // std::cout << "obs_pos = " << obs_pos.transpose() << std::endl;
    }
  }
}

const bool ParallelParInPlanner::IsEgoInSlot() const {
  return std::fabs(frame_.ego_slot_info.ego_pos_slot.y() -
                   t_lane_.pt_terminal_pos.y()) <=
         0.5 * (frame_.ego_slot_info.slot_width +
                apa_param.GetParam().car_width - 0.2);
}

const bool ParallelParInPlanner::IsEgoInSlot(
    const pnc::geometry_lib::PathPoint& pose) const {
  return pose.pos.y() * t_lane_.slot_side_sgn <=
         t_lane_.slot_side_sgn * t_lane_.corner_inside_slot.y();
}

const uint8_t ParallelParInPlanner::PathPlanOnce() {
  // construct input

  ParallelPathPlanner::Input path_planner_input;
  path_planner_input.tlane = t_lane_;
  path_planner_input.sample_ds = simu_param_.sample_ds;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;

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

  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;

  JSON_DEBUG_VALUE("ref_gear", path_planner_input.ref_gear);
  JSON_DEBUG_VALUE("ref_arc_steer", path_planner_input.ref_arc_steer);

  DEBUG_PRINT("ref gear to path planner input ="
              << static_cast<int>(path_planner_input.ref_gear));
  DEBUG_PRINT("ref steer to path planner input ="
              << static_cast<int>(path_planner_input.ref_arc_steer));

  parallel_path_planner_.SetInput(path_planner_input);

  const bool path_plan_success =
      parallel_path_planner_.Update(apa_world_ptr_->GetCollisionDetectorPtr());

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

  // bool is_allow_entend_line = true;
  // if (frame_.ego_slot_info.slot_occupied_ratio < 0.05) {
  //   const auto path_end_idx = path_planner_output.path_seg_index.second;
  //   const auto& end_pose =
  //       path_planner_output.path_segment_vec[path_end_idx].GetEndPose();

  //   if (IsEgoInSlot(end_pose)) {
  //     is_allow_entend_line = false;
  //   }
  // }

  // if (is_allow_entend_line) {
  //   const double extend_lenth = 0.15;
  //   parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
  //       extend_lenth);
  // }
  if (ego_slot_info.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    const double extend_lenth = 0.15;
    parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
        extend_lenth);
  }
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
  gear_command_ = planner_output.current_gear;

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
  if (!is_simulation_) {
    parallel_optimization_enable = apa_param.GetParam().parallel_lat_opt_enable;
    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    parallel_optimization_enable = simu_param_.is_path_optimization;
    cilqr_optimization_enable = simu_param_.is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (parallel_optimization_enable && is_use_optimizer) {
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

    lateral_path_optimizer_ptr_->Init(cilqr_optimization_enable);

    lateral_path_optimizer_ptr_->SetParam(param);

    auto time_start = IflyTime::Now_ms();
    lateral_path_optimizer_ptr_->Update(planner_output.path_point_vec,
                                        gear_command_);

    auto time_end = IflyTime::Now_ms();
    lat_path_opt_cost_time_ms = time_end - time_start;

    const auto& optimized_path_vec =
        lateral_path_optimizer_ptr_->GetOutputPathVec();

    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(optimized_path_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : optimized_path_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }

    const auto plan_debug_info =
        lateral_path_optimizer_ptr_->GetOutputDebugInfo();

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

const bool ParallelParInPlanner::CheckPaused() {
  if (apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_SUSPEND_ACTIVATE ||
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_SUSPEND_CLOSE) {
    return true;
  } else {
    return false;
  }
}

const bool ParallelParInPlanner::CheckStuckFailed() {
  return frame_.stuck_time > apa_param.GetParam().stuck_failed_time;
}

const bool ParallelParInPlanner::CheckFinished() {
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
  const bool lon_condition = lon_condition_1 || lon_condition_2;

  const bool heading_condition =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_parallel_heading_err / 57.3;

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

  const bool static_condition =
      apa_world_ptr_->GetMeasurementsPtr()->static_flag;

  DEBUG_PRINT("terminal x error = " << ego_slot_info.terminal_err.pos.x());
  DEBUG_PRINT("terminal y error = " << ego_slot_info.terminal_err.pos.y());
  DEBUG_PRINT(
      "terminal heading error = " << ego_slot_info.terminal_err.heading * 57.3);
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

  return lon_condition && lat_condition && heading_condition &&
         static_condition;
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

  // send slot type to control
  planning_output_.mutable_trajectory()
      ->mutable_trajectory_points(2)
      ->set_distance(static_cast<double>(
          Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL));

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

  // // for debug, incase of same last several points
  //   if (current_path_point_global_vec_.size() > 5) {
  //     for (size_t i = current_path_point_global_vec_.size() - 4;
  //          i < current_path_point_global_vec_.size(); i++) {
  //       pnc::geometry_lib::PrintPose("last several points",
  //                                    current_path_point_global_vec_[i]);
  //     }
  //   }
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

    if (!is_simulation_) {
      apa_world_ptr_->GetSlotManagerPtr()->Reset();
    }

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
  const size_t origin_trajectory_size = current_path_point_global_vec_.size();

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
    // return false;
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

const double ParallelParInPlanner::CalcSlotOccupiedRatio(
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

void ParallelParInPlanner::Log() const {
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

  const std::vector<Eigen::Vector2d>& obstacles =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetObstacles();

  std::vector<double> obstaclesX;
  std::vector<double> obstaclesY;
  obstaclesX.clear();
  obstaclesY.clear();
  obstaclesX.reserve(obstacles.size());
  obstaclesY.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    const auto tmp_obstacle = l2g_tf.GetPos(obstacle);
    obstaclesX.emplace_back(tmp_obstacle.x());
    obstaclesY.emplace_back(tmp_obstacle.y());
  }
  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

  DEBUG_PRINT("obs " << obstaclesX.front() << ", " << obstaclesY.front());

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
  JSON_DEBUG_VALUE(
      "car_static_timer_by_pos",
      apa_world_ptr_->GetMeasurementsPtr()->car_static_timer_by_pos)
  JSON_DEBUG_VALUE(
      "car_static_timer_by_vel",
      apa_world_ptr_->GetMeasurementsPtr()->car_static_timer_by_vel)
  JSON_DEBUG_VALUE("static_flag",
                   apa_world_ptr_->GetMeasurementsPtr()->static_flag)
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
      lateral_path_optimizer_ptr_->GetOutputDebugInfo();

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
