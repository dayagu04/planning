#include "diagonal/diagonal_in_trajectory_generator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <ostream>
#include <queue>
#include <vector>

#include "common/apa_cos_sin.h"
#include "common/apa_utils.h"
#include "common/planning_log_helper.h"
#include "common/vehicle_param_helper.h"
#include "debug_info_log.h"
#include "dubins_lib/dubins_lib.h"
#include "environmental_model.h"
#include "general_planning_context.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math/line_segment2d.h"
#include "math/math_utils.h"
#include "planning_output_context.h"
#include "spline_projection.h"
#include "utils_math.h"

// #define __PYBIND_DEBUG__

namespace planning {
namespace apa_planner {

using ::Common::GearCommandValue;
using framework::Frame;
using ::FuncStateMachine::FunctionalState;
using planning::planning_math::LineSegment2d;
using planning::planning_math::Vec2d;
using ::PlanningOutput::PlanningOutput;
using ::PlanningOutput::Trajectory;
using ::PlanningOutput::TrajectoryPoint;
using ::pnc::dubins_lib::DubinsLibrary;
using ::pnc::geometry_lib::TangentOutput;

namespace {
constexpr double kEps = 1e-6;
constexpr double kMaxAcc = 0.5;
constexpr double kMaxSpd = 0.5;
constexpr double kMinSegmentLen = 0.5 * kMaxSpd * kMaxSpd / kMaxAcc;
constexpr double kStep = 0.1;
constexpr double kYawStep = 0.1;
constexpr double kStanstillSpd = 0.01;
constexpr double kRemainingDisThreshold = 0.2;
constexpr uint64_t kMinStandstillTime = 500;  // ms
constexpr uint64_t kMinPosUnchangedCount = 5;
constexpr double kMaxXOffset = 0.2;
constexpr double kMaxYOffset = 0.2;
constexpr double kMaxThetaOffset = 0.05;
constexpr double kMockedObjYOffset = 4.0;

static const double kNormalSlotLength = 4.8;
static const double target_x_init = 1.5;
static const double min_radius = 5.2;
static const double kLineStep = 0.1;
static const double kMinProperBCLength = 0.3;
static const double plan_time = 0.1;
static const double terminal_target_x = 1.1;
static const double ideal_length_level_0 = 5.0;
static const double max_path_length = 20.0;
static const double min_search_length = 6.0;

}  // namespace

const bool DiagonalInTrajectoryGenerator::Plan(framework::Frame* const frame) {
  auto planning_output = &(frame->mutable_session()
                               ->mutable_planning_output_context()
                               ->mutable_planning_status()
                               ->planning_result.planning_output);
  if (planning_output->has_planning_status() &&
      planning_output->planning_status().has_apa_planning_status() &&
      planning_output->planning_status().apa_planning_status() ==
          ::PlanningOutput::ApaPlanningStatus::FINISHED) {
    AINFO << "apa is finished";
    return true;
  }
  frame_ = frame;
  local_view_ = &(frame->session()->environmental_model().get_local_view());

  // set local view to uss oa
  // uss_oa_.SetLocalView(local_view_);

  // update uss oa
  // uss_oa_.Update(planning_output);

  // set local view to collision detector
  // collision_detector_.SetLocalView(local_view_);

  // update collision detection
  // collision_detector_.SetUssOA(&uss_oa_);

  // TODO: collision_detector_.SetVision();

  // generate obstacles just by uss oa
  // collision_detector_.GenObstacles();

  auto& origin_parking_fusion_info = local_view_->parking_fusion_info;

  // slot management update
  if (g_context.GetStatemachine().apa_reset_flag) {
    Reset();
  }

  slot_manager_.Update(&local_view_->function_state_machine_info,
                       &origin_parking_fusion_info,
                       &local_view_->localization_estimate);

  // update measurement
  UpdateMeasurement();

  // log by json
  Log();

  const auto& parking_fusion_info = origin_parking_fusion_info;

  const int slots_size = parking_fusion_info.parking_fusion_slot_lists_size();
  if (slots_size == 0) {
    AERROR << "Error: slot size is 0";
    return false;
  }

  current_state_ = FunctionalState::INIT;
  if (local_view_->function_state_machine_info.has_current_state()) {
    current_state_ = local_view_->function_state_machine_info.current_state();
  }

  const auto& slots = parking_fusion_info.parking_fusion_slot_lists();
  if (IsSlotSelected(frame)) {
    if (!parking_fusion_info.has_select_slot_id()) {
      AERROR << "no select_slot_id";
      return false;
    }

    int select_slot_index = -1;
    const size_t selected_slot_id = parking_fusion_info.select_slot_id();
    AINFO << "selected_slot_id:" << selected_slot_id;
    for (int i = 0; i < parking_fusion_info.parking_fusion_slot_lists_size();
         ++i) {
      if (selected_slot_id != slots[i].id()) {
        continue;
      }
      if (slots[i].type() ==
              Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
          slots[i].type() ==
              Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
        select_slot_index = i;
        AINFO << "diagonal slot selected";
        break;
      }
    }
    if (select_slot_index == -1) {
      AERROR << "selected slot is not diagonal";
      return false;
    }

    // update managed parking fusion info by slot_manager
    UpdateManagedParkingFusion(select_slot_index);

    const bool flag = PathPlanOnce(select_slot_index, planning_output);
    GeneratePlanningOutput(flag, planning_output);

    return flag;
  } else {
    // bool is_planning_ok = false;
    // for (int i = 0; i < parking_fusion_info.parking_fusion_slot_lists_size();
    //      ++i) {
    //   if (slots[i].type() ==
    //           Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
    //       slots[i].type() ==
    //           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    //     AINFO << "diagonal slot id:" << slots[i].id();
    //     is_planning_ok = SingleSlotPlan(i, planning_output) ||
    //     is_planning_ok;
    //   }
    // }
    // return is_planning_ok;
    return false;
  }

  return true;
}

void DiagonalInTrajectoryGenerator::Reset() {
  plan_state_machine_ = INIT;
  slot_manager_.Reset();
  collision_detector_.Reset();
  replan_in_slot_count_ = 0;
  spline_success_ = false;
  is_replan_ = false;
  is_finished_ = false;
}

void DiagonalInTrajectoryGenerator::GeneratePlanningOutput(
    const bool plan_success, PlanningOutput* const planning_output) {
  if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
    return;
  }

  if (plan_success) {
    planning_output->Clear();

    auto trajectory = planning_output->mutable_trajectory();
    trajectory->set_available(true);

    trajectory->set_trajectory_type(
        Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);

    const auto& plan_result = dubins_planner_.GetOutput();

    for (const auto& path_point : plan_result.path_point_vec) {
      auto trajectory_point = trajectory->add_trajectory_points();

      trajectory_point->set_x(path_point.pos.x());
      trajectory_point->set_y(path_point.pos.y());
      trajectory_point->set_heading_yaw(path_point.heading);
      trajectory_point->set_v(0.5);
    }

    // set plan gear cmd
    auto gear_command = planning_output->mutable_gear_command();
    gear_command->set_available(true);
    if (plan_result.current_gear_cmd ==
        pnc::dubins_lib::DubinsLibrary::NORMAL) {
      gear_command->set_gear_command_value(
          Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE);
    } else {
      gear_command->set_gear_command_value(
          Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);
    }
  } else {
    if (!simulation_enable_flag_) {
      SetFinishedPlanningOutput(frame_);
    }
  }
}

void DiagonalInTrajectoryGenerator::UpdateManagedParkingFusion(
    const int select_slot_index) {
  // std::cout << "local_view_->parking_fusion_info:"
  //           << local_view_->parking_fusion_info.DebugString() << std::endl;
  // copy parking fusion info
  managed_parking_fusion_info_.CopyFrom(local_view_->parking_fusion_info);

  auto fusion_selected_slot =
      managed_parking_fusion_info_.mutable_parking_fusion_slot_lists(
          select_slot_index);

  auto managed_selected_slot = slot_manager_.GetSelectedSlot(
      managed_parking_fusion_info_.select_slot_id());

  // std::cout << "managed_selected_slot = " <<
  // managed_selected_slot.DebugString()
  //           << std::endl;

  fusion_selected_slot->mutable_corner_points(0)->set_x(
      managed_selected_slot.corner_points().corner_point(0).x());
  fusion_selected_slot->mutable_corner_points(0)->set_y(
      managed_selected_slot.corner_points().corner_point(0).y());

  fusion_selected_slot->mutable_corner_points(1)->set_x(
      managed_selected_slot.corner_points().corner_point(1).x());
  fusion_selected_slot->mutable_corner_points(1)->set_y(
      managed_selected_slot.corner_points().corner_point(1).y());

  fusion_selected_slot->mutable_corner_points(2)->set_x(
      managed_selected_slot.corner_points().corner_point(2).x());
  fusion_selected_slot->mutable_corner_points(2)->set_y(
      managed_selected_slot.corner_points().corner_point(2).y());

  fusion_selected_slot->mutable_corner_points(3)->set_x(
      managed_selected_slot.corner_points().corner_point(3).x());
  fusion_selected_slot->mutable_corner_points(3)->set_y(
      managed_selected_slot.corner_points().corner_point(3).y());
}

const bool DiagonalInTrajectoryGenerator::PathPlanOnceSimulation(
    common::SlotManagementInfo& slot_mangement_info) {
  simulation_enable_flag_ = true;

  // set local view for these two modules for simulation
  // since plan() is not included in pybind simulation
  // uss_oa_.SetLocalView(local_view_);
  // collision_detector_.SetLocalView(local_view_);

  if (local_view_->function_state_machine_info.has_current_state()) {
    current_state_ = local_view_->function_state_machine_info.current_state();
  }

  plan_state_machine_ = simu_param_.force_plan_stm;

  std::cout << "-------------------------------------------- frame start "
               "-------------------------------------------- "
            << std::endl;
  std::cout << "current_status = " << static_cast<int>(current_state_)
            << std::endl;

  auto& origin_parking_fusion_info = local_view_->parking_fusion_info;
  const auto& slots = origin_parking_fusion_info.parking_fusion_slot_lists();

  int select_slot_index = -1;
  size_t selected_slot_id = simu_param_.selected_id_;

  std::cout << "selected_slot_id:" << selected_slot_id << std::endl;
  for (int i = 0;
       i < origin_parking_fusion_info.parking_fusion_slot_lists_size(); ++i) {
    if (selected_slot_id != slots[i].id()) {
      continue;
    }
    if (slots[i].type() ==
            Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
        slots[i].type() ==
            Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      select_slot_index = i;
      std::cout << "diagonal slot selected" << std::endl;
      break;
    }
  }
  if (select_slot_index == -1) {
    std::cout << "selected slot is not diagonal" << std::endl;
    return false;
  }

  // copy parking fusion info
  managed_parking_fusion_info_.CopyFrom(local_view_->parking_fusion_info);

  auto fusion_selected_slot =
      managed_parking_fusion_info_.mutable_parking_fusion_slot_lists(
          select_slot_index);
  // selected_slot_id
  int select_slot_index_slm = -1;
  const auto& slot_info_vec = slot_mangement_info.slot_info_vec();
  for (int i = 0; i < slot_mangement_info.slot_info_vec_size(); ++i) {
    if (selected_slot_id == slot_info_vec[i].id()) {
      select_slot_index_slm = i;
      break;
    }
  }

  if (select_slot_index_slm == -1) {
    std::cout << "selected slot is not diagonal" << std::endl;
    return false;
  }

  auto managed_selected_slot =
      slot_mangement_info.slot_info_vec(select_slot_index_slm);

  fusion_selected_slot->mutable_corner_points(0)->set_x(
      managed_selected_slot.corner_points().corner_point(0).x());
  fusion_selected_slot->mutable_corner_points(0)->set_y(
      managed_selected_slot.corner_points().corner_point(0).y());

  fusion_selected_slot->mutable_corner_points(1)->set_x(
      managed_selected_slot.corner_points().corner_point(1).x());
  fusion_selected_slot->mutable_corner_points(1)->set_y(
      managed_selected_slot.corner_points().corner_point(1).y());

  fusion_selected_slot->mutable_corner_points(2)->set_x(
      managed_selected_slot.corner_points().corner_point(2).x());
  fusion_selected_slot->mutable_corner_points(2)->set_y(
      managed_selected_slot.corner_points().corner_point(2).y());

  fusion_selected_slot->mutable_corner_points(3)->set_x(
      managed_selected_slot.corner_points().corner_point(3).x());
  fusion_selected_slot->mutable_corner_points(3)->set_y(
      managed_selected_slot.corner_points().corner_point(3).y());

  // update measurement
  UpdateMeasurement();

  const bool flag = PathPlanOnce(select_slot_index, &planning_output_);
  GeneratePlanningOutput(flag, &planning_output_);

  return flag;
}

void DiagonalInTrajectoryGenerator::UpdateEgoSlotInfo(const int slot_index) {
  const auto& slot_points =
      managed_parking_fusion_info_.parking_fusion_slot_lists()[slot_index]
          .corner_points();

  const Eigen::Vector2d p0(slot_points[0].x(), slot_points[0].y());
  const Eigen::Vector2d p1(slot_points[1].x(), slot_points[1].y());
  const Eigen::Vector2d p2(slot_points[2].x(), slot_points[2].y());
  const Eigen::Vector2d p3(slot_points[3].x(), slot_points[3].y());

  const auto pM = 0.5 * (p0 + p1);
  const auto v_01 = p1 - p0;
  const auto v_02 = p2 - p0;

  auto n = Eigen::Vector2d(-v_01.y(), v_01.x());

  double k = 1.0;
  if (n.dot(v_02) < 0.0) {
    k = -1.0;
  }

  // std::cout << "pM = \n" << pM << std::endl;

  ego_slot_info_.slot_origin_pos = k * n.normalized() * kNormalSlotLength + pM;
  ego_slot_info_.slot_origin_heading = std::atan2(-n.y(), -n.x());

  // tf init
  g2l_tf_.Init(ego_slot_info_.slot_origin_pos,
               ego_slot_info_.slot_origin_heading);

  l2g_tf_.Init(ego_slot_info_.slot_origin_pos,
               ego_slot_info_.slot_origin_heading);

  ego_slot_info_.ego_pos_slot = g2l_tf_.GetPos(measure_.ego_pos);
  ego_slot_info_.ego_heading_slot = g2l_tf_.GetHeading(measure_.heading);

  // std::cout << "measure_.ego_pos = " << measure_.ego_pos << std::endl;
  // std::cout << "slot_origin_pos_ = \n" << slot_origin_pos_ << std::endl;
  // std::cout << "slot_origin_heading_deg = " << slot_origin_heading_ * 57.3
  //           << std::endl;
  // std::cout << "ego_pos_slot_ = \n" << ego_pos_slot_ << std::endl;
  // std::cout << "ego_heading_slot_deg = " << ego_heading_slot_ * 57.3
  //           << std::endl;
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanOneStep(
    const PlanInput& plan_input, const uint8_t plan_algorithm,
    const uint8_t level) {
  dubins_planner_.SetRadius(plan_input.path_radius);
  dubins_planner_.SetTarget(plan_input.target_pos, plan_input.target_heading);

  // dubins method
  if (plan_algorithm == PlanAlgorithm::DUBINS) {
    // dubins search loop in one tuning step
    for (size_t i = 0; i < DubinsLibrary::CASE_COUNT; ++i) {
      for (size_t j = 0; j < DubinsLibrary::R_S_L + 1; ++j) {
        if (dubins_planner_.Solve(j, i)) {
          if (PathEvaluateOnce(level)) {
            return true;
          }
        }
      }
    }  // dubins loop
  }

  // TODO
  // // line arc method
  // if (plan_algorithm == PlanAlgorithm::LINE_ARC) {
  //   for (size_t i = 0; i < DubinsLibrary::LINEARC_TYPE_COUNT; ++i) {
  //     if (dubins_planner_.Solve(i)) {
  //       if (output.gear_change_count == 0) {
  //         return true;
  //       }
  //     }
  //   }  // linearc loop
  // }

  return false;
}

void DiagonalInTrajectoryGenerator::UpdateDubinsInputByLevel(
    const uint8_t level) {
  if (level == DUBINS_LEVEL_0) {  // LEVEL_0, usually for first try
    plan_input_.target_pos << target_x_init, 0.0;
    plan_input_.target_heading = 0.0;
    plan_input_.path_radius = min_radius;

  } else if (level == DUBINS_LEVEL_1) {
  }
}

const bool DiagonalInTrajectoryGenerator::PathEvaluateOnce(
    const uint8_t level) {
  const auto& output = dubins_planner_.GetOutput();
  if (level == DUBINS_LEVEL_0) {  // LEVEL_0, usually for first try

    // no gear change
    if (output.gear_change_count == 0) {
      return true;
    }

    // once gear change
    if (output.gear_change_count == 1 &&
        output.line_BC.length >= kMinProperBCLength) {
      // TODO: consider heading
      // if (std::fabs(output.dtheta_arc_AB) < 10.0 / 57.3)
      return true;
    }
  } else if (level == DUBINS_LEVEL_1) {
  }

  return false;
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanFuncByLevel(
    const uint8_t level) {
  // step1: update target pose and radius
  UpdateDubinsInputByLevel(level);

  const double max_search_length =
      std::max(min_search_length,
               ego_slot_info_.ego_pos_slot.x() - plan_input_.target_pos.x());

  // priority_queue to get index of shortest output
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                      std::greater<std::pair<int, int>>>
      path_length_queue;

  std::vector<DubinsLibrary::Output> dubins_output_vec;

  bool success_once = false;
  if (level == DUBINS_LEVEL_0) {
    dubins_output_vec.reserve(std::ceil(max_search_length / kLineStep));

    double s = 0.0;
    size_t index = 0;
    while (s < max_search_length) {
      // try linearc method first
      if (DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS, level)) {
        const auto length = dubins_planner_.GetOutput().length;
        if (length < ideal_length_level_0) {
          return true;
        }

        dubins_output_vec.emplace_back(dubins_planner_.GetOutput());
        path_length_queue.push({length, index});
        index++;

        success_once = true;
      }

      plan_input_.target_pos.x() += kLineStep;
      s += kLineStep;
    }
  }

  if (success_once) {
    dubins_planner_.SetOutput(
        dubins_output_vec[path_length_queue.top().second]);
    return true;
  } else {
    return false;
  }
}

void DiagonalInTrajectoryGenerator::PrintDubinsOutput() {
  const auto& output = dubins_planner_.GetOutput();

  // std::cout << "pA = " << output.arc_AB.pA.transpose() << std::endl;
  // std::cout << "pB = " << output.arc_AB.pB.transpose() << std::endl;
  // std::cout << "pC = " << output.arc_CD.pA.transpose() << std::endl;
  // std::cout << "pD = " << output.arc_CD.pB.transpose() << std::endl;
  std::cout << "------------- dubins result " << std::endl;

  std::cout << "path length = " << output.length << std::endl;

  const auto tmp_gear_cmd_vec = Eigen::Vector3d(
      output.gear_cmd_vec[0], output.gear_cmd_vec[1], output.gear_cmd_vec[2]);
  std::cout << "gear_cmd_vec = " << tmp_gear_cmd_vec.transpose() << std::endl;

  std::cout << "gear_change_count = "
            << static_cast<int>(output.gear_change_count) << std::endl;

  std::cout << "is_line_arc = " << output.is_line_arc << std::endl;
  std::cout << "dubins_type = " << static_cast<int>(output.dubins_type)
            << std::endl;
  std::cout << "line_arc_type = " << static_cast<int>(output.line_arc_type)
            << std::endl;
}

const bool DiagonalInTrajectoryGenerator::CheckFinish() const {
  if (simulation_enable_flag_) {
    return false;
  }

  const bool parking_success =
      (std::fabs(ego_slot_info_.ego_pos_slot.x() - terminal_target_x) <=
           kMaxXOffset &&
       std::fabs(ego_slot_info_.ego_pos_slot.y() - 0.0) <= kMaxYOffset &&
       std::fabs(ego_slot_info_.ego_heading_slot) <= kMaxThetaOffset &&
       measure_.static_flag);

  const bool parking_failed =
      (std::fabs(ego_slot_info_.ego_pos_slot.x() - terminal_target_x) < 0.5) &&
      measure_.standstill_timer_by_pos > 3.0;

  return parking_success || parking_failed;
}

const bool DiagonalInTrajectoryGenerator::CheckReplan(
    PlanningOutput* const planning_output) {
  if (simulation_enable_flag_ && simu_param_.force_planning_) {
    std::cout << "tune force_planning on!" << std::endl;
    return true;
  }

  if (IsReplanEachFrame(local_view_->function_state_machine_info)) {
    std::cout << "apa is not active!" << std::endl;
    return true;
  }

  if (!planning_output->has_trajectory() ||
      planning_output->trajectory().trajectory_points_size() == 0) {
    std::cout << "no trajectory!" << std::endl;
    return true;
  }

  if (CheckIfNearTerminalPoint()) {
    std::cout << "close to target!" << std::endl;
    return true;
  }

  planning_output->mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);

  return false;
}

const bool DiagonalInTrajectoryGenerator::UpdateSplineGlobal() {
  const auto& output = dubins_planner_.GetOutput();
  const auto N = output.path_point_vec.size();
  if (N < 3) {
    return false;
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;

  x_vec.reserve(N);
  y_vec.reserve(N);
  s_vec.reserve(N);

  x_vec.clear();
  y_vec.clear();
  s_vec.clear();

  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < N - 1; ++i) {
    x_vec.emplace_back(output.path_point_vec[i].pos.x());
    y_vec.emplace_back(output.path_point_vec[i].pos.y());
    s_vec.emplace_back(s);

    ds = std::max(
        (output.path_point_vec[i].pos - output.path_point_vec[i + 1].pos)
            .norm(),
        1e-2);
    s += ds;
  }

  x_vec.back() = output.path_point_vec.back().pos.x();
  y_vec.back() = output.path_point_vec.back().pos.y();
  s_vec.back() = s;

  x_s_spline_l_.set_points(s_vec, x_vec);
  y_s_spline_l_.set_points(s_vec, y_vec);

  current_path_length_ = s_vec.back();

  return true;
}

const bool DiagonalInTrajectoryGenerator::PathPlanOnce(
    const int slot_index, PlanningOutput* const planning_output) {
  // update slot info and target point in both global and slot coordinate
  UpdateEgoSlotInfo(slot_index);

  // check if finish
  if (CheckFinish()) {
    std::cout << "apa is finished" << std::endl;
    SetFinishedPlanningOutput(frame_);
    return true;
  }

  // check if replan
  if (!CheckReplan(planning_output)) {
    is_replan_ = false;
    return true;
  } else {
    is_replan_ = true;
  }

  // start apa dubins planning
  dubins_planner_.SetStart(ego_slot_info_.ego_pos_slot,
                           ego_slot_info_.ego_heading_slot);

  // main loop for dubins planning
  if (plan_state_machine_ == INIT) {
    if (DubinsPlanFuncByLevel(DUBINS_LEVEL_0)) {
      if (dubins_planner_.GetOutput().gear_change_count == 0 &&
          dubins_planner_.GetOutput().current_gear_cmd ==
              pnc::dubins_lib::DubinsLibrary::REVERSE) {
        plan_state_machine_ = FINAL;
      }
    } else {
      return false;
    }
  } else if (plan_state_machine_ == FINAL) {
    if (DubinsPlanFuncByLevel(DUBINS_LEVEL_0)) {
      plan_state_machine_ = INIT;
    } else {
      return false;
    }
  }

  // enable complete path when simulation
  bool is_complete_path = false;
  if (simulation_enable_flag_) {
    is_complete_path = simu_param_.is_complete_path;
    dubins_planner_.Sampling(simu_param_.sample_ds, is_complete_path);
  } else {
    dubins_planner_.Sampling(kLineStep, is_complete_path);
  }

  if (plan_state_machine_ == FINAL) {
    const double extend_s =
        dubins_planner_.GetOutput().arc_CD.pB.x() - terminal_target_x;
    dubins_planner_.Extend(extend_s);

    // std::cout << "plan_input_.target_pos = "
    //           << plan_input_.target_pos.transpose() << std::endl;
    // std::cout << "extend_s = " << extend_s << std::endl;
  }

  dubins_planner_.Transform(l2g_tf_);

  // update spline for projection point dist calculation
  // note that update after transform
  spline_success_ = UpdateSplineGlobal();

  PrintDubinsOutput();

  std::cout << "diagonal replan triggered, plan statemachine = "
            << static_cast<int>(plan_state_machine_) << std::endl;

  // if (replan_in_slot_count_ >= 2) {
  //   std::cout << "replan cnts in slot are more than 1" << std::endl;
  //   plan_state_machine_ = FINISH;
  //   return false;
  // }

  return true;
}

void DiagonalInTrajectoryGenerator::Log() const {
  JSON_DEBUG_VALUE("is_replan", is_replan_)
}

const bool DiagonalInTrajectoryGenerator::CheckIfNearTerminalPoint() const {
  if (spline_success_) {
    if (remain_dist_ < 0.1 && measure_.static_flag) {
      return true;
    }
  }

  return false;
}

void DiagonalInTrajectoryGenerator::UpdateMeasurement() {
  const auto& pose = local_view_->localization_estimate.pose();
  const auto current_pos =
      Eigen::Vector2d(pose.local_position().x(), pose.local_position().y());

  // check standstill by pos
  if (!simulation_enable_flag_) {
    if ((measure_.ego_pos - current_pos).norm() < kEps) {
      measure_.standstill_timer_by_pos += plan_time;
    } else {
      measure_.standstill_timer_by_pos = 0.0;
    }
  }

  measure_.ego_pos = current_pos;
  measure_.heading = pose.heading();
  measure_.v_ego = local_view_->vehicle_service_output_info.vehicle_speed();

  // check standstill by velocity
  if (!simulation_enable_flag_) {
    if (std::fabs(measure_.v_ego) < kStanstillSpd) {
      measure_.standstill_timer += plan_time;
    } else {
      measure_.standstill_timer = 0.0;
    }
  }

  if (!simulation_enable_flag_) {
    measure_.static_flag = measure_.standstill_timer >= 0.5 &&
                           measure_.standstill_timer_by_pos > 0.5;
  } else {
    measure_.static_flag = true;
  }

  if (spline_success_) {
    pnc::spline::Projection proj;
    proj.CalProjectionPoint(x_s_spline_l_, y_s_spline_l_, 0.0, max_path_length,
                            measure_.ego_pos);
    remain_dist_ = current_path_length_ - proj.GetOutput().s_proj;
    std::cout << "remain_dist = " << remain_dist_ << std::endl;
  } else {
    remain_dist_ = 5.01;
  }
}

}  // namespace apa_planner
}  // namespace planning
