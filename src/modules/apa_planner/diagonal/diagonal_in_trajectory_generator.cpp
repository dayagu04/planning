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

#include "Platform_Types.h"
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
#include "math_lib.h"
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
constexpr double kMaxYOffset = 0.1;
constexpr double kMaxThetaOffset = 2.8 / 57.3;
constexpr double kMockedObjYOffset = 4.0;

static const double kNormalSlotLength = 4.8;
static const double target_x_init = 1.5;
static const double min_radius = 5.5;
static const double min_radius_final = 5.2;
static const double kLineStep = 0.1;
static const double kMinProperBCLength = 0.3;
static const double plan_time = 0.1;
static const double terminal_target_x = 1.1;
static const double max_path_length = 20.0;
static const double min_search_length = 6.0;
static const double path_sample_ds = 0.05;
static const double safe_uss_remain_dist = 0.35;
static const double stuck_failed_time = 6.0;
static const double stuck_replan_time = 4.0;
static const double min_replan_remain_dist =
    0.2;  // in control, this value must be smaller
static const uint8_t max_gear_change_count = 6;

static const double kSublaneWidth = 8.5;
static const double kRightSublaneLength = 11.0;
static const double kLeftSublaneLength = 11.0;

// vehicle params
static const double kFrontOverhanging = 0.924;
static const double kWheelBase = 2.7;
static const double kVehicleWidth = 1.89;

static const Eigen::Vector2d kEgoFLCornerVec((kFrontOverhanging + kWheelBase),
                                             0.5 * kVehicleWidth);
static const Eigen::Vector2d kEgoFRCornerVec((kFrontOverhanging + kWheelBase),
                                             -0.5 * kVehicleWidth);

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

  // set local view to uss oa
  uss_oa_.SetLocalView(local_view_);

  // update measurement
  UpdateMeasurement();

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

  bool generate_trajectory = false;

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

    // plan once
    PathPlanOnce(select_slot_index, planning_output);

    // generate planning output
    GeneratePlanningOutput(planning_output);

    // update uss oa
    uss_oa_.Update(planning_output);

    // generate planning output by uss remain dist
    GeneratePlanningOutputByUssOA(planning_output);

    generate_trajectory = is_plan_success_;
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

    generate_trajectory = false;
  }

  // log by json
  Log();

  return generate_trajectory;
}

void DiagonalInTrajectoryGenerator::Reset() {
  plan_state_machine_ = INIT;
  slot_manager_.Reset();
  collision_detector_.Reset();
  replan_in_slot_count_ = 0;
  spline_success_ = false;
  is_replan_ = false;
  is_finished_ = false;
  dubins_iter_count_ = 0;
  gear_change_count_ = max_gear_change_count;
  path_level_ = DUBINS_LEVEL_NONE;
  is_plan_success_ = false;
  terminal_err_.Set(Eigen::Vector2d(1.0, 1.0), 0.5);
  target_err_.Set(Eigen::Vector2d(1.0, 1.0), 0.5);
  sublane_left_length_ = kLeftSublaneLength;
  sublane_right_length_ = kRightSublaneLength;
  sublane_width_ = kSublaneWidth;
  stuck_time_ = 0.0;
}

void DiagonalInTrajectoryGenerator::GeneratePlanningOutputByUssOA(
    PlanningOutput* const planning_output) {
  // std::cout << "planning_output->trajectory()->trajectory_points_size()"
  //           << planning_output->trajectory().trajectory_points_size()
  //           << std::endl;
  if (planning_output->trajectory().trajectory_points_size() < 1) {
    return;
  }
  if (spline_success_) {
    planning_output->mutable_trajectory()
        ->mutable_trajectory_points(0)
        ->set_distance(uss_oa_.GetRemainDist() - safe_uss_remain_dist);
  } else {
    planning_output->mutable_trajectory()
        ->mutable_trajectory_points(0)
        ->set_distance(100.0);
  }
  // std::cout << "Uss RemainDist = " << uss_oa_.GetRemainDist() << std::endl;
  // std::cout << "Plan RemainDist = " << remain_dist_ << std::endl;
  // std::cout << "current_path_length_ = " << current_path_length_ <<
  // std::endl;
}

void DiagonalInTrajectoryGenerator::GeneratePlanningOutput(
    PlanningOutput* const planning_output) {
  if (!simulation_enable_flag_) {
    // when finished
    if (is_finished_) {
      SetFinishedPlanningOutput(frame_);
      return;
    }

    // not active
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      return;
    }
  }

  // plan suceess
  if (is_plan_success_) {
    std::cout << "------------- ready to send traj" << std::endl;
    planning_output->Clear();

    planning_output->mutable_planning_status()->set_apa_planning_status(
        ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);

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
      return;
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

  for (size_t i = 0; i < fusion_selected_slot->corner_points_size(); ++i) {
    fusion_selected_slot->mutable_corner_points(i)->set_x(
        managed_selected_slot.corner_points().corner_point(i).x());
    fusion_selected_slot->mutable_corner_points(i)->set_y(
        managed_selected_slot.corner_points().corner_point(i).y());
  }
}

const bool DiagonalInTrajectoryGenerator::PathPlanOnceSimulation(
    common::SlotManagementInfo& slot_mangement_info) {
  simulation_enable_flag_ = true;

  // set local view for these two modules for simulation
  // since plan() is not included in pybind simulation
  uss_oa_.SetLocalView(local_view_);
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

  for (size_t i = 0; i < fusion_selected_slot->corner_points_size(); ++i) {
    fusion_selected_slot->mutable_corner_points(i)->set_x(
        managed_selected_slot.corner_points().corner_point(i).x());
    fusion_selected_slot->mutable_corner_points(i)->set_y(
        managed_selected_slot.corner_points().corner_point(i).y());
  }

  // update measurement
  UpdateMeasurement();

  PathPlanOnce(select_slot_index, &planning_output_);
  GeneratePlanningOutput(&planning_output_);

  uss_oa_.Update(&planning_output_);

  GeneratePlanningOutputByUssOA(&planning_output_);

  return is_plan_success_;
}

void DiagonalInTrajectoryGenerator::UpdateEgoSlotInfo(const int slot_index) {
  const auto& slot_points =
      managed_parking_fusion_info_.parking_fusion_slot_lists()[slot_index]
          .corner_points();

  const Eigen::Vector2d p0(slot_points[0].x(), slot_points[0].y());
  const Eigen::Vector2d p1(slot_points[1].x(), slot_points[1].y());
  const Eigen::Vector2d p2(slot_points[2].x(), slot_points[2].y());
  const Eigen::Vector2d p3(slot_points[3].x(), slot_points[3].y());

  ego_slot_info_.slot_width = (p0 - p1).norm();

  const auto pM01 = 0.5 * (p0 + p1);
  const auto pM23 = 0.5 * (p2 + p3);

  const auto n = pM01 - pM23;

  ego_slot_info_.slot_origin_pos = pM01 - kNormalSlotLength * n.normalized();
  ego_slot_info_.slot_origin_heading = std::atan2(n.y(), n.x());

  // global2slot tf init
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

  // update terminal error
  terminal_err_.Set(
      Eigen::Vector2d(ego_slot_info_.ego_pos_slot.x() - terminal_target_x,
                      ego_slot_info_.ego_pos_slot.y() - 0.0),
      ego_slot_info_.ego_heading_slot);

  std::cout << "lon_err = " << terminal_err_.pos.x() << std::endl;
  std::cout << "lat_err = " << terminal_err_.pos.y() << std::endl;
  std::cout << "heading_err_deg = " << terminal_err_.heading * 57.3
            << std::endl;
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanOneStep(
    const PlanInput& plan_input, const uint8_t plan_algorithm,
    const uint8_t level) {
  dubins_planner_.SetRadius(plan_input.path_radius);
  dubins_planner_.SetTarget(plan_input.target_pos, plan_input.target_heading);
  plan_algorithm_ = plan_algorithm;

  // dubins method
  if (plan_algorithm == PlanAlgorithm::DUBINS) {
    // dubins search loop in one tuning step
    for (size_t i = 0; i < DubinsLibrary::CASE_COUNT; ++i) {
      for (size_t j = 0; j < DubinsLibrary::DUBINS_TYPE_COUNT; ++j) {
        if (dubins_planner_.Solve(j, i)) {
          if (PathEvaluateOnce(level)) {
            return true;
          }
        }
      }
    }  // dubins loop
  }

  // line arc method
  if (plan_algorithm == PlanAlgorithm::LINE_ARC) {
    for (size_t i = 0; i < DubinsLibrary::LINEARC_TYPE_COUNT; ++i) {
      if (dubins_planner_.Solve(i)) {
        if (PathEvaluateOnce(level)) {
          return true;
        }
      }
    }  // linearc loop
  }

  return false;
}

void DiagonalInTrajectoryGenerator::UpdateDubinsInputByLevel(
    const uint8_t level) {
  // set radius
  if (plan_state_machine_ == FINAL) {
    plan_input_.path_radius = min_radius_final;
  } else {
    plan_input_.path_radius = min_radius;
  }

  if (level ==
      DUBINS_LEVEL_ZERO_GEAR_CHANGE) {  // LEVEL_0, usually for first try
    plan_input_.target_pos << target_x_init, 0.0;
    plan_input_.target_heading = 0.0;

  } else if (level == DUBINS_LEVEL_ONCE_GEAR_CHANGE) {
  }
}

const bool DiagonalInTrajectoryGenerator::CheckIfCrossSublane() const {
  const auto& output = dubins_planner_.GetOutput();
  pnc::dubins_lib::DubinsLibrary::PathPoint check_point;

  // if ego car drive forward in BC segment,check Point C collision
  if (output.path_available && output.gear_cmd_vec.size() > 0) {
    if (output.gear_cmd_vec[1] == 1) {
      check_point.Set(output.arc_CD.pA, output.arc_CD.headingA);
    } else {
      check_point.Set(output.arc_AB.pB, output.arc_AB.headingB);
    }
    return CheckIfCrossSublane(check_point);
  } else {
    return true;
  }
}

const bool DiagonalInTrajectoryGenerator::CheckIfCrossSublane(
    DubinsLibrary::PathPoint ego_point_in_slot) const {
  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(ego_point_in_slot.pos, ego_point_in_slot.heading);

  Eigen::Vector2d ego_fl_corner_in_slot = ego2slot.GetPos(kEgoFLCornerVec);
  Eigen::Vector2d ego_fr_corner_in_slot = ego2slot.GetPos(kEgoFRCornerVec);

  // check if ego x coor cross sublane width
  const double ego_corner_x_limit =
      std::max(ego_fl_corner_in_slot.x(), ego_fr_corner_in_slot.x());

  if (ego_corner_x_limit > sublane_width_ + kNormalSlotLength) {
    return true;
  }

  // ego heading is less than zero, check right sublane length
  double ego_corner_y_limit = 0.0;
  if (ego_point_in_slot.heading < 0.0) {
    ego_corner_y_limit =
        std::min(ego_fl_corner_in_slot.y(), ego_fr_corner_in_slot.y());

    if (ego_corner_y_limit <
        -0.5 * ego_slot_info_.slot_width - sublane_right_length_) {
      return true;
    }
  } else {  // ego heading is more than zero, check left sublane length
    ego_corner_y_limit =
        std::max(ego_fl_corner_in_slot.y(), ego_fr_corner_in_slot.y());
    if (ego_corner_y_limit >
        0.5 * ego_slot_info_.slot_width + sublane_left_length_) {
      return true;
    }
  }
  return false;
}

const bool DiagonalInTrajectoryGenerator::PathEvaluateOnce(
    const uint8_t level) {
  const auto& output = dubins_planner_.GetOutput();

  // check if the ego car cross sublane in slot system(dubins result is now in
  // slot system)
  if (CheckIfCrossSublane()) {
    return false;
  }

  if (level ==
      DUBINS_LEVEL_ZERO_GEAR_CHANGE) {  // LEVEL_0, usually for first try

    const auto& is_line_arc = output.is_line_arc;

    // no gear change: force no gear change in final
    if (plan_state_machine_ == FINAL) {
      const bool is_toward_terminal =
          ego_slot_info_.ego_pos_slot.x() > plan_input_.target_pos.x();
      if (output.gear_change_count == 0 && is_toward_terminal) {
        if (is_line_arc) {
          // const auto arc_length = output.arc_AB.length +
          // output.arc_CD.length; arc_length / output.line_arc_radius < 2.2
          // / 57.3 ||
          if (output.line_arc_radius > min_radius_final) {
            return true;
          } else {
            return false;
          }
        } else {
          return true;
        }
      }
    }

    if (plan_state_machine_ == INIT) {
      if (output.gear_change_count == 1) {
        if (output.line_BC.length >= kMinProperBCLength &&
            is_line_arc == false) {
          return true;
        }
      }
    }
  } else if (level == DUBINS_LEVEL_ONCE_GEAR_CHANGE) {
  }

  return false;
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanFunc(const uint8_t level) {
  path_level_ = level;
  dubins_iter_count_ = 0;
  // step1: update target pose and radius
  UpdateDubinsInputByLevel(level);

  const double max_search_length =
      std::max(min_search_length,
               ego_slot_info_.ego_pos_slot.x() - plan_input_.target_pos.x());

  // priority_queue to get index of shortest output
  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      path_length_queue;

  std::vector<DubinsLibrary::Output> dubins_output_vec;
  dubins_output_vec.clear();
  dubins_output_vec.reserve(std::ceil(max_search_length / kLineStep));

  bool success_once = false;
  double s = 0.0;
  size_t index = 0;
  if (level == DUBINS_LEVEL_ZERO_GEAR_CHANGE) {
    // try dubins method first
    s = 0.0;
    index = 0;
    while (s < max_search_length) {
      if (DubinsPlanOneStep(plan_input_, PlanAlgorithm::LINE_ARC, level)) {
        const auto length = dubins_planner_.GetOutput().length;
        dubins_output_vec.emplace_back(dubins_planner_.GetOutput());
        path_length_queue.push({length, index});
        index++;

        success_once = true;
      }

      plan_input_.target_pos.x() += kLineStep;
      s += kLineStep;

      dubins_iter_count_++;
    }

    if (success_once) {
      std::cout << "--------line arc success!" << std::endl;
    }

    if (!success_once) {
      path_length_queue.empty();
      dubins_output_vec.clear();
      s = 0.0;
      index = 0;
      dubins_iter_count_ = 0;
      UpdateDubinsInputByLevel(level);

      // then try line arc method
      while (s < max_search_length) {
        if (DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS, level)) {
          const auto length = dubins_planner_.GetOutput().length;
          dubins_output_vec.emplace_back(dubins_planner_.GetOutput());
          path_length_queue.push({length, index});
          index++;

          success_once = true;
        }

        plan_input_.target_pos.x() += kLineStep;
        s += kLineStep;

        dubins_iter_count_++;
      }

      if (success_once) {
        std::cout << "--------dubins success!" << std::endl;
      }
    }
  }

  if (success_once) {
    dubins_planner_.SetOutput(
        dubins_output_vec[path_length_queue.top().second]);

    // std::cout << "path_length_queue.top() = " <<
    // path_length_queue.top().second
    //           << std::endl;

    return true;
  } else {
    std::cout << "--------all failed!" << std::endl;
    return false;
  }
}

void DiagonalInTrajectoryGenerator::PrintDubinsOutput() {
  const auto& output = dubins_planner_.GetOutput();

  std::cout << "pA = " << output.arc_AB.pA.transpose() << std::endl;
  std::cout << "pB = " << output.arc_AB.pB.transpose() << std::endl;
  std::cout << "pC = " << output.arc_CD.pA.transpose() << std::endl;
  std::cout << "pD = " << output.arc_CD.pB.transpose() << std::endl;
  std::cout << "------------------------------------- dubins result "
            << std::endl;

  std::cout << "target_pos = "
            << dubins_planner_.GetOutput().arc_CD.pB.transpose()
            << ", target_heading_deg = "
            << dubins_planner_.GetOutput().arc_CD.headingB * 57.3 << std::endl;

  std::cout << "ego_pos = " << ego_slot_info_.ego_pos_slot.transpose()
            << ", ego_heading_deg = " << ego_slot_info_.ego_heading_slot * 57.3
            << std::endl;

  std::cout << "path length = " << output.length << std::endl;

  std::cout << "AB_length = " << output.arc_AB.length
            << ", BC_length = " << output.line_BC.length
            << ", CD_length = " << output.arc_CD.length << std::endl;

  std::cout << "radius = " << plan_input_.path_radius << std::endl;

  const auto tmp_gear_cmd_vec = Eigen::Vector3d(
      output.gear_cmd_vec[0], output.gear_cmd_vec[1], output.gear_cmd_vec[2]);

  std::cout << "gear_cmd_vec = " << tmp_gear_cmd_vec.transpose() << std::endl;

  std::cout << "gear_change_count = "
            << static_cast<int>(output.gear_change_count) << std::endl;

  std::cout << "is_line_arc = " << output.is_line_arc << std::endl;
  std::cout << "line_arc_radius = " << output.line_arc_radius << std::endl;

  std::cout << "dubins_type = " << static_cast<int>(output.dubins_type)
            << std::endl;
  std::cout << "case_type = " << static_cast<int>(output.case_type)
            << std::endl;

  std::cout << "line_arc_type = " << static_cast<int>(output.line_arc_type)
            << std::endl;
}

const bool DiagonalInTrajectoryGenerator::CheckFinish() {
  const bool parking_success =
      std::fabs(terminal_err_.pos.x()) < kMaxXOffset &&
      std::fabs(terminal_err_.pos.y()) <= kMaxYOffset &&
      std::fabs(terminal_err_.heading) <= kMaxThetaOffset &&
      measure_.static_flag;

  // will be moved to other place
  if (current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL &&
      measure_.static_flag) {
    stuck_time_ += plan_time;
  } else {
    stuck_time_ = 0.0;
  }

  const bool parking_failed = stuck_time_ > stuck_failed_time;

  is_finished_ = parking_success || parking_failed;

  return is_finished_;
}

const bool DiagonalInTrajectoryGenerator::CheckReplan(
    PlanningOutput* const planning_output) {
  if (simulation_enable_flag_ && simu_param_.force_planning_) {
    std::cout << "tune force_planning on!" << std::endl;
    return true;
  }

  if (!simulation_enable_flag_) {
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

    if (CheckIfReplanByStuck()) {
      std::cout << "replan by stuck!" << std::endl;
      return true;
    }
  } else {
    if (simu_param_.force_planning_) {
      std::cout << "tune force_planning on!" << std::endl;
      return true;
    }
  }

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

  x_vec.reserve(N + 1);
  y_vec.reserve(N + 1);
  s_vec.reserve(N + 1);

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

  x_vec.emplace_back(output.path_point_vec.back().pos.x());
  y_vec.emplace_back(output.path_point_vec.back().pos.y());
  s_vec.emplace_back(s);

  current_path_length_ = s_vec.back();

  // extend end point when remain dist < zero
  const auto diff_vec =
      Eigen::Vector2d(x_vec[N - 1] - x_vec[N - 2], y_vec[N - 1] - y_vec[N - 2]);

  const double extend_s = 1.0;

  const auto extend_p = Eigen::Vector2d(x_vec.back(), y_vec.back()) +
                        extend_s * diff_vec.normalized();

  x_vec.emplace_back(extend_p.x());
  y_vec.emplace_back(extend_p.y());
  s_vec.emplace_back(current_path_length_ + extend_s);

  x_s_spline_l_.set_points(s_vec, x_vec);
  y_s_spline_l_.set_points(s_vec, y_vec);

  return true;
}

const bool DiagonalInTrajectoryGenerator::PathPlanCoreIteration() {
  // start apa dubins planning
  dubins_planner_.SetStart(ego_slot_info_.ego_pos_slot,
                           ego_slot_info_.ego_heading_slot);

  // main loop for dubins planning
  bool plan_success = false;

  // first dubins iteration: try to plan with final: zero gear change
  std::cout << "----------------try final plan first!" << std::endl;
  plan_state_machine_ = FINAL;
  if (DubinsPlanFunc(DUBINS_LEVEL_ZERO_GEAR_CHANGE)) {
    plan_success = (dubins_planner_.GetOutput().gear_change_count == 0 &&
                    dubins_planner_.GetOutput().current_gear_cmd ==
                        pnc::dubins_lib::DubinsLibrary::REVERSE);

    // TODO: obstacle detection
  }

  if (plan_success) {
    std::cout << "--final plan success!" << std::endl;
  } else {
    std::cout << "--final plan failed!" << std::endl;
  }

  // second dubins iteration: try to plan again with init: once gear change
  if (!plan_success) {
    plan_state_machine_ = INIT;
    std::cout << "----------------try init plan!" << std::endl;
    if (DubinsPlanFunc(DUBINS_LEVEL_ZERO_GEAR_CHANGE)) {
      std::cout << "--init plan success!" << std::endl;
      plan_success = true;
    } else {
      std::cout << "--init plan failed!" << std::endl;
    }
    // TODO: obstacle detection
  }

  // set is_plan_success
  is_plan_success_ = plan_success;

  return plan_success;
}

void DiagonalInTrajectoryGenerator::PathPlanOnce(
    const int slot_index, PlanningOutput* const planning_output) {
  // update slot info and target point in both global and slot coordinate
  UpdateEgoSlotInfo(slot_index);

  // check if finish
  if (CheckFinish()) {
    std::cout << "apa is finished" << std::endl;
    return;
  }

  // check if replan
  is_replan_ = CheckReplan(planning_output);
  if (!is_replan_) {
    return;
  }

  // run plan core
  if (!PathPlanCoreIteration()) {
    return;
  }

  // enable complete path when simulation
  bool is_complete_path = false;
  if (simulation_enable_flag_) {
    is_complete_path = simu_param_.is_complete_path;
    dubins_planner_.Sampling(simu_param_.sample_ds, is_complete_path);
  } else {
    dubins_planner_.Sampling(path_sample_ds, is_complete_path);
  }

  double extend_s = 0.0;
  if (plan_state_machine_ == FINAL) {
    extend_s = dubins_planner_.GetOutput().arc_CD.pB.x() - terminal_target_x;
    dubins_planner_.Extend(extend_s);

    // std::cout << "plan_input_.target_pos = "
    //           << plan_input_.target_pos.transpose() << std::endl;
  }

  std::cout << "extend_s = " << extend_s << std::endl;

  dubins_planner_.Transform(l2g_tf_);

  // update spline for projection point dist calculation
  // note that must update after transform
  spline_success_ = UpdateSplineGlobal();

  // record gear change count, must be after successful path plan
  gear_change_count_ = dubins_planner_.GetOutput().gear_change_count;

  // print dubins output for debug
  PrintDubinsOutput();

  std::cout << "diagonal replan triggered, plan statemachine = "
            << static_cast<int>(plan_state_machine_) << std::endl;
}

void DiagonalInTrajectoryGenerator::Log() const {
  JSON_DEBUG_VALUE("is_replan", is_replan_)
  JSON_DEBUG_VALUE("is_finished", is_finished_)
  JSON_DEBUG_VALUE("dubins_type", dubins_planner_.GetOutput().dubins_type)
  JSON_DEBUG_VALUE("case_type", dubins_planner_.GetOutput().case_type)

  JSON_DEBUG_VALUE("gear_change_count",
                   dubins_planner_.GetOutput().gear_change_count)

  JSON_DEBUG_VALUE("path_length", dubins_planner_.GetOutput().length)
  JSON_DEBUG_VALUE("plan_state_machine", plan_state_machine_)

  JSON_DEBUG_VALUE("AB_length", dubins_planner_.GetOutput().arc_AB.length)
  JSON_DEBUG_VALUE("BC_length", dubins_planner_.GetOutput().line_BC.length)
  JSON_DEBUG_VALUE("CD_length", dubins_planner_.GetOutput().arc_CD.length)

  const std::vector<double> gear_cmd_vec = {
      static_cast<double>(dubins_planner_.GetOutput().gear_cmd_vec[0]),
      static_cast<double>(dubins_planner_.GetOutput().gear_cmd_vec[1]),
      static_cast<double>(dubins_planner_.GetOutput().gear_cmd_vec[2])};

  JSON_DEBUG_VECTOR("gear_cmd_vec", gear_cmd_vec, 1)
  JSON_DEBUG_VALUE("dubins_iter_count", dubins_iter_count_)

  JSON_DEBUG_VALUE("terminal_lon_err", terminal_err_.pos.x())
  JSON_DEBUG_VALUE("terminal_lat_err", terminal_err_.pos.y())
  JSON_DEBUG_VALUE("terminal_heading", terminal_err_.heading)

  JSON_DEBUG_VALUE("target_pos_x", dubins_planner_.GetOutput().arc_CD.pB.x())
  JSON_DEBUG_VALUE("target_pos_y", dubins_planner_.GetOutput().arc_CD.pB.y())
  JSON_DEBUG_VALUE("target_heading",
                   dubins_planner_.GetOutput().arc_CD.headingB)

  JSON_DEBUG_VALUE("ego_pos_slot_x", ego_slot_info_.ego_pos_slot.x())
  JSON_DEBUG_VALUE("ego_pos_slot_y", ego_slot_info_.ego_pos_slot.y())
  JSON_DEBUG_VALUE("ego_heading_slot", ego_slot_info_.ego_heading_slot)

  JSON_DEBUG_VALUE("stuck_time", stuck_time_)
  JSON_DEBUG_VALUE("standstill_timer_by_pos", measure_.standstill_timer_by_pos)
  JSON_DEBUG_VALUE("standstill_timer", measure_.standstill_timer)
  JSON_DEBUG_VALUE("static_flag", measure_.static_flag)

  JSON_DEBUG_VALUE("sublane_left_length", sublane_left_length_)
  JSON_DEBUG_VALUE("sublane_right_length", sublane_right_length_)
  JSON_DEBUG_VALUE("sublane_width", sublane_width_)

  JSON_DEBUG_VALUE("remain_dist", remain_dist_)
}

const bool DiagonalInTrajectoryGenerator::CheckIfNearTerminalPoint() const {
  if (spline_success_) {
    if (remain_dist_ < min_replan_remain_dist && measure_.static_flag) {
      return true;
    }
  }

  return false;
}

const bool DiagonalInTrajectoryGenerator::CheckIfReplanByStuck() const {
  return stuck_time_ > stuck_replan_time;
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
  } else {
    measure_.standstill_timer_by_pos = 0.0;
  }

  measure_.ego_pos = current_pos;
  measure_.heading = pose.heading();
  measure_.v_ego = local_view_->vehicle_service_output_info.vehicle_speed();

  if (simulation_enable_flag_) {
    sublane_left_length_ = simu_param_.sublane_left_length;
    sublane_right_length_ = simu_param_.sublane_right_length;
    sublane_width_ = simu_param_.sublane_width;
  }

  // check standstill by velocity
  if (!simulation_enable_flag_) {
    if (std::fabs(measure_.v_ego) < kStanstillSpd) {
      measure_.standstill_timer += plan_time;
    } else {
      measure_.standstill_timer = 0.0;
    }
  }

  // static flag
  if (!simulation_enable_flag_) {
    measure_.static_flag = measure_.standstill_timer >= 0.5 &&
                           measure_.standstill_timer_by_pos > 0.5;
  } else {
    measure_.static_flag = std::fabs(measure_.v_ego) < kStanstillSpd;
  }

  if (spline_success_) {
    pnc::spline::Projection proj;
    proj.CalProjectionPoint(x_s_spline_l_, y_s_spline_l_, 0.0, max_path_length,
                            measure_.ego_pos);

    if (proj.GetOutput().success) {
      remain_dist_ = current_path_length_ - proj.GetOutput().s_proj;
    } else {
      remain_dist_ = 5.01;
      std::cout << "projection calculation error!" << std::endl;
    }

    std::cout << "remain_dist = " << remain_dist_ << std::endl;
  } else {
    remain_dist_ = 5.01;
  }
}

}  // namespace apa_planner
}  // namespace planning
