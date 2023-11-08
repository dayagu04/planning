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
#include "common.pb.h"
#include "common/apa_cos_sin.h"
#include "common/apa_utils.h"
#include "common/planning_log_helper.h"
#include "common/vehicle_param_helper.h"
#include "debug_info_log.h"
#include "dubins_lib/dubins_lib.h"
#include "dubins_lib/geometry_math.h"
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
constexpr double kStanstillSpd = 0.082;
constexpr double kRemainingDisThreshold = 0.2;
constexpr uint64_t kMinStandstillTime = 500;  // ms
constexpr uint64_t kMinPosUnchangedCount = 5;
constexpr double kMaxLonOffset = 0.2;
constexpr double kMaxLatOffset = 0.1;
constexpr double kMaxHeadingOffset = 2.8 / 57.3;
constexpr double kMockedObjYOffset = 4.0;

static const double kNormalSlotLength = 4.8;
static const double target_y_offset = -0.1;
static const double target_x_init = 1.5;
static const double min_radius = 5.5;
static const double min_radius_final = 5.2;
static const double kLineStep = 0.1;
static const double kMinProperLength = 0.5;
static const double plan_time = 0.1;
static const double terminal_target_x = 1.4;
static const double max_path_length = 20.0;
static const double min_search_length = 6.0;
static const double path_sample_ds = 0.05;
static const double collision_check_sample_ds = 0.5;
static const double safe_uss_remain_dist = 0.35;
static const double stuck_failed_time = 6.0;
static const double stuck_replan_time = 4.0;
static const double slot_width_offset_empty = 0.5;
static const double min_replan_remain_dist =
    0.2;  // in control, this value must be smaller
static const double standard_slot_length = 5.2;
static const uint8_t max_gear_change_count = 6;

static const double kMinSlotCoverRatioUpdateObs = 0.8;
static const double kSublaneWidth = 5.3;
static const double kRightSublaneLength = 7.0;
static const double kLeftSublaneLength = 7.0;
static const double max_slot_target_angle = 60.0 / 57.3;
static const double min_right_slot_target_angle = 10.0 / 57.3;
static const double multi_gear_change_slot_target_y = 0.0;
static const double yaw_step = 1.0 / 57.3;

static const double kEmergencyFlashTime = 0.6;
static const double kMaxVelocity = 0.6;

// vehicle params
static const double kFrontOverhanging = 0.924;
static const double kRearOverhanging = 0.94;
static const double kWheelBase = 2.7;
static const double kVehicleWidth = 1.89;
static const double kEgoVertexLatBuffer = 0.1;
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
    std::cout << "apa is finished" << std::endl;
    return true;
  }
  frame_ = frame;
  local_view_ = &(frame->session()->environmental_model().get_local_view());

  // TODO: collision_detector_.SetVision();

  auto& origin_parking_fusion_info = local_view_->parking_fusion_info;

  // slot management update
  if (g_context.GetStatemachine().apa_reset_flag) {
    Reset();
  }

  UpdateApaFunctionType();

  slot_manager_.Update(&local_view_->function_state_machine_info,
                       &origin_parking_fusion_info,
                       &local_view_->localization_estimate);

  std::cout << "slot manager: fused slot size = "
            << slot_manager_.GetFusedSlotSize() << std::endl;

  // set local view to uss oa
  uss_oa_.SetLocalView(local_view_);

  // set local view to collision detector
  collision_detector_.SetLocalView(local_view_);

  // update measurement
  UpdateMeasurement();

  const auto& parking_fusion_info = origin_parking_fusion_info;

  const int slots_size = parking_fusion_info.parking_fusion_slot_lists_size();
  if (slots_size == 0) {
    std::cout << "Error: slot size is 0" << std::endl;

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
      std::cout << "no select_slot_id" << std::endl;
      return false;
    }

    int select_slot_index = -1;
    const size_t selected_slot_id = parking_fusion_info.select_slot_id();
    std::cout << "selected_slot_id:" << selected_slot_id << std::endl;
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
        std::cout << "diagonal slot selected" << std::endl;
        break;
      }
    }

    if (select_slot_index == -1) {
      std::cout << "selected slot is not diagonal" << std::endl;
      return false;
    }

    // update managed parking fusion info by slot_manager
    if (!UpdateManagedParkingFusion(select_slot_index)) {
      std::cout << "selceted slot is not released!" << std::endl;
      return false;
    }

    // plan once
    PathPlanOnce(select_slot_index, planning_output);
    std::cout << "run PathPlanOnce!" << std::endl;

    // generate planning output
    GeneratePlanningOutput(planning_output);

    // update uss oa
    if (is_plan_success_) {
      uss_oa_.Update(planning_output);
      // generate planning output by uss remain dist
      GeneratePlanningOutputByUssOA(planning_output);
    } else {
      uss_oa_.Reset();
    }

    generate_trajectory = is_plan_success_;
  } else {
    std::cout << "no slot has been selected!" << std::endl;
    // bool is_planning_ok = false;
    // for (int i = 0; i < parking_fusion_info.parking_fusion_slot_lists_size();
    //      ++i) {
    //   if (slots[i].type() ==
    //           Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
    //       slots[i].type() ==
    //           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    //     std::cout << "diagonal slot id:" << slots[i].id();
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
  slot_occupied_ratio_ = 0.0;
  parking_continue_time_ = 0.0;
  ego_slot_info_.Reset();
  left_slot_width_offset_ = slot_width_offset_empty;
  right_slot_width_offset_ = slot_width_offset_empty;
  multi_step_plan_result_.clear();
  multi_step_plan_result_.reserve(DUBINS_LEVEL_COUNT);
  twice_gear_change_enable_ = true;
  remain_dist_uss_ = 5.01;
  remain_dist_ = 5.01;
  is_replan_by_uss_ = false;
  uss_obstacles_vec_.clear();
  plan_result_.path_available = false;
  multi_gear_change_plan_count_ = 0;
  is_last_path_ = false;
  is_replan_once_ = false;
  replan_count_ = 0;
  apa_function_type_ = APA_FUNC_TYPE_IDLE;
}

void DiagonalInTrajectoryGenerator::GeneratePlanningOutputByUssOA(
    PlanningOutput* const planning_output) {
  // std::cout << "planning_output->trajectory()->trajectory_points_size()"
  //           << planning_output->trajectory().trajectory_points_size()
  //           << std::endl;
  remain_dist_uss_ = 5.01;
  if (planning_output->trajectory().trajectory_points_size() < 1) {
    return;
  }

  if (spline_success_ && uss_oa_.GetAvailable()) {
    // update remain_dist_uss
    remain_dist_uss_ = uss_oa_.GetRemainDist() - safe_uss_remain_dist;
  }

  std::cout << "remain_dist_uss = " << remain_dist_uss_ << std::endl;

  planning_output->mutable_trajectory()
      ->mutable_trajectory_points(0)
      ->set_distance(remain_dist_uss_);

  // std::cout << "Uss RemainDist = " << uss_oa_.GetRemainDist() << std::endl;
  // std::cout << "Plan RemainDist = " << remain_dist_ << std::endl;
  // std::cout << "current_path_length_ = " << current_path_length_ <<
  // std::endl;
}

void DiagonalInTrajectoryGenerator::UpdateApaFunctionType() {
  const auto current_state = current_state_ =
      local_view_->function_state_machine_info.current_state();

  if (current_state >= FuncStateMachine::PARK_IN_APA_IN &&
      current_state <= FuncStateMachine::PARK_IN_COMPLETED) {
    apa_function_type_ = APA_FUNC_TYPE_PARKING_IN;
  } else if (current_state >= FuncStateMachine::PARK_OUT_SEARCHING &&
             current_state <= FuncStateMachine::PARK_OUT_COMPLETED) {
    apa_function_type_ = APA_FUNC_TYPE_PARKING_IN;
  } else {
    apa_function_type_ = APA_FUNC_TYPE_IDLE;
  }
}

void DiagonalInTrajectoryGenerator::GeneratePlanningOutput(
    PlanningOutput* const planning_output) {
  if (!simulation_enable_flag_) {
    // when finished
    if (is_finished_) {
      SetFinishedPlanningOutput(frame_);
      return;
    }

    if (current_state_ == FunctionalState::PARK_IN_ACTIVATE_WAIT ||
        current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      parking_continue_time_ += plan_time;
    }

    // not active
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      return;
    }
  }

  // plan suceess
  if (is_plan_success_) {
    // std::cout << "------------- ready to send traj" << std::endl;
    planning_output->Clear();

    planning_output->mutable_planning_status()->set_apa_planning_status(
        ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);

    auto trajectory = planning_output->mutable_trajectory();
    trajectory->set_available(true);

    trajectory->set_trajectory_type(
        Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);

    for (const auto& path_point : plan_result_.path_point_vec) {
      auto trajectory_point = trajectory->add_trajectory_points();

      trajectory_point->set_x(path_point.pos.x());
      trajectory_point->set_y(path_point.pos.y());
      trajectory_point->set_heading_yaw(path_point.heading);
      trajectory_point->set_v(0.5);
    }

    // set target velocity to control as a limit
    const std::vector<double> ratio_tab = {0.0, 0.4, 0.8, 1.0};
    const std::vector<double> vel_limit_tab = {kMaxVelocity, kMaxVelocity, 0.45,
                                               0.35};
    const double vel_limit =
        pnc::mathlib::Interp1(ratio_tab, vel_limit_tab, slot_occupied_ratio_);

    planning_output->mutable_trajectory()
        ->mutable_target_reference()
        ->set_target_velocity(vel_limit);

    // send slot occupation ratio to control
    planning_output->mutable_trajectory()
        ->mutable_trajectory_points(1)
        ->set_distance(slot_occupied_ratio_);

    // set plan gear cmd
    auto gear_command = planning_output->mutable_gear_command();
    gear_command->set_available(true);
    if (plan_result_.current_gear_cmd ==
        pnc::dubins_lib::DubinsLibrary::NORMAL) {
      gear_command->set_gear_command_value(
          Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE);
    } else {
      gear_command->set_gear_command_value(
          Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);
    }

    // set emergency flash cmd
    if (parking_continue_time_ < kEmergencyFlashTime) {
      planning_output->mutable_turn_signal_command()->set_turn_signal_value(
          Common::TurnSignalType::TURN_SIGNAL_TYPE_EMERGENCY_FLASH);
    } else {
      planning_output->mutable_turn_signal_command()->set_turn_signal_value(
          Common::TurnSignalType::TURN_SIGNAL_TYPE_NONE);
    }
  } else {
    if (!simulation_enable_flag_) {
      SetFinishedPlanningOutput(frame_);
    } else {
      planning_output->Clear();
    }
  }
}

const bool DiagonalInTrajectoryGenerator::UpdateManagedParkingFusion(
    const int select_slot_index) {
  // std::cout << "local_view_->parking_fusion_info:"
  //           << local_view_->parking_fusion_info.DebugString() << std::endl;
  // copy parking fusion info
  managed_parking_fusion_info_.CopyFrom(local_view_->parking_fusion_info);

  auto fusion_selected_slot =
      managed_parking_fusion_info_.mutable_parking_fusion_slot_lists(
          select_slot_index);

  common::SlotInfo managed_selected_slot;

  const bool is_selected_slot_released = slot_manager_.GetSelectedSlot(
      managed_selected_slot, managed_parking_fusion_info_.select_slot_id());

  if (is_selected_slot_released) {
    for (auto i = 0; i < fusion_selected_slot->corner_points_size(); ++i) {
      fusion_selected_slot->mutable_corner_points(i)->set_x(
          managed_selected_slot.corner_points().corner_point(i).x());
      fusion_selected_slot->mutable_corner_points(i)->set_y(
          managed_selected_slot.corner_points().corner_point(i).y());
    }
    return true;
  } else {
    // std::cout << "managed_selected_slot = " <<
    // managed_selected_slot.DebugString()
    //           << std::endl;
    return false;
  }
}

const bool DiagonalInTrajectoryGenerator::PathPlanOnceSimulation(
    common::SlotManagementInfo& slot_mangement_info) {
  simulation_enable_flag_ = true;

  // set local view for these two modules for simulation
  // since plan() is not included in pybind simulation
  uss_oa_.SetLocalView(local_view_);
  collision_detector_.SetLocalView(local_view_);

  if (local_view_->function_state_machine_info.has_current_state()) {
    current_state_ = local_view_->function_state_machine_info.current_state();
  }

  UpdateApaFunctionType();

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

  for (auto i = 0; i < fusion_selected_slot->corner_points_size(); ++i) {
    fusion_selected_slot->mutable_corner_points(i)->set_x(
        managed_selected_slot.corner_points().corner_point(i).x());
    fusion_selected_slot->mutable_corner_points(i)->set_y(
        managed_selected_slot.corner_points().corner_point(i).y());
  }

  // update measurement
  UpdateMeasurement();

  // plan once
  PathPlanOnce(select_slot_index, &planning_output_);
  std::cout << "run PathPlanOnce!" << std::endl;

  // generate planning output
  GeneratePlanningOutput(&planning_output_);

  // update uss oa
  if (is_plan_success_) {
    uss_oa_.Update(&planning_output_);
    // generate planning output by uss remain dist
    GeneratePlanningOutputByUssOA(&planning_output_);
  } else {
    uss_oa_.Reset();
  }

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

  const auto n = (pM01 - pM23).normalized();
  const auto t = Eigen::Vector2d(-n.y(), n.x());

  ego_slot_info_.slot_origin_pos =
      pM01 - kNormalSlotLength * n + target_y_offset * t;

  ego_slot_info_.slot_origin_heading = std::atan2(n.y(), n.x());

  // global2slot tf init
  g2l_tf_.Init(ego_slot_info_.slot_origin_pos,
               ego_slot_info_.slot_origin_heading);

  l2g_tf_.Init(ego_slot_info_.slot_origin_pos,
               ego_slot_info_.slot_origin_heading);

  ego_slot_info_.ego_pos_slot = g2l_tf_.GetPos(measure_.ego_pos);
  ego_slot_info_.ego_heading_slot = g2l_tf_.GetHeading(measure_.heading);

  ego2slot_tf_.Init(ego_slot_info_.ego_pos_slot,
                    ego_slot_info_.ego_heading_slot);

  // move away obstacles if ego car is crashed with new slot line.
  // calc extreme y value of ego car
  Eigen::Vector2d FL_corner_vec_ego(kFrontOverhanging + kWheelBase,
                                    0.5 * kVehicleWidth);
  Eigen::Vector2d FR_corner_vec_ego(FL_corner_vec_ego.x(),
                                    -FL_corner_vec_ego.y());

  Eigen::Vector2d RL_corner_vec_ego(-kRearOverhanging, FL_corner_vec_ego.y());
  Eigen::Vector2d RR_corner_vec_ego(RL_corner_vec_ego.x(),
                                    -RL_corner_vec_ego.y());

  Eigen::Vector2d FL_corner_vec_slot = ego2slot_tf_.GetPos(FL_corner_vec_ego);
  Eigen::Vector2d FR_corner_vec_slot = ego2slot_tf_.GetPos(FR_corner_vec_ego);
  Eigen::Vector2d RL_corner_vec_slot = ego2slot_tf_.GetPos(RL_corner_vec_ego);
  Eigen::Vector2d RR_corner_vec_slot = ego2slot_tf_.GetPos(RR_corner_vec_ego);
  const double ego_max_y =
      kEgoVertexLatBuffer +
      std::max(FL_corner_vec_slot.y(), RL_corner_vec_slot.y());
  const double ego_min_y =
      -kEgoVertexLatBuffer +
      std::min(FR_corner_vec_slot.y(), RR_corner_vec_slot.y());

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

  if (std::fabs(terminal_err_.pos.y()) < 3.5 * kMaxLatOffset) {
    slot_occupied_ratio_ = pnc::mathlib::Clamp(
        1.0 - (terminal_err_.pos.x() / standard_slot_length), 0.0, 1.0);
  } else {
    slot_occupied_ratio_ = 0.0;
  }

  std::cout << "slot_occupied_ratio = " << slot_occupied_ratio_ << std::endl;

  // update obstacles in current, left or right slot

  // update current slot obstacles
  ego_slot_info_.slot_obs.first = true;
  const auto half_slot_width = 0.5 * ego_slot_info_.slot_width;

  // std::cout << "half_slot_width = " << half_slot_width << std::endl;
  // std::cout << "ego max y = " << ego_max_y << std::endl;
  // std::cout << "ego min y = " << ego_min_y << std::endl;
  if (slot_occupied_ratio_ > kMinSlotCoverRatioUpdateObs) {
    if (ego_max_y > half_slot_width) {
      left_slot_width_offset_ =
          slot_width_offset_empty + std::fabs(ego_max_y - half_slot_width);
      // std::cout << "adjust left obs line " << std::endl;
    }
    if (ego_min_y < -half_slot_width) {
      right_slot_width_offset_ =
          slot_width_offset_empty + std::fabs(ego_min_y + half_slot_width);
      // std::cout << "adjust righgt obs line " << std::endl;
    }
  }
  Eigen::Vector2d origin = ego_slot_info_.slot_origin_pos;

  Eigen::Vector2d left_pA =
      origin + (half_slot_width + left_slot_width_offset_) * t;
  Eigen::Vector2d right_pA =
      origin - (half_slot_width + right_slot_width_offset_) * t;

  Eigen::Vector2d left_pB = left_pA + kNormalSlotLength * n * 0.95;

  Eigen::Vector2d right_pB = right_pA + kNormalSlotLength * n * 0.95;

  ego_slot_info_.slot_obs.second[EgoSlotInfo::LEFT] =
      pnc::geometry_lib::LineSegment(left_pA, left_pB);

  ego_slot_info_.slot_obs.second[EgoSlotInfo::RIGHT] =
      pnc::geometry_lib::LineSegment(right_pA, right_pB);

  // update left slot obstacles
  // TODO: to check if left and right slot are occupied by uss_OA or slot_m
  ego_slot_info_.left_slot_obs.first = false;

  ego_slot_info_.left_slot_obs.second = pnc::geometry_lib::LineSegment(
      left_pB, left_pB + ego_slot_info_.slot_width * t);

  // update right slot obstacles
  ego_slot_info_.right_slot_obs.first = false;

  ego_slot_info_.right_slot_obs.second = pnc::geometry_lib::LineSegment(
      right_pB, right_pB - ego_slot_info_.slot_width * t);

  ego_slot_info_.obstacles_vec.clear();
  // clear obstacles first
  if (ego_slot_info_.slot_obs.first) {
    ego_slot_info_.obstacles_vec.emplace_back(
        ego_slot_info_.slot_obs.second[LEFT]);
    ego_slot_info_.obstacles_vec.emplace_back(
        ego_slot_info_.slot_obs.second[RIGHT]);
  }

  if (ego_slot_info_.left_slot_obs.first) {
    ego_slot_info_.obstacles_vec.emplace_back(
        ego_slot_info_.left_slot_obs.second);
  }

  if (ego_slot_info_.right_slot_obs.first) {
    ego_slot_info_.obstacles_vec.emplace_back(
        ego_slot_info_.right_slot_obs.second);
  }

  // update channel obstacles
  const double chanel_width_x = sublane_width_ + kNormalSlotLength;
  const double chanel_length_y1 =
      -0.5 * ego_slot_info_.slot_width - sublane_right_length_;
  const double chanel_length_y2 = -chanel_length_y1;

  const auto pA1 = origin + chanel_length_y1 * t;
  const auto pB1 = pA1 + chanel_width_x * n;

  const auto pA2 = origin + chanel_length_y2 * t;
  const auto pB2 = pA2 + chanel_width_x * n;

  const pnc::geometry_lib::LineSegment A1B1(pA1, pB1);
  const pnc::geometry_lib::LineSegment A2B2(pA2, pB2);
  const pnc::geometry_lib::LineSegment B1B2(pB1, pB2);

  // add channel object
  ego_slot_info_.obstacles_vec.emplace_back(A1B1);
  ego_slot_info_.obstacles_vec.emplace_back(A2B2);
  ego_slot_info_.obstacles_vec.emplace_back(B1B2);
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanOneStep(
    const PlanInput& plan_input, const uint8_t plan_algorithm) {
  dubins_planner_.SetRadius(plan_input.path_radius);
  dubins_planner_.SetTarget(plan_input.target_pos, plan_input.target_heading);
  plan_algorithm_ = plan_algorithm;

  // dubins method
  if (plan_algorithm == PlanAlgorithm::DUBINS) {
    // dubins search loop in one tuning step
    for (size_t i = 0; i < DubinsLibrary::CASE_COUNT; ++i) {
      for (size_t j = 0; j < DubinsLibrary::DUBINS_TYPE_COUNT; ++j) {
        if (dubins_planner_.Solve(j, i)) {
          if (PathEvaluateOnce()) {
            if (!CollisionCheck()) {
              return true;
            }
          }
        }
      }
    }  // dubins loop
  }

  // line arc method
  if (plan_algorithm == PlanAlgorithm::LINE_ARC) {
    for (size_t i = 0; i < DubinsLibrary::LINEARC_TYPE_COUNT; ++i) {
      if (dubins_planner_.Solve(i)) {
        if (PathEvaluateOnce()) {
          if (!CollisionCheck()) {
            return true;
          }
        }
      }
    }  // linearc loop
  }

  return false;
}

const bool DiagonalInTrajectoryGenerator::CheckPathPointsInSlot() const {
  const auto& output = dubins_planner_.GetOutput();
  if (output.path_available) {
    const auto& pB = output.arc_AB.pB;
    const auto& pC = output.arc_CD.pA;
    const auto& pD = output.arc_CD.pB;

    if (pB.x() > terminal_target_x && pC.x() > terminal_target_x &&
        pD.x() > terminal_target_x) {
      return true;
    }
  }

  return false;
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

  static const Eigen::Vector2d kEgoFLCornerVec((kFrontOverhanging + kWheelBase),
                                               0.5 * kVehicleWidth);
  static const Eigen::Vector2d kEgoFRCornerVec((kFrontOverhanging + kWheelBase),
                                               -0.5 * kVehicleWidth);

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

const bool DiagonalInTrajectoryGenerator::CollisionCheck() {
  auto out = dubins_planner_.GetOutput();

  // sample by large ds to get path point vector
  DubinsLibrary::GetSampling(out, collision_check_sample_ds, true);

  // transform to global
  DubinsLibrary::GetTransform(out.path_point_vec, l2g_tf_);

  // gen car circles by path point vector
  collision_detector_.GenCarCircles(out.path_point_vec);

  // collision detect
  return collision_detector_.CollisionDetect();
}

const bool DiagonalInTrajectoryGenerator::PathEvaluateOnce() const {
  const auto& output = dubins_planner_.GetOutput();

  if (!CheckPathPointsInSlot()) {
    return false;
  }

  const auto& is_line_arc = output.is_line_arc;

  // force reversed gear cmd when replan by uss
  // if (is_replan_by_uss_ &&
  //     output.current_gear_cmd == plan_result_.current_gear_cmd) {
  //   return false;
  // }

  // no gear change: force no gear change in final
  if (plan_state_machine_ == FINAL) {
    const bool is_toward_terminal =
        ego_slot_info_.ego_pos_slot.x() > plan_input_.target_pos.x();
    if (output.gear_change_count == 0 && is_toward_terminal) {
      if (is_line_arc) {
        if (output.line_arc_radius > min_radius_final) {
          return true;
        }
      } else {
        return true;
      }
    }
  }

  if (plan_state_machine_ == INIT) {
    if (output.gear_change_count == 1) {
      if (output.current_length >= kMinProperLength && is_line_arc == false) {
        return true;
      }
    }
  }

  return false;
}

const bool
DiagonalInTrajectoryGenerator::DubinsPlanOnceGearChangeFixedTarget() {
  if (multi_gear_change_plan_count_ == 0) {
    return false;
  }

  plan_input_.path_radius = min_radius_final;
  dubins_planner_.SetStart(ego_slot_info_.ego_pos_slot,
                           ego_slot_info_.ego_heading_slot);
  plan_input_ = twice_gear_change_outer_plan_input_;

  bool success = DubinsPlanOneStep(plan_input_, PlanAlgorithm::LINE_ARC);

  if (!success) {
    plan_input_.path_radius = min_radius;
    success = DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS);
  }

  if (!success) {
    plan_state_machine_ = INIT;
    success = DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS);
  }

  if (success) {
    return true;
  } else {
    return false;
  }
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanOnceGearChange(
    uint8_t plan_state_machine) {
  plan_state_machine_ = plan_state_machine;
  const auto last_path_level = path_level_;

  path_level_ = DUBINS_LEVEL_ONCE_GEAR_CHANGE;

  if (last_path_level == DUBINS_LEVEL_TWICE_GEAR_CHANGE) {
    if (DubinsPlanOnceGearChangeFixedTarget()) {
      std::cout << "try fixed target success!" << std::endl;
      return true;
    } else {
      std::cout << "try fixed target failed!" << std::endl;
    }
  }

  dubins_iter_count_ = 0;

  // step 1: update target pose and radius
  // set radius
  if (plan_state_machine_ == FINAL) {
    plan_input_.path_radius = min_radius_final;
  } else {
    plan_input_.path_radius = min_radius;
  }

  plan_input_.target_pos << terminal_target_x, 0.0;
  plan_input_.target_heading = 0.0;

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

  // try line arc method first
  while (s < max_search_length) {
    if (DubinsPlanOneStep(plan_input_, PlanAlgorithm::LINE_ARC)) {
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

    plan_input_.target_pos << terminal_target_x, 0.0;
    plan_input_.target_heading = 0.0;

    // then try dubins method
    while (s < max_search_length) {
      if (DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS)) {
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

  if (success_once) {
    // set shortest path
    // TODO: can be iterated
    dubins_planner_.SetOutput(
        dubins_output_vec[path_length_queue.top().second]);

    // std::cout << "path_length_queue.top() = " <<
    // path_length_queue.top().second
    //           << std::endl;
    if (plan_state_machine_ == FINAL) {
      if (!(dubins_planner_.GetOutput().gear_change_count == 0 &&
            dubins_planner_.GetOutput().current_gear_cmd ==
                pnc::dubins_lib::DubinsLibrary::REVERSE)) {
        success_once = false;
      }
    }
  }

  if (success_once) {
    if (plan_state_machine_ == FINAL) {
      is_last_path_ = true;
    }
    return true;
  } else {
    std::cout << "--------all failed!" << std::endl;
    return false;
  }
}

const bool DiagonalInTrajectoryGenerator::DubinsPlanTwiceGearChange(
    uint8_t plan_state_machine) {
  if (!twice_gear_change_enable_) {
    std::cout << "twice_gear_change is not enabled!" << std::endl;
    return false;
  }

  plan_state_machine_ = plan_state_machine;
  path_level_ = DUBINS_LEVEL_TWICE_GEAR_CHANGE;
  dubins_iter_count_ = 0;

  // multi_step_plan_result must be cleared every time
  multi_step_plan_result_.clear();

  bool success_once = false;

  const auto ego_heading_sign =
      ego_slot_info_.ego_heading_slot < 0.0 ? -1.0 : 1.0;
  // step1: update target pose and radius
  // set radius
  plan_input_.path_radius = min_radius;

  // outer dubins search loop
  auto search_angle = ego_heading_sign * max_slot_target_angle;

  const auto search_angle_length =
      ego_heading_sign * min_right_slot_target_angle - search_angle;

  const auto end_search_angle = search_angle + search_angle_length;
  const double angle_search_step = -ego_heading_sign * yaw_step;

  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      outer_path_length_queue;

  std::vector<std::pair<DubinsLibrary::Output, DubinsLibrary::Output>>
      dubins_output_pair_vec;

  dubins_output_pair_vec.clear();
  dubins_output_pair_vec.reserve(
      std::ceil(std::fabs(search_angle_length / angle_search_step)));

  std::vector<PlanInput> outer_plan_input_vec;
  outer_plan_input_vec.reserve(dubins_output_pair_vec.capacity());
  size_t outter_loop_index = 0;
  bool inner_success_once = false;

  // angle (outer) loop: current car pos is set as dubins start, multi-gear
  // change target pose is set to dubins end.
  while (search_angle < end_search_angle) {
    dubins_planner_.SetStart(ego_slot_info_.ego_pos_slot,
                             ego_slot_info_.ego_heading_slot);

    // update target anyway
    plan_input_.target_pos << kNormalSlotLength + 0.1,
        -ego_heading_sign * multi_gear_change_slot_target_y;
    plan_input_.target_heading = search_angle;

    const bool outer_success =
        DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS);

    search_angle += angle_search_step;

    if (outer_success == false) {
      // std::cout << "outer search failed" << std::endl;
      continue;
    } else {
      // std::cout << "outer search success" << std::endl;
    }

    // inner loop: one step target_x loop integrated in DubinsPlanOneStep
    // dubins curve start from multi-gear change target pose to terminate
    // target pose

    const auto outer_dubins_output = dubins_planner_.GetOutput();
    const auto outer_plan_input = plan_input_;

    double s = 0.0;
    size_t inner_loop_index = 0;

    const double max_search_length = std::max(
        min_search_length, outer_plan_input.target_pos.x() - terminal_target_x);

    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>>
        inner_path_length_queue;

    std::vector<DubinsLibrary::Output> inner_dubins_output_vec;
    inner_dubins_output_vec.clear();
    inner_dubins_output_vec.reserve(std::ceil(max_search_length / kLineStep));

    dubins_planner_.SetStart(outer_plan_input.target_pos,
                             outer_plan_input.target_heading);

    // update target anyway
    plan_input_.target_pos << terminal_target_x, 0.0;
    plan_input_.target_heading = 0.0;

    while (s < max_search_length) {
      if (DubinsPlanOneStep(plan_input_, PlanAlgorithm::DUBINS)) {
        const auto length = dubins_planner_.GetOutput().length;
        inner_dubins_output_vec.emplace_back(dubins_planner_.GetOutput());
        inner_path_length_queue.push({length, inner_loop_index});
        inner_loop_index++;
        inner_success_once = true;
      }

      s += kLineStep;
      plan_input_.target_pos.x() = terminal_target_x + s;
    }

    if (inner_success_once) {
      size_t seleteced_index = inner_path_length_queue.top().second;

      if (seleteced_index > inner_dubins_output_vec.size() - 1) {
        return false;
      }

      const auto inner_dubins_output = inner_dubins_output_vec[seleteced_index];

      dubins_output_pair_vec.emplace_back(
          std::make_pair(outer_dubins_output, inner_dubins_output));

      outer_plan_input_vec.emplace_back(outer_plan_input);

      const auto two_step_length =
          outer_dubins_output.length + inner_dubins_output.length;

      outer_path_length_queue.push({two_step_length, outter_loop_index});

      outter_loop_index++;
    }
  }

  if (inner_success_once) {
    const auto result_pair =
        dubins_output_pair_vec[outer_path_length_queue.top().second];

    twice_gear_change_outer_plan_input_ =
        outer_plan_input_vec[outer_path_length_queue.top().second];

    multi_step_plan_result_.emplace_back(result_pair.first);
    multi_step_plan_result_.emplace_back(result_pair.second);

    dubins_planner_.SetOutput(multi_step_plan_result_.front());

    success_once = true;
  } else {
    success_once = false;
  }

  multi_gear_change_plan_count_++;

  return success_once;
}

void DiagonalInTrajectoryGenerator::PrintDubinsOutput() {
  const auto& output = dubins_planner_.GetOutput();
  std::cout << "------------------------------------- dubins result "
            << std::endl;

  std::cout << "path_level = " << static_cast<int>(path_level_) << std::endl;

  std::cout << "pA = " << output.arc_AB.pA.transpose() << std::endl;
  std::cout << "pB = " << output.arc_AB.pB.transpose() << std::endl;
  std::cout << "pC = " << output.arc_CD.pA.transpose() << std::endl;
  std::cout << "pD = " << output.arc_CD.pB.transpose() << std::endl;

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
      std::fabs(terminal_err_.pos.x()) < kMaxLonOffset &&
      std::fabs(terminal_err_.pos.y()) <= kMaxLatOffset &&
      std::fabs(terminal_err_.heading) <= kMaxHeadingOffset &&
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

  return is_finished_ && (!simulation_enable_flag_);
}

const bool DiagonalInTrajectoryGenerator::CheckReplan(
    PlanningOutput* const planning_output) {
  if (simulation_enable_flag_ && simu_param_.force_planning_) {
    std::cout << "tune force_planning on!" << std::endl;
    is_replan_once_ = false;
    return true;
  }

  if (CheckIfNearTerminalPoint()) {
    if (!is_replan_by_uss_) {
      std::cout << "close to target!" << std::endl;
    } else {
      std::cout << "close to obstacle by uss!" << std::endl;
    }

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

    if (CheckIfReplanByStuck()) {
      std::cout << "replan by stuck!" << std::endl;
      return true;
    }
  }

  return false;
}

void DiagonalInTrajectoryGenerator::PostProcessPath() {
  // transform current dubins result to global
  dubins_planner_.Transform(l2g_tf_);

  const auto& output = dubins_planner_.GetOutput();
  const auto N = output.path_point_vec.size();
  if (N < 3) {
    spline_success_ = false;
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

  x_s_spline_g_.set_points(s_vec, x_vec);
  y_s_spline_g_.set_points(s_vec, y_vec);

  spline_success_ = true;
}

const bool DiagonalInTrajectoryGenerator::PathPlanCoreIteration() {
  // start apa dubins planning
  dubins_planner_.SetStart(ego_slot_info_.ego_pos_slot,
                           ego_slot_info_.ego_heading_slot);

  // main loop for dubins planning
  is_plan_success_ = false;

  // hack for 1024
  // if (replan_count_ == 0) {
  //   std::cout << "----------------fist plan: try multi-step plan!" <<
  //   std::endl; if (DubinsPlanTwiceGearChange(INIT)) {
  //     std::cout << "--init multi-step success!" << std::endl;
  //     is_plan_success_ = true;

  //     return true;
  //   } else {
  //     std::cout << "--init multi-step failed!" << std::endl;
  //   }
  // }

  // first dubins iteration: try to plan with final: zero gear change
  std::cout << "----------------try final plan first!" << std::endl;
  if (DubinsPlanOnceGearChange(FINAL)) {
    is_plan_success_ = true;
    std::cout << "--final plan success!" << std::endl;
  } else {
    std::cout << "--final plan failed!" << std::endl;
  }

  // second dubins iteration: try to plan again with init: once gear change
  if (!is_plan_success_) {
    std::cout << "----------------try init plan!" << std::endl;
    if (DubinsPlanOnceGearChange(INIT)) {
      std::cout << "--init plan success!" << std::endl;
      is_plan_success_ = true;
    } else {
      std::cout << "--init plan failed!" << std::endl;
    }
  }

  // third dubins iteration: try multi-step planning
  if (!is_plan_success_) {
    std::cout << "----------------try multi-step plan!" << std::endl;
    if (DubinsPlanTwiceGearChange(INIT)) {
      std::cout << "--init multi-step success!" << std::endl;
      is_plan_success_ = true;
    } else {
      std::cout << "--init multi-step failed!" << std::endl;
    }
  }

  return is_plan_success_;
}

void DiagonalInTrajectoryGenerator::ClearUssObstacles() {
  uss_obstacles_vec_.clear();
  uss_oa_.SetDisable();
  UpdateObstacles();
}

void DiagonalInTrajectoryGenerator::StupidParkingOut() {}

void DiagonalInTrajectoryGenerator::PathPlanOnce(
    const int slot_index, PlanningOutput* const planning_output) {
  // hack for parking out, will be removed after stupid 1024
  if (apa_function_type_ == APA_FUNC_TYPE_PARKING_OUT) {
    StupidParkingOut();

    return;
  }

  // update slot info and target point in both global and slot coordinate
  UpdateEgoSlotInfo(slot_index);

  // check if finish
  if (CheckFinish()) {
    std::cout << "apa is finished" << std::endl;
    return;
  }

  // check if replan
  is_replan_ = CheckReplan(planning_output);

  is_last_path_ = false;

  if (is_replan_once_ && !is_replan_) {
    if (fabs(measure_.v_ego) > 0.1) {
      is_replan_once_ = false;
    }
  }

  if (!is_replan_) {
    std::cout << "replan is not required!" << std::endl;
    return;
  }

  // avoid repeated replan
  if (!is_replan_once_) {
    is_replan_once_ = true;
    std::cout << "need replan once!" << std::endl;
  } else {
    std::cout << "has replaned once!, cancel replan!" << std::endl;
    return;
  }

  if (!simulation_enable_flag_ && replan_count_ == 2) {
    slot_manager_.SetRealtime();
    UpdateManagedParkingFusion(slot_index);
    UpdateEgoSlotInfo(slot_index);
  }

  // update obstacles before replan
  UpdateObstacles();

  // run plan core
  if (!PathPlanCoreIteration()) {
    std::cout << "PathPlanCoreIteration is failed!" << std::endl;
  }

  // clear uss obstacles and try again
  if (!is_plan_success_) {
    ClearUssObstacles();
    if (!PathPlanCoreIteration()) {
      std::cout << "PathPlanCoreIteration after clear uss obstacles is failed !"
                << std::endl;
    }
  }

  // update slot info and try again
  if (!simulation_enable_flag_ && !is_plan_success_) {
    slot_manager_.SetRealtime();
    if (UpdateManagedParkingFusion(slot_index)) {
      UpdateEgoSlotInfo(slot_index);
      UpdateObstacles();

      if (!PathPlanCoreIteration()) {
        std::cout << "PathPlanCoreIteration after update slot is failed!"
                  << std::endl;
      }
    }
  }

  if (!is_plan_success_) {
    return;
  }

  // count replan
  replan_count_++;

  // enable complete path when simulation
  bool is_complete_path = false;
  if (simulation_enable_flag_) {
    is_complete_path = simu_param_.is_complete_path;
    dubins_planner_.Sampling(simu_param_.sample_ds, is_complete_path);
  } else {
    dubins_planner_.Sampling(path_sample_ds, is_complete_path);
  }

  double extend_s = 0.0;
  if (is_last_path_) {
    extend_s = dubins_planner_.GetOutput().arc_CD.pB.x() - terminal_target_x;
    dubins_planner_.Extend(extend_s);

    // std::cout << "plan_input_.target_pos = "
    //           << plan_input_.target_pos.transpose() << std::endl;
    // std::cout << "extend_s = " << extend_s << std::endl;
  }

  // process path
  PostProcessPath();

  // record gear change count, must be after successful path plan
  gear_change_count_ = dubins_planner_.GetOutput().gear_change_count;
  plan_result_ = dubins_planner_.GetOutput();

  // print dubins output for debug
  PrintDubinsOutput();

  std::cout << "diagonal replan triggered, plan statemachine = "
            << static_cast<int>(plan_state_machine_) << std::endl;
}

void DiagonalInTrajectoryGenerator::Log() const {
  JSON_DEBUG_VALUE("is_replan", is_replan_)
  JSON_DEBUG_VALUE("path_level", path_level_)
  JSON_DEBUG_VALUE("is_finished", is_finished_)
  JSON_DEBUG_VALUE("dubins_type", dubins_planner_.GetOutput().dubins_type)
  JSON_DEBUG_VALUE("case_type", dubins_planner_.GetOutput().case_type)

  JSON_DEBUG_VALUE("gear_change_count",
                   dubins_planner_.GetOutput().gear_change_count)

  JSON_DEBUG_VALUE("path_length", dubins_planner_.GetOutput().length)
  JSON_DEBUG_VALUE("plan_state_machine", plan_state_machine_)
  JSON_DEBUG_VALUE("replan_count", replan_count_)
  JSON_DEBUG_VALUE("is_replan_once", is_replan_once_)

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
  JSON_DEBUG_VALUE("remain_dist_uss", remain_dist_uss_)

  JSON_DEBUG_VALUE("slot_occupied_ratio", slot_occupied_ratio_)

  const auto& obstacles = collision_detector_.GetObstacles();
  std::vector<double> obs_x_vec;
  std::vector<double> obs_y_vec;

  obs_x_vec.reserve(2 * obstacles.size());
  obs_y_vec.reserve(2 * obstacles.size());

  for (const auto& obs : obstacles) {
    obs_x_vec.emplace_back(obs.pA.x());
    obs_x_vec.emplace_back(obs.pB.x());

    obs_y_vec.emplace_back(obs.pA.y());
    obs_y_vec.emplace_back(obs.pB.y());
  }

  JSON_DEBUG_VECTOR("obs_x_vec", obs_x_vec, 2)
  JSON_DEBUG_VECTOR("obs_y_vec", obs_y_vec, 2)
}

void DiagonalInTrajectoryGenerator::UpdateObstacles() {
  // clear obstacles
  collision_detector_.ClearObstacles();

  // update uss obstacles
  if (is_replan_by_uss_) {
    uss_obstacles_vec_.clear();
    if (uss_oa_.GetAvailable()) {
      pnc::geometry_lib::LocalToGlobalTf l2g_tf(measure_.ego_pos,
                                                measure_.heading);

      const auto uss_local_line = uss_oa_.GetMinDistUssLine();

      uss_obstacles_vec_.emplace_back(pnc::geometry_lib::LineSegment(
          l2g_tf.GetPos(uss_local_line.pA), l2g_tf.GetPos(uss_local_line.pB)));
    }
  }

  // add uss obstacles
  for (auto const& obs : uss_obstacles_vec_) {
    collision_detector_.AddObstacle(obs);
  }
  std::cout << "uss obstacle size = " << uss_obstacles_vec_.size() << std::endl;

  // add slot and channel obstacles
  for (auto const& obs : ego_slot_info_.obstacles_vec) {
    collision_detector_.AddObstacle(obs);
  }
  std::cout << "slot obstacle size = " << ego_slot_info_.obstacles_vec.size()
            << std::endl;
}

const bool DiagonalInTrajectoryGenerator::CheckIfNearTerminalPoint() {
  is_replan_by_uss_ = false;
  if (spline_success_) {
    const auto min_remain_dist = std::min(remain_dist_uss_, remain_dist_);
    if (min_remain_dist < min_replan_remain_dist && measure_.static_flag) {
      is_replan_by_uss_ = (remain_dist_uss_ < remain_dist_);
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
    measure_.static_flag = (measure_.standstill_timer >= 0.5 &&
                            measure_.standstill_timer_by_pos > 0.5) ||
                           (measure_.standstill_timer_by_pos > 1.5);
  } else {
    measure_.static_flag = std::fabs(measure_.v_ego) < kStanstillSpd ||
                           measure_.standstill_timer_by_pos > 0.5;
  }

  if (spline_success_) {
    pnc::spline::Projection proj;
    proj.CalProjectionPoint(x_s_spline_g_, y_s_spline_g_, 0.0,
                            current_path_length_, measure_.ego_pos);

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
