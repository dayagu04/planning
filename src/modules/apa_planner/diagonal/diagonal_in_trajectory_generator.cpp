#include "diagonal/diagonal_in_trajectory_generator.h"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <ostream>

#include "Eigen/src/Core/Matrix.h"
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

const static double kNormalSlotLength = 4.8;
}  // namespace

bool DiagonalInTrajectoryGenerator::Plan(framework::Frame* const frame) {
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

  auto& origin_parking_fusion_info = local_view_->parking_fusion_info;

  // slot management update
  if (g_context.GetStatemachine().apa_reset_flag) {
    plan_state_machine_ = IDLE;
    slot_manager_.Reset();
  }

  slot_manager_.Update(&local_view_->function_state_machine_info,
                       &origin_parking_fusion_info,
                       &local_view_->localization_estimate);

  UpdateStandstillTime();

  UpdatePosUnchangedCount();

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

  is_rough_calc_ = IsRoughCalc(frame);

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

#ifndef USE_DUBINS_LIB
    return SingleSlotPlan(select_slot_index, &planning_output_);
#else
    return SingleDubinsSlotPlan(select_slot_index, &planning_output_);
#endif

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

bool DiagonalInTrajectoryGenerator::SingleSlotPlanSimulation(
    common::SlotManagementInfo& slot_mangement_info) {
  simulation_enable_flag_ = true;

  if (local_view_->function_state_machine_info.has_current_state()) {
    current_state_ = local_view_->function_state_machine_info.current_state();
  }

  if (simu_param_.force_planning_) {
    current_state_ = FunctionalState::PARK_IN_ACTIVATE_CONTROL;
  }

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
  for (size_t i = 0; i < slot_mangement_info.slot_info_vec_size(); ++i) {
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

#ifndef USE_DUBINS_LIB
  SingleSlotPlan(select_slot_index, &planning_output_);
#else
  SingleDubinsSlotPlan(select_slot_index, &planning_output_);
#endif

  return true;
}

bool DiagonalInTrajectoryGenerator::SingleSlotPlan(
    const int slot_index, PlanningOutput* const planning_output) {
  CalSlotPointsInM(slot_index);

  slot_sign_ = CalSlotSide(slot_index);
  std::cout << "slot side: " << slot_sign_ << std::endl;

  CalSlotOriginInodom();

  CalApaTargetInSlot();

  CalEgoPostionInSlotAndOdom();

  if (!simulation_enable_flag_) {
    if (IsApaFinished()) {
      AINFO << "apa is finished";
      SetFinishedPlanningOutput(frame_);
      return true;
    }

    if (!IsReplan(planning_output)) {
      return true;
    }
  } else {
    // when simualtion
    last_segment_name_ = simu_param_.force_last_seg_name_;
    if (IsSimulatedApaFinished()) {
      AINFO << "apa is finished";
      SetFinishedPlanningOutput(frame_);
      return true;
    }
    // todo: add simulation  is_replan
  }

  std::cout << "diagonal replan triggered, replan state:"
            << static_cast<int>(current_state_) << std::endl;
  std::cout << "cur segment name:" << static_cast<int>(last_segment_name_)
            << std::endl;

  PrintSlotInfo();

  if (!GeometryPlan(cur_pos_in_slot_, slot_index, planning_output)) {
    std::cout << "geometry diagonal plan failed" << std::endl;
    return false;
  }

  SetPlanningOutputInfo(planning_output);
  return true;
}

void DiagonalInTrajectoryGenerator::UpdateSlotInfo(const int slot_index) {
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

  slot_origin_pos_ = k * n.normalized() * kNormalSlotLength + pM;
  slot_origin_heading_ = std::atan2(-n.y(), -n.x());
}

bool DiagonalInTrajectoryGenerator::DubinsPlan() {
  if (plan_state_machine_ == IDLE) {
    dubins_planner_.SetRadius(5.5);
    dubins_planner_.SetTarget(Eigen::Vector2d(target_x_, target_y_), 0.0);

    // dubins_planner_.Solve(uint8_t dubins_type, uint8_t case_type);
  }
}

bool DiagonalInTrajectoryGenerator::SingleDubinsSlotPlan(
    const int slot_index, PlanningOutput* const planning_output) {
  // update measurement
  UpdateMeasurement();

  // update slot info for target pose
  UpdateSlotInfo(slot_index);

  // tf init
  g2l_tf_.Init(slot_origin_pos_, slot_origin_heading_);
  l2g_tf_.Init(slot_origin_pos_, slot_origin_heading_);

  const auto ego_pos_slot = g2l_tf_.GetPos(measure_.ego_pos);
  const auto ego_heading_slot = g2l_tf_.GetHeading(measure_.heading);

  dubins_planner_.SetStart(ego_pos_slot, ego_heading_slot);

  // main loop
  if (plan_state_machine_ == IDLE) {
    if (!DubinsPlan()) {
      PreparePlan();
    }
  } else if (plan_state_machine_ == INIT) {
    if (!LineArcPlan()) {
      DubinsPlan();
    }
  } else if (plan_state_machine_ == PREPARE) {
    DubinsPlan();
  } else if (plan_state_machine_ == FINAL) {
    if (CheckPose()) {
      plan_state_machine_ = FINISH;
    } else {
      DubinsPlan();
      plan_state_machine_ = ADJUST;
    }
  }

  return true;
}

bool DiagonalInTrajectoryGenerator::GeometryPlan(
    const PlanningPoint& start_point, int idx,
    PlanningOutput* const planning_output) {
  geometry_planning_ = DiagonalInGeometryPlan();
  SetGeometryPlanningParameter(idx, &geometry_planning_);
  bool is_planning_ok = false;
  planning_output->mutable_trajectory()->mutable_trajectory_points()->Clear();

  JSON_DEBUG_VALUE("last_segment_name", last_segment_name_)

  if (last_segment_name_ == SEGMENT_NONE) {
    if (ABSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = SEGMENT_BC;
      is_planning_ok = true;
    } else if (ReverseABSegmentPlan(start_point, true, idx, &geometry_planning_,
                                    planning_output)) {
      last_segment_name_ = SEGMENT_CD;
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == SEGMENT_AB) {
    if (BCSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = SEGMENT_BC;
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == SEGMENT_BC) {
    if (CDSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = SEGMENT_CD;
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == SEGMENT_CD) {
    if (DESegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = SEGMENT_DE;
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == SEGMENT_DE) {
    if (CDSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = SEGMENT_CD;
      is_planning_ok = true;
    }
  } else {
    std::cout << "Invalid parallel segment name" << std::endl;
    return false;
  }

  if (!simulation_enable_flag_) {
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      last_segment_name_ = SEGMENT_NONE;
    }
  }

  // if (is_planning_ok) {
  //   apa_speed_smoother_.Smooth(objects_map_in_global_cor_, planning_output);
  // }

  return is_planning_ok;
}

bool DiagonalInTrajectoryGenerator::ABSegmentPlan(
    const PlanningPoint& point_a, bool is_start, int idx,
    DiagonalInGeometryPlan* const geometry_planning,
    PlanningOutput* const planning_output) const {
  DiagonalSegmentsInfo segments_info;
  segments_info.opt_point_a = point_a;
  if (geometry_planning->ABSegment(point_a, is_start, is_rough_calc_,
                                   &segments_info)) {
    std::cout << "plan a-b success, slot index:" << idx << std::endl;
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      planning_output->add_successful_slot_info_list()->set_id(
          managed_parking_fusion_info_.parking_fusion_slot_lists()[idx].id());
      return true;
    }
    GenerateABSegmentTrajectory(segments_info, planning_output);
    GenerateBCSegmentTrajectory(segments_info, planning_output);

#ifdef __PYBIND_DEBUG__

    std::cout << "segments_info A: (" << segments_info.opt_point_a.x << ", "
              << segments_info.opt_point_a.y << ", "
              << segments_info.opt_point_a.theta << ")" << std::endl;
    std::cout << "segments_info B: (" << segments_info.opt_point_b.x << ", "
              << segments_info.opt_point_b.y << ", "
              << segments_info.opt_point_b.theta << ")" << std::endl;
    std::cout << "segments_info C: (" << segments_info.opt_point_c.x << ", "
              << segments_info.opt_point_c.y << ", "
              << segments_info.opt_point_c.theta << ")" << std::endl;
    std::cout << "segments_info D: (" << segments_info.opt_point_d.x << ", "
              << segments_info.opt_point_d.y << ", "
              << segments_info.opt_point_d.theta << ")" << std::endl;
    std::cout << "segments_info E: (" << segments_info.opt_point_e.x << ", "
              << segments_info.opt_point_e.y << ", "
              << segments_info.opt_point_e.theta << ")" << std::endl;
    std::cout << "segments_info F: (" << segments_info.opt_point_f.x << ", "
              << segments_info.opt_point_f.y << ", "
              << segments_info.opt_point_f.theta << ")" << std::endl;
    if (IsSegmentExist(segments_info.opt_point_d)) {
      GenerateCDSegmentTrajectory(segments_info, planning_output);
    }
    if (IsSegmentExist(segments_info.opt_point_e)) {
      GenerateDESegmentTrajectory(segments_info, planning_output);
    }

#endif

    // PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    std::cout << "plan a-b fail, point_a, x:" << point_a.x
              << ", y:" << point_a.y << ", theta:" << point_a.theta
              << std::endl;
    return false;
  }

  return true;
}

bool DiagonalInTrajectoryGenerator::ReverseABSegmentPlan(
    const PlanningPoint& point_a, bool is_start, int idx,
    DiagonalInGeometryPlan* const geometry_planning,
    PlanningOutput* const planning_output) const {
  DiagonalSegmentsInfo segments_info;
  segments_info.opt_point_a = point_a;
  if (geometry_planning->ReverseABSegment(point_a, is_start, is_rough_calc_,
                                          &segments_info)) {
    std::cout << "plan r_a-c success" << std::endl;
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      planning_output->add_successful_slot_info_list()->set_id(
          managed_parking_fusion_info_.parking_fusion_slot_lists()[idx].id());
      return true;
    }
    GenerateRACSegmentTrajectory(segments_info, planning_output);
    GenerateCDSegmentTrajectory(segments_info, planning_output);
    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    std::cout << "plan r_a_c fail, point_a, x:" << point_a.x
              << ", y:" << point_a.y << ", theta:" << point_a.theta
              << std::endl;
    return false;
  }
  return true;
}

bool DiagonalInTrajectoryGenerator::BCSegmentPlan(
    const PlanningPoint& point_b, bool is_start, int idx,
    DiagonalInGeometryPlan* const geometry_planning,
    PlanningOutput* const planning_output) const {
  DiagonalSegmentsInfo segments_info;
  segments_info.opt_point_b = point_b;
  if (geometry_planning->BCSegment(point_b, is_start, is_rough_calc_, 0.0,
                                   &segments_info)) {
    std::cout << "plan b-c success" << std::endl;
    GenerateBCSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    std::cout << "plan b-c fail, point_b, x:" << point_b.x
              << ", y:" << point_b.y << ", theta:" << point_b.theta
              << std::endl;
    return false;
  }

  return true;
}

bool DiagonalInTrajectoryGenerator::CDSegmentPlan(
    const PlanningPoint& point_c, bool is_start, int idx,
    DiagonalInGeometryPlan* const geometry_planning,
    PlanningOutput* const planning_output) const {
  DiagonalSegmentsInfo segments_info;
  segments_info.opt_point_c = point_c;
  if (geometry_planning->CDSegment(point_c, is_start, is_rough_calc_,
                                   &segments_info)) {
    std::cout << "plan c-d success" << std::endl;
    GenerateCDSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    std::cout << "plan c-d fail, point_c, x:" << point_c.x
              << ", y:" << point_c.y << ", theta:" << point_c.theta
              << std::endl;
    return false;
  }

  return true;
}

bool DiagonalInTrajectoryGenerator::DESegmentPlan(
    const PlanningPoint& point_d, bool is_start, int idx,
    DiagonalInGeometryPlan* const geometry_planning,
    PlanningOutput* const planning_output) const {
  DiagonalSegmentsInfo segments_info;
  segments_info.opt_point_d = point_d;
  if (geometry_planning->DESegment(point_d, is_start, is_rough_calc_,
                                   &segments_info)) {
    std::cout << "plan d-e success" << std::endl;
    GenerateDESegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    std::cout << "plan d-e fail, point_d, x:" << point_d.x
              << ", y:" << point_d.y << ", theta:" << point_d.theta
              << std::endl;
    return false;
  }

  return true;
}

bool DiagonalInTrajectoryGenerator::GenerateABSegmentTrajectory(
    const DiagonalSegmentsInfo& segments_info,
    PlanningOutput* const planning_output) const {
  double line_step = kStep;
  const auto& point_a = segments_info.opt_point_a;
  const auto& point_b = segments_info.opt_point_b;

  const double segment_len =
      std::hypot(point_b.x - point_a.x, point_b.y - point_a.y);
  int size_i_ab = static_cast<int>(segment_len / line_step);
  if (!IsSamePoint(point_a, point_b)) {
    size_i_ab = std::max(size_i_ab, 1);
  }
  if (size_i_ab > 0) {
    line_step = segment_len / size_i_ab;
  }

  const double cos_point_a_theta = apa_cos(point_a.theta);
  const double sin_point_a_theta = apa_sin(point_a.theta);

  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  double s = 0.0;
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  for (int i = 0; i <= size_i_ab; ++i) {
    point_tmp_in_slot.x = point_a.x + cos_point_a_theta * line_step * i;
    point_tmp_in_slot.y = point_a.y + sin_point_a_theta * line_step * i;
    point_tmp_in_slot.theta = point_a.theta;
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint* trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(0.0);
    GetCurPtSpeed(1.0, trajectory_point);
    trajectory_point->set_distance(s);
    trajectory_point->set_jerk(0.0);
    s += line_step;
  }
  return true;
}

bool DiagonalInTrajectoryGenerator::GenerateRACSegmentTrajectory(
    const DiagonalSegmentsInfo& segments_info,
    PlanningOutput* const planning_output) const {
  double line_step = kStep;
  const auto& point_a = segments_info.opt_point_a;
  const auto& point_b = segments_info.opt_point_c;

  const double segment_len =
      std::hypot(point_b.x - point_a.x, point_b.y - point_a.y);
  int size_i_ab = static_cast<int>(segment_len / line_step);
  if (!IsSamePoint(point_a, point_b)) {
    size_i_ab = std::max(size_i_ab, 1);
  }
  if (size_i_ab > 0) {
    line_step = segment_len / size_i_ab;
  }

  const double cos_point_a_theta = apa_cos(point_a.theta);
  const double sin_point_a_theta = apa_sin(point_a.theta);

  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  double s = 0.0;
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  for (int i = 0; i <= size_i_ab; ++i) {
    point_tmp_in_slot.x = point_a.x - cos_point_a_theta * line_step * i;
    point_tmp_in_slot.y = point_a.y - sin_point_a_theta * line_step * i;
    point_tmp_in_slot.theta = point_a.theta;
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint* trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(0.0);
    GetCurPtSpeed(-1.0, trajectory_point);
    trajectory_point->set_distance(s);
    trajectory_point->set_jerk(0.0);
    s += line_step;
  }
  return true;
}

bool DiagonalInTrajectoryGenerator::GenerateBCSegmentTrajectory(
    const DiagonalSegmentsInfo& segments_info,
    PlanningOutput* const planning_output) const {
  const auto& point_b = segments_info.opt_point_b;
  const auto& point_c = segments_info.opt_point_c;
  const double theta_diff = fabs(point_b.theta - point_c.theta);
  const double radius_bc = segments_info.opt_radius_bc;
  double yaw_step = kYawStep / radius_bc;
  int size_i_bc = static_cast<int>(theta_diff / yaw_step);
  if (!IsSamePoint(point_b, point_c)) {
    size_i_bc = std::max(size_i_bc, 1);
  }
  if (size_i_bc > 0) {
    yaw_step = theta_diff / size_i_bc;
  }

  const double step_size = yaw_step * radius_bc;
  const double curvature_bc = slot_sign_ / radius_bc;

  const double cos_point_b_theta = apa_cos(point_b.theta);
  const double sin_point_b_theta = apa_sin(point_b.theta);

  double s = 0.0;
  int start_index = 0;
  if (planning_output->trajectory().trajectory_points_size() != 0) {
    s = planning_output->trajectory().trajectory_points().rbegin()->distance() +
        step_size;
    start_index = 1;
  }
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  for (int i = start_index; i <= size_i_bc; ++i) {
    point_tmp_in_slot.theta = point_b.theta + slot_sign_ * yaw_step * i;
    point_tmp_in_slot.x =
        point_b.x - slot_sign_ * radius_bc * sin_point_b_theta +
        slot_sign_ * radius_bc * apa_sin(point_tmp_in_slot.theta);
    point_tmp_in_slot.y =
        point_b.y + slot_sign_ * radius_bc * cos_point_b_theta -
        slot_sign_ * radius_bc * apa_cos(point_tmp_in_slot.theta);
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint* trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(curvature_bc);
    GetCurPtSpeed(1.0, trajectory_point);
    trajectory_point->set_distance(s);
    trajectory_point->set_jerk(0.0);
    s += step_size;
  }
  trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);

  return true;
}

bool DiagonalInTrajectoryGenerator::GenerateCDSegmentTrajectory(
    const DiagonalSegmentsInfo& segments_info,
    PlanningOutput* const planning_output) const {
  double s = 0.0;
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  const auto& point_c = segments_info.opt_point_c;
  const auto& point_d = segments_info.opt_point_d;
  const double radius_cd = segments_info.opt_radius_cd;
  const double theta_diff = fabs(point_d.theta - point_c.theta);
  double yaw_step = kYawStep / radius_cd;
  int size_i_cd = static_cast<int>(theta_diff / yaw_step);
  if (!IsSamePoint(point_c, point_d)) {
    size_i_cd = std::max(size_i_cd, 1);
  }
  if (size_i_cd > 0) {
    yaw_step = theta_diff / size_i_cd;
  }

  const double step_size = yaw_step * radius_cd;
  const double curvature_cd = -slot_sign_ / radius_cd;
  const double cos_point_c_theta = apa_cos(point_c.theta);
  const double sin_point_c_theta = apa_sin(point_c.theta);
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  for (int i = 0; i <= size_i_cd; ++i) {
    point_tmp_in_slot.theta = point_c.theta + slot_sign_ * yaw_step * i;
    point_tmp_in_slot.x =
        point_c.x + slot_sign_ * radius_cd * sin_point_c_theta -
        slot_sign_ * radius_cd * apa_sin(point_tmp_in_slot.theta);
    point_tmp_in_slot.y =
        point_c.y - slot_sign_ * radius_cd * cos_point_c_theta +
        slot_sign_ * radius_cd * apa_cos(point_tmp_in_slot.theta);
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint* trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(curvature_cd);
    GetCurPtSpeed(-1.0, trajectory_point);
    trajectory_point->set_distance(s);
    trajectory_point->set_jerk(0.0);
    // GetCurPtSpeed(segment_len, s, -1.0, trajectory_point);
    s += step_size;
  }

  if (std::fabs(point_d.theta - target_point_in_slot_.theta) < kEps) {
    double line_step = kStep;
    const double segment_len = fabs(point_d.y - target_point_in_slot_.y);
    int size_i_f_end = static_cast<int>(segment_len / line_step);
    if (!IsSamePoint(point_c, point_d)) {
      size_i_f_end = std::max(size_i_f_end, 1);
    }
    if (size_i_f_end > 0) {
      line_step = segment_len / size_i_f_end;
    }

    s = trajectory->trajectory_points().rbegin()->distance() + line_step;
    for (int i = 1; i <= size_i_f_end; ++i) {
      point_tmp_in_slot.x = point_d.x;
      point_tmp_in_slot.y = point_d.y - slot_sign_ * line_step * i;
      point_tmp_in_slot.theta = point_d.theta;
      point_tmp_in_odom =
          FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
      TrajectoryPoint* trajectory_point = trajectory->add_trajectory_points();
      trajectory_point->set_x(point_tmp_in_odom.x);
      trajectory_point->set_y(point_tmp_in_odom.y);
      trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
      trajectory_point->set_curvature(0.0);
      GetCurPtSpeed(-1.0, trajectory_point);
      trajectory_point->set_distance(s);
      trajectory_point->set_jerk(0.0);
      s += line_step;
    }
  }

  trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);

  return true;
}

bool DiagonalInTrajectoryGenerator::GenerateDESegmentTrajectory(
    const DiagonalSegmentsInfo& segments_info,
    PlanningOutput* const planning_output) const {
  DiagonalSegmentsInfo segments_info_tmp;
  segments_info_tmp.opt_point_b = segments_info.opt_point_d;
  segments_info_tmp.opt_point_c = segments_info.opt_point_e;
  segments_info_tmp.opt_radius_bc = segments_info.opt_radius_de;
  GenerateBCSegmentTrajectory(segments_info_tmp, planning_output);

  return true;
}

void DiagonalInTrajectoryGenerator::SetApaObjectInfo(
    int idx, DiagonalInGeometryPlan* geometry_planning) {
  objects_map_in_global_cor_.clear();
  objects_map_in_global_cor_.reserve(4);

  const auto& slots = managed_parking_fusion_info_.parking_fusion_slot_lists();

  // consider slot line as obstacle
  // TODO(xjli32): use real obstacles instead
  objects_map_in_global_cor_.emplace_back(
      Vec2d(raw_slot_points_in_m_[2].x, raw_slot_points_in_m_[2].y),
      Vec2d(raw_slot_points_in_m_[3].x, raw_slot_points_in_m_[3].y));
  // if (slots[idx].type() !=
  //     Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
  //   objects_map_in_global_cor_.emplace_back(
  //       Vec2d(raw_slot_points_in_m_[0].x, raw_slot_points_in_m_[0].y),
  //       Vec2d(raw_slot_points_in_m_[2].x, raw_slot_points_in_m_[2].y));
  //   objects_map_in_global_cor_.emplace_back(
  //       Vec2d(raw_slot_points_in_m_[3].x, raw_slot_points_in_m_[3].y),
  //       Vec2d(raw_slot_points_in_m_[1].x, raw_slot_points_in_m_[1].y));
  // }

  // mocked obstacle to avoid collision with opposite object
  // TODO(xjli32): use real obstacles instead
  double mocked_obj_y_offset = 10.0;
  if (slots[idx].type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    mocked_obj_y_offset = kMockedObjYOffset;
  }
  const double obj_half_len = 10.0;

  const double vec_10_x =
      raw_slot_points_in_m_[0].x - raw_slot_points_in_m_[1].x;
  const double vec_10_y =
      raw_slot_points_in_m_[0].y - raw_slot_points_in_m_[1].y;
  const double len_vec_10 = std::hypot(vec_10_x, vec_10_y);
  const double unit_10_x = vec_10_x / len_vec_10;
  const double unit_10_y = vec_10_y / len_vec_10;

  const double center_10_x =
      (raw_slot_points_in_m_[0].x + raw_slot_points_in_m_[1].x) * 0.5;
  const double center_10_y =
      (raw_slot_points_in_m_[0].y + raw_slot_points_in_m_[1].y) * 0.5;

  const double rotated_unit_10_x = -slot_sign_ * unit_10_y;
  const double rotated_unit_10_y = slot_sign_ * unit_10_x;

  const double obj_center_x =
      center_10_x + rotated_unit_10_x * mocked_obj_y_offset;
  const double obj_center_y =
      center_10_y + rotated_unit_10_y * mocked_obj_y_offset;

  PlanningPoint mocked_obj_pt0;
  PlanningPoint mocked_obj_pt1;
  mocked_obj_pt0.x = obj_center_x + obj_half_len * unit_10_x;
  mocked_obj_pt0.y = obj_center_y + obj_half_len * unit_10_y;
  mocked_obj_pt1.x = obj_center_x - obj_half_len * unit_10_x;
  mocked_obj_pt1.y = obj_center_y - obj_half_len * unit_10_y;
  std::cout << "mocked obj x:" << mocked_obj_pt0.x << ", y:" << mocked_obj_pt0.y
            << ", x:" << mocked_obj_pt1.x << ", y:" << mocked_obj_pt1.y
            << std::endl;
  objects_map_in_global_cor_.emplace_back(
      Vec2d(mocked_obj_pt0.x, mocked_obj_pt0.y),
      Vec2d(mocked_obj_pt1.x, mocked_obj_pt1.y));

  std::vector<LineSegment2d> objects_map;
  objects_map.reserve(objects_map_in_global_cor_.size());
  for (const auto& obs : objects_map_in_global_cor_) {
    PlanningPoint p0 = FromGlobal2LocalCor(
        slot_origin_in_odom_,
        PlanningPoint(obs.start().x(), obs.start().y(), 0.0));
    PlanningPoint p1 = FromGlobal2LocalCor(
        slot_origin_in_odom_, PlanningPoint(obs.end().x(), obs.end().y(), 0.0));
    objects_map.emplace_back(Vec2d(p0.x, p0.y), Vec2d(p1.x, p1.y));
  }

  geometry_planning->SetObjectMap(objects_map);
}

void DiagonalInTrajectoryGenerator::SetGeometryPlanningParameter(
    int idx, DiagonalInGeometryPlan* geometry_planning) {
  SetApaObjectInfo(idx, geometry_planning);
  geometry_planning->SetSlotType(slot_sign_);
  geometry_planning->SetTargetPoint(target_point_in_slot_);
}

PlanningPoint DiagonalInTrajectoryGenerator::FromLocal2GlobalCor(
    const PlanningPoint& ego, const PlanningPoint& local) const {
  PlanningPoint out;
  const double sin_ego_theta = apa_sin(ego.theta);
  const double cos_ego_theta = apa_cos(ego.theta);
  out.x = ego.x + local.x * cos_ego_theta - local.y * sin_ego_theta;
  out.y = ego.y + local.x * sin_ego_theta + local.y * cos_ego_theta;
  out.theta = planning_math::NormalizeAngle(ego.theta + local.theta);
  return out;
}

PlanningPoint DiagonalInTrajectoryGenerator::FromGlobal2LocalCor(
    const PlanningPoint& ego, const PlanningPoint& global) const {
  PlanningPoint out;
  const double sin_ego_theta = apa_sin(ego.theta);
  const double cos_ego_theta = apa_cos(ego.theta);
  out.x =
      (global.x - ego.x) * cos_ego_theta + (global.y - ego.y) * sin_ego_theta;
  out.y =
      (global.y - ego.y) * cos_ego_theta - (global.x - ego.x) * sin_ego_theta;
  out.theta = planning_math::NormalizeAngle(global.theta - ego.theta);
  return out;
}

double DiagonalInTrajectoryGenerator::CalApaTargetY() const {
  const double dst_front_edge_to_rear_axle = 3.624;
  const double dst_rear_edge_to_rear_axle = 0.947;
  const double stop_buffer = 0.10;
  double end_point_y_by_veh = dst_front_edge_to_rear_axle + stop_buffer;
  const double slot_depth =
      std::hypot(slot_points_in_m_[0].x - slot_points_in_m_[2].x,
                 slot_points_in_m_[0].y - slot_points_in_m_[2].y);
  double end_point_y_by_slot =
      slot_depth - stop_buffer - dst_rear_edge_to_rear_axle;
  return -slot_sign_ * std::fmin(end_point_y_by_veh, end_point_y_by_slot);
}

double DiagonalInTrajectoryGenerator::CalApaTargetX() const {
  PlanningPoint mid_point_in_odom;
  mid_point_in_odom.x = (slot_points_in_m_[0].x + slot_points_in_m_[1].x) * 0.5;
  mid_point_in_odom.y = (slot_points_in_m_[0].y + slot_points_in_m_[1].y) * 0.5;
  PlanningPoint mid_point_in_slot =
      FromGlobal2LocalCor(slot_origin_in_odom_, mid_point_in_odom);
  return mid_point_in_slot.x;
}

void DiagonalInTrajectoryGenerator::CalApaTargetInSlot() {
  target_point_in_slot_.x = CalApaTargetX();
  target_point_in_slot_.y = CalApaTargetY();
  target_point_in_slot_.theta = M_PI_2 * slot_sign_;

  target_point_in_odom_ =
      FromLocal2GlobalCor(slot_origin_in_odom_, target_point_in_slot_);
}

void DiagonalInTrajectoryGenerator::CalEgoPostionInSlotAndOdom() {
  const auto& pose = local_view_->localization_estimate.pose();
  cur_pos_in_odom_.x = pose.local_position().x();
  cur_pos_in_odom_.y = pose.local_position().y();
  cur_pos_in_odom_.theta = pose.euler_angles().yaw();
  cur_pos_in_slot_ =
      FromGlobal2LocalCor(slot_origin_in_odom_, cur_pos_in_odom_);

  std::cout << "cur_pos_in_odom_ x:" << cur_pos_in_odom_.x
            << ", y:" << cur_pos_in_odom_.y
            << ", theta:" << cur_pos_in_odom_.theta << std::endl;

  std::cout << "cur_pos_in_slot_ x:" << cur_pos_in_slot_.x
            << ", y:" << cur_pos_in_slot_.y
            << ", theta:" << cur_pos_in_slot_.theta << std::endl;
}

void DiagonalInTrajectoryGenerator::GetCurPtSpeed(
    const double segment_len, const double cur_s, const double spd_sign,
    TrajectoryPoint* trajectory_point) const {
  double t = 0.0;
  double speed = 0.0;
  double acc = 0.0;
  if (segment_len > kMinSegmentLen) {
    if (cur_s < 0.5 * kMinSegmentLen) {
      speed = std::sqrt(2.0 * kMaxAcc * cur_s);
      acc = kMaxAcc;
      t = speed / kMaxAcc;
    } else if (cur_s < segment_len - 0.5 * kMinSegmentLen) {
      speed = kMaxSpd;
      acc = 0.0;
      t = kMaxSpd / kMaxAcc + (cur_s - 0.5 * kMinSegmentLen) / kMaxSpd;
    } else {
      speed = std::sqrt(2.0 * kMaxAcc * fmax(segment_len - cur_s, 0.0));
      acc = -kMaxAcc;
      t = kMaxSpd / kMaxAcc + (cur_s - 0.5 * kMinSegmentLen) / kMaxSpd +
          (kMaxSpd - speed) / kMaxAcc;
    }
  } else {
    if (cur_s < 0.5 * segment_len) {
      speed = std::sqrt(2.0 * kMaxAcc * cur_s);
      acc = kMaxAcc;
      t = speed / kMaxAcc;
    } else {
      speed = std::sqrt(2.0 * kMaxAcc * fmax(segment_len - cur_s, 0.0));
      acc = -kMaxAcc;
      const double max_speed = std::sqrt(segment_len * kMaxAcc);
      t = max_speed / kMaxAcc + (max_speed - speed) / kMaxAcc;
    }
  }

  trajectory_point->set_t(t);
  trajectory_point->set_v(speed * spd_sign);
  trajectory_point->set_a(acc);
}

void DiagonalInTrajectoryGenerator::GetCurPtSpeed(
    const double spd_sign, TrajectoryPoint* trajectory_point) const {
  trajectory_point->set_t(0.0);
  trajectory_point->set_v(kMaxSpd * spd_sign);
  trajectory_point->set_a(0.0);
}

void DiagonalInTrajectoryGenerator::CalSlotOriginInodom() {
  slot_origin_in_odom_.x = slot_points_in_m_[0].x;
  slot_origin_in_odom_.y = slot_points_in_m_[0].y;
  slot_origin_in_odom_.theta =
      std::atan2(slot_points_in_m_[0].y - slot_points_in_m_[1].y,
                 slot_points_in_m_[0].x - slot_points_in_m_[1].x);
}

void DiagonalInTrajectoryGenerator::CalSlotPointsInM(const int idx) {
  const auto& slot_points =
      managed_parking_fusion_info_.parking_fusion_slot_lists()[idx]
          .corner_points();
  const double x0 = static_cast<double>(slot_points[0].x());
  const double y0 = static_cast<double>(slot_points[0].y());
  const double x1 = static_cast<double>(slot_points[1].x());
  const double y1 = static_cast<double>(slot_points[1].y());
  const double x2 = static_cast<double>(slot_points[2].x());
  const double y2 = static_cast<double>(slot_points[2].y());
  const double x3 = static_cast<double>(slot_points[3].x());
  const double y3 = static_cast<double>(slot_points[3].y());
  raw_slot_points_in_m_.clear();
  raw_slot_points_in_m_.reserve(4);
  raw_slot_points_in_m_.emplace_back(x0, y0, 0.0);
  raw_slot_points_in_m_.emplace_back(x1, y1, 0.0);
  raw_slot_points_in_m_.emplace_back(x2, y2, 0.0);
  raw_slot_points_in_m_.emplace_back(x3, y3, 0.0);
  slot_points_in_m_ = raw_slot_points_in_m_;

  const auto pO = k * n + pM;

  SquareSlot();
}

int DiagonalInTrajectoryGenerator::CalSlotSide(const int slot_index) {
  const auto& slots = managed_parking_fusion_info_.parking_fusion_slot_lists();

  PlanningPoint slot_point2_in_odom;
  slot_point2_in_odom.x = slots[slot_index].corner_points(2).x();
  slot_point2_in_odom.y = slots[slot_index].corner_points(2).y();

  slot_point2_in_slot_ =
      FromGlobal2LocalCor(slot_origin_in_odom_, slot_point2_in_odom);
  return (cur_pos_in_slot_.y >= slot_point2_in_slot_.y ? 1.0 : -1.0);
}

// assume slot is parallelogram
void DiagonalInTrajectoryGenerator::SquareSlot() {
  const double vec_01_x = slot_points_in_m_[1].x - slot_points_in_m_[0].x;
  const double vec_01_y = slot_points_in_m_[1].y - slot_points_in_m_[0].y;
  const double vec_02_x = slot_points_in_m_[2].x - slot_points_in_m_[0].x;
  const double vec_02_y = slot_points_in_m_[2].y - slot_points_in_m_[0].y;

  const double cross_prod = vec_02_x * vec_01_x + vec_02_y * vec_01_y;
  const double len_vec_02 = std::hypot(vec_02_x, vec_02_y);
  const double len_01_proj = std::fabs(cross_prod) / len_vec_02;

  const double unit_02_x = vec_02_x / len_vec_02;
  const double unit_02_y = vec_02_y / len_vec_02;

  if (cross_prod > 0) {
    // slot_angle < M_PI_2
    slot_points_in_m_[0].x = slot_points_in_m_[0].x + unit_02_x * len_01_proj;
    slot_points_in_m_[0].y = slot_points_in_m_[0].y + unit_02_y * len_01_proj;
    slot_points_in_m_[3].x = slot_points_in_m_[3].x - unit_02_x * len_01_proj;
    slot_points_in_m_[3].y = slot_points_in_m_[3].y - unit_02_y * len_01_proj;
  } else {
    // slot_angle >= M_PI_2
    slot_points_in_m_[1].x = slot_points_in_m_[1].x + unit_02_x * len_01_proj;
    slot_points_in_m_[1].y = slot_points_in_m_[1].y + unit_02_y * len_01_proj;
    slot_points_in_m_[2].x = slot_points_in_m_[2].x - unit_02_x * len_01_proj;
    slot_points_in_m_[2].y = slot_points_in_m_[2].y - unit_02_y * len_01_proj;
  }

  slot_width_ = std::hypot(slot_points_in_m_[0].x - slot_points_in_m_[1].x,
                           slot_points_in_m_[0].y - slot_points_in_m_[1].y);
  slot_length_ = std::hypot(slot_points_in_m_[0].x - slot_points_in_m_[2].x,
                            slot_points_in_m_[0].y - slot_points_in_m_[2].y);
}

void DiagonalInTrajectoryGenerator::UpdateMeasurement() {
  const auto& pose = local_view_->localization_estimate.pose();
  measure_.ego_pos << pose.local_position().x(), pose.local_position().y();
  measure_.heading = pose.heading();
  measure_.v_ego = local_view_->vehicle_service_output_info.vehicle_speed();
}

bool DiagonalInTrajectoryGenerator::IsReplan(
    PlanningOutput* const planning_output) {
  if (IsReplanEachFrame(local_view_->function_state_machine_info)) {
    return true;
  }

  planning_output->mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);
  const auto& local_position =
      local_view_->localization_estimate.pose().local_position();
  const double ego_x = local_position.x();
  const double ego_y = local_position.y();
  std::cout << "ego x:" << ego_x << ", y:" << ego_y << std::endl;
  std::cout << "veh_spd:"
            << local_view_->vehicle_service_output_info.vehicle_speed()
            << std::endl;

  if (!planning_output->has_trajectory() ||
      planning_output->trajectory().trajectory_points_size() == 0) {
    is_replan_ = true;
    last_segment_name_ = SEGMENT_NONE;
    return true;
  }

  double min_dis_sq = std::numeric_limits<double>::max();
  int min_dis_index = -1;
  const auto& traj = planning_output->trajectory();
  const int traj_point_size = traj.trajectory_points_size();
  const auto& traj_points = traj.trajectory_points();
  for (int i = 0; i < traj_point_size; ++i) {
    const double dis_sq = std::pow(ego_x - traj_points[i].x(), 2.0) +
                          std::pow(ego_y - traj_points[i].y(), 2.0);
    if (dis_sq < min_dis_sq) {
      min_dis_sq = dis_sq;
      min_dis_index = i;
    }
  }
  if (min_dis_index == -1) {
    std::cout << "nearest traj pt not found" << std::endl;
    return false;
  }
  std::cout << "min_dis_index:" << min_dis_index
            << ", min_dis:" << std::sqrt(min_dis_sq) << std::endl;

  const double remaining_s =
      traj_points.rbegin()->distance() - traj_points[min_dis_index].distance();
  std::cout << "remaining_s:" << remaining_s << std::endl;

  if (standstill_time_ >= kMinStandstillTime &&
      pos_unchanged_cnt_ >= kMinPosUnchangedCount) {
    if (remaining_s <= kRemainingDisThreshold) {
      return true;
    }
  }

  return false;
}

void DiagonalInTrajectoryGenerator::SetPlanningOutputInfo(
    PlanningOutput* const planning_output) const {
  if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
    return;
  }

  auto gear_command = planning_output->mutable_gear_command();
  gear_command->set_available(true);
  if (planning_output->trajectory().trajectory_points()[0].v() >= 0.0) {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE);
  } else {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);
  }
  std::cout << "gear:" << gear_command->gear_command_value() << std::endl;
}

bool DiagonalInTrajectoryGenerator::IsSamePoint(const PlanningPoint& p1,
                                                const PlanningPoint& p2) const {
  return fabs(p1.x - p2.x) < kEps && abs(p1.y - p2.y) < kEps &&
         abs(p1.theta - p2.theta) < kEps;
}

void DiagonalInTrajectoryGenerator::UpdateStandstillTime() {
  const uint64_t cur_time = IflyTime::Now_ms();
  const double veh_spd_mps =
      fabs(local_view_->vehicle_service_output_info.vehicle_speed());
  if (veh_spd_mps < kStanstillSpd) {
    if (last_time_ != 0) {
      standstill_time_ += cur_time - last_time_;
    }
  } else {
    standstill_time_ = 0;
  }
  last_time_ = cur_time;
}

void DiagonalInTrajectoryGenerator::UpdatePosUnchangedCount() {
  if (std::fabs(last_pos_in_odom_.x - cur_pos_in_odom_.x) < kEps &&
      std::fabs(last_pos_in_odom_.y - cur_pos_in_odom_.y) < kEps) {
    ++pos_unchanged_cnt_;
  } else {
    pos_unchanged_cnt_ = 0;
  }
  last_pos_in_odom_ = cur_pos_in_odom_;
}

bool DiagonalInTrajectoryGenerator::IsApaFinished() const {
  return current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL &&
         standstill_time_ >= kMinStandstillTime &&
         fabs(target_point_in_slot_.x - cur_pos_in_slot_.x) < kMaxXOffset &&
         fabs(target_point_in_slot_.y - cur_pos_in_slot_.y) < kMaxYOffset &&
         fabs(target_point_in_slot_.theta - cur_pos_in_slot_.theta) <
             kMaxThetaOffset;
}

bool DiagonalInTrajectoryGenerator::IsSimulatedApaFinished() const {
  return current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL &&
         fabs(target_point_in_slot_.x - cur_pos_in_slot_.x) < kMaxXOffset &&
         fabs(target_point_in_slot_.y - cur_pos_in_slot_.y) < kMaxYOffset &&
         fabs(target_point_in_slot_.theta - cur_pos_in_slot_.theta) <
             kMaxThetaOffset;
}

void DiagonalInTrajectoryGenerator::PrintTrajectoryPoints(
    const PlanningOutput& planning_output) const {
  const auto& traj = planning_output.trajectory();
  const int traj_point_num = traj.trajectory_points_size();
  for (int i = 0; i < traj_point_num; ++i) {
    const auto& pt = traj.trajectory_points()[i];
    std::cout << "seg traj pt [" << i << "], x:" << pt.x() << ", y:" << pt.y()
              << ", theta:" << pt.heading_yaw() << ", kappa:" << pt.curvature()
              << ", v:" << pt.v() << ", s:" << pt.distance() << std::endl;
  }
}

bool DiagonalInTrajectoryGenerator::IsSegmentExist(
    const PlanningPoint& end_point) const {
  if (std::fabs(end_point.x) < 1e-6 && std::fabs(end_point.y) < 1e-6) {
    return false;
  } else {
    return true;
  }
}

void DiagonalInTrajectoryGenerator::PrintSlotInfo() const {
  if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
    return;
  }

  std::cout << "slot_points_in_m_ x0:" << slot_points_in_m_[0].x
            << ", y0:" << slot_points_in_m_[0].y
            << ", x1:" << slot_points_in_m_[1].x
            << ", y1:" << slot_points_in_m_[1].y
            << ", x2:" << slot_points_in_m_[2].x
            << ", y2:" << slot_points_in_m_[2].y
            << ", x3:" << slot_points_in_m_[3].x
            << ", y3:" << slot_points_in_m_[3].y << std::endl;

  std::cout << "slot_width_:" << slot_width_
            << ", slot_length_:" << slot_length_ << std::endl;

  std::cout << "slot origin in odom x:" << slot_origin_in_odom_.x
            << ", y:" << slot_origin_in_odom_.y
            << ", theta:" << slot_origin_in_odom_.theta << std::endl;

  std::cout << "target_point_in_slot_ x:" << target_point_in_slot_.x
            << ", y:" << target_point_in_slot_.y
            << ", theta:" << target_point_in_slot_.theta << std::endl;

  std::cout << "target_point_in_odom_ x:" << target_point_in_odom_.x
            << ", y:" << target_point_in_odom_.y
            << ", theta:" << target_point_in_odom_.theta << std::endl;
}

}  // namespace apa_planner
}  // namespace planning
