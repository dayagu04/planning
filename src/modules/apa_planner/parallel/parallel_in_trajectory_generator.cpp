#include "parallel/parallel_in_trajectory_generator.h"

#include <limits>

#include "common/apa_cos_sin.h"
#include "common/apa_utils.h"
#include "common/planning_log_helper.h"
#include "common/vehicle_param_helper.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "math/line_segment2d.h"
#include "math/math_utils.h"
#include "planning_output_context.h"
#include "utils_math.h"

namespace planning {
namespace apa_planner {

using ::Common::GearCommandValue;
using ::FuncStateMachine::FunctionalState;
using ::PlanningOutput::PlanningOutput;
using ::PlanningOutput::Trajectory;
using ::PlanningOutput::TrajectoryPoint;
using framework::Frame;
using planning::planning_math::LineSegment2d;
using planning::planning_math::Vec2d;

namespace {
constexpr double kEps = 1e-6;
constexpr double kMaxAcc = 0.5;
constexpr double kMaxSpd = 0.5;
constexpr double kMinSegmentLen = 0.5 * kMaxSpd * kMaxSpd / kMaxAcc;
constexpr double kStep = 0.1;
constexpr double kYawStep = 0.1;
constexpr double kLateralBuffer = 0.2;
constexpr double kStanstillSpd = 0.01;
constexpr double kRemainingDisThreshold = 0.2;
constexpr uint64_t kMinStandstillTime = 500;  // ms
constexpr uint64_t kMinPosUnchangedCount = 5;
constexpr double kMaxXOffset = 0.2;
constexpr double kMaxYOffset = 0.2;
constexpr double kMaxThetaOffset = 0.05;
constexpr double kMinSlotLength = 5.8;
constexpr double kMaxLenOfSmallSpeed = 1.0;
constexpr double kMaxSpdInLineStep = 0.5;
constexpr double kMinSpdInCircleStep = 0.4;
constexpr double kMockedObjYOffset = 4.0;
}  // namespace

ParallelInTrajectoryGenerator::ParallelInTrajectoryGenerator() {
  iterative_anchoring_smoother_ =
      std::make_unique<IterativeAnchoringSmoother>();
}

bool ParallelInTrajectoryGenerator::Plan(framework::Frame *const frame) {
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

  UpdateStandstillTime();

  UpdatePosUnchangedCount();

  const auto &parking_fusion_info = local_view_->parking_fusion_info;

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

  const auto &slots = parking_fusion_info.parking_fusion_slot_lists();
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
      if (selected_slot_id == slots[i].id() &&
          slots[i].type() ==
              Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
        select_slot_index = i;
        AINFO << "parallel slot selected";
        break;
      }
    }
    if (select_slot_index == -1) {
      AERROR << "selected slot is not parallel";
      return false;
    }
    return SingleSlotPlan(select_slot_index, planning_output);
  } else {
    bool is_planning_ok = false;
    for (int i = 0; i < parking_fusion_info.parking_fusion_slot_lists_size();
         ++i) {
      if (slots[i].type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
        AINFO << "parallel slot id:" << slots[i].id();
        is_planning_ok = SingleSlotPlan(i, planning_output) || is_planning_ok;
      }
    }
    return is_planning_ok;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::SingleSlotPlan(
    const int slot_index, PlanningOutput *const planning_output) {
  CalSlotPointsInM(slot_index);

  if (slot_length_ < kMinSlotLength) {
    AERROR << "slot_length_:" << slot_length_
           << ", lower than:" << kMinSlotLength;
    return false;
  }

  const auto &slots =
      local_view_->parking_fusion_info.parking_fusion_slot_lists();

  slot_sign_ = slots[slot_index].slot_side() == 0 ? -1.0 : 1.0;

  AINFO << "slot_sign_:" << slot_sign_;

  CalSlotOriginInodom(slot_index);

  CalApaTargetInSlot(slot_index);

  CalEgoPostionInSlotAndOdom(slot_index);

  if (IsApaFinished()) {
    AINFO << "apa is finished";
    SetFinishedPlanningOutput(frame_);
    return true;
  }

  if (!IsReplan(planning_output)) {
    return true;
  }
  AINFO << "parallel replan triggered, replan state:" << current_state_;
  AINFO << "cur segment name:" << last_segment_name_;

  PrintSlotInfo();

  if (!GeometryPlan(cur_pos_in_slot_, slot_index, planning_output)) {
    AERROR << "geometry parallel plan failed";
    return false;
  }

  SetPlanningOutputInfo(planning_output);

  return true;
}

bool ParallelInTrajectoryGenerator::GeometryPlan(
    const PlanningPoint &start_point, int idx,
    PlanningOutput *const planning_output) {
  geometry_planning_ = ParallelInGeometryPlan();
  SetGeometryPlanningParameter(idx, &geometry_planning_);
  bool is_planning_ok = false;
  planning_output->mutable_trajectory()->mutable_trajectory_points()->Clear();
  if (last_segment_name_.empty()) {
    if (ABSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      if (current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
        last_segment_name_ = "BC";
      }
      is_planning_ok = true;
    } else if (ReverseABSegmentPlan(start_point, true, idx, &geometry_planning_,
                                    planning_output)) {
      if (current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
        last_segment_name_ = "EF";
      }
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == "AB") {
    if (BCSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = "BC";
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == "BC") {
    if (CDSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = "EF";
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == "CD") {
    if (DESegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = "DE";
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == "DE") {
    if (EFSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = "EF";
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == "EF") {
    if (FHSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = "FH";
      is_planning_ok = true;
    }
  } else if (last_segment_name_ == "FH") {
    if (EFSegmentPlan(start_point, true, idx, &geometry_planning_,
                      planning_output)) {
      last_segment_name_ = "EF";
      is_planning_ok = true;
    }
  } else {
    AERROR << "Invalid parallel segment name";
    return false;
  }

  if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
    last_segment_name_.clear();
  }

  // if (is_planning_ok) {
  //   apa_speed_smoother_.Smooth(objects_map_in_global_cor_, planning_output);
  // }

  return is_planning_ok;
}

bool ParallelInTrajectoryGenerator::ABSegmentPlan(
    const PlanningPoint &point_a, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) const {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_a = point_a;
  if (geometry_planning->ABSegment(point_a, is_start, is_rough_calc_,
                                   &segments_info)) {
    AINFO << "plan a-b success, slot index:" << idx;
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      planning_output->add_successful_slot_info_list()->set_id(
          local_view_->parking_fusion_info.parking_fusion_slot_lists()[idx]
              .id());
      return true;
    }
    GenerateABSegmentTrajectory(segments_info, planning_output);
    GenerateBCSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan a-b fail, point_a, x:" << point_a.x << ", y:" << point_a.y
           << ", theta:" << point_a.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::ReverseABSegmentPlan(
    const PlanningPoint &point_a, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) const {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_a = point_a;
  if (geometry_planning->ReverseABSegment(point_a, is_start, is_rough_calc_,
                                          &segments_info)) {
    AINFO << "plan backward a-b success";
    if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
      planning_output->add_successful_slot_info_list()->set_id(
          local_view_->parking_fusion_info.parking_fusion_slot_lists()[idx]
              .id());
      return true;
    }
    GenerateABSegmentTrajectory(segments_info, planning_output);
    GenerateCDSegmentTrajectory(segments_info, planning_output);
    GenerateDESegmentTrajectory(segments_info, planning_output);
    GenerateEFSegmentTrajectory(segments_info, planning_output);
    // GenerateSpdInMultiSegmentTrajectory(planning_output);
    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan backward a-b fail, point_a, x:" << point_a.x
           << ", y:" << point_a.y << ", theta:" << point_a.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::BCSegmentPlan(
    const PlanningPoint &point_b, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) const {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_b = point_b;
  if (geometry_planning->BCSegment(point_b, is_start, is_rough_calc_, 0.0,
                                   &segments_info)) {
    AINFO << "plan b-c success";
    GenerateBCSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan b-c fail, point_b, x:" << point_b.x << ", y:" << point_b.y
           << ", theta:" << point_b.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::CDSegmentPlan(
    const PlanningPoint &point_c, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) const {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_c = point_c;
  if (geometry_planning->CDSegment(point_c, is_start, is_rough_calc_,
                                   &segments_info)) {
    AINFO << "plan c-d success";
    GenerateCDSegmentTrajectory(segments_info, planning_output);
    GenerateDESegmentTrajectory(segments_info, planning_output);
    GenerateEFSegmentTrajectory(segments_info, planning_output);

    // iterative_anchoring_smoother_->Smooth(
    //     objects_map_in_global_cor_, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan c-d fail, point_c, x:" << point_c.x << ", y:" << point_c.y
           << ", theta:" << point_c.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::DESegmentPlan(
    const PlanningPoint &point_d, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) const {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_d = point_d;
  if (geometry_planning->DESegment(point_d, is_start, is_rough_calc_,
                                   &segments_info)) {
    AINFO << "plan d-e success";
    GenerateDESegmentTrajectory(segments_info, planning_output);
    GenerateEFSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan d-e fail, point_d, x:" << point_d.x << ", y:" << point_d.y
           << ", theta:" << point_d.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::EFSegmentPlan(
    const PlanningPoint &point_e, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_e = point_e;
  if (geometry_planning->EFSegment(point_e, is_start, is_rough_calc_,
                                   &segments_info)) {
    AINFO << "plan e-f success";

    UpdateTargetPointInSlot(*geometry_planning);

    GenerateEFSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan e-f fail, point_e, x:" << point_e.x << ", y:" << point_e.y
           << ", theta:" << point_e.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::FHSegmentPlan(
    const PlanningPoint &point_f, bool is_start, int idx,
    ParallelInGeometryPlan *const geometry_planning,
    PlanningOutput *const planning_output) {
  ParallelSegmentsInfo segments_info;
  segments_info.opt_point_f = point_f;
  if (geometry_planning->FHSegment(point_f, is_start, is_rough_calc_,
                                   &segments_info)) {
    AINFO << "plan f-h success";

    UpdateTargetPointInSlot(*geometry_planning);

    GenerateFHSegmentTrajectory(segments_info, planning_output);

    PrintTrajectoryPoints(*planning_output);

    return true;
  } else {
    AERROR << "plan f-h fail, point_f, x:" << point_f.x << ", y:" << point_f.y
           << ", theta:" << point_f.theta;
    return false;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::GenerateABSegmentTrajectory(
    const ParallelSegmentsInfo &segments_info,
    PlanningOutput *const planning_output) const {
  double line_step = kStep;
  const auto &point_a = segments_info.opt_point_a;
  const auto &point_b = segments_info.opt_point_b;
  double spd_sign = 0.0;
  if (point_a.x > point_b.x + kEps) {
    spd_sign = -1.0;
  } else if (point_a.x < point_b.x - kEps) {
    spd_sign = 1.0;
  } else {
    spd_sign = 0.0;
  }

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
    TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(0.0);
    // GetCurPtSpeed(segment_len, s, 1.0, trajectory_point);
    GetCurPtSpeed(segment_len, spd_sign, trajectory_point);
    trajectory_point->set_distance(s);
    s += line_step;
  }
  return true;
}

bool ParallelInTrajectoryGenerator::GenerateBCSegmentTrajectory(
    const ParallelSegmentsInfo &segments_info,
    PlanningOutput *const planning_output) const {
  const auto &point_b = segments_info.opt_point_b;
  const auto &point_c = segments_info.opt_point_c;
  const double theta_diff = fabs(point_b.theta - point_c.theta);
  const double radius_bc = segments_info.opt_radius_bc;
  const double len_bc = theta_diff * radius_bc;
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
    TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(curvature_bc);
    trajectory_point->set_distance(s);
    GetCurPtSpeed(len_bc, 1.0, trajectory_point);
    s += step_size;
  }
  trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);

  return true;
}

bool ParallelInTrajectoryGenerator::GenerateCDSegmentTrajectory(
    const ParallelSegmentsInfo &segments_info,
    PlanningOutput *const planning_output) const {
  double s = 0.0;
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  const auto &point_c = segments_info.opt_point_c;
  const auto &point_d = segments_info.opt_point_d;
  const double radius_cd = segments_info.opt_radius_cd;
  const double theta_diff = fabs(point_d.theta - point_c.theta);
  const double len_cd = theta_diff * radius_cd;
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
    TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(curvature_cd);
    trajectory_point->set_distance(s);
    // GetCurPtSpeed(segment_len, s, -1.0, trajectory_point);
    GetCurPtSpeed(len_cd, -1.0, trajectory_point);
    s += step_size;
  }
  return true;
}

bool ParallelInTrajectoryGenerator::GenerateDESegmentTrajectory(
    const ParallelSegmentsInfo &segments_info,
    PlanningOutput *const planning_output) const {
  double line_step = kStep;
  const auto &point_d = segments_info.opt_point_d;
  const auto &point_e = segments_info.opt_point_e;

  const double segment_len =
      std::hypot(point_e.x - point_d.x, point_e.y - point_d.y);
  int size_i_de = static_cast<int>(segment_len / line_step);
  if (!IsSamePoint(point_d, point_e)) {
    size_i_de = std::max(size_i_de, 1);
  }
  if (size_i_de > 0) {
    line_step = segment_len / size_i_de;
  }

  const double cos_point_d_theta = apa_cos(point_d.theta);
  const double sin_point_d_theta = apa_sin(point_d.theta);

  double s = 0.0;
  int start_index = 0;
  if (planning_output->trajectory().trajectory_points_size() != 0) {
    s = planning_output->trajectory().trajectory_points().rbegin()->distance() +
        line_step;
    start_index = 1;
  }
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  for (int i = start_index; i <= size_i_de; ++i) {
    point_tmp_in_slot.x = point_d.x - cos_point_d_theta * line_step * i;
    point_tmp_in_slot.y = point_d.y - sin_point_d_theta * line_step * i;
    point_tmp_in_slot.theta = point_d.theta;
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(0.0);
    // GetCurPtSpeed(segment_len, s, -1.0, trajectory_point);
    GetCurPtSpeed(segment_len, -1.0, trajectory_point);
    trajectory_point->set_distance(s);
    s += line_step;
  }

  return true;
}

bool ParallelInTrajectoryGenerator::GenerateEFSegmentTrajectory(
    const ParallelSegmentsInfo &segments_info,
    PlanningOutput *const planning_output) const {
  const auto &point_e = segments_info.opt_point_e;
  const auto &point_f = segments_info.opt_point_f;
  const double radius_ef = segments_info.opt_radius_ef;
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  if (std::isinf(radius_ef)) {
    double line_step = kStep;
    const double segment_len = fabs(point_e.x - target_point_in_slot_.x);
    int size_i_e_end = static_cast<int>(segment_len / line_step);
    if (!IsSamePoint(point_e, point_f)) {
      size_i_e_end = std::max(size_i_e_end, 1);
    }
    if (size_i_e_end > 0) {
      line_step = segment_len / size_i_e_end;
    }

    const double cos_point_target_theta = apa_cos(target_point_in_slot_.theta);
    const double sin_point_target_theta = apa_sin(target_point_in_slot_.theta);
    double s = 0.0;
    for (int i = 0; i <= size_i_e_end; ++i) {
      point_tmp_in_slot.x = point_e.x - cos_point_target_theta * line_step * i;
      point_tmp_in_slot.y = point_e.y - sin_point_target_theta * line_step * i;
      point_tmp_in_slot.theta = target_point_in_slot_.theta;
      point_tmp_in_odom =
          FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
      TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
      trajectory_point->set_x(point_tmp_in_odom.x);
      trajectory_point->set_y(point_tmp_in_odom.y);
      trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
      trajectory_point->set_curvature(0.0);
      // GetCurPtSpeed(segment_len, s, 1.0, trajectory_point);
      GetCurPtSpeed(segment_len, -1.0, trajectory_point);
      trajectory_point->set_distance(s);
      s += line_step;
    }
    trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);
    return true;
  }

  const double theta_diff = fabs(point_f.theta - point_e.theta);
  const double len_ef = theta_diff * radius_ef;
  double yaw_step = kYawStep / radius_ef;
  int size_i_ef = static_cast<int>(theta_diff / yaw_step);
  if (!IsSamePoint(point_e, point_f)) {
    size_i_ef = std::max(size_i_ef, 1);
  }
  if (size_i_ef > 0) {
    yaw_step = theta_diff / size_i_ef;
  }

  yaw_step = theta_diff / size_i_ef;
  const double step_size = yaw_step * radius_ef;

  double s = 0.0;
  int start_index = 0;
  if (planning_output->trajectory().trajectory_points_size() != 0) {
    s = planning_output->trajectory().trajectory_points().rbegin()->distance() +
        step_size;
    start_index = 1;
  }

  const double curvature_ef = slot_sign_ / radius_ef;
  const double cos_point_e_theta = apa_cos(point_e.theta);
  const double sin_point_e_theta = apa_sin(point_e.theta);
  for (int i = start_index; i <= size_i_ef; ++i) {
    point_tmp_in_slot.theta = point_e.theta - slot_sign_ * yaw_step * i;
    point_tmp_in_slot.x =
        point_e.x - slot_sign_ * radius_ef * sin_point_e_theta +
        slot_sign_ * radius_ef * apa_sin(point_tmp_in_slot.theta);
    point_tmp_in_slot.y =
        point_e.y + slot_sign_ * radius_ef * cos_point_e_theta -
        slot_sign_ * radius_ef * apa_cos(point_tmp_in_slot.theta);
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(curvature_ef);
    trajectory_point->set_distance(s);
    GetCurPtSpeed(len_ef, -1.0, trajectory_point);
    s += step_size;
  }

  if (point_f.x >= target_point_in_slot_.x &&
      std::fabs(point_f.theta - target_point_in_slot_.theta) < kEps) {
    double line_step = kStep;
    const double segment_len = fabs(point_f.x - target_point_in_slot_.x);
    int size_i_f_end = static_cast<int>(segment_len / line_step);
    if (!IsSamePoint(point_e, point_f)) {
      size_i_f_end = std::max(size_i_f_end, 1);
    }
    if (size_i_f_end > 0) {
      line_step = segment_len / size_i_f_end;
    }

    const double cos_point_f_theta = apa_cos(point_f.theta);
    const double sin_point_f_theta = apa_sin(point_f.theta);

    s = trajectory->trajectory_points().rbegin()->distance() + line_step;
    for (int i = 1; i <= size_i_f_end; ++i) {
      point_tmp_in_slot.x = point_f.x - cos_point_f_theta * line_step * i;
      point_tmp_in_slot.y = point_f.y - sin_point_f_theta * line_step * i;
      point_tmp_in_slot.theta = point_f.theta;
      point_tmp_in_odom =
          FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
      TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
      trajectory_point->set_x(point_tmp_in_odom.x);
      trajectory_point->set_y(point_tmp_in_odom.y);
      trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
      trajectory_point->set_curvature(0.0);
      GetCurPtSpeed(segment_len, -1.0, trajectory_point);
      trajectory_point->set_distance(s);
      s += line_step;
    }
  }

  trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);

  return true;
}

bool ParallelInTrajectoryGenerator::GenerateFHSegmentTrajectory(
    const ParallelSegmentsInfo &segments_info,
    PlanningOutput *const planning_output) const {
  const auto &point_f = segments_info.opt_point_f;
  const auto &point_h = segments_info.opt_point_h;
  const double radius_fh = segments_info.opt_radius_fh;
  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  PlanningPoint point_tmp_in_odom;
  PlanningPoint point_tmp_in_slot;
  if (std::isinf(radius_fh)) {
    double line_step = kStep;
    const double segment_len = fabs(point_f.x - target_point_in_slot_.x);
    int size_i_f_end = static_cast<int>(segment_len / line_step);
    if (!IsSamePoint(point_f, point_h)) {
      size_i_f_end = std::max(size_i_f_end, 1);
    }
    if (size_i_f_end > 0) {
      line_step = segment_len / size_i_f_end;
    }

    const double cos_point_target_theta = apa_cos(target_point_in_slot_.theta);
    const double sin_point_target_theta = apa_sin(target_point_in_slot_.theta);
    double s = 0.0;
    for (int i = 0; i <= size_i_f_end; ++i) {
      point_tmp_in_slot.x = point_f.x + cos_point_target_theta * line_step * i;
      point_tmp_in_slot.y = point_f.y + sin_point_target_theta * line_step * i;
      point_tmp_in_slot.theta = target_point_in_slot_.theta;
      point_tmp_in_odom =
          FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
      TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
      trajectory_point->set_x(point_tmp_in_odom.x);
      trajectory_point->set_y(point_tmp_in_odom.y);
      trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
      trajectory_point->set_curvature(0.0);
      // GetCurPtSpeed(segment_len, s, 1.0, trajectory_point);
      GetCurPtSpeed(segment_len, 1.0, trajectory_point);
      trajectory_point->set_distance(s);
      s += line_step;
    }
    trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);
    return true;
  }

  const double theta_diff = fabs(point_h.theta - point_f.theta);
  const double len_fh = theta_diff * radius_fh;
  double yaw_step = kYawStep / radius_fh;

  int size_i_fh = static_cast<int>(theta_diff / yaw_step);
  if (!IsSamePoint(point_f, point_h)) {
    size_i_fh = std::max(size_i_fh, 1);
  }
  if (size_i_fh > 0) {
    yaw_step = theta_diff / size_i_fh;
  }

  const double step_size = yaw_step * radius_fh;
  double s = 0.0;

  const double curvature_fh = -slot_sign_ / radius_fh;
  const double cos_point_f_theta = apa_cos(point_f.theta);
  const double sin_point_f_theta = apa_sin(point_f.theta);
  for (int i = 0; i <= size_i_fh; ++i) {
    point_tmp_in_slot.theta = point_f.theta - slot_sign_ * yaw_step * i;
    point_tmp_in_slot.x =
        point_f.x + slot_sign_ * radius_fh * sin_point_f_theta -
        slot_sign_ * radius_fh * apa_sin(point_tmp_in_slot.theta);
    point_tmp_in_slot.y =
        point_f.y - slot_sign_ * radius_fh * cos_point_f_theta +
        slot_sign_ * radius_fh * apa_cos(point_tmp_in_slot.theta);
    point_tmp_in_odom =
        FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
    TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(point_tmp_in_odom.x);
    trajectory_point->set_y(point_tmp_in_odom.y);
    trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
    trajectory_point->set_curvature(curvature_fh);
    trajectory_point->set_distance(s);
    GetCurPtSpeed(len_fh, 1.0, trajectory_point);
    s += step_size;
  }

  if (point_h.x <= target_point_in_slot_.x &&
      std::fabs(point_h.theta - target_point_in_slot_.theta) < kEps) {
    double line_step = kStep;
    const double segment_len = fabs(point_h.x - target_point_in_slot_.x);
    int size_i_h_end = static_cast<int>(segment_len / line_step);
    if (!IsSamePoint(point_f, point_h)) {
      size_i_h_end = std::max(size_i_h_end, 1);
    }
    if (size_i_h_end > 0) {
      line_step = segment_len / size_i_h_end;
    }

    const double cos_point_h_theta = apa_cos(point_h.theta);
    const double sin_point_h_theta = apa_sin(point_h.theta);
    s = trajectory->trajectory_points().rbegin()->distance() + line_step;
    for (int i = 1; i <= size_i_h_end; ++i) {
      point_tmp_in_slot.x = point_h.x + cos_point_h_theta * line_step * i;
      point_tmp_in_slot.y = point_h.y + sin_point_h_theta * line_step * i;
      point_tmp_in_slot.theta = point_h.theta;
      point_tmp_in_odom =
          FromLocal2GlobalCor(slot_origin_in_odom_, point_tmp_in_slot);
      TrajectoryPoint *trajectory_point = trajectory->add_trajectory_points();
      trajectory_point->set_x(point_tmp_in_odom.x);
      trajectory_point->set_y(point_tmp_in_odom.y);
      trajectory_point->set_heading_yaw(point_tmp_in_odom.theta);
      trajectory_point->set_curvature(0.0);
      // GetCurPtSpeed(segment_len, s, 1.0, trajectory_point);
      GetCurPtSpeed(segment_len, 1.0, trajectory_point);
      trajectory_point->set_distance(s);
      s += line_step;
    }
  }
  trajectory->mutable_trajectory_points()->rbegin()->set_v(0.0);

  return true;
}

void ParallelInTrajectoryGenerator::SetApaObjectInfo(
    int idx, ParallelInGeometryPlan *geometry_planning) {
  objects_map_in_global_cor_.clear();
  objects_map_in_global_cor_.reserve(4);

  // consider slot line as obstacle
  objects_map_in_global_cor_.emplace_back(
      Vec2d(raw_slot_points_in_m_[0].x, raw_slot_points_in_m_[0].y),
      Vec2d(raw_slot_points_in_m_[2].x, raw_slot_points_in_m_[2].y));
  objects_map_in_global_cor_.emplace_back(
      Vec2d(raw_slot_points_in_m_[2].x, raw_slot_points_in_m_[2].y),
      Vec2d(raw_slot_points_in_m_[3].x, raw_slot_points_in_m_[3].y));
  objects_map_in_global_cor_.emplace_back(
      Vec2d(raw_slot_points_in_m_[3].x, raw_slot_points_in_m_[3].y),
      Vec2d(raw_slot_points_in_m_[1].x, raw_slot_points_in_m_[1].y));

  // mocked obstacle to avoid collision with opposite object
  const double mocked_obj_y_offset = kMockedObjYOffset;
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
  AINFO << "mocked obj x:" << mocked_obj_pt0.x << ", y:" << mocked_obj_pt0.y
        << ", x:" << mocked_obj_pt1.x << ", y:" << mocked_obj_pt1.y;
  // objects_map_in_global_cor_.emplace_back(
  //     Vec2d(mocked_obj_pt0.x, mocked_obj_pt0.y),
  //     Vec2d(mocked_obj_pt1.x, mocked_obj_pt1.y));

  std::vector<LineSegment2d> objects_map;
  objects_map.reserve(objects_map_in_global_cor_.size());
  for (const auto &obs : objects_map_in_global_cor_) {
    PlanningPoint p0 = FromGlobal2LocalCor(
        slot_origin_in_odom_,
        PlanningPoint(obs.start().x(), obs.start().y(), 0.0));
    PlanningPoint p1 = FromGlobal2LocalCor(
        slot_origin_in_odom_, PlanningPoint(obs.end().x(), obs.end().y(), 0.0));
    objects_map.emplace_back(Vec2d(p0.x, p0.y), Vec2d(p1.x, p1.y));
  }

  geometry_planning->SetObjectMap(objects_map);
}

void ParallelInTrajectoryGenerator::SetGeometryPlanningParameter(
    int idx, ParallelInGeometryPlan *geometry_planning) {
  SetApaObjectInfo(idx, geometry_planning);
  geometry_planning->SetSlotType(slot_sign_);
  geometry_planning->SetTargetPoint(target_point_in_slot_);
}

PlanningPoint ParallelInTrajectoryGenerator::FromLocal2GlobalCor(
    const PlanningPoint &ego, const PlanningPoint &local) const {
  PlanningPoint out;
  const double sin_ego_theta = apa_sin(ego.theta);
  const double cos_ego_theta = apa_cos(ego.theta);
  out.x = ego.x + local.x * cos_ego_theta - local.y * sin_ego_theta;
  out.y = ego.y + local.x * sin_ego_theta + local.y * cos_ego_theta;
  out.theta = planning_math::NormalizeAngle(ego.theta + local.theta);
  return out;
}

PlanningPoint ParallelInTrajectoryGenerator::FromGlobal2LocalCor(
    const PlanningPoint &ego, const PlanningPoint &global) const {
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

double ParallelInTrajectoryGenerator::CalApaTargetX() const {
  const double dst_front_edge_to_rear_axle =
      VehicleParamHelper::Instance()
          ->GetParam()
          .lon_distance_from_front_edge_to_rear_axis();
  const double dst_rear_edge_to_rear_axle =
      VehicleParamHelper::Instance()
          ->GetParam()
          .lon_distance_from_front_edge_to_rear_axis();

  const double center_y = -slot_length_ * 0.5;
  const double target_y =
      center_y -
      (dst_front_edge_to_rear_axle - dst_rear_edge_to_rear_axle) * 0.5;
  return target_y;
}

double ParallelInTrajectoryGenerator::CalApaTargetY() const {
  const double half_width = 1.89 * 0.5;
  return -slot_sign_ * std::fmin(slot_width_ * 0.5,
                                 slot_width_ - half_width - kLateralBuffer);
}

void ParallelInTrajectoryGenerator::CalApaTargetInSlot(int idx) {
  target_point_in_slot_.x = CalApaTargetX();
  target_point_in_slot_.y = CalApaTargetY();
  target_point_in_slot_.theta = 0.0;

  target_point_in_odom_ =
      FromLocal2GlobalCor(slot_origin_in_odom_, target_point_in_slot_);
}

void ParallelInTrajectoryGenerator::CalEgoPostionInSlotAndOdom(int idx) {
  const auto &pose = local_view_->localization_estimate.pose();
  cur_pos_in_odom_.x = pose.local_position().x();
  cur_pos_in_odom_.y = pose.local_position().y();
  cur_pos_in_odom_.theta = pose.euler_angles().yaw();
  cur_pos_in_slot_ =
      FromGlobal2LocalCor(slot_origin_in_odom_, cur_pos_in_odom_);

  AINFO << "cur_pos_in_odom_ x:" << cur_pos_in_odom_.x
        << ", y:" << cur_pos_in_odom_.y << ", theta:" << cur_pos_in_odom_.theta;

  AINFO << "cur_pos_in_slot_ x:" << cur_pos_in_slot_.x
        << ", y:" << cur_pos_in_slot_.y << ", theta:" << cur_pos_in_slot_.theta;
}

void ParallelInTrajectoryGenerator::GetCurPtSpeed(
    const double segment_len, const double cur_s, const double spd_sign,
    TrajectoryPoint *trajectory_point) const {
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

void ParallelInTrajectoryGenerator::GetCurPtSpeed(
    const double spd_sign, TrajectoryPoint *trajectory_point) const {
  trajectory_point->set_t(0.0);
  trajectory_point->set_v(kMaxSpd * spd_sign);
  trajectory_point->set_a(0.0);
}

void ParallelInTrajectoryGenerator::GetCurPtSpeed(
    const double segment_len, const double spd_sign,
    TrajectoryPoint *trajectory_point) const {
  trajectory_point->set_t(0.0);
  trajectory_point->set_a(0.0);
  if (segment_len <= kMaxLenOfSmallSpeed) {
    trajectory_point->set_v(kMinSpdInCircleStep * spd_sign);
    return;
  }
  if (trajectory_point->curvature() == 0.0) {
    if (spd_sign > 0.0) {
      trajectory_point->set_v(kMaxSpdInLineStep * spd_sign);
    } else {
      trajectory_point->set_v(kMinSpdInCircleStep * spd_sign);
    }
  } else {
    trajectory_point->set_v(kMinSpdInCircleStep * spd_sign);
  }
}

void ParallelInTrajectoryGenerator::CalSlotOriginInodom(const int idx) {
  slot_origin_in_odom_.x = slot_points_in_m_[0].x;
  slot_origin_in_odom_.y = slot_points_in_m_[0].y;
  slot_origin_in_odom_.theta =
      std::atan2(slot_points_in_m_[0].y - slot_points_in_m_[1].y,
                 slot_points_in_m_[0].x - slot_points_in_m_[1].x);
}

void ParallelInTrajectoryGenerator::CalSlotPointsInM(const int idx) {
  const auto &slot_points =
      local_view_->parking_fusion_info.parking_fusion_slot_lists()[idx]
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

  AINFO << "raw slot_points_in_m_ x0:" << x0 << ", y0:" << y0 << ", x1:" << x1
        << ", y1:" << y1 << ", x2:" << x2 << ", y2:" << y2 << ", x3:" << x3
        << ", y3:" << y3;

  // SquareSlot();
  slot_width_ = std::hypot(slot_points_in_m_[0].x - slot_points_in_m_[2].x,
                           slot_points_in_m_[0].y - slot_points_in_m_[2].y);
  slot_length_ = std::hypot(slot_points_in_m_[0].x - slot_points_in_m_[1].x,
                            slot_points_in_m_[0].y - slot_points_in_m_[1].y);
}

// assume slot is parallelogram
void ParallelInTrajectoryGenerator::SquareSlot() {
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
}

bool ParallelInTrajectoryGenerator::IsReplan(
    PlanningOutput *const planning_output) {
  if (IsReplanEachFrame(local_view_->function_state_machine_info)) {
    AINFO << "replan state:" << current_state_;
    return true;
  }

  planning_output->mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);
  const auto &local_position =
      local_view_->localization_estimate.pose().local_position();
  const double ego_x = local_position.x();
  const double ego_y = local_position.y();
  AINFO << "ego x:" << ego_x << ", y:" << ego_y;
  AINFO << "veh_spd:"
        << local_view_->vehicle_service_output_info.vehicle_speed();

  if (!planning_output->has_trajectory() ||
      planning_output->trajectory().trajectory_points_size() == 0) {
    is_replan_ = true;
    last_segment_name_.clear();
    return true;
  }

  double min_dis_sq = std::numeric_limits<double>::max();
  int min_dis_index = -1;
  const auto &traj = planning_output->trajectory();
  const int traj_point_size = traj.trajectory_points_size();
  const auto &traj_points = traj.trajectory_points();
  for (int i = 0; i < traj_point_size; ++i) {
    const double dis_sq = std::pow(ego_x - traj_points[i].x(), 2.0) +
                          std::pow(ego_y - traj_points[i].y(), 2.0);
    if (dis_sq < min_dis_sq) {
      min_dis_sq = dis_sq;
      min_dis_index = i;
    }
  }
  if (min_dis_index == -1) {
    AERROR << "nearest traj pt not found";
    return false;
  }
  AINFO << "min_dis_index:" << min_dis_index
        << ", min_dis:" << std::sqrt(min_dis_sq);
  const double remaining_s =
      traj_points.rbegin()->distance() - traj_points[min_dis_index].distance();
  AINFO << "remaining_s:" << remaining_s;

  if (standstill_time_ >= kMinStandstillTime &&
      pos_unchanged_cnt_ >= kMinPosUnchangedCount) {
    if (remaining_s <= kRemainingDisThreshold) {
      return true;
    }
  }

  return false;
}

void ParallelInTrajectoryGenerator::SetPlanningOutputInfo(
    PlanningOutput *const planning_output) const {
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
  AINFO << "gear:" << gear_command->gear_command_value();
}

bool ParallelInTrajectoryGenerator::IsSamePoint(const PlanningPoint &p1,
                                                const PlanningPoint &p2) const {
  return fabs(p1.x - p2.x) < kEps && abs(p1.y - p2.y) < kEps &&
         abs(p1.theta - p2.theta) < kEps;
}

void ParallelInTrajectoryGenerator::UpdateStandstillTime() {
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

void ParallelInTrajectoryGenerator::UpdatePosUnchangedCount() {
  if (std::fabs(last_pos_in_odom_.x - cur_pos_in_odom_.x) < kEps &&
      std::fabs(last_pos_in_odom_.y - cur_pos_in_odom_.y) < kEps) {
    ++pos_unchanged_cnt_;
  } else {
    pos_unchanged_cnt_ = 0;
  }
  last_pos_in_odom_ = cur_pos_in_odom_;
}

bool ParallelInTrajectoryGenerator::IsApaFinished() const {
  return current_state_ == FunctionalState::PARK_IN_ACTIVATE_CONTROL &&
         standstill_time_ >= kMinStandstillTime &&
         fabs(target_point_in_slot_.x - cur_pos_in_slot_.x) < kMaxXOffset &&
         fabs(target_point_in_slot_.y - cur_pos_in_slot_.y) < kMaxYOffset &&
         fabs(target_point_in_slot_.theta - cur_pos_in_slot_.theta) <
             kMaxThetaOffset;
}

void ParallelInTrajectoryGenerator::PrintTrajectoryPoints(
    const PlanningOutput &planning_output) const {
  const auto &traj = planning_output.trajectory();
  const int traj_point_num = traj.trajectory_points_size();
  for (int i = 0; i < traj_point_num; ++i) {
    const auto &pt = traj.trajectory_points()[i];
    AINFO << "seg traj pt [" << i << "], x:" << pt.x() << ", y:" << pt.y()
          << ", theta:" << pt.heading_yaw() << ", kappa:" << pt.curvature()
          << ", v:" << pt.v() << ", s:" << pt.distance();
  }
}

void ParallelInTrajectoryGenerator::PrintSlotInfo() const {
  if (current_state_ != FunctionalState::PARK_IN_ACTIVATE_CONTROL) {
    return;
  }

  AINFO << "slot_points_in_m_ x0:" << slot_points_in_m_[0].x
        << ", y0:" << slot_points_in_m_[0].y
        << ", x1:" << slot_points_in_m_[1].x
        << ", y1:" << slot_points_in_m_[1].y
        << ", x2:" << slot_points_in_m_[2].x
        << ", y2:" << slot_points_in_m_[2].y
        << ", x3:" << slot_points_in_m_[3].x
        << ", y3:" << slot_points_in_m_[3].y;

  AINFO << "slot_width_:" << slot_width_ << ", slot_length_:" << slot_length_;

  AINFO << "slot origin in odom x:" << slot_origin_in_odom_.x
        << ", y:" << slot_origin_in_odom_.y
        << ", theta:" << slot_origin_in_odom_.theta;

  AINFO << "target_point_in_slot_ x:" << target_point_in_slot_.x
        << ", y:" << target_point_in_slot_.y
        << ", theta:" << target_point_in_slot_.theta;

  AINFO << "target_point_in_odom_ x:" << target_point_in_odom_.x
        << ", y:" << target_point_in_odom_.y
        << ", theta:" << target_point_in_odom_.theta;
}

}  // namespace apa_planner
}  // namespace planning
