#include "parallel_out_path_generator.h"

#include <chrono>
#include <cstddef>

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

static const double kColBufferInSlot = 0.25;       // in slot
static const double kColSmallBufferInSlot = 0.16;  // in slot
static const double kLonBufferTrippleStep = 0.2;   // tripple step
static const double MinLengthPath = 0.16;

void ParallelOutPathGenerator::Reset() {
  output_.Reset();
  debug_info_.Reset();
  calc_params_.Reset();
}

void ParallelOutPathGenerator::Preprocess() {
  // ego pose, obs, slot coord system, parking direction, ref gear are needed!
  ILOG_INFO << "------------ in Preprocess ------------";

  using namespace pnc::geometry_lib;

  Reset();

  input_.ego_info_under_slot.cur_pose.heading =
      NormalizeAngle(input_.ego_info_under_slot.cur_pose.heading);
  PrintPose("start pose", input_.ego_info_under_slot.cur_pose);

  ILOG_INFO << "input_.is_replan_first = " << input_.is_replan_first;
  ILOG_INFO << "input_.is_complete_path = " << input_.is_complete_path;

  PrintGear("input_.ref_gear", input_.ref_gear);
  PrintSteer("input_.ref_arc_steer = ", input_.ref_arc_steer);

  ILOG_INFO << "tlane Pt in = " << input_.tlane.obs_pt_inside.x() << " "
            << input_.tlane.obs_pt_inside.y();
  ILOG_INFO << "tlane Pt out = " << input_.tlane.obs_pt_outside.x() << " "
            << input_.tlane.obs_pt_outside.y();
  ExpandObstacles();
  MoveChannelObstacles();
  CalcEgoParams();

  calc_params_.lat_outside_slot_buffer = 0.04;
  if (input_.tlane.pt_inside.x() - input_.tlane.pt_outside.x() >= 6.2) {
    calc_params_.lon_buffer_rev_trials =
        apa_param.GetParam().lat_lon_path_buffer.parallel_col_buffer_in_slot;
  } else {
    calc_params_.lon_buffer_rev_trials =
        apa_param.GetParam()
            .lat_lon_path_buffer.parallel_col_small_buffer_in_slot;
  }

  calc_params_.is_left_side =
      (input_.ref_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT);

  calc_params_.slot_side_sgn = (calc_params_.is_left_side ? -1.0 : 1.0);

  ILOG_INFO << "calc_params_.slot_side_sgn = " << calc_params_.slot_side_sgn;

  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.06, false));
}

const bool ParallelOutPathGenerator::Update(
    const std::shared_ptr<CollisionDetector> &collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  const auto start_time = std::chrono::high_resolution_clock::now();

  const bool success = Update();
  DeletePInVirtualObstacles();
  RecorverChannelObstacles();
  pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);

  // const auto time1 = std::chrono::high_resolution_clock::now();
  const auto duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - start_time)
          .count();
  JSON_DEBUG_VALUE("path_plan_time_ms", duration_ms);
  ILOG_INFO << "parallel cost time(ms) = " << duration_ms;
  return success;
}

const bool ParallelOutPathGenerator::Update() {
  Preprocess();
  AddPInVirtualObstacles();

  bool success = false;
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));
  std::vector<pnc::geometry_lib::PathSegment> inversed_path_seg_vec;
  if (input_.ref_gear == pnc::geometry_lib::SEG_GEAR_INVALID) {
    success = AdvancedInversedTrialsInSlot(inversed_path_seg_vec,
                                           input_.ego_info_under_slot.cur_pose);
    ILOG_INFO << "AdvancedInversedTrialsInSlot  --------------------------";
    pnc::geometry_lib::PrintSegmentsVecInfo(inversed_path_seg_vec);

    if (!success) {
      inversed_path_seg_vec.clear();
      success = InversedTrialsByGivenGear(inversed_path_seg_vec,
                                          input_.ego_info_under_slot.cur_pose,
                                          pnc::geometry_lib::SEG_GEAR_DRIVE);
    }
  } else {
    success = InversedTrialsByGivenGear(inversed_path_seg_vec,
                                        input_.ego_info_under_slot.cur_pose,
                                        input_.ref_gear);
    if (!success ||
        std::fabs(inversed_path_seg_vec.back().GetStartPos().y()) >
            (input_.tlane.slot_width * 0.5) ||
        !CheckShortToFirstPath(inversed_path_seg_vec, MinLengthPath)) {
      inversed_path_seg_vec.clear();
      success = AdvancedInversedTrialsInSlot(
          inversed_path_seg_vec, input_.ego_info_under_slot.cur_pose);
    }
  }

  if (!success || inversed_path_seg_vec.size() == 0) {
    ILOG_INFO << "inversed search in slot failed!";
    return false;
  }
  ILOG_INFO << "inversed search in slot success! --------------------------";
  pnc::geometry_lib::PrintSegmentsVecInfo(inversed_path_seg_vec);
  ILOG_INFO << "inversed search in slot success! end-----------------------";

  success = false;
  std::vector<pnc::geometry_lib::PathPoint> preparing_pose_vec;
  GenParallelPreparingLineVecOut(preparing_pose_vec);
  ILOG_INFO << "preparing_pose_vec size = " << preparing_pose_vec.size();
  for (const auto &pt : preparing_pose_vec) {
    ILOG_INFO << "preparing y = " << pt.pos.y();
  }

  const auto &park_out_pose = inversed_path_seg_vec.back().GetStartPose();
  pnc::geometry_lib::PrintPose("park_out_pose", park_out_pose);

  calc_params_.valid_target_pt_vec.clear();
  calc_params_.valid_target_pt_vec.emplace_back(park_out_pose);

  std::vector<pnc::geometry_lib::PathSegment> park_out_path_vec;
  for (const auto &prepare_pose : preparing_pose_vec) {
    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));
    if (!PlanFromTargetToLine(park_out_path_vec, prepare_pose, true)) {
      continue;
    }

    ReversePathSegVec(park_out_path_vec);
    ILOG_INFO << "park_out_path_vec ----------------------";
    pnc::geometry_lib::PrintSegmentsVecInfo(park_out_path_vec);
    success = true;
    ILOG_INFO << "plan to preparing line success!";
    break;
  }

  if (success) {
    std::vector<pnc::geometry_lib::PathSegment> path_res;
    for (size_t i = 0; i < inversed_path_seg_vec.size() - 1; i++) {
      path_res.emplace_back(inversed_path_seg_vec[i]);
    }
    for (size_t i = 0; i < park_out_path_vec.size(); i++) {
      path_res.emplace_back(park_out_path_vec[i]);
    }
    AddPathSegVecToOutput(path_res);
  } else {
    ILOG_INFO << "PlanToPreparingLine failed in total loop!";
  }
  return success;
}

const bool ParallelOutPathGenerator::GenParallelPreparingLineVecOut(
    std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec) {
  const double half_slot_width = 0.5 * input_.tlane.slot_width;
  const double slot_side_sgn = input_.tlane.slot_side_sgn;

  const double pin_y = input_.tlane.obs_pt_inside.y();

  const bool front_vacant =
      input_.tlane.obs_pt_inside.x() > input_.tlane.slot_length + 2.8;

  const double tlane_outer_y =
      std::fabs(pin_y) > half_slot_width - 1e-5
          ? pin_y
          : (half_slot_width + std::fabs(pin_y)) * 0.5 * slot_side_sgn;

  const double rac_tlane_bound =
      tlane_outer_y +
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.3);

  double tlane_outer_y_ref_line =
      std::max(input_.tlane.obs_pt_inside.y() * slot_side_sgn,
               half_slot_width) *
      slot_side_sgn;
  double rac_tlane_bound_ref_line =
      tlane_outer_y_ref_line +
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.6);

  ILOG_INFO << "obs_pt_inside y = " << input_.tlane.obs_pt_inside.y()
            << " half_slot_width = " << half_slot_width
            << " input_.tlane.slot_side_sgn = " << input_.tlane.slot_side_sgn
            << "calc_params_.slot_side_sgn" << calc_params_.slot_side_sgn;

  const double rac_tlane_bound_near =
      std::fabs(rac_tlane_bound) > std::fabs(rac_tlane_bound_ref_line)
          ? rac_tlane_bound_ref_line
          : rac_tlane_bound;
  const double rac_tlane_bound_far =
      std::fabs(rac_tlane_bound) > std::fabs(rac_tlane_bound_ref_line)
          ? rac_tlane_bound
          : rac_tlane_bound_ref_line;

  double rac_channel_bound =
      input_.tlane.channel_y -
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.3);

  if (slot_side_sgn * (rac_channel_bound - rac_tlane_bound_near) < 0.0) {
    ILOG_INFO << "rac_channel_bound - rac_tlane_bound_near "
              << rac_channel_bound - rac_tlane_bound_near;
    ILOG_INFO << "SGN DIFFERENT!";
    return false;
  }

  if (rac_channel_bound * slot_side_sgn > 10) {
    rac_channel_bound = 10.0 * slot_side_sgn;
  }

  const double y_bound = std::fabs(rac_channel_bound - rac_tlane_bound_near);

  const double channel_width =
      std::fabs(input_.tlane.channel_y) - half_slot_width;

  double dy = channel_width > 4.0 ? 0.1 : 0.05;

  int nums = static_cast<int>(y_bound / dy);
  nums = pnc::mathlib::Clamp(nums, 5, 16);
  dy = y_bound / nums;

  pnc::geometry_lib::PathPoint prepare_pose(input_.tlane.pt_inside, 0.0);
  prepare_pose.pos.y() = rac_tlane_bound_near;

  const auto y_vec =
      pnc::geometry_lib::Linspace(rac_tlane_bound_near, rac_channel_bound, dy);

  if (front_vacant) {
    for (int i = 0; i < y_vec.size(); ++i) {
      prepare_pose.pos.y() = y_vec[i];
      preparing_pose_vec.emplace_back(prepare_pose);
    }
  } else {
    int idx = 0;
    for (int i = 0; i < y_vec.size(); ++i) {
      if (std::fabs(y_vec[i]) < std::fabs(rac_tlane_bound_far)) {
        idx = i;
        continue;
      }
      prepare_pose.pos.y() = y_vec[i];
      preparing_pose_vec.emplace_back(prepare_pose);
    }
    for (int i = 0; i < idx; ++i) {
      prepare_pose.pos.y() = y_vec[i];
      preparing_pose_vec.emplace_back(prepare_pose);
    }
  }
  ILOG_INFO << "rac_tlane_bound_near = " << rac_tlane_bound_near
            << " rac_tlane_bound_far = " << rac_tlane_bound_far;

  return true;
}

}  // namespace apa_planner
}  // namespace planning