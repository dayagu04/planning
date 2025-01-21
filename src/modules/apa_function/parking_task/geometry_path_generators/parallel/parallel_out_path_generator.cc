#include "parallel_out_path_generator.h"

#include <chrono>
#include <cstddef>

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParallelOutPathGenerator::Reset() {
  output_.Reset();
  debug_info_.Reset();
  calc_params_.Reset();
}

void ParallelOutPathGenerator::Preprocess() {
  // ego pose, obs, slot coord system, parking direction, ref gear are needed!
  ILOG_INFO << "--------------------------------park out path planner "
               "----------------------------------";
  ILOG_INFO << "------------ in Preprocess ------------";

  using namespace pnc::geometry_lib;

  Reset();
  input_.ego_pose.heading = NormalizeAngle(input_.ego_pose.heading);
  PrintPose("start pose", input_.ego_pose);

  ILOG_INFO << "input_.is_replan_first = " << input_.is_replan_first;
  ILOG_INFO << "input_.is_complete_path = " << input_.is_complete_path;

  PrintGear("input_.ref_gear", input_.ref_gear);
  PrintSteer("input_.ref_arc_steer = ", input_.ref_arc_steer);

  ILOG_INFO << "tlane Pt in = " << input_.tlane.obs_pt_inside.transpose();
  ILOG_INFO << "tlane Pt out = " << input_.tlane.obs_pt_outside.transpose();
  ExpandPInObstacles();
  MoveChannelObstacles();
  CalcEgoParams();

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
  bool success = false;
  std::vector<pnc::geometry_lib::PathSegment> inversed_path_seg_vec;
  if (input_.ref_gear == pnc::geometry_lib::SEG_GEAR_INVALID) {
    success = InverseSearchLoopInSlot(inversed_path_seg_vec, input_.ego_pose);

    if (success) {
      const auto &start_seg = inversed_path_seg_vec.front();
      if (start_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE &&
          start_seg.Getlength() < apa_param.GetParam().min_path_length) {
        success = false;
        ILOG_INFO << "first line is too short!";
      }
    } else {
      ILOG_INFO << "calc InverseSearchLoopInSlot failed!";
    }

    if (!success) {
      inversed_path_seg_vec.clear();
      success =
          InversedTrialsByGivenGear(inversed_path_seg_vec, input_.ego_pose,
                                    pnc::geometry_lib::SEG_GEAR_DRIVE);
    }
  } else {
    success = InversedTrialsByGivenGear(inversed_path_seg_vec, input_.ego_pose,
                                        input_.ref_gear);
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
  GenParallelPreparingLineVec(preparing_pose_vec, true);
  ILOG_INFO << "preparing_pose_vec size = " << preparing_pose_vec.size();
  for (const auto &pt : preparing_pose_vec) {
    ILOG_INFO << "preparing y = " << pt.pos.y();
  }

  std::vector<pnc::geometry_lib::PathSegment> park_out_path_vec;
  const auto &park_out_pose = inversed_path_seg_vec.back().GetStartPose();
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0, false));
  pnc::geometry_lib::PrintPose("park_out_pose", park_out_pose);

  calc_params_.valid_target_pt_vec.clear();
  calc_params_.valid_target_pt_vec.emplace_back(park_out_pose);

  for (const auto &prepare_pose : preparing_pose_vec) {

    if (PlanFromTargetToLine(park_out_path_vec, prepare_pose, true)) {
      ReversePathSegVec(park_out_path_vec);
      ILOG_INFO << "park_out_path_vec ----------------------";
      pnc::geometry_lib::PrintSegmentsVecInfo(park_out_path_vec);
      ILOG_INFO << "park_out_path_vec end ----------------------";
      success = true;
      ILOG_INFO << "plan to preparing line success!";
      break;
    }
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

}  // namespace apa_planner
}  // namespace planning