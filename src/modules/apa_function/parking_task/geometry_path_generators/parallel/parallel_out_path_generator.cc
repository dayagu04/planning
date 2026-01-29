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
static const double kMinChannelWidth = 3.3;
static const double kRearDetaXMagWhenFrontOccupiedRearVacant = 1.0;
static const double kEps = 1e-5;

void ParallelOutPathGenerator::Reset() {
  output_.Reset();
  debug_info_.Reset();
  calc_params_.Reset();
  parkout_path_by_direction_.clear();
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

  calc_params_.scene_type = ParallelParkSceneType::PARALLEL_PARK_OUT_SCENE;
  if (std::fabs(input_.tlane.channel_y) - (input_.tlane.slot_width * 0.5) <
      kMinChannelWidth) {
    calc_params_.scene_type =
        ParallelParkSceneType::PARALLEL_PARK_OUT_NARROW_CHANNEL_SCENE;
    const double cur_pose_x = input_.ego_info_under_slot.cur_pose.GetX() -
                              apa_param.GetParam().rear_overhanging;
    if (std::fabs(input_.tlane.obs_pt_outside.x() - cur_pose_x) >
        kRearDetaXMagWhenFrontOccupiedRearVacant) {
      calc_params_.scene_type = ParallelParkSceneType::
          PARALLEL_PARK_OUT_NARROW_CHANNEL_REAR_VACANT_SCENE;
    }
  }

  ILOG_INFO << "calc_params_.slot_side_sgn = " << calc_params_.slot_side_sgn
            << " scene_type = " << int(calc_params_.scene_type);

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
  const bool is_in_slot = CheckEgoInSlot();

  if (input_.is_before_running_stage) {
    arc_slot_init_out_heading_ =
        input_.ego_info_under_slot.neigbor_front_heading;
    if (arc_slot_init_out_heading_ > M_PI ||
        arc_slot_init_out_heading_ < -M_PI) {
      arc_slot_init_out_heading_ = 0.0;
    }
  }
  if (!is_in_slot) {
    collision_detector_ptr_->SetParam(
        CollisionDetector::Paramters(0.1, true, true));
    auto last_target_pos = input_.last_target_pose_.pos;
    auto last_target_heading = input_.last_target_pose_.heading;
    auto pose_univ = Eigen::Vector2d(std::cos(last_target_heading),
                                     std::sin(last_target_heading));

    auto upper_normal = Eigen::Vector2d(-pose_univ.y(), pose_univ.x());
    auto lower_normal = Eigen::Vector2d(pose_univ.y(), -pose_univ.x());
    std::vector<pnc::geometry_lib::LineSegment> prepareline_candi;
    prepareline_candi.reserve(25);
    for( int i = 0; i < 10; i++){
      auto upper_point = last_target_pos + upper_normal * 0.1 * (i + 1);
      prepareline_candi.emplace_back(pnc::geometry_lib::BuildLineSegByPose(
        upper_point, last_target_heading));
      auto lower_point = last_target_pos + lower_normal * 0.1 * (i + 1);
      prepareline_candi.emplace_back(pnc::geometry_lib::BuildLineSegByPose(
        lower_point, last_target_heading));

    }




    std::vector<pnc::geometry_lib::PathSegment> out_path_vec;
    for (auto line : prepareline_candi) {
      success = PlanToPreparingLine(out_path_vec,
                                    input_.ego_info_under_slot.cur_pose, line);
      if (success) {
        break;
      }
    }
    if (!success || out_path_vec.size() == 0) {
      ILOG_INFO << "three  prepareline out search in slot failed!";
      return false;
    }
    // remove short path <0.16
    if (out_path_vec.size() > 2) {
      if (out_path_vec[0].GetLength() < 0.16 &&
          out_path_vec[0].seg_gear != out_path_vec[1].seg_gear) {
        out_path_vec.erase(out_path_vec.begin());
      }
      if (out_path_vec.back().GetLength() < 0.16 &&
          out_path_vec[out_path_vec.size() - 2].seg_gear !=
              out_path_vec.back().seg_gear) {
        out_path_vec.erase(out_path_vec.end());
      }
    }
    pnc::geometry_lib::PrintSegmentsVecInfo(out_path_vec);
    ILOG_INFO << "three  prepareline out search in slot success! --------------------------";
    AddPathSegVecToOutput(out_path_vec);


  } else {
    std::vector<pnc::geometry_lib::PathSegment> inversed_path_seg_vec;
    if (input_.ref_gear == pnc::geometry_lib::SEG_GEAR_INVALID) {
      success = AdvancedInversedTrialsInSlot(
          inversed_path_seg_vec, input_.ego_info_under_slot.cur_pose);
      ILOG_INFO << "AdvancedInversedTrialsInSlot  --------------------------";
      pnc::geometry_lib::PrintSegmentsVecInfo(inversed_path_seg_vec);

      if (!success) {
        inversed_path_seg_vec.clear();
        success = InversedTrialsByGivenGear(inversed_path_seg_vec,
                                            input_.ego_info_under_slot.cur_pose,
                                            pnc::geometry_lib::SEG_GEAR_DRIVE);
        if (inversed_path_seg_vec.size() == 1) {
          if (std::abs(inversed_path_seg_vec[0].GetArcSeg().headingA -
                       arc_slot_init_out_heading_) >
              pnc::mathlib::Deg2Rad(50.0)) {
            success = false;
          }

        } else if (inversed_path_seg_vec.size() > 1) {
          if (std::abs(inversed_path_seg_vec.back().GetArcSeg().headingA -
                       arc_slot_init_out_heading_) >
              pnc::mathlib::Deg2Rad(50.0)) {
            success = false;
            ILOG_INFO
                << "InversedTrialsByGivenGear heading over change heading!!!";
          }
        }
      }
    } else {
      success = InversedTrialsByGivenGear(inversed_path_seg_vec,
                                          input_.ego_info_under_slot.cur_pose,
                                          input_.ref_gear);
      if (inversed_path_seg_vec.size() == 1) {
          if (std::abs(inversed_path_seg_vec[0].GetArcSeg().headingA -
                       arc_slot_init_out_heading_) >
              pnc::mathlib::Deg2Rad(50.0)) {
            success = false;
          }

        } else if (inversed_path_seg_vec.size() > 1) {
          if (std::abs(inversed_path_seg_vec.back().GetArcSeg().headingA -
                       arc_slot_init_out_heading_) >
              pnc::mathlib::Deg2Rad(50.0)) {
            success = false;
            ILOG_INFO
                << "InversedTrialsByGivenGear heading over change heading!!!";
          }
        }
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

    ApaParkOutDirection out_dir = input_.tlane.slot_side_sgn > 0.0
                                      ? ApaParkOutDirection::LEFT_FRONT
                                      : ApaParkOutDirection::RIGHT_FRONT;
    parkout_path_by_direction_[out_dir] = inversed_path_seg_vec;

    ILOG_INFO << "out_dir: " << int(out_dir);

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

    std::vector<std::vector<pnc::geometry_lib::PathSegment>>
        all_park_out_path_vec;
    for (const auto &prepare_pose : preparing_pose_vec) {
      collision_detector_ptr_->SetParam(
          CollisionDetector::Paramters(0.1, true));
      std::vector<pnc::geometry_lib::PathSegment> park_out_path_vec;
      if (!PlanFromTargetToLine(park_out_path_vec, prepare_pose)) {
        continue;
      }
      all_park_out_path_vec.emplace_back(park_out_path_vec);
      if (calc_params_.scene_type !=
          ParallelParkSceneType::
              PARALLEL_PARK_OUT_NARROW_CHANNEL_REAR_VACANT_SCENE) {
        ILOG_INFO
            << "not narrow channel rear vacant scene! only use first path";
        break;
      }
    }
    const int best_path_idx = SelectParkOutPathVec(all_park_out_path_vec);
    if (best_path_idx < 0 || best_path_idx > all_park_out_path_vec.size() - 1) {
      ILOG_INFO << "plan to preparing line failed!";
      return false;
    }
    ILOG_INFO << "park_out_path_vec best_path_idx: " << best_path_idx;
    std::vector<pnc::geometry_lib::PathSegment> &park_out_path_vec =
        all_park_out_path_vec[best_path_idx];
    ReversePathSegVec(park_out_path_vec);
    pnc::geometry_lib::PrintSegmentsVecInfo(park_out_path_vec);
    success = true;
    ILOG_INFO << "plan to preparing line success!";

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
  }
  return success;
}

const int ParallelOutPathGenerator::SelectParkOutPathVec(
    const std::vector<std::vector<pnc::geometry_lib::PathSegment>>&
        park_out_path_vec) {
  if (park_out_path_vec.empty()) {
    ILOG_INFO << "park_out_path_vec is empty!";
    return -1;
  }
  if (park_out_path_vec.size() == 1) {
    return 0;
  }

  int best_path_index = 0;
  double max_dis_ObsPin = 0.0;
  for (int i = 0; i < park_out_path_vec.size(); i++) {
    if (park_out_path_vec[i].size() != 2) {
      continue;
    }
    double cur_dis_ObsPin = 0;
    if (park_out_path_vec[i][0].seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      const double dis_ObsPin = park_out_path_vec[i][0].GetArcSeg().dis_ObsPin;
      if (dis_ObsPin < 100.0 - kEps) {
        cur_dis_ObsPin = park_out_path_vec[i][0].GetArcSeg().dis_ObsPin;
      }
    }

    ILOG_INFO << "park_out_path_vec[" << i
              << "] dis_ObsPin = " << cur_dis_ObsPin;
    if (cur_dis_ObsPin > max_dis_ObsPin) {
      best_path_index = i;
      max_dis_ObsPin = cur_dis_ObsPin;
    }
  }
  return best_path_index;
}

const bool ParallelOutPathGenerator::GenParallelPreparingLineVecOut(
    std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec) {
  const double half_slot_width = 0.5 * input_.tlane.slot_width;
  const double slot_side_sgn = input_.tlane.slot_side_sgn;

  const double obs_pt_inside_y = input_.tlane.obs_pt_inside.y();

  bool narrow_front_vacant = false;
  if (obs_pt_inside_y > half_slot_width * 0.5 - kEps &&
      obs_pt_inside_y < half_slot_width * 0.5 + kEps &&
      calc_params_.scene_type ==
          ParallelParkSceneType::PARALLEL_PARK_OUT_NARROW_CHANNEL_SCENE) {
    narrow_front_vacant = true;
  }

  const double pin_y =
      narrow_front_vacant ? obs_pt_inside_y - half_slot_width : obs_pt_inside_y;

  double tlane_outer_y =
      std::fabs(pin_y) > half_slot_width - 1e-5
          ? pin_y
          : (half_slot_width + std::fabs(pin_y)) * 0.5 * slot_side_sgn;

  if (narrow_front_vacant) {
    tlane_outer_y = half_slot_width * 0.5 * slot_side_sgn;
  }

  const double rac_tlane_bound =
      tlane_outer_y +
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.3);

  double tlane_outer_y_ref_line =
      std::max(input_.tlane.obs_pt_inside.y() * slot_side_sgn,
               half_slot_width) *
      slot_side_sgn;
  double prepare_line_starty_from_slot =
      input_.ego_info_under_slot.slot.GetLimiter().valid
          ? 0.5 * apa_param.GetParam().car_width +0.5
          : 0.6;
  double rac_tlane_bound_ref_line =
      tlane_outer_y_ref_line +
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width +
                       prepare_line_starty_from_slot);

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
  ILOG_INFO << "input_.tlane.channel_y" << input_.tlane.channel_y;
  const double channel_width =
      std::fabs(input_.tlane.channel_y) - half_slot_width;

  double dy = channel_width > 4.0 ? 0.1 : 0.05;
  ILOG_INFO << "dy = " << dy << " y_bound = " << y_bound
            << " channel_width = " << channel_width;

  int nums = static_cast<int>(y_bound / dy);
  nums = pnc::mathlib::Clamp(nums, 5, 16);
  dy = y_bound / nums;

  bool is_inner_arc_slot = false;
  if (slot_side_sgn < 0.0 && arc_slot_init_out_heading_ < -pnc::mathlib::Deg2Rad(10.0) &&
      arc_slot_init_out_heading_ > -pnc::mathlib::Deg2Rad(45.0)) {
    is_inner_arc_slot = true;
  }
  if (slot_side_sgn > 0.0 && arc_slot_init_out_heading_ > pnc::mathlib::Deg2Rad(10.0) &&
      arc_slot_init_out_heading_ < pnc::mathlib::Deg2Rad(45.0)) {
    is_inner_arc_slot = true;
  }

  Eigen::Vector2d prepare_pose_start = input_.tlane.pt_inside;
  if ((is_inner_arc_slot ) && input_.tlane.pt_inside.x() > 6.5) {
    prepare_pose_start.x() = 5.5;
    ILOG_INFO << "prepare_pose_start.x = " << prepare_pose_start.x();
  }

  pnc::geometry_lib::PathPoint prepare_pose(prepare_pose_start, arc_slot_init_out_heading_);
  ILOG_INFO << "park out prepare line front_heading = " << arc_slot_init_out_heading_;
  prepare_pose.pos.y() = rac_tlane_bound_near;

  const auto y_vec =
      pnc::geometry_lib::Linspace(rac_tlane_bound_near, rac_channel_bound, dy);

  const double start_y =
      slot_side_sgn *
      (0.5 * (apa_param.GetParam().car_width) + half_slot_width + 0.4);
  ILOG_INFO << "rac_tlane_bound_near = " << rac_tlane_bound_near
            << " rac_tlane_bound_far = " << rac_tlane_bound_far
            << " start_y = " << start_y;
  int idx = 0;
  double heading_threshold = 0.0;
  if (std::abs(arc_slot_init_out_heading_) > pnc::mathlib::Deg2Rad(15.0)) {
    if (arc_slot_init_out_heading_ > 0) {
      heading_threshold = 15.0;
    } else if (arc_slot_init_out_heading_ < 0) {
      heading_threshold = -15.0;
    }
  }

    const auto headings =
        pnc::geometry_lib::Linspace(0, pnc::mathlib::Deg2Rad(heading_threshold),
                                    pnc::mathlib::Deg2Rad(3.0));
    if (headings.size() > 0) {
    for (int j = 0; j < headings.size(); ++j) {
      ILOG_INFO << "headings = " << headings[j];
      for (int i = 0; i < y_vec.size(); ++i) {
        if (std::fabs(y_vec[i]) < std::fabs(start_y)) {
            idx = i;
            continue;
        }
        prepare_pose.pos.y() = y_vec[i];
        prepare_pose.heading = prepare_pose.heading + headings[j];
        preparing_pose_vec.emplace_back(prepare_pose);
      }
      for (int i = 0; i <= idx; ++i) {
        prepare_pose.pos.y() = y_vec[i];
        prepare_pose.heading = prepare_pose.heading + headings[j];
        preparing_pose_vec.emplace_back(prepare_pose);
      }
    }
    }else{
      for (int i = 0; i < y_vec.size(); ++i) {
        if (std::fabs(y_vec[i]) < std::fabs(start_y)) {
            idx = i;
            continue;
        }
        prepare_pose.pos.y() = y_vec[i];
        preparing_pose_vec.emplace_back(prepare_pose);
      }
      for (int i = 0; i <= idx; ++i) {
        prepare_pose.pos.y() = y_vec[i];
        preparing_pose_vec.emplace_back(prepare_pose);
      }
    }

    return true;
  }

}  // namespace apa_planner
}  // namespace planning