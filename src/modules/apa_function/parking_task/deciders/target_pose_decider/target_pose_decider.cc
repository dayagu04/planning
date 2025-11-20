#include "target_pose_decider.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_param_config.h"
#include "collision_detection/gjk_collision_detector.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "parking_scenario_manager.h"

namespace planning {
namespace apa_planner {
const TargetPoseDeciderResult TargetPoseDecider::CalcTargetPose(
    const ApaSlot& slot, const TargetPoseDeciderRequest& request) {
  result_.Reset();
  slot_ = slot;
  lat_body_buffer_vec_ = request.lat_body_buffer_vec;
  lat_mirror_buffer_vec_ = request.lat_mirror_buffer_vec;
  lon_buffer_ = request.lon_buffer;
  consider_obs_ = request.consider_obs;
  base_on_slot_ = request.base_on_slot;
  slot_lat_pos_preference_ = request.slot_lat_pos_preference;
  is_searching_stage_ = request.is_searching_stage;
  ego_pose_local_ = request.ego_pose_local;

  if (request.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    return CalcTargetPoseForPerpendicularTailIn();
  } else if (request.scenario_type ==
             ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    return CalcTargetPoseForPerpendicularHeadIn();
  } else {
    return result_;
  }

  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForPerpendicularTailIn() {
  ILOG_INFO << "CalcTargetPoseForPerpendicularTailIn with fold mirror = "
            << col_det_interface_ptr_->GetFoldMirrorFlag();
  const ApaParameters& param = apa_param.GetParam();
  const geometry_lib::GlobalToLocalTf g2l_tf = slot_.g2l_tf_;
  const geometry_lib::LocalToGlobalTf l2g_tf = slot_.l2g_tf_;
  slot_.TransformCoordFromGlobalToLocal(g2l_tf);

  double virtual_tar_x = 0.0;
  if (slot_.limiter_.valid) {
    Eigen::Vector2d pt1 = g2l_tf.GetPos(slot_.limiter_.start_pt);
    Eigen::Vector2d pt2 = g2l_tf.GetPos(slot_.limiter_.end_pt);

    virtual_tar_x = 0.5 * (pt1 + pt2).x() + param.limiter_move_dist;

    // avoid limit error
    if (virtual_tar_x >
        slot_.processed_corner_coord_local_.pt_01_mid.x() - 0.68) {
      slot_.limiter_.valid = false;
    } else if (virtual_tar_x <
               slot_.processed_corner_coord_local_.pt_23_mid.x() + 0.18) {
      slot_.limiter_.valid = false;
    }
  }
  if (!slot_.limiter_.valid) {
    virtual_tar_x = slot_.processed_corner_coord_local_.pt_23_mid.x() +
                    param.terminal_target_x;

    if (slot_.slot_type_ == SlotType::SLANT) {
      virtual_tar_x = slot_.origin_corner_coord_local_.pt_23_mid.x() +
                      param.terminal_target_x;
    }

    // park in the middle of the slot
    double mid_x = (slot_.processed_corner_coord_local_.pt_01_mid.x() +
                    slot_.processed_corner_coord_local_.pt_23_mid.x()) *
                   0.5;

    if (slot_.slot_type_ == SlotType::SLANT) {
      mid_x = (slot_.origin_corner_coord_local_.pt_01_mid.x() +
               slot_.origin_corner_coord_local_.pt_23_mid.x()) *
              0.5;
    }

    const double mid_ego_x =
        mid_x - (0.5 * param.car_length - param.rear_overhanging);

    virtual_tar_x = std::max(virtual_tar_x, mid_ego_x);
  }

  // If the limiter is too far back or the slot is too long,  can combine the
  // front corner information of slot
  // virtual_tar_x = std::max(virtual_tar_x,
  //                          slot_.processed_corner_coord_local_.pt_01_mid.x()
  //                          -
  //                              param.limiter_length - param.wheel_base -
  //                              param.front_overhanging);

  double redundant_y = (slot_.slot_width_ - param.car_width) * 0.5;
  double offset_y = 0.0;
  if (redundant_y >
      param.lat_lon_target_pose_buffer.preference_lat_offset + 1e-2f) {
    if (slot_lat_pos_preference_ == ApaSlotLatPosPreference::LEFT) {
      offset_y =
          redundant_y - param.lat_lon_target_pose_buffer.preference_lat_offset;
    } else if (slot_lat_pos_preference_ == ApaSlotLatPosPreference::RIGHT) {
      offset_y =
          param.lat_lon_target_pose_buffer.preference_lat_offset - redundant_y;
    }
  }

  geometry_lib::PathPoint tar_pose_local;
  geometry_lib::PathPoint tar_pose_global;
  tar_pose_local.pos << virtual_tar_x, offset_y;
  tar_pose_local.heading = param.terminal_target_heading * kDeg2Rad;

  tar_pose_global =
      geometry_lib::TransformPoseFromLocalToGlobal(tar_pose_local, l2g_tf);

  ILOG_INFO << "target pose offset_y = " << offset_y;

  if (!consider_obs_) {
    result_.target_pose_type = TargetPoseType::NORMAL;
    result_.target_pose_local = tar_pose_local;
    result_.target_pose_global = tar_pose_global;
    return result_;
  }

  Eigen::Vector2d lon_move_dir =
      slot_.origin_corner_coord_global_.pt_23mid_01mid_unit_vec;

  Eigen::Vector2d lat_move_dir =
      slot_.origin_corner_coord_global_.pt_01_unit_vec;

  if (base_on_slot_) {
    lon_move_dir = slot_.origin_corner_coord_local_.pt_23mid_01mid_unit_vec;

    lat_move_dir = slot_.origin_corner_coord_local_.pt_01_unit_vec;
  }

  ApaObsMovementType consider_obs_movement_type = ApaObsMovementType::ALL;
  if (!is_searching_stage_) {
    consider_obs_movement_type = ApaObsMovementType::STATIC;
  }

  const GJKColDetRequest gjk_col_det_request(
      base_on_slot_, param.uss_config.use_uss_pt_cloud,
      CarBodyType::EXPAND_MIRROR_TO_FRONT, consider_obs_movement_type,
      param.use_obs_height_method);

  const std::shared_ptr<GJKCollisionDetector>& gjl_det_ptr =
      col_det_interface_ptr_->GetGJKColDetPtr();

  if (std::fabs(offset_y) > 1e-3) {
    std::vector<geometry_lib::PathPoint> tmp_pose_vec;
    geometry_lib::PathPoint tmp_pose;
    std::vector<double> lon_path_vec{
        -param.lat_lon_target_pose_buffer.preference_lon_buffer, 1.0, 2.0};
    for (const double dist : lon_path_vec) {
      if (base_on_slot_) {
        tmp_pose = tar_pose_local;
      } else {
        tmp_pose = tar_pose_global;
      }
      tmp_pose.pos = tmp_pose.pos + dist * lon_move_dir;
      tmp_pose_vec.emplace_back(tmp_pose);
    }

    if (gjl_det_ptr
            ->Update(tmp_pose_vec,
                     param.lat_lon_target_pose_buffer.max_lat_body_buffer, 0.0,
                     gjk_col_det_request, true,
                     param.lat_lon_target_pose_buffer.max_lat_mirror_buffer)
            .col_flag) {
      offset_y = 0.0;
      tar_pose_local.pos << virtual_tar_x, offset_y;
      tar_pose_local.heading = param.terminal_target_heading * kDeg2Rad;
      tar_pose_global =
          geometry_lib::TransformPoseFromLocalToGlobal(tar_pose_local, l2g_tf);

      ILOG_INFO << "target pose offset_y is invalid, recover to 0.0 = "
                << offset_y;
    }
  }

  // calc max_lat_move_dist and max_lon_move_dist
  double max_lat_move_dist{0.};
  max_lat_move_dist = 0.5 * (slot_.slot_width_ - param.car_width);
  // The distance between the body and the slot line: positive value indicates
  // the line cannot be crossed, negative value means crossing is allowed
  double car2line_dist_threshold = param.car2line_dist_threshold;
  if (!is_searching_stage_) {
    car2line_dist_threshold -= 0.03;
  }
  max_lat_move_dist -= param.car2line_dist_threshold;
  max_lat_move_dist = std::max(max_lat_move_dist, 0.0001);
  double max_lon_move_dist{0.};
  double dx = slot_.origin_corner_coord_local_.pt_01_mid.x() -
              (virtual_tar_x + param.wheel_base + param.front_overhanging);
  double front_exceed_line_dx = apa_param.GetParam().max_front_exceed_line_dx;
  if (slot_.slot_type_ == SlotType::SLANT) {
    front_exceed_line_dx /= std::max(slot_.sin_angle_, 0.1);
  }

  if (!is_searching_stage_ && base_on_slot_) {
    if ((slot_.IsPointInCustomSlot(
             ego_pose_local_.pos, param.believe_obs_ego_area,
             param.believe_obs_ego_area, 0.2, 0.2, true) &&
         std::fabs(ego_pose_local_.heading) * kRad2Deg < 60.0)) {
      front_exceed_line_dx += 0.168;
    } else {
      front_exceed_line_dx += 0.68;
    }
  }

  if (slot_.slot_source_type_ == SlotSourceType::SELF_DEFINE) {
    front_exceed_line_dx = 0.0;
  }

  if (param.smart_fold_mirror_params.has_smart_fold_mirror &&
      !col_det_interface_ptr_->GetFoldMirrorFlag()) {
    front_exceed_line_dx = std::min(front_exceed_line_dx, 0.6);
  }

  max_lon_move_dist = front_exceed_line_dx + dx - 0.05;
  max_lon_move_dist = std::max(max_lon_move_dist, 0.0001);

  // calc lat_move_step and lon_move_step
  double lat_move_step{0.02};
  double lon_move_step{0.05};
  if (is_searching_stage_) {
    lat_move_step = 0.05;
    lon_move_step = 0.1;
  }

  ILOG_INFO << "max_lon_move_dist = " << max_lon_move_dist
            << "  max_lat_move_dist = " << max_lat_move_dist
            << "  lat_move_step = " << lat_move_step
            << "  lon_move_step = " << lon_move_step;

  Polygon2D polygon;
  polygon.FillTangentCircleParams(slot_.GetCustomSlotPolygon(
      2.68, -slot_.slot_length_ * 0.3, -slot_.slot_width_ * 0.25,
      -slot_.slot_width_ * 0.25, base_on_slot_));
  if (col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(
          polygon, gjk_col_det_request)) {
    ILOG_INFO << "slot min parking area is occupied";
    result_.target_pose_type = TargetPoseType::FAIL;
    return result_;
  }

  std::vector<double> small_lon_dist_vec;
  std::vector<double> big_lon_dist_vec;
  std::vector<std::vector<double>> all_lon_dist_vec;
  for (double lon_move_dist = 0.0;
       lon_move_dist < max_lon_move_dist + lon_move_step * 0.5;
       lon_move_dist += lon_move_step) {
    if (lon_move_dist < max_lon_move_dist * 0.8) {
      small_lon_dist_vec.emplace_back(lon_move_dist);
    } else {
      big_lon_dist_vec.emplace_back(lon_move_dist);
    }
  }

  if (is_searching_stage_) {
    small_lon_dist_vec.insert(small_lon_dist_vec.end(),
                              big_lon_dist_vec.begin(), big_lon_dist_vec.end());
    all_lon_dist_vec.emplace_back(small_lon_dist_vec);
  } else {
    all_lon_dist_vec.emplace_back(small_lon_dist_vec);
    all_lon_dist_vec.emplace_back(big_lon_dist_vec);
  }

  std::vector<double> lat_dist_vec{0.0};
  for (double lat_move_dist = lat_move_step;
       lat_move_dist < max_lat_move_dist + lat_move_step * 0.5;
       lat_move_dist += lat_move_step) {
    lat_dist_vec.emplace_back(lat_move_dist);
    lat_dist_vec.emplace_back(-lat_move_dist);
  }

  const double mirror_lower_body_lat_buffer =
      (param.lat_lon_path_buffer.in_slot_body_lat_buffer -
       param.lat_lon_path_buffer.in_slot_mirror_lat_buffer);

  std::vector<std::pair<double, double>> lat_body_mirror_buf_vec;
  lat_body_mirror_buf_vec.resize(lat_body_buffer_vec_.size());
  for (size_t i = 0; i < lat_body_buffer_vec_.size(); ++i) {
    lat_body_mirror_buf_vec[i] = {lat_body_buffer_vec_[i],
                                  lat_mirror_buffer_vec_[i]};
  }

  bool exist_target_pose = false;
  geometry_lib::PathPoint tmp_pose;
  for (const std::vector<double>& lon_dist_vec : all_lon_dist_vec) {
    for (const auto& lat_buffer : lat_body_mirror_buf_vec) {
      for (const double lat_move_dist : lat_dist_vec) {
        for (const double lon_move_dist : lon_dist_vec) {
          std::vector<geometry_lib::PathPoint> tmp_pose_vec;
          std::vector<double> lon_path_vec{lon_move_dist - lon_buffer_,
                                           lon_move_dist, lon_move_dist + 1.0,
                                           lon_move_dist + 2.0};
          for (const double dist : lon_path_vec) {
            if (base_on_slot_) {
              tmp_pose = tar_pose_local;
            } else {
              tmp_pose = tar_pose_global;
            }
            tmp_pose.pos = tmp_pose.pos + lat_move_dist * lat_move_dir +
                           dist * lon_move_dir;
            tmp_pose_vec.emplace_back(tmp_pose);
          }

          const ColResult& res =
              gjl_det_ptr->Update(tmp_pose_vec, lat_buffer.first, 0.0,
                                  gjk_col_det_request, true, lat_buffer.second);
          if (!res.col_flag) {
            exist_target_pose = true;
            result_.safe_lon_move_dist = lon_move_dist;
            result_.safe_lat_move_dist = lat_move_dist;
            result_.safe_lat_body_buffer = lat_buffer.first;
            result_.safe_lat_mirror_buffer = lat_buffer.second;
            if (base_on_slot_) {
              result_.target_pose_local = tmp_pose_vec[1];
              result_.target_pose_global =
                  geometry_lib::TransformPoseFromLocalToGlobal(
                      result_.target_pose_local, l2g_tf);
            } else {
              result_.target_pose_global = tmp_pose_vec[1];
              result_.target_pose_local =
                  geometry_lib::TransformPoseFromGlobalToLocal(
                      result_.target_pose_global, g2l_tf);
            }

            ILOG_INFO << "exist_target_pose = " << exist_target_pose
                      << "  safe_lon_move_dist = " << result_.safe_lon_move_dist
                      << "  safe_lat_move_dist = " << result_.safe_lat_move_dist
                      << "  safe_lat_body_buffer = "
                      << result_.safe_lat_body_buffer
                      << "  safe_lat_mirror_buffer = "
                      << result_.safe_lat_mirror_buffer;
            break;
          }
        }
        if (exist_target_pose) {
          break;
        }
      }
      if (exist_target_pose) {
        break;
      }
    }
    if (exist_target_pose) {
      break;
    }
  }

  if (exist_target_pose) {
    dx = result_.target_pose_local.pos.x() + param.wheel_base +
         param.front_overhanging -
         slot_.origin_corner_coord_local_.pt_01_mid.x();
    result_.exceed_allow_max_dx = dx - front_exceed_line_dx;
    ILOG_INFO << "exceed_allow_max_dx = " << result_.exceed_allow_max_dx
              << "  front_exceed_line_dx = " << front_exceed_line_dx
              << "  dx = " << dx;
    if (result_.exceed_allow_max_dx > 0.1) {
      exist_target_pose = false;
    }
  }

  if (!exist_target_pose) {
    result_.target_pose_type = TargetPoseType::FAIL;
  } else if (col_det_interface_ptr_->GetFoldMirrorFlag()) {
    result_.target_pose_type = TargetPoseType::FOLD_MIRROR;
  } else {
    result_.target_pose_type = TargetPoseType::NORMAL;
  }

  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForPerpendicularHeadIn() {
  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForPerpendicularTailOut() {
  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForPerpendicularHeadOut() {
  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForParallelTailIn() {
  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForParallelHeadingOut() {
  return result_;
}

}  // namespace apa_planner
}  // namespace planning