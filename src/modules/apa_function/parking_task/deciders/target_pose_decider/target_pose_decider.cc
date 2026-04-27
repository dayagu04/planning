#include "target_pose_decider.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_param_config.h"
#include "collision_detection/gjk_collision_detector.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "math_lib.h"
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
  ego_in_believe_slot_area_ = request.ego_in_believe_slot_area;
  scenario_type_ = request.scenario_type;

  const double find_target_pose_start_time = IflyTime::Now_ms();

  if (scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
      scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    result_ = CalcTargetPoseForPerpendicularParkingIn();
  }

  TimeBenchmark::Instance().SetTime(
      TimeBenchmarkType::TB_APA_FIND_TARGET_POSE_TIME,
      IflyTime::Now_ms() - find_target_pose_start_time);

  return result_;
}

const TargetPoseDeciderResult
TargetPoseDecider::CalcTargetPoseForPerpendicularParkingIn() {
  const bool heading_in =
      !is_searching_stage_ &&
      (scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN);
  ILOG_INFO << "CalcTargetPoseForPerpendicularParkingIn with fold mirror = "
            << col_det_interface_ptr_->GetFoldMirrorFlag()
            << "  heading in = " << heading_in;
  const ApaParameters& param = apa_param.GetParam();
  const geometry_lib::GlobalToLocalTf& g2l_tf = slot_.g2l_tf_;
  const geometry_lib::LocalToGlobalTf& l2g_tf = slot_.l2g_tf_;
  slot_.TransformCoordFromGlobalToLocal(g2l_tf);

  const SlotCoord& processed_pt_local = slot_.processed_corner_coord_local_;
  const SlotCoord& origin_pt_local = slot_.origin_corner_coord_local_;
  const SlotCoord& origin_pt_global = slot_.origin_corner_coord_global_;

  double virtual_tar_x = 0.0;
  if (slot_.limiter_.valid) {
    const Eigen::Vector2d pt1 = g2l_tf.GetPos(slot_.limiter_.start_pt);
    const Eigen::Vector2d pt2 = g2l_tf.GetPos(slot_.limiter_.end_pt);

    virtual_tar_x = 0.5 * (pt1 + pt2).x() + param.limiter_move_dist;

    // avoid limit error
    double max_tar_x = processed_pt_local.pt_01_mid.x() - 0.68;
    double min_tar_x = processed_pt_local.pt_23_mid.x() + 0.18;

    if (heading_in) {
      virtual_tar_x += param.wheel_base;
      max_tar_x = processed_pt_local.pt_01_mid.x() + 1.68;
    }

    if (!mathlib::IsInBound(virtual_tar_x, min_tar_x, max_tar_x)) {
      slot_.limiter_.valid = false;
    }
  }

  if (!slot_.limiter_.valid) {
    virtual_tar_x =
        origin_pt_local.pt_23_mid.x() + param.terminal_target_x_to_line;

    if (heading_in) {
      virtual_tar_x += (param.front_overhanging + param.wheel_base);
    } else {
      virtual_tar_x += param.rear_overhanging;
    }

    // park in the middle of the slot
    const double mid_x =
        (origin_pt_local.pt_01_mid.x() + origin_pt_local.pt_23_mid.x()) * 0.5;

    const double half_offset = 0.5 * param.car_length - param.rear_overhanging;
    const double mid_ego_x =
        heading_in ? (mid_x + half_offset) : (mid_x - half_offset);

    virtual_tar_x = std::max(virtual_tar_x, mid_ego_x);
  }

  const double redundant_y = (slot_.slot_width_ - param.car_width) * 0.5;
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

  const double terminal_target_heading =
      param.terminal_target_heading * kDeg2Rad;
  const double target_heading =
      heading_in ? (terminal_target_heading + M_PI) : terminal_target_heading;
  geometry_lib::PathPoint tar_pose_local;
  geometry_lib::PathPoint tar_pose_global;
  auto update_target_pose = [&](const double lat_offset) {
    tar_pose_local.pos << virtual_tar_x, lat_offset;
    tar_pose_local.heading = target_heading;
    tar_pose_global =
        geometry_lib::TransformPoseFromLocalToGlobal(tar_pose_local, l2g_tf);
  };
  update_target_pose(offset_y);

  ILOG_INFO << "target pose offset_y = " << offset_y;

  if (!consider_obs_) {
    result_.target_pose_type = TargetPoseType::NORMAL;
    result_.target_pose_local = tar_pose_local;
    result_.target_pose_global = tar_pose_global;
    return result_;
  }

  const Eigen::Vector2d lon_move_dir =
      base_on_slot_ ? origin_pt_local.pt_23mid_01mid_unit_vec
                    : origin_pt_global.pt_23mid_01mid_unit_vec;
  const Eigen::Vector2d lat_move_dir = base_on_slot_
                                           ? origin_pt_local.pt_01_unit_vec
                                           : origin_pt_global.pt_01_unit_vec;

  ApaObsMovementType consider_obs_movement_type;
  bool use_limiter;
  CarBodyType car_body_type;
  if (is_searching_stage_) {
    consider_obs_movement_type = slot_.is_selected_ ? ApaObsMovementType::STATIC
                                                    : ApaObsMovementType::ALL;
    use_limiter = false;
    car_body_type = CarBodyType::ONLY_MAX_POLYGAN;
  } else {
    consider_obs_movement_type = ApaObsMovementType::STATIC;
    use_limiter = true;
    car_body_type = heading_in ? CarBodyType::EXPAND_MIRROR_TO_END
                               : CarBodyType::EXPAND_MIRROR_TO_FRONT;
  }

  const GJKColDetRequest gjk_col_det_request(
      base_on_slot_, param.uss_config.use_uss_pt_cloud, car_body_type,
      consider_obs_movement_type, param.use_obs_height_method, use_limiter);

  const std::shared_ptr<GJKCollisionDetector>& gjk_det_ptr =
      col_det_interface_ptr_->GetGJKColDetPtr();
  const geometry_lib::PathPoint& base_target_pose =
      base_on_slot_ ? tar_pose_local : tar_pose_global;

  if (std::fabs(offset_y) > 1e-3) {
    std::vector<geometry_lib::PathPoint> tmp_pose_vec(3);
    const double preference_lon_path_vec[] = {
        -param.lat_lon_target_pose_buffer.preference_lon_buffer, 1.0, 2.0};
    for (size_t i = 0; i < tmp_pose_vec.size(); ++i) {
      tmp_pose_vec[i] = base_target_pose;
      tmp_pose_vec[i].pos += preference_lon_path_vec[i] * lon_move_dir;
    }

    if (gjk_det_ptr
            ->Update(tmp_pose_vec,
                     param.lat_lon_target_pose_buffer.max_lat_body_buffer, 0.0,
                     gjk_col_det_request, true,
                     param.lat_lon_target_pose_buffer.max_lat_mirror_buffer)
            .col_flag) {
      offset_y = 0.0;
      update_target_pose(offset_y);

      ILOG_INFO << "target pose offset_y is invalid, recover to 0.0 = "
                << offset_y;
    }
  }

  // calc max_lat_move_dist
  double max_lat_move_dist = 0.5 * (slot_.slot_width_ - param.car_width);
  // The distance between the body and the slot line: positive value indicates
  // the line cannot be crossed, negative value means crossing is allowed
  const double car2line_dist_threshold =
      is_searching_stage_ ? param.car2line_dist_threshold
                          : param.car2line_dist_threshold - 0.03;
  max_lat_move_dist -= car2line_dist_threshold;
  max_lat_move_dist = std::max(max_lat_move_dist, 0.0001);

  // calc max_lon_move_dist
  const double ego_front_x =
      heading_in ? (virtual_tar_x + param.rear_overhanging)
                 : (virtual_tar_x + param.wheel_base + param.front_overhanging);
  const double dx = origin_pt_local.pt_01_mid.x() - ego_front_x;

  double front_exceed_line_dx = 0.0;
  if (slot_.slot_source_type_ != SlotSourceType::SELF_DEFINE) {
    front_exceed_line_dx = param.max_front_exceed_line_dx;
    if (slot_.slot_type_ == SlotType::SLANT) {
      front_exceed_line_dx /= std::max(slot_.sin_angle_, 0.1);
    }
    if (!is_searching_stage_) {
      front_exceed_line_dx += ego_in_believe_slot_area_ ? 0.168 : 0.68;
    }
  }

  if (param.smart_fold_mirror_params.has_smart_fold_mirror &&
      !col_det_interface_ptr_->GetFoldMirrorFlag()) {
    front_exceed_line_dx = std::min(front_exceed_line_dx, 0.6);
    max_lat_move_dist = std::min(max_lat_move_dist, 0.1);
  }

  const double max_lon_move_dist =
      std::max(front_exceed_line_dx + dx - 0.05, 0.0001);

  // calc lat_move_step and lon_move_step
  const double lat_move_step = is_searching_stage_ ? 0.05 : 0.02;
  const double lon_move_step = is_searching_stage_ ? 0.1 : 0.05;

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

  bool exist_target_pose = false;
  std::vector<geometry_lib::PathPoint> tmp_pose_vec(4);
  const auto search_target_pose_in_lon_group =
      [&](const std::vector<double>& lon_dist_vec) {
        for (size_t i = 0; i < lat_body_buffer_vec_.size(); ++i) {
          const double lat_body_buffer = lat_body_buffer_vec_[i];
          const double lat_mirror_buffer = lat_mirror_buffer_vec_[i];
          for (const double lat_move_dist : lat_dist_vec) {
            const Eigen::Vector2d lat_offset = lat_move_dist * lat_move_dir;
            for (const double lon_move_dist : lon_dist_vec) {
              const double lon_path_vec[] = {lon_move_dist - lon_buffer_,
                                             lon_move_dist, lon_move_dist + 1.0,
                                             lon_move_dist + 2.0};
              for (size_t pose_idx = 0; pose_idx < tmp_pose_vec.size();
                   ++pose_idx) {
                tmp_pose_vec[pose_idx] = base_target_pose;
                tmp_pose_vec[pose_idx].pos +=
                    lat_offset + lon_path_vec[pose_idx] * lon_move_dir;
              }

              const ColResult& res = gjk_det_ptr->Update(
                  tmp_pose_vec, lat_body_buffer, 0.0, gjk_col_det_request, true,
                  lat_mirror_buffer);
              if (res.col_flag) {
                continue;
              }

              exist_target_pose = true;
              result_.safe_lon_move_dist = lon_move_dist;
              result_.safe_lat_move_dist = lat_move_dist;
              result_.safe_lat_body_buffer = lat_body_buffer;
              result_.safe_lat_mirror_buffer = lat_mirror_buffer;
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
                        << "  safe_lon_move_dist = "
                        << result_.safe_lon_move_dist
                        << "  safe_lat_move_dist = "
                        << result_.safe_lat_move_dist
                        << "  safe_lat_body_buffer = "
                        << result_.safe_lat_body_buffer
                        << "  safe_lat_mirror_buffer = "
                        << result_.safe_lat_mirror_buffer;
              return true;
            }
          }
        }
        return false;
      };

  if (is_searching_stage_) {
    exist_target_pose = search_target_pose_in_lon_group(small_lon_dist_vec);
  } else {
    exist_target_pose = search_target_pose_in_lon_group(small_lon_dist_vec) ||
                        search_target_pose_in_lon_group(big_lon_dist_vec);
  }

  if (exist_target_pose) {
    const double target_ego_front_x =
        heading_in ? (result_.target_pose_local.GetX() + param.rear_overhanging)
                   : (result_.target_pose_local.GetX() + param.wheel_base +
                      param.front_overhanging);
    const double target_dx = target_ego_front_x - origin_pt_local.pt_01_mid.x();
    result_.exceed_allow_max_dx = target_dx - front_exceed_line_dx;
    ILOG_INFO << "exceed_allow_max_dx = " << result_.exceed_allow_max_dx
              << "  front_exceed_line_dx = " << front_exceed_line_dx
              << "  target_dx = " << target_dx;
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