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
  lat_buffer_vec_ = request.lat_buffer_vec;
  lon_buffer_ = request.lon_buffer;
  consider_obs_ = request.consider_obs;
  base_on_slot_ = request.base_on_slot;
  slot_lat_pos_preference_ = request.slot_lat_pos_preference;

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
  ILOG_INFO << "CalcTargetPoseForPerpendicularTailIn";
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
    }
  }
  if (!slot_.limiter_.valid) {
    virtual_tar_x = slot_.processed_corner_coord_local_.pt_23_mid.x() +
                    param.terminal_target_x;

    // park in the middle of the slot
    const double mid_ego_x =
        (slot_.processed_corner_coord_local_.pt_01_mid.x() -
         slot_.processed_corner_coord_local_.pt_23_mid.x()) *
            0.5 -
        (0.5 * param.car_length - param.rear_overhanging);

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
  // 人为设定偏左偏右 该值一般为0 后续可以根据HMI输入进行改变
  tar_pose_local.pos << virtual_tar_x, offset_y;
  // 人为设定航向 该值一般为0
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

  const GJKColDetRequest gjl_col_det_request(
      base_on_slot_, param.uss_config.use_uss_pt_cloud,
      CarBodyType::EXPAND_MIRROR_TO_FRONT);

  // 检查终点位置是否碰撞
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
                     param.lat_lon_target_pose_buffer.max_lat_buffer, 0.0,
                     gjl_col_det_request)
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
  // 车轮离车位线的距离 正数表示不能越过线 负数表示可以越过
  max_lat_move_dist -= param.car2line_dist_threshold;
  max_lat_move_dist = std::max(max_lat_move_dist, 0.0001);
  double max_lon_move_dist{0.};
  // 首先计算不移动时车头到车位前两个角点中点距离
  double dx = slot_.processed_corner_coord_local_.pt_01_mid.x() -
              (virtual_tar_x + param.wheel_base + param.front_overhanging);
  const double front_exceed_line_dx =
      apa_param.GetParam().max_front_exceed_line_dx;
  max_lon_move_dist = front_exceed_line_dx + dx;
  max_lon_move_dist = std::max(max_lon_move_dist, 0.0001);

  ILOG_INFO << "max_lon_move_dist = " << max_lon_move_dist
            << "  max_lat_move_dist = " << max_lat_move_dist;

  // calc lat_move_step and lon_move_step
  double lat_move_step{0.01};
  double lon_move_step{0.025};

  // 生成纵向移动距离数组
  std::vector<double> lon_dist_vec;
  for (double lon_move_dist = 0.0;
       lon_move_dist < max_lon_move_dist + lon_move_step * 0.5;
       lon_move_dist += lon_move_step) {
    lon_dist_vec.emplace_back(lon_move_dist);
  }

  // 生成横向移动距离数组
  std::vector<double> lat_dist_vec{0.0};
  for (double lat_move_dist = lat_move_step;
       lat_move_dist < max_lat_move_dist + lat_move_step * 0.5;
       lat_move_dist += lat_move_step) {
    lat_dist_vec.emplace_back(lat_move_dist);
    lat_dist_vec.emplace_back(-lat_move_dist);
  }

  bool exist_target_pose = false;

  geometry_lib::PathPoint tmp_pose;
  for (const double lat_buffer : lat_buffer_vec_) {
    for (const double lat_move_dist : lat_dist_vec) {
      for (const double lon_move_dist : lon_dist_vec) {
        std::vector<geometry_lib::PathPoint> tmp_pose_vec;
        // 考虑车辆纵向行驶路径上 如果有障碍物 则也视为不安全
        std::vector<double> lon_path_vec{
            lon_move_dist, lon_move_dist - lon_buffer_, lon_move_dist + 1.0,
            lon_move_dist + 2.0};
        for (const double dist : lon_path_vec) {
          if (base_on_slot_) {
            tmp_pose = tar_pose_local;
          } else {
            tmp_pose = tar_pose_global;
          }
          tmp_pose.pos =
              tmp_pose.pos + lat_move_dist * lat_move_dir + dist * lon_move_dir;
          tmp_pose_vec.emplace_back(tmp_pose);
        }
        if (!gjl_det_ptr
                 ->Update(tmp_pose_vec, lat_buffer, 0.0, gjl_col_det_request)
                 .col_flag) {
          exist_target_pose = true;
          result_.safe_lon_move_dist = lon_move_dist;
          result_.safe_lat_move_dist = lat_move_dist;
          result_.safe_lat_buffer = lat_buffer;
          if (base_on_slot_) {
            result_.target_pose_local = tmp_pose_vec.front();
            result_.target_pose_global =
                geometry_lib::TransformPoseFromLocalToGlobal(
                    result_.target_pose_local, l2g_tf);
          } else {
            result_.target_pose_global = tmp_pose_vec.front();
            result_.target_pose_local =
                geometry_lib::TransformPoseFromGlobalToLocal(
                    result_.target_pose_global, g2l_tf);
          }

          ILOG_INFO << "exist_target_pose = " << exist_target_pose
                    << "  safe_lon_move_dist = " << result_.safe_lon_move_dist
                    << "  safe_lat_move_dist = " << result_.safe_lat_move_dist
                    << "  safe_lat_buffer = " << result_.safe_lat_buffer;
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
    dx = slot_.processed_corner_coord_local_.pt_01_mid.x() -
         (result_.target_pose_local.pos.x() + param.wheel_base +
          param.front_overhanging);
    if (dx < -front_exceed_line_dx) {
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