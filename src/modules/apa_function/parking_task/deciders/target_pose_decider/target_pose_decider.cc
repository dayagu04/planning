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
  // 根据车位建立车位坐标系，方便计算终点与逻辑判断
  const Eigen::Vector2d heading_vec =
      slot_.processed_corner_coord_global_.pt_23mid_01mid_unit_vec;

  const double origin_heading = std::atan2(heading_vec.y(), heading_vec.x());

  const Eigen::Vector2d origin_pos =
      slot_.processed_corner_coord_global_.pt_01_mid -
      slot_.slot_length_ * heading_vec;

  geometry_lib::GlobalToLocalTf g2l_tf =
      geometry_lib::GlobalToLocalTf(origin_pos, origin_heading);

  geometry_lib::LocalToGlobalTf l2g_tf =
      geometry_lib::LocalToGlobalTf(origin_pos, origin_heading);

  slot_.TransformCoordFromGlobalToLocal(g2l_tf);

  double virtual_tar_x = 0.0;
  if (slot_.limiter_.valid) {
    Eigen::Vector2d pt1 = g2l_tf.GetPos(slot_.limiter_.start_pt);
    Eigen::Vector2d pt2 = g2l_tf.GetPos(slot_.limiter_.end_pt);

    virtual_tar_x = 0.5 * (pt1 + pt2).x() + param.limiter_move_dist;

    // avoid limit error
    if (virtual_tar_x > slot_.processed_corner_coord_local_.pt_01_mid.x() -
                            param.wheel_base - 0.168) {
      slot_.limiter_.valid = false;
    }
  }
  if (!slot_.limiter_.valid) {
    virtual_tar_x = slot_.processed_corner_coord_local_.pt_23_mid.x() +
                    param.terminal_target_x;
  }

  // If the limiter is too far back or the slot is too long,  can combine the
  // front corner information of slot
  virtual_tar_x = std::max(virtual_tar_x,
                           slot_.processed_corner_coord_local_.pt_01_mid.x() -
                               param.limiter_length - param.wheel_base -
                               param.front_overhanging);

  geometry_lib::PathPoint tar_pose_local;
  geometry_lib::PathPoint tar_pose_global;
  // 人为设定偏左偏右 该值一般为0 后续可以根据HMI输入进行改变
  tar_pose_local.pos << virtual_tar_x, param.terminal_target_y;
  // 人为设定航向 该值一般为0
  tar_pose_local.heading = param.terminal_target_heading * kDeg2Rad;

  tar_pose_global =
      geometry_lib::TransformPoseFromLocalToGlobal(tar_pose_local, l2g_tf);

  if (!consider_obs_) {
    result_.exist_target_pose = true;
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

  // 检查终点位置是否碰撞
  const std::shared_ptr<GJKCollisionDetector>& gjl_det_ptr =
      col_det_interface_ptr_->GetGJKCollisionDetectorPtr();

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

  GJKColDetRequest gjl_col_det_request(base_on_slot_, false,
                                       CarBodyType::EXPAND_MIRROR_TO_FRONT);
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
          result_.exist_target_pose = true;
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

          ILOG_INFO << "exist_target_pose = " << result_.exist_target_pose
                    << "  safe_lon_move_dist = " << result_.safe_lon_move_dist
                    << "  safe_lat_move_dist = " << result_.safe_lat_move_dist
                    << "  safe_lat_buffer = " << result_.safe_lat_buffer;
          break;
        }
      }
      if (result_.exist_target_pose) {
        break;
      }
    }
    if (result_.exist_target_pose) {
      break;
    }
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