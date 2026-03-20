#include "initial_action_decider.h"
#include "apa_param_config.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"

namespace planning {
namespace apa_planner {

void InitalActionDecider::Process(InitalActionRequest& inital_action_request,
                                  const geometry_lib::PathPoint& ego_pose,
                                  const geometry_lib::LineSegment& tar_line,
                                  const double node_step,
                                  const double sample_dist,
                                  const double max_safe_dist) {
  if (col_det_interface_ptr_ == nullptr) {
    return;
  }

  const double drive_ref_path_length =
      GenerateRefPathLength(ego_pose, sample_dist, max_safe_dist, true);

  const double reverse_ref_path_length =
      GenerateRefPathLength(ego_pose, sample_dist, max_safe_dist, false);

  ILOG_INFO << "drive_ref_path_length = " << drive_ref_path_length
            << ", reverse_ref_path_length = " << reverse_ref_path_length;

  inital_action_request.ref_reverse_length =
      std::min(reverse_ref_path_length, 6.8);
  inital_action_request.ref_drive_length = std::min(drive_ref_path_length, 6.8);

  const double dist_to_ref_line =
      geometry_lib::CalPoint2LineDist(ego_pose.pos, tar_line);

  double ref_length = std::max(dist_to_ref_line, 1.2);

  if (inital_action_request.ref_gear == AstarPathGear::DRIVE) {
    ref_length = std::min(ref_length, drive_ref_path_length);
  } else if (inital_action_request.ref_gear == AstarPathGear::REVERSE) {
    ref_length = std::min(ref_length, reverse_ref_path_length);
  } else {
    ref_length =
        std::min({ref_length, drive_ref_path_length, reverse_ref_path_length});
  }

  // ensure that the reference length is an integer multiple of node_step to
  // avoid invalid searches caused by the need to extend an additional node
  ref_length = std::floor(ref_length / node_step) * node_step;

  inital_action_request.ref_length = ref_length;

  return;
}

const double InitalActionDecider::GenerateRefPathLength(
    const geometry_lib::PathPoint& ego_pose, const double sample_dist,
    const double traj_length, const bool is_forward) {
  double ref_length = traj_length;
  std::vector<geometry_lib::PathPoint> path;
  const auto& edt_col_det_ptr = col_det_interface_ptr_->GetEDTColDetPtr();

  const ApaParameters& param = apa_param.GetParam();

  const double lat_buffer = param.gen_ref_length_lat_buffer;
  const double lon_buffer = param.gen_ref_length_lon_buffer;

  ColResult res;
  GeneratePath(ego_pose, sample_dist, traj_length, is_forward, 0.0, true, path);
  res = edt_col_det_ptr->Update(path, lat_buffer, lon_buffer);
  ref_length = std::min(ref_length, res.remain_dist);

  GeneratePath(ego_pose, sample_dist, traj_length, is_forward,
               apa_param.GetParam().min_turn_radius, true, path);
  res = edt_col_det_ptr->Update(path, lat_buffer, lon_buffer);
  ref_length = std::min(ref_length, res.remain_dist);

  GeneratePath(ego_pose, sample_dist, traj_length, is_forward,
               -apa_param.GetParam().min_turn_radius, true, path);
  res = edt_col_det_ptr->Update(path, lat_buffer, lon_buffer);
  ref_length = std::min(ref_length, res.remain_dist);

  return ref_length;
}

void InitalActionDecider::GeneratePath(
    const geometry_lib::PathPoint& ego_pose, const double sample_dist,
    const double traj_length, const bool is_forward, const double turn_radius,
    const bool is_straight, std::vector<geometry_lib::PathPoint>& path) {
  path.clear();
  const size_t path_point_num = std::ceil(traj_length / sample_dist) + 1;
  path.resize(path_point_num);
  const Eigen::Vector2d heading_vec =
      geometry_lib::GenHeadingVec(ego_pose.heading);
  Eigen::Vector2d move_vec;
  Eigen::Matrix2d rot_mat;
  Eigen::Vector2d center;
  double rot_angle = 0.0;
  if (is_straight) {
    // straight path
    move_vec = heading_vec * sample_dist;
    if (!is_forward) {
      move_vec *= -1.0;
    }
  } else {
    // turn path
    rot_angle = 90.0 * kDeg2Rad;
    if (turn_radius < 0.0) {
      rot_angle *= -1.0;
    }
    rot_mat = geometry_lib::GetRotm2dFromTheta(rot_angle);
    center = ego_pose.pos + rot_mat * heading_vec * std::fabs(turn_radius);

    rot_angle = sample_dist / std::fabs(turn_radius);
    if ((is_forward && turn_radius < 0.0) ||
        (!is_forward && turn_radius > 0.0)) {
      rot_angle *= -1.0;
    }
    rot_mat = pnc::geometry_lib::GetRotm2dFromTheta(rot_angle);
  }

  geometry_lib::PathPoint new_pose = ego_pose;
  new_pose.s = 0.0;
  path[0] = new_pose;
  for (size_t i = 1; i < path_point_num; ++i) {
    if (is_straight) {
      new_pose.pos += move_vec;
    } else {
      new_pose.pos = rot_mat * (new_pose.pos - center) + center;
      new_pose.heading =
          geometry_lib::NormalizeAngle(new_pose.heading + rot_angle);
    }
    new_pose.s += sample_dist;
    path[i] = new_pose;
  }
}

}  // namespace apa_planner
}  // namespace planning