
// #include <Eigen/src/Core/Matrix.h>
// #include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"

#include "relative_loc_observer_maneger.h"




namespace planning {
namespace apa_planner {

RelativeLocObserverManeger::RelativeLocObserverManeger() { Reset(); }

RelativeLocObserverManeger::~RelativeLocObserverManeger() {}

void RelativeLocObserverManeger::Reset() {
  relative_loc_.Reset();
  angle_changed_flag_ = false;
}

void RelativeLocObserverManeger::SetRelativeLoc(
    const RelativeLoc& relative_loc) {
  relative_loc_ = relative_loc;
}

RelativeLoc RelativeLocObserverManeger::GetRelativeLoc() const {
  return relative_loc_;
}


bool RelativeLocObserverManeger::RelativeApproachFlag() const {
  return angle_changed_flag_;
}

std::vector<Eigen::Vector2d>
RelativeLocObserverManeger::CalculateCameraDirections() const {
  std::vector<Eigen::Vector2d> camera_directions;
  camera_directions.reserve(camera_positions_.size());

//   for (const auto& position : camera_positions_) {
//     Eigen::Vector2d direction;

    // if (std::abs(position.x()) < 1e-6) {
    //   direction =
    //       position.y() > 0 ? Eigen::Vector2d(1, 0) : Eigen::Vector2d(-1, 0);
    // } else if (std::abs(position.y() - 2.0) < 1e-6) {
    //   direction =
    //       position.x() < 0 ? Eigen::Vector2d(0, 1) : Eigen::Vector2d(-1, 0);
    // } else {
    //   direction = position.normalized();
    // }
    Eigen::Vector2d vq ;
    // vq << 1, 0;

    // camera_directions.push_back(vq);
    // vq << -1, 0;
    // camera_directions.push_back(vq);
    vq << 0, 1;
    camera_directions.push_back(vq);
    vq << 0, -1;
    camera_directions.push_back(vq);



//   }

  return camera_directions;
}

std::vector<Eigen::Vector2d>
RelativeLocObserverManeger::BuildCameraCoordinateSystems(
    const Eigen::Matrix2d& rotation) const {
  std::vector<Eigen::Matrix2d> camera_coords_transforms;
  camera_coords_transforms.reserve(camera_positions_.size());

  std::vector<Eigen::Vector2d> cameras_norm_ego = CalculateCameraDirections();

  std::vector<Eigen::Vector2d> cameras_norm_slot;
  for (size_t i = 0; i < camera_positions_.size(); ++i) {
    cameras_norm_slot.emplace_back(rotation * cameras_norm_ego[i]);
  }

  return cameras_norm_slot;
}

std::vector<Eigen::Vector2d>
RelativeLocObserverManeger::CalculateCamsInSlotCoords(
    const pnc::geometry_lib::PathPoint& pose,
    const std::vector<Eigen::Vector2d>& cam_positions,
    const Eigen::Matrix2d& rotation) const {
  std::vector<Eigen::Vector2d> slot_origins_in_camera_coords;

  slot_origins_in_camera_coords.reserve(camera_positions_.size());

  Eigen::Vector2d slot_to_vehicle = pose.pos;

  Eigen::Vector2d slot_origin_in_vehicle = -slot_to_vehicle;

  std::vector<Eigen::Vector2d> cameras_in_slot;
  for (size_t i = 0; i < camera_positions_.size(); ++i) {
    cameras_in_slot.emplace_back(rotation * camera_positions_[i] + pose.pos);
  }

  return cameras_in_slot;
}

double RelativeLocObserverManeger::CalculateAngleDiffFrom180(
    const Eigen::Vector2d& vec_a, const Eigen::Vector2d& vec_b) const {
  double dot_product = vec_a.dot(vec_b);

  dot_product = std::max(-1.0, std::min(1.0, dot_product));
  double angle_rad = std::acos(dot_product);

  double angle_deg = angle_rad * (180.0 / M_PI);

  double diff_from_180 = std::abs(180.0 - angle_deg);

  return diff_from_180;
}

double RelativeLocObserverManeger::CalculateAngleDiffInSlotCoord(
    const Eigen::Vector2d& camera_to_slot_global,
    const Eigen::Vector2d& cam_norm_in_global)  {
  Eigen::Vector2d camera_to_slot = cam_norm_in_global;

//   if (camera_to_slot_global.norm() < 1e-6) {
//     return 180.0;
//   }

ILOG_INFO << "Camera to slot global: ";
  double dot_product = cam_norm_in_global.normalized().dot(camera_to_slot_global.normalized());

  dot_product = std::max(-1.0, std::min(1.0, dot_product));
  double angle_rad = std::acos(dot_product);

  double angle_deg = angle_rad * (180.0 / M_PI);

  double diff = std::abs( angle_deg);
  ILOG_INFO << "Angle diff: " << diff << " degrees";
  return diff;
}

double RelativeLocObserverManeger::CalCameraOberserveAngel(
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
              Eigen::Vector2d& slot_loc_in_world ) {


//   ILOG_INFO << "Update RelativeLocObserverManeger";



  const auto& cur_pose = measure_data_ptr->GetPose();

  ILOG_INFO << "Current pose: " << cur_pose.GetX() << ", " << cur_pose.GetY()
            << ", " << cur_pose.GetPhi();

  double cos_theta = std::cos(cur_pose.GetPhi() );
  double sin_theta = std::sin(cur_pose.GetPhi() );

  Eigen::Matrix2d rotation;
  rotation << cos_theta, -sin_theta, sin_theta, cos_theta;
  // cameras' optical center  norms in slot coods
  std::vector<Eigen::Vector2d> camera_norms_in_global =
      BuildCameraCoordinateSystems(rotation);
//   ILOG_INFO << "Camera norms in global coords: ";
  std::vector<Eigen::Vector2d>  camera_positions_in_global;
  for (auto& position : camera_positions_) {
    camera_positions_in_global.emplace_back(rotation * position + Eigen::Vector2d(cur_pose.GetX(),cur_pose.GetY()));
  }

  std::vector<double> angle_diffs;
  angle_diffs.reserve(camera_positions_.size());

  for (size_t i = 0; i < camera_positions_.size(); ++i) {
    // ILOG_INFO << "Camera " << i << " position: " << camera_positions_[i].x();
    Eigen::Vector2d cam_norm_in_global = camera_norms_in_global[i];
    Eigen::Vector2d camera_to_slot_global = slot_loc_in_world - camera_positions_in_global[i];
    double diff = CalculateAngleDiffInSlotCoord(camera_to_slot_global,
                                                cam_norm_in_global);
    angle_diffs.push_back(diff);

    // ILOG_INFO << "Camera angle diff: " << diff << " degrees";
  }

  double min_angle_diff =
      *std::min_element(angle_diffs.begin(), angle_diffs.end());

  angle_changed_flag_ = (relative_loc_.last_min_angle_diff > min_angle_diff);

  relative_loc_.last_min_angle_diff = min_angle_diff;

//   ILOG_INFO << "Min angle diff from 180 degrees: " << min_angle_diff;
//   ILOG_INFO << "Angle changed flag: "
//             << (angle_changed_flag_ ? "true" : "false");

  return min_angle_diff ;


}

}  // namespace apa_planner
}  // namespace planning
