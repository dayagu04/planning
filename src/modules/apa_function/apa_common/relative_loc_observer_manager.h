// #include "Eigen/Matrix"
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "apa_obstacle_manager.h"
#include "apa_param_config.h"
#include "apa_predict_path_manager.h"
// #include "apa_slot_manager.h"
#include "apa_state_machine_manager.h"
#include "geometry_math.h"

#ifndef __RELATIVE_LOC_OBSERVER_MANEGER_H__
#define __RELATIVE_LOC_OBSERVER_MANEGER_H__

namespace planning {
namespace apa_planner {

struct RelativeLoc {
  double last_min_angle_diff;

  void Reset() { last_min_angle_diff = 180.0; }
};

class RelativeLocObserverManager {
 public:
  RelativeLocObserverManager();
  ~RelativeLocObserverManager();

  double CalCameraOberserveAngel(

              const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
              Eigen::Vector2d& slot_loc_in_world

  );
  void Reset();

  void SetRelativeLoc(const RelativeLoc& relative_loc);
  RelativeLoc GetRelativeLoc() const;


  bool RelativeApproachFlag() const;

 private:
  std::vector<Eigen::Vector2d> CalculateCameraDirections() const;

  std::vector<Eigen::Vector2d> BuildCameraCoordinateSystems(
      const Eigen::Matrix2d& rotation) const;

  std::vector<Eigen::Vector2d> CalculateCamsInSlotCoords(
      const pnc::geometry_lib::PathPoint& pose,
      const std::vector<Eigen::Vector2d>& cam_positions,
      const Eigen::Matrix2d& rotation) const;

  double CalculateAngleDiffFrom180(const Eigen::Vector2d& vec_a,
                                   const Eigen::Vector2d& vec_b) const;

  double CalculateAngleDiffInSlotCoord(
      const Eigen::Vector2d& slot_origin_in_camera_coord,
      const Eigen::Vector2d& cam_norm_in_global) ;

 private:
  RelativeLoc relative_loc_;

  std::shared_ptr<ApaObstacleManager> obstacle_manager_;



  const std::vector<Eigen::Vector2d> camera_positions_ = {
    //   Eigen::Vector2d(3.9, 0.0),   // 前摄像头
    //   Eigen::Vector2d(-1.0, 0.0),  // 后摄像头
      Eigen::Vector2d(2.0, 1.0),   // 左摄像头
      Eigen::Vector2d(2.0, -1.0)   // 右摄像头
  };

  bool angle_changed_flag_;
};

}  // namespace apa_planner
}  // namespace planning

#endif