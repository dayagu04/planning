#pragma once
#include <array>
#include <vector>
#include "config/basic_type.h"
#include "lateral_obstacle.h"
#include "lateral_offset_decider_utils.h"
#include "session.h"
#include "task_basic_types.h"

constexpr double MIN_T_EXCEED_AVD_CAR = 0.1;
namespace planning {
class AvoidObstacleMaintainer5V {
 public:
  AvoidObstacleMaintainer5V() = default;
  ~AvoidObstacleMaintainer5V() = default;

  bool Process(planning::framework::Session *session);
  const std::array<AvoidObstacleInfo, 2> &avd_obstacles() const {
    return avd_obstacles_;
  };
  const std::array<AvoidObstacleInfo, 2> &avd_obstacles_history() const {
    return avd_obstacles_history_;
  };
  const std::array<AvoidObstacleInfo, 2> &avd_sp_obstacles() const {
    return avd_sp_obstacles_;
  }
  double dist_rblane() const { return dist_rblane_; }
  bool flag_avd() const { return flag_avd_; }
  void Reset();

 private:
  bool UpdateLFrontAvdsInfo(bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
                            AvoidObstacleInfo &avd_obstacle2);
  bool UpdateRFrontAvdsInfo(bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
                            AvoidObstacleInfo &avd_obstacle2);
  bool UpdateLSideAvdsInfo(bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
                           AvoidObstacleInfo &avd_obstacle2);
  bool UpdateRSideAvdsInfo(bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
                           AvoidObstacleInfo &avd_obstacle2);
  bool IsOutAvoidArea(const std::shared_ptr<LateralObstacle> lateral_obstacle,
                      AvoidObstacleInfo &avd_obstacle1);

  void SelectCurAvoidObstacles(
      const std::shared_ptr<LateralObstacle> lateral_obstacle, double v_ego,
      std::vector<AvoidObstacleInfo> &avd_obstacles);
  void UpdateAvoidObstacle(
      const std::shared_ptr<LateralObstacle> lateral_obstacle);
  void SelectAvoidObstacle(const std::vector<AvoidObstacleInfo> &avd_obstacles);
  void UpdateAvoidObstacleInfo1(std::vector<AvoidObstacleInfo> &avd_obstacles);
  void UpdateAvoidObstacleInfo2(
      const std::shared_ptr<LateralObstacle> lateral_obstacle,
      double t_interval);
  void CheckAvoidObstacle(
      const std::shared_ptr<LateralObstacle> lateral_obstacle);
  void UpdateAvoidObstacleInfo3();
  bool CampareDistance(const AvoidObstacleInfo &avd_obstacle1,
                       const AvoidObstacleInfo &avd_obstacle2);
  void SaveDebugInfo();
  planning::framework::Session *session_;
  bool is_ncar_ = false;
  double final_y_rel_ = 10;

  int flag_avd_ = 0;
  int avd_back_start_time_ = 0;

  double dist_rblane_ = 0;
  double lane_width_ = 0.;
  double ego_length_ = 0.;

  std::array<AvoidObstacleInfo, 2> avd_obstacles_;
  std::array<AvoidObstacleInfo, 2> avd_obstacles_history_;
  std::array<AvoidObstacleInfo, 2> avd_sp_obstacles_;

  std::vector<AvoidObstacleInfo> avd_obstacles_cur_;
};
}  // namespace planning