#pragma once

#include "avoid_obstacle_maintainer5V.h"
#include "config/basic_type.h"
#include "define/planning_status.h"
#include "lateral_obstacle.h"
#include "utils/pose2d_utils.h"
#include "virtual_lane_manager.h"

namespace planning {

class LateralOffsetCalculatorV2 {
 public:
  LateralOffsetCalculatorV2() = default;
  LateralOffsetCalculatorV2(const EgoPlanningConfigBuilder* config_builder);
  ~LateralOffsetCalculatorV2() = default;

  bool Process(framework::Session* session,
               const std::array<AvoidObstacleInfo, 2>& avd_obstacle,
               const std::array<AvoidObstacleInfo, 2>& avd_sp_obstacle,
               double dist_rblane, bool flag_avd);

  double lat_offset() const { return avoid_info_.lat_offset; }
  void Reset();

 private:
  void CalLaneWidth();
  bool UpdateBasicPath(const int& status);
  bool UpdateAvoidPath(int status, bool flag_avd, bool accident_ahead,
                       bool should_premove, double dist_rblane,
                       const std::array<AvoidObstacleInfo, 2>& avd_obstacle,
                       const std::array<AvoidObstacleInfo, 2>& avd_sp_obstacle);
  bool UpdateLateralOffset(
      int status, bool flag_avd, bool accident_ahead, bool should_premove,
      double dist_rblane, const std::array<AvoidObstacleInfo, 2>& avd_obstacle,
      const std::array<AvoidObstacleInfo, 2>& avd_sp_obstacle);

  bool update(int lane_status, bool flag_avoid, bool exist_accident_ahead,
              bool execute_premove, bool should_suspend, double dist_rblane,
              const std::array<AvoidObstacleInfo, 2>& avoid_car_info,
              const std::array<AvoidObstacleInfo, 2>& avoid_sp_car_info);

  void CalculateNormalLateralOffsetThreshold();
  bool AvoidWaySelectForTwoObstacle(const AvoidObstacleInfo& avoid_obstacle_1,
                                    const AvoidObstacleInfo& avoid_obstacle_2,
                                    double* t_exceed_obstacle_1);
  void LateralOffsetCalculateOneObstacle(
      const AvoidObstacleInfo& avoid_obstacle);
  void LateralOffsetCalculateTwoObstacle(
      const AvoidObstacleInfo& avoid_obstacle_1,
      const AvoidObstacleInfo& avoid_obstacle_2);
  void DealwithObstacleL(const AvoidObstacleInfo& avoid_obstacle);
  void DealwithObstacleR(const AvoidObstacleInfo& avoid_obstacle);
  double DealwithTwoObstacleTwoSide(const AvoidObstacleInfo& avoid_obstacle_1,
                            const AvoidObstacleInfo& avoid_obstacle_2,
                            bool is_side_way);
  double DealwithTwoObstacleOneSide(const AvoidObstacleInfo& avoid_obstacle_1,
                            const AvoidObstacleInfo& avoid_obstacle_2,
                            bool is_side_way);
  double DealwithObstacleLL(const AvoidObstacleInfo& avoid_obstacle_1,
                            const AvoidObstacleInfo& avoid_obstacle_2,
                            bool is_side_way);
  double DealwithObstacleRR(const AvoidObstacleInfo& avoid_obstacle_1,
                            const AvoidObstacleInfo& avoid_obstacle_2,
                            bool is_side_way);
  double LateralOffsetCompensate(const AvoidObstacleInfo& avoid_obstacle);
  double DesireLateralOffsetSideWay(const AvoidObstacleInfo& avoid_obstacle,
                                    const AvoidWay &avoid_way, double coeff,
                                    double lat_compensate, double base_distance);
  double DesireLateralOffsetCenterWay(const AvoidObstacleInfo& avoid_obstacle_1,
                                      const AvoidObstacleInfo& avoid_obstacle_2,
                                      bool is_left, double lat_compensate_1,
                                      double lat_compensate_2);
  void CalcMaxOppositeOffset(
      const AvoidObstacleInfo& avoid_obstacle_1, int except_id = -1,
      const AvoidObstacleInfo& avoid_obstacle_2 = AvoidObstacleInfo());
  void CalcFrontMaxOppositeOffset(const vector<int>& front_ids, bool is_left);
  double LimitLateralOffset(double lateral_offset, const AvoidWay &avoid_way);
  double SmoothLateralOffset(const AvoidObstacleInfo& avoid_obstacle,
                             double lat_offset, const AvoidWay* avoid_way);
  void PostProcess(const std::array<AvoidObstacleInfo, 2> &avd_obstacle);
  void SaveDebugInfo();

 private:
  LateralOffsetDeciderConfig config_;
  framework::Session* session_;

  // double lat_offset_ = 0.0;
  // double last_lat_offset_ = 0.0;
  double curr_time_ = 0;

  double lane_width_ = 3.8;
  std::shared_ptr<ReferencePath> fix_reference_path_;
  std::shared_ptr<VirtualLane> flane_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;

  bool is_on_rightest_lane_ = false;
  bool is_on_leftest_lane_ = false;
  AvoidInfo avoid_info_;
  AvoidInfo last_avoid_info_;
  // double normal_avoid_threshold_ = 0.0;
  // AvoidWay avoid_way_;
  // double allow_front_max_opposite_offset_;
  // int allow_front_max_opposite_offset_id_ = -1;
  // double allow_side_max_opposite_offset_;
  // int allow_side_max_opposite_offset_id_ = -1;
  // double last_front_allow_max_opposite_offset_;
  // double last_side_allow_max_opposite_offset_;
};



}  // namespace planning
