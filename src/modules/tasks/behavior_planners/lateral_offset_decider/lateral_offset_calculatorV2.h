#pragma once

#include "avoid_obstacle_maintainer5V.h"
#include "config/basic_type.h"
#include "define/planning_status.h"
#include "lateral_obstacle.h"
#include "utils/pose2d_utils.h"
#include "virtual_lane_manager.h"

namespace planning {
enum class AvoidWay { None, Left, Right, Center, Ego };
class LateralOffsetCalculatorV2 {
 public:
  LateralOffsetCalculatorV2() = default;
  LateralOffsetCalculatorV2(const EgoPlanningConfigBuilder* config_builder);
  ~LateralOffsetCalculatorV2() = default;

  bool Process(framework::Session* session,
               const std::array<AvoidObstacleInfo, 2>& avd_obstacle,
               const std::array<AvoidObstacleInfo, 2>& avd_sp_obstacle,
               double dist_rblane, bool flag_avd);

  double lat_offset() const { return lat_offset_; }

  const std::vector<double>& left_lane_boundary_poly() {
    return left_lane_boundary_poly_;
  };
  const std::vector<double>& right_lane_boundary_poly() {
    return right_lane_boundary_poly_;
  };
  void Reset();

 private:
  void set_left_lane_boundary_poly() {
    for (auto i = 0;
         i < flane_->get_left_lane_boundary().poly_coefficient_size(); ++i) {
      if (i < 4) {
        left_lane_boundary_poly_.push_back(
            flane_->get_left_lane_boundary().poly_coefficient(i));
      }
    }
  }

  void set_right_lane_boundary_poly() {
    for (auto i = 0;
         i < flane_->get_right_lane_boundary().poly_coefficient_size(); ++i) {
      if (i < 4) {
        right_lane_boundary_poly_.push_back(
            flane_->get_right_lane_boundary().poly_coefficient(i));
      }
    }
  }
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

  void calc_desired_path(const std::array<double, 4>& l_poly,
                         const std::array<double, 4>& r_poly, double l_prob,
                         double r_prob, double intercept_width,
                         std::array<double, 4>& d_poly);

  double calc_lane_width_by_dist(const std::vector<double>& left_poly,
                                 const std::vector<double>& right_poly,
                                 const double& dist_x);

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
  double DealwithObstacleLR(const AvoidObstacleInfo& avoid_obstacle_1,
                            const AvoidObstacleInfo& avoid_obstacle_2,
                            bool is_side_way);
  double DealwithObstacleRL(const AvoidObstacleInfo& avoid_obstacle_1,
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
                                    bool is_left, double coeff,
                                    double lat_compensate);
  double DesireLateralOffsetCenterWay(const AvoidObstacleInfo& avoid_obstacle_1,
                                      const AvoidObstacleInfo& avoid_obstacle_2,
                                      bool is_left, double lat_compensate_1,
                                      double lat_compensate_2);
  void CalcMaxOppositeOffset(
      const AvoidObstacleInfo& avoid_obstacle_1, int except_id = -1,
      const AvoidObstacleInfo& avoid_obstacle_2 = AvoidObstacleInfo());
  double LimitLateralOffset(double lateral_offset, bool is_left,
                            AvoidWay* avoid_way);
  double SmoothLateralOffset(const AvoidObstacleInfo& avoid_obstacle,
                             double lat_offset, const AvoidWay* avoid_way);
  void PostProcess();
  void SaveDebugInfo();

 private:
  VisionLateralMotionPlannerConfig config_;
  framework::Session* session_;

  double lat_offset_ = 0.0;
  double last_lat_offset_ = 0.0;
  double curr_time_ = 0;

  int reject_reason_ = NO_REJECTION;
  bool l_reject_ = false;
  bool r_reject_ = false;
  double intercept_width_ = 3.8;

  double lane_width_ = 3.8;
  std::array<double, 4> c_poly_;
  std::array<double, 4> d_poly_;
  std::array<double, 4> l_poly_;
  std::array<double, 4> r_poly_;
  std::vector<double> left_lane_boundary_poly_;
  std::vector<double> right_lane_boundary_poly_;
  std::shared_ptr<ReferencePath> fix_reference_path_;
  std::shared_ptr<VirtualLane> flane_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;

  bool is_on_rightest_lane_ = false;
  bool is_on_leftest_lane_ = false;
  double normal_avoid_threshold_ = 0.0;
  AvoidWay avoid_way_;
  double allow_front_max_opposite_offset_;
  int allow_front_max_opposite_offset_id_ = -1;
  double allow_side_max_opposite_offset_;
  int allow_side_max_opposite_offset_id_ = -1;
  double last_front_allow_max_opposite_offset_;
  double last_side_allow_max_opposite_offset_;
};

}  // namespace planning
