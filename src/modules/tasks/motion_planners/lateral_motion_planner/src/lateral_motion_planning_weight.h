#ifndef __LATERAL_MOTION_PLANNING_WEIGHT_H__
#define __LATERAL_MOTION_PLANNING_WEIGHT_H__

#include <cstddef>
#include <vector>
#include "ego_planning_config.h"
#include "lateral_motion_planner.pb.h"
#include "virtual_lane.h"

namespace pnc {
namespace lateral_planning {

enum LateralMotionSceneEnum {
  LANE_KEEP = 0,
  AVOID,
  LANE_CHANGE,
  STATIC_AVOID,
  SPLIT,
  RAMP
};

struct PathWeight {  // temp
  size_t point_num = 26;
  size_t proximal_index = 5;
  size_t remotely_index = 20;
  bool complete_follow = false;
  double dt = 0.2;
  double total_t = 5.0;
  double acc_soft_bound = 2.0;
  double acc_hard_bound = 3.0;
  double jerk_soft_bound = 0.4;
  double jerk_hard_bound = 0.8;
  std::vector<double> q_ref_x;
  std::vector<double> q_ref_y;
  std::vector<double> q_ref_theta;
  std::vector<double> q_continuity;
  std::vector<double> q_acc;
  std::vector<double> q_jerk;
  std::vector<double> q_acc_soft_bound;
  std::vector<double> q_acc_hard_bound;
  std::vector<double> q_jerk_soft_bound;
  std::vector<double> q_jerk_hard_bound;
  std::vector<double> q_pos_soft_bound;
  std::vector<double> q_pos_hard_bound;
  std::unordered_map<size_t, double> time2soft_ratio = {
    {0, 5.0},
    {1, 3.5},
    {2, 2.0},
    {3, 1.0},
    {4, 0.3}
  };
  std::unordered_map<size_t, double> time2hard_ratio = {
    {0, 10.0},
    {1, 8.0},
    {2, 5.0},
    {3, 2.0},
    {4, 0.6}
  };
  void Init() {
    q_ref_x.resize(point_num, 0);
    q_ref_y.resize(point_num, 0);
    q_ref_theta.resize(point_num, 0);
    q_continuity.resize(point_num, 0);
    q_acc.resize(point_num, 0);
    q_jerk.resize(point_num, 0);
    q_acc_soft_bound.resize(point_num, 0);
    q_acc_hard_bound.resize(point_num, 0);
    q_jerk_soft_bound.resize(point_num, 0);
    q_jerk_hard_bound.resize(point_num, 0);
    q_pos_soft_bound.resize(point_num, 0);
    q_pos_hard_bound.resize(point_num, 0);
  }
};

class LateralMotionPlanningWeight {
 public:
  explicit LateralMotionPlanningWeight(
      const planning::LateralMotionPlannerConfig &config);
  ~LateralMotionPlanningWeight() = default;

  void Init();

  void CalculateInitInfo(
      const planning::common::LateralPlanningInput &planning_input);

  void CalculateLatAvoidDistance(
      const std::vector<std::pair<double, double>> &bounds);

  void CalculateLatAvoidBoundPriority(
      const std::vector<std::pair<double, double>> &soft_bounds,
      const std::vector<std::pair<double, double>> &hard_bounds,
      const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>> &soft_bounds_info,
      const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>> &hard_bounds_info);

  void SetLateralMotionWeight(
      const LateralMotionSceneEnum scene,
      planning::common::LateralPlanningInput &planning_input);

  void SetInitDisToRef(const double init_dis_to_ref) {
    init_dis_to_ref_ = init_dis_to_ref;
  }

  void SetInitRefThetaError(const double init_ref_theta_error) {
    init_ref_theta_error_ = init_ref_theta_error;
  }

  void SetEgoVel(const double ego_vel) { ego_vel_ = ego_vel; }

  void SetEgoL(const double ego_l) { ego_l_ = ego_l; }

  void SetInitL(const double init_l) { init_l_ = init_l; }

  void SetLateralOffset(const double lat_offset) { lat_offset_ = lat_offset; }

  void SetMaxAcc(const double max_acc) { max_acc_ = max_acc; }

  void SetMaxJerk(const double max_jerk) { max_jerk_ = max_jerk; }

  void SetLCBackFlag(const bool is_lane_change_back) {
    is_lane_change_back_ = is_lane_change_back;
  }

  void SetIsInIntersection(const bool is_in_intersection) {
    is_in_intersection_ = is_in_intersection;
  }

  void SetIsSearchSuccess(const bool is_search_success) {
    is_search_success_ = is_search_success;
  }

  void SetMotionPlanConcernedEndIndex(
      bool origin_complete_follow, bool is_divide_lane_into_two, double ego_s,
      planning::common::LateralPlanningInput &planning_input,
      const std::shared_ptr<planning::planning_math::KDPath> &frenet_coord);

  double GetInitDisToRef() const { return init_dis_to_ref_; }

  double GetConcernedStartQJerk() const { return concerned_start_q_jerk_; }

  double GetConcernedEndRatioForXY() const { return end_ratio_for_qrefxy_; }

  double GetConcernedEndRatioForTheta() const {
    return end_ratio_for_qreftheta_;
  }

  double GetConcernedEndRatioForJerk() const {
    return end_ratio_for_qjerk_;
  }

  const std::vector<double>& GetSoftBoundWeightRatioVec() const { return q_soft_bound_vec_; }

  const std::vector<double>& GetHardBoundWeightRatioVec() const { return q_hard_bound_vec_; }

  const PathWeight& GetPathWeights() const { return weight_; }

 private:
  void SetAccJerkBoundAndWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeLaneChangeDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeSplitDynamicWeight(
      planning::common::LateralPlanningInput &planning_input);

  void MakeDynamicPosBoundWeight(
      planning::common::LateralPlanningInput &planning_input);

 private:
  planning::LateralMotionPlannerConfig config_;
  LateralMotionSceneEnum lateral_motion_scene_;
  PathWeight weight_;
  double lat_offset_;
  double avoid_dist_;
  double init_dis_to_ref_;
  double init_ref_theta_error_;
  double concerned_start_q_jerk_;
  double ego_vel_;
  double ego_l_;
  double init_l_;
  double end_ratio_for_qrefxy_;
  double end_ratio_for_qreftheta_;
  double end_ratio_for_qjerk_;
  double max_acc_;
  double max_jerk_;
  bool is_lane_change_back_;
  bool is_in_intersection_;
  bool is_emergence_;
  bool is_search_success_;
  std::vector<double> q_soft_bound_vec_;
  std::vector<double> q_hard_bound_vec_;
};

}  // namespace lateral_planning
}  // namespace pnc
#endif