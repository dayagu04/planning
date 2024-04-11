#pragma once

/**
 *  @file
 *  @brief Construct obstacle S-T graph for generating S-bounds and performing
 *velocity planning
 **/
#include <utility>
#include "debug_info_log.h"
#include "filters.h"
#include "lateral_obstacle.h"
#include "lon_behavior_planner.pb.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "scc_lon_behavior_types.h"
#include "task_basic_types.h"
#include "tasks/behavior_planners/real_time_lane_change_decider/real_time_lane_change_decider.h"

namespace planning {
namespace scc {

// <id, s-t bounds>
using ObstacleStGraphs = std::map<int, Bounds>;

class StGraphGenerator {
 public:
  explicit StGraphGenerator(const SccLonBehaviorPlannerConfig &config);

  virtual ~StGraphGenerator() = default;

  // 更新
  void Update(
      std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input);

  void SetConfig(
      planning::common::RealTimeLonBehaviorTunedParams &tuned_params);

  // 获取所有关心的障碍物都st boundaries
  const scc::STboundaries &GetSTboundaries() const { return st_boundaries_; }
  const std::vector<double> &GetSTRefs() const { return st_refs_; }

  const std::vector<double> &GetVTRefs() const { return vt_refs_; }

  const common::StartStopInfo &GetStartStopState() const {
    return start_stop_info_;
  }

  const std::pair<double, double> &GetAccBound() const { return acc_bound_; }

  const std::pair<double, double> &GetJerkBound() const { return jerk_bound_; }

 private:
  // 计算单个障碍物的st
  void ComputeYieldStGraph();
  void CalculateCruiseSrefs(const double v_ego, const double v_cruise,
                            const double acc_ego, std::vector<double> &s_refs);
  std::pair<double, double> CalculateMaxAcc(double ego_v);

  // 计算lead障碍物的st信息，包括期望跟车距离、最小跟车距离与期望速度
  bool CalcSpeedInfoWithLead(
      const planning::common::TrackedObjectInfo &lead_one,
      const planning::common::TrackedObjectInfo &lead_two,
      const string &lc_request, const double v_ego,
      std::vector<planning::common::RealTimeLonObstacleSTInfo> &leads_st_info);

  bool CalcSpeedInfoWithTempLead(
      const planning::common::TrackedObjectInfo &lead_one,
      const planning::common::TrackedObjectInfo &lead_two, double v_ego,
      const planning::common::LatOutputInfo &lateral_outputs,
      std::vector<planning::common::RealTimeLonObstacleSTInfo>
          &temp_leads_st_info);

  void CalcSpeedInfoWithCutin(
      const planning::common::LatObsInfo &lateral_obstacles,
      const string &lc_request, double v_cruise, double v_ego,
      std::vector<planning::common::RealTimeLonObstacleSTInfo> &cut_in_st_info);

  bool CalcSpeedWithRamp(double dis_to_ramp, double dis_to_merge,
                         bool is_on_ramp, double ramp_v_limit,
                         double acc_to_ramp, double v_ego);

  bool CalcSpeedWithTurns(const double v_ego, const double angle_steers,
                          const std::vector<double> &d_poly);

  void UpdateSTRefs(const std::vector<double> &sref_vec);

  void UpdateSTGraphs(
      const std::vector<common::RealTimeLonObstacleSTInfo> &st_infos,
      const std::vector<double> &sref_vec);

  // 对障碍物加速度进行降噪处理，吸收0.5m/s2的波动
  double ProcessObstacleAcc(const double a_lead);

  void UpdateNearObstacles(
      const planning::common::LatObsInfo &lateral_obstacles,
      const string &lc_request, double v_ego);

  // compute desired distance
  double CalcDesiredDistance(
      const planning::common::TrackedObjectInfo &lead_obstacle,
      const double v_ego, const std::string &lc_request);
  double GetRSSDistance(const double obstacle_velocity, double ego_velocity);
  double GetCalibratedDistance(const double v_lead, const double v_ego,
                               const std::string &lc_request,
                               const bool is_accident_car = false,
                               const bool is_temp_lead = false,
                               const bool is_lead = false);

  // compute safe distance
  double CalcSafeDistance(const double obstacle_velocity, const double v_ego);

  // compute desired speed
  double CalcDesiredVelocity(const double d_rel, const double d_des,
                             const double v_lead);

  void UpdateSpeedWithPotentialCutinCar(
      const planning::common::LatObsInfo &lateral_obstacles,
      const std::string &lc_request, double v_cruise, double v_ego,
      std::vector<planning::common::RealTimeLonObstacleSTInfo> &cut_in_st_info);

  void CalcSpeedInfoWithGap(
      const planning::common::TrackedObjectInfo &lead_one,
      const double v_cruise, const double v_ego, const string &lc_request,
      const string &lc_status,
      std::vector<planning::common::RealTimeLonObstacleSTInfo>
          &lane_change_st_info);

  double DesiredDistanceFilter(
      const planning::common::TrackedObjectInfo &lead_obstacle,
      const double v_ego, double safe_distance, double desired_distance);

  double LCGapDesiredDistanceFilter(
      const planning::common::TrackedObjectInfo &lead_obstacle,
      const double v_ego, double safe_distance, double desired_distance,
      bool is_front);

  double TmpLeadDesiredDistanceFilter(
      const planning::common::TrackedObjectInfo &lead_obstacle,
      const double v_ego, double safe_distance, double desired_distance);

  double CutInDesiredDistanceFilter(
      const planning::common::TrackedObjectInfo &lead_obstacle,
      const double v_ego, double safe_distance, double predict_distance,
      double desired_distance);

  void CalculateSrefsByVref(const double v_ego, std::vector<double> &v_refs,
                            const double acc_ego, std::vector<double> &s_refs);

  // 计算启停状态，避免二次起步
  common::StartStopInfo::StateType UpdateStartStopState(
      const planning::common::TrackedObjectInfo &lead_one, const double v_ego);

  // update vt_refs_
  void UpdateVelRefs();

  // make s v a j bound
  // TODO: add s bound make
  void MakeAccBound();

  void MakeJerkBound();

 private:
  std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input_;
  SccLonBehaviorPlannerConfig config_;
  std::shared_ptr<RealTimeLaneChangeDecider> lane_changing_decider_ = nullptr;
  std::vector<double> st_refs_;
  std::vector<double> vt_refs_;
  double v_target_;
  double last_v_target_;
  scc::STboundaries st_boundaries_;
  common::StartStopInfo start_stop_info_;
  std::array<double, 3> lon_init_state_;

  // lead障碍物期望距离膨胀速率
  pnc::filters::SlopeFilter lead_desired_distance_filter_;
  // cut in障碍物期望距离膨胀速率
  pnc::filters::SlopeFilter cut_in_desired_distance_filter_;
  pnc::filters::SlopeFilter accel_vel_filter_;

  // acc bound
  std::pair<double, double> acc_bound_;
  // jerk bound
  std::pair<double, double> jerk_bound_;
  // s bound
  // std::pair<double, double> s_bound_;
  //为计算lc st时前车和后车期望距离，添加以下变量
  int lc_front_id_ = -10;
  int lc_rear_id_ = -20;
  double lc_front_desired_distance_;
  double lc_rear_desired_distance_;

  // 需要进行优化和剔除
 private:
  // lookup tables VS speed to determine min and max accels in cruise
  const std::vector<double> _A_CRUISE_MIN_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MIN_V{-0.5, -0.5, -0.5, -0.5, -0.3};
  // need fast accel at very low speed for stop and go
  const std::vector<double> _A_CRUISE_MAX_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MAX_V{1.0, 0.85, 0.6, 0.5, 0.3};
  // change the ay_limit values according to the angle steers
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{100, 200, 400, 600};
  const std::vector<double> _AY_MAX_CURV_V{1.2, 0.6, 0.4, 0.3};
  const std::vector<double> _T_GAP_VEGO_BP{5.0, 15.0, 30.0};
  const std::vector<double> _T_GAP_VEGO_V{1.35, 1.55, 2.0};
  // linear slope
  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  // parabola slope
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};
  // do not consider a_lead at 0m/s, fully consider it at 5m/s
  const std::vector<double> _A_LEAD_LOW_SPEED_BP{0.0, 5.0};
  const std::vector<double> _A_LEAD_LOW_SPEED_V{0.0, 1.0};
  // lower a_lead when distance is far away
  const std::vector<double> _A_LEAD_DISTANCE_BP{50.0, 100.0};
  const std::vector<double> _A_LEAD_DISTANCE_V{1.0, 0.5};
  // different offset based on the likelyhood that lead decels abruptly
  const std::vector<double> _DECEL_OFFSET_BP{0.0, 4.0, 15.0, 30.0, 40.0};
  const std::vector<double> _DECEL_OFFSET_V{-0.3, -0.5, -0.5, -0.4, -0.3};
  // maximum acceleration adjustment
  const std::vector<double> _A_CORR_BY_SPEED_BP{0.0, 2.0, 10.0};
  const std::vector<double> _A_CORR_BY_SPEED_V{0.4, 0.4, 0.0};
  // lower y_thres when cutin lon speed is faster
  const std::vector<double> _Y_THRES_SPEED_BP{0.0, 1.0};
  const std::vector<double> _Y_THRES_SPEED_V{0.8, 0.0};
  const std::vector<double> _CUT_IN_COEFF_BP{1.6, 1.8, 2.0, 2.5};
  const std::vector<double> _CUT_IN_COEFF_V{1.0, 1.0, 1.33, 0.9 / 0.65};
  const std::vector<double> _D_THRES_SPEED_BP{0.5, 1.0};
  const std::vector<double> _D_THRES_SPEED_V{0.5, 1.0};
  // lower prebrk by ttc
  const std::vector<double> _A_PREBRK_TTC_BP{5.0, 10.0};
  const std::vector<double> _A_PREBRK_TTC_V{1.0, 0.0};

  const double _J_MAX = 7.0;
  const double _J_MIN = -3.0;

  // cutin calibration value
  const double CUIIN_WIDTH = 1.6;  // 类似半个车道宽，取窄
  const double p1min_speed = 2.0;
  const double p2min_speed = 3.0;
};

}  // namespace scc
}  // namespace planning
