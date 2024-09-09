#pragma once

/**
 *  @file
 *  @brief Construct obstacle S-T graph for generating S-bounds and performing
 *velocity planning
 **/
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "agent_node_manager.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "filters.h"
#include "lateral_obstacle.h"
#include "lon_behavior_planner.pb.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "scc_lon_behavior_types.h"
#include "task_basic_types.h"
#include "tasks/behavior_planners/real_time_lane_change_decider/real_time_lane_change_decider.h"
#include "utils_math.h"
#include "virtual_lane.h"
namespace planning {
namespace scc {

// <id, s-t bounds>
using ObstacleStGraphs = std::map<int, Bounds>;

class StGraphGenerator {
 public:
  explicit StGraphGenerator(const SccLonBehaviorPlannerConfig &config,
                            framework::Session *session);

  virtual ~StGraphGenerator() = default;

  // 更新
  void Update(std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input,
              framework::Session *session);

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
                         bool is_on_ramp, bool is_continuous_ramp,
                         double ramp_v_limit, double acc_to_ramp, double v_ego);

  bool CalcCruiseAccelLimits(const double v_ego);

  bool CalcSpeedWithTurns(const double v_ego, const double angle_steers,
                          const std::vector<double> &d_poly);

  // Calculate acceleration range
  bool CalcAccLimits(const planning::common::TrackedObjectInfo &lead_obstacle,
                     const double desired_distance, const double v_target,
                     const double v_ego, const double lead_one_a_processed,
                     std::pair<double, double> &acc_target);
  double CalcPositiveAccLimit(const double v_ego, const double v_rel,
                              const double a_max_const);
  double CalcCriticalDecel(const double d_lead, const double v_rel,
                           const double d_offset, const double v_offset);

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

  bool CalcSpeedInfoWithVirtualObstacle(
      const std::shared_ptr<planning::planning_data::DynamicWorld>
          &dynamic_world,
      std::vector<planning::common::RealTimeLonObstacleSTInfo>
          &virtual_obs_st_info);

  bool CalcSpeedInfoWithIntersection();

  double DesiredDistanceFilter(
      const planning::common::TrackedObjectInfo &lead_obstacle,
      const double v_ego, double safe_distance, double desired_distance);

  double LeadtwoDesiredDistanceFilter(
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
      const planning::common::TrackedObjectInfo &lead_one, const double v_ego,
      const TrajectoryPoints &last_traj);

  // update vt_refs_
  void UpdateVelRefs();

  // make s v a j bound
  // TODO: add s bound make
  void MakeAccBound();

  void MakeJerkBound();

  void CalculateNarrowLimitSpeed(
      const planning::common::LatObsInfo &lateral_obstacles,
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
      std::shared_ptr<VirtualLane> current_lane,
      std::vector<planning::common::RealTimeLonObstacleSTInfo> &leads_st_info);

  void CalculateMergeSpeedLimit(
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
      std::vector<planning::common::RealTimeLonObstacleSTInfo> &merge_st_info,
      const double v_ego);

  void MergeSplitStaitcInfoProcess(std::shared_ptr<VirtualLane> current_lane);

  // use prediction info in dynamic word
  void CalculateMergeInfoWithAgent(
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
      const int64_t agent_node_id, const bool is_merging_to_left);

  bool EgoHasRightOfTargetLaneJudge(
      const std::shared_ptr<VirtualLane> target_lane,
      const std::shared_ptr<VirtualLane> ego_lane);

  // use prediction info in agent node manager
  void CalculateMergeInfoWithAgent(const int64_t agent_id,
                                   const bool is_merging_to_left,
                                   const string semantic_orientation_to_ego);

  double CalcDesiredDistance(const double intersection_front_one_velocity,
                             const bool is_lead, const bool is_accident_car,
                             const bool is_temp_lead, const double v_ego,
                             const std::string &lc_request);
  double MergeDesiredDistanceFilter(const double v_ego, double safe_distance,
                                    double desired_distance,
                                    const agent::Agent *merge_target_one);

  bool LateralCollisionCheck(const double &start_s, const double &end_s,
                             const double &agent_min_l);
  // TODO: need to remove when apply prediction from upstream
  // get prediction info from agent node manager
  void EgoNearByAgentsPredictionTrajProcess(
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world);

  void DebugAgentsPredictionTraj(
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world);

  bool FilterEgoRearAgentsWhenMerge(
      const int32_t agent_id,
      std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
      const std::shared_ptr<VirtualLane> ego_lane);

  void MergeInfoReset();

  void SetDefaultDebugValues(const std::vector<string> *names);
  void SetDefaultDebugValues(std::vector<string> names);

  // HACK: cross障碍物判断
  bool FastCrossAgentChecker(const double lead_one_v_lat, double &end_time,
                                   const double kwidth);

 private:
  framework::Session *session_;
  std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input_;
  SccLonBehaviorPlannerConfig config_;
  std::shared_ptr<RealTimeLaneChangeDecider> lane_changing_decider_ = nullptr;
  std::vector<double> st_refs_;
  std::vector<double> vt_refs_;
  double v_target_;
  double last_v_target_;
  std::pair<double, double> acc_target_;
  scc::STboundaries st_boundaries_;
  common::StartStopInfo start_stop_info_;
  std::array<double, 3> lon_init_state_;
  std::shared_ptr<KDPath> lat_path_coord_;
  std::vector<NarrowLead> narrow_agent_;

  // lead障碍物期望距离膨胀速率
  pnc::filters::SlopeFilter lead_desired_distance_filter_;
  pnc::filters::SlopeFilter lead_two_desired_distance_filter_;
  bool is_far_slow_safe_lead_ = false;
  // cut in障碍物期望距离膨胀速率
  pnc::filters::SlopeFilter cut_in_desired_distance_filter_;
  pnc::filters::SlopeFilter accel_vel_filter_;
  pnc::filters::SlopeFilter accel_vel_in_turns_filter_;

  // acc bound
  std::pair<double, double> acc_bound_;
  // jerk bound
  std::pair<double, double> jerk_bound_;
  // 为计算lc st时前车和后车期望距离，添加以下变量
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
  const std::vector<double> _AY_MAX_CURV_V{1.6, 0.9, 0.5, 0.4};
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

  const double _A_MAX = 2.0;
  const double _A_MIN = -4.0;
  const double _J_MAX = 7.0;
  const double _J_MIN = -3.0;

  // cutin calibration value
  const double CUIIN_WIDTH = 1.6;          // 类似半个车道宽，取窄
  const double CUIIN_WIDTH_STATIC = 1.35;  // 静态车辆cutin
  const double p1min_speed = 2.0;
  const double p2min_speed = 3.0;
  double v_limit_on_turns_and_road_;
  double v_limit_on_ramp_;
  double v_limit_lc_;
  double v_last_target_ = 0.0;
  double v_limit_with_intersection_ = 0.0;
  double v_hold_ = 0.0;
  planning::common::IntersectionState last_intersection_state_ =
      planning::common::UNKNOWN;
  planning::common::IntersectionState current_intersection_state_ =
      planning::common::UNKNOWN;

  // merge info
  std::shared_ptr<AgentNodeManager> agent_node_manager_;
  std::unordered_map<int64_t, ObstaclePredicatedInfo>
      agent_node_origin_lane_map_;
  std::unordered_map<int64_t, ObstaclePredicatedInfo>
      agent_node_left_neibor_lane_map_;
  std::unordered_map<int64_t, ObstaclePredicatedInfo>
      agent_node_right_neibor_lane_map_;

  MergeSplitPoints merge_split_points_;  // from perception
  // merge info from plan
  bool is_merge_region_ = false;
  //   MergeDirection merge_direction_ = MergeDirection::NONE_LANE_MERGE;
  MergeSplitPoints::MergeSplitOrientation merge_direction_ =
      MergeSplitPoints::UNKNOWN;
  int merge_lane_virtual_id_ = -1;
  // first: agent_node_id, second: t_intersect
  std::pair<int64_t, double> t_merge_with_agent_{
      planning_data::kInvalidId, std::numeric_limits<double>::max()};
  // first: angent_node_id, seconde: d_reletive_intersect
  std::pair<int64_t, double> d_relative_merge_with_agent_{
      planning_data::kInvalidId, std::numeric_limits<double>::max()};
  std::pair<int64_t, double> v_agent_merge_with_ego_{
      planning_data::kInvalidId, std::numeric_limits<double>::lowest()};
  std::pair<int64_t, double> d_current_relative_to_ego_{
      planning_data::kInvalidId, std::numeric_limits<double>::max()};
  string merge_target_one_semantic_orientation_to_ego_{};
  pnc::filters::SlopeFilter merge_desired_distance_filter_;
  bool ego_has_right_of_target_lane_{false};
  bool merge_target_one_has_changed_{false};
  int64_t last_merge_target_one_id_{planning_data::kInvalidId};
//   common::IntersectionState intersection_state_ =
//       common::IntersectionState::UNKNOWN;
};

}  // namespace scc
}  // namespace planning
