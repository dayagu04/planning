#pragma once

#include <array>
#include <memory>
#include <vector>

// #include "agent/agent.h"
// #include "agent_node_manager.h"
#include <assert.h>
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "dp_st_cost.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "filters.h"
#include "grid_map.h"
#include "reference_path.h"
#include "slt_graph_point.h"
#include "speed_data.h"
#include "src/modules/common/math/curve1d/cubic_polynomial_curve1d.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
#include "utils_math.h"
#include "virtual_lane.h"

// #define OPENMP_DEBUG     // 开启调试输出
// #define SERIAL_MODE   // 开启性能对比
// #define VALIDATE_PARALLEL  // 开启正确性校验

namespace planning {

struct SLTGraphMessage {
  SLTGraphMessage(const uint32_t c_, const int32_t r_, const int32_t k_)
      : c(c_), r(r_), k(k_) {}
  uint32_t c;
  uint32_t r;
  uint32_t k;
};

class SpatioTemporalUnionDp {
 public:
  void Init();

  bool Update(TrajectoryPoints &traj_points,
              const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
              const planning::common::SpationTemporalUnionDpInput
                  &spatio_temporal_union_plan_input,
              const double &target_s, planning_math::KDPath &current_lane_coord,
              const int &half_lateral_sample_nums,
              const bool &last_enable_using_st_plan);

  planning::common::TrajectoryPoints &GetOutput() { return trajectory_points_; }

  void Reset();

 private:
  bool InitCostTable(const planning::common::SpationTemporalUnionDpInput
                         &spatio_temporal_union_plan_input,
                     const int &half_lateral_sample_nums,
                     const double &target_s);

  bool InitSpeedLimitLookUp(const planning::common::SpationTemporalUnionDpInput
                                &spatio_temporal_union_plan_input);

  bool RetrieveSpeedProfile(TrajectoryPoints &traj_points,
                            const planning::common::SpationTemporalUnionDpInput
                                &spatio_temporal_union_plan_input);

  bool CalculateTotalCost(
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
      const double &target_s,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input);

  // defined for cyber task

  void CalculateCostAt(
      const SLTGraphMessage *msg,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
      const double &target_s,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input);

  double CalculateEdgeCost(const SLTGraphPoint &first,
                           const SLTGraphPoint &second,
                           const SLTGraphPoint &third,
                           const SLTGraphPoint &forth,
                           const double &speed_limit,
                           const double &cruise_speed);
  double CalculateEdgeCostForSecondCol(
      const uint32_t row, const uint32_t col, const double &speed_limit,
      const double &cruise_speed,
      const planning::common::EgoInitInfo &init_state);
  double CalculateEdgeCostForThirdCol(
      const uint32_t curr_row, const uint32_t curr_col, const uint32_t pre_row,
      const uint32_t pre_col, const double &speed_limit,
      const double &cruise_speed,
      const planning::common::EgoInitInfo &init_state);

  double CalculatePathCost(const SLTGraphPoint &start, const SLTGraphPoint &end,
                           const CubicPolynomialCurve1d &lateral_curve,
                           const double &acc,
                           const planning::common::SpationTemporalUnionDpInput
                               &spatio_temporal_union_plan_input);

  double CalculateStitchingCost(
      const Point2D &current, const double &current_time,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input);

  double CalculateDynamicObstacleCost(
      const SLTGraphPoint &pre_point, const SLTGraphPoint &cur_point,
      const CubicPolynomialCurve1d &lateral_curve, const double &acc,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input,
      double *distance_to_point, int *agent_id);

  double GetLongitCostBetweenObsBoxes(
      const double &longit_dis_to_ego,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input);

  double GetLateralCostBetweenObsBoxes(
      const double &lateral_dis_to_ego,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input);

  // get the row-range of next time step
  void GetRowRange(const SLTGraphPoint &point, int *next_highest_row,
                   int *next_lowest_row, const double &init_v,
                   const double &v_cruise);

  void GetColumnRange(const SLTGraphPoint &point, int *next_highest_l,
                      int *next_lowest_l);

  bool IsOffRoad(const double ref_s, const double l, const double dl,
                 const bool is_change_lane_path);

  bool CheckOverlapOnDpSltGraph(
      const SLTGraphPoint &cur_cost_point,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs);

  void GetVehicleBoxSLVertices(const planning_math::Vec2d &ego_point,
                               const double &cos_theta, const double &sin_theta,
                               std::array<planning_math::Vec2d, 8> &vertices);

  double ComputeFirstDerivative(const std::array<double, 4> &lateral_curve_coef,
                                const std::array<double, 4> &longit_curve_coef,
                                const double param);

  double ComputeSecondDerivative(
      const std::array<double, 4> &lateral_curve_coef, const double &acc,
      const double &v0, const double &param);

  void CaculateCurvatureLimitSpeed(
      double target_s, double &speed_limit,
      const planning::common::SpationTemporalUnionDpInput
          &spatio_temporal_union_plan_input);

  void CalculateMapSpeedLimit(
      double &speed_limit, const planning::common::SpationTemporalUnionDpInput
                               &spatio_temporal_union_plan_input);

  bool IsValidCurve(const CubicPolynomialCurve1d &curve);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>> &lane_s_width);

  void GenerateExpandedTrajectoryPoint(const SpeedInfo &init_point,
                                       const SpeedInfo &next_point);

  bool JudgeSVecisMonotonicIncreasing(const std::vector<double> &vec);

  void PrecomputeInvSpeedLimit();

  void FallbackFunction(const planning::common::SpationTemporalUnionDpInput
                            &spatio_temporal_union_plan_input,
                        TrajectoryPoints &traj_points, const bool &last_enable_using_st_plan);

  planning::common::TrajectoryPoints trajectory_points_;
  std::vector<double> speed_limit_by_index_;
  std::vector<double> inv_speed_limit_table_;

  std::vector<double> spatial_distance_by_index_;

  std::vector<double> lateral_distance_by_index_;

  // 当前车道中心线kd_path
  planning_math::KDPath current_lane_coord_;

  // 轨迹kd_path
  std::shared_ptr<KDPath> last_planning_result_coord_;

  std::shared_ptr<ReferencePath> current_refline_;

  std::vector<std::pair<double, double>>
      current_lane_s_width_;  // <s, lane_width>
  // initial status
  PlanningInitPoint planning_init_point_;

  Point2D ego_cart_point_;

  bool enable_use_ego_cart_point_ = true;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  // planning::common::SpationTemporalUnionDpInput
  // spatio_temporal_union_dp_input_;

  double total_length_t_ = 0.0;
  double unit_t_ = 0.0;
  double inv_unit_t_ = 0.0;
  double t_squared_ = 0.0;
  int dimension_t_ = 0;

  double total_length_s_ = 0.0;
  double dense_unit_s_ = 0.0;
  double sparse_unit_s_ = 0.0;
  double unit_l_ = 0.0;
  int dense_dimension_s_ = 0;

  int sparse_dimension_s_ = 0;
  int dimension_s_ = 0;
  int dimension_l_ = 0;

  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  bool s_vec_is_monotonic_increasing_ = false;

  double longit_risk_distance_between_obstacle_ = 20.0;

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<std::vector<SLTGraphPoint>>> cost_table_;

  std::vector<std::vector<std::vector<SLTGraphPoint>>> serial_cost_table_;

  std::vector<SpeedInfo> dp_speed_profile_;

  // used in curv speed limit
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{50, 100, 200, 300, 400};
  const std::vector<double> _AY_MAX_CURV_V{2.2, 1.6, 1.1, 0.9, 0.8};

  const std::array<double, 8> lateral_sampling_length_interval_{
      0.3, 0.3, 0.2, 0.15, 0.15, 0.2, 0.3, 0.3};

  double l0_ = 1.50;
  double b_ = 0.40;
  double k_ = 1.5;
};

}  // namespace planning
