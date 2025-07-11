#include "spatio_temporal_union_dp.h"
#include <math.h>
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/spatio_temporal_planner/slt_graph_point.h"
#include "behavior_planners/spatio_temporal_planner/slt_point.h"
#include "behavior_planners/spatio_temporal_planner/speed_data.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "math/box2d.h"
#include "spatio_temporal_union_dp_input.pb.h"
#include "src/modules/common/math/line_segment2d.h"
#include "src/modules/common/math/curve1d/cubic_polynomial_curve1d.h"
#include "src/modules/common/math/math_utils.h"
#include "src/modules/tasks/task_interface/lane_change_decider_output.h"
#include "src/modules/common/trajectory1d/variable_coordinate_time_optimal_trajectory.h"
#include "src/modules/common/trajectory1d/trajectory1d.h"
#include "src/library/advanced_ctrl_lib/include/spline.h"
#include "src/modules/common/math/linear_interpolation.h"
#include "src/modules/common/agent/agent.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>
#include "log.h"
#include "vec2d.h"
#include <omp.h>

namespace planning {
using namespace planning_math;


namespace {
constexpr double kDoubleEpsilon = 1.0e-6;
constexpr double kMaxLateralDistance = 20.0;
constexpr double kDynamicObsWeight = 1e-4;
constexpr double kDynamicObsConsiderTime = 3.0;
constexpr double kPathCostComputeSampleTime = 0.5;
constexpr double kDynamicObstacleCostSampleTime = 0.2;
constexpr double kDynamicObstacleCostLaterSampleTime = 0.5;
constexpr double kPlanningUpperSpeedLimit = 16.67;
constexpr double kHighVel = 100 / 3.6;
constexpr double kVirtualAgentBuffer = 2.0;
constexpr double kJerkMin = -2.0;
constexpr double kJerkMax = 2.0;
constexpr double kEgoStaticThreshold = 0.1;
constexpr double kDeltaTime = 0.2;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
constexpr double kDefaultlLinearInterpolationDistance = 20.0;
constexpr double kDefaultConsiderObstacleTajsTime = 2.6;
constexpr double kConsiderLaneLineMinLength = 50.0;
constexpr double kDefaultTrajsTimeLength = 5.0;
constexpr double kSpeedRangeBuffer = 0.20;
constexpr double kObstacleRiskDistanceCost = 2.0e3;
constexpr double kWithoutLateralOverlapObstacleRiskDistanceCost = 2.0e4;
constexpr double kSpecialObstacleDistanceCost = 1.2;
constexpr int kDefaultValidBoxNums = 4;
constexpr int kDefaultTrajsPointNums = 5;
constexpr int kDefaultIndexInterval = 1;
constexpr double kCollisionBetweenObstacleLongitDistanceThreshold = 0.3;
constexpr double kCollisionBetweenObstacleLateralDistanceThreshold = 0.2;
constexpr double kCollisionBetweenObstacleDistanceThreshold = 0.35;
constexpr double kObstacleCollisionDistanceCost = 1.0e8;
constexpr double kLateralMaxAcceleration = 1.44;
constexpr double kLateralMaxDeceleration = -1.44;
constexpr double kDefaultMaxDisBetweenObs = 100.0;
constexpr double kDefaultMaxDeceleration = -4.4;
constexpr double kDefaultMaxAcceleration = 2.5;
constexpr double kDefaultRearObsGenerateCostDistance= 3.0;
constexpr double kDefaultHalfSamplingRange= 0.95;
constexpr int kDefaultTrajectoryPointSize = 26;
constexpr double kConsiderDynamicObstacleCostTimeLength = 4.0;


}
// namespace

void SpatioTemporalUnionDp::Init() {
}

void SpatioTemporalUnionDp::Reset() {
  trajectory_points_.Clear();
  last_planning_result_coord_ = nullptr;
}

bool SpatioTemporalUnionDp::Update(
    TrajectoryPoints &traj_points,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input,
    const double &target_s,
    planning_math::KDPath &current_lane_coord,
    const int &half_lateral_sample_nums,
    const bool &last_enable_using_st_plan) {
  dp_st_cost_.Init(spatio_temporal_union_plan_input.long_weight_params());
  enable_use_ego_cart_point_ = true;
  current_lane_coord_ = current_lane_coord;
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();
  Point2D ego_frenet_point(ego_init_state.s(), ego_init_state.l());
  if(!current_lane_coord_.SLToXY(ego_frenet_point,
                              ego_cart_point_)) {
    enable_use_ego_cart_point_ = false;
  }

  if (!InitCostTable(spatio_temporal_union_plan_input, half_lateral_sample_nums, target_s)) {
    LOG_DEBUG("Initialize cost table failed.");
    return false;
  }

  if (!InitSpeedLimitLookUp(spatio_temporal_union_plan_input)) {
    LOG_DEBUG("Initialize speed limit lookup table failed.");
    return false;
  }

  auto time_start = IflyTime::Now_ms();
  if (!CalculateTotalCost(agent_trajs, target_s, spatio_temporal_union_plan_input)) {
    LOG_DEBUG("Calculate total cost failed.");
    return false;
  }
  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("SpatioTemporalUnionDp::Update() CalculateTotalCost cost:%f\n", time_end - time_start);


  if (!RetrieveSpeedProfile(traj_points, spatio_temporal_union_plan_input)) {
    LOG_DEBUG("Retrieve best speed profile failed.");
    FallbackFunction(spatio_temporal_union_plan_input, traj_points, last_enable_using_st_plan);
    return false;
  }

  return true;
}

bool SpatioTemporalUnionDp::InitCostTable(
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input,
    const int &half_lateral_sample_nums,
    const double &target_s) {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  const auto& dp_search_paramms = spatio_temporal_union_plan_input.dp_search_paramms();
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();

  // total_length_s_ = std::max(ego_init_state.v0(), ego_init_state.v_cruise()) * 5.0;
  // total_length_s_ = std::max(total_length_s_, kConsiderLaneLineMinLength);
  total_length_s_ = std::min(target_s, current_lane_coord_.Length() - ego_init_state.s());

  unit_t_ = dp_search_paramms.unit_t();
  inv_unit_t_ = 1.0 / unit_t_;
  t_squared_ = unit_t_ * unit_t_;
  dense_dimension_s_ = dp_search_paramms.dense_dimension_s();
  total_length_t_ = dp_search_paramms.total_length_t();
  dense_unit_s_ = dp_search_paramms.dense_unit_s();
  sparse_unit_s_ = dp_search_paramms.sparse_unit_s();
  unit_l_ = dp_search_paramms.unit_l();
  if (unit_t_ < kDoubleEpsilon) {
    LOG_DEBUG("unit_t is smaller than the kDoubleEpsilon.");
    return false;
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    LOG_DEBUG("dense_dimension_s is at least 1.");
    return false;
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;

  double sparse_length_s =
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  sparse_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;
  dimension_l_ = 2 * half_lateral_sample_nums + 1;
  max_acceleration_ = dp_search_paramms.max_acceleration();
  max_deceleration_ = dp_search_paramms.max_deceleration();

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    LOG_DEBUG("Dp st cost table size incorrect.");
    return false;
  }

  cost_table_ = std::vector<std::vector<std::vector<SLTGraphPoint>>>(
      dimension_t_, std::vector<std::vector<SLTGraphPoint>>(
        dimension_s_, std::vector<SLTGraphPoint>(dimension_l_, SLTGraphPoint())));

  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    // double curr_s = 0.0;
    double curr_s = ego_init_state.s();
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      double curr_l = ego_init_state.l() - kDefaultHalfSamplingRange;
      for (uint32_t k = 0; k < dimension_l_; ++k, curr_l += lateral_sampling_length_interval_[k-1]) {
        cost_table_i[j][k].Init(i, j, k,  SLTPoint(curr_s,curr_l, curr_t));
      }
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_ + ego_init_state.s();
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {
      double curr_l = ego_init_state.l() - kDefaultHalfSamplingRange;
      for (uint32_t k = 0; k < dimension_l_; ++k, curr_l += lateral_sampling_length_interval_[k-1]) {
        cost_table_i[j][k].Init(i, j, k,  SLTPoint(curr_s,curr_l, curr_t));
      }
    }
  }
  auto& cost_init = cost_table_[0][0][0];
  SLTPoint init_point(ego_init_state.s(), ego_init_state.l(), 0.0);
  cost_init.Init(0, 0, 0, init_point);

  const auto& cost_table_0 = cost_table_[0];
  const auto& cost_table_0_0 = cost_table_[0][0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  lateral_distance_by_index_ = std::vector<double>(cost_table_0_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i][0].point().s();
  }
  double min_l = ego_init_state.l() - kDefaultHalfSamplingRange;
  for (uint32_t i = 0; i < cost_table_0_0.size(); ++i) {
    lateral_distance_by_index_[i] = min_l;
    min_l += lateral_sampling_length_interval_[i];
  }
  l0_ = spatio_temporal_union_plan_input.lat_path_weight_params().path_l_cost_param_l0();
  b_ = spatio_temporal_union_plan_input.lat_path_weight_params().path_l_cost_param_b();
  k_ = spatio_temporal_union_plan_input.lat_path_weight_params().path_l_cost_param_k();
  return true;
}

bool SpatioTemporalUnionDp::InitSpeedLimitLookUp(
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();
  speed_limit_by_index_.clear();
  inv_speed_limit_table_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  for (size_t i = 0;i < speed_limit_by_index_.size(); i++) {
    speed_limit_by_index_[i] = ego_init_state.v_cruise();
    CaculateCurvatureLimitSpeed(
        spatial_distance_by_index_[i], speed_limit_by_index_[i], spatio_temporal_union_plan_input);
    CalculateMapSpeedLimit(
        speed_limit_by_index_[i], spatio_temporal_union_plan_input);
  }
  // 依赖纵向提供地图限速、曲率限速信息
  // const auto& speed_limit = st_graph_data_.speed_limit();

  // for (uint32_t i = 0; i < dimension_s_; ++i) {
  //   speed_limit_by_index_[i] =
  //       speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  // }

  PrecomputeInvSpeedLimit();
  return true;
}

bool SpatioTemporalUnionDp::CalculateTotalCost(
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const double &target_s,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  // x y and z are for SLTGraph
  // l corresponding to y
  // s corresponding to x
  // t corresponding to z
  int next_highest_row = 0;
  int next_lowest_row = 0;
  // const int lowest_l = 0;
  // const int highest_l = dimension_l_;
  int next_highest_l = 0;
  int next_lowest_l = 0;
  const int consider_dynamic_obs_time_thd_index = 3;
  const double init_v = spatio_temporal_union_plan_input.init_state().v0();
  const double v_cruise = spatio_temporal_union_plan_input.init_state().v_cruise();

  #ifdef OPENMP_DEBUG
  // 添加线程数量探测
  #pragma omp parallel
  {
    #pragma omp single
    LOG_DEBUG("[OpenMP] Using %d threads", omp_get_num_threads());
  }
  #endif

  // 保留串行结果
  #ifdef VALIDATE_PARALLEL
  serial_cost_table_ = cost_table_;  // 深拷贝当前状态
  #endif

  double total_parallel_time = 0.0;
  double total_serial_time = 0.0;
  for (int c = 0; c < cost_table_.size(); ++c) {
    // LOG_DEBUG(
    //     "CalculateTotalCost c: %d \n, next_lowest_row: %d \n, next_highest_row : %d \n", c, next_lowest_row, next_highest_row);
    int highest_row = 0;
    int lowest_row = cost_table_.back().size() - 1;
    int highest_l = 0;
    int lowest_l = dimension_l_ - 1;

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;

    auto time_start = IflyTime::Now_us();
    if (count > 0) {
      if (spatio_temporal_union_plan_input.dp_search_paramms().enable_use_parallel_calculate_cost()) {
        // 添加OpenMP并行
        #pragma omp parallel for collapse(2) schedule(dynamic)
        for (int r = next_lowest_row; r <= next_highest_row; ++r) {
          for (int k = next_lowest_l; k <= next_highest_l; ++k) {
            #ifdef OPENMP_DEBUG
            // 打印线程分配信息（控制频率）
            if (r % 10 == 0 && k == lowest_l) {
              #pragma omp critical
              LOG_DEBUG("[OpenMP] c=%d r=%d k=%d -> Thread %d",
                      c, r, k, omp_get_thread_num());
            }
            #endif

            // auto msg = std::make_shared<SLTGraphMessage>(c, r, k);
            SLTGraphMessage msg(c, r, k);
            // LOG_DEBUG("cost_cr c: %d, r: %d, k: %d \n", c, (int)r, (int)k);
            CalculateCostAt(
                &msg, agent_trajs, target_s, spatio_temporal_union_plan_input);
          }
        }
      } else {
        for (int r = next_lowest_row; r <= next_highest_row; ++r) {
          for (int k = next_lowest_l; k <= next_highest_l; ++k) {

            // auto msg = std::make_shared<SLTGraphMessage>(c, r, k);
            SLTGraphMessage msg(c, r, k);
            // LOG_DEBUG("cost_cr c: %d, r: %d, k: %d \n", c, (int)r, (int)k);
            auto t1 = IflyTime::Now_us();
            CalculateCostAt(
                &msg, agent_trajs, target_s, spatio_temporal_union_plan_input);
            auto t2 = IflyTime::Now_us();
            // LOG_DEBUG("one time CalculateCostAt: %.8f\n", t2 - t1);
          }
        }
      }
    }
    auto time_end = IflyTime::Now_us();
    total_parallel_time += (time_end - time_start);
    // LOG_DEBUG("CalculateTotalCost: time dimension cost:%.8f\n", time_end - time_start);

    // 保留串行版本用于对比
    #ifdef SERIAL_MODE
    auto serial_start = IflyTime::Now_us();
    if (count > 0) {
      for (int r = next_lowest_row; r <= next_highest_row; ++r) {
        for (int k = next_lowest_l; k <= next_highest_l; ++k) {

          // auto msg = std::make_shared<SLTGraphMessage>(c, r, k);
          SLTGraphMessage msg(c, r, k);
          // LOG_DEBUG("cost_cr c: %d, r: %d, k: %d \n", c, (int)r, (int)k);
          CalculateCostAt(
              &msg, agent_trajs, target_s, spatio_temporal_union_plan_input);
        }
      }
    }
    auto serial_end = IflyTime::Now_us();
    total_serial_time += (serial_end - serial_start);
    #endif

    for (int r = next_lowest_row; r <= next_highest_row; ++r) {
      for (int k = next_lowest_l; k <= next_highest_l; ++k) {
        const auto& cost_cr = cost_table_[c][r][k];
        if ((cost_cr.index_t() <= consider_dynamic_obs_time_thd_index && cost_cr.total_cost() < kObstacleCollisionDistanceCost) ||
            (cost_cr.index_t() > consider_dynamic_obs_time_thd_index && !std::isinf(cost_cr.total_cost()))) {
          int h_r = 0;
          int l_r = 0;
          int h_l = 0;
          int l_l = 0;
          GetRowRange(cost_cr, &h_r, &l_r, init_v, v_cruise);
          GetColumnRange(cost_cr, &h_l, &l_l);
          highest_row = std::max(highest_row, h_r);
          lowest_row = std::min(lowest_row, l_r);
          highest_l = std::max(highest_l, h_l);
          lowest_l = std::min(lowest_l, l_l);
        }
      }
    }

    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
    next_highest_l = highest_l;
    next_lowest_l = lowest_l;
  }

  #ifdef VALIDATE_PARALLEL
  // 执行串行计算
  std::vector<std::vector<std::vector<SLTGraphPoint>>> parallel_result = cost_table_;
  cost_table_ = serial_cost_table_;  // 恢复原始状态
  next_highest_row = 0;
  next_lowest_row = 0;
  next_highest_l = 0;
  next_lowest_l = 0;
  // 重新串行计算
  for (int c = 0; c < cost_table_.size(); ++c) {
    // LOG_DEBUG(
    //     "CalculateTotalCost c: %d \n, next_lowest_row: %d \n, next_highest_row : %d \n", c, next_lowest_row, next_highest_row);
    int highest_row = 0;
    int lowest_row = cost_table_.back().size() - 1;

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;

    auto time_start = IflyTime::Now_us();
    if (count > 0) {
      for (int r = next_lowest_row; r <= next_highest_row; ++r) {
        for (int k = next_lowest_l; k <= next_highest_l; ++k) {

          // auto msg = std::make_shared<SLTGraphMessage>(c, r, k);
          SLTGraphMessage msg(c, r, k);
          // LOG_DEBUG("cost_cr c: %d, r: %d, k: %d \n", c, (int)r, (int)k);
          CalculateCostAt(
              &msg, agent_trajs, target_s, spatio_temporal_union_plan_input);
        }
      }
    }

    for (int r = next_lowest_row; r <= next_highest_row; ++r) {
      for (int k = next_lowest_l; k <= next_highest_l; ++k) {
        const auto& cost_cr = cost_table_[c][r][k];
        if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
          int h_r = 0;
          int l_r = 0;
          int h_l = 0;
          int l_l = 0;
          GetRowRange(cost_cr, &h_r, &l_r, init_v);
          GetColumnRange(cost_cr, &h_l, &l_l);
          highest_row = std::max(highest_row, h_r);
          lowest_row = std::min(lowest_row, l_r);
          highest_l = std::max(highest_l, h_l);
          lowest_l = std::min(lowest_l, l_l);
        }
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
    next_highest_l = highest_l;
    next_lowest_l = lowest_l;
  }
  // 结果比对
  bool is_consistent = true;
  for (int c = 0; c < cost_table_.size(); ++c) {
    for (size_t r = 0; r < cost_table_[c].size(); ++r) {
      for (size_t k = 0; k < cost_table_[c][r].size(); ++k) {
        if (std::abs(parallel_result[c][r][k].total_cost() -
                    cost_table_[c][r][k].total_cost()) > 1e-6) {
          LOG_ERROR("Validation failed at c=%d r=%zu k=%zu", c, r, k);
          is_consistent = false;
        }
      }
    }
  }
  LOG_DEBUG("Result consistency: %s \n", is_consistent ? "PASS" : "FAIL");
  #endif

  #if defined(OPENMP_DEBUG) && defined(SERIAL_MODE)
  LOG_DEBUG("[Performance] Parallel: %.3f ms, Serial: %.3f ms, Speedup: %.2fx",
           total_parallel_time/1000.0,
           total_serial_time/1000.0,
           total_serial_time/total_parallel_time);
  #endif

  return true;
}

void SpatioTemporalUnionDp::GetRowRange(const SLTGraphPoint& point,
                                       int* next_highest_row,
                                       int* next_lowest_row,
                                       const double &init_v,
                                       const double &v_cruise) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_v;
  } else {
    v0 = point.GetOptimalSpeed();
  }

  double acc_ve = v0;
  double acc_t = 0.0;
  const auto max_s_size = dimension_s_ - 1;
  double s_upper_bound = v_cruise * unit_t_;
  if (v0 < v_cruise) {
    acc_ve = v0 + max_acceleration_ * unit_t_;
    acc_t = unit_t_;
    if (acc_ve > v_cruise) {
      acc_ve = std::min(acc_ve, v_cruise);
      acc_t = std::fabs(v_cruise - v0) / max_acceleration_;
    }
    s_upper_bound = point.point().s() + v0 * acc_t +
                    acc_coeff * max_acceleration_ * acc_t * acc_t + v_cruise * (unit_t_ - acc_t);
  } else {
    s_upper_bound = point.point().s() + v0 * unit_t_;
  }

  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  double dece_t = std::fabs(v0 / max_deceleration_);
  double dece_s = dece_t > unit_t_ ? v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared_
                                   : -1.0 * acc_coeff * max_deceleration_ * dece_t * dece_t;
  const double s_lower_bound =
      std::fmax(0.0, dece_s) + point.point().s();
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

void SpatioTemporalUnionDp::GetColumnRange(const SLTGraphPoint& point,
                                       int* next_highest_l,
                                       int* next_lowest_l) {
  double lateral_v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.25;

  const auto max_l_size = dimension_l_ - 1;
  const double l_upper_bound = acc_coeff * kLateralMaxAcceleration * t_squared_ +
                               point.point().l();
  const auto next_highest_itr =
      std::lower_bound(lateral_distance_by_index_.begin(),
                       lateral_distance_by_index_.end(), l_upper_bound);
  if (next_highest_itr == lateral_distance_by_index_.end()) {
    *next_highest_l = max_l_size;
  } else {
    *next_highest_l =
        std::distance(lateral_distance_by_index_.begin(), next_highest_itr);
  }

  const double l_lower_bound = acc_coeff * kLateralMaxDeceleration * t_squared_ +
      point.point().l();
  const auto next_lowest_itr =
      std::lower_bound(lateral_distance_by_index_.begin(),
                       lateral_distance_by_index_.end(), l_lower_bound);
  if (next_lowest_itr == lateral_distance_by_index_.end()) {
    *next_lowest_l = max_l_size;
  } else {
    *next_lowest_l =
        std::distance(lateral_distance_by_index_.begin(), next_lowest_itr);
  }
}

void SpatioTemporalUnionDp::CalculateCostAt(
    const SLTGraphMessage* msg,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const double &target_s,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  const uint32_t k = msg->k;
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();
  double ego_v = ego_init_state.v0();
  auto& cost_cr = cost_table_[c][r][k];

  // 依靠target_s 进行纵向剪枝
  if (cost_cr.point().s() > target_s) {
    return;
  }
  // cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  const auto& cost_init = cost_table_[0][0][0];
  if (c == 0) {
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(ego_init_state.v0());
    cost_cr.SetAcc(ego_init_state.a());
    return;
  }

  const double speed_limit = std::max(speed_limit_by_index_[r], 1e-2);
  const double cruise_speed = std::max(ego_init_state.v_cruise(), 1e-2);

  // The mininal s to model as constant acceleration formula
  // default: 0.2 * 5 = 1.0 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  if (c == 1) {
    const double delta_s = cost_cr.point().s() - cost_init.point().s();
    const double delta_l = cost_cr.point().l() - cost_init.point().l();
    double v0 = cost_init.GetOptimalSpeed();
    if (std::fabs(v0) < 2.0e-1) {
      v0 = 0.0;
    }
    const double acc =
        2 * (delta_s * inv_unit_t_ - v0) * inv_unit_t_;
    if (acc < kDefaultMaxDeceleration || acc > kDefaultMaxAcceleration) {
      return;
    }
    const double lateral_acc = 4.0 * delta_l * inv_unit_t_ * inv_unit_t_;
    if (lateral_acc < kLateralMaxDeceleration || lateral_acc > kLateralMaxAcceleration) {
      return;
    }
    double s_upper_bound = v0 * unit_t_ +
                                0.5 * kDefaultMaxAcceleration * t_squared_ +
                                cost_init.point().s();
    double s_lower_bound = v0 * unit_t_ +
                                0.5 * kDefaultMaxDeceleration * t_squared_ +
                                cost_init.point().s();
    double t_decel = - v0 / kDefaultMaxDeceleration;
    if(t_decel < unit_t_) {
      s_lower_bound = 0.5 * v0 * t_decel +
                                cost_init.point().s();
    }

    if (cost_cr.point().s() < s_lower_bound || cost_cr.point().s() > s_upper_bound) {
      return;
    }
    // if (init_state.v() + acc * unit_t_ < -kDoubleEpsilon &&
    //     delta_s > min_s_consider_speed) {
    //   return;
    // }

    // 生成L方向三次多项式
    std::array<double, 2> later_start_state{cost_init.point().l(), 0.0};
    std::array<double, 2> later_end_state{cost_cr.point().l(), 0.0};
    double remain_time = cost_cr.point().t() - cost_init.point().t();
    CubicPolynomialCurve1d lateral_cubic_curve_c1(
        later_start_state, later_end_state, remain_time);

    // auto c1 = IflyTime::Now_us();
    // frenet系下检测是否发生碰撞
    if (CheckOverlapOnDpSltGraph(cost_cr, agent_trajs)) {
      return;
    }
    auto overlap = IflyTime::Now_us();
    // LOG_DEBUG("c1 CheckOverlapOnDpSltGraph: %.8f\n", overlap - c1);

    // 计算相邻两节点之间路径的cost
    double path_cost_c1 =
        CalculatePathCost(cost_init, cost_cr, lateral_cubic_curve_c1, acc, spatio_temporal_union_plan_input);
    cost_cr.SetPathCost(path_cost_c1);
    auto pathcost = IflyTime::Now_us();
    // LOG_DEBUG("c1 CalculatePathCost: %.8f\n", pathcost - overlap);

    // 计算自车与动态障碍物之间的cost
    double dis = kDefaultMaxDisBetweenObs;
    int agent_id = -1;
    double dynamic_obstacle_cost_c1 =
        CalculateDynamicObstacleCost(
            cost_init, cost_cr, lateral_cubic_curve_c1, acc, agent_trajs, spatio_temporal_union_plan_input, &dis, &agent_id);
    auto dynamicobstaclecost = IflyTime::Now_us();
    // LOG_DEBUG("c1 CalculateDynamicObstacleCost: %.8f\n", dynamicobstaclecost - pathcost);
    double long_cost_c1 = CalculateEdgeCostForSecondCol(r, k, speed_limit, cruise_speed, ego_init_state);

    cost_cr.SetTotalCost(
        path_cost_c1 + dynamic_obstacle_cost_c1 +
        cost_init.total_cost() +
        long_cost_c1);
    // auto edgecost = IflyTime::Now_us();
    // LOG_DEBUG("c1 CalculateEdgeCostForSecondCol: %.8f\n", edgecost - dynamicobstaclecost);

    cost_cr.SetDynamicObstacleCost(dynamic_obstacle_cost_c1);
    cost_cr.SetMinObsDistance(dis);
    cost_cr.SetMinDistanceAgentId(agent_id);
    cost_cr.SetPathCost(path_cost_c1);
    cost_cr.SetLongitinalCost(long_cost_c1);
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(std::max(v0 + acc * unit_t_, 0.0));
    cost_cr.SetAcc(acc);
    // LOG_DEBUG("path_cost_c1: %f, dynamic_obstacle_cost_c1: %f, long_cost_c1: %f \n, total_cost: %f \n",
    //     path_cost_c1, dynamic_obstacle_cost_c1, long_cost_c1, cost_cr.total_cost());
    return;
  }

  double pre_lowest_s =
      cost_cr.point().s() -
      std::max(kPlanningUpperSpeedLimit, ego_v) * (1 + kSpeedRangeBuffer) * unit_t_;
  pre_lowest_s = std::max(pre_lowest_s, cost_init.point().s());
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  const uint32_t r_pre_size = r - r_low + 1;
  const auto& pre_col = cost_table_[c - 1];
  double curr_speed_limit = speed_limit;

  if (c == 2) {
    // frenet系下检测当前采样点与障碍物box是否发生碰撞
    if (CheckOverlapOnDpSltGraph(cost_cr, agent_trajs)) {
      return;
    }

    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;
      for (int later_j = 0; later_j < dimension_l_; ++later_j) {
        if ((pre_col[r_pre][later_j].total_cost() > kObstacleCollisionDistanceCost) ||
            pre_col[r_pre][later_j].pre_point() == nullptr) {
          continue;
        }

        const double delta_s =
            cost_cr.point().s() - pre_col[r_pre][later_j].point().s();
        const double delta_l = cost_cr.point().l() - pre_col[r_pre][later_j].point().l();
        const double curr_a =
            2 *
            (delta_s * inv_unit_t_ -
            pre_col[r_pre][later_j].GetOptimalSpeed()) * inv_unit_t_;
        if (curr_a < kDefaultMaxDeceleration || curr_a > kDefaultMaxAcceleration) {
          continue;
        }

        const double lateral_acc = 4.0 * delta_l * inv_unit_t_ * inv_unit_t_;
        if (lateral_acc < kLateralMaxDeceleration || lateral_acc > kLateralMaxAcceleration) {
          continue;
        }
        double s_upper_bound = pre_col[r_pre][later_j].GetOptimalSpeed() * unit_t_ +
                                    0.5 * kDefaultMaxAcceleration * t_squared_ +
                                    pre_col[r_pre][later_j].point().s();
        double s_lower_bound = pre_col[r_pre][later_j].GetOptimalSpeed() * unit_t_ +
                                    0.5 * kDefaultMaxDeceleration * t_squared_ +
                                    pre_col[r_pre][later_j].point().s();
        double t_decel = - pre_col[r_pre][later_j].GetOptimalSpeed() / kDefaultMaxDeceleration;
        if(t_decel < unit_t_) {
          s_lower_bound = 0.5 * pre_col[r_pre][later_j].GetOptimalSpeed() * t_decel +
                                    pre_col[r_pre][later_j].point().s();
        }
        if (cost_cr.point().s() < s_lower_bound || cost_cr.point().s() > s_upper_bound) {
          continue;
        }

        // if (pre_col[r_pre][later_j].GetOptimalSpeed() + curr_a * unit_t_ <
        //         -kDoubleEpsilon && delta_s > min_s_consider_speed) {
        //   continue;
        // }

        const auto& pre_cost = pre_col[r_pre][later_j];
        // 生成L方向三次多项式
        std::array<double, 2> later_start_state{pre_cost.point().l(), 0.0};
        std::array<double, 2> later_end_state{cost_cr.point().l(), 0.0};
        double remain_time = cost_cr.point().t() - pre_cost.point().t();
        CubicPolynomialCurve1d lateral_cubic_curve_c2(
            later_start_state, later_end_state, remain_time);

        // 计算相邻两节点之间路径的cost
        auto overlap_2 = IflyTime::Now_us();
        double path_cost_c2 =
            CalculatePathCost(pre_cost, cost_cr, lateral_cubic_curve_c2, curr_a, spatio_temporal_union_plan_input);
        auto pathcost = IflyTime::Now_us();
        LOG_DEBUG("c2 CalculatePathCost: %f\n", pathcost - overlap_2);

        // 计算自车与动态障碍物之间的cost
        double distance_c2 = kDefaultMaxDisBetweenObs;
        int agent_id_c2;
        double dynamic_obstacle_cost_c2 =
            CalculateDynamicObstacleCost(pre_cost, cost_cr, lateral_cubic_curve_c2, curr_a, agent_trajs, spatio_temporal_union_plan_input, &distance_c2, &agent_id_c2);
        auto dynamicobstaclecost = IflyTime::Now_us();
        // LOG_DEBUG("c2 CalculateDynamicObstacleCost: %.8f\n", dynamicobstaclecost - pathcost);
        curr_speed_limit =
            std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
        curr_speed_limit = std::max(curr_speed_limit, 1e-2);
        double long_cost_c2 = CalculateEdgeCostForThirdCol(
                                r, k, r_pre, later_j, curr_speed_limit, cruise_speed, ego_init_state);
        auto edgecost = IflyTime::Now_us();
        // LOG_DEBUG("c2 CalculateEdgeCostForThirdCol: %.8f\n", edgecost - dynamicobstaclecost);

        const double cost = path_cost_c2 +
                            dynamic_obstacle_cost_c2 +
                            pre_col[r_pre][later_j].total_cost() + long_cost_c2;

        // LOG_DEBUG("pre_cost: c: %d, r : %d, k: %d \n, total_cost: %f \n", c-1, r_pre, later_j, pre_col[r_pre][later_j].total_cost());
        // LOG_DEBUG("path_cost_c2: %f, dynamic_obstacle_cost_c2: %f, long_cost_c2: %f \n, total_cost: %f \n",
        //     path_cost_c2, dynamic_obstacle_cost_c2, long_cost_c2, cost);
        if (cost < cost_cr.total_cost()) {
          cost_cr.SetDynamicObstacleCost(dynamic_obstacle_cost_c2);
          cost_cr.SetMinObsDistance(distance_c2);
          cost_cr.SetMinDistanceAgentId(agent_id_c2);
          cost_cr.SetPathCost(path_cost_c2);
          cost_cr.SetLongitinalCost(long_cost_c2);
          cost_cr.SetTotalCost(cost);
          cost_cr.SetPathCost(path_cost_c2);
          cost_cr.SetPrePoint(pre_col[r_pre][later_j]);
          cost_cr.SetOptimalSpeed(
              std::max(pre_col[r_pre][later_j].GetOptimalSpeed() +
                                  curr_a * unit_t_, 0.0));
          cost_cr.SetAcc(curr_a);
          // LOG_DEBUG("c is %d \n", c);
        }
      }
    }
    return;
  }

  // frenet系下检测当前采样点与障碍物box是否发生碰撞
  // if (CheckOverlapOnDpSltGraph(cost_cr, agent_trajs)) {
  //   return;
  // }
  for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;
    for (int later_j = 0; later_j < dimension_l_; ++later_j) {
      if (std::isinf(pre_col[r_pre][later_j].total_cost()) ||
          pre_col[r_pre][later_j].pre_point() == nullptr) {
        continue;
      }
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const double delta_s =
          cost_cr.point().s() - pre_col[r_pre][later_j].point().s();
      const double delta_l = cost_cr.point().l() - pre_col[r_pre][later_j].point().l();
      const double curr_a =
          2 *
          (delta_s * inv_unit_t_ -
          pre_col[r_pre][later_j].GetOptimalSpeed()) * inv_unit_t_;
      if (curr_a > kDefaultMaxAcceleration || curr_a < kDefaultMaxDeceleration) {
        continue;
      }
      const double lateral_acc = 4.0 * delta_l * inv_unit_t_ * inv_unit_t_;
      if (lateral_acc < kLateralMaxDeceleration || lateral_acc > kLateralMaxAcceleration) {
        continue;
      }
      double s_upper_bound = pre_col[r_pre][later_j].GetOptimalSpeed() * unit_t_ +
                                  0.5 * kDefaultMaxAcceleration * t_squared_ +
                                  pre_col[r_pre][later_j].point().s();
      double s_lower_bound = pre_col[r_pre][later_j].GetOptimalSpeed() * unit_t_ +
                                  0.5 * kDefaultMaxDeceleration * t_squared_ +
                                  pre_col[r_pre][later_j].point().s();
      double t_decel = - pre_col[r_pre][later_j].GetOptimalSpeed() / kDefaultMaxDeceleration;
      if(t_decel < unit_t_) {
        s_lower_bound = 0.5 * pre_col[r_pre][later_j].GetOptimalSpeed() * t_decel +
                                  pre_col[r_pre][later_j].point().s();
      }
      if (cost_cr.point().s() < s_lower_bound || cost_cr.point().s() > s_upper_bound) {
        continue;
      }

      // if (pre_col[r_pre][later_j].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
      //     delta_s > min_s_consider_speed) {
      //   continue;
      // }

      const auto& pre_cost = pre_col[r_pre][later_j];
      // 生成L方向三次多项式
      std::array<double, 2> later_start_state{pre_cost.point().l(), 0.0};
      std::array<double, 2> later_end_state{cost_cr.point().l(), 0.0};
      double remain_time = cost_cr.point().t() - pre_cost.point().t();
      CubicPolynomialCurve1d lateral_cubic_curve(
          later_start_state, later_end_state, remain_time);

      // 计算相邻两节点之间路径的cost
      double path_cost_c =
          CalculatePathCost(pre_cost, cost_cr, lateral_cubic_curve, curr_a, spatio_temporal_union_plan_input);
      auto pathcost = IflyTime::Now_us();
      // LOG_DEBUG("c3 CalculatePathCost: %.8f\n", pathcost - overlap);

      // 计算自车与动态障碍物之间的cost
      double distance_c3 = kDefaultMaxDisBetweenObs;
      int agent_id_c3;
      double dynamic_obstacle_cost = 0.0;
      if (cost_cr.point().t() <= kConsiderDynamicObstacleCostTimeLength) {
        dynamic_obstacle_cost =
            CalculateDynamicObstacleCost(
                pre_cost, cost_cr, lateral_cubic_curve, curr_a, agent_trajs, spatio_temporal_union_plan_input, &distance_c3, &agent_id_c3);
        auto dynamicobstaclecost = IflyTime::Now_us();
        // LOG_DEBUG("c3 CalculateDynamicObstacleCost: %.8f\n", dynamicobstaclecost - pathcost);
      }

      uint32_t r_prepre = pre_col[r_pre][later_j].pre_point()->index_s();
      const auto& prepre_graph_point = pre_col[r_pre][later_j].pre_point();
      if (prepre_graph_point->total_cost() > kObstacleCollisionDistanceCost) {
        continue;
      }

      if (!prepre_graph_point->pre_point()) {
        continue;
      }
      const auto& triple_pre_point = prepre_graph_point->pre_point();
      const auto& prepre_point = prepre_graph_point;
      const auto& pre_point = pre_col[r_pre][later_j];
      const auto& curr_point = cost_cr;
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      curr_speed_limit = std::max(curr_speed_limit, 1e-2);
      double long_cost = CalculateEdgeCost(*triple_pre_point, *prepre_point, pre_point,
                                      curr_point, curr_speed_limit, cruise_speed);
      double cost = path_cost_c + dynamic_obstacle_cost +
                    pre_col[r_pre][later_j].total_cost() + long_cost;

      // auto edgecost = IflyTime::Now_us();
      // LOG_DEBUG("c3 CalculateEdgeCost: %.8f\n", edgecost - dynamicobstaclecost);
      // LOG_DEBUG("pre_cost: c: %d, r : %d, k: %d \n, total_cost: %f \n", c-1, r_pre, later_j, pre_col[r_pre][later_j].total_cost());
      // LOG_DEBUG("path_cost_c: %f, dynamic_obstacle_cost: %f, long_cost: %f \n, total_cost: %f \n",
      //     path_cost_c, dynamic_obstacle_cost, long_cost, cost);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetDynamicObstacleCost(dynamic_obstacle_cost);
        cost_cr.SetMinObsDistance(distance_c3);
        cost_cr.SetMinDistanceAgentId(agent_id_c3);
        cost_cr.SetPathCost(path_cost_c);
        cost_cr.SetLongitinalCost(long_cost);
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPathCost(path_cost_c);
        cost_cr.SetPrePoint(pre_col[r_pre][later_j]);
        cost_cr.SetOptimalSpeed(
            std::max(pre_col[r_pre][later_j].GetOptimalSpeed() +
                                curr_a * unit_t_, 0.0));
        cost_cr.SetAcc(curr_a);
        // LOG_DEBUG("c is %d \n", c);
      }
    }
  }

  return;
}

bool SpatioTemporalUnionDp::RetrieveSpeedProfile(
    TrajectoryPoints &traj_points,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  double min_cost = std::numeric_limits<double>::infinity();
  const SLTGraphPoint* best_end_point = nullptr;
  int c = dimension_t_ - 1;
  double s_end = total_length_s_;
  for (const auto& cur_points_set : cost_table_.back()) {
    // std::vector<std::vector<SLTGraphPoint>> cur_points_set = iter;
    for (const auto& cur_point : cur_points_set) {
      if (!std::isinf(cur_point.total_cost()) &&
          cur_point.total_cost() < min_cost) {
        best_end_point = &cur_point;
        min_cost = cur_point.total_cost();
        s_end = cur_point.index_s();
      }
    }
  }

  // for (const auto& row : cost_table_) {
  //   const std::vector<SLTGraphPoint>& cur_points = row[s_end];
  //   for (const auto &cur_point : cur_points) {
  //     if (!std::isinf(cur_point.total_cost()) &&
  //         cur_point.total_cost() < min_cost) {
  //       best_end_point = &cur_point;
  //       min_cost = cur_point.total_cost();
  //     }
  //   }
  // }

  if (best_end_point == nullptr) {
    LOG_DEBUG("Fail to find the best feasible trajectory: best_end_point = nullptr!");
    return false;
  }

  std::vector<SpeedInfo> speed_profile;
  const SLTGraphPoint* cur_point = best_end_point;
  pnc::mathlib::spline s_t_spline_;
  pnc::mathlib::spline s_l_spline_;
  std::vector<double> s_vec;
  std::vector<double> t_vec;
  std::vector<double> l_vec;

  while (cur_point != nullptr) {
    LOG_DEBUG(
        "[RetrieveSpeedProfile] s: %f, l: %f, t: %f, v: %f \n",
        cur_point->point().s(),
        cur_point->point().l(),
        cur_point->point().t(),
        cur_point->GetOptimalSpeed());
#ifdef X86
    ILOG_INFO << "\n s = " << cur_point->point().s()
              << "  l = " << cur_point->point().l()
              << "  t = " << cur_point->point().t()
              << "  t_index = " << cur_point->index_t()
              << "  s_index =" << cur_point->index_s()
              << "  l_index = " << cur_point->index_l()
              << "  obstacle_cost = " << cur_point->obstacle_cost()
              << "  path_cost = " << cur_point->path_cost()
              << "  long_cost = " << cur_point->longitinal_cost()
              << "  obstacle_min_distance = " << cur_point->min_obs_distance()
              << "  obstacle_id = " << cur_point->min_distance_agent_id()
              << "  total_cost = " << cur_point->total_cost()
              << " \n";
#endif
    SpeedInfo speed_point;
    speed_point.s = cur_point->point().s();
    speed_point.l = cur_point->point().l();
    speed_point.v = cur_point->GetOptimalSpeed();
    speed_point.t = cur_point->point().t();
    Point2D reference_frenet_point(
        cur_point->point().s(), cur_point->point().l());
    Point2D global_reference_point;
    current_lane_coord_.SLToXY(reference_frenet_point,
                               global_reference_point);

    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
    s_vec.emplace_back(speed_point.s);
    l_vec.emplace_back(speed_point.l);
    t_vec.emplace_back(speed_point.t);
  }
  std::reverse(speed_profile.begin(), speed_profile.end());
  std::reverse(s_vec.begin(), s_vec.end());
  std::reverse(l_vec.begin(), l_vec.end());
  std::reverse(t_vec.begin(), t_vec.end());
  s_t_spline_.set_points(t_vec, s_vec);
  // s_t_spline_.set_boundary(pnc::mathlib::spline::first_deriv, speed_profile[0].v, pnc::mathlib::spline::first_deriv, speed_profile.back().v);
  s_vec_is_monotonic_increasing_ = false;
  if (JudgeSVecisMonotonicIncreasing(s_vec)) {
    s_l_spline_.set_points(s_vec, l_vec);
    s_vec_is_monotonic_increasing_ = true;
  }

  const double init_delta_s = speed_profile.front().s - cost_table_[0][0][0].point().s();
  const double total_s = speed_profile.back().s - speed_profile.front().s;
  if (speed_profile.front().t > kEpsilon ||
      init_delta_s > kEpsilon) {
    LOG_DEBUG("Fail to retrieve speed profile : init_delta_s > kEpsilon !");
    return false;
  }

  dp_speed_profile_.clear();

  if(s_vec_is_monotonic_increasing_ && total_s > kDefaultlLinearInterpolationDistance) {
    for (int i = 0; i < speed_profile.size() - 1; ++i) {
      for (int j = 0; j < 5; ++j) {
        SpeedInfo point;
        double relative_time = i + j * kDeltaTime;
        point.t = relative_time;
        point.s = s_t_spline_(relative_time);
        point.l = s_l_spline_(point.s);
        point.v = s_t_spline_.deriv(1, relative_time);
        point.a = s_t_spline_.deriv(2, relative_time);
        point.da = 0.0;
        dp_speed_profile_.emplace_back(point);
      }
    }

    if (!speed_profile.empty()) {
      dp_speed_profile_.push_back(speed_profile.back());
    }
  } else {
    double avg_v = std::fabs((speed_profile.back().s - speed_profile.front().s)) / kDefaultTrajsTimeLength;
    for (size_t i = 0; i < traj_points.size(); ++i) {
      SpeedInfo point;
      double relative_time = i * kDeltaTime;
      point.t = relative_time;
      point.s =
          planning_math::LerpWithLimit(speed_profile.front().s, speed_profile.front().t, speed_profile.back().s, speed_profile.back().t, point.t);
      point.l =
          planning_math::LerpWithLimit(speed_profile.front().l, speed_profile.front().s, speed_profile.back().l, speed_profile.back().s, point.s);
      point.v = avg_v;
      point.a = 0.0;
      point.da = 0.0;
      dp_speed_profile_.emplace_back(point);
    }
  }

  std::vector<planning_math::PathPoint> path_points;
  path_points.reserve(traj_points.size());
  TrajectoryPoint pre_trajectory_point;
  trajectory_points_.Clear();
  last_planning_result_coord_ = nullptr;
  for (size_t j = 0; j < traj_points.size(); ++j) {
    TrajectoryPoint trajectory_point;
    if (j == 0) {
      Point2D reference_frenet_point(
          dp_speed_profile_[j].s, dp_speed_profile_[j].l);
      Point2D global_reference_point;
      current_lane_coord_.SLToXY(reference_frenet_point,
                                global_reference_point);
      trajectory_point.s = dp_speed_profile_[0].s;
      trajectory_point.l = dp_speed_profile_[0].l;
      trajectory_point.t = dp_speed_profile_[0].t;
      trajectory_point.v = dp_speed_profile_[0].v;
      trajectory_point.x = global_reference_point.x;
      trajectory_point.y = global_reference_point.y;
      trajectory_point.heading_angle =
          spatio_temporal_union_plan_input.init_state().heading_angle();
      traj_points[j] = trajectory_point;
      pre_trajectory_point = trajectory_point;

      auto iter = trajectory_points_.add_point();
      iter->set_s(trajectory_point.s);
      iter->set_l(trajectory_point.l);
      iter->set_t(trajectory_point.t);
      iter->set_v(trajectory_point.v);
      iter->set_x(trajectory_point.x);
      iter->set_y(trajectory_point.y);
    } else if (j + 1 < dp_speed_profile_.size()) {
      const double v = (dp_speed_profile_[j + 1].s - dp_speed_profile_[j].s) /
                       (dp_speed_profile_[j + 1].t - dp_speed_profile_[j].t + 1e-3);
      dp_speed_profile_[j].v = v;
      Point2D reference_frenet_point(
          dp_speed_profile_[j].s, dp_speed_profile_[j].l);
      Point2D global_reference_point;
      current_lane_coord_.SLToXY(reference_frenet_point,
                                global_reference_point);
      trajectory_point.s = dp_speed_profile_[j].s;
      trajectory_point.l = dp_speed_profile_[j].l;
      trajectory_point.t = dp_speed_profile_[j].t;
      trajectory_point.v = dp_speed_profile_[j].v;
      trajectory_point.x = global_reference_point.x;
      trajectory_point.y = global_reference_point.y;
      trajectory_point.heading_angle =
          std::atan2(trajectory_point.y - pre_trajectory_point.y,
                              trajectory_point.x - pre_trajectory_point.x);
      traj_points[j] = trajectory_point;
      pre_trajectory_point = trajectory_point;

      auto iter = trajectory_points_.add_point();
      iter->set_s(trajectory_point.s);
      iter->set_l(trajectory_point.l);
      iter->set_t(trajectory_point.t);
      iter->set_v(trajectory_point.v);
      iter->set_x(trajectory_point.x);
      iter->set_y(trajectory_point.y);
    } else {
      double cur_s = dp_speed_profile_[j].s;
      Point2D reference_frenet_point(
          cur_s, dp_speed_profile_[j].l);
      Point2D global_reference_point;
      current_lane_coord_.SLToXY(reference_frenet_point,
                                global_reference_point);
      trajectory_point.s = cur_s;
      trajectory_point.l = dp_speed_profile_[j].l;
      trajectory_point.t = traj_points[j].t;
      trajectory_point.v = dp_speed_profile_[j].v;
      trajectory_point.x = global_reference_point.x;
      trajectory_point.y = global_reference_point.y;
      trajectory_point.heading_angle = pre_trajectory_point.heading_angle;
      traj_points[j] = trajectory_point;
      pre_trajectory_point = trajectory_point;

      auto iter = trajectory_points_.add_point();
      iter->set_s(trajectory_point.s);
      iter->set_l(trajectory_point.l);
      iter->set_t(trajectory_point.t);
      iter->set_v(trajectory_point.v);
      iter->set_x(trajectory_point.x);
      iter->set_y(trajectory_point.y);
    }
    auto pt =
        planning_math::PathPoint(trajectory_point.x, trajectory_point.y);
    if (!path_points.empty()) {
      auto& last_pt = path_points.back();
      if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length() < 1e-2) {
        continue;
      }
    }

    path_points.emplace_back(pt);
  }

  if (path_points.size() < 3) {
    last_planning_result_coord_ = nullptr;
  } else {
    last_planning_result_coord_ =
        std::make_shared<planning_math::KDPath>(std::move(path_points));
  }
  return true;
}


double SpatioTemporalUnionDp::CalculateStitchingCost(
    const Point2D &current, const double &current_time,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  double stitching_cost = 0.0;
  const auto& stitching_cost_params = spatio_temporal_union_plan_input.stitching_cost_params();
  Point2D current_point;
  Point2D current_frenet_point;
  if (!current_lane_coord_.SLToXY(current, current_point)) {
    // LOG_DEBUG("CalculateStitchingCost find sl to current lane failed!");
    return stitching_cost;
  }

  if (last_planning_result_coord_) {
    if (!last_planning_result_coord_->XYToSL(current_point, current_frenet_point)) {
      // LOG_DEBUG("CalculateStitchingCost find xy to last_planning_result_coord failed!");
      return stitching_cost;
    }

    stitching_cost = std::fabs(current_frenet_point.y) * stitching_cost_params.path_l_stitching_cost_param() *
        (1.0 - (current_time * stitching_cost_params.stitching_cost_time_decay_factor()) * (current_time * stitching_cost_params.stitching_cost_time_decay_factor()));
  } else {
    return stitching_cost;
  }

  // double square_adjacent_frame_distance = 0.0;
  // if (trajectory_points_.point_size() > 0) {
  //   int index = static_cast<int>(std::ceil(current_time / kDeltaTime));
  //   if (index < trajectory_points_.point_size()) {
  //     square_adjacent_frame_distance =
  //         (trajectory_points_.point(index).x() - current_point.x) * (trajectory_points_.point(index).y() - current_point.y);

  //     // if (last_planning_result_coord_) {
  //     //   if (!last_planning_result_coord_->XYToSL(current_point, current_last_planning_point)) {
  //     //     // LOG_DEBUG("CalculateStitchingCost find xy to last planning result failed!");
  //     //     return stitching_cost;
  //     //   }
  //     // } else {
  //     //   return stitching_cost;
  //     // }
  //   }


  //   stitching_cost = square_adjacent_frame_distance * stitching_cost_params.path_l_stitching_cost_param() *
  //       (1.0 - (current_time * stitching_cost_params.stitching_cost_time_decay_factor()) * (current_time * stitching_cost_params.stitching_cost_time_decay_factor()));
  // }

  return stitching_cost;
}

double SpatioTemporalUnionDp::CalculateEdgeCost(
    const SLTGraphPoint& first, const SLTGraphPoint& second, const SLTGraphPoint& third,
    const SLTGraphPoint& forth, const double &speed_limit, const double &cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, inv_speed_limit_table_[forth.index_s()], cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double SpatioTemporalUnionDp::CalculateEdgeCostForSecondCol(
    const uint32_t row, const uint32_t col, const double &speed_limit, const double &cruise_speed,
    const planning::common::EgoInitInfo &init_state) {
  double init_speed = init_state.v0();
  double init_acc = init_state.a();
  const auto& pre_point = cost_table_[0][0][0];
  const auto& curr_point = cost_table_[1][row][col];
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit, inv_speed_limit_table_[row],
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

double SpatioTemporalUnionDp::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t curr_col,
    const uint32_t pre_row, const uint32_t pre_col,
    const double &speed_limit,
    const double &cruise_speed,
    const planning::common::EgoInitInfo &init_state) {
  double init_speed = init_state.v0();
  const auto& first = cost_table_[0][0][0];
  const auto& second = cost_table_[1][pre_row][pre_col];
  const auto& third = cost_table_[2][curr_row][curr_col];
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, inv_speed_limit_table_[curr_row], cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

double SpatioTemporalUnionDp::CalculatePathCost(
    const SLTGraphPoint &start, const SLTGraphPoint &end,
    const CubicPolynomialCurve1d &lateral_curve,
    const double &acc,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  double path_cost = 0.0;
  double v0 = start.GetOptimalSpeed();

  std::array<double, 4> lateral_curve_coef;
  lateral_curve_coef[0] = lateral_curve.Coef(0);
  lateral_curve_coef[1] = lateral_curve.Coef(1);
  lateral_curve_coef[2] = lateral_curve.Coef(2);
  lateral_curve_coef[3] = lateral_curve.Coef(3);

  const auto& lat_path_weight_params = spatio_temporal_union_plan_input.lat_path_weight_params();
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();

  // std::function<double(const double)> quasi_softmax = [this](const double x) {
  //   return (b_ + std::exp(-k_ * (x - l0_))) / (1.0 + std::exp(-k_ * (x - l0_)));
  // };

  Point2D current_point;
  double current_t = start.point().t();
  for (double curve_t = kPathCostComputeSampleTime; curve_t <= (end.point().t() - start.point().t());
       curve_t += kPathCostComputeSampleTime) {
    current_t = start.point().t() + curve_t;
    current_point.x = v0 * curve_t + 0.5 * acc * curve_t * curve_t + start.point().s();
    current_point.y = lateral_curve.Evaluate(0, curve_t);
    // Vec2d curve_point(cur_s, cur_l);
    path_cost += current_point.y * current_point.y * lat_path_weight_params.path_l_cost();
    double ds = v0 + acc * curve_t;
    ds = std::max(ds, 1e-6);
    double dl = lateral_curve.Evaluate(1, curve_t);
    double dl_ds = dl / ds;
    path_cost += dl_ds * dl_ds * lat_path_weight_params.path_dl_cost();

    const double ddl =
        ComputeSecondDerivative(lateral_curve_coef, acc, v0, curve_t);
    path_cost += ddl * ddl * lat_path_weight_params.path_ddl_cost();
    // path_cost += CalculateStitchingCost(current_point, current_t, spatio_temporal_union_plan_input);
  }
  path_cost *= kPathCostComputeSampleTime;

  // if (end.point().t() == total_length_t_ || end.point().s() == total_length_s_) {
  //   double remain_t = end.point().t() - start.point().t();
  //   const double end_l = lateral_curve.Evaluate(0, remain_t);
  //   path_cost +=
  //       std::sqrt(fabs(end_l)) * lat_path_weight_params.path_end_l_cost();
  // }

  return path_cost;
}

double SpatioTemporalUnionDp::CalculateDynamicObstacleCost(
    const SLTGraphPoint &pre_point, const SLTGraphPoint &cur_point,
    const CubicPolynomialCurve1d &lateral_curve,
    const double &acc,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input,
    double* distance_to_point, int* agent_id) {
  double obstacle_cost = 0.0;
  double v0 = pre_point.GetOptimalSpeed();
  const double total_t = cur_point.point().t() - pre_point.point().t();
  if (agent_trajs.empty()) {
    return obstacle_cost;
  }
  // if (pre_point.point().t() > kDefaultConsiderObstacleTajsTime) {
  //   return obstacle_cost;
  // }
  double min_dis = std::numeric_limits<double>::max();
  double cur_s = 0.0;
  double cur_l = 0.0;
  Vec2d cur_ego_point(cur_s, cur_l);
  double ds = 1e-6;
  double dl = 1e-6;
  double dl_ds = 0.0;
  double ego_heading_angle = 0.0;
  // 获取sl坐标系下自车box的顶点坐标集合
  double cos_theta = 0.0;
  double sin_theta = 0.0;
  std::array<planning_math::Vec2d, 8> ego_box_vertices;
  // std::vector<planning_math::Vec2d> ego_box_vertices;
  double distance = std::numeric_limits<double>::max();
  double dis_to_agent_max_box = 0.0;
  int index = 0;
  double longit_dis_to_ego = 20.0;
  double lateral_dis_to_ego = 10.0;

  auto t_begin = IflyTime::Now_us();
  int count = 1;
  double time_gap = kDynamicObstacleCostSampleTime;
  if (pre_point.point().t() > kDynamicObsConsiderTime) {
    time_gap = kDynamicObstacleCostLaterSampleTime;
  }

  for (double cur_t = kDeltaTime; cur_t <= total_t;
       cur_t += time_gap) {
    // double cur_s = longit_curve.Evaluate(0, cur_t);
    cur_s = v0 * cur_t + 0.5 * acc * cur_t * cur_t + pre_point.point().s();
    cur_l = lateral_curve.Evaluate(0, cur_t);
    cur_ego_point.set_x(cur_s);
    cur_ego_point.set_y(cur_l);
    ds = v0 + acc * cur_t;
    ds = std::max(ds, 1e-6);
    dl = lateral_curve.Evaluate(1, cur_t);
    dl_ds = dl / ds;
    ego_heading_angle = NormalizeAngle(dl_ds);
    // 获取sl坐标系下自车box的顶点坐标集合
    cos_theta = cos(ego_heading_angle);
    sin_theta = sin(ego_heading_angle);

    GetVehicleBoxSLVertices(
        cur_ego_point, cos_theta, sin_theta, ego_box_vertices);
    // 构建自车box
    AABox2d cur_ego_box(ego_box_vertices, kDefaultValidBoxNums);
    for (const auto& agent_traj : agent_trajs) {
      if (agent_traj.agent_id == -1) {
        continue;
      }
      if (agent_traj.agent_boxs_set.empty()) {
        continue;
      }

      dis_to_agent_max_box = agent_traj.max_agent_box.DistanceTo(cur_ego_box);
      if(dis_to_agent_max_box >
          spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params().obstacle_longit_risk_distance()) {
        continue;
      }

      index = pre_point.index_t() * kDefaultTrajsPointNums + count;
      auto iter = agent_traj.agent_boxs_set.find(index);
      if (iter != agent_traj.agent_boxs_set.end()) {
        // AABox2d cur_agent_box;
        const auto& cur_agent_box = iter->second;
        distance = cur_agent_box.DistanceTo(cur_ego_box);
        longit_dis_to_ego = cur_agent_box.LongitDistanceTo(cur_ego_box);
        lateral_dis_to_ego = cur_agent_box.LateralDistanceTo(cur_ego_box);
        // ILOG_INFO << " index" << index
        //           << " cur_agent_box.max_x" << cur_agent_box.max_x()
        //           << " cur_agent_box.max_y" << cur_agent_box.max_y()
        //           << " cur_agent_box.min_x" << cur_agent_box.min_x()
        //           << " cur_agent_box.min_y" << cur_agent_box.min_y()
        //           << " cur_t" << cur_t + pre_point.point().t();
        if (distance < min_dis) {
          min_dis = distance;
          *distance_to_point = distance;
          *agent_id = agent_traj.agent_id;
        }

        if (distance < kCollisionBetweenObstacleDistanceThreshold) {
          return kObstacleCollisionDistanceCost;
        }

        // 过滤范围外障碍物
        if (longit_dis_to_ego > spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params().obstacle_longit_risk_distance() ||
            lateral_dis_to_ego > spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params().obstacle_lateral_risk_distance()) {
          continue;
        }

        // 判断自车box与障碍物agent_box的cost
        // 计算障碍物与自车之间的纵向cost
        obstacle_cost += GetLongitCostBetweenObsBoxes(
            longit_dis_to_ego, spatio_temporal_union_plan_input);

        if (cur_point.point().t() <= kDefaultConsiderObstacleTajsTime) {
          if (agent_traj.agent_type == 6 || agent_traj.agent_type == 7 ||
              agent_traj.agent_type == 8) {
            obstacle_cost += kSpecialObstacleDistanceCost * GetLateralCostBetweenObsBoxes(
                lateral_dis_to_ego, spatio_temporal_union_plan_input);
          } else {
            obstacle_cost += spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params().default_obstacle_cost_weight() *
                GetLateralCostBetweenObsBoxes(lateral_dis_to_ego, spatio_temporal_union_plan_input);
          }
        }
      }
    }
    count += kDefaultIndexInterval;
  }
  auto t_end = IflyTime::Now_us();
  // LOG_DEBUG("CalculateDynamicObstacleCost: %.8f\n", t_end - t_begin);

  obstacle_cost *=
      (time_gap * spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params().dynamic_obstacle_weight());
  return obstacle_cost;
}

// Simple version: calculate obstacle cost by distance
double SpatioTemporalUnionDp::GetLongitCostBetweenObsBoxes(
    const double &longit_dis_to_ego,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  const auto& dp_dynamic_agent_weight_params =
      spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params();
  double obstacle_cost = 0.0;
  // double inv_collision_distance =
  //     1.0 / dp_dynamic_agent_weight_params.obstacle_longit_collision_distance();
  // double inv_risk_distance =
  //     1.0 / dp_dynamic_agent_weight_params.obstacle_longit_risk_distance();

  if (longit_dis_to_ego > dp_dynamic_agent_weight_params.obstacle_longit_risk_distance()) {
    return obstacle_cost;
  }
  // if (distance < 1e-2) {
  //   ILOG_ERROR << "obstacle_collision!!!";
  // }

  obstacle_cost +=
      dp_dynamic_agent_weight_params.obstacle_collision_cost() *
      Sigmoid(dp_dynamic_agent_weight_params.obstacle_longit_collision_distance() - longit_dis_to_ego);

  return obstacle_cost;
}

double SpatioTemporalUnionDp::GetLateralCostBetweenObsBoxes(
    const double &lateral_dis_to_ego,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  const auto& dp_dynamic_agent_weight_params =
      spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params();
  double obstacle_cost = 0.0;

  if (lateral_dis_to_ego > dp_dynamic_agent_weight_params.obstacle_lateral_risk_distance()) {
    return obstacle_cost;
  }

  obstacle_cost +=
      dp_dynamic_agent_weight_params.obstacle_collision_cost_without_lateral_overlap() *
      Sigmoid(dp_dynamic_agent_weight_params.obstacle_lateral_collision_distance() - lateral_dis_to_ego);

  return obstacle_cost;
}

bool SpatioTemporalUnionDp::IsOffRoad(const double ref_s, const double l,
                                     const double dl,
                                     const bool is_change_lane_path) {
  static constexpr double kIgnoreDistance = 5.0;
  if (ref_s - planning_init_point_.frenet_state.s < kIgnoreDistance) {
    return false;
  }

  current_lane_s_width_.clear();
  current_lane_s_width_.reserve(current_refline_->get_points().size());
  for (auto i = 0; i < current_refline_->get_points().size(); i++) {
    const ReferencePathPoint &ref_path_point =
        current_refline_->get_points()[i];
    current_lane_s_width_.emplace_back(std::make_pair(
        ref_path_point.path_point.s(), ref_path_point.lane_width));
  }

  Vec2d rear_center(0.0, l);

  const auto &param  =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double front_edge_to_center = param.front_edge_to_rear_axle - param.rear_axle_to_center;
  double back_edge_to_center = param.rear_axle_to_center + param.rear_edge_to_rear_axle;
  double left_edge_to_center = 0.5 * param.width;
  double right_edge_to_center = 0.5 * param.width;

  Vec2d vec_to_center(
      (front_edge_to_center - param.rear_edge_to_rear_axle) / 2.0,
      (left_edge_to_center - right_edge_to_center) / 2.0);

  Vec2d rear_center_to_center = vec_to_center.rotate(std::atan(dl));
  Vec2d center = rear_center + rear_center_to_center;
  Vec2d front_center = center + rear_center_to_center;

  const double buffer = 0.1;  // in meters
  const double r_w =
      (left_edge_to_center + right_edge_to_center) / 2.0;
  const double r_l = back_edge_to_center;
  const double r = std::sqrt(r_w * r_w + r_l * r_l);

  double lane_width = 0.0;
  lane_width = QueryLaneWidth(ref_s, current_lane_s_width_);

  double left_bound =
      std::max(planning_init_point_.frenet_state.r + r + buffer, 0.5 * lane_width);
  double right_bound =
      std::min(planning_init_point_.frenet_state.r - r - buffer, -0.5 * lane_width);
  if (rear_center.y() + r + buffer / 2.0 > left_bound ||
      rear_center.y() - r - buffer / 2.0 < right_bound) {
    return true;
  }
  if (front_center.y() + r + buffer / 2.0 > left_bound ||
      front_center.y() - r - buffer / 2.0 < right_bound) {
    return true;
  }

  return false;
}

bool SpatioTemporalUnionDp::CheckOverlapOnDpSltGraph(
    const SLTGraphPoint &cur_cost_point,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs) {
  // 生成L方向三次多项式
  // std::array<double, 2> later_start_state{pre_cost_point.point().l(), 0.0};
  // std::array<double, 2> later_end_state{cur_cost_point.point().l(), 0.0};
  // double remain_time = cur_cost_point.point().t() - pre_cost_point.point().t();
  // CubicPolynomialCurve1d lateral_cubic_polynomial_curve(
  //     later_start_state, later_end_state, remain_time);

  // const double target_t = cur_cost_point.point().t();
  // double dl_dot =
  //     lateral_cubic_polynomial_curve.Evaluate(1, remain_time);
  // double ds_dot = pre_cost_point.GetOptimalSpeed() + acc * remain_time;
  // ds_dot = std::max(ds_dot, 1e-3);
  // double dl_ds = dl_dot / ds_dot;
  double cos_theta = 1.0;
  double sin_theta = 0.0;
  planning_math::Vec2d cur_ego_point(cur_cost_point.point().s(), cur_cost_point.point().l());
  std::array<planning_math::Vec2d, 8> ego_box_vertices;

  // 获取sl坐标系下自车box的顶点坐标集合
  GetVehicleBoxSLVertices(cur_ego_point, cos_theta, sin_theta, ego_box_vertices);
  // 构建自车box
  AABox2d cur_ego_box(ego_box_vertices, kDefaultValidBoxNums);
  int index = cur_cost_point.index_t() * kDefaultTrajsPointNums;

  // 遍历所有障碍物集合 判断是否存在overlap
  for (const auto& agent_traj : agent_trajs) {
    if (agent_traj.agent_id == -1) {
      continue;
    }
    if (agent_traj.agent_boxs_set.empty()) {
      continue;
    }

    auto it = agent_traj.agent_boxs_set.find(index);

    if (it != agent_traj.agent_boxs_set.end()) {
      const auto& cur_agent_box = it->second;

      // 判断自车box与障碍物agent_box是否存在overlap
      if (cur_agent_box.HasOverlap(cur_ego_box)) {
        return true;
      }
    }
  }

  return false;
}

void SpatioTemporalUnionDp::GetVehicleBoxSLVertices(
    const planning_math::Vec2d &ego_point,
    const double &cos_theta,
    const double &sin_theta,
    std::array<planning_math::Vec2d, 8> &vertices) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  double c_x = ego_point.x() + vehicle_param.rear_axle_to_center * cos_theta;
  double c_y = ego_point.y() + vehicle_param.rear_axle_to_center * sin_theta;

  double d_wx = vehicle_param.width * 0.5 * sin_theta;
  double d_wy = vehicle_param.width * 0.5  * cos_theta;
  double d_lx = vehicle_param.length * 0.5 * cos_theta;
  double d_ly = vehicle_param.length * 0.5 * sin_theta;

  // Counterclockwise from left-front vertex
  // vertices->emplace_back(Vec2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
  vertices[0] = Vec2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly);

  // vertices->emplace_back(Vec2d(c_x - d_wx, c_y + d_wy));

  // vertices->emplace_back(Vec2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
  vertices[1] = Vec2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy);
  // vertices->emplace_back(Vec2d(c_x - d_lx, c_y - d_ly));
  // vertices->emplace_back(Vec2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
  vertices[2] = Vec2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly);

  // vertices->emplace_back(Vec2d(c_x + d_wx, c_y - d_wy));
  // vertices->emplace_back(Vec2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
  vertices[3] = Vec2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy);
  // vertices->emplace_back(Vec2d(c_x + d_lx, c_y + d_ly));

  return;
}

double SpatioTemporalUnionDp::ComputeFirstDerivative(
    const std::array<double, 4> &lateral_curve_coef,
    const std::array<double, 4> &longit_curve_coef,
    const double param) {
  double a0 = longit_curve_coef[0], a1 = longit_curve_coef[1], a2 = longit_curve_coef[2], a3 = longit_curve_coef[3];
  double b0 = lateral_curve_coef[0], b1 = lateral_curve_coef[1], b2 = lateral_curve_coef[2], b3 = lateral_curve_coef[3];

  double dlds = 0.0;
  // Derivative of s with respect to t
  double dsdt = 3 * a3 * param * param + 2 * a2 * param + a1;
  dsdt = std::max(dsdt, 1e-3);
  // Derivative of l with respect to t
  double dldt = 3 * b3 * param * param + 2 * b2 * param + b1;
  // First derivative of l with respect to s
  dlds = dldt / dsdt;

  return dlds;
}

double SpatioTemporalUnionDp::ComputeSecondDerivative(
    const std::array<double, 4> &lateral_curve_coef,
    const double &acc,
    const double &v0,
    const double &param) {
  // double a0 = longit_curve_coef[0], a1 = longit_curve_coef[1], a2 = longit_curve_coef[2], a3 = longit_curve_coef[3];
  double b0 = lateral_curve_coef[0], b1 = lateral_curve_coef[1], b2 = lateral_curve_coef[2], b3 = lateral_curve_coef[3];

  // Derivative of s with respect to t
  double dsdt = v0 + acc * param;
  dsdt = std::max(dsdt, 1e-6);
  // Derivative of l with respect to t
  double dldt = 3.0 * b3 * param * param + 2.0 * b2 * param + b1;
  // First derivative of l with respect to s
  double dlds = dldt / dsdt;

  // Second derivative of l with respect to s using the product rule
  double d2lds2 =
      ((6.0 * b3 * param + 2.0 * b2) * dsdt - acc * dldt) / (dsdt * dsdt * dsdt);

  return d2lds2;
}

bool SpatioTemporalUnionDp::IsValidCurve(const CubicPolynomialCurve1d &curve) {
  for (double s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const double l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}

double SpatioTemporalUnionDp::QueryLaneWidth(
    const double s0,
    const std::vector<std::pair<double, double>> &lane_s_width) {
  auto comp = [](const std::pair<double, double> &s_width, const double s) {
    return s_width.first < s;
  };
  double lane_width;
  const auto &first_pair_on_lane =
      std::lower_bound(lane_s_width.begin(), lane_s_width.end(), s0, comp);

  if (first_pair_on_lane == lane_s_width.begin()) {
    lane_width = lane_s_width.front().second;
  } else if (first_pair_on_lane == lane_s_width.end()) {
    lane_width = lane_s_width.back().second;
  } else {
    lane_width = planning_math::lerp(
        (first_pair_on_lane - 1)->second, (first_pair_on_lane - 1)->first,
        first_pair_on_lane->second, first_pair_on_lane->first, s0);
  }
  return std::fmax(lane_width, 2.8);
}

void SpatioTemporalUnionDp::CaculateCurvatureLimitSpeed(
    double target_s, double &speed_limit,
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_ratio = vehicle_param.steer_ratio;
  double wheel_base = vehicle_param.wheel_base;
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();
  const auto& speed_limit_params = spatio_temporal_union_plan_input.speed_limit_params();

  double angle_steers = ego_init_state.steer_angle();
  double angle_steers_deg = angle_steers * DEG_PER_RAD;
  double v_ego = ego_init_state.v0();
  double acc_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V);
  double acc_lat =
      std::pow(v_ego, 2) * angle_steers / (steer_ratio * wheel_base);
  double acc_lon_allowed = std::sqrt(
      std::max(std::pow(acc_total_max, 2) - std::pow(acc_lat, 2), 0.0));

  // And limit the logitudinal velocity for a safe turn
  double acc_lat_max =
      interp(std::abs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  // HACK: close v_limit_steering in high vel
  bool is_high_vel = v_ego > kHighVel;
  double v_limit_steering = 100.0;
  if (!is_high_vel) {
    v_limit_steering = std::sqrt((acc_lat_max * steer_ratio * wheel_base) /
                                 std::max(std::abs(angle_steers), 0.001));
  }
  double v_limit_in_turns = v_limit_steering;
  // calculate the velocity limit according to the road curvature
  double preview_x =
      speed_limit_params.dis_curv() + speed_limit_params.t_curv() * v_ego;

  std::vector<double> curv_window_vec;
  for (int idx = -3; idx <= 3; ++idx) {
    double curv;
    if (current_lane_coord_.GetKappaByS(target_s + preview_x + idx * 2.0, &curv)) {
      curv = std::fabs(curv);
    } else {
      curv = 0.0001;
    }
    curv_window_vec.emplace_back(curv);
  }
  double curv_sum = 0.0;
  for (int ind = 0; ind < curv_window_vec.size(); ++ind) {
    curv_sum = curv_sum + curv_window_vec[ind];
  }
  double avg_curv = curv_sum / curv_window_vec.size();
  double road_radius = 1 / std::max(avg_curv, 0.0001);
  if (road_radius < 400) {
    acc_lat_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
  }
  double v_limit_road = std::sqrt(acc_lat_max * road_radius);
  v_limit_in_turns = std::min(v_limit_in_turns, v_limit_road);
  if (v_limit_in_turns < speed_limit) {
    speed_limit = v_limit_in_turns;
  }

  return;
}

void SpatioTemporalUnionDp::CalculateMapSpeedLimit(
      double &speed_limit,
      const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input) {
  const auto& speed_limit_params = spatio_temporal_union_plan_input.speed_limit_params();
  double dis_to_ramp = speed_limit_params.dis_to_ramp();
  double dis_to_merge = speed_limit_params.dis_to_merge();
  bool is_on_ramp = speed_limit_params.is_on_ramp();
  bool is_continuous_ramp = speed_limit_params.is_continuous_ramp();

  double v_target_ramp = 40;
  double v_target_near_ramp_zone = 40;
  double pre_acc_dis = speed_limit_params.pre_accelerate_distance_for_merge();
  bool sdmap_has_curv = true;  //先不考虑匝道内小曲率的情况，后面补充
  // 通过接口获取是否在匝道的信息
  if (is_on_ramp) {
    if (dis_to_merge > pre_acc_dis || is_continuous_ramp) {
      if (sdmap_has_curv) {
        v_target_ramp = speed_limit_params.v_limit_ramp();
      } else {
        v_target_ramp = speed_limit_params.straight_ramp_v_limit();
      }
    }
    if (v_target_ramp < speed_limit) {
      speed_limit = v_target_ramp;
    }
    return;
  }
  if (dis_to_ramp <= speed_limit_params.dis_near_ramp_zone()) {
    double pre_brake_dis_near_ramp_zone = std::max(
        dis_to_ramp - speed_limit_params.brake_dis_near_ramp_zone(), 0.0);
    v_target_near_ramp_zone = std::pow(
        std::pow(speed_limit_params.v_limit_near_ramp_zone(), 2.0) -
            2 * pre_brake_dis_near_ramp_zone * speed_limit_params.acc_to_ramp(),
        0.5);
  }
  double pre_brake_dis_to_ramp = std::max(dis_to_ramp - 50, 0.0);
  v_target_ramp =
      std::pow(std::pow(speed_limit_params.v_limit_ramp(), 2.0) -
                   2 * pre_brake_dis_to_ramp * speed_limit_params.acc_to_ramp(),
               0.5);
  v_target_ramp = std::min(v_target_near_ramp_zone, v_target_ramp);
  if (v_target_ramp < speed_limit) {
    speed_limit = v_target_ramp;
  }

  return;
}

void SpatioTemporalUnionDp::GenerateExpandedTrajectoryPoint(
    const SpeedInfo &init_point, const SpeedInfo &next_point) {

  // 生成L方向三次多项式
  std::array<double, 2> later_start_state{init_point.l, 0.0};
  std::array<double, 2> later_end_state{next_point.l, 0.0};
  double remain_time = next_point.t - init_point.t;
  CubicPolynomialCurve1d lateral_cubic_curve(
      later_start_state, later_end_state, remain_time);

  // 计算必须满足的约束条件
  const double delta_s = next_point.s - init_point.s;
  const double avg_speed = delta_s / remain_time;

  // 调整边界速度确保单调性
  double adjusted_v0 = std::max(0.0, init_point.v);
  double adjusted_v1 = std::max(0.0, next_point.v);

  // 应用Fritsch-Carlson单调性约束
  const double alpha = adjusted_v0 / avg_speed;
  const double beta = adjusted_v1 / avg_speed;
  if (alpha * alpha + beta * beta > 9.0) {
    const double tau = 3.0 / std::sqrt(alpha * alpha + beta * beta);
    adjusted_v0 = tau * alpha * avg_speed;
    adjusted_v1 = tau * beta * avg_speed;
  }
  // 生成S方向三次多项式（强制单调递增）
  std::array<double, 2> longit_start_state{
      init_point.s, adjusted_v0};
  std::array<double, 2> longit_end_state{
      next_point.s, adjusted_v1};
  CubicPolynomialCurve1d longit_cubic_curve(
      longit_start_state, longit_end_state, remain_time);

  // 生成插值点（排除终点，避免重复）
  constexpr double delta_time = 0.2;
  for (int i = 0; i < 5; ++i) { // 0.0, 0.2, 0.4, 0.6, 0.8
    const double relative_time = i * delta_time;

    SpeedInfo point;
    point.t = init_point.t + relative_time;
    point.s = longit_cubic_curve.Evaluate(0, relative_time);
    point.v = longit_cubic_curve.Evaluate(1, relative_time);
    point.a = longit_cubic_curve.Evaluate(2, relative_time);
    point.l = lateral_cubic_curve.Evaluate(0, relative_time);
    point.da = 0.0;

    // 验证单调性
    if (!dp_speed_profile_.empty() &&
        point.s <= dp_speed_profile_.back().s) {
      point.s = dp_speed_profile_.back().s +
               (next_point.s - init_point.s) * delta_time / remain_time;
    }

    dp_speed_profile_.emplace_back(point);
  }

  return;
}

bool SpatioTemporalUnionDp::JudgeSVecisMonotonicIncreasing(const std::vector<double> &vec) {
  if (vec.empty()) {
    return false;
  }
  for (size_t i = 1; i < vec.size(); ++i) {
    if (vec[i] - kDoubleEpsilon <= vec[i-1]) {
      return false;
    }
  }

  return true;

}

void SpatioTemporalUnionDp::PrecomputeInvSpeedLimit() {
  for (auto& limit : speed_limit_by_index_) {
    inv_speed_limit_table_.push_back(1.0 / std::max(limit, 1e-2));
  }
}

void SpatioTemporalUnionDp::FallbackFunction(
    const planning::common::SpationTemporalUnionDpInput &spatio_temporal_union_plan_input,
    TrajectoryPoints &traj_points, const bool &last_enable_using_st_plan) {
  const auto& ego_init_state = spatio_temporal_union_plan_input.init_state();
  double ego_v = ego_init_state.v0();
  const double acc_coeff = 0.5;
  bool enable_use_fallback = true;

  if (trajectory_points_.point_size() == kDefaultTrajectoryPointSize &&
      last_planning_result_coord_ && ego_v > kEgoStaticThreshold &&
      enable_use_ego_cart_point_ && last_enable_using_st_plan) {
    Point2D ego_frenet_in_last_result;
    if (!last_planning_result_coord_->XYToSL(ego_cart_point_, ego_frenet_in_last_result)) {
      ego_frenet_in_last_result.x = 0.0;
      ego_frenet_in_last_result.y = 0.0;
    }
    const double ref_theta =
        last_planning_result_coord_->GetPathCurveHeading(ego_frenet_in_last_result.x);
    const double theta_diff = NormalizeAngle(ref_theta - ego_init_state.heading_angle());
    double v_s = ego_v * std::cos(std::fabs(theta_diff));
    double v_end =
        2.0 * (last_planning_result_coord_->Length() - ego_frenet_in_last_result.x) / 5.0 - v_s;
    double acc = (v_end - v_s) / 5.0;
    TrajectoryPoint point;
    for (int i = 0; i < kDefaultTrajectoryPointSize; ++i) {
      if (i == 0) {
        point.s = ego_frenet_in_last_result.x + ego_init_state.s();
        point.l = ego_init_state.l();
        point.heading_angle = ego_init_state.heading_angle();
        point.t = 0.0;
        point.x = ego_cart_point_.x;
        point.y = ego_cart_point_.y;
        traj_points[i] = point;
      } else {
        double time = kDeltaTime * i;
        double last_frame_frenet_point_s = v_s * time + acc_coeff * acc * time * time + ego_frenet_in_last_result.x;
        last_frame_frenet_point_s = std::min(last_frame_frenet_point_s, last_planning_result_coord_->Length());
        double last_frame_frenet_point_l = 0.0;
        point.heading_angle = last_planning_result_coord_->GetPathCurveHeading(last_frame_frenet_point_s);
        point.t = time;
        Point2D last_frame_frenet_point(last_frame_frenet_point_s, 0.0);
        Point2D cur_point;
        last_planning_result_coord_->SLToXY(last_frame_frenet_point, cur_point);
        Point2D cur_frenet_point;
        current_lane_coord_.XYToSL(cur_point, cur_frenet_point);
        point.x = cur_point.x;
        point.y = cur_point.y;
        point.s = cur_frenet_point.x;
        point.l = cur_frenet_point.y;
        if (point.s <= traj_points[i-1].s) {
          enable_use_fallback = false;
          break;
        }
        traj_points[i] = point;
      }
    }
  }

  if (enable_use_fallback) {
    return;
  }

  TrajectoryPoint point;
  int target_count = 0;
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (i == 0) {
      point.s = ego_init_state.s();
      point.l = ego_init_state.l();
      point.heading_angle = ego_init_state.heading_angle();
      point.t = traj_points[i].t;
      point.x = ego_cart_point_.x;
      point.y = ego_cart_point_.y;
      traj_points[i] = point;
      target_count = i;
    } else {
      double delta_time = traj_points[i].t - traj_points[0].t;
        double constant_speed_time = delta_time;
        Point2D cur_frenet_point;
        cur_frenet_point.x = traj_points[target_count].s + constant_speed_time * ego_v;
        cur_frenet_point.y = ego_init_state.l();

        // point.x = sample_point.x();
        // point.y = sample_point.y();
        // point.heading_angle = lane_change_quintic_path.heading(traj_points[i].t);

        Point2D cart_point;
        if (!current_lane_coord_.SLToXY(cur_frenet_point, cart_point)) {
          LOG_ERROR("FallbackFunction()::ERROR! Frenet Point -> Cart Point Failed!!!");
        }
        point.s = cur_frenet_point.x;
        point.l = cur_frenet_point.y;
        point.t = traj_points[i].t;
        point.x = cart_point.x;
        point.y = cart_point.y;
        planning_math::PathPoint cur_s_nearest_point =
            current_lane_coord_.GetPathPointByS(cur_frenet_point.x);
        point.heading_angle =
            current_lane_coord_.GetPathCurveHeading(cur_s_nearest_point.s());

        traj_points[i] = point;
    }
  }

  return;
}

}  // namespace planning