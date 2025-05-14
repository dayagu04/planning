#include "dp_speed_optimizer.h"

#include <bits/stdint-intn.h>

#include <algorithm>
#include <limits>
#include <string>

#include "debug_info_log.h"
#include "dp_speed_common.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "speed/apa_speed_decision.h"
#include "sv_graph_node.h"

namespace planning {
namespace apa_planner {

#define DEBUG_INIT_COST_TABLE (0)
#define DEBUG_RETRIEVE_SPEED_PROFILE (0)
#define DEBUG_SEARCH (0)
#define DEBUG_RESULT (0)

DpSpeedOptimizer::DpSpeedOptimizer() {}

bool DpSpeedOptimizer::Init() {
  config_.Init();

  start_node_ = nullptr;
  end_node_ = nullptr;
  speed_data_.clear();
  state_ = TaskExcuteState::NONE;
  ClearDebugInfo();

  ILOG_INFO << "dp init";
  return true;
}

void DpSpeedOptimizer::Excute(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const Pose2D& ego_pose, const SVPoint& init_point,
    const SpeedDecisions* speed_decisions,
    const SpeedLimitProfile* speed_limit_profile) {
  Init();

  if (speed_decisions == nullptr || speed_limit_profile == nullptr) {
    return;
  }

  // init
  total_s_ = 0.0;
  if (path.size() > 0) {
    total_s_ = path.back().s;
  }

  double opt_start_time = IflyTime::Now_ms();

  const ParkLonDecision* stop_decision = GetCloseStopDecision(speed_decisions);
  if (stop_decision != nullptr) {
    total_s_ = std::min(
        total_s_, stop_decision->path_s - stop_decision->lon_decision_buffer);
  }

  // update resolution
  if (total_s_ > config_.long_path_thresh) {
    config_.unit_v = config_.unit_v_for_long_path;
    config_.unit_s = config_.unit_s_for_long_path;
  } else if (total_s_ > config_.extreme_short_path_thresh) {
    config_.unit_v = config_.unit_v_for_short_path;
    config_.unit_s = config_.unit_s_for_short_path;
  } else {
    config_.unit_v = config_.unit_v_for_extream_short_path;
    config_.unit_s = config_.unit_s_for_extream_short_path;

    ILOG_INFO << "need use jlt speed profile";
  }

  ego_v_ = init_point.v;
  ego_acc_ = init_point.acc;
  speed_decisions_ = speed_decisions;
  speed_limit_profile_ = speed_limit_profile;

  if (total_s_ < config_.extreme_short_path_thresh) {
    return;
  }

  cost_generator_.Init(total_s_, &config_);

  Search();

  RecordDebugInfo(path);

  ILOG_INFO << "dp optimizer time = " << IflyTime::Now_ms() - opt_start_time;

  return;
}

void DpSpeedOptimizer::UpdateSearchBoundary() {
  // update s dimension
  dimension_s_ = static_cast<int32_t>(std::ceil(total_s_ / config_.unit_s)) + 1;

  // update v dimension
  max_search_v_ = std::max(ego_v_, config_.dp_cruise_speed) + 0.01;
  if (max_search_v_ > config_.enter_apa_speed_margin) {
    error_code_ = DpSpeedErrorCode::FAIL_BY_VEH_HIGH_SPEED;
    ILOG_INFO << "speed is high, do not call APA";
  }

  dimension_v_ =
      static_cast<int32_t>(std::ceil(max_search_v_ / config_.unit_v)) + 1;

  ILOG_INFO << "s node size = " << dimension_s_
            << ", v node size = " << dimension_v_
            << ", max search v = " << max_search_v_;

  return;
}

void DpSpeedOptimizer::UpdateCostTable() {
  // cost table init
  // todo: if time consumption is high, use c array.
  cost_table_ = std::vector<std::vector<SVGraphNode>>(
      dimension_s_, std::vector<SVGraphNode>(dimension_v_, SVGraphNode()));

  double curr_s = 0.0;
  double speed_limit;
  double curr_v;
  for (int32_t i = 0; i < cost_table_.size(); ++i, curr_s += config_.unit_s) {
    auto& cost_table_i = cost_table_[i];

    curr_v = 0.0;
    speed_limit = speed_limit_by_index_[i];
    for (int32_t j = 0; j < dimension_v_; ++j, curr_v += config_.unit_v) {
      cost_table_i[j].Init(i, j, SVPoint(curr_s, curr_v), speed_limit);

#if DEBUG_INIT_COST_TABLE
      if (i == 0 || i == 1 || i == 2) {
        ILOG_INFO << "s index = " << i << ", v index = " << j
                  << ", s value = " << curr_s;
      }

#endif
    }
  }

  return;
}

void DpSpeedOptimizer::UpdateSpeedLimitLookUp() {
  speed_limit_by_index_.clear();
  speed_limit_by_index_.resize(dimension_s_);

  double curr_s = 0.0;
  double speed_limit;
  for (int32_t i = 0; i < dimension_s_; ++i) {
    speed_limit = speed_limit_profile_->GetSpeedLimitByRange(curr_s, 0.1);
    speed_limit_by_index_[i] = speed_limit;

    curr_s += config_.unit_s;
    curr_s = std::min(curr_s, total_s_);
  }

  // DebugSpeedLimitLookUp();

  return;
}

void DpSpeedOptimizer::GetNextNodeSpeedRange(const double node_v,
                                             SpeedBoundary* bound) {
  double v_square = node_v * node_v;

  // update upper v
  bound->upper =
      std::sqrt(v_square + 2 * config_.acceleration_limit * config_.unit_s);
  bound->upper = std::min(bound->upper, max_search_v_);

  double v_low = v_square + 2 * config_.deceleration_limit * config_.unit_s;
  if (v_low < 0) {
    bound->lower = 0.0;
  } else {
    bound->lower = std::sqrt(v_low);
  }

  return;
}

void DpSpeedOptimizer::GetNodeLayerSpeedRange(const double s,
                                              SpeedBoundary* bound) {
  double v_square = ego_v_ * ego_v_;
  bound->upper = std::sqrt(v_square + 2 * config_.acceleration_limit * s);
  bound->upper = std::min(bound->upper, max_search_v_);

  double v_low = v_square + 2 * config_.deceleration_limit * s;
  if (v_low < 0) {
    bound->lower = 0.0;
  } else {
    bound->lower = std::sqrt(v_low);
  }

  return;
}

void DpSpeedOptimizer::GetNodeSpeedRangeIndex(const SpeedBoundary* v_bound,
                                              SpeedBoundaryIndex* v_index) {
  v_index->upper_index = std::ceil(v_bound->upper / config_.unit_v);
  v_index->lower_index = std::floor(v_bound->lower / config_.unit_v);

  return;
}

SVGraphNode* DpSpeedOptimizer::GetNodeFromPool(const SVGridIndex& index) {
  if (index.s_index >= cost_table_.size()) {
    return nullptr;
  }

  if (index.v_index >= cost_table_[index.s_index].size()) {
    return nullptr;
  }

  return &cost_table_[index.s_index][index.v_index];
}

void DpSpeedOptimizer::GridIndexTransform(const SVGridIndex* index,
                                          const DpSpeedConfig* config,
                                          SVPoint* point) {
  point->s = index->s_index * config->unit_s;
  point->v = index->v_index * config->unit_v;

  return;
}

void DpSpeedOptimizer::GridIndexTransform(const SVPoint* point,
                                          const DpSpeedConfig* config,
                                          SVGridIndex* index) {
  index->s_index = std::round(point->s / config->unit_s);
  index->v_index = std::round(point->v / config->unit_v);

  return;
}

void DpSpeedOptimizer::UpdateStartNode() {
  SVPoint point_sv;
  point_sv.s = 0;
  point_sv.acc = ego_acc_;
  point_sv.v = ego_v_;
  point_sv.t = 0.0;
  point_sv.jerk = 0.0;

  SVGridIndex point_index;
  point_index.s_index = 0;
  point_index.v_index = std::round(point_sv.v / config_.unit_v);

  start_node_ = GetNodeFromPool(point_index);
  if (start_node_ != nullptr) {
    start_node_->Init(point_index.s_index, point_index.v_index, point_sv,
                      speed_limit_by_index_[0]);
    start_node_->SetZeroCost();
  }

  return;
}

void DpSpeedOptimizer::UpdateEndNode() {
  SVGridIndex point_index;
  point_index.s_index = dimension_s_ - 1;
  point_index.v_index = 0;

  end_node_ = GetNodeFromPool(point_index);
  if (end_node_ != nullptr) {
    SVPoint point_sv;
    point_sv.s = total_s_;
    point_sv.acc = 0;
    point_sv.v = 0;
    point_sv.jerk = 0;
    end_node_->Init(point_index.s_index, point_index.v_index, point_sv, 0.0);
  }

  return;
}

const double DpSpeedOptimizer::GetSpeedLimitByIndex(const int32_t s_index) {
  if (s_index < 0) {
    return 0;
  }

  if (s_index >= speed_limit_by_index_.size()) {
    return 0;
  }

  return speed_limit_by_index_[s_index];
}

void DpSpeedOptimizer::GenerateNextNode(const SVGraphNode* parent_node,
                                        const int32_t v_index,
                                        SVGraphNode* child_node) {
  double node_s = parent_node->GetSVPoint().s + config_.unit_s;
  node_s = std::min(node_s, total_s_);

  child_node->Init(parent_node->IndexS() + 1, v_index,
                   SVPoint(node_s, GetSpeedByIndex(v_index)),
                   GetSpeedLimitByIndex(parent_node->IndexS() + 1));

  child_node->SetPrePoint(parent_node);
  double delta_s = node_s - parent_node->GetSVPoint().s;
  child_node->EvaluateAcc(parent_node, delta_s);
  child_node->EvaluateTime(parent_node, delta_s);
  child_node->EvaluateJerk(parent_node);

  return;
}

void DpSpeedOptimizer::CalculateNodeCost(const SVGraphNode* parent,
                                         const int32_t s_index,
                                         const int32_t v_index,
                                         SVGraphNode* child) {
  child->SetZeroCost();
  // start point
  if (s_index == 0) {
    return;
  }

  // stopover cost
  cost_generator_.CalcStopoverCost(child);

  // acc cost
  cost_generator_.CalcAccCost(child);

  double speed_limit;
  if (end_node_->GetSVPoint().s - child->GetSVPoint().s < 0.001 ||
      child->GetSVPoint().s > end_node_->GetSVPoint().s) {
    speed_limit = 0.0;
  } else {
    speed_limit = GetSpeedLimitByIndex(s_index);
  }

  cost_generator_.CalcSpeedCost(speed_limit, child);
  cost_generator_.CalcJerkCost(child);
  cost_generator_.UpdateTotalCost(parent, child);

  return;
}

const bool DpSpeedOptimizer::IsNodeShouldStop(SVGraphNode& node) {
  if (node.IndexS() >= dimension_s_ - 1) {
    return true;
  }

  return false;
}

void DpSpeedOptimizer::CalculateTotalCost() {
  if (start_node_ == nullptr) {
    ILOG_INFO << "start_node_ is null";
    return;
  }

  SpeedBoundary next_node_v_bound;
  SpeedBoundaryIndex next_node_v_bound_index;
  SVGraphNode* current_node = nullptr;
  SVGraphNode* pool_node = nullptr;
  SVGraphNode child_node;

  // init search range for v
  SpeedBoundary node_layer_v_bound;
  SpeedBoundaryIndex node_layer_v_bound_index;
  node_layer_v_bound_index.lower_index = start_node_->GetSVIndex().v_index;
  node_layer_v_bound_index.upper_index = node_layer_v_bound_index.lower_index;

  // next layer range for v
  SpeedBoundaryIndex next_layer_v_bound_index = node_layer_v_bound_index;

  // No need to search end point.
  int32_t s_id;
  for (s_id = 0; s_id < dimension_s_ - 1; s_id++) {
    // search current layer
#if DEBUG_SEARCH
    ILOG_INFO << "-------debug search------";
#endif

    for (int32_t cur_node_v_id = node_layer_v_bound_index.lower_index;
         cur_node_v_id <= node_layer_v_bound_index.upper_index;
         cur_node_v_id++) {
      current_node = GetNodeFromPool(SVGridIndex(s_id, cur_node_v_id));
      if (current_node == nullptr) {
        ILOG_INFO << "current node is null";
        continue;
      }

      GetNextNodeSpeedRange(current_node->GetSVPoint().v, &next_node_v_bound);
      GetNodeSpeedRangeIndex(&next_node_v_bound, &next_node_v_bound_index);

#if DEBUG_SEARCH
      ILOG_INFO << "-------current node search------";
      current_node->DebugString();
      current_node->ConstCost().DebugCost();
      next_node_v_bound.DebugString();
      next_node_v_bound_index.DebugString();
#endif

      // generate child node
      for (int32_t next_node_v_id = next_node_v_bound_index.lower_index;
           next_node_v_id <= next_node_v_bound_index.upper_index;
           next_node_v_id++) {
        GenerateNextNode(current_node, next_node_v_id, &child_node);

        // check speed, if not stop, return
        if (IsNodeShouldStop(child_node) && child_node.GetSVPoint().v > 1e-3) {
          continue;
        }

        CalculateNodeCost(current_node, s_id + 1, next_node_v_id, &child_node);

#if DEBUG_SEARCH
        ILOG_INFO << "child node";
        child_node.DebugString();
        child_node.ConstCost().DebugCost();
#endif

        // compare cost
        pool_node = GetNodeFromPool(child_node.GetSVIndex());
        if (pool_node == nullptr) {
          ILOG_INFO << "pool node is null";
          continue;
        }

        if (child_node.ConstCost().total_cost <
            pool_node->ConstCost().total_cost) {
          pool_node->SetPrePoint(current_node);
          pool_node->SetAcc(child_node.GetSVPoint().acc);
          pool_node->SetCost(child_node.ConstCost());
          pool_node->SetTime(child_node.GetSVPoint().t);
        }
      }

      next_layer_v_bound_index.lower_index =
          std::min(next_layer_v_bound_index.lower_index,
                   next_node_v_bound_index.lower_index);

      next_layer_v_bound_index.upper_index =
          std::max(next_layer_v_bound_index.upper_index,
                   next_node_v_bound_index.upper_index);
    }

    // 更新下一层搜索范围
    node_layer_v_bound_index = next_layer_v_bound_index;

#if DEBUG_SEARCH
    ILOG_INFO << "next range lower = " << node_layer_v_bound_index.lower_index
              << ",upper = " << node_layer_v_bound_index.upper_index;
#endif
  }

  return;
}

void DpSpeedOptimizer::Search() {
  ILOG_INFO << "dp speed optimizer";
  UpdateSearchBoundary();

  // init speed limit
  UpdateSpeedLimitLookUp();

  // sv graph search, sv point init
  UpdateCostTable();

  UpdateStartNode();

  UpdateEndNode();

  // dp search
  CalculateTotalCost();

  // backtrace
  RetrieveSpeedProfile(&speed_data_);

  DebugSpeedData();

  state_ = TaskExcuteState::SUCCESS;

  return;
}

void DpSpeedOptimizer::RetrieveSpeedProfile(SpeedData* const speed_data) {
  if (end_node_ == nullptr) {
    return;
  }

#if DEBUG_RETRIEVE_SPEED_PROFILE
  ILOG_INFO << "end node";
  end_node_->DebugString();
#endif

  // 回溯
  std::vector<const SVGraphNode*> speed_profile_node;
  const SVGraphNode* cur_point = end_node_;
  while (cur_point != nullptr) {
    speed_profile_node.push_back(cur_point);

#if DEBUG_RETRIEVE_SPEED_PROFILE
    if (cur_point != nullptr) {
      cur_point->GetSVPoint().DebugString();
    }
#endif

    cur_point = cur_point->PrePoint();
  }

  // 速度曲线反向
  std::reverse(speed_profile_node.begin(), speed_profile_node.end());

  // 第一个点是起点，s=0.0, 如果不是，说明内部出了错误
  if (speed_profile_node.front() != nullptr &&
      speed_profile_node.front()->GetSVPoint().s > 1e-3) {
    ILOG_INFO << "Fail to retrieve speed profile.";
    return;
  }

  speed_data->clear();
  const SVGraphNode* next_point;
  int32_t size = speed_profile_node.size();
  std::vector<SVPoint> speed_segment;
  SpeedPoint speed_point;

  for (int32_t i = 0; i < size - 1; ++i) {
    cur_point = speed_profile_node[i];
    next_point = speed_profile_node[i + 1];

    cur_point->Interpolate(config_.s_interpolate_step, next_point,
                           speed_segment);

    for (auto& obj : speed_segment) {
      speed_point.a = obj.acc;
      speed_point.s = obj.s;
      speed_point.v = obj.v;
      speed_point.da = obj.jerk;
      speed_point.t = obj.t;
      speed_data->emplace_back(speed_point);
    }
  }

  // Exception process
  if (size <= 1) {
    speed_point.a = 0;
    speed_point.s = 0;
    speed_point.v = ego_v_;
    speed_point.t = 0.0;
    speed_data->emplace_back(speed_point);
  }

  // fill end point
  if (size > 0 && speed_data->back().s < end_node_->GetSVPoint().s) {
    speed_point.a = speed_data->back().a;
    speed_point.s = end_node_->GetSVPoint().s;
    speed_point.v = end_node_->GetSVPoint().v;

    if (std::fabs(speed_data->back().a) > 0.0001) {
      speed_point.t = speed_data->back().t +
                      (end_node_->GetSVPoint().v - speed_data->back().v) /
                          speed_data->back().a;
    } else {
      if (speed_data->back().v < 0.0001) {
        speed_point.t = 1000.0;
      } else {
        speed_point.t = speed_data->back().t +
                        (end_node_->GetSVPoint().s - speed_data->back().s) /
                            speed_data->back().v;
      }
    }

    speed_data->emplace_back(speed_point);
  }

  return;
}

void DpSpeedOptimizer::DebugSpeedData() {
#if DEBUG_RESULT
  ILOG_INFO << "speed profile point size = " << speed_data_.size();

  for (int32_t i = 0; i < speed_data_.size(); i++) {
    ILOG_INFO << "s = " << speed_data_[i].s << ", t=" << speed_data_[i].t
              << ",v=" << speed_data_[i].v << ",acc = " << speed_data_[i].a
              << ",jerk = " << speed_data_[i].da;
  }

#endif
  return;
}

void DpSpeedOptimizer::DebugSpeedLimitLookUp() const {
  for (int32_t i = 0; i < speed_limit_by_index_.size(); ++i) {
    ILOG_INFO << "i = " << i << ", speed limit = " << speed_limit_by_index_[i];
  }

  return;
}

void DpSpeedOptimizer::ClearDebugInfo() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  speed_debug->clear_dp_profile();
  speed_debug->clear_dp_speed_constraint();
  speed_debug->clear_jlt_profile();
  return;
}

void DpSpeedOptimizer::RecordDebugInfo(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();

  common::StPoint2D proto_point;
  for (size_t i = 0; i < speed_data_.size(); i++) {
    const SpeedPoint& point = speed_data_[i];
    proto_point.set_s(point.s);
    proto_point.set_t(point.t);
    proto_point.set_vel(point.v);
    proto_point.set_acc(point.a);
    proto_point.set_jerk(point.da);

    speed_debug->add_dp_profile()->CopyFrom(proto_point);
  }

  if (speed_limit_profile_ != nullptr) {
    const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

    common::SVGraphSpeedConstraint* dp_speed_constraint_debug =
        speed_debug->mutable_dp_speed_constraint();

    const std::vector<std::pair<double, double>>& points =
        speed_limit_profile_->SpeedLimitPoints();

    for (size_t i = 0; i < points.size(); i++) {
      dp_speed_constraint_debug->add_s(points[i].first);
      dp_speed_constraint_debug->add_obs_dist(path[i].dist_to_obs);
      dp_speed_constraint_debug->add_v_upper_bound(points[i].second);
      dp_speed_constraint_debug->add_a_upper_bound(speed_config.acc_upper);
      dp_speed_constraint_debug->add_a_lower_bound(speed_config.acc_lower);
      dp_speed_constraint_debug->add_jerk_upper_bound(speed_config.jerk_upper);
      dp_speed_constraint_debug->add_jerk_lower_bound(speed_config.jerk_lower);

#if DECIDER_DEBUG
      ILOG_INFO << "i = " << i << ",s = " << points[i].first
                << ",v up = " << points[i].second << ",jerk upper = "
                << dp_speed_constraint_debug->jerk_upper_bound(i);
#endif
    }
  }

  return;
}

const SVPoint DpSpeedOptimizer::GetStartSpeedPoint() const {
  if (start_node_ != nullptr) {
    return start_node_->GetSVPoint();
  }

  SVPoint point_sv;
  point_sv.s = 0;
  point_sv.acc = 0;
  point_sv.v = ego_v_;
  point_sv.t = 0.0;
  point_sv.jerk = 0.0;

  return point_sv;
}
}  // namespace apa_planner
}  // namespace planning
