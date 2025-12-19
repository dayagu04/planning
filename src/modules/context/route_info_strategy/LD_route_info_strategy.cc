#include "LD_route_info_strategy.h"

#include <iostream>
#include <utility>

#include "config/basic_type.h"
#include "environmental_model.h"
#include "local_view.h"
#include "route_info_strategy.h"

namespace planning {
namespace {
constexpr double kEpsilon = 1.0e-4;
}

LDRouteInfoStrategy::LDRouteInfoStrategy(
    const MLCDeciderConfig* config_builder,
    const planning::framework::Session* session)
    : RouteInfoStrategy(config_builder, session) {
  local_view_ = &session_->environmental_model().get_local_view();
}

void LDRouteInfoStrategy::Update(RouteInfoOutput& route_info_output) {
  local_view_ = &session_->environmental_model().get_local_view();
  route_info_output_.reset();

  if (!UpdateLDMap()) {
    return;
  }

  if (!CalculateRouteInfo()) {
    return;
  }

  route_info_output = route_info_output_;
}

bool LDRouteInfoStrategy::UpdateLDMap() {
  const auto& ld_map_info = local_view_->sdpro_map_info;
  const auto ld_map_info_current_timestamp = ld_map_info.header().timestamp();
  if (ld_map_info_current_timestamp != ld_map_info_updated_timestamp_) {
    int res = ld_map_.LoadMapFromProto(ld_map_info);
    if (res == 0) {
      ldmap_valid_ = true;
      ld_map_info_updated_timestamp_ = ld_map_info_current_timestamp;
    }
  }
  if (ld_map_info_current_timestamp - ld_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    // 距离上一次更新时间超过阈值，则认为无效报错
    ldmap_valid_ = false;
    ILOG_ERROR << "error!!! because more than 20s no update hdmap!!!";
  }
  JSON_DEBUG_VALUE("sdpromap_valid_", ldmap_valid_)
  ILOG_INFO << "ldmap_valid_:" << ldmap_valid_;
  return ldmap_valid_;
}

bool LDRouteInfoStrategy::CalculateRouteInfo() {
  if (!ld_map_.isRouteValid()) {
    return false;
  }

  if (!CalculateCurrentLink()) {
    return false;
  }

  if (!IsInExpressWay()) {
    return false;
  }

  route_info_output_.is_on_ramp = ld_map_.isRamp(current_link_->link_type());
  route_info_output_.current_segment_passed_distance = ego_on_cur_link_s_;
  route_info_output_.is_update_segment_success = true;

  const auto& sdpro_map_info = local_view_->sdpro_map_info;
  route_info_output_.map_vendor = sdpro_map_info.data_source();

  merge_info_vec_.clear();
  split_info_vec_.clear();
  ramp_info_vec_.clear();

  CalculateMergeInfo();

  CalculateSplitInfo();

  // 一定要先计算split info，再计算ramp info
  CalculateRampInfo();

  CaculateDistanceToRoadEnd(current_link_, ego_on_cur_link_s_);

  CaculateDistanceToTollStation(current_link_, ego_on_cur_link_s_);

  return true;
}

bool LDRouteInfoStrategy::CalculateCurrentLink() {
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  // 获取当前的segment
  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);

  double temp_nearest_s = 0;
  double nearest_l = 0;
  const double ego_heading_angle = ego_state->heading_angle();

  current_link_ = ld_map_.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      temp_nearest_s, nearest_l);
  if (!current_link_) {
    return false;
  }

  ego_on_cur_link_s_ = temp_nearest_s;
  ego_on_cur_link_l_ = nearest_l;
  return true;
}

bool LDRouteInfoStrategy::IsInExpressWay() {
  // 判断自车当前是否在高速或者高架上
  if (current_link_->link_class() ==
          iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
      current_link_->link_class() ==
          iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY ||
      current_link_->link_type() == iflymapdata::sdpro::LT_IC) {
    route_info_output_.is_ego_on_expressway = true;
    if (current_link_->link_class() ==
        iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY) {
      route_info_output_.is_ego_on_expressway_hmi = true;
    } else {
      route_info_output_.is_ego_on_city_expressway_hmi = true;
    }
    route_info_output_.is_in_sdmaproad = true;
  } else {
    return false;
  }

  return true;
}

void LDRouteInfoStrategy::CalculateMLCDecider(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    RouteInfoOutput& route_info_output) {
  mlc_decider_info_base_baidu_.reset();

  if (relative_id_lanes.empty()) {
    return;
  }
  MLCSceneTypeDecider();

  MLCSceneType mlc_scene_type = mlc_decider_info_base_baidu_.mlc_scene_type;

  route_info_output_.baidu_mlc_scene = mlc_scene_type;

  TopoLinkGraph feasible_lane_graph;
  switch (mlc_scene_type) {
    case SPLIT_SCENE: {
      if (!CalculateFeasibleLaneInRampScene(feasible_lane_graph)) {
        return;
      }
      break;
    }
    case NORMAL_SCENE: {
      if (!CalculateFeasibleLaneInNormalScene(feasible_lane_graph)) {
        return;
      }
      break;
    }
    case MERGE_SCENE: {
      if (!CalculateFeasibleLaneInMergeScene(feasible_lane_graph)) {
        return;
      }
      break;
    }
    case NONE_SCENE:
      break;
  }

  if (route_info_output_.is_ego_on_expressway_hmi) {
    CalculateAvoidMergeFeasibleLane(feasible_lane_graph);
  }

  UpdateLCNumTask(relative_id_lanes, feasible_lane_graph);

  route_info_output = route_info_output_;
}

bool LDRouteInfoStrategy::IsNearingSplit() {
  if (split_info_vec_.empty()) {
    return false;
  }

  bool is_near_split =
      split_info_vec_[0].second <
      mlc_decider_config_
          ->default_pre_triggle_road_to_ramp_distance_threshold_value;
  return is_near_split;
}

bool LDRouteInfoStrategy::IsNearingRamp() {
  if (ramp_info_vec_.empty()) {
    return false;
  }

  const auto& first_ramp_info = ramp_info_vec_[0];
  bool is_near_ramp =
      first_ramp_info.second <
      mlc_decider_config_
          ->default_pre_triggle_road_to_ramp_distance_threshold_value;
  // 如果没有接近ramp，则直接return
  if (!is_near_ramp) {
    return is_near_ramp;
  }

  // 如果没有merge信息，那么可以不需要考虑在自车和ramp之间是否有merge的场景，可以直接return
  if (merge_info_vec_.empty()) {
    mlc_decider_info_base_baidu_.set_value(
        SPLIT_SCENE, CalculateSplitDirection(*first_ramp_info.first, ld_map_),
        first_ramp_info.second);
    return is_near_ramp;
  }

  // 在接近ramp，且有merge信息，需要判断一下是先处理merge场景还是ramp场景
  const double dis_to_ramp = first_ramp_info.second;

  for (const auto& merge_info : merge_info_vec_) {
    if (merge_info.second > (dis_to_ramp - kEpsilon)) {
      continue;
    }

    if (!IsIgnoreMerge(merge_info)) {
      mlc_decider_info_base_baidu_.reset();
      return false;
    }
  }

  mlc_decider_info_base_baidu_.set_value(
      SPLIT_SCENE, CalculateSplitDirection(*first_ramp_info.first, ld_map_),
      first_ramp_info.second);

  return is_near_ramp;
}

bool LDRouteInfoStrategy::IsNearingMerge() {
  if (merge_info_vec_.empty()) {
    return false;
  }

  for (const auto& merge_info: merge_info_vec_) {
    bool dis_condition = merge_info.second < 500;

    bool is_ignore_merge = IsIgnoreMerge(merge_info);

    if (dis_condition && !is_ignore_merge) {
      mlc_decider_info_base_baidu_.set_value(
          MERGE_SCENE, CalculateMergeDirection(*merge_info.first, ld_map_),
          merge_info.second);
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsTwoSplitClose() {
  if (split_info_vec_.size() < 2) {
    return false;
  }

  bool is_two_splits_close =
      split_info_vec_[1].second - split_info_vec_[0].second <
      mlc_decider_config_->split_split_gap_threshold;

  return is_two_splits_close;
}

void LDRouteInfoStrategy::MLCSceneTypeDecider() {
  if (IsNearingRamp()) {
    return;
  }

  if (!IsNearingMerge()) {
    mlc_decider_info_base_baidu_.is_valid = true;
    mlc_decider_info_base_baidu_.mlc_scene_type = NORMAL_SCENE;
  }
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneGraph(
    TopoLinkGraph& feasible_lane_graph,
    const std::vector<iflymapdata::sdpro::Lane>& start_lane_vec,
    const iflymapdata::sdpro::LinkInfo_Link& target_link) {
  // -------------------------- 1. 初始化与输入校验
  feasible_lane_graph.lane_topo_groups.clear();
  if (start_lane_vec.empty()) {
    return false;
  }

  const auto& first_start_lane = start_lane_vec.front();
  const auto* start_link = ld_map_.GetLinkOnRoute(first_start_lane.link_id());
  if (!start_link) {
    return false;
  }

  std::vector<iflymapdata::sdpro::Lane> current_lane_vec = start_lane_vec;
  if (!SortLaneBaseSeq(current_lane_vec)) {
    return false;
  }

  // 关键变量：避免死循环（记录已处理的link ID，防止拓扑环导致无限遍历）
  std::unordered_set<uint64_t> processed_link_ids;
  bool is_target_found = false;
  const uint64_t target_link_id = target_link.id();

  // 增加计算front feasible distance
  double front_sum_distance = 0;

  // -------------------------- 2.遍历拓扑构建车道图
  const iflymapdata::sdpro::LinkInfo_Link* current_link = start_link;
  while (current_link != nullptr) {
    if (processed_link_ids.count(current_link->id()) > 0) {
      return false;
    }

    if (current_link->id() == current_link_->id()) {
      front_sum_distance = front_sum_distance + current_link->length() * 0.01 -
                           ego_on_cur_link_s_;
    } else {
      front_sum_distance = front_sum_distance + current_link->length() * 0.01;
    }

    processed_link_ids.insert(current_link->id());

    // -------------------------- 3.构建当前link的车道拓扑组
    LaneTopoGroup current_topo_group;
    current_topo_group.link_id = current_link->id();
    current_topo_group.lane_nums = current_link->lane_num();

    for (const auto& lane : current_lane_vec) {
      if (lane.link_id() != current_link->id()) {
        return false;
      }

      TopoLane topo_lane;
      topo_lane.id = lane.id();
      topo_lane.link_id = lane.link_id();
      topo_lane.order_id = lane.sequence();
      topo_lane.length = lane.length() * 0.01;
      topo_lane.front_feasible_distance = front_sum_distance;

      // 更新当前topo lane的前继车道
      const int pre_lane_count = lane.predecessor_lane_ids_size();
      if (pre_lane_count == 0) {
        topo_lane.predecessor_lane_ids.clear();
      } else if (pre_lane_count == 1) {
        topo_lane.predecessor_lane_ids.emplace(lane.predecessor_lane_ids()[0]);
      } else {
        for (const auto& pre_lane_id : lane.predecessor_lane_ids()) {
          const auto* pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
          if (pre_lane == nullptr) {
            continue;
          }

          // 处理合流车道：判断是否有效，无效则跳过
          if (IsMergeLane(pre_lane)) {
            if (IsInvalidLaneMergeLaneOppositeSide(pre_lane)) {
              continue;
              // merge lane的反方向是无效车道，既这条merge lane不应该在feasible lane中，continue掉，不加入到feasible lane中
            }
          }

          topo_lane.predecessor_lane_ids.emplace(pre_lane_id);
        }
      }

      // 更新当前topo lane的后继车道
      topo_lane.successor_lane_ids.clear();
      for (int i = 0; i < lane.successor_lane_ids_size(); ++i) {
        topo_lane.successor_lane_ids.emplace(lane.successor_lane_ids()[i]);
      }

      current_topo_group.topo_lanes.emplace_back(std::move(topo_lane));
    }

    // -------------------------- 4. 检查目标link并更新输出
    if (current_link->id() == target_link_id) {
      is_target_found = true;
    }
    if (current_topo_group.topo_lanes.empty()) {
      return false;
    }
    feasible_lane_graph.lane_topo_groups.emplace_back(
        std::move(current_topo_group));

    if (is_target_found) {
      return true;
    }

    // ------------------------- 5. 获取下一个pre link并更新车道列表
    const auto* next_pre_link =
        ld_map_.GetPreviousLinkOnRoute(current_link->id());
    if (!next_pre_link) {
      return false;
    }

    std::vector<iflymapdata::sdpro::Lane> next_lane_vec;
    if (feasible_lane_graph.lane_topo_groups.empty()) {
      return false;
    }

    for (const auto& topo_lane :
         feasible_lane_graph.lane_topo_groups.back().topo_lanes) {
      if (topo_lane.predecessor_lane_ids.empty()) {
        continue;
      }

      const uint64_t pre_lane_id = *topo_lane.predecessor_lane_ids.begin();
      const auto* pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
      if (!pre_lane) {
        return false;
      }

      // 校验前继车道是否属于routelink的pre link
      if (pre_lane->link_id() == next_pre_link->id()) {
        HandleMainLinkPreLane(pre_lane, next_lane_vec);
      } else {
        // 需要处理前继lane不是在route link上的场景
        HandleOtherMergeLinkPreLane(topo_lane, next_pre_link, current_link,
                                    pre_lane, next_lane_vec);
      }
    }

    if (next_lane_vec.empty()) {
      return false;
    }

    // -------------------------- 6. 更新迭代变量，进入下一轮循环
    current_lane_vec = std::move(next_lane_vec);
    current_link = next_pre_link;
  }

  return false;
}

bool LDRouteInfoStrategy::IsValidInputLanes(
    const iflymapdata::sdpro::LinkInfo_Link* link,
    const std::vector<iflymapdata::sdpro::Lane>& start_lane_vec) {
  if (start_lane_vec.empty()) {
    return false;
  }

  bool is_same_link = true;
  uint64 link_id;

  if (start_lane_vec.size() == 1) {
    link = ld_map_.GetLinkOnRoute(start_lane_vec[0].link_id());
    return true;
  }

  for (int i = 1; i < start_lane_vec.size(); i++) {
    if (start_lane_vec[i].link_id() != start_lane_vec[i - 1].link_id()) {
      return false;
    }
  }

  link = ld_map_.GetLinkOnRoute(start_lane_vec[0].link_id());
  return true;
}

bool LDRouteInfoStrategy::SortLaneBaseSeq(
    std::vector<iflymapdata::sdpro::Lane>& start_lane_vec) {
  if (start_lane_vec.empty()) {
    return false;
  }

  std::sort(start_lane_vec.begin(), start_lane_vec.end(),
            [](const iflymapdata::sdpro::Lane& lane_a,
               const iflymapdata::sdpro::Lane& lane_b) {
              return lane_a.sequence() < lane_b.sequence();
            });

  return true;
}

bool LDRouteInfoStrategy::CalculateExtenedFeasibleLane(
    TopoLinkGraph& before_split_feasible_lane_graph) {
  if (before_split_feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  const int link_size =
      before_split_feasible_lane_graph.lane_topo_groups.size();
  double link_sum_dis = 0;

  const double v_limit =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
  const double kResponseOffset = 300.;

  std::array<double, 3> xp{11.111, 22.222, 33.333};
  std::array<double, 3> fp{300.0, 600.0, 1000.0};
  const double adaptor_interval = interp(v_limit, xp, fp);

  for (int i = 0; i < link_size; i++) {
    const auto& temp_link_topo =
        before_split_feasible_lane_graph.lane_topo_groups[i];

    const auto& temp_topo_lanes = temp_link_topo.topo_lanes;

    if (temp_topo_lanes.empty()) {
      return false;
    }

    const auto& temp_link = ld_map_.GetLinkOnRoute(temp_topo_lanes[0].link_id);
    if (temp_link == nullptr) {
      return false;
    }

    // 校验前面计算的feasible lane是否正确的,计算出feasible的范围
    int min_seq = temp_topo_lanes[0].order_id;
    int max_seq = temp_topo_lanes[0].order_id;
    for (const auto& temp_topo_lane : temp_topo_lanes) {
      if (temp_topo_lane.link_id != temp_link->id()) {
        return false;
      }
      if (temp_topo_lane.order_id < min_seq) {
        min_seq = temp_topo_lane.order_id;
      }

      if (temp_topo_lane.order_id > max_seq) {
        max_seq = temp_topo_lane.order_id;
      }
    }

    if (temp_link->id() == current_link_->id()) {
      link_sum_dis =
          link_sum_dis + temp_topo_lanes[0].length - ego_on_cur_link_s_;
    } else {
      link_sum_dis = link_sum_dis + temp_topo_lanes[0].length;
    }

    for (const auto& lane_id : temp_link->lane_ids()) {
      const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
      if (temp_lane == nullptr) {
        continue;
      }

      // 根据车道类型判断车道是否有效
      if (IsInvalidLane(temp_lane)) {
        continue;
      }

      int lc_num = 0;
      if (temp_lane->sequence() <= max_seq &&
          temp_lane->sequence() >= min_seq) {
        continue;
      } else if (temp_lane->sequence() < min_seq) {
        lc_num = min_seq - temp_lane->sequence();
      } else if (temp_lane->sequence() > max_seq) {
        lc_num = temp_lane->sequence() - max_seq;
      }

      const double lc_need_dis =
          kResponseOffset + adaptor_interval * std::fabs(lc_num);

      if (link_sum_dis < lc_need_dis) {
        // 距离ramp点的距离小于变道需要的距离，因此该lane不需要加入进去
        continue;
      }

      if (!IsLaneSuccessorInPlannedRoute(temp_lane)) {
        continue;
      }

      if (IsLaneSuccessorIsMergeLane(temp_lane)) {
        continue;
      }

      TopoLane topo_lane;
      topo_lane.id = temp_lane->id();
      topo_lane.link_id = temp_lane->link_id();
      topo_lane.order_id = temp_lane->sequence();
      topo_lane.length = temp_lane->length() * 0.01;  // cm to m
      topo_lane.front_feasible_distance = link_sum_dis - lc_need_dis;

      // 把符合条件的lane更新到车道组里面去
      before_split_feasible_lane_graph
          .lane_topo_groups[i].link_id = temp_link->id();
      before_split_feasible_lane_graph
          .lane_topo_groups[i].lane_nums = temp_link->lane_num();
      before_split_feasible_lane_graph
          .lane_topo_groups[i]
          .topo_lanes.emplace_back(std::move(topo_lane));

    }
  }

  return true;
}

void LDRouteInfoStrategy::UpdateLCNumTask(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const TopoLinkGraph& feasible_lane_graph) {
  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return;
  }

  const auto& cur_link_feasible_lane =
      feasible_lane_graph.lane_topo_groups.back();
  if (cur_link_feasible_lane.topo_lanes.empty()) {
    return;
  }

  // 计算出原始的order
  std::vector<std::pair<int, double>> origin_order_id_seq;
  origin_order_id_seq.reserve(cur_link_feasible_lane.topo_lanes.size());
  for (const auto& topo_lane : cur_link_feasible_lane.topo_lanes) {
    origin_order_id_seq.emplace_back(topo_lane.order_id,
                                     topo_lane.front_feasible_distance);
  }
  if (origin_order_id_seq.empty()) {
    return;
  }

  // 继续判断是否有导流区车道，如果有的话，需要更新origin_order_id_seq
  int diversion_lane_num = 0;
  int most_left_emergency_lane_num = 0;
  const int cur_lane_size = current_link_->lane_ids_size();
  for (int i = cur_lane_size - 1; i >= 0; i--) {
    const auto& lane_id = current_link_->lane_ids()[i];
    const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (temp_lane == nullptr) {
      continue;
    }

    if (IsDiversionLane(temp_lane)) {
      diversion_lane_num++;
      const int origin_order_size = origin_order_id_seq.size();
      for (int j = origin_order_size - 1; j >= 0; j--) {
        if (origin_order_id_seq[j].first > temp_lane->sequence()) {
          origin_order_id_seq[j].first = origin_order_id_seq[j].first - 1;
        }
      }
    }

    // 默认紧急车道在最右侧，
    // 增加应急车道在左边时，计算车道从左向右序号的逻辑
    if (IsEmergencyLane(temp_lane) &&
        temp_lane->sequence() == cur_lane_size) {
      most_left_emergency_lane_num++;
    }
  }

  const int link_total_lane_num = cur_link_feasible_lane.lane_nums -
                                  diversion_lane_num -
                                  most_left_emergency_lane_num;

  if (link_total_lane_num < 1) {
    return;
  }

  std::vector<std::pair<int, double>> feasible_lane_seq;
  feasible_lane_seq.reserve(origin_order_id_seq.size());
  for (const auto& order_id : origin_order_id_seq) {
    // 把从右向左的顺序转换成从左向右的顺序
    const int seq = link_total_lane_num - order_id.first + 1;
    if (seq > 0) {
      feasible_lane_seq.emplace_back(seq, order_id.second);
    }
  }

  if (feasible_lane_seq.empty()) {
    return;
  }
  int minVal_seq = feasible_lane_seq[0].first;
  int maxVal_seq = feasible_lane_seq[0].first;

  std::vector<int> feasible_lane_seq_vec;
  feasible_lane_seq_vec.reserve(feasible_lane_seq.size());
  for (const auto& num : feasible_lane_seq) {
    if (num.first < minVal_seq) {
      minVal_seq = num.first;
    }
    if (num.first > maxVal_seq) {
      maxVal_seq = num.first;
    }
    feasible_lane_seq_vec.emplace_back(num.first);
  }

  route_info_output_.maxVal_seq = maxVal_seq;
  route_info_output_.minVal_seq = minVal_seq;
  route_info_output_.mlc_decider_route_info.feasible_lane_sequence =
      std::move(feasible_lane_seq_vec);

  std::unordered_map<int, double> feasible_lane_seq_map;
  for (const auto& temp_feasible_lane_seq : feasible_lane_seq) {
    feasible_lane_seq_map.insert(
        {temp_feasible_lane_seq.first, temp_feasible_lane_seq.second});
  }

  for (auto relative_id_lane : relative_id_lanes) {
    // （fengwang31）TODO:后面把这个函数与route_info中的统一起来
    ProcessLaneDistance(relative_id_lane, feasible_lane_seq_map);
  }

  for (auto relative_id_lane : relative_id_lanes) {
    if (relative_id_lane == nullptr) {
      continue;
    }

    if (relative_id_lane->get_relative_id() != 0) {
      continue;
    }

    // 计算当前位置感知提供的车道数，当前感知提供的车道数是默认包含了右边的应急车道的
    const auto& lane_nums = relative_id_lane->get_lane_nums();
    int left_lane_num = 0;
    int right_lane_num = 0;
    for (const auto& lane_num : lane_nums) {
      if (lane_num.end > kEpsilon) {
        left_lane_num = lane_num.left_lane_num;
        right_lane_num = lane_num.right_lane_num;
        break;
      }
    }

    route_info_output_.left_lane_num = left_lane_num;
    route_info_output_.right_lane_num = right_lane_num;

    int real_lane_num = link_total_lane_num;
    // 判断是否有应急车道、加速车道、入口车道
    bool cur_link_is_exist_emergency_lane = false;
    bool cur_link_is_exist_accelerate_lane = false;
    bool cur_link_is_exist_entry_lane = false;
    for (const auto& lane_id : current_link_->lane_ids()) {
      const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
      if (lane == nullptr) {
        continue;
      }

      if (IsEmergencyLane(lane)) {
        // 先假设应急车道都在最右边上，因此seq=1
        if (lane->sequence() == 1) {
          real_lane_num = link_total_lane_num - 1;
        }
        cur_link_is_exist_emergency_lane = true;
      }

      if (IsAccelerateLane(lane)) {
        cur_link_is_exist_accelerate_lane = true;
      }

      if (IsEntryLane(lane)) {
        cur_link_is_exist_entry_lane = true;
      }
    }

    std::vector<int> lc_num_task;

    bool is_nearing_ramp = false;
    const bool is_exist_ramp =
        !ramp_info_vec_.empty() && !split_info_vec_.empty();
    if (is_exist_ramp) {
      const auto& first_ramp = ramp_info_vec_[0].first;
      const auto& first_split = split_info_vec_[0].first;

      if (first_ramp && first_split) {
        is_nearing_ramp = (route_info_output_.baidu_mlc_scene == SPLIT_SCENE) &&
                          (first_ramp->id() == first_split->id());
      }
    }

    const bool lane_type_condition =
        !cur_link_is_exist_accelerate_lane && !cur_link_is_exist_entry_lane;
    RampDirection front_ramp_dir = route_info_output_.ramp_direction;
    if (maxVal_seq == minVal_seq && maxVal_seq == real_lane_num &&
        is_nearing_ramp && lane_type_condition &&
        front_ramp_dir == RAMP_ON_RIGHT) {
      //split场景，目标车道在最右边的情况，一直向右变道
      // 右边有加速车道或入口车道则需要至少留一个车道
      lc_num_task.emplace_back(1);
    } else if (maxVal_seq == minVal_seq && maxVal_seq == 1 && is_nearing_ramp) {
      //split场景，目标车道在最左边的情况，一直向左变道
      lc_num_task.emplace_back(-1);
    } else {
      int ego_seq = left_lane_num + 1;
      if (ego_seq >= minVal_seq && ego_seq <= maxVal_seq) {
        continue;
      } else if (ego_seq > maxVal_seq) {
        int err = ego_seq - maxVal_seq;
        for (int i = 0; i < err; i++) {
          lc_num_task.emplace_back(-1);
        }
      } else if (ego_seq < minVal_seq) {
        int err = minVal_seq - ego_seq;
        for (int i = 0; i < err; i++) {
          lc_num_task.emplace_back(1);
        }
      }
    }

    if (lc_num_task.empty()) {
      return;
    }

    relative_id_lane->set_current_tasks(lc_num_task);
  }

  return;
}

bool LDRouteInfoStrategy::CalculateFrontTargetLinkBaseFixDis(
    iflymapdata::sdpro::LinkInfo_Link* target_link,
    std::vector<iflymapdata::sdpro::Lane>& start_lane_vec,
    const iflymapdata::sdpro::LinkInfo_Link* cur_link,
    const MLCSceneType scene) {
  if (cur_link == nullptr) {
    return false;
  }

  double front_search_dis = 500.0;
  double sum_dis = 0.0;
  if (scene == NORMAL_SCENE) {
    sum_dis = cur_link->length() * 0.01 - ego_on_cur_link_s_;
    front_search_dis = 2000.0;
    // 进入normal场景，有2种情况，
    // 1、不在nearing ramp场景，前面3km内都没有ramp和需要并入的merge，这种情况下向前搜索2000m是合理的
    // 2、不在nearing merge场景,前面500m内，没有需要并入的merge,只能保证在500m内是normal。
    // 此时向前搜索的距离应该在500m内才是合理的，因此需要在此判断更新这个向前搜索的距离值。
    if (!merge_info_vec_.empty()) {
      for (const auto& merge_info : merge_info_vec_) {
        if (!IsIgnoreMerge(merge_info) &&
            merge_info.second <
                mlc_decider_config_
                    ->default_pre_triggle_road_to_ramp_distance_threshold_value) {
          front_search_dis = 500.0;
        }
      }
    }
  } else {
    sum_dis = cur_link->length() * 0.01;
  }
  const iflymapdata::sdpro::LinkInfo_Link* temp_link = cur_link;

  if (temp_link == nullptr) {
    return false;
  }

  while (sum_dis < front_search_dis) {
    const auto& temp_next_link = ld_map_.GetNextLinkOnRoute(temp_link->id());
    if (temp_next_link == nullptr) {
      break;
    }

    temp_link = temp_next_link;
    sum_dis = sum_dis + temp_next_link->length() * 0.01;
  }

  for (const auto& lane_id : temp_link->lane_ids()) {
    const iflymapdata::sdpro::Lane* lane_info =
        ld_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr || IsEmergencyLane(lane_info)) {
      continue;
    }
    start_lane_vec.emplace_back(*lane_info);
  }

  if (start_lane_vec.empty()) {
    return false;
  }

  *target_link = *temp_link;
  return true;
}

bool LDRouteInfoStrategy::CalculateMergePreFeasibleLane(
    std::vector<iflymapdata::sdpro::Lane>& merge_pre_link_lane_vec,
    const TopoLinkGraph& feasible_lane_graph,
    const iflymapdata::sdpro::LinkInfo_Link* merge_link) {
  if (merge_link == nullptr) {
    return false;
  }

  const auto& merge_pre_link = ld_map_.GetPreviousLinkOnRoute(merge_link->id());
  if (merge_pre_link == nullptr) {
    return false;
  }

  const auto& merge_link_feasible_lane =
      feasible_lane_graph.lane_topo_groups.back();

  for (const auto& feasible_lane : merge_link_feasible_lane.topo_lanes) {
    const auto lane_info =
        ld_map_.GetLaneInfoByID(*feasible_lane.predecessor_lane_ids.begin());
    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->link_id() == merge_pre_link->id()) {
      merge_pre_link_lane_vec.emplace_back(*lane_info);
    }
  }

  if (merge_pre_link_lane_vec.empty()) {
    std::vector<int> merge_link_feasible_lane_order_id_seq;
    merge_link_feasible_lane_order_id_seq.reserve(
        merge_link_feasible_lane.topo_lanes.size());
    for (const auto& feasible_lane : merge_link_feasible_lane.topo_lanes) {
      merge_link_feasible_lane_order_id_seq.emplace_back(
          feasible_lane.order_id);
    }
    std::sort(merge_link_feasible_lane_order_id_seq.begin(),
              merge_link_feasible_lane_order_id_seq.end());

    std::vector<std::pair<int, iflymapdata::sdpro::Lane>> first_merge_lane_info;
    first_merge_lane_info.reserve(merge_link->lane_ids().size());
    for (const auto& lane_id : merge_link->lane_ids()) {
      const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
      if (temp_lane == nullptr) {
        continue;
      }

      first_merge_lane_info.emplace_back(temp_lane->sequence(), *temp_lane);
    }

    if (first_merge_lane_info.empty()) {
      return false;
    }

    std::sort(first_merge_lane_info.begin(), first_merge_lane_info.end(),
              [](const std::pair<int, iflymapdata::sdpro::Lane>& a,
                 const std::pair<int, iflymapdata::sdpro::Lane>& b) {
                return a.first < b.first;  // 按 first 升序排列
              });

    // 先检查右边的车道
    const int min_seq = first_merge_lane_info.front().first;
    const int max_seq = first_merge_lane_info.back().first;

    const int min_order = merge_link_feasible_lane_order_id_seq.front();
    if (min_order <= max_seq && min_order >= min_seq) {
      for (int i = min_order; i > 0; i--) {
        for (const auto& [seq, lane] : first_merge_lane_info) {
          if (seq != (i - 1)) {
            continue;
          }

          for (const auto& pre_lane_id : lane.predecessor_lane_ids()) {
            const auto& pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
            if (pre_lane == nullptr) {
              continue;
            }

            if (pre_lane->lane_transiton() != iflymapdata::sdpro::LTS_MERGE &&
                pre_lane->link_id() == merge_pre_link->id()) {
              merge_pre_link_lane_vec.emplace_back(*pre_lane);

              return true;
            }
          }
        }
      }
    }

    // 还是没有找到pre_lane,检查左边的车道
    if (merge_pre_link_lane_vec.empty()) {
      const int max_order = merge_link_feasible_lane_order_id_seq.back();
      if (max_order <= max_seq && max_order >= min_seq) {
        for (int i = max_order; i < max_seq; i++) {
          for (const auto& [seq, lane] : first_merge_lane_info) {
            if (seq != (i + 1)) {
              continue;
            }

            for (const auto& pre_lane_id : lane.predecessor_lane_ids()) {
              const auto& pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
              if (pre_lane == nullptr) {
                continue;
              }

              if (pre_lane->lane_transiton() != iflymapdata::sdpro::LTS_MERGE &&
                  pre_lane->link_id() == merge_pre_link->id()) {
                merge_pre_link_lane_vec.emplace_back(*pre_lane);

                return true;
              }
            }
          }
        }
      }
    }
  }

  return true;
}

bool LDRouteInfoStrategy::IsEmergencyLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_EMERGENCY) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsAccelerateLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_ACCELERATE) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsDecelerateLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_DECELERATE) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsEntryLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_ENTRY) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsExitLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_EXIT) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsMergeLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  return lane_info->lane_connection() == iflymapdata::sdpro::LAN_STATUS_MERGING;
}

bool LDRouteInfoStrategy::IsDiversionLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_DIVERSION) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::HasLaneId(
    const std::vector<iflymapdata::sdpro::Lane>& lane_vec,
    uint64 target_id) const {
  // 使用 std::find_if 查找是否有 lane 的 id 等于 target_id
  auto it = std::find_if(lane_vec.begin(), lane_vec.end(),
                         [target_id](const iflymapdata::sdpro::Lane& lane) {
                           return lane.id() == target_id;
                         });
  return it != lane_vec.end();
}

bool LDRouteInfoStrategy::IsInvalidLane(
    const iflymapdata::sdpro::Lane* temp_lane) const {
  bool is_emergency_lane = IsEmergencyLane(temp_lane);
  bool is_merge_lane = IsMergeLane(temp_lane);
  bool is_accelerate_lane = IsAccelerateLane(temp_lane);
  bool is_entry_lane = IsEntryLane(temp_lane);
  bool is_diversion_lane = IsDiversionLane(temp_lane);

  // 1、考虑有汇入车道的场景
  if (is_diversion_lane || is_emergency_lane || is_merge_lane ||
      is_accelerate_lane || is_entry_lane) {
    return true;
  }

  // 2、考虑自车当前位置与目标split之间存在分流的场景
  bool is_exit_lane = IsExitLane(temp_lane);
  bool is_decelerate_lane = IsDecelerateLane(temp_lane);
  if (is_decelerate_lane || is_exit_lane) {
    if (route_info_output_.baidu_mlc_scene == SPLIT_SCENE) {
      const double dis_to_ramp = route_info_output_.dis_to_ramp;
      for (const auto& split_info : split_info_vec_) {
        if (split_info.second < dis_to_ramp - kEpsilon) {
          return true;
        }
      }
    }
    // todo:merge场景、normal场景
  }

  return false;
}

bool LDRouteInfoStrategy::IsIgnoreMerge(
    const std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>&
        merge_info) const {
  const auto& merge_link = merge_info.first;
  if (merge_link == nullptr) {
    return true;
  }

  // 用link上的车道数作为判断可以ignore的条件：
  // 1、在merge
  // link的pre_link上车道数多的link认为是主路，车道数少的link认为是merge进来的；
  // 2、车道数相等的情况下，判断哪边有车道收窄的，则认为收窄那边是merge的，另一边则是主路

  const uint64 merge_link_id = merge_link->id();

  const auto& merge_pre_link = ld_map_.GetPreviousLinkOnRoute(merge_link_id);
  if (merge_pre_link == nullptr) {
    return true;
  }

  if (merge_link->predecessor_link_ids_size() != 2) {
    return true;
  }

  const uint64 other_merge_link_id =
      merge_link->predecessor_link_ids()[0] == merge_pre_link->id()
          ? merge_link->predecessor_link_ids()[1]
          : merge_link->predecessor_link_ids()[0];
  const auto& other_merge_link = ld_map_.GetLinkOnRoute(other_merge_link_id);

  if (other_merge_link == nullptr) {
    return true;
  }

  int other_merge_link_lane_num = 0;
  for (const auto& lane_id : other_merge_link->lane_ids()) {
    const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
    if (!IsEmergencyLane(lane)) {
      other_merge_link_lane_num++;
    }
  }

  int merge_pre_link_lane_num = 0;
  for (const auto& lane_id : merge_pre_link->lane_ids()) {
    const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
    if (!IsEmergencyLane(lane)) {
      merge_pre_link_lane_num++;
    }
  }

  if (merge_pre_link_lane_num > other_merge_link_lane_num) {
    return true;
  } else if (merge_pre_link_lane_num < other_merge_link_lane_num) {
    return false;
  } else {
    // TODO:后续补充根据收窄车道的判断条件
    return true;
  }

  return true;
}

void LDRouteInfoStrategy::CalculateMergeInfo() {
  merge_info_vec_ = ld_map_.GetMergeInfoList(
      current_link_->id(), ego_on_cur_link_s_, kMaxSearchLength);
  // 筛选掉距离为负的merge信息
  while (!merge_info_vec_.empty()) {
    if (merge_info_vec_.front().second > kEpsilon) {
      break;
    } else {
      merge_info_vec_.erase(merge_info_vec_.begin());
    }
  }

  if (merge_info_vec_.empty()) {
    return;
  }

  route_info_output_.first_merge_direction =
      CalculateMergeDirection(*merge_info_vec_[0].first, ld_map_);
  route_info_output_.distance_to_first_road_merge = merge_info_vec_[0].second;
}

void LDRouteInfoStrategy::CalculateSplitInfo() {
  split_info_vec_ = ld_map_.GetSplitInfoList(
      current_link_->id(), ego_on_cur_link_s_, kMaxSearchLength);
  // 筛选掉距离为负的merge信息
  while (!split_info_vec_.empty()) {
    if (split_info_vec_.front().second > kEpsilon) {
      break;
    } else {
      split_info_vec_.erase(split_info_vec_.begin());
    }
  }

  if (split_info_vec_.empty()) {
    return;
  }

  route_info_output_.distance_to_first_road_split = split_info_vec_[0].second;
  route_info_output_.first_split_direction =
      CalculateSplitDirection(*split_info_vec_[0].first, ld_map_);

  // for xykuai，这里有重复定义变量的问题，后续删掉
  route_info_output_.first_split_dir_dis_info =
      std::make_pair(static_cast<SplitRelativeDirection>(
                         route_info_output_.first_split_direction),
                     route_info_output_.distance_to_first_road_split);

  // 增加输出split_region_info_list，给纵向做接近匝道预减速
  route_info_output_.split_region_info_list.reserve(split_info_vec_.size());
  for (const auto& split_info : split_info_vec_) {
    NOASplitRegionInfo split_region_info;
    split_region_info.distance_to_split_point = split_info.second;
    split_region_info.split_link_id = split_info.first->id();
    split_region_info.split_direction = static_cast<SplitDirection>(
        CalculateSplitDirection(*split_info.first, ld_map_));

    route_info_output_.split_region_info_list.emplace_back(
        std::move(split_region_info));
  }
}

void LDRouteInfoStrategy::CalculateRampInfo() {
  for (const auto& split_info : split_info_vec_) {
    uint64 split_link_id = split_info.first->id();
    const auto& split_next_link = ld_map_.GetNextLinkOnRoute(split_link_id);
    if (split_next_link == nullptr) {
      continue;
    }

    if (ld_map_.isRamp(split_next_link->link_type())) {
      ramp_info_vec_.emplace_back(split_info);
    }
  }

  if (ramp_info_vec_.empty()) {
    return;
  }

  route_info_output_.dis_to_ramp = ramp_info_vec_[0].second;
  route_info_output_.ramp_direction =
      CalculateSplitDirection(*ramp_info_vec_[0].first, ld_map_);
}

bool LDRouteInfoStrategy::get_sdpromap_valid() { return ldmap_valid_; }

const ad_common::sdpromap::SDProMap& LDRouteInfoStrategy::get_sdpro_map() {
  return ld_map_;
}

const iflymapdata::sdpro::LinkInfo_Link* LDRouteInfoStrategy::get_current_link() {
  return current_link_;
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneInRampScene(
    TopoLinkGraph& feasible_lane_graph) {
  // TopoLinkGraph before_split_feasible_lane_graph;
  TopoLinkGraph after_feasible_lane_graph;
  std::vector<iflymapdata::sdpro::Lane> start_lane_vec;

  if (ramp_info_vec_.empty()) {
    return false;
  }
  const iflymapdata::sdpro::LinkInfo_Link* split_link = ramp_info_vec_[0].first;
  if (split_link == nullptr) {
    return false;
  }


  if (route_info_output_.is_on_ramp) {
    split_link = FindFrontValidRampSplitLink();
    if (split_link == nullptr) {
      return false;
    }
  }

  const iflymapdata::sdpro::LinkInfo_Link* target_link =
      ld_map_.GetNextLinkOnRoute(split_link->id());
  if (target_link == nullptr) {
    return false;
  }

  for (const auto& lane_id : target_link->lane_ids()) {
    const iflymapdata::sdpro::Lane* lane_info =
        ld_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr || IsEmergencyLane(lane_info) ||
        IsDiversionLane(lane_info)) {
      continue;
    }
    start_lane_vec.emplace_back(*lane_info);
  }

  if (!CalculateFeasibleLaneGraph(after_feasible_lane_graph, start_lane_vec,
                                  *target_link)) {
    return false;
  }

  // 计算split_next_link上pre_lane在split_link上的lane
  std::vector<iflymapdata::sdpro::Lane> split_link_lane_vec;
  if (after_feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  const auto& split_next_link_lane_info =
      after_feasible_lane_graph.lane_topo_groups.back();
  for (const auto& topo_lane : split_next_link_lane_info.topo_lanes) {
    if (topo_lane.predecessor_lane_ids.empty()) {
      continue;
    }
    const auto lane_info =
        ld_map_.GetLaneInfoByID(*topo_lane.predecessor_lane_ids.begin());
    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->link_id() == split_link->id()) {
      split_link_lane_vec.emplace_back(*lane_info);
    }
  }

  // 反向遍历得到从当前link的lane能直达split_link上targte_lane的feasible
  // lane
  if (!CalculateFeasibleLaneGraph(feasible_lane_graph, split_link_lane_vec,
                                  *current_link_)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 再次反向遍历横向上扩展feasible lane
  // 根据距离把可行驶车道加上
  if (!CalculateExtenedFeasibleLane(feasible_lane_graph)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  return true;
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneInMergeScene(
    TopoLinkGraph& feasible_lane_graph) {
  // 1、计算target link
  if (merge_info_vec_.empty()) {
    return false;
  }
  const auto& first_merge_link_info = merge_info_vec_[0].first;
  const auto& merge_pre_link =
      ld_map_.GetPreviousLinkOnRoute(first_merge_link_info->id());
  if (merge_pre_link == nullptr) {
    return false;
  }

  iflymapdata::sdpro::LinkInfo_Link first_merge_link = *first_merge_link_info;
  iflymapdata::sdpro::LinkInfo_Link* target_link = &first_merge_link;
  std::vector<iflymapdata::sdpro::Lane> start_lane_vec;
  if (!CalculateFrontTargetLinkBaseFixDis(target_link, start_lane_vec,
                                          first_merge_link_info,
                                          route_info_output_.baidu_mlc_scene)) {
    return false;
  }

  // 2、计算merge之后的feasible lane
  TopoLinkGraph after_merge_feasible_lane_graph;
  if (!CalculateFeasibleLaneGraph(after_merge_feasible_lane_graph,
                                  start_lane_vec, *first_merge_link_info)) {
    return false;
  }

  if (after_merge_feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 3、计算merge_pre_link上的feasible lane
  std::vector<iflymapdata::sdpro::Lane> merge_pre_link_feasible_lane;
  if (!CalculateMergePreFeasibleLane(merge_pre_link_feasible_lane,
                                     after_merge_feasible_lane_graph,
                                     first_merge_link_info)) {
    return false;
  }

  // 4、反向遍历至自车当前位置，计算feasible lane
  // TopoLinkGraph before_merge_feasible_lane_graph;
  if (!CalculateFeasibleLaneGraph(
          feasible_lane_graph, merge_pre_link_feasible_lane, *current_link_)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 5、再次反向遍历横向上扩展feasible lane
  // 根据距离把可行驶车道加上
  if (!CalculateExtenedFeasibleLane(feasible_lane_graph)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  return true;
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneInNormalScene(
    TopoLinkGraph& feasible_lane_graph) {
  // 1、 确定target_link
  iflymapdata::sdpro::LinkInfo_Link current_link = *current_link_;
  iflymapdata::sdpro::LinkInfo_Link* target_link = &current_link;
  std::vector<iflymapdata::sdpro::Lane> start_lane_vec;
  if (!CalculateFrontTargetLinkBaseFixDis(target_link, start_lane_vec,
                                          current_link_,
                                          route_info_output_.baidu_mlc_scene)) {
    return false;
  }

  // 2、计算feasible lane
  // TopoLinkGraph normal_feasible_lane_graph;
  if (!CalculateFeasibleLaneGraph(feasible_lane_graph, start_lane_vec,
                                  *current_link_)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 再次反向遍历横向上扩展feasible lane
  // 根据距离把可行驶车道加上
  if (!CalculateExtenedFeasibleLane(feasible_lane_graph)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  return true;
}

void LDRouteInfoStrategy::ProcessLaneDistance(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::unordered_map<int, double>& feasible_lane_distance) {
  if (relative_id_lane == nullptr) {
    return;
  }

  const auto& lane_nums = relative_id_lane->get_lane_nums();
  int left_lane_num = 0;

  for (const auto& lane_num : lane_nums) {
    if (lane_num.end > kEpsilon) {
      left_lane_num = lane_num.left_lane_num;
      break;
    }
  }

  auto it = feasible_lane_distance.find(left_lane_num + 1);
  std::pair<bool, double> virtual_lane_distance;

  if (it != feasible_lane_distance.end()) {
    virtual_lane_distance = std::make_pair(true, it->second);
  } else {
    virtual_lane_distance = std::make_pair(false, 0.0);
  }

  relative_id_lane->set_feasible_lane_distance(virtual_lane_distance);
}

void LDRouteInfoStrategy::CaculateDistanceToRoadEnd(
    const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s) {
  if (segment == nullptr) {
    return;
  }

  double dis_to_end = NL_NMAX;
  int result =
      ld_map_.GetDistanceToRouteEnd(segment->id(), nearest_s, dis_to_end);
  if (result == 0) {
    route_info_output_.distance_to_route_end = dis_to_end;
  } else {
    route_info_output_.distance_to_route_end = NL_NMAX;
  }
}

void LDRouteInfoStrategy::CaculateDistanceToTollStation(
    const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s) {
  if (segment == nullptr) {
    return;
  }

  const auto& toll_station_info =
      ld_map_.GetTollStationInfo(segment->id(), nearest_s, kMaxSearchLength);
  if (toll_station_info.first != nullptr) {
    route_info_output_.distance_to_toll_station = toll_station_info.second;
    route_info_output_.is_exist_toll_station = true;
  } else {
    route_info_output_.distance_to_toll_station = NL_NMAX;
    route_info_output_.is_exist_toll_station = false;
  }
}

bool LDRouteInfoStrategy::IsLaneSuccessorInPlannedRoute(
    const iflymapdata::sdpro::Lane* lane_info) {
  if (lane_info == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::Lane* iterator_lane = lane_info;
  const double kFrontDis = 500.0;
  double sum_dis = 0;

  while (iterator_lane) {
    if (!ld_map_.isOnRouteLinks(iterator_lane->link_id())) {
      return false;
    }

    if (iterator_lane->successor_lane_ids_size() != 1) {
      // 由于目前1分2车道检测不稳定，后继不为1的直接认为不在route上，不会被放进feasible lane中
      // 后续根据测试效果，确定是否需要更精确的判断
      return false;
    }

    const uint64 successor_lane_id = iterator_lane->successor_lane_ids()[0];
    const auto& successor_lane = ld_map_.GetLaneInfoByID(successor_lane_id);
    if (successor_lane == nullptr) {
      return false;
    }

    if (iterator_lane->link_id() == current_link_->id()) {
      sum_dis = sum_dis + iterator_lane->length() * 0.01 - ego_on_cur_link_s_;
    } else {
      sum_dis = sum_dis + iterator_lane->length() * 0.01;
    }

    if (sum_dis > kFrontDis) {
      break;
    }

    iterator_lane = successor_lane;
  }

  return true;
}

bool LDRouteInfoStrategy::IsLaneSuccessorIsMergeLane(
    const iflymapdata::sdpro::Lane* lane_info) {
  if (lane_info == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::Lane* iterator_lane = lane_info;
  const double kFrontDis = 500.0;
  double sum_dis = 0.0;

  while (iterator_lane) {
    if (IsMergeLane(iterator_lane)) {
      return true;
    }

    if (iterator_lane->successor_lane_ids_size() != 1) {
      // 由于目前1分2车道检测不稳定，后继不为1的直接认为不在route上，不会被放进feasible lane中
      // 后续根据测试效果，确定是否需要更精确的判断
      return false;
    }

    const uint64 successor_lane_id = iterator_lane->successor_lane_ids()[0];
    const auto& successor_lane = ld_map_.GetLaneInfoByID(successor_lane_id);
    if (successor_lane == nullptr) {
      return false;
    }

    if (iterator_lane->link_id() == current_link_->id()) {
      sum_dis = sum_dis + iterator_lane->length() * 0.01 - ego_on_cur_link_s_;
    } else {
      sum_dis = sum_dis + iterator_lane->length() * 0.01;
    }

    if (sum_dis > kFrontDis) {
      break;
    }

    iterator_lane = successor_lane;
  }

  return false;
}

const iflymapdata::sdpro::LinkInfo_Link*
LDRouteInfoStrategy::FindFrontValidRampSplitLink() const {
  // 在处理ramp场景时，是以前方split是ramp作为判断条件的，在主路上可以有效计算出feasible lane
  // 自车在匝道上时，前方所有的split都是匝道的话，则只会处理最近的split，这样存在无法处理匝道上有连续split的场景
  // 因此，当自车在匝道上nearing ramp，且前方500m内有多个split时，至少需要判断500m处的ramp。
  if (ramp_info_vec_.empty() || !route_info_output_.is_on_ramp) {
    return nullptr;
  }

  const double min_front_search_dis = 500.0;
  for (size_t ramp_idx = 0; ramp_idx < ramp_info_vec_.size(); ++ramp_idx) {
    const auto& [ramp_link, ramp_dis] = ramp_info_vec_[ramp_idx];
    // 场景1：匝道距离≥最小搜索距离 → 直接取当前匝道链路作为分流链路
    if (ramp_dis > min_front_search_dis) {
      return ramp_link;
    }

    // 场景2：匝道距离小于最小搜索距离 → 检查合流信息
    for (const auto& merge_info : merge_info_vec_) {
      const double merge_distance = merge_info.second;
      // 合流点距离 < 匝道距离 + 不忽略该合流 + 非第一个匝道（避免ramp_idx-1越界）
      const bool is_valid_merge = (merge_distance < ramp_dis) &&
                                  !IsIgnoreMerge(merge_info) &&
                                  (ramp_idx > 0);
      if (is_valid_merge) {
        return ramp_info_vec_[ramp_idx - 1].first;
      }
    }
  }

  return ramp_info_vec_.back().first;
}

void LDRouteInfoStrategy::CalculateAvoidMergeFeasibleLane(
    TopoLinkGraph& feasible_lane_graph) {
  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return;
  }

  auto& current_link_feasible_lane = feasible_lane_graph.lane_topo_groups.back();
  // 如果feasible lane中车道数只有有一条，则不再避让merge
  if (current_link_feasible_lane.topo_lanes.size() < 2) {
    return;
  }

  auto IsLinkInMergeInfoVec =
      [this](const iflymapdata::sdpro::LinkInfo_Link* target_link) {
        return std::any_of(merge_info_vec_.begin(), merge_info_vec_.end(),
                           [target_link](const auto& merge_info) {
                             return merge_info.first == target_link;
                           });
      };

  auto& topo_lanes = current_link_feasible_lane.topo_lanes;
  for (auto it = topo_lanes.begin(); it != topo_lanes.end();) {
    // 拿掉feasible lane中车道数后，车道数少于2了，直接break掉。
    if (topo_lanes.size() < 2) {
      break;
    }
    const auto lane_id = it->id;
    const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (temp_lane == nullptr) {
      ++it;
      continue;
    }

    const iflymapdata::sdpro::Lane* entry_lane_info = nullptr;
    entry_lane_info = IsEntryLanePresentOnEitherSideOfSuccessorLane(temp_lane);
    if (entry_lane_info == nullptr) {
      ++it;
      continue;
    }

    const auto& entry_lane_belong_link = ld_map_.GetLinkOnRoute(entry_lane_info->link_id());
    if (entry_lane_belong_link == nullptr) {
      ++it;
      continue;
    }

    if (IsLinkInMergeInfoVec(entry_lane_belong_link)) {
      it = topo_lanes.erase(it);
    } else {
      ++it;
    }

  }
}

const iflymapdata::sdpro::Lane*
LDRouteInfoStrategy::IsEntryLanePresentOnEitherSideOfSuccessorLane(
    const iflymapdata::sdpro::Lane* cur_link_lane_info) {
  if (cur_link_lane_info == nullptr) {
    return nullptr;
  }

  const double front_search_dis = 500.0;
  double sum_dis = 0.0;
  const auto* current_traverse_lane = cur_link_lane_info;
  while (current_traverse_lane) {
    if (current_traverse_lane->link_id() == current_link_->id()) {
      sum_dis += current_traverse_lane->length() * 0.01 - ego_on_cur_link_s_;
    } else {
      sum_dis += current_traverse_lane->length() * 0.01;
    }

    const auto [left_nerghbor_lane, right_nerghbor_lane] =
        FindLaneLeftRightNeighbors(current_traverse_lane);

    if (IsEntryLane(left_nerghbor_lane)) {
      return left_nerghbor_lane;
    }

    if (IsEntryLane(right_nerghbor_lane)) {
      return right_nerghbor_lane;
    }

    if (sum_dis > front_search_dis) {
      break;
    }

    if (current_traverse_lane->successor_lane_ids_size() != 1) {
      return nullptr;
    }

    current_traverse_lane =
        ld_map_.GetLaneInfoByID(current_traverse_lane->successor_lane_ids()[0]);
  }

  return nullptr;
}


std::pair<const iflymapdata::sdpro::Lane*, const iflymapdata::sdpro::Lane*>
LDRouteInfoStrategy::FindLaneLeftRightNeighbors(const iflymapdata::sdpro::Lane* target_lane) {
  if (target_lane == nullptr) {
    return {nullptr, nullptr};
  }

  const auto* lane_link = ld_map_.GetLinkOnRoute(target_lane->link_id());
  if (lane_link == nullptr) {
    return {nullptr, nullptr};
  }

  const int target_seq = target_lane->sequence();
  const int left_neighbor_seq = target_seq + 1;  //sequence左大右小
  const int right_neighbor_seq = target_seq - 1;
  const iflymapdata::sdpro::Lane* left_lane = nullptr;
  const iflymapdata::sdpro::Lane* right_lane = nullptr;

  // 查找左右侧车道，找到后提前终止循环
  for (const auto& lane_id : lane_link->lane_ids()) {
    const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }

    if (lane->sequence() == left_neighbor_seq) {
      left_lane = lane;
    } else if (lane->sequence() == right_neighbor_seq) {
      right_lane = lane;
    }

    // 左右都找到，提前退出
    if (left_lane != nullptr && right_lane != nullptr) {
      break;
    }
  }

  return {left_lane, right_lane};
}

bool LDRouteInfoStrategy::IsInvalidNonDrivingLane(const iflymapdata::sdpro::Lane* lane) {
  if (lane == nullptr) {
    return true;
  }
  return IsEmergencyLane(lane) || IsDiversionLane(lane);
}

bool LDRouteInfoStrategy::IsInvalidLaneMergeLaneOppositeSide(
    const iflymapdata::sdpro::Lane* merge_lane) {
  if (merge_lane == nullptr) {
    return false;
  }

  // 步骤1：确定合流方向
  auto merge_lane_type = CalculateMergeLaneType(merge_lane);

  if (merge_lane_type == NONE_MERGE) {
    return false;
  }

  // 步骤2：查找合流车道的左右侧邻居车道
  const auto [left_neighbor, right_neighbor] =
      FindLaneLeftRightNeighbors(merge_lane);

  // 步骤3：根据合流方向判断邻居车道是否有效
  if (merge_lane_type == MERGE_TO_LEFT) {
    return IsInvalidNonDrivingLane(right_neighbor);
  } else if (merge_lane_type == MERGE_TO_RIGHT) {
    return IsInvalidNonDrivingLane(left_neighbor);
  }

  return false;
}

MergeLaneType LDRouteInfoStrategy::CalculateMergeLaneType(const iflymapdata::sdpro::Lane* merge_lane) {
  if (merge_lane == nullptr) {
    return NONE_MERGE;
  }

  if (merge_lane->successor_lane_ids().empty()) {
    return NONE_MERGE;
  }

  const auto& merge_lane_succesor_lane =
      ld_map_.GetLaneInfoByID(merge_lane->successor_lane_ids()[0]);
  if (merge_lane_succesor_lane == nullptr) {
    return NONE_MERGE;
  }

  for (const auto& pre_lane_id : merge_lane_succesor_lane->predecessor_lane_ids()) {
    const auto* pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
    if (pre_lane == nullptr || IsMergeLane(pre_lane)) {
      continue;
    }

    if (pre_lane->sequence() < merge_lane->sequence()) {
      return MERGE_TO_RIGHT;
    } else {
      return MERGE_TO_LEFT;
    }
  }

  return NONE_MERGE;
}

void LDRouteInfoStrategy::HandleMainLinkPreLane(
    const iflymapdata::sdpro::Lane* pre_lane,
    std::vector<iflymapdata::sdpro::Lane>& next_lane_vec) {
    if (pre_lane == nullptr) {
      return;
    }

    if (!HasLaneId(next_lane_vec, pre_lane->id())) { // 避免重复添加
        next_lane_vec.emplace_back(*pre_lane);
    }
}

void LDRouteInfoStrategy::HandleOtherMergeLinkPreLane(
    const TopoLane& topo_lane,
    const iflymapdata::sdpro::LinkInfo_Link* next_pre_link,
    const iflymapdata::sdpro::LinkInfo_Link* current_link,
    const iflymapdata::sdpro::Lane* pre_lane,
    std::vector<iflymapdata::sdpro::Lane>& next_lane_vec) {
  if (next_pre_link == nullptr || current_link == nullptr ||
      pre_lane == nullptr) {
    return;
  }

  std::unordered_set<uint64_t> other_link_id_set;
  for (const auto& pre_link_id: current_link->predecessor_link_ids()) {
    if (pre_link_id == next_pre_link->id()) {
      continue;
    }
    other_link_id_set.insert(pre_link_id);
  }

  if (other_link_id_set.empty()) {
    return;
  }

  auto it = other_link_id_set.find(pre_lane->link_id());
  if (it == other_link_id_set.end()) {
    return;
  }

  // 在主link中找到与拓扑车道最匹配的前继车道
  const auto matching_pre_lane =
      FindMatchingPreLaneInMainLink(topo_lane, next_pre_link);

  if (matching_pre_lane.link_id() == next_pre_link->id()) {
    next_lane_vec.emplace_back(matching_pre_lane);
  }
}

iflymapdata::sdpro::Lane LDRouteInfoStrategy::FindMatchingPreLaneInMainLink(
    const TopoLane& topo_lane,
    const iflymapdata::sdpro::LinkInfo_Link* next_pre_link) {
  int min_order_error = 100;
  iflymapdata::sdpro::Lane best_matching_lane;

  if (next_pre_link == nullptr) {
    return best_matching_lane;
  }

  for (const auto& temp_lane_id : next_pre_link->lane_ids()) {
    const auto* temp_lane = ld_map_.GetLaneInfoByID(temp_lane_id);
    if (!temp_lane || IsEmergencyLane(temp_lane) ||
        IsDiversionLane(temp_lane)) {
      continue;
    }

    // 遍历当前车道的后继车道，匹配拓扑车道所属link
    for (const auto& temp_suc_lane_id : temp_lane->successor_lane_ids()) {
      const auto* temp_suc_lane = ld_map_.GetLaneInfoByID(temp_suc_lane_id);
      if (!temp_suc_lane || IsEmergencyLane(temp_suc_lane) ||
          IsDiversionLane(temp_suc_lane)) {
        continue;
      }

      // 确保后继车道与拓扑车道属于同一link
      if (temp_suc_lane->link_id() != topo_lane.link_id) {
        continue;
      }

      // 计算序号差，更新最优匹配车道
      const int order_error = temp_suc_lane->sequence() - topo_lane.order_id;
      if (order_error < min_order_error) {
        min_order_error = order_error;
        best_matching_lane = *temp_lane;
      }
    }
  }

  return best_matching_lane;
}
}