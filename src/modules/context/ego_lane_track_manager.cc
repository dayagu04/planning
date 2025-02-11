#include "ego_lane_track_manager.h"
// #include <cyber/common/log.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "ifly_localization_c.h"
#include "ifly_time.h"
#include "local_view.h"
#include "log.h"
// #include "log_glog.h"
#include "math/box2d.h"
#include "math/math_utils.h"
#include "math/vec2d.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "stop_line.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"
#include "vehicle_config_context.h"
#include "virtual_lane.h"
namespace planning {
using namespace planning_math;

namespace {
constexpr double kLastPlanLengthThr = 2.0;
constexpr double kBoundaryCrossEgoBehindThr = 5.0;
constexpr double kBoundaryCrossEgoFrontThr = 10.0;
constexpr double kCrossLaneCostDefault = 0.2;
constexpr double kInitPosCostStandardThr = 3.6;
constexpr double kInitPosCostWeight = 2.0;
constexpr double kCumuLateralDistanceCostWeight = 1.5;
constexpr double kCrossLaneCostWeight = 1.0;
constexpr double kLaneChangeExecutionWeightRatio = 0.5;
constexpr int32_t kLaneCenterMinPointsThr = 3;
constexpr double kLaneLineSegmentLength = 5.0;
constexpr double kConsiderLaneLineLength = 50.0;
constexpr double kDefaultRoadRadius = 750.0;
constexpr int32_t kDefaultPointNums = 35;
constexpr int32_t kLeastDefaultPointNums = 3;
constexpr double kConsiderLaneStraightFrontEgo = 30.0;
constexpr double kDefaultMappingConsiderLaneLength = 75.0;
constexpr double kDefaultConsiderLaneMarksLength = 80.0;
constexpr double kEgoPreviewTimeMinThd = 3.0;
constexpr double kEgoPreviewTimeMaxThd = 5.0;
constexpr double kExistSplitLateralDisThd = 1.5;
constexpr double kCenterLineLateralDisThd = 0.8;
constexpr double kNearPreviewDistanceThd = 20.0;

constexpr double kAverageKappaCostWeight = 2.5;
constexpr double kAverageThetaDiffCostWeight = 0.5;
}  // namespace

EgoLaneTrackManger::EgoLaneTrackManger(
    const EgoPlanningConfigBuilder* config_builder, planning::framework::Session* session) {
  session_ = session;
  SetConfig(config_builder);
  // order_ids_of_same_zero_relative_id_.clear();
}

void EgoLaneTrackManger::SetConfig(const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<EgoPlanningConfig>();
}

void EgoLaneTrackManger::TrackEgoLane(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    std::vector<int>& order_ids_of_same_zero_relative_id,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane) {
  const auto& function_info = session_->environmental_model().function_info();
  const auto& planning_context = session_->planning_context();
  const auto& planning_result = planning_context.last_planning_result();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const bool is_in_lane_borrow_status = session_->planning_context()
                                            .lane_borrow_decider_output()
                                            .is_in_lane_borrow_status;
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  const bool lane_keep_status = lane_change_status == kLaneKeeping;

  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const double dis_to_split_threshold = 1000.0;
  // const double dis_to_last_split_threshold = 150.0;
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const int zero_relative_id_nums = order_ids_of_same_zero_relative_id.size();
  const bool enable_use_ground_mark = config_.enable_use_ground_mark_process_split;

  is_exist_split_on_ramp_ = false;
  is_exist_ramp_on_road_ = false;
  is_exist_split_on_expressway_ = false;
  is_exist_split_on_intersection_ = false;
  virtual_lane_relative_id_switch_flag_ = false;
  is_select_ego_lane_without_plan_ = false;
  is_select_ego_lane_with_plan_ = false;
  is_in_ramp_select_split_situation_ = false;
  is_on_road_select_ramp_situation_ = false;

  //判断自车是否处于分流场景
  ComputeIsSplitRegion(
      relative_id_lanes, order_ids_of_same_zero_relative_id);

  if (zero_relative_id_nums < 2 || !ego_in_split_region_) {
    last_zero_relative_id_order_id_index_ = -1;
    last_track_ego_lane_ = nullptr;
  }
  if (!active ||
      function_info.function_mode() == common::DrivingFunctionInfo::ACC) {
    SelectEgoLaneWithoutPlan(relative_id_lanes);
    is_select_ego_lane_without_plan_ = true;
  } else {
    if (!planning_result.traj_points.empty()) {
      double last_plan_length = planning_result.traj_points.back().s -
                                planning_result.traj_points.front().s;
      if (last_plan_length < kLastPlanLengthThr) {
        SelectEgoLaneWithoutPlan(relative_id_lanes);
        is_select_ego_lane_without_plan_ = true;
        return;
      }
      if (function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
        if (is_ego_on_expressway_ && zero_relative_id_nums >= 2
            &&!is_in_lane_borrow_status
            && ego_in_split_region_) {
          bool is_on_road_select_ramp = CheckIfInRoadSelectRamp(
              relative_id_lanes, order_ids_of_same_zero_relative_id);
          is_on_road_select_ramp_situation_ = is_on_road_select_ramp;
          LOG_DEBUG(
              "EgoLaneTrackManger::is_on_road_select_ramp_situation: %d \n",
              is_on_road_select_ramp_situation_);

          if (is_on_road_select_ramp_situation_ &&
              distance_to_first_road_split_ < dis_to_split_threshold &&
              !is_leaving_ramp_ && lane_keep_status) {
            // hack::针对分流 感知未提供分汇流点信息 作如下后处理
            PreprocessRoadSplit(relative_id_lanes,
                                order_ids_of_same_zero_relative_id);
            LOG_DEBUG("EgoLaneTrackManger::is_exist_ramp_on_road: %d \n",
                      is_exist_ramp_on_road_);

            if (is_exist_ramp_on_road_) {
              return;
            }
          }

          bool is_in_ramp_select_split = CheckIfInRampSelectSplit(
              relative_id_lanes, order_ids_of_same_zero_relative_id);
          is_in_ramp_select_split_situation_ = is_in_ramp_select_split;
          LOG_DEBUG(
              "EgoLaneTrackManger::is_in_ramp_select_split_situation: %d \n",
              is_in_ramp_select_split_situation_);
          if (is_in_ramp_select_split_situation_ && lane_keep_status) {
            //选择匝道上的分叉
            PreprocessRampSplit(relative_id_lanes,
                                order_ids_of_same_zero_relative_id);
            LOG_DEBUG("EgoLaneTrackManger::is_exist_split_on_ramp: %d \n",
                      is_exist_split_on_ramp_);

            if (is_exist_split_on_ramp_) {
              return;
            }
          }

          //处理高架快速路普通分流口
          if (lane_keep_status) {
            PreprocessOrdinarySplit(relative_id_lanes,
                                    order_ids_of_same_zero_relative_id,
                                    virtual_id_mapped_lane);
            LOG_DEBUG(
                "EgoLaneTrackManger::is_exist_split_on_expressway_: %d \n",
                is_exist_split_on_expressway_);
            if (is_exist_split_on_expressway_) {
              return;
            }
          }
        }
      } else if (function_info.function_mode() ==
                 common::DrivingFunctionInfo::SCC) {
        if (zero_relative_id_nums > 1 && lane_keep_status
            && !is_in_lane_borrow_status
            && ego_in_split_region_) {
          if (enable_use_ground_mark) {
            ProcessSplitWithGroundMark(relative_id_lanes,
                                        order_ids_of_same_zero_relative_id);
            LOG_DEBUG("EgoLaneTrackManger::is_exist_split_on_intersection: %d \n",
                      is_exist_split_on_intersection_);

            if (is_exist_split_on_intersection_) {
              return;
            }
          }

          ProcessIntersectionSplit(relative_id_lanes,
                          order_ids_of_same_zero_relative_id);
          LOG_DEBUG("EgoLaneTrackManger::is_exist_split_on_intersection: %d \n",
                    is_exist_split_on_intersection_);

          if (is_exist_split_on_intersection_) {
            return;
          }
        }
      }

      SelectEgoLaneWithPlan(relative_id_lanes,
                            order_ids_of_same_zero_relative_id,
                            virtual_id_mapped_lane);
    } else {
      SelectEgoLaneWithoutPlan(relative_id_lanes);
      is_select_ego_lane_without_plan_ = true;
    }
  }

  LOG_DEBUG("EgoLaneTrackManger::virtual_lane_relative_id_switch_flag: %d \n",
            virtual_lane_relative_id_switch_flag_);
  JSON_DEBUG_VALUE("virtual_lane_relative_id_switch_flag",
                   virtual_lane_relative_id_switch_flag_);
  return;
}

void EgoLaneTrackManger::UpdateLaneVirtualId(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane,
    int* last_fix_lane_virtual_id) {
  int lane_virtual_id;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& coarse_planning_info =
      lane_change_decider_output.coarse_planning_info;
  const double lane_point_match_lateral_dis_threshold = 1.8;
  const double split_lane_point_match_threshold = 0.18;
  auto lc_state = coarse_planning_info.target_state;
  int target_lane_vitrual_id =
      lane_change_decider_output.target_lane_virtual_id;
  int target_lane_order_id = 0;
  int origin_lane_virtual_id =
      lane_change_decider_output.origin_lane_virtual_id;
  int origin_lane_order_id = 0;
  int target_lane_order_and_virtual_diff = 0;
  int origin_lane_order_and_virtual_diff = 0;
  double target_lane_maping_diff_total = std::numeric_limits<double>::max();
  double origin_lane_maping_diff_total = std::numeric_limits<double>::max();
  double current_relative_id_lane_mapping_cost = 10.0;
  bool is_lc_change =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete) ||
       (coarse_planning_info.target_state == kLaneChangeCancel));

  if (is_lc_change && (lc_state != kLaneKeeping)) {
    for (const auto& relative_id_lane : relative_id_lanes) {
      if (relative_id_lane != nullptr) {
        current_relative_id_lane_mapping_cost = ComputeLanesMatchlaterakDisCost(
            target_lane_vitrual_id, relative_id_lane, relative_id_lanes,
            virtual_id_mapped_lane);
        if (current_relative_id_lane_mapping_cost <
            target_lane_maping_diff_total) {
          target_lane_maping_diff_total = current_relative_id_lane_mapping_cost;
          target_lane_order_id = relative_id_lane->get_order_id();
        }
      } else {
        continue;
      }
    }
    LOG_DEBUG("target_lane_maping_diff_total: %f \n",
              target_lane_maping_diff_total);

    if (target_lane_maping_diff_total <
        lane_point_match_lateral_dis_threshold) {
      target_lane_order_and_virtual_diff =
          target_lane_vitrual_id - target_lane_order_id;
      virtual_id_mapped_lane.clear();
      for (auto& lane : relative_id_lanes) {
        if (lane != nullptr) {
          auto lane_virtual_id =
              lane->get_order_id() + target_lane_order_and_virtual_diff;
          virtual_id_mapped_lane[lane_virtual_id] = lane;
          lane->set_virtual_id(lane_virtual_id);
          if (lane->get_relative_id() == 0) {
            current_lane_virtual_id_ = lane_virtual_id;
          }
        } else {
          continue;
        }
      }
      return;
    }

    // 换道过程中目标车道匹配不成功 则匹配原车道
    for (const auto& relative_id_lane : relative_id_lanes) {
      if (relative_id_lane != nullptr) {
        current_relative_id_lane_mapping_cost = ComputeLanesMatchlaterakDisCost(
            origin_lane_virtual_id, relative_id_lane, relative_id_lanes,
            virtual_id_mapped_lane);
        if (current_relative_id_lane_mapping_cost <
            origin_lane_maping_diff_total) {
          origin_lane_maping_diff_total = current_relative_id_lane_mapping_cost;
          origin_lane_order_id = relative_id_lane->get_order_id();
        }
      } else {
        continue;
      }
    }
    LOG_DEBUG("origin_lane_maping_diff_total: %f \n",
              origin_lane_maping_diff_total);

    if (origin_lane_maping_diff_total <
        lane_point_match_lateral_dis_threshold) {
      origin_lane_order_and_virtual_diff =
          origin_lane_virtual_id - origin_lane_order_id;
      virtual_id_mapped_lane.clear();
      for (auto& lane : relative_id_lanes) {
        if (lane != nullptr) {
          auto lane_virtual_id =
              lane->get_order_id() + origin_lane_order_and_virtual_diff;
          virtual_id_mapped_lane[lane_virtual_id] = lane;
          lane->set_virtual_id(lane_virtual_id);
          if (lane->get_relative_id() == 0) {
            current_lane_virtual_id_ = lane_virtual_id;
          }
        } else {
          continue;
        }
      }
      return;
    }

    virtual_id_mapped_lane.clear();
    for (auto& lane : relative_id_lanes) {
      auto lane_virtual_id = lane->get_relative_id() + current_lane_virtual_id_;
      virtual_id_mapped_lane[lane_virtual_id] = lane;
      lane->set_virtual_id(lane_virtual_id);
    }
    return;
  }

  lane_virtual_id = current_lane_virtual_id_;
  // update fixlane id
  const auto& lateral_outputs =
      session_->planning_context().lateral_behavior_planner_output();
  if (lateral_outputs.lc_status == "none") {
    // sync fix lane id with current lane id while no lane change
    *last_fix_lane_virtual_id = lane_virtual_id;
  }

  virtual_id_mapped_lane.clear();
  for (auto& lane : relative_id_lanes) {
    auto lane_virtual_id = lane->get_relative_id() + current_lane_virtual_id_;
    virtual_id_mapped_lane[lane_virtual_id] = lane;
    lane->set_virtual_id(lane_virtual_id);
  }
  virtual_lane_relative_id_switch_flag_ = false;
}

void EgoLaneTrackManger::Reset() {
  current_lane_virtual_id_ = 0;
  // order_id_mapped_lanes_.clear();
  dis_to_ramp_ = NL_NMAX;
  distance_to_first_road_merge_ = NL_NMAX;
  distance_to_first_road_split_ = NL_NMAX;
  is_leaving_ramp_ = false;
  is_on_ramp_ = false;
  is_ego_on_expressway_ = false;
  first_split_dir_dis_info_.first = None;
  first_split_dir_dis_info_.second = NL_NMAX;
  current_segment_passed_distance_ = 0.0;
  split_direction_dis_info_list_.clear();
  is_exist_split_on_ramp_ = false;
  is_exist_ramp_on_road_ = false;
  is_exist_split_on_expressway_ = false;
  is_exist_split_on_intersection_ = false;
  virtual_lane_relative_id_switch_flag_ = false;
  is_select_ego_lane_without_plan_ = false;
  is_select_ego_lane_with_plan_ = false;
  is_in_ramp_select_split_situation_ = false;
  is_on_road_select_ramp_situation_ = false;
}

void EgoLaneTrackManger::Update(const RouteInfoOutput& route_info_output) {
  is_ego_on_expressway_ = route_info_output.is_ego_on_expressway;
  is_on_ramp_ = route_info_output.is_on_ramp;
  dis_to_ramp_ = route_info_output.dis_to_ramp;
  is_leaving_ramp_ = false;
  first_split_dir_dis_info_.first =
      route_info_output.first_split_dir_dis_info.first;
  first_split_dir_dis_info_.second =
      route_info_output.first_split_dir_dis_info.second;
  distance_to_first_road_merge_ =
      route_info_output.distance_to_first_road_merge;
  distance_to_first_road_split_ =
      route_info_output.distance_to_first_road_split;
  current_segment_passed_distance_ =
      route_info_output.current_segment_passed_distance;
  sum_dis_to_last_split_point_ = route_info_output.sum_dis_to_last_split_point;
  split_direction_dis_info_list_.clear();

  for (int i = 0; i < route_info_output.split_dir_dis_info_list.size(); i++) {
    split_direction_dis_info_list_.emplace_back(
        route_info_output.split_dir_dis_info_list[i]);
  }
}

void EgoLaneTrackManger::SelectEgoLaneWithoutPlan(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double ego_heading_angle = ego_state->heading_angle();
  double ego2lane_min = std::numeric_limits<double>::max();
  int origin_order_id = 0;
  double total_lane_cost = 0.0;
  std::unordered_map<int32_t, std::vector<double>> lane_cost_list;
  const double init_pos_lateral_offset_weight = 1.0;
  const double heading_angle_diff_weight = 2.5;
  Point2D ego_cart(ego_state->ego_pose().x, ego_state->ego_pose().y);

  for (auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane != nullptr) {
      if (relative_id_lane->get_lane_frenet_coord() != nullptr) {
        double average_heading_angle_cost = 0.0;
        double heading_angle_cost = 0.0;
        double pos_lateral_offset_cost = 0.0;
        std::shared_ptr<KDPath> frenet_coord =
            relative_id_lane->get_lane_frenet_coord();
        double ego_s = 0.0;
        double ego_l = 0.0;
        Point2D ego_frenet_point;
        if (!frenet_coord->XYToSL(ego_cart, ego_frenet_point)) {
          continue;
        } else {
          ego_s = ego_frenet_point.x;
          ego_l = ego_frenet_point.y;
        }
        if (ego_s > frenet_coord->Length()) {
          continue;
        }
        planning_math::PathPoint ego_s_nearest_point =
            frenet_coord->GetPathPointByS(ego_s);
        int iter_count = 0;
        for (double s = ego_s_nearest_point.s(); s < frenet_coord->Length();
             s += kLaneLineSegmentLength) {
          if (s > kDefaultMappingConsiderLaneLength + ego_s) {
            break;
          }
          double heading_angle = frenet_coord->GetPathCurveHeading(s);
          double theta_err = NormalizeAngle(heading_angle - ego_heading_angle);
          heading_angle_cost += std::fabs(theta_err);
          iter_count++;
        }
        pos_lateral_offset_cost = std::fabs(ego_l);
        iter_count = std::max(1, iter_count);
        average_heading_angle_cost = heading_angle_cost / iter_count;
        total_lane_cost =
            average_heading_angle_cost * heading_angle_diff_weight +
            pos_lateral_offset_cost * init_pos_lateral_offset_weight;
        std::vector<double> cost_list{average_heading_angle_cost, ego_l,
                                      total_lane_cost};
        lane_cost_list[relative_id_lane->get_order_id()] = cost_list;

        if (total_lane_cost < ego2lane_min) {
          ego2lane_min = total_lane_cost;
          origin_order_id = relative_id_lane->get_order_id();
          relative_id_lane->set_relative_id(0);
        }
      }
    }
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }

  return;
}

void EgoLaneTrackManger::SelectEgoLaneWithPlan(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const bool is_in_lane_borrow_status = session_->planning_context()
                                            .lane_borrow_decider_output()
                                            .is_in_lane_borrow_status;
  int origin_order_id = 0;
  int current_order_id = 0;
  const double default_lane_mapping_cost = 10.0;
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const int fix_lane_virtual_id =
      lane_change_decider_output.fix_lane_virtual_id;
  const int lc_state = lane_change_decider_output.curr_state;
  std::unordered_map<int32_t, std::vector<double>> lane_cost_list;
  std::shared_ptr<VirtualLane> last_track_virtual_id_lane;
  int last_track_virtual_id = 0;
  double last_ego_lane_curv = 0.0;

  if (!virtual_id_mapped_lane.empty()) {
    for (const auto& virtual_id_lane : virtual_id_mapped_lane) {
      if (virtual_id_lane.second->get_virtual_id() == fix_lane_virtual_id) {
        last_track_virtual_id_lane = virtual_id_lane.second;
        last_track_virtual_id = virtual_id_lane.first;
      }
    }
  } else {
    return;
  }

  last_ego_lane_curv =
      ComputeTargetLaneSpecifiedRangeCurvature(last_track_virtual_id_lane);
  double road_radius = 1 / std::max(last_ego_lane_curv, 0.0001);

  const auto coarse_planning_info = session_->planning_context()
                                        .lane_change_decider_output()
                                        .coarse_planning_info;
  bool is_lc_change =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  bool is_lc_back = coarse_planning_info.target_state == kLaneChangeCancel;
  bool is_lane_change = (is_lc_change || is_lc_back);
  double k_init_pos_cost_weight =
      is_lane_change ? kLaneChangeExecutionWeightRatio * kInitPosCostWeight
                     : kInitPosCostWeight;
  double lateral_distance_cost_weight = kCumuLateralDistanceCostWeight;

  if ((lc_state == kLaneKeeping || lc_state == kLaneChangePropose) &&
      order_ids.size() < 2) {
    lateral_distance_cost_weight = 0.28;
  }
  if (is_in_lane_borrow_status) {
    k_init_pos_cost_weight = 0.0;
  }

  double clane_min_diff_total = std::numeric_limits<double>::max();
  for (auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane == nullptr) {
      continue;
    }

    if (relative_id_lane->get_relative_id() == 0) {
      origin_order_id = relative_id_lane->get_order_id();
    }
    auto& lane_points = relative_id_lane->lane_points();
    if (lane_points.size() < kLaneCenterMinPointsThr) {
      continue;
    }
    double total_cost = 0.0;
    double cumu_lat_dis_cost = 0.0;
    double init_pose_cost = 0.0;
    double crosslane_cost = 0.0;

    if ((!is_lane_change) &&
        CalcCrosslaneStatus(relative_id_lane, lane_points)) {
      crosslane_cost = kCrossLaneCostDefault;
    }

    if (last_track_virtual_id_lane != nullptr) {
      auto track_lane_frenet_coord =
          last_track_virtual_id_lane->get_lane_frenet_coord();
      if (track_lane_frenet_coord != nullptr) {
        const auto& lane_points = relative_id_lane->lane_points();
        if (lane_points.size() <= 2) {
          continue;
        }
        int point_nums = 0;
        double total_lateral_offset = 0.0;
        double target_ego_s = 0.0;
        double target_ego_l = 0.0;
        Point2D ego_cart_target_frenet;
        auto cur_lane_frenet_coord = relative_id_lane->get_lane_frenet_coord();
        if (cur_lane_frenet_coord == nullptr) {
          continue;
        }
        if (!cur_lane_frenet_coord->XYToSL(ego_cart_point,
                                           ego_cart_target_frenet)) {
          continue;
        } else {
          target_ego_s = ego_cart_target_frenet.x;
          target_ego_l = ego_cart_target_frenet.y;
        }
        if (road_radius > kDefaultRoadRadius) {
          int select_lane_point_interval = 1;
          for (int i = 0; i < lane_points.size();
               i += select_lane_point_interval) {
            iflyauto::ReferencePoint point = lane_points[i];
            if (std::isnan(point.local_point.x) ||
                std::isnan(point.local_point.y)) {
              LOG_ERROR("update_lane_points: skip NaN point");
              continue;
            }
            double lateral_offset = 0.0;
            double s = 0.0;
            Point2D cur_point_frenet;
            Point2D cur_point(point.local_point.x, point.local_point.y);
            if (!track_lane_frenet_coord->XYToSL(cur_point, cur_point_frenet)) {
              lateral_offset = 10.0;
            } else {
              s = cur_point_frenet.x;
              lateral_offset = cur_point_frenet.y;
            }
            if (s < target_ego_s) {
              continue;
            }
            total_lateral_offset += lateral_offset;
            point_nums += 1;
            if (point_nums >= kDefaultPointNums ||
                s > target_ego_s + kDefaultMappingConsiderLaneLength) {
              break;
            }
          }
        } else {
          for (int i = 0; i < lane_points.size(); i++) {
            iflyauto::ReferencePoint point = lane_points[i];
            if (std::isnan(point.local_point.x) ||
                std::isnan(point.local_point.y)) {
              LOG_ERROR("update_lane_points: skip NaN point");
              continue;
            }
            double lateral_offset = 0.0;
            double s = 0.0;
            Point2D cur_point_frenet;
            Point2D cur_point(point.local_point.x, point.local_point.y);
            if (!track_lane_frenet_coord->XYToSL(cur_point, cur_point_frenet)) {
              lateral_offset = 10.0;
            } else {
              s = cur_point_frenet.x;
              lateral_offset = cur_point_frenet.y;
            }
            if (s < target_ego_s) {
              continue;
            }
            if (point_nums >= kDefaultPointNums ||
                s > target_ego_s + kConsiderLaneLineLength) {
              break;
            }
            total_lateral_offset += lateral_offset;
            point_nums += 1;
          }
        }
        point_nums = std::max(1, point_nums);
        cumu_lat_dis_cost = std::fabs(total_lateral_offset / point_nums);
      } else {
        cumu_lat_dis_cost = default_lane_mapping_cost;
      }
    } else {
      return;
    }

    std::shared_ptr<KDPath> frenet_coord;
    if (relative_id_lane->get_lane_frenet_coord() == nullptr) {
      continue;
    }
    frenet_coord = relative_id_lane->get_lane_frenet_coord();

    Point2D ego_cart_frenet_in_lane;
    if (!frenet_coord->XYToSL(ego_cart_point, ego_cart_frenet_in_lane)) {
      init_pose_cost = 0.0;
    } else {
      init_pose_cost =
          std::fabs(ego_cart_frenet_in_lane.y) / kInitPosCostStandardThr;
    }
    total_cost = lateral_distance_cost_weight * cumu_lat_dis_cost +
                 kCrossLaneCostWeight * crosslane_cost +
                 k_init_pos_cost_weight * init_pose_cost;
    std::vector<double> cost_list{cumu_lat_dis_cost, crosslane_cost,
                                  init_pose_cost, total_cost};
    lane_cost_list[relative_id_lane->get_order_id()] = cost_list;

    if (total_cost < clane_min_diff_total) {
      clane_min_diff_total = total_cost;
      current_order_id = relative_id_lane->get_order_id();
      last_track_ego_lane_ = relative_id_lane;
      relative_id_lane->set_relative_id(0);
    }
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - current_order_id;
    lane->set_relative_id(lane_relative_id);
  }

  if (order_ids.size() == 1 && current_order_id != origin_order_id) {
    virtual_lane_relative_id_switch_flag_ = true;
  }
  if (order_ids.size() > 1) {
    for (size_t i = 0; i < order_ids.size(); i++) {
      if (current_order_id == order_ids[i]) {
        last_zero_relative_id_order_id_index_ = i;
      } else {
        continue;
      }
    }
  }
  return;
}

bool EgoLaneTrackManger::CalcCrosslaneStatus(
    const std::shared_ptr<VirtualLane> lane,
    const std::vector<iflyauto::ReferencePoint>& center_line_pathpoints) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  bool cross_lane = false;
  Vec2d ego_pose(plannig_init_point.lat_init_state.x(),
                 plannig_init_point.lat_init_state.y());

  double left_width = 0.5 * lane->width();
  double right_width = 0.5 * lane->width();
  double ego_lateral_offset = lane->get_ego_lateral_offset();
  bool is_outside_lane = false;
  is_outside_lane =
      (ego_lateral_offset > left_width) || (ego_lateral_offset < -right_width);

  if (!is_outside_lane) {
    const auto& left_boundary = lane->get_left_lane_boundary();
    const auto& right_boundary = lane->get_right_lane_boundary();
    std::shared_ptr<planning_math::KDPath> left_boundary_path;
    std::shared_ptr<planning_math::KDPath> right_boundary_path;
    left_boundary_path = MakeBoundaryPath(left_boundary);
    right_boundary_path = MakeBoundaryPath(right_boundary);

    if (left_boundary_path != nullptr) {
      CalcBoundaryCross(*left_boundary_path, center_line_pathpoints,
                        &cross_lane);
    }
    if (cross_lane) {
      return true;
    }

    if (right_boundary_path != nullptr) {
      CalcBoundaryCross(*right_boundary_path, center_line_pathpoints,
                        &cross_lane);
    }
    if (cross_lane) {
      return true;
    }
  }

  return false;
}

std::shared_ptr<planning_math::KDPath> EgoLaneTrackManger::MakeBoundaryPath(
    const iflyauto::LaneBoundary& boundary) {
  std::shared_ptr<planning_math::KDPath> boundary_path;
  std::vector<Vec2d> center_line_points;
  center_line_points.clear();

  for (const auto& point : boundary.enu_points) {
    if (center_line_points.size() < boundary.enu_points_size) {
      center_line_points.emplace_back(point.x, point.y);
    } else {
      break;
    }
  }

  if (center_line_points.size() <= 2) {
    return nullptr;
  }
  std::vector<planning_math::PathPoint> boundary_points;
  boundary_points.reserve(center_line_points.size());
  for (const auto& point : center_line_points) {
    if (std::isnan(point.x()) || std::isnan(point.y())) {
      LOG_ERROR("update_center_line_points: skip NaN point");
      continue;
    }
    auto pt = planning_math::PathPoint(point.x(), point.y());
    // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
    if (!boundary_points.empty()) {
      auto& last_pt = boundary_points.back();
      if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length() < 1e-2) {
        continue;
      }
    }
    boundary_points.emplace_back(pt);
  }

  if (boundary_points.size() <= 2) {
    return nullptr;
  }
  boundary_path =
      std::make_shared<planning_math::KDPath>(std::move(boundary_points));

  return boundary_path;
}

void EgoLaneTrackManger::CalcBoundaryCross(
    const planning_math::KDPath& lane_boundary_path,
    const std::vector<iflyauto::ReferencePoint>& center_line_pathpoints,
    bool* cross_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();

  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  if (!lane_boundary_path.KdtreeValid()) {
    *cross_lane = false;
    return;
  }
  if (lane_boundary_path.KdtreeValid() &&
      !lane_boundary_path.XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
    *cross_lane = false;
    return;
  }

  bool llane_pos = false;
  bool llane_neg = false;
  for (const auto& ref_pt : center_line_pathpoints) {
    double s, l;
    if (!lane_boundary_path.XYToSL(ref_pt.local_point.x, ref_pt.local_point.y,
                                   &s, &l)) {
      continue;
    }
    if (s > lane_boundary_path.Length()) {
      continue;
    }
    if (l > 0.0) {
      llane_pos = true;
    }
    if (l < 0.0) {
      llane_neg = true;
    }
    if (l * ego_l < 0.0 && (s > ego_s - kBoundaryCrossEgoBehindThr) &&
        (s < ego_s + kBoundaryCrossEgoFrontThr)) {
      *cross_lane = true;
      break;
    }
    if (llane_pos && llane_neg) {
      *cross_lane = true;
      break;
    }
  }
  return;
}

void EgoLaneTrackManger::PreprocessRoadSplit(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  int origin_order_id = 0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  int zero_relative_id_order_id_index = last_zero_relative_id_order_id_index_;
  bool enable_using_last_frame_track_ego_lane = true;
  bool find_last_frame_track_ego_lane = true;

  if (last_zero_relative_id_nums_ > 1) {
    LOG_DEBUG("PreprocessRoadSplit::last_zero_relative_id_nums_ > 1");
    if (last_zero_relative_id_order_id_index_ != -1) {
      if (order_ids.size() == 2 &&
          last_zero_relative_id_nums_ == order_ids.size() &&
          zero_relative_id_order_id_index < order_ids.size()) {
        last_track_ego_lane_ =
            relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
        for (auto& lane : relative_id_lanes) {
          int lane_order_id = lane->get_order_id();
          int lane_relative_id =
              lane_order_id - order_ids[last_zero_relative_id_order_id_index_];
          lane->set_relative_id(lane_relative_id);
        }
        is_exist_ramp_on_road_ = true;
        return;
      } else {
        ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_,
                                          relative_id_lanes, order_ids,
                                          zero_relative_id_order_id_index);

        if (enable_using_last_frame_track_ego_lane) {
          if (zero_relative_id_order_id_index < order_ids.size() &&
              relative_id_lanes.size() >
                  order_ids[zero_relative_id_order_id_index]) {
            last_track_ego_lane_ =
                relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
            last_zero_relative_id_order_id_index_ =
                zero_relative_id_order_id_index;
            for (auto& lane : relative_id_lanes) {
              int lane_order_id = lane->get_order_id();
              int lane_relative_id =
                  lane_order_id - order_ids[zero_relative_id_order_id_index];
              lane->set_relative_id(lane_relative_id);
            }
            is_exist_ramp_on_road_ = true;
            return;
          } else {
            return;
          }
        }
      }
    }
  }

  if (first_split_dir_dis_info_.first == ON_RIGHT) {
    is_exist_ramp_on_road_ = true;
    relative_id_lanes[order_ids[1]]->set_relative_id(0);
    origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
    last_zero_relative_id_order_id_index_ = 1;
    last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
  } else if (first_split_dir_dis_info_.first == ON_LEFT) {
    is_exist_ramp_on_road_ = true;
    relative_id_lanes[order_ids[0]]->set_relative_id(0);
    origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
    last_zero_relative_id_order_id_index_ = 0;
    last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
  } else {
    is_exist_ramp_on_road_ = false;
    return;
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  return;
}

void EgoLaneTrackManger::PreprocessRampSplit(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  int origin_order_id = 0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{ego_x, ego_y};
  int zero_relative_id_order_id_index = last_zero_relative_id_order_id_index_;
  bool enable_using_last_frame_track_ego_lane = true;
  bool find_last_frame_track_ego_lane = true;

  if (last_zero_relative_id_nums_ > 1) {
    LOG_DEBUG("PreprocessRampSplit::last_zero_relative_id_nums_ > 1");
    if (last_zero_relative_id_order_id_index_ != -1) {
      if (order_ids.size() == 2 &&
          last_zero_relative_id_nums_ == order_ids.size() &&
          zero_relative_id_order_id_index < order_ids.size()) {
        last_track_ego_lane_ =
            relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
        for (auto& lane : relative_id_lanes) {
          int lane_order_id = lane->get_order_id();
          int lane_relative_id =
              lane_order_id - order_ids[last_zero_relative_id_order_id_index_];
          lane->set_relative_id(lane_relative_id);
        }
        is_exist_split_on_ramp_ = true;
        return;
      } else {
        ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_,
                                          relative_id_lanes, order_ids,
                                          zero_relative_id_order_id_index);

        if (enable_using_last_frame_track_ego_lane) {
          if (zero_relative_id_order_id_index < order_ids.size() &&
              relative_id_lanes.size() >
                  order_ids[zero_relative_id_order_id_index]) {
            last_track_ego_lane_ =
                relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
            last_zero_relative_id_order_id_index_ =
                zero_relative_id_order_id_index;
            for (auto& lane : relative_id_lanes) {
              int lane_order_id = lane->get_order_id();
              int lane_relative_id =
                  lane_order_id - order_ids[zero_relative_id_order_id_index];
              lane->set_relative_id(lane_relative_id);
            }
            is_exist_split_on_ramp_ = true;
            return;
          } else {
            return;
          }
        }
      }
    }
  }

  if (!is_on_ramp_ && split_direction_dis_info_list_.size() > 1) {
    if (distance_to_first_road_merge_ >
        split_direction_dis_info_list_[1].second) {
      is_exist_split_on_ramp_ = true;
      if (split_direction_dis_info_list_[1].first == ON_RIGHT) {
        relative_id_lanes[order_ids[1]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 1;
        last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
      } else if (split_direction_dis_info_list_[1].first == ON_LEFT) {
        relative_id_lanes[order_ids[0]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 0;
        last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
      } else {
        is_exist_split_on_ramp_ = false;
        return;
      }
    } else {
      is_exist_split_on_ramp_ = false;
      return;
    }
  } else {
    if (distance_to_first_road_merge_ > distance_to_first_road_split_) {
      is_exist_split_on_ramp_ = true;
      if (first_split_dir_dis_info_.first == ON_RIGHT) {
        relative_id_lanes[order_ids[1]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 1;
        last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
      } else if (first_split_dir_dis_info_.first == ON_LEFT) {
        relative_id_lanes[order_ids[0]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 0;
        last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
      } else {
        is_exist_split_on_ramp_ = false;
        return;
      }
    } else {
      bool last_ego_lane_exist_virtual = true;
      MakesureVirtualLaneIsVirtual(last_track_ego_lane_,
                                   last_ego_lane_exist_virtual);

      if (!last_ego_lane_exist_virtual) {
        is_exist_split_on_ramp_ = false;
        return;
      }
      for (size_t i = 0; i < order_ids.size(); i++) {
        if (relative_id_lanes.size() > order_ids[i]) {
          std::shared_ptr<VirtualLane> base_lane =
              relative_id_lanes[order_ids[i]];
          if (base_lane == nullptr) {
            continue;
          }
          bool virtual_lane_exist_virtual = true;
          MakesureVirtualLaneIsVirtual(base_lane, virtual_lane_exist_virtual);
          if (virtual_lane_exist_virtual) {
            continue;
          } else {
            is_exist_split_on_ramp_ = true;
            relative_id_lanes[order_ids[i]]->set_relative_id(0);
            origin_order_id = relative_id_lanes[order_ids[i]]->get_order_id();
            last_zero_relative_id_order_id_index_ = i;
            last_track_ego_lane_ = relative_id_lanes[order_ids[i]];
            break;
          }
        }
      }
    }
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  return;
}

void EgoLaneTrackManger::PreprocessOrdinarySplit(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane) {
  int origin_order_id = 0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{ego_x, ego_y};
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const int fix_lane_virtual_id =
      lane_change_decider_output.fix_lane_virtual_id;
  int zero_relative_id_order_id_index = last_zero_relative_id_order_id_index_;
  bool enable_using_last_frame_track_ego_lane = true;
  std::shared_ptr<VirtualLane> last_track_virtual_id_lane;

  if (!virtual_id_mapped_lane.empty()) {
    for (const auto& virtual_id_lane : virtual_id_mapped_lane) {
      if (virtual_id_lane.second->get_virtual_id() == fix_lane_virtual_id) {
        last_track_virtual_id_lane = virtual_id_lane.second;
      }
    }
  } else {
    last_track_virtual_id_lane = nullptr;
  }

  if (last_zero_relative_id_nums_ > 1) {
    LOG_DEBUG("PreprocessOrdinarySplit::last_zero_relative_id_nums_ > 1");
    if (last_zero_relative_id_order_id_index_ != -1) {
      if (order_ids.size() == 2 &&
          last_zero_relative_id_nums_ == order_ids.size() &&
          zero_relative_id_order_id_index < order_ids.size()) {
        last_track_ego_lane_ =
            relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
        for (auto& lane : relative_id_lanes) {
          int lane_order_id = lane->get_order_id();
          int lane_relative_id =
              lane_order_id - order_ids[last_zero_relative_id_order_id_index_];
          lane->set_relative_id(lane_relative_id);
        }
        is_exist_split_on_expressway_ = true;
        return;
      } else {
        ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_,
                                          relative_id_lanes, order_ids,
                                          zero_relative_id_order_id_index);

        if (enable_using_last_frame_track_ego_lane) {
          if (zero_relative_id_order_id_index < order_ids.size() &&
              relative_id_lanes.size() >
                  order_ids[zero_relative_id_order_id_index]) {
            last_track_ego_lane_ =
                relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
            last_zero_relative_id_order_id_index_ =
                zero_relative_id_order_id_index;
            for (auto& lane : relative_id_lanes) {
              int lane_order_id = lane->get_order_id();
              int lane_relative_id =
                  lane_order_id - order_ids[zero_relative_id_order_id_index];
              lane->set_relative_id(lane_relative_id);
            }
            is_exist_split_on_expressway_ = true;
            return;
          } else {
            return;
          }
        }
      }
    }
  }

  ComputeZeroRelativeIdOrderIdIndex(last_track_virtual_id_lane,
                                    relative_id_lanes, order_ids,
                                    zero_relative_id_order_id_index);
  bool enable_using_last_track_virtual_id_lane = true;
  if (zero_relative_id_order_id_index < order_ids.size() &&
      relative_id_lanes.size() > order_ids[zero_relative_id_order_id_index]) {
    std::shared_ptr<VirtualLane> first_check_last_lane =
        relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
    bool last_ego_lane_exist_virtual = true;
    MakesureVirtualLaneIsVirtual(first_check_last_lane,
                                 last_ego_lane_exist_virtual);

    if (last_ego_lane_exist_virtual) {
      enable_using_last_track_virtual_id_lane = false;
    } else {
      last_track_ego_lane_ =
          relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
      last_zero_relative_id_order_id_index_ = zero_relative_id_order_id_index;
      is_exist_split_on_expressway_ = true;
      relative_id_lanes[order_ids[zero_relative_id_order_id_index]]
          ->set_relative_id(0);
      origin_order_id =
          relative_id_lanes[order_ids[zero_relative_id_order_id_index]]
              ->get_order_id();
    }
  }

  if (!enable_using_last_track_virtual_id_lane) {
    for (size_t i = 0; i < order_ids.size(); i++) {
      if (relative_id_lanes.size() > order_ids[i]) {
        std::shared_ptr<VirtualLane> base_lane =
            relative_id_lanes[order_ids[i]];
        if (base_lane == nullptr) {
          continue;
        }
        bool base_lane_exist_virtual = true;
        MakesureVirtualLaneIsVirtual(base_lane, base_lane_exist_virtual);

        if (base_lane_exist_virtual) {
          continue;
        } else {
          is_exist_split_on_expressway_ = true;
          relative_id_lanes[order_ids[i]]->set_relative_id(0);
          origin_order_id = relative_id_lanes[order_ids[i]]->get_order_id();
          last_zero_relative_id_order_id_index_ = i;
          last_track_ego_lane_ = relative_id_lanes
              [order_ids[last_zero_relative_id_order_id_index_]];
          break;
        }
      }
    }
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  return;
}

void EgoLaneTrackManger::ProcessIntersectionSplit(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const bool is_in_lane_borrow_status = session_->planning_context()
                                            .lane_borrow_decider_output()
                                            .is_in_lane_borrow_status;
  int origin_order_id = 0;
  int current_order_id = 0;
  const double default_lane_mapping_cost = 10.0;
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  int zero_relative_id_order_id_index = last_zero_relative_id_order_id_index_;
  bool enable_using_last_frame_track_ego_lane = true;
  bool find_last_frame_track_ego_lane = true;

  if (last_zero_relative_id_nums_ > 1) {
    LOG_DEBUG("ProcessIntersectionSplit::last_zero_relative_id_nums_ > 1");
    if (last_zero_relative_id_order_id_index_ != -1) {
      if (order_ids.size() == 2 &&
          last_zero_relative_id_nums_ == order_ids.size() &&
          zero_relative_id_order_id_index < order_ids.size()) {
        last_track_ego_lane_ =
            relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
        for (auto& lane : relative_id_lanes) {
          int lane_order_id = lane->get_order_id();
          int lane_relative_id =
              lane_order_id - order_ids[last_zero_relative_id_order_id_index_];
          lane->set_relative_id(lane_relative_id);
        }
        is_exist_split_on_intersection_ = true;
        return;
      } else {
        ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_,
                                          relative_id_lanes, order_ids,
                                          zero_relative_id_order_id_index);

        if (enable_using_last_frame_track_ego_lane) {
          if (zero_relative_id_order_id_index < order_ids.size() &&
              relative_id_lanes.size() >
                  order_ids[zero_relative_id_order_id_index]) {
            last_track_ego_lane_ =
                relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
            last_zero_relative_id_order_id_index_ =
                zero_relative_id_order_id_index;
            for (auto& lane : relative_id_lanes) {
              int lane_order_id = lane->get_order_id();
              int lane_relative_id =
                  lane_order_id - order_ids[zero_relative_id_order_id_index];
              lane->set_relative_id(lane_relative_id);
            }
            is_exist_split_on_intersection_ = true;
            return;
          } else {
            return;
          }
        }
      }
    }
  }

  const double ego_heading_angle = ego_state->heading_angle();
  double clane_min_cost_total = std::numeric_limits<double>::max();
  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      if (relative_id_lane == nullptr) {
        continue;
      }
      if (relative_id_lane->get_lane_frenet_coord() != nullptr) {
        double total_cost = 0.0;
        double average_heading_angle_cost = 0.0;
        double heading_angle_cost = 0.0;
        double average_kappa_cost = 0.0;
        double total_kappa_cost = 0.0;
        std::shared_ptr<KDPath> frenet_coord =
            relative_id_lane->get_lane_frenet_coord();
        double ego_s = 0.0;
        double ego_l = 0.0;
        Point2D ego_frenet_point;
        if (!frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
          continue;
        } else {
          ego_s = ego_frenet_point.x;
          ego_l = ego_frenet_point.y;
        }
        if (ego_s > frenet_coord->Length()) {
          continue;
        }
        planning_math::PathPoint ego_s_nearest_point =
            frenet_coord->GetPathPointByS(ego_s);
        int iter_count = 0;
        int theta_diff_iter_count = 0;
        for (double s = ego_s_nearest_point.s(); s < frenet_coord->Length();
             s += kLaneLineSegmentLength) {
          if (s > kDefaultMappingConsiderLaneLength + ego_s) {
            break;
          }
          double current_kappa;
          frenet_coord->GetKappaByS(s, &current_kappa);
          total_kappa_cost += std::fabs(current_kappa);
          iter_count++;

          double heading_angle = frenet_coord->GetPathCurveHeading(s);
          double theta_diff = NormalizeAngle(heading_angle - ego_heading_angle);
          heading_angle_cost += std::fabs(theta_diff);
          theta_diff_iter_count++;
        }
        iter_count = std::max(1, iter_count);
        average_kappa_cost = total_kappa_cost / iter_count;
        theta_diff_iter_count = std::max(1, theta_diff_iter_count);
        average_heading_angle_cost = heading_angle_cost / theta_diff_iter_count;

        total_cost = kAverageThetaDiffCostWeight * average_heading_angle_cost +
            kAverageKappaCostWeight * average_kappa_cost;
        if (total_cost < clane_min_cost_total) {
          clane_min_cost_total = total_cost;
          origin_order_id = relative_id_lane->get_order_id();
          relative_id_lane->set_relative_id(0);
          last_zero_relative_id_order_id_index_ = i;
          last_track_ego_lane_ = relative_id_lanes[order_ids[i]];
          is_exist_split_on_intersection_ = true;
        }
      }
    }
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }

  return;
}

void EgoLaneTrackManger::ProcessSplitWithGroundMark(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  int origin_order_id = 0;
  bool both_exist_straight_direction = true;
  bool enable_using_last_frame_track_ego_lane = true;
  bool find_last_frame_track_ego_lane = true;
  int zero_relative_id_order_id_index = last_zero_relative_id_order_id_index_;
  if (order_ids.size() < 2) {
    LOG_DEBUG("ProcessSplitWithGroundMark::order_ids.size() < 2");
    is_exist_split_on_intersection_ = false;
    return;
  }
  // 根据感知所检出的分流场景（一分二或者一分多） 选择符合预期分叉
  // 优先判断上一帧选择的车道结果是否为直行车道
  if (last_zero_relative_id_order_id_index_ != -1) {
    ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                      order_ids,
                                      zero_relative_id_order_id_index);

    if (zero_relative_id_order_id_index < order_ids.size() &&
        relative_id_lanes.size() > order_ids[zero_relative_id_order_id_index]) {
      std::shared_ptr<VirtualLane> base_lane =
          relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
      bool is_exist_strgight_direction =
          MakesureVirtualLaneExistStraightDirecton(base_lane);
      if (!is_exist_strgight_direction) {
        enable_using_last_frame_track_ego_lane = false;
      }
    } else {
      enable_using_last_frame_track_ego_lane = false;
    }

    if (enable_using_last_frame_track_ego_lane) {
      last_track_ego_lane_ =
          relative_id_lanes[order_ids[zero_relative_id_order_id_index]];
      last_zero_relative_id_order_id_index_ = zero_relative_id_order_id_index;
      for (auto& lane : relative_id_lanes) {
        int lane_order_id = lane->get_order_id();
        int lane_relative_id =
            lane_order_id - order_ids[zero_relative_id_order_id_index];
        lane->set_relative_id(lane_relative_id);
      }
      is_exist_split_on_intersection_ = true;
      return;
    }
  }

  // 优先判断上一帧追踪的车道是否为直行车道
  bool last_ego_lane_exist_straight =
      MakesureLastEgoLaneExistStraightDirecton(last_track_ego_lane_);

  if (relative_id_lanes.size() == order_ids.size() ||
      last_ego_lane_exist_straight) {
    LOG_DEBUG("relative_id_lanes.size() == order_ids.size()");
    is_exist_split_on_intersection_ = false;
    return;
  }

  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> base_lane = relative_id_lanes[order_ids[i]];
      bool exist_straight_direction =
          MakesureVirtualLaneExistStraightDirecton(base_lane);
      if (!exist_straight_direction) {
        both_exist_straight_direction = false;
        break;
      }
    }
  }

  if (!both_exist_straight_direction) {
    for (size_t i = 0; i < order_ids.size(); i++) {
      if (relative_id_lanes.size() > order_ids[i]) {
        std::shared_ptr<VirtualLane> base_lane =
            relative_id_lanes[order_ids[i]];
        if (base_lane == nullptr) {
          continue;
        }
        std::shared_ptr<KDPath> base_lane_frenet_crd =
            base_lane->get_lane_frenet_coord();
        if (base_lane_frenet_crd == nullptr) {
          continue;
        }

        // bool only_exist_straight_direction = true;
        bool exist_only_left_or_right_direction = false;
        exist_only_left_or_right_direction =
            MakesureVirtualLaneExistOtherDirecton(base_lane);
        if (exist_only_left_or_right_direction) {
          continue;
        } else {
          relative_id_lanes[order_ids[i]]->set_relative_id(0);
          origin_order_id = relative_id_lanes[order_ids[i]]->get_order_id();
          is_exist_split_on_intersection_ = true;
          last_zero_relative_id_order_id_index_ = i;
          last_track_ego_lane_ = relative_id_lanes[order_ids[i]];
          break;
        }
      }
    }
  }

  if (!is_exist_split_on_intersection_ && order_ids.size() == 2) {
    bool is_on_right_side_lane = false;
    bool is_on_left_side_lane = false;
    for (size_t i = 0; i < order_ids.size(); i++) {
      if (order_ids[i] == 0) {
        is_on_left_side_lane = true;
        break;
      }
    }
    for (size_t i = 0; i < order_ids.size(); i++) {
      if (order_ids[i] == relative_id_lanes.size() - 1) {
        is_on_right_side_lane = true;
        break;
      }
    }
    if (is_on_left_side_lane && is_on_right_side_lane) {
      is_exist_split_on_intersection_ = false;
      return;
    } else if (is_on_left_side_lane) {
      if (MakesureVirtualLaneExistStraightDirecton(
              relative_id_lanes[order_ids[1]])) {
        relative_id_lanes[order_ids[1]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 1;
        last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
      } else {
        is_exist_split_on_intersection_ = false;
        return;
      }
    } else if (is_on_right_side_lane) {
      if (MakesureVirtualLaneExistStraightDirecton(
              relative_id_lanes[order_ids[0]])) {
        relative_id_lanes[order_ids[0]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 0;
        last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
      } else {
        is_exist_split_on_intersection_ = false;
        return;
      }
    } else {
      is_exist_split_on_intersection_ = false;
      return;
    }

    is_exist_split_on_intersection_ = true;
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  return;
}

void EgoLaneTrackManger::CalculateVirtualLaneAttributes(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes) {
  const auto& location_valid = session_->environmental_model().location_valid();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  Point2D ego_cart_point_frenet;
  for (auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane != nullptr) {
      double ego_lateral_offset = 0.0;
      auto& lane_points = relative_id_lane->lane_points();
      if (lane_points.size() < 3) {
        continue;
      }
      std::shared_ptr<KDPath> frenet_coord;
      std::vector<planning_math::PathPoint> path_points;
      path_points.reserve(lane_points.size());

      if (location_valid) {
        for (const auto& point : lane_points) {
          if (std::isnan(point.local_point.x) ||
              std::isnan(point.local_point.y)) {
            LOG_ERROR("update_lane_points: skip NaN point");
            continue;
          }
          auto pt = planning_math::PathPoint(point.local_point.x,
                                             point.local_point.y);
          // std ::cout << "path_point: " << pt.x() << "," << pt.y()
          // <<std::endl;
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
          frenet_coord = nullptr;
          continue;
        }
        frenet_coord =
            std::make_shared<planning_math::KDPath>(std::move(path_points));
        relative_id_lane->set_lane_frenet_coord(frenet_coord);

        if (!frenet_coord->XYToSL(ego_cart_point, ego_cart_point_frenet)) {
          ego_lateral_offset = 0.0;
        } else {
          ego_lateral_offset = ego_cart_point_frenet.y;
        }
      } else {
        for (const auto& point : lane_points) {
          if (std::isnan(point.car_point.x) || std::isnan(point.car_point.y)) {
            LOG_ERROR("update_lane_points: skip NaN point");
            continue;
          }
          auto pt =
              planning_math::PathPoint(point.car_point.x, point.car_point.y);
          // std ::cout << "path_point: " << pt.x() << "," << pt.y()
          // <<std::endl;
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
          frenet_coord = nullptr;
          continue;
        }
        frenet_coord =
            std::make_shared<planning_math::KDPath>(std::move(path_points));
        relative_id_lane->set_lane_frenet_coord(frenet_coord);

        if (!frenet_coord->XYToSL(ego_cart_point, ego_cart_point_frenet)) {
          ego_lateral_offset = 0.0;
        } else {
          ego_lateral_offset = ego_cart_point_frenet.y;
        }
      }
      relative_id_lane->set_ego_lateral_offset(ego_lateral_offset);
    }
  }
}

double EgoLaneTrackManger::ComputeLanesMatchlaterakDisCost(
    int virtual_id, const std::shared_ptr<VirtualLane> current_relative_id_lane,
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane) {
  const double default_lane_mapping_cost = 10.0;
  double average_curv = 0.0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  for (const auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane->get_relative_id() == 0) {
      average_curv = ComputeTargetLaneSpecifiedRangeCurvature(relative_id_lane);
      break;
    } else {
      continue;
    }
  }
  double road_radius = 1 / std::max(average_curv, 0.0001);

  auto iter = virtual_id_mapped_lane.find(virtual_id);
  if (iter != virtual_id_mapped_lane.end()) {
    auto target_lane_frenet_coord = iter->second->get_lane_frenet_coord();
    if (target_lane_frenet_coord != nullptr) {
      const auto& lane_points = current_relative_id_lane->lane_points();
      if (lane_points.size() < 3) {
        return default_lane_mapping_cost;
      }
      double lane_mapping_cost = 0.0;
      double total_lateral_offset = 0.0;
      int point_nums = 0;
      double target_ego_s = 0.0;
      double target_ego_l = 0.0;
      if (!target_lane_frenet_coord->XYToSL(ego_state->ego_pose().x,
                                            ego_state->ego_pose().y,
                                            &target_ego_s, &target_ego_l)) {
        return default_lane_mapping_cost;
      }

      if (road_radius > kDefaultRoadRadius) {
        int select_lane_point_interval = 1;
        for (int i = 0; i < lane_points.size();
             i += select_lane_point_interval) {
          iflyauto::ReferencePoint point = lane_points[i];
          if (std::isnan(point.local_point.x) ||
              std::isnan(point.local_point.y)) {
            LOG_ERROR("update_lane_points: skip NaN point");
            continue;
          }
          double lateral_offset = 0.0;
          double s = 0.0;
          double l = 0.0;
          if (!target_lane_frenet_coord->XYToSL(point.local_point.x,
                                                point.local_point.y, &s, &l)) {
            lateral_offset = 10.0;
          } else {
            lateral_offset = l;
          }
          if (s < target_ego_s) {
            continue;
          }
          if (s > target_ego_s + kDefaultMappingConsiderLaneLength) {
            break;
          }
          total_lateral_offset += lateral_offset;
          point_nums += 1;
          if (point_nums >= kDefaultPointNums) {
            break;
          }
        }
      } else {
        for (int i = 0; i < lane_points.size(); i++) {
          iflyauto::ReferencePoint point = lane_points[i];
          if (std::isnan(point.local_point.x) ||
              std::isnan(point.local_point.y)) {
            LOG_ERROR("update_lane_points: skip NaN point");
            continue;
          }
          double lateral_offset = 0.0;
          double s = 0.0;
          double l = 0.0;
          if (!target_lane_frenet_coord->XYToSL(point.local_point.x,
                                                point.local_point.y, &s, &l)) {
            lateral_offset = 10.0;
          } else {
            lateral_offset = l;
          }
          if (s < target_ego_s) {
            continue;
          }
          if (point_nums >= kDefaultPointNums ||
              s > target_ego_s + kConsiderLaneLineLength) {
            break;
          }
          total_lateral_offset += lateral_offset;
          point_nums += 1;
        }
      }
      point_nums = std::max(1, point_nums);
      if (point_nums < kLeastDefaultPointNums) {
        lane_mapping_cost = default_lane_mapping_cost;
      } else {
        lane_mapping_cost = std::fabs(total_lateral_offset / point_nums);
      }
      return lane_mapping_cost;
    } else {
      return default_lane_mapping_cost;
    }
  } else {
    return default_lane_mapping_cost;
  }
  return default_lane_mapping_cost;
}

double EgoLaneTrackManger::ComputeTargetLaneSpecifiedRangeCurvature(
    const std::shared_ptr<VirtualLane> virtual_lane) {
  double average_curv = 0.0;
  const double default_begin_length = 50.0;
  const double default_end_length = 100.0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  std::shared_ptr<KDPath> frenet_coord = virtual_lane->get_lane_frenet_coord();
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!frenet_coord->XYToSL(ego_state->ego_pose().x, ego_state->ego_pose().y,
                            &ego_s, &ego_l)) {
    return average_curv;
  }
  if (ego_s > frenet_coord->Length()) {
    return average_curv;
  }
  planning_math::PathPoint ego_s_nearest_point =
      frenet_coord->GetPathPointByS(ego_s + default_begin_length);
  int iter_count = 0;
  double total_curv = 0.0;
  for (double s = ego_s_nearest_point.s(); s < frenet_coord->Length();
       s += kLaneLineSegmentLength) {
    if (s > default_end_length + ego_s) {
      break;
    }
    double curv = 0.0;
    frenet_coord->GetKappaByS(s, &curv);
    total_curv += curv;
    iter_count++;
  }
  iter_count = std::max(1, iter_count);
  average_curv = std::fabs(total_curv / iter_count);
  return average_curv;
}

double EgoLaneTrackManger::ComputeAverageHeadingDiff(
    std::shared_ptr<VirtualLane> base_lane, const double ego_heading_angle) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};

  if (base_lane != nullptr) {
    if (base_lane->get_lane_frenet_coord() != nullptr) {
      double average_heading_angle_cost = 0.0;
      double heading_angle_cost = 0.0;
      std::shared_ptr<KDPath> frenet_coord = base_lane->get_lane_frenet_coord();
      double ego_s = 0.0;
      double ego_l = 0.0;
      Point2D ego_frenet_point;
      if (!frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
        return M_PI;
      } else {
        ego_s = ego_frenet_point.x;
        ego_l = ego_frenet_point.y;
      }
      if (ego_s > frenet_coord->Length()) {
        return M_PI;
      }
      planning_math::PathPoint ego_s_nearest_point =
          frenet_coord->GetPathPointByS(ego_s);
      int iter_count = 0;
      for (double s = ego_s_nearest_point.s(); s < frenet_coord->Length();
           s += kLaneLineSegmentLength) {
        if (s > kConsiderLaneLineLength + ego_s) {
          break;
        }
        double heading_angle = frenet_coord->GetPathCurveHeading(s);
        double theta_err = NormalizeAngle(heading_angle - ego_heading_angle);
        heading_angle_cost += std::fabs(theta_err);
        iter_count++;
      }
      iter_count = std::max(1, iter_count);
      average_heading_angle_cost = heading_angle_cost / iter_count;

      return average_heading_angle_cost;
    } else {
      return M_PI;
    }
  } else {
    return M_PI;
  }
}

bool EgoLaneTrackManger::CheckIfInRampSelectSplit(
    std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes,
    const std::vector<int>& order_ids) {
  if (!session_->environmental_model().get_route_info()->get_sdmap_valid()) {
    LOG_DEBUG("CheckIfInRampSelectSplit::sd_map is invalid!!!");
    return false;
  }
  LOG_DEBUG("CheckIfInRampSelectSplit::sd_map is valid");

  if (order_ids.size() < 2) {
    LOG_DEBUG("CheckIfInRampSelectSplit::order_ids.size() < 2");
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double default_consider_lane_length = 150.0;
  const double search_distance = 50.0;
  const double max_heading_diff = M_PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};

  const auto& sd_map =
      session_->environmental_model().get_route_info()->get_sd_map();

  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> base_lane = relative_id_lanes[order_ids[i]];
      if (base_lane != nullptr) {
        std::shared_ptr<KDPath> base_lane_frenet_crd =
            base_lane->get_lane_frenet_coord();
        if (base_lane_frenet_crd != nullptr) {
          Point2D ego_cart_frenet_point;
          double ego_s = 0.0;
          if (!base_lane_frenet_crd->XYToSL(ego_cart_point,
                                            ego_cart_frenet_point)) {
            return false;
          } else {
            ego_s = ego_cart_frenet_point.x;
          }
          double target_s = std::min(ego_s + default_consider_lane_length,
                                     base_lane_frenet_crd->Length());
          planning_math::PathPoint ego_s_nearest_point =
              base_lane_frenet_crd->GetPathPointByS(target_s);

          ad_common::math::Vec2d segment_target_point;
          segment_target_point.set_x(ego_s_nearest_point.x());
          segment_target_point.set_y(ego_s_nearest_point.y());
          double nearest_s = 0;
          double nearest_l = 0;
          const auto target_segment = sd_map.GetNearestRoadWithHeading(
              segment_target_point, search_distance, ego_heading_angle,
              max_heading_diff, nearest_s, nearest_l);

          if (target_segment != nullptr) {
            if (target_segment->usage() != SdMapSwtx::RoadUsage::RAMP) {
              return false;
            }
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  return true;
}

bool EgoLaneTrackManger::CheckIfInRoadSelectRamp(
    std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes,
    const std::vector<int>& order_ids) {
  if (!session_->environmental_model().get_route_info()->get_sdmap_valid()) {
    LOG_DEBUG("CheckIfInRoadSelectRamp::sd_map is invalid!!!");
    return false;
  }
  LOG_DEBUG("CheckIfInRoadSelectRamp::sd_map is valid");

  if (order_ids.size() < 2) {
    LOG_DEBUG("CheckIfInRoadSelectRamp::order_ids.size() < 2");
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double default_consider_lane_length = 30.0;
  const double search_distance = 50.0;
  const double max_heading_diff = M_PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};

  const auto& sd_map =
      session_->environmental_model().get_route_info()->get_sd_map();

  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> base_lane = relative_id_lanes[order_ids[i]];
      if (base_lane != nullptr) {
        std::shared_ptr<KDPath> base_lane_frenet_crd =
            base_lane->get_lane_frenet_coord();
        if (base_lane_frenet_crd != nullptr) {
          Point2D ego_cart_frenet_point;
          double ego_s = 0.0;
          if (!base_lane_frenet_crd->XYToSL(ego_cart_point,
                                            ego_cart_frenet_point)) {
            return false;
          } else {
            ego_s = ego_cart_frenet_point.x;
          }
          double target_s = std::min(ego_s + default_consider_lane_length,
                                     base_lane_frenet_crd->Length());
          planning_math::PathPoint ego_s_nearest_point =
              base_lane_frenet_crd->GetPathPointByS(target_s);

          ad_common::math::Vec2d segment_target_point;
          segment_target_point.set_x(ego_s_nearest_point.x());
          segment_target_point.set_y(ego_s_nearest_point.y());
          double nearest_s = 0;
          double nearest_l = 0;
          const auto target_segment = sd_map.GetNearestRoadWithHeading(
              segment_target_point, search_distance, ego_heading_angle,
              max_heading_diff, nearest_s, nearest_l);
          if (target_segment != nullptr) {
            if (target_segment->usage() ==
                    SdMapSwtx::RoadUsage::ORDINARY_ROAD ||
                target_segment->usage() == SdMapSwtx::RoadUsage::SEPARATE) {
              return true;
            }
          } else {
            continue;
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    }
  }

  return false;
}

int EgoLaneTrackManger::CalcTargetLaneLineSegment(
    const std::shared_ptr<VirtualLane> base_lane, Point2D ego_cart_point) {
  int segment = -1;
  if (base_lane == nullptr) {
    return segment;
  }
  std::shared_ptr<KDPath> base_lane_frenet_crd =
      base_lane->get_lane_frenet_coord();
  if (base_lane_frenet_crd == nullptr) {
    return segment;
  }

  Point2D ego_cart_frenet_point;
  double ego_s = 0.0;
  if (!base_lane_frenet_crd->XYToSL(ego_cart_point, ego_cart_frenet_point)) {
    return segment;
  } else {
    ego_s = ego_cart_frenet_point.x;
  }

  const auto& lane_marks = base_lane->lane_marks();
  double lane_line_length = 0.0;
  for (int i = 0; i < lane_marks.size(); i++) {
    lane_line_length = lane_marks[i].end;
    if ((lane_line_length > ego_s + kConsiderLaneStraightFrontEgo) &&
        (lane_marks[i].begin <= ego_s + kConsiderLaneStraightFrontEgo)) {
      segment = i;
      break;
    }
  }
  return segment;
}

void EgoLaneTrackManger::ComputeZeroRelativeIdOrderIdIndex(
    const std::shared_ptr<VirtualLane> last_track_ego_lane,
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids, int& zero_relative_id_order_id_index) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{ego_x, ego_y};

  if (last_track_ego_lane == nullptr) {
    return;
  }
  auto last_track_ego_lane_frenet_coord =
      last_track_ego_lane->get_lane_frenet_coord();
  if (last_track_ego_lane_frenet_coord == nullptr) {
    return;
  }

  double clane_min_diff_total = std::numeric_limits<double>::max();
  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      const auto& lane_points = relative_id_lane->lane_points();
      if (lane_points.size() <= 2) {
        continue;
      }
      auto cur_lane_frenet_coord = relative_id_lane->get_lane_frenet_coord();
      if (cur_lane_frenet_coord == nullptr) {
        continue;
      }
      double target_ego_s = 0.0;
      double target_ego_l = 0.0;
      Point2D ego_cart_target_frenet;
      if (!cur_lane_frenet_coord->XYToSL(ego_cart_point,
                                         ego_cart_target_frenet)) {
        continue;
      } else {
        target_ego_s = ego_cart_target_frenet.x;
        target_ego_l = ego_cart_target_frenet.y;
      }
      int point_nums = 0;
      double total_lateral_offset = 0.0;
      double cumu_lat_dis_cost = 0.0;
      int select_lane_point_interval = 1;
      for (int i = 0; i < lane_points.size(); i += select_lane_point_interval) {
        iflyauto::ReferencePoint point = lane_points[i];
        if (std::isnan(point.local_point.x) ||
            std::isnan(point.local_point.y)) {
          LOG_ERROR("update_lane_points: skip NaN point");
          continue;
        }
        double lateral_offset = 0.0;
        double s = 0.0;
        Point2D cur_point_frenet;
        Point2D cur_point(point.local_point.x, point.local_point.y);
        if (!last_track_ego_lane_frenet_coord->XYToSL(cur_point,
                                                      cur_point_frenet)) {
          lateral_offset = 10.0;
        } else {
          s = cur_point_frenet.x;
          lateral_offset = cur_point_frenet.y;
        }
        if (s < target_ego_s) {
          continue;
        }
        total_lateral_offset += lateral_offset;
        point_nums += 1;
        if (point_nums >= kDefaultPointNums ||
            s > target_ego_s + kDefaultMappingConsiderLaneLength) {
          break;
        }
      }
      point_nums = std::max(1, point_nums);
      cumu_lat_dis_cost = std::fabs(total_lateral_offset / point_nums);
      if (cumu_lat_dis_cost < clane_min_diff_total) {
        clane_min_diff_total = cumu_lat_dis_cost;
        zero_relative_id_order_id_index = i;
      }
    }
  }
  return;
}

bool EgoLaneTrackManger::MakesureVirtualLaneExistStraightDirecton(
    const std::shared_ptr<VirtualLane> base_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  if (base_lane == nullptr) {
    return false;
  }
  std::shared_ptr<KDPath> base_lane_frenet_crd =
      base_lane->get_lane_frenet_coord();
  if (base_lane_frenet_crd == nullptr) {
    return false;
  }

  const auto& lane_marks = base_lane->lane_marks();
  Point2D ego_cart_frenet_point;
  bool exist_straight_direction = true;
  double ego_s = 0.0;
  if (!base_lane_frenet_crd->XYToSL(ego_cart_point, ego_cart_frenet_point)) {
    return false;
  } else {
    ego_s = ego_cart_frenet_point.x;
  }

  int segment = CalcTargetLaneLineSegment(base_lane, ego_cart_point);
  if (segment >= 0) {
    for (int i = segment; i < lane_marks.size(); i++) {
      if (lane_marks[i].begin > ego_s + kDefaultConsiderLaneMarksLength) {
        break;
      }
      if (lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_LEFT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_RIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_LEFT_RIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_UTURN_LEFT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_UTURN_RIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_OFF_ROUTE &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_UNKNOWN) {
        return false;
      }
    }
  } else {
    return false;
  }
  return true;
}

bool EgoLaneTrackManger::MakesureLastEgoLaneExistStraightDirecton(
    const std::shared_ptr<VirtualLane> base_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  if (base_lane == nullptr) {
    return false;
  }
  std::shared_ptr<KDPath> base_lane_frenet_crd =
      base_lane->get_lane_frenet_coord();
  if (base_lane_frenet_crd == nullptr) {
    return false;
  }

  const auto& lane_marks = base_lane->lane_marks();
  Point2D ego_cart_frenet_point;
  bool exist_straight_direction = true;
  double ego_s = 0.0;
  if (!base_lane_frenet_crd->XYToSL(ego_cart_point, ego_cart_frenet_point)) {
    return false;
  } else {
    ego_s = ego_cart_frenet_point.x;
  }

  int segment = CalcTargetLaneLineSegment(base_lane, ego_cart_point);
  if (segment >= 0) {
    for (int i = segment; i < lane_marks.size(); i++) {
      if (lane_marks[i].begin > ego_s + kDefaultConsiderLaneMarksLength) {
        break;
      }
      if (lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_LEFT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_RIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_LEFT_RIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_UTURN_LEFT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_UTURN_RIGHT &&
          lane_marks[i].lane_mark !=
              iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT_OFF_ROUTE) {
        return false;
      }
    }
  } else {
    return false;
  }
  return true;
}

bool EgoLaneTrackManger::MakesureVirtualLaneExistOtherDirecton(
    const std::shared_ptr<VirtualLane> base_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  if (base_lane == nullptr) {
    return true;
  }
  std::shared_ptr<KDPath> base_lane_frenet_crd =
      base_lane->get_lane_frenet_coord();
  if (base_lane_frenet_crd == nullptr) {
    return true;
  }

  const auto& lane_marks = base_lane->lane_marks();
  Point2D ego_cart_frenet_point;
  double ego_s = 0.0;
  if (!base_lane_frenet_crd->XYToSL(ego_cart_point, ego_cart_frenet_point)) {
    return true;
  } else {
    ego_s = ego_cart_frenet_point.x;
  }

  int segment = CalcTargetLaneLineSegment(base_lane, ego_cart_point);
  if (segment >= 0) {
    for (int i = segment; i < lane_marks.size(); i++) {
      if (lane_marks[i].begin > ego_s + kDefaultConsiderLaneMarksLength) {
        break;
      }
      if (lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_LEFT ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_RIGHT ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_UTURN_LEFT ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_UTURN_RIGHT ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_LEFT_UTURN ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_RIGHT_UTURN ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_UNKNOWN ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_LEFT_RIGHT ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_LEFT_MERGE ||
          lane_marks[i].lane_mark ==
              iflyauto::LaneDrivableDirection_DIRECTION_RIGHT_MERGE) {
        return true;
      }
    }
  } else {
    return true;
  }
  return false;
}

void EgoLaneTrackManger::MakesureVirtualLaneIsVirtual(
    const std::shared_ptr<VirtualLane> base_lane,
    bool& virtual_lane_exist_virtual) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  if (base_lane == nullptr) {
    return;
  }
  bool left_boundary_exist_virtual_type = false;
  double left_lane_line_length = 0.0;
  int left_current_segment_count = 0;
  double left_ego_s = 0.0, left_ego_l = 0.0;
  // 判断左侧车道线类型
  const auto& left_lane_boundarys = base_lane->get_left_lane_boundary();
  std::shared_ptr<planning_math::KDPath> left_base_boundary_path =
      MakeBoundaryPath(left_lane_boundarys);
  if (left_base_boundary_path != nullptr) {
    if (!left_base_boundary_path->XYToSL(ego_x, ego_y, &left_ego_s,
                                         &left_ego_l)) {
      return;
    }
  } else {
    return;
  }
  for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
    left_lane_line_length += left_lane_boundarys.type_segments[i].length;
    if (left_lane_line_length > left_ego_s) {
      left_current_segment_count = i;
      break;
    }
  }
  for (int i = left_current_segment_count;
       i < left_lane_boundarys.type_segments_size; i++) {
    if (left_lane_boundarys.type_segments[i].type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      left_boundary_exist_virtual_type = true;
      break;
    } else {
      continue;
    }
  }

  // 判断右侧车道线类型
  bool right_boundary_exist_virtual_type = false;
  double right_lane_line_length = 0.0;
  int right_current_segment_count = 0;
  double right_ego_s = 0.0, right_ego_l = 0.0;
  const auto& right_lane_boundarys = base_lane->get_right_lane_boundary();
  std::shared_ptr<planning_math::KDPath> right_base_boundary_path =
      MakeBoundaryPath(right_lane_boundarys);
  if (right_base_boundary_path != nullptr) {
    if (!right_base_boundary_path->XYToSL(ego_x, ego_y, &right_ego_s,
                                          &right_ego_l)) {
      return;
    }
  } else {
    return;
  }
  for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
    right_lane_line_length += right_lane_boundarys.type_segments[i].length;
    if (right_lane_line_length > right_ego_s) {
      right_current_segment_count = i;
      break;
    }
  }
  for (int i = right_current_segment_count;
       i < right_lane_boundarys.type_segments_size; i++) {
    if (right_lane_boundarys.type_segments[i].type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      right_boundary_exist_virtual_type = true;
      break;
    } else {
      continue;
    }
  }

  if (!left_boundary_exist_virtual_type && !right_boundary_exist_virtual_type) {
    virtual_lane_exist_virtual = false;
  }
  return;
}

void EgoLaneTrackManger::ComputeIsSplitRegion(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  std::shared_ptr<VirtualLane> relative_left_lane = nullptr;
  std::shared_ptr<VirtualLane> relative_right_lane = nullptr;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_vel = ego_state->ego_v();
  // if (ego_vel < kApprochingRampSpeedThd) {
  //   return;
  // }
  Point2D ego_point = {ego_state->planning_init_point().x,
                       ego_state->planning_init_point().y};
  if (order_ids.size() < 2) {
    ego_in_split_region_ = false;
    return;
  }else if (order_ids.size() > 2) {
    ego_in_split_region_ = true;
    return;
  } else {
    if (order_ids[0] < relative_id_lanes.size()) {
      relative_left_lane = relative_id_lanes[order_ids[0]];
    }
    if (order_ids[1] < relative_id_lanes.size()) {
      relative_right_lane = relative_id_lanes[order_ids[1]];
    }

    bool has_left_lane = false;
    bool has_right_lane = false;
    if (relative_left_lane != nullptr) {
      if (relative_left_lane->get_lane_frenet_coord() != nullptr) {
        has_left_lane = true;
      }
    }
    if (relative_right_lane != nullptr) {
      if (relative_right_lane->get_lane_frenet_coord() != nullptr) {
        has_right_lane = true;
      }
    }

    const auto& relative_left_lane_points = relative_left_lane->lane_points();
    if (!has_left_lane || !has_right_lane ||
        relative_left_lane_points.size() < 3) {
      ego_in_split_region_ = false;
      return;
    }

    const auto& right_lane_frenet_crd =
        relative_right_lane->get_lane_frenet_coord();
    double ego_s_base_right = 0.0;
    double ego_l_base_right = 0.0;
    right_lane_frenet_crd->XYToSL(ego_point.x, ego_point.y, &ego_s_base_right,
                                  &ego_l_base_right);
    double near_average_l = 0.0;
    double far_average_l = 0.0;
    int32_t near_pt_count = 0;
    int32_t far_pt_count = 0;
    double near_pt_sum_l = 0.0;
    double far_pt_sum_l = 0.0;
    for (const auto point : relative_left_lane_points) {
      if (std::isnan(point.local_point.x) || std::isnan(point.local_point.y)) {
        LOG_ERROR("update_lane_points: skip NaN point");
        continue;
      }
      double pt_s = 0.0;
      double pt_l = 0.0;
      right_lane_frenet_crd->XYToSL(point.local_point.x, point.local_point.y,
                                    &pt_s, &pt_l);
      const double pt_distance_to_ego = pt_s - ego_s_base_right;
      if (pt_l < 0.0 || pt_s > right_lane_frenet_crd->Length()) {
        continue;
      }
      if (pt_distance_to_ego > -kNearPreviewDistanceThd &&
          pt_distance_to_ego < 0.0) {
        ++near_pt_count;
        near_pt_sum_l += pt_l;
      }
      if (pt_distance_to_ego > kEgoPreviewTimeMinThd * ego_vel &&
          pt_distance_to_ego < kEgoPreviewTimeMaxThd * ego_vel) {
        ++far_pt_count;
        far_pt_sum_l += pt_l;
      }
    }
    near_pt_count = std::max(near_pt_count, 1);
    far_pt_count = std::max(far_pt_count, 1);

    near_average_l = std::fabs(near_pt_sum_l / near_pt_count);
    far_average_l = std::fabs(far_pt_sum_l / far_pt_count);

    if ((far_average_l - kExistSplitLateralDisThd > near_average_l) ||
        near_average_l < kCenterLineLateralDisThd) {
      ego_in_split_region_ = true;
      return;
    }
  }

  return;
}

}  // namespace planning