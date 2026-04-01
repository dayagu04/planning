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
#include "log_glog.h"
#include "math/box2d.h"
#include "math/math_utils.h"
#include "math/vec2d.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "stop_line.h"
#include "task_basic_types.h"
#include "tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"
#include "vec2d.h"
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
constexpr double kInitPosCostWeight = 2.5;
constexpr double kCumuLateralDistanceCostWeight = 1.5;
constexpr double kCrossLaneCostWeight = 1.0;
constexpr double kLaneChangeExecutionWeightRatio = 0.5;
constexpr double kLaneChangeOrderidDiffWeight = 1.0;

constexpr int32_t kLaneCenterMinPointsThr = 3;
constexpr double kLaneLineSegmentLength = 3.0;
constexpr double kConsiderLaneLineLength = 50.0;
constexpr double kDefaultRoadRadius = 400.0;
constexpr int32_t kDefaultPointNums = 40;
constexpr int32_t kLeastDefaultPointNums = 3;
constexpr double kConsiderLaneStraightFrontEgo = 30.0;
constexpr double kDefaultMappingConsiderLaneLength = 80.0;
constexpr double kDefaultConsiderLaneMarksLength = 80.0;
constexpr double kEgoPreviewTimeMinThd = 3.0;
constexpr double kEgoPreviewTimeMaxThd = 5.0;
constexpr double kExistSplitLateralDisThd = 1.5;
constexpr double kCenterLineLateralDisThd = 0.8;
constexpr double kExistSplitEgoRearLateralDisThd = 1.5;
constexpr double kNearPreviewDistanceThd = 20.0;
constexpr double kManualLaneChangeDisThd = 1.5;
constexpr double kConsiderManualLength = 80.0;

constexpr int kDefaultLaneChangeOrderIdDiff = 1;
constexpr double kDistanceToLaneMergeSplitPointThd = 10.0;
constexpr double kDefaultConsiderVirtualLineLength = 65.0;
constexpr double kDefaultConsiderSplitSelectorDistance = 15.0;
constexpr double kDefaultSurpressSplitSelectorDistance = 5.0;
constexpr double kDefaultSplitPointExistDistanceThd = 0.16;
constexpr double kComputeSplitPointMoveStep = 2.0;
constexpr double kSplitSelectEgoToExchangeAreaDistanceThd = 150.0;
constexpr double kEnableSplitSelectionEgoLateralDistanceToBothLaneLines = 0.5;
constexpr double kEgoLookAheadTime = 3.0;     // 自车前瞻时间
constexpr double kSamplingStepI = 20.0;       // i循环采样步长
constexpr double kSamplingStepJ = 5.5;        // j循环采样步长
constexpr int kSamplingRangeJ = 2;            // j循环范围（-2到2）
constexpr double kMinCurvature = 0.0001;      // 最小曲率（避免除零）
constexpr double kDefaultCurvature = 0.0001;  // 默认曲率值
// 曲率最大值
constexpr double kMaxKappa = 0.01;
// 横向距离最大值（m）：车道宽度（比如3.75m）
constexpr double kMaxLateralDistance = 3.75;
constexpr double kMinCurvatureRadius = 100.0;  // 最小曲率半径（对应最大曲率）
constexpr double kMaxCurvatureRadius = 1000.0;  // 最大曲率半径（视为纯直道）
constexpr double kAverageKappaCostWeight = 0.2;
constexpr double kAverageThetaDiffCostWeight = 0.2;
constexpr double kEgoLateralDistanceCostWeight = 0.1;
constexpr double kUseVirtualLaneProcessSplitCostThd = 0.3;
constexpr double kACCLeftOffsetThreshold =
    0.8;  // ACC模式下左偏移阈值（米），可根据需求调整
constexpr double kACCRightOffsetThreshold =
    0.8;  // ACC模式下左偏移阈值（米），可根据需求调整
constexpr double kACCHeadingLeftThreshold =
    M_PI / 36.0;  // 航向角向左偏差阈值（5°），可调整
constexpr double kACCHeadingRightThreshold =
    M_PI / 36.0;  // 航向角向左偏差阈值（5°），可调整
constexpr double kLeftRightRatioThreshold = 0.7;  // 偏左/偏右点占比阈值（70%）
constexpr double kNormalizeAnglePI = M_PI;
}  // namespace

EgoLaneTrackManger::EgoLaneTrackManger(
    const EgoPlanningConfigBuilder* config_builder,
    planning::framework::Session* session) {
  session_ = session;
  SetConfig(config_builder);
  // order_ids_of_same_zero_relative_id_.clear();
}

void EgoLaneTrackManger::SetConfig(
    const EgoPlanningConfigBuilder* config_builder) {
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
  const int lane_change_cmd =
      lane_change_decider_output.coarse_planning_info.int_lane_change_cmd;
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  bool is_process_split_exchange = false;
  const bool is_in_lane_borrow_status = session_->planning_context()
                                            .lane_borrow_decider_output()
                                            .is_in_lane_borrow_status;
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  const bool lane_keep_status = (lane_change_status == kLaneKeeping ||
                                 lane_change_status == kLaneChangePropose ||
                                 lane_change_status == kLaneChangeComplete);

  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const double dis_to_split_threshold = 1000.0;
  // const double dis_to_last_split_threshold = 150.0;
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const int zero_relative_id_nums = order_ids_of_same_zero_relative_id.size();
  const bool enable_use_ground_mark =
      config_.enable_use_ground_mark_process_split;
  const auto intersection_state = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                          .get_virtual_lane_manager()
                                          ->GetEgoDistanceToStopline();

  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      distance_to_stopline <= 10.0;

  is_exist_split_on_ramp_ = false;
  is_exist_ramp_on_road_ = false;
  is_exist_split_on_expressway_ = false;
  is_exist_split_on_intersection_ = false;
  is_interactive_select_split_situation_ = false;
  is_exist_interactive_select_split_ = false;
  virtual_lane_relative_id_switch_flag_ = false;
  is_select_ego_lane_without_plan_ = false;
  is_select_ego_lane_with_plan_ = false;
  is_in_ramp_select_split_situation_ = false;
  is_on_road_select_ramp_situation_ = false;
  other_split_lane_right_side_ = false;
  other_split_lane_left_side_ = false;
  current_fix_lane_order_id_ = -1;
  double road_merge_and_split_exchange_distance_thld =
      std::max(60.0, ego_state->ego_v() * 4.0);

  //判断自车是否处于分流场景
  ComputeIsSplitRegion(relative_id_lanes, order_ids_of_same_zero_relative_id,
                       virtual_id_mapped_lane);

  if (!route_info_output.map_split_region_info_list.empty()) {
    if (!route_info_output.map_merge_region_info_list.empty()) {
      is_process_split_exchange =
          (route_info_output.map_split_region_info_list[0]
               .distance_to_split_point <
           route_info_output.map_merge_region_info_list[0]
               .distance_to_merge_point) ||
          (std::fabs(route_info_output.map_split_region_info_list[0]
                         .distance_to_split_point -
                     route_info_output.map_merge_region_info_list[0]
                         .distance_to_merge_point) <
           road_merge_and_split_exchange_distance_thld);
    } else {
      is_process_split_exchange = true;
    }
  } else {
    is_process_split_exchange = false;
  }
  if (zero_relative_id_nums < 2 || !ego_in_split_region_) {
    last_zero_relative_id_order_id_index_ = -1;
    last_track_ego_lane_ = nullptr;
    lcc_split_select_is_finish_ = false;
    road_split_select_is_finish_ = false;
    ramp_split_select_is_finish_ = false;
    interactive_select_split_counter_ = 0;
    enable_output_split_select_classical_chinese_ = false;
    raw_min_road_radius_ = 1500.0;
    raw_max_road_radius_ = 1500.0;
  }
  if (!active ||
      function_info.function_mode() == common::DrivingFunctionInfo::ACC) {
    SelectEgoLaneWithoutPlan(relative_id_lanes);
    is_select_ego_lane_without_plan_ = true;
  } else {
    if (!planning_result.traj_points.empty()) {
      double last_plan_length = planning_result.traj_points.back().s -
                                planning_result.traj_points.front().s;
      // if (last_plan_length < kLastPlanLengthThr) {
      //   SelectEgoLaneWithoutPlan(relative_id_lanes);
      //   is_select_ego_lane_without_plan_ = true;
      //   return;
      // }
      if (function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
        double distance_to_first_road_split = NL_NMAX;
        double ego_dis_to_split_exchange_area_start = NL_NMAX;
        const auto& split_region_info_list =
            route_info_output.map_split_region_info_list;
        bool enable_process_road_to_ramp = true;
        double ego_to_last_split_end_point_distance =
            route_info_output.sum_dis_to_last_split_exchange_area_end_point;
        if (!split_region_info_list.empty()) {
          distance_to_first_road_split =
              split_region_info_list[0].distance_to_split_point;
          ego_dis_to_split_exchange_area_start =
              distance_to_first_road_split +
              split_region_info_list[0]
                  .start_fp_point.fp_distance_to_split_point;
          enable_process_road_to_ramp = split_region_info_list[0].is_ramp_split;
        }

        if (!is_in_lane_borrow_status && ego_in_split_region_ &&
            ego_dis_to_split_exchange_area_start <
                kSplitSelectEgoToExchangeAreaDistanceThd &&
            sum_distance_from_ego_to_both_center_lines_ <
                kEnableSplitSelectionEgoLateralDistanceToBothLaneLines) {
          bool is_on_road_select_ramp = CheckIfInRoadSelectRampForSdpro(
              relative_id_lanes, order_ids_of_same_zero_relative_id);
          is_on_road_select_ramp_situation_ = is_on_road_select_ramp;
          ILOG_DEBUG << "EgoLaneTrackManger::is_on_road_select_ramp_situation:"
                     << is_on_road_select_ramp_situation_;

          if (is_on_road_select_ramp_situation_ && is_process_split_exchange &&
              lane_keep_status) {
            // hack::针对分流 感知未提供分汇流点信息 作如下后处理
            PreprocessRoadSplit(relative_id_lanes,
                                order_ids_of_same_zero_relative_id);
            ILOG_DEBUG << "EgoLaneTrackManger::is_exist_ramp_on_road:"
                       << is_exist_ramp_on_road_;

            if (is_exist_ramp_on_road_) {
              if ((lane_change_cmd != 0 &&
                   lane_change_status != kLaneChangeComplete) &&
                  ego_in_split_region_ &&
                  sum_distance_from_ego_to_both_center_lines_ <
                      kEnableSplitSelectionEgoLateralDistanceToBothLaneLines) {
                ProcessSplitRegionInteractiveSelectEgoLane(
                    relative_id_lanes, order_ids_of_same_zero_relative_id,
                    lane_change_cmd);
                if (is_exist_interactive_select_split_) {
                  interactive_select_split_counter_ = 3;
                  return;
                } else {
                  interactive_select_split_counter_--;
                  interactive_select_split_counter_ =
                      std::min(interactive_select_split_counter_, 0);
                }
              }
              return;
            }
          }

          bool is_in_ramp_select_split = CheckIfInRampSelectSplitForSdpro(
              relative_id_lanes, order_ids_of_same_zero_relative_id);
          is_in_ramp_select_split_situation_ = is_in_ramp_select_split;
          ILOG_DEBUG << "EgoLaneTrackManger::is_in_ramp_select_split_situation:"
                     << is_in_ramp_select_split_situation_;

          if (is_in_ramp_select_split_situation_ && lane_keep_status) {
            //选择匝道上的分叉
            PreprocessRampSplit(relative_id_lanes,
                                order_ids_of_same_zero_relative_id);
            ILOG_DEBUG << "EgoLaneTrackManger::is_exist_split_on_ramp:"
                       << is_exist_split_on_ramp_;

            if (is_exist_split_on_ramp_) {
              if ((lane_change_cmd != 0 &&
                   lane_change_status != kLaneChangeComplete) &&
                  ego_in_split_region_ &&
                  sum_distance_from_ego_to_both_center_lines_ < 0.5) {
                ProcessSplitRegionInteractiveSelectEgoLane(
                    relative_id_lanes, order_ids_of_same_zero_relative_id,
                    lane_change_cmd);
                if (is_exist_interactive_select_split_) {
                  interactive_select_split_counter_ = 3;
                  return;
                } else {
                  interactive_select_split_counter_--;
                  interactive_select_split_counter_ =
                      std::min(interactive_select_split_counter_, 0);
                }
              }
              return;
            }
          }
        }

        if (ego_in_split_region_ &&
            sum_distance_from_ego_to_both_center_lines_ <
                kEnableSplitSelectionEgoLateralDistanceToBothLaneLines &&
            ego_to_last_split_end_point_distance > 20.0 && lane_keep_status) {
          ProcessIntersectionSplit(relative_id_lanes,
                                   order_ids_of_same_zero_relative_id);
          ILOG_DEBUG << "EgoLaneTrackManger::is_exist_split_on_intersection:"
                     << is_exist_split_on_intersection_;

          if (is_exist_split_on_intersection_) {
            if ((lane_change_cmd != 0 &&
                 lane_change_status != kLaneChangeComplete) &&
                ego_in_split_region_ &&
                sum_distance_from_ego_to_both_center_lines_ < 0.5) {
              ProcessSplitRegionInteractiveSelectEgoLane(
                  relative_id_lanes, order_ids_of_same_zero_relative_id,
                  lane_change_cmd);
              if (is_exist_interactive_select_split_) {
                interactive_select_split_counter_ = 3;
                return;
              } else {
                interactive_select_split_counter_--;
                interactive_select_split_counter_ =
                    std::min(interactive_select_split_counter_, 0);
              }
            }
            return;
          }
        }

        // for sd_map
        // if (is_ego_on_expressway_ && zero_relative_id_nums >= 2
        //     &&!is_in_lane_borrow_status
        //     && ego_in_split_region_) {
        //   bool is_on_road_select_ramp = CheckIfInRoadSelectRamp(
        //       relative_id_lanes, order_ids_of_same_zero_relative_id);
        //   is_on_road_select_ramp_situation_ = is_on_road_select_ramp;
        //   LOG_DEBUG(
        //       "EgoLaneTrackManger::is_on_road_select_ramp_situation: %d \n",
        //       is_on_road_select_ramp_situation_);

        //   if (is_on_road_select_ramp_situation_ &&
        //       distance_to_first_road_split_ < dis_to_split_threshold &&
        //       !is_leaving_ramp_ && lane_keep_status) {
        //     // hack::针对分流 感知未提供分汇流点信息 作如下后处理
        //     PreprocessRoadSplit(relative_id_lanes,
        //                         order_ids_of_same_zero_relative_id);
        //     LOG_DEBUG("EgoLaneTrackManger::is_exist_ramp_on_road: %d \n",
        //               is_exist_ramp_on_road_);

        //     if (is_exist_ramp_on_road_) {
        //       return;
        //     }
        //   }

        //   bool is_in_ramp_select_split = CheckIfInRampSelectSplit(
        //       relative_id_lanes, order_ids_of_same_zero_relative_id);
        //   is_in_ramp_select_split_situation_ = is_in_ramp_select_split;
        //   LOG_DEBUG(
        //       "EgoLaneTrackManger::is_in_ramp_select_split_situation: %d \n",
        //       is_in_ramp_select_split_situation_);
        //   if (is_in_ramp_select_split_situation_ && lane_keep_status) {
        //     //选择匝道上的分叉
        //     PreprocessRampSplit(relative_id_lanes,
        //                         order_ids_of_same_zero_relative_id);
        //     LOG_DEBUG("EgoLaneTrackManger::is_exist_split_on_ramp: %d \n",
        //               is_exist_split_on_ramp_);

        //     if (is_exist_split_on_ramp_) {
        //       return;
        //     }
        //   }

        //   //处理高架快速路普通分流口
        //   if (lane_keep_status) {
        //     PreprocessOrdinarySplit(relative_id_lanes,
        //                             order_ids_of_same_zero_relative_id,
        //                             virtual_id_mapped_lane);
        //     LOG_DEBUG(
        //         "EgoLaneTrackManger::is_exist_split_on_expressway_: %d \n",
        //         is_exist_split_on_expressway_);
        //     if (is_exist_split_on_expressway_) {
        //       return;
        //     }
        //   }
        // }
      } else if (function_info.function_mode() ==
                 common::DrivingFunctionInfo::SCC) {
        road_split_select_is_finish_ = false;
        ramp_split_select_is_finish_ = false;
        if (zero_relative_id_nums > 1 && lane_keep_status &&
            ego_in_split_region_ &&
            sum_distance_from_ego_to_both_center_lines_ <
                kEnableSplitSelectionEgoLateralDistanceToBothLaneLines) {
          if (enable_use_ground_mark) {
            ProcessSplitWithGroundMark(relative_id_lanes,
                                       order_ids_of_same_zero_relative_id);
            ILOG_DEBUG << "EgoLaneTrackManger::is_exist_split_on_intersection:"
                       << is_exist_split_on_intersection_;

            if (is_exist_split_on_intersection_) {
              if ((lane_change_cmd != 0 &&
                   lane_change_status != kLaneChangeComplete) &&
                  ego_in_split_region_ &&
                  sum_distance_from_ego_to_both_center_lines_ < 0.5) {
                ProcessSplitRegionInteractiveSelectEgoLane(
                    relative_id_lanes, order_ids_of_same_zero_relative_id,
                    lane_change_cmd);
                if (is_exist_interactive_select_split_) {
                  interactive_select_split_counter_ = 3;
                  return;
                } else {
                  interactive_select_split_counter_--;
                  interactive_select_split_counter_ =
                      std::min(interactive_select_split_counter_, 0);
                }
              }
              return;
            }
          }

          ProcessIntersectionSplit(relative_id_lanes,
                                   order_ids_of_same_zero_relative_id);
          ILOG_DEBUG << "EgoLaneTrackManger::is_exist_split_on_intersection:"
                     << is_exist_split_on_intersection_;

          if (is_exist_split_on_intersection_) {
            if ((lane_change_cmd != 0 &&
                 lane_change_status != kLaneChangeComplete) &&
                ego_in_split_region_ &&
                sum_distance_from_ego_to_both_center_lines_ < 0.5) {
              ProcessSplitRegionInteractiveSelectEgoLane(
                  relative_id_lanes, order_ids_of_same_zero_relative_id,
                  lane_change_cmd);
              if (is_exist_interactive_select_split_) {
                interactive_select_split_counter_ = 3;
                // enable_output_split_select_classical_chinese_ = true;
                return;
              } else {
                interactive_select_split_counter_--;
                interactive_select_split_counter_ =
                    std::max(interactive_select_split_counter_, 0);
              }
            }
            interactive_select_split_counter_--;
            interactive_select_split_counter_ =
                std::max(interactive_select_split_counter_, 0);
            return;
          }
        }
      }

      SelectEgoLaneWithPlan(relative_id_lanes,
                            order_ids_of_same_zero_relative_id,
                            virtual_id_mapped_lane);
      interactive_select_split_counter_--;
      interactive_select_split_counter_ =
          std::max(interactive_select_split_counter_, 0);
    } else {
      SelectEgoLaneWithoutPlan(relative_id_lanes);
      is_select_ego_lane_without_plan_ = true;
    }
  }

  ILOG_DEBUG << "EgoLaneTrackManger::virtual_lane_relative_id_switch_flag:"
             << virtual_lane_relative_id_switch_flag_;
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
  const int fix_lane_virtual_id =
      lane_change_decider_output.fix_lane_virtual_id;
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
       (coarse_planning_info.target_state == kLaneChangeHold) ||
       (coarse_planning_info.target_state == kLaneChangeCancel));

  if (is_lc_change && (lc_state != kLaneKeeping)) {
    for (const auto& relative_id_lane : relative_id_lanes) {
      if (relative_id_lane != nullptr) {
        current_relative_id_lane_mapping_cost = ComputeLanesMatchlaterakDisCost(
            target_lane_vitrual_id, relative_id_lane, relative_id_lanes,
            virtual_id_mapped_lane,
            target_lane_vitrual_id == fix_lane_virtual_id);
        if (current_relative_id_lane_mapping_cost <
            target_lane_maping_diff_total) {
          target_lane_maping_diff_total = current_relative_id_lane_mapping_cost;
          target_lane_order_id = relative_id_lane->get_order_id();
        }
      } else {
        continue;
      }
    }
    ILOG_DEBUG << "target_lane_maping_diff_total:"
               << target_lane_maping_diff_total;

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
            virtual_id_mapped_lane,
            origin_lane_virtual_id == fix_lane_virtual_id);
        if (current_relative_id_lane_mapping_cost <
            origin_lane_maping_diff_total) {
          origin_lane_maping_diff_total = current_relative_id_lane_mapping_cost;
          origin_lane_order_id = relative_id_lane->get_order_id();
        }
      } else {
        continue;
      }
    }
    ILOG_DEBUG << "origin_lane_maping_diff_total:"
               << origin_lane_maping_diff_total;

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
  first_split_dir_dis_info_.first = NO_SPLIT;
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
  acc_predict_right_change_count_ = 0;
  acc_predict_left_change_count_ = 0;
}

void EgoLaneTrackManger::Update(const RouteInfoOutput& route_info_output) {
  is_ego_on_expressway_ = route_info_output.is_ego_on_expressway;
  is_on_ramp_ = route_info_output.is_on_ramp;
  dis_to_ramp_ = route_info_output.dis_to_ramp;
  is_leaving_ramp_ = false;
  const auto first_split_dir = route_info_output.first_split_direction;
  first_split_dir_dis_info_.first =
      first_split_dir == RAMP_ON_LEFT
          ? ON_LEFT
          : (first_split_dir == RAMP_ON_RIGHT ? ON_RIGHT : NO_SPLIT);
  first_split_dir_dis_info_.second =
      route_info_output.distance_to_first_road_split;
  distance_to_first_road_merge_ =
      route_info_output.distance_to_first_road_merge;
  distance_to_first_road_split_ =
      route_info_output.distance_to_first_road_split;
  current_segment_passed_distance_ =
      route_info_output.current_segment_passed_distance;
  sum_dis_to_last_split_point_ =
      route_info_output.sum_dis_to_last_link_split_point;
  split_direction_dis_info_list_.clear();

  for (int i = 0; i < route_info_output.map_split_region_info_list.size();
       i++) {
    const auto& split_region_info =
        route_info_output.map_split_region_info_list[i];
    split_direction_dis_info_list_.emplace_back(
        static_cast<SplitRelativeDirection>(split_region_info.split_direction),
        split_region_info.distance_to_split_point);
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
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  bool is_manual_lane_change = false;  // 是否发生了手动接管导致的变道

  const auto& function_info = session_->environmental_model().function_info();

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

  if (function_info.function_mode() == common::DrivingFunctionInfo::ACC) {
    // 1. 找到选中的base lane
    std::shared_ptr<VirtualLane> original_base_lane = nullptr;
    double original_base_lane_ego_l = 0.0;
    double original_base_lane_ego_s = 0.0;
    for (auto& lane : relative_id_lanes) {
      if (!lane) {
        continue;
      }
      if (lane->get_order_id() == origin_order_id) {
        original_base_lane = lane;
        if (lane_cost_list.count(origin_order_id)) {
          original_base_lane_ego_l =
              lane_cost_list[origin_order_id][1];  // ego_l

          // 计算自车在base lane的s
          std::shared_ptr<KDPath> frenet_coord = lane->get_lane_frenet_coord();
          Point2D ego_frenet_point;
          if (frenet_coord &&
              frenet_coord->XYToSL(ego_cart, ego_frenet_point)) {
            original_base_lane_ego_s = ego_frenet_point.x;
          }
        }
        break;
      }
    }

    // 2. 检查条件：base lane存在 + 航向明显指向左/右 + 横向偏移超阈值
    if (original_base_lane != nullptr) {
      bool is_heading_to_left = false;   // 航向指向左车道
      bool is_heading_to_right = false;  // 航向指向右车道
      std::shared_ptr<KDPath> base_lane_frenet =
          original_base_lane->get_lane_frenet_coord();

      if (base_lane_frenet != nullptr &&
          original_base_lane_ego_s < base_lane_frenet->Length()) {
        int left_count = 0;   // 满足“偏左”的采样点数量
        int right_count = 0;  // 满足“偏右”的采样点数量
        int valid_count = 0;  // 有效采样点总数

        for (double s = original_base_lane_ego_s;
             s < base_lane_frenet->Length(); s += kLaneLineSegmentLength) {
          // 超出10m停止遍历
          if (s > 10 + original_base_lane_ego_s) {
            break;
          }

          // 获取采样点的车道航向角（全局）
          double lane_heading = base_lane_frenet->GetPathCurveHeading(s);
          // 计算航向偏差（归一化）
          double theta_err = NormalizeAngle(ego_heading_angle - lane_heading);

          // 分别统计偏左/偏右的采样点
          if (theta_err > kACCHeadingLeftThreshold) {  // 偏左
            left_count++;
          } else if (theta_err < -kACCHeadingRightThreshold) {  // 偏右
            right_count++;
          }
          valid_count++;
        }

        // 综合判断：有效采样点>0 且 偏左/偏右点占比≥阈值
        if (valid_count > 0) {
          double left_ratio = static_cast<double>(left_count) / valid_count;
          double right_ratio = static_cast<double>(right_count) / valid_count;

          is_heading_to_left = (left_ratio >= kLeftRightRatioThreshold);
          is_heading_to_right = (right_ratio >= kLeftRightRatioThreshold);
        }
      }

      // 横向偏移判断（左/右）
      bool is_left_offset_exceed =
          (original_base_lane_ego_l > kACCLeftOffsetThreshold);  // 左偏移
      bool is_right_offset_exceed =
          (original_base_lane_ego_l < -kACCRightOffsetThreshold);  // 右偏移

      if (is_heading_to_left && is_left_offset_exceed) {
        // 更新ACC预测向左/右的计数
        acc_predict_left_change_count_++;
        acc_predict_right_change_count_ = 0;
        if (acc_predict_left_change_count_ >= 3) {
          std::shared_ptr<VirtualLane> left_lane = nullptr;
          for (auto& lane : relative_id_lanes) {
            if (lane->get_order_id() == origin_order_id - 1) {
              left_lane = lane;
              break;
            }
          }
          if (left_lane != nullptr) {
            origin_order_id = left_lane->get_order_id();
          }
        }
      } else if (is_heading_to_right && is_right_offset_exceed) {
        // 更新ACC预测向左/右的计数
        acc_predict_right_change_count_++;
        acc_predict_left_change_count_ = 0;
        if (acc_predict_right_change_count_ >= 3) {
          std::shared_ptr<VirtualLane> right_lane = nullptr;
          for (auto& lane : relative_id_lanes) {
            if (lane->get_order_id() == origin_order_id + 1) {
              right_lane = lane;
              break;
            }
          }
          if (right_lane != nullptr) {
            origin_order_id = right_lane->get_order_id();
          }
        }
      } else {
        // 更新ACC预测向左/右的计数
        acc_predict_right_change_count_ = 0;
        acc_predict_left_change_count_ = 0;
      }
    }
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
    if (lane_relative_id == 0) {
      current_zero_relative_id_lane_ = lane;
    }
  }
  if (!active) {
    MakesureManualLaneChangeByLaneOffset(last_zero_relative_id_lane_,
                                         current_zero_relative_id_lane_,
                                         is_manual_lane_change);
  }
  last_zero_relative_id_lane_ = current_zero_relative_id_lane_;

  return;
}

void EgoLaneTrackManger::MakesureManualLaneChangeByLaneOffset(
    std::shared_ptr<VirtualLane>& last_zero_relative_id_lane_,
    const std::shared_ptr<VirtualLane>& current_zero_relative_id_lane_,
    bool& is_manual_lane_change) {
  is_manual_lane_change = false;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  Point2D ego_point = {ego_state->planning_init_point().x,
                       ego_state->planning_init_point().y};

  if (!last_zero_relative_id_lane_ ||
      !last_zero_relative_id_lane_->get_lane_frenet_coord() ||
      !current_zero_relative_id_lane_ ||
      current_zero_relative_id_lane_->lane_points().empty()) {
    return;
  }

  double ego_s_base = 0.0, ego_l_base = 0.0;
  last_zero_relative_id_lane_->get_lane_frenet_coord()->XYToSL(
      ego_point.x, ego_point.y, &ego_s_base, &ego_l_base);

  double sum_l = 0.0;
  int lane_pt_count = 0;
  for (const auto& pt : current_zero_relative_id_lane_->lane_points()) {
    if (std::isnan(pt.local_point.x) || std::isnan(pt.local_point.y)) {
      continue;
    }

    double pt_s = 0.0, pt_l = 0.0;
    const auto& last_zero_relative_id_lane_frenet_crd =
        last_zero_relative_id_lane_->get_lane_frenet_coord();

    if (!last_zero_relative_id_lane_frenet_crd->XYToSL(
            pt.local_point.x, pt.local_point.y, &pt_s, &pt_l)) {
      continue;
    } else {
      if (pt_s > last_zero_relative_id_lane_frenet_crd->Length()) {
        continue;
      }
      if (pt_s > ego_s_base && pt_s < ego_s_base + kConsiderManualLength) {
        sum_l += pt_l;
        ++lane_pt_count;
      }
    }
  }
  if (lane_pt_count == 0) {
    return;
  }
  double average_l = sum_l / lane_pt_count;
  if (average_l > kManualLaneChangeDisThd) {
    current_lane_virtual_id_ -= 1;
    is_manual_lane_change = true;
  } else if (average_l < -kManualLaneChangeDisThd) {
    current_lane_virtual_id_ += 1;
    is_manual_lane_change = true;
  } else {
    is_manual_lane_change = false;
  }
}

void EgoLaneTrackManger::SelectEgoLaneWithPlan(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane) {
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  const bool is_merge_region =
      ego_lane_road_right_decider_output.is_merge_region;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool enable_using_st_plan = session_->planning_context()
                                  .spatio_temporal_union_plan_output()
                                  .enable_using_st_plan;
  int origin_lane_virtual_id =
      lane_change_decider_output.origin_lane_virtual_id;
  int origin_lane_order_id = -1;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const bool is_in_lane_borrow_status = session_->planning_context()
                                            .lane_borrow_decider_output()
                                            .is_in_lane_borrow_status;
  const bool is_change_target_lane = session_->planning_context()
                                         .lane_borrow_decider_output()
                                         .is_change_target_lane;
  int origin_order_id = 0;
  int current_order_id = 0;
  const double default_lane_mapping_cost = 10.0;
  const int fix_lane_virtual_id =
      lane_change_decider_output.fix_lane_virtual_id;
  const int lc_state = lane_change_decider_output.curr_state;
  std::unordered_map<int32_t, std::vector<double>> lane_cost_list;
  std::shared_ptr<VirtualLane> last_track_virtual_id_lane;
  int last_track_virtual_id = 0;
  double last_ego_lane_curv = 0.0;
  const auto& plannig_init_point = ego_state->planning_init_point();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};

  if (!virtual_id_mapped_lane.empty()) {
    for (const auto& virtual_id_lane : virtual_id_mapped_lane) {
      if (virtual_id_lane.second->get_virtual_id() == fix_lane_virtual_id) {
        last_track_virtual_id_lane = virtual_id_lane.second;
        last_track_virtual_id = virtual_id_lane.first;
      }
      if (virtual_id_lane.second->get_virtual_id() == origin_lane_virtual_id) {
        origin_lane_order_id = virtual_id_lane.second->get_order_id();
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
       (coarse_planning_info.target_state == kLaneChangeHold) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  bool is_lc_back = coarse_planning_info.target_state == kLaneChangeCancel;
  bool is_lane_change = (is_lc_change || is_lc_back);
  double k_init_pos_cost_weight =
      (is_lane_change || ego_in_split_region_)
          ? kLaneChangeExecutionWeightRatio * kInitPosCostWeight
          : kInitPosCostWeight * 2.0;
  double lateral_distance_cost_weight =
      (enable_using_st_plan && !ego_in_split_region_)
          ? 0.01
          : kCumuLateralDistanceCostWeight;
  double k_lane_change_order_id_diff_wegiht =
      (is_lc_change && origin_lane_order_id != -1 && ego_in_split_region_)
          ? kLaneChangeOrderidDiffWeight
          : 0.0;
  int k_lane_change_order_id_diff = 0;
  if (is_lc_change) {
    if (lc_request_direction == LEFT_CHANGE) {
      k_lane_change_order_id_diff = 0;
    } else if (lc_request_direction == RIGHT_CHANGE) {
      k_lane_change_order_id_diff = kDefaultLaneChangeOrderIdDiff;
    }
  }

  if ((lc_state == kLaneKeeping || lc_state == kLaneChangePropose) &&
      !ego_in_split_region_ && !enable_using_st_plan) {
    lateral_distance_cost_weight = 0.28;
  }

  if (is_in_lane_borrow_status || ego_in_split_region_) {
    k_init_pos_cost_weight = 0.0;
  }
  if (is_change_target_lane) {
    k_init_pos_cost_weight = 8.0;
    lateral_distance_cost_weight = 0.01;
  }

  double clane_min_diff_total = std::numeric_limits<double>::max();
  for (auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane == nullptr) {
      continue;
    }

    if (relative_id_lane->get_lane_type() == iflyauto::LANETYPE_OPPOSITE) {
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
    double order_id_diff_cost = 0.0;

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
        double begin_ego_s = 50.0;
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
          if (is_merge_region) {
            begin_ego_s = ego_cart_target_frenet.x - 20.0;
            target_ego_l = ego_cart_target_frenet.y;
          } else {
            begin_ego_s = ego_cart_target_frenet.x;
            target_ego_l = ego_cart_target_frenet.y;
          }
        }
        if (road_radius > kDefaultRoadRadius) {
          int select_lane_point_interval = 1;
          for (int i = 0; i < lane_points.size();
               i += select_lane_point_interval) {
            iflyauto::ReferencePoint point = lane_points[i];
            if (std::isnan(point.local_point.x) ||
                std::isnan(point.local_point.y)) {
              ILOG_ERROR << "update_lane_points: skip NaN point";
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
            if (s < begin_ego_s) {
              continue;
            }
            total_lateral_offset += lateral_offset;
            point_nums += 1;
            if (point_nums >= kDefaultPointNums ||
                s > ego_cart_target_frenet.x +
                        kDefaultMappingConsiderLaneLength) {
              break;
            }
          }
        } else {
          for (int i = 0; i < lane_points.size(); i++) {
            iflyauto::ReferencePoint point = lane_points[i];
            if (std::isnan(point.local_point.x) ||
                std::isnan(point.local_point.y)) {
              ILOG_ERROR << "update_lane_points: skip NaN point";
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
            if (s < begin_ego_s) {
              continue;
            }
            if (point_nums >= kDefaultPointNums ||
                s > ego_cart_target_frenet.x + kConsiderLaneLineLength) {
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

    int lane_order_id = relative_id_lane->get_order_id();
    order_id_diff_cost =
        std::fabs(std::fabs(lane_order_id - origin_lane_order_id) -
                  k_lane_change_order_id_diff);
    total_cost = lateral_distance_cost_weight * cumu_lat_dis_cost +
                 kCrossLaneCostWeight * crosslane_cost +
                 k_init_pos_cost_weight * init_pose_cost +
                 k_lane_change_order_id_diff_wegiht * order_id_diff_cost;
    std::vector<double> cost_list{cumu_lat_dis_cost, crosslane_cost,
                                  init_pose_cost, order_id_diff_cost,
                                  total_cost};
    lane_cost_list[lane_order_id] = cost_list;

    if (total_cost < clane_min_diff_total) {
      clane_min_diff_total = total_cost;
      current_order_id = relative_id_lane->get_order_id();
      last_track_ego_lane_ = relative_id_lane;
      relative_id_lane->set_relative_id(0);
      current_fix_lane_order_id_ = relative_id_lane->get_order_id();
      ;
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
      ILOG_ERROR << "update_center_line_points: skip NaN point";
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

  if (boundary_points.size() <=
      planning_math::KDPath::kKDPathMinPathPointSize) {
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
  double surpress_select_lane_dis_to_split = ego_state->ego_v() * 2.5;
  double surpress_lane_change_dis_to_detect_split =
      std::max(10.0, ego_state->ego_v() * 1.25);
  double ego_distance_to_lane_merge_split_point = 0.0;
  double dis_to_lane_split_point_threshold =
      std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      if (relative_id_lane == nullptr) {
        continue;
      }
      if (relative_id_lane->get_lane_merge_split_point()
              .merge_split_point_data_size > 0) {
        const auto& lane_merge_split_point =
            relative_id_lane->get_lane_merge_split_point();
        // lane_merge_split_point.x =
        // lane_merge_split_point.merge_split_point_data[0].point.x;
        // lane_merge_split_point.y =
        // lane_merge_split_point.merge_split_point_data[0].point.y;

        if (!lane_merge_split_point.merge_split_point_data[0].is_split) {
          continue;
        }
        if (lane_merge_split_point.merge_split_point_data[0].distance <
            dis_to_lane_split_point_threshold) {
          dis_to_lane_split_point_threshold =
              lane_merge_split_point.merge_split_point_data[0].distance;
          ego_distance_to_lane_merge_split_point =
              lane_merge_split_point.merge_split_point_data[0].distance;
        }
      }
    }
  }

  if ((last_zero_relative_id_nums_ > 1 && road_split_select_is_finish_ &&
       ego_distance_to_lane_merge_split_point <
           surpress_select_lane_dis_to_split) ||
      ego_distance_to_lane_merge_split_point <
          surpress_lane_change_dis_to_detect_split) {
    ILOG_DEBUG << "PreprocessRoadSplit::last_zero_relative_id_nums_ > 1";
    if (last_zero_relative_id_order_id_index_ != -1) {
      ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                        order_ids,
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

  // Find current frame lane closest to previous frame's ego lane
  int closest_lane_index = -1;
  ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                    order_ids, closest_lane_index);

  // Use closest lane if found, otherwise fallback to last_track_ego_lane_
  std::shared_ptr<VirtualLane> collision_check_lane = last_track_ego_lane_;
  if (closest_lane_index >= 0 && closest_lane_index < order_ids.size() &&
      order_ids[closest_lane_index] < relative_id_lanes.size()) {
    collision_check_lane = relative_id_lanes[order_ids[closest_lane_index]];
  }

  TrajectoryPoints ego_future_trajs;
  if (first_split_dir_dis_info_.first == ON_RIGHT) {
    GenerateEgoFutureTrajectory(relative_id_lanes[order_ids[1]], 0.0, ego_future_trajs);
    if (!IsPathCollisionWithRoadEdge(collision_check_lane, relative_id_lanes[order_ids[1]], ego_future_trajs)) {
      is_exist_ramp_on_road_ = true;
      relative_id_lanes[order_ids[1]]->set_relative_id(0);
      origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
      last_zero_relative_id_order_id_index_ = 1;
      last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
    }
  } else if (first_split_dir_dis_info_.first == ON_LEFT) {
    GenerateEgoFutureTrajectory(relative_id_lanes[order_ids[0]], 0.0, ego_future_trajs);
    if (!IsPathCollisionWithRoadEdge(collision_check_lane, relative_id_lanes[order_ids[0]], ego_future_trajs)) {
      is_exist_ramp_on_road_ = true;
      relative_id_lanes[order_ids[0]]->set_relative_id(0);
      origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
      last_zero_relative_id_order_id_index_ = 0;
      last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
    }
  } else {
    is_exist_ramp_on_road_ = false;
    return;
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  road_split_select_is_finish_ = true;
  return;
}
static bool IsDiversionLane(const iflymapdata::sdpro::Lane* lane_info) {
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
static bool IsEmergencyLane(const iflymapdata::sdpro::Lane* lane_info) {
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
void EgoLaneTrackManger::CheckIfConsiderSecondSplit(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    bool& is_consider_second_split) {
  is_consider_second_split = false;

  // step1: Check if there are consecutive splits in different directions
  bool is_cons_diff_dir_split = false;
  if (split_direction_dis_info_list_.size() > 1) {
    auto const& first_split = split_direction_dis_info_list_[0];
    auto const& second_split = split_direction_dis_info_list_[1];

    if (second_split.second < 120.0 &&
        SplitRelativeDirection::NO_SPLIT != first_split.first &&
        SplitRelativeDirection::NO_SPLIT != second_split.first &&
        first_split.first != second_split.first) {
      is_cons_diff_dir_split = true;
    }
  }

  if (!is_cons_diff_dir_split || nullptr == current_link_) {
    is_consider_second_split = false;
    return;
  }

  auto const& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto& ld_map =
      session_->environmental_model().get_route_info()->get_sdpro_map();

  auto const& split_region_info_list =
      route_info_output.map_split_region_info_list;
  double const ego_on_cur_link_s = current_segment_passed_distance_;

  auto* first_split_link = ld_map.GetLinkOnRoute(
      split_region_info_list.front().end_fp_point.link_id);

  if (first_split_link == nullptr) {
    is_consider_second_split = false;
    return;
  }

  if (first_split_link->id() == current_link_->id()) {
    is_consider_second_split = true;
    return;
  }

  // All lanes that can directly lead to the split link
  std::vector<uint64_t> lanes_to_split_link =
      split_region_info_list.front().end_fp_point.lane_ids;
  uint64_t link_id = first_split_link->id();

  /**
   * step2:
   * Trace backward through lane topology to determine if the current lane can
   * directly lead to the target link of the first split.
   *
   * All predecessor lanes in the tracing process must be able to lead to the
   * split target link
   */

  size_t loop_num = 0;
  do {
    // Prevent falling into an infinite loop
    loop_num++;

    // 1. 记录通过第一个分流的车道，反向推导出来的所有Lane
    // 2. 如果不在导航路线上的Lane需要被舍弃
    auto* pre_link = ld_map.GetPreviousLinkOnRoute(link_id);
    if (pre_link == nullptr) {
      break;
    }

    link_id = pre_link->id();
    auto const& all_pre_lanes = pre_link->lane_ids();

    std::set<uint64_t> tmp_lanes_on_route;

    for (uint64 lane_id : lanes_to_split_link) {
      auto* lane_ptr = ld_map.GetLaneInfoByID(lane_id);
      if (nullptr == lane_ptr) {
        is_consider_second_split = false;
        return;
      }

      // save preceeding lanes on route
      for (uint64 pre_lane_id : lane_ptr->predecessor_lane_ids()) {
        if (all_pre_lanes.end() != std::find(all_pre_lanes.begin(),
                                             all_pre_lanes.end(),
                                             pre_lane_id)) {
          tmp_lanes_on_route.emplace(pre_lane_id);
        }
      }
    }

    bool all_pre_lane_lead_to_split_link = !tmp_lanes_on_route.empty();
    for (uint64 lane_id : tmp_lanes_on_route) {
      auto* lane_ptr = ld_map.GetLaneInfoByID(lane_id);

      if (nullptr == lane_ptr) {
        is_consider_second_split = false;
        return;
      }

      for (uint64 suc_lane_id : lane_ptr->successor_lane_ids()) {
        if (lanes_to_split_link.end() == std::find(lanes_to_split_link.begin(),
                                                   lanes_to_split_link.end(),
                                                   suc_lane_id)) {
          all_pre_lane_lead_to_split_link = false;
          break;
        }
      }
    }

    if (!all_pre_lane_lead_to_split_link) {
      lanes_to_split_link.clear();
      break;
    }
    lanes_to_split_link.clear();
    lanes_to_split_link.assign(tmp_lanes_on_route.begin(),
                               tmp_lanes_on_route.end());

  } while (!lanes_to_split_link.empty() && link_id != current_link_->id() &&
           loop_num < 100);

  if (lanes_to_split_link.empty() || loop_num >= 100) {
    is_consider_second_split = false;
    return;
  }

  /**
   * step3:
   * If the lane the ego vehicle is in can directly lead to the target road of
   * the first split, then the direction of the second split needs to be
   * considered.
   *
   * lanes saved in lanes_to_split_link
   */
  if (link_id != current_link_->id()) {
    is_consider_second_split = false;
    return;
  }

  // Get lane sequence from left to right
  std::set<uint32> lanes_seq_to_split_link;

  // Need filter some invalid lanes
  bool cur_link_is_exist_emergency_lane = false;
  std::vector<uint32> invalid_seq;
  for (uint64 const lane_id : current_link_->lane_ids()) {
    auto const* lane_ptr = ld_map.GetLaneInfoByID(lane_id);
    if (IsDiversionLane(lane_ptr)) {
      invalid_seq.emplace_back(lane_ptr->sequence());
    }
    if (IsEmergencyLane(lane_ptr)) {
      cur_link_is_exist_emergency_lane = true;
    }
  }

  int const total_lane_nums =
      current_link_->lane_ids_size() - invalid_seq.size();

  for (uint64 const lane_id : lanes_to_split_link) {
    auto const* lane_ptr = ld_map.GetLaneInfoByID(lane_id);

    size_t const invalid_num = std::count_if(
        invalid_seq.begin(), invalid_seq.end(),
        [&](int sequence_id) { return lane_ptr->sequence() > sequence_id; });

    lanes_seq_to_split_link.emplace(1 + total_lane_nums -
                                    (lane_ptr->sequence() - invalid_num));
  }

  // Calculate ego sequence
  auto find_relative_lanes = std::find_if(
      relative_id_lanes.begin(), relative_id_lanes.end(),
      [](std::shared_ptr<planning::VirtualLane> const virtual_lane) {
        return nullptr != virtual_lane && 0 == virtual_lane->get_relative_id();
      });

  if (relative_id_lanes.end() == find_relative_lanes) {
    is_consider_second_split = false;
    return;
  }

  std::vector<iflyauto::LaneNumMsg> const& per_lane_nums =
      (*find_relative_lanes)->get_lane_nums();

  int left_lane_num = 0;
  int right_lane_num = 0;
  for (const auto& lane_num : per_lane_nums) {
    if (lane_num.end > 1e-4) {
      left_lane_num = lane_num.left_lane_num;
      right_lane_num = lane_num.right_lane_num;
      break;
    }
  }

  int ego_seq = left_lane_num + 1;

  // 如果自车左边的车道数大于等于link的总车道数，则认为左侧车道数误检，直接return;
  bool is_per_left_lane_error = left_lane_num >= total_lane_nums;

  if (is_per_left_lane_error) {
    // 当左侧的观测数量大于总车道数时，若右侧车道数量小于一定值时，此时认为我们在地图的最右侧车道。
    if (cur_link_is_exist_emergency_lane && right_lane_num <= 1) {
      // 假设自车不会在应急车道上行驶
      ego_seq = total_lane_nums - 1;
    } else if (!cur_link_is_exist_emergency_lane && right_lane_num == 0) {
      ego_seq = total_lane_nums;
    }
  }

  // 如果ego_seq在可以通往第一个分流车道的seq，那么开始考虑第二个分流
  is_consider_second_split = lanes_seq_to_split_link.count(ego_seq) > 0;
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
  double ego_distance_to_lane_merge_split_point = 0.0;
  double surpress_select_lane_dis_to_split = ego_state->ego_v() * 2.5;
  double surpress_lane_change_dis_to_detect_split =
      std::max(10.0, ego_state->ego_v() * 1.25);
  double dis_to_lane_split_point_threshold =
      std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      if (relative_id_lane == nullptr) {
        continue;
      }
      if (relative_id_lane->get_lane_merge_split_point()
              .merge_split_point_data_size > 0) {
        const auto& lane_merge_split_point =
            relative_id_lane->get_lane_merge_split_point();
        // lane_merge_split_point.x =
        // lane_merge_split_point.merge_split_point_data[0].point.x;
        // lane_merge_split_point.y =
        // lane_merge_split_point.merge_split_point_data[0].point.y;

        if (!lane_merge_split_point.merge_split_point_data[0].is_split) {
          continue;
        }
        if (lane_merge_split_point.merge_split_point_data[0].distance <
            dis_to_lane_split_point_threshold) {
          dis_to_lane_split_point_threshold =
              lane_merge_split_point.merge_split_point_data[0].distance;
          ego_distance_to_lane_merge_split_point =
              lane_merge_split_point.merge_split_point_data[0].distance;
        }
      }
    }
  }

  // 尝试这里识别第一个分流已经不用考虑，可以考虑第二个分流场景了。
  // 仅针对连续分流、且方向相反的场景进行判断。

  // Check whether to consider the second split. That is: the current lane the
  // ego vehicle is in will definitely not miss the first split.
  bool need_consider_second_split = false;
  CheckIfConsiderSecondSplit(relative_id_lanes, need_consider_second_split);

  if ((!need_consider_second_split && last_zero_relative_id_nums_ > 1 &&
       (ramp_split_select_is_finish_ || road_split_select_is_finish_) &&
       ego_distance_to_lane_merge_split_point <
           surpress_select_lane_dis_to_split) ||
      ego_distance_to_lane_merge_split_point <
          surpress_lane_change_dis_to_detect_split) {
    ILOG_DEBUG << "PreprocessRampSplit::last_zero_relative_id_nums_ > 1";
    if (last_zero_relative_id_order_id_index_ != -1) {
      ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                        order_ids,
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

  // if (split_direction_dis_info_list_.size() > 1) {
  //   if (distance_to_first_road_merge_ >
  //       split_direction_dis_info_list_[1].second) {
  //     is_exist_split_on_ramp_ = true;
  //     if (split_direction_dis_info_list_[1].first == ON_RIGHT) {
  //       relative_id_lanes[order_ids[1]]->set_relative_id(0);
  //       origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
  //       last_zero_relative_id_order_id_index_ = 1;
  //       last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
  //     } else if (split_direction_dis_info_list_[1].first == ON_LEFT) {
  //       relative_id_lanes[order_ids[0]]->set_relative_id(0);
  //       origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
  //       last_zero_relative_id_order_id_index_ = 0;
  //       last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
  //     } else {
  //       is_exist_split_on_ramp_ = false;
  //       return;
  //     }
  //   } else {
  //     is_exist_split_on_ramp_ = false;
  //     return;
  //   }
  // } else {

  // 这里识别需要判断的split，此处直接用first不太合理。
  double road_merge_and_split_distance_thld_surpress_lane_select =
      std::max(60.0, ego_state->ego_v() * 4.0);

  // Find current frame lane closest to previous frame's ego lane
  int closest_lane_index = -1;
  ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                    order_ids, closest_lane_index);

  // Use closest lane if found, otherwise fallback to last_track_ego_lane_
  std::shared_ptr<VirtualLane> collision_check_lane = last_track_ego_lane_;
  if (closest_lane_index >= 0 && closest_lane_index < order_ids.size() &&
      order_ids[closest_lane_index] < relative_id_lanes.size()) {
    collision_check_lane = relative_id_lanes[order_ids[closest_lane_index]];
  }

  TrajectoryPoints ego_future_trajs;
  if (distance_to_first_road_merge_ > distance_to_first_road_split_ ||
      (std::fabs(distance_to_first_road_merge_ - distance_to_first_road_split_) < road_merge_and_split_distance_thld_surpress_lane_select)) {
    is_exist_split_on_ramp_ = false;
    std::pair<SplitRelativeDirection, double> const& split_dir_dis =
        need_consider_second_split ? split_direction_dis_info_list_[1]
                                   : first_split_dir_dis_info_;

    if (split_dir_dis.first == ON_RIGHT) {
      GenerateEgoFutureTrajectory(relative_id_lanes[order_ids[1]], 0.0, ego_future_trajs);
      if (!IsPathCollisionWithRoadEdge(collision_check_lane, relative_id_lanes[order_ids[1]], ego_future_trajs)) {
        relative_id_lanes[order_ids[1]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[1]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 1;
        last_track_ego_lane_ = relative_id_lanes[order_ids[1]];
        is_exist_split_on_ramp_ = true;
      }
    } else if (split_dir_dis.first == ON_LEFT) {
      GenerateEgoFutureTrajectory(relative_id_lanes[order_ids[0]], 0.0, ego_future_trajs);
      if (!IsPathCollisionWithRoadEdge(collision_check_lane, relative_id_lanes[order_ids[0]], ego_future_trajs)) {
        relative_id_lanes[order_ids[0]]->set_relative_id(0);
        origin_order_id = relative_id_lanes[order_ids[0]]->get_order_id();
        last_zero_relative_id_order_id_index_ = 0;
        last_track_ego_lane_ = relative_id_lanes[order_ids[0]];
        is_exist_split_on_ramp_ = true;
      }
    } else {
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
  // }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - origin_order_id;
    lane->set_relative_id(lane_relative_id);
  }
  ramp_split_select_is_finish_ = true;
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
    ILOG_DEBUG << "PreprocessOrdinarySplit::last_zero_relative_id_nums_ > 1";
    if (last_zero_relative_id_order_id_index_ != -1) {
      ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                        order_ids,
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
void EgoLaneTrackManger::ProcessSplitRegionInteractiveSelectEgoLane(
    std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids, const int lane_change_cmd) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  int origin_order_id = -1;
  int zero_relative_id_order_id_index = -1;
  int current_order_id = 0;
  double ego_distance_to_lane_merge_split_point = 0.0;
  double k_surpresss_interactive_split_selector_distance = 50.0;
  k_surpresss_interactive_split_selector_distance =
      std::max(k_surpresss_interactive_split_selector_distance,
               ego_state->ego_v() * 2.0);

  // Point2D lane_merge_split_point;
  bool find_virtual_lane_split_point = false;
  int split_lane_index = -1;
  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      if (relative_id_lane == nullptr) {
        continue;
      }
      if (relative_id_lane->get_lane_merge_split_point()
              .merge_split_point_data_size > 0) {
        split_lane_index = i;
        find_virtual_lane_split_point = true;
        const auto& lane_merge_split_point =
            relative_id_lane->get_lane_merge_split_point();
        ego_distance_to_lane_merge_split_point =
            lane_merge_split_point.merge_split_point_data[0].distance;
        // lane_merge_split_point.x =
        // lane_merge_split_point.merge_split_point_data[0].point.x;
        // lane_merge_split_point.y =
        // lane_merge_split_point.merge_split_point_data[0].point.y;

        if (!lane_merge_split_point.merge_split_point_data[0].is_split ||
            (lane_merge_split_point.merge_split_point_data[0].is_split &&
             ego_distance_to_lane_merge_split_point >
                 k_surpresss_interactive_split_selector_distance)) {
          return;
        }
      }
    }
  }

  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> base_lane = relative_id_lanes[order_ids[i]];
      if (base_lane == nullptr) {
        continue;
      }
      if (base_lane->get_relative_id() == 0) {
        origin_order_id = base_lane->get_order_id();
        zero_relative_id_order_id_index = i;
        break;
      }
    }
  }
  if (origin_order_id != -1 && zero_relative_id_order_id_index != -1) {
    if (lane_change_cmd == 1) {
      current_order_id = origin_order_id - 1;
      auto iter =
          std::find(order_ids.begin(), order_ids.end(), current_order_id);
      if (iter != order_ids.end()) {
        is_exist_interactive_select_split_ = true;
        other_split_lane_left_side_ = true;
        if (ego_distance_to_lane_merge_split_point < 20.0) {
          enable_output_split_select_classical_chinese_ = true;
        }
      } else {
        return;
      }
    } else if (lane_change_cmd == 2) {
      current_order_id = origin_order_id + 1;
      auto iter =
          std::find(order_ids.begin(), order_ids.end(), current_order_id);
      if (iter != order_ids.end()) {
        is_exist_interactive_select_split_ = true;
        other_split_lane_right_side_ = true;
        if (ego_distance_to_lane_merge_split_point < 20.0) {
          enable_output_split_select_classical_chinese_ = true;
        }
      } else {
        return;
      }
    } else {
      return;
    }
  } else {
    return;
  }

  for (auto& lane : relative_id_lanes) {
    int lane_order_id = lane->get_order_id();
    int lane_relative_id = lane_order_id - current_order_id;
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
  double cos_theta = std::cos(ego_state->heading_angle());
  double sin_theta = std::sin(ego_state->heading_angle());
  double x0 = ego_state->ego_pose().x;
  double y0 = ego_state->ego_pose().y;
  int zero_relative_id_order_id_index = last_zero_relative_id_order_id_index_;
  bool enable_using_last_frame_track_ego_lane = true;
  bool find_last_frame_track_ego_lane = true;
  bool enable_use_virtual_lane_process_split = false;
  double ego_distance_to_lane_merge_split_point = 0.0;

  // Point2D lane_merge_split_point;
  Point2D virtual_lane_split_local_point;
  Point2D virtual_lane_split_point;
  bool find_virtual_lane_split_point = false;
  std::vector<int> other_split_lane_index_set;
  int other_split_lane_index = -1;
  int split_lane_index = -1;
  double road_radius_origin = 10000;
  double max_road_radius = 100.0;
  double min_road_radius = 10000.0;
  std::vector<std::pair<int, double>> lane_curv_info_set;
  std::unordered_map<int, LaneCurvInfo> lanes_curv_info;
  for (size_t i = 0; i < order_ids.size(); i++) {
    // double road_radius = 10000;
    LaneCurvInfo lane_curv_info;
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      if (relative_id_lane == nullptr) {
        continue;
      }
      const auto& lane_frenet_coord = relative_id_lane->get_lane_frenet_coord();
      if (lane_frenet_coord == nullptr) {
        road_radius_origin = 10000.0;
      } else {
        CalculateLaneCurvature(lane_frenet_coord, lane_curv_info_set,
                               lane_curv_info, max_road_radius, min_road_radius,
                               relative_id_lane);
      }
      lanes_curv_info.insert(std::make_pair(order_ids[i], lane_curv_info));

      if (relative_id_lane->get_lane_merge_split_point()
              .merge_split_point_data_size > 0) {
        split_lane_index = i;
        const auto& lane_merge_split_point =
            relative_id_lane->get_lane_merge_split_point();
        ego_distance_to_lane_merge_split_point =
            lane_merge_split_point.merge_split_point_data[0].distance;
        // lane_merge_split_point.x =
        // lane_merge_split_point.merge_split_point_data[0].point.x;
        // lane_merge_split_point.y =
        // lane_merge_split_point.merge_split_point_data[0].point.y;

        if (!lane_merge_split_point.merge_split_point_data[0].is_split ||
            (lane_merge_split_point.merge_split_point_data[0].is_split &&
             ego_distance_to_lane_merge_split_point <
                 kDefaultSurpressSplitSelectorDistance)) {
          return;
        }
        find_virtual_lane_split_point = true;
        virtual_lane_split_local_point.x =
            relative_id_lane->get_lane_merge_split_point()
                .merge_split_point_data[0]
                .point.x;
        virtual_lane_split_local_point.y =
            relative_id_lane->get_lane_merge_split_point()
                .merge_split_point_data[0]
                .point.y;
        double tmp_x = virtual_lane_split_local_point.x * cos_theta -
                       virtual_lane_split_local_point.y * sin_theta;
        double tmp_y = virtual_lane_split_local_point.x * sin_theta +
                       virtual_lane_split_local_point.y * cos_theta;
        virtual_lane_split_point.x = tmp_x + x0;
        virtual_lane_split_point.y = tmp_y + y0;
      } else {
        other_split_lane_index_set.emplace_back(i);
      }
    }
  }

  double ewma_alpha = 0.3;
  double current_max_road_radius =
      EWMAFilter(max_road_radius, ewma_alpha, raw_max_road_radius_);
  double current_min_road_radius =
      EWMAFilter(min_road_radius, ewma_alpha, raw_min_road_radius_);
  if (current_max_road_radius < road_radius_origin) {
    road_radius_origin = current_max_road_radius;
  }
  if (last_zero_relative_id_nums_ > 1 &&
      (lcc_split_select_is_finish_ || road_split_select_is_finish_ ||
       ramp_split_select_is_finish_) &&
      ego_distance_to_lane_merge_split_point < ego_state->ego_v() * 1.5) {
    ILOG_DEBUG << "ProcessIntersectionSplit::last_zero_relative_id_nums_ > 1";
    if (last_zero_relative_id_order_id_index_ != -1) {
      ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                        order_ids,
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
          if (ego_distance_to_lane_merge_split_point < 20.0) {
            enable_output_split_select_classical_chinese_ = true;
          }
          return;
        } else {
          return;
        }
      }
    }
  }

  if (!find_virtual_lane_split_point) {
    return;
  }

  double split_point_lateral_max_distance = 0.0;
  if (other_split_lane_index_set.size() == 1) {
    other_split_lane_index = other_split_lane_index_set[0];
  } else if (other_split_lane_index_set.size() > 1) {
    std::shared_ptr<VirtualLane> split_lane1 =
        relative_id_lanes[order_ids[split_lane_index]];
    if (split_lane1 != nullptr) {
      std::shared_ptr<KDPath> split_lane_frenet =
          split_lane1->get_lane_frenet_coord();
      if (split_lane_frenet != nullptr) {
        Point2D split_lane_perception_split_point_frenet1;
        Point2D split_lane_perception_split_point1;
        if (!split_lane_frenet->XYToSL(
                virtual_lane_split_point,
                split_lane_perception_split_point_frenet1)) {
          return;
        }
        Point2D split_lane_perception_split_point_frenet2(
            split_lane_perception_split_point_frenet1.x, 0.0);
        if (!split_lane_frenet->SLToXY(
                split_lane_perception_split_point_frenet2,
                split_lane_perception_split_point1)) {
          return;
        }
        virtual_lane_split_point.x = split_lane_perception_split_point1.x;
        virtual_lane_split_point.y = split_lane_perception_split_point1.y;
      }
    }

    for (int i = 0; i < other_split_lane_index_set.size(); ++i) {
      std::shared_ptr<VirtualLane> other_split_lane1 =
          relative_id_lanes[order_ids[other_split_lane_index_set[i]]];
      if (other_split_lane1 != nullptr) {
        std::shared_ptr<KDPath> other_split_lane_frenet_coord1 =
            other_split_lane1->get_lane_frenet_coord();
        if (other_split_lane_frenet_coord1 != nullptr) {
          Point2D other_split_lane_perception_split_point_frenet1;
          if (!other_split_lane_frenet_coord1->XYToSL(
                  virtual_lane_split_point,
                  other_split_lane_perception_split_point_frenet1)) {
            continue;
          }
          if (std::fabs(other_split_lane_perception_split_point_frenet1.y) >
              split_point_lateral_max_distance) {
            split_point_lateral_max_distance =
                std::fabs(other_split_lane_perception_split_point_frenet1.y);
            other_split_lane_index = other_split_lane_index_set[i];
          }
        }
      }
    }
  }

  if (other_split_lane_index != -1 && split_lane_index != -1) {
    std::shared_ptr<VirtualLane> other_split_lane =
        relative_id_lanes[order_ids[other_split_lane_index]];
    std::shared_ptr<VirtualLane> split_lane =
        relative_id_lanes[order_ids[split_lane_index]];
    if (other_split_lane != nullptr && split_lane != nullptr) {
      std::shared_ptr<KDPath> other_split_lane_frenet_coord =
          other_split_lane->get_lane_frenet_coord();
      std::shared_ptr<KDPath> split_lane_frenet_coord =
          split_lane->get_lane_frenet_coord();
      if (other_split_lane_frenet_coord != nullptr &&
          split_lane_frenet_coord != nullptr) {
        Point2D other_split_lane_perception_split_point_frenet;
        Point2D split_lane_perception_split_point_frenet;
        if (!other_split_lane_frenet_coord->XYToSL(
                virtual_lane_split_point,
                other_split_lane_perception_split_point_frenet)) {
          ILOG_DEBUG << "virtual_lane_split_point in other_split_lane failed!";
        }
        double perception_split_point_lateral_distance =
            std::fabs(other_split_lane_perception_split_point_frenet.y);
        Point2D virtual_split_point;
        // 最大迭代次数
        const int kMaxIterations = 100;
        int iteration_count = 0;
        if (perception_split_point_lateral_distance >
            kDefaultSplitPointExistDistanceThd) {
          while (perception_split_point_lateral_distance >
                     kDefaultSplitPointExistDistanceThd &&
                 iteration_count < kMaxIterations) {
            Point2D virtual_split_point_frenet(
                other_split_lane_perception_split_point_frenet.x -
                    kComputeSplitPointMoveStep,
                0.0);
            Point2D next_virtual_split_point_frenet;
            // 转换失败时直接退出循环，避免无效计算
            if (!other_split_lane_frenet_coord->SLToXY(
                    virtual_split_point_frenet, virtual_split_point) ||
                !split_lane_frenet_coord->XYToSL(
                    virtual_split_point, next_virtual_split_point_frenet)) {
              break;
            }
            perception_split_point_lateral_distance =
                std::fabs(next_virtual_split_point_frenet.y);
            other_split_lane_perception_split_point_frenet.x -=
                kComputeSplitPointMoveStep;
            iteration_count++;
          }
          virtual_lane_split_point.x = virtual_split_point.x;
          virtual_lane_split_point.y = virtual_split_point.y;
        } else if (perception_split_point_lateral_distance <
                   kDefaultSplitPointExistDistanceThd) {
          while (perception_split_point_lateral_distance <
                     kDefaultSplitPointExistDistanceThd &&
                 iteration_count < kMaxIterations) {
            Point2D virtual_split_point_frenet(
                other_split_lane_perception_split_point_frenet.x +
                    kComputeSplitPointMoveStep,
                0.0);
            Point2D next_virtual_split_point_frenet;
            // 转换失败时直接退出循环
            if (!other_split_lane_frenet_coord->SLToXY(
                    virtual_split_point_frenet, virtual_split_point) ||
                !split_lane_frenet_coord->XYToSL(
                    virtual_split_point, next_virtual_split_point_frenet)) {
              break;
            }
            perception_split_point_lateral_distance =
                std::fabs(next_virtual_split_point_frenet.y);
            other_split_lane_perception_split_point_frenet.x +=
                kComputeSplitPointMoveStep;
            iteration_count++;
          }
          virtual_lane_split_point.x = virtual_split_point.x;
          virtual_lane_split_point.y = virtual_split_point.y;
        }
      }
    }
  }

  const double ego_heading_angle = ego_state->heading_angle();
  double clane_min_cost_total = std::numeric_limits<double>::infinity();
  double split_point_front_distance = 25.0;
  double split_point_rear_distance = 20.0;

  if (road_radius_origin < kDefaultRoadRadius) {
    split_point_front_distance = 10.0;
  }

  // 计算归一化的曲率程度（0=纯直道，1=最大曲率）
  double curv_degree =
      std::fabs(current_max_road_radius - current_min_road_radius) > 200.0
          ? 0.0
          : NormalizeCurvatureRadius(road_radius_origin);
  double relative_theta_diff_cost_weight, road_boundary_collision_cost_weight,
      kappa_cost_weight;
  //计算各项cost权重
  const double lateral_dis_cost_weight = 0.01;  // 横向距离权重（固定值）
  CalculateDynamicCostWeights(curv_degree, road_boundary_collision_cost_weight,
                              relative_theta_diff_cost_weight,
                              kappa_cost_weight);

  // Find current frame lane closest to previous frame's ego lane
  int closest_lane_index = -1;
  ComputeZeroRelativeIdOrderIdIndex(last_track_ego_lane_, relative_id_lanes,
                                    order_ids, closest_lane_index);

  // Use closest lane if found, otherwise fallback to last_track_ego_lane_
  std::shared_ptr<VirtualLane> collision_check_lane = last_track_ego_lane_;
  if (closest_lane_index >= 0 && closest_lane_index < order_ids.size() &&
      order_ids[closest_lane_index] < relative_id_lanes.size()) {
    collision_check_lane = relative_id_lanes[order_ids[closest_lane_index]];
  }

  TrajectoryPoints ego_future_trajs;
  for (size_t i = 0; i < order_ids.size(); i++) {
    if (relative_id_lanes.size() > order_ids[i]) {
      std::shared_ptr<VirtualLane> relative_id_lane =
          relative_id_lanes[order_ids[i]];
      if (relative_id_lane == nullptr) {
        continue;
      }
      if (relative_id_lane->get_lane_frenet_coord() != nullptr) {
        GenerateEgoFutureTrajectory(relative_id_lane, 0.0, ego_future_trajs);
        if (IsPathCollisionWithRoadEdge(
            collision_check_lane, relative_id_lane, ego_future_trajs)) {
          continue;
        }
        double total_cost = 0.0;
        double average_heading_angle = 0.0;
        double average_heading_angle_cost = 0.0;
        double heading_angle_total = 0.0;
        double average_kappa_cost = 0.0;
        double total_kappa_cost = 0.0;
        double lateral_dis_cost = 0.0;
        double road_boundary_collision_cost = 0.0;
        std::shared_ptr<KDPath> frenet_coord =
            relative_id_lane->get_lane_frenet_coord();
        Point2D merge_split_frenet_point;
        Point2D virtual_lane_split_front_point;
        Point2D virtual_lane_split_rear_point;
        if (!frenet_coord->XYToSL(virtual_lane_split_point,
                                  merge_split_frenet_point)) {
          continue;
        }
        Point2D virtual_lane_split_front_frenet_point(
            merge_split_frenet_point.x - split_point_front_distance, 0.0);
        if (!frenet_coord->SLToXY(virtual_lane_split_front_frenet_point,
                                  virtual_lane_split_front_point)) {
          continue;
        }

        Point2D virtual_lane_split_rear_frenet_point(
            merge_split_frenet_point.x + split_point_rear_distance, 0.0);
        if (!frenet_coord->SLToXY(virtual_lane_split_rear_frenet_point,
                                  virtual_lane_split_rear_point)) {
          continue;
        }

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
        int target_x = order_ids[i];
        auto iter =
            std::find_if(lane_curv_info_set.begin(), lane_curv_info_set.end(),
                         [target_x](const std::pair<int, double>& elem) {
                           return elem.first == target_x;
                         });
        planning_math::Vec2d ego_to_split_point(
            virtual_lane_split_point.x - virtual_lane_split_front_point.x,
            virtual_lane_split_point.y - virtual_lane_split_front_point.y);
        ego_to_split_point.Normalize();
        planning_math::Vec2d split_point_to_rear(
            virtual_lane_split_rear_point.x - virtual_lane_split_point.x,
            virtual_lane_split_rear_point.y - virtual_lane_split_point.y);
        split_point_to_rear.Normalize();

        double cos_theta = ego_to_split_point.InnerProd(split_point_to_rear);
        double theta_diff = 1.0 - cos_theta;
        average_heading_angle_cost = std::fabs(theta_diff);

        if (iter != lane_curv_info_set.end()) {
          average_kappa_cost = Normalize(iter->second, kMaxKappa);
        }
        lateral_dis_cost = Normalize(std::fabs(ego_l), kMaxLateralDistance);
        auto lane_curv_iter = lanes_curv_info.find(target_x);
        if (lane_curv_iter != lanes_curv_info.end()) {
          CalculateRoadBoundaryCollisionCost(lane_curv_iter->second,
                                             road_boundary_collision_cost,
                                             relative_id_lane);
        }
        // ------------- 计算总代价（归一化代价 × 动态权重）-------------
        total_cost =
            relative_theta_diff_cost_weight * average_heading_angle_cost +
            kappa_cost_weight * average_kappa_cost +
            lateral_dis_cost_weight * lateral_dis_cost +
            road_boundary_collision_cost_weight * road_boundary_collision_cost;
        if (!std::isinf(clane_min_cost_total) &&
            (std::fabs(total_cost - clane_min_cost_total) <
             kUseVirtualLaneProcessSplitCostThd)) {
          enable_use_virtual_lane_process_split = true;
        }
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

  // 转向cost较为接近时采用虚拟车道属性
  bool use_min_steering_result = true;
  if (enable_use_virtual_lane_process_split && order_ids.size() == 2) {
    bool origin_lane_exist_virtual = true;
    std::shared_ptr<VirtualLane> origin_orider_id_lane =
        relative_id_lanes[origin_order_id];
    MakesureVirtualLaneSideIsVirtual(origin_orider_id_lane,
                                     origin_lane_exist_virtual,
                                     last_zero_relative_id_order_id_index_);
    if (origin_lane_exist_virtual) {
      use_min_steering_result = false;
    }
    if (!use_min_steering_result) {
      bool lane_exist_virtual = true;
      for (size_t i = 0; i < order_ids.size(); i++) {
        if (relative_id_lanes.size() > order_ids[i]) {
          std::shared_ptr<VirtualLane> relative_id_lane =
              relative_id_lanes[order_ids[i]];
          MakesureVirtualLaneSideIsVirtual(relative_id_lane, lane_exist_virtual,
                                           i);
          if (!lane_exist_virtual) {
            origin_order_id = relative_id_lane->get_order_id();
            relative_id_lane->set_relative_id(0);
            last_zero_relative_id_order_id_index_ = i;
            last_track_ego_lane_ = relative_id_lane;
            is_exist_split_on_intersection_ = true;
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

  lcc_split_select_is_finish_ = true;
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
    ILOG_DEBUG << "ProcessSplitWithGroundMark::order_ids.size() < 2";
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
    ILOG_DEBUG << "relative_id_lanes.size() == order_ids.size()";
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
      double ego_longit_s = 0.0;
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
            ILOG_ERROR << "update_lane_points: skip NaN point";
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

        if (path_points.size() <
            planning_math::KDPath::kKDPathMinPathPointSize + 1) {
          frenet_coord = nullptr;
          continue;
        }
        frenet_coord =
            std::make_shared<planning_math::KDPath>(std::move(path_points));
        relative_id_lane->set_lane_frenet_coord(frenet_coord);

        if (!frenet_coord->XYToSL(ego_cart_point, ego_cart_point_frenet)) {
          ego_lateral_offset = 0.0;
          ego_longit_s = 0.0;
        } else {
          ego_lateral_offset = ego_cart_point_frenet.y;
          ego_longit_s = ego_cart_point_frenet.x;
        }
      } else {
        for (const auto& point : lane_points) {
          if (std::isnan(point.car_point.x) || std::isnan(point.car_point.y)) {
            ILOG_ERROR << "update_lane_points: skip NaN point";
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
        if (path_points.size() < KDPath::kKDPathMinPathPointSize + 1) {
          frenet_coord = nullptr;
          continue;
        }
        frenet_coord =
            std::make_shared<planning_math::KDPath>(std::move(path_points));
        relative_id_lane->set_lane_frenet_coord(frenet_coord);

        if (!frenet_coord->XYToSL(ego_cart_point, ego_cart_point_frenet)) {
          ego_lateral_offset = 0.0;
          ego_longit_s = 0.0;
        } else {
          ego_lateral_offset = ego_cart_point_frenet.y;
          ego_longit_s = ego_cart_point_frenet.x;
        }
      }
      relative_id_lane->set_ego_lateral_offset(ego_lateral_offset);
      relative_id_lane->set_ego_longit_s(ego_longit_s);
    }
  }
}

double EgoLaneTrackManger::ComputeLanesMatchlaterakDisCost(
    int virtual_id, const std::shared_ptr<VirtualLane> current_relative_id_lane,
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane,
    const bool& is_fix) {
  const double default_lane_mapping_cost = 10.0;
  double order_id_diff_cost = 0.0;
  double k_lane_change_order_id_diff_wegiht =
      is_fix ? kLaneChangeOrderidDiffWeight : 0.0;
  double average_curv = 0.0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  const bool is_merge_region =
      ego_lane_road_right_decider_output.is_merge_region;
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
      double target_ego_s = 50.0;
      double begin_ego_s = 50.0;
      double target_ego_l = 0.0;
      if (!target_lane_frenet_coord->XYToSL(ego_state->ego_pose().x,
                                            ego_state->ego_pose().y,
                                            &target_ego_s, &target_ego_l)) {
        return default_lane_mapping_cost;
      }
      if (is_merge_region) {
        begin_ego_s = target_ego_s - 20.0;
      } else {
        begin_ego_s = target_ego_s;
      }

      if (road_radius > kDefaultRoadRadius) {
        int select_lane_point_interval = 1;
        for (int i = 0; i < lane_points.size();
             i += select_lane_point_interval) {
          iflyauto::ReferencePoint point = lane_points[i];
          if (std::isnan(point.local_point.x) ||
              std::isnan(point.local_point.y)) {
            ILOG_ERROR << "update_lane_points: skip NaN point";
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
          if (s < begin_ego_s) {
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
            ILOG_ERROR << "update_lane_points: skip NaN point";
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
          if (s < begin_ego_s) {
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
      int lane_order_id = current_relative_id_lane->get_order_id();
      order_id_diff_cost =
          std::fabs(lane_order_id - current_fix_lane_order_id_);
      if (point_nums < kLeastDefaultPointNums) {
        lane_mapping_cost = default_lane_mapping_cost;
      } else {
        lane_mapping_cost =
            std::fabs(total_lateral_offset / point_nums) +
            k_lane_change_order_id_diff_wegiht * order_id_diff_cost;
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
  if (frenet_coord == nullptr) {
    return average_curv;
  }
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!frenet_coord->XYToSL(ego_state->ego_pose().x, ego_state->ego_pose().y,
                            &ego_s, &ego_l)) {
    return average_curv;
  }
  if (ego_s + default_begin_length > frenet_coord->Length()) {
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
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  if (!session_->environmental_model().get_route_info()->get_sdmap_valid()) {
    ILOG_DEBUG << "CheckIfInRampSelectSplit::sd_map is invalid!!!";
    return false;
  }
  ILOG_DEBUG << "CheckIfInRampSelectSplit::sd_map is valid";

  if (order_ids.size() < 2) {
    ILOG_DEBUG << "CheckIfInRampSelectSplit::order_ids.size() < 2";
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

bool EgoLaneTrackManger::CheckIfInRampSelectSplitForSdpro(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  if (!session_->environmental_model().get_route_info()->get_sdpromap_valid()) {
    LOG_DEBUG("CheckIfInRampSelectSplitForSdpro::sd_map is invalid!!!");
    return false;
  }
  LOG_DEBUG("CheckIfInRampSelectSplitForSdpro::sd_map is valid");

  if (order_ids.size() < 2) {
    LOG_DEBUG("CheckIfInRampSelectSplitForSdpro::order_ids.size() < 2");
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

  const auto& sdpro_map =
      session_->environmental_model().get_route_info()->get_sdpro_map();
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  // 获取当前的segment
  double temp_nearest_s = 0;
  double nearest_l = 0;

  ad_common::math::Vec2d current_point;
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  bool is_search_cur_link = true;
  current_link_ = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      temp_nearest_s, nearest_l, is_search_cur_link);
  if (current_link_ == nullptr) {
    return false;
  }
  const bool current_is_on_ramp = sdpro_map.isRamp(current_link_->link_type());
  if (current_is_on_ramp) {
    return true;
  }

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
          const iflymapdata::sdpro::LinkInfo_Link* target_link =
              sdpro_map.GetNearestLinkWithHeading(
                  segment_target_point, search_distance, ego_heading_angle,
                  max_heading_diff, temp_nearest_s, nearest_l, false);
          if (target_link != nullptr) {
            if (!sdpro_map.isRamp(target_link->link_type())) {
              return false;
            }
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    }
  }

  return true;
}

bool EgoLaneTrackManger::CheckIfInRoadSelectRamp(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  if (!session_->environmental_model().get_route_info()->get_sdmap_valid()) {
    ILOG_DEBUG << "CheckIfInRoadSelectRamp::sd_map is invalid!!!";
    return false;
  }
  ILOG_DEBUG << "CheckIfInRoadSelectRamp::sd_map is valid";

  if (order_ids.size() < 2) {
    ILOG_DEBUG << "CheckIfInRoadSelectRamp::order_ids.size() < 2";
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

bool EgoLaneTrackManger::CheckIfInRoadSelectRampForSdpro(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids) {
  if (!session_->environmental_model().get_route_info()->get_sdpromap_valid()) {
    LOG_DEBUG("CheckIfInRoadSelectRampForSdpro::sd_map is invalid!!!");
    return false;
  }
  LOG_DEBUG("CheckIfInRoadSelectRampForSdpro::sd_map is valid");

  if (order_ids.size() < 2) {
    LOG_DEBUG("CheckIfInRoadSelectRampForSdpro::order_ids.size() < 2");
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

  const auto& sdpro_map =
      session_->environmental_model().get_route_info()->get_sdpro_map();
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  // 获取当前的segment
  ad_common::math::Vec2d current_point;
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  double temp_nearest_s = 0;
  double nearest_l = 0;
  bool is_search_cur_link = true;
  const iflymapdata::sdpro::LinkInfo_Link* current_link =
      sdpro_map.GetNearestLinkWithHeading(
          current_point, search_distance, ego_heading_angle, max_heading_diff,
          temp_nearest_s, nearest_l, is_search_cur_link);
  if (current_link == nullptr) {
    return false;
  }
  const bool current_is_on_ramp = sdpro_map.isRamp(current_link->link_type());
  if (!current_is_on_ramp) {
    for (size_t i = 0; i < order_ids.size(); i++) {
      if (relative_id_lanes.size() > order_ids[i]) {
        std::shared_ptr<VirtualLane> base_lane =
            relative_id_lanes[order_ids[i]];
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
            const iflymapdata::sdpro::LinkInfo_Link* next_link =
                sdpro_map.GetNearestLinkWithHeading(
                    segment_target_point, search_distance, ego_heading_angle,
                    max_heading_diff, temp_nearest_s, nearest_l, false);
            if (next_link != nullptr) {
              if (!sdpro_map.isRamp(next_link->link_type())) {
                return true;
              }
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
          ILOG_ERROR << "update_lane_points: skip NaN point";
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
        total_lateral_offset += std::fabs(lateral_offset);
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

void EgoLaneTrackManger::MakesureVirtualLaneSideIsVirtual(
    const std::shared_ptr<VirtualLane> base_lane,
    bool& virtual_lane_exist_virtual, const int lane_index) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  if (base_lane == nullptr) {
    return;
  }

  // 判断左侧车道线类型
  bool left_boundary_exist_virtual_type = false;
  double ego_s = base_lane->get_ego_longit_s();
  for (const auto& point : base_lane->lane_points()) {
    if (point.s < ego_s + kLaneLineSegmentLength) {
      continue;
    }
    if (point.s > ego_s + kDefaultConsiderVirtualLineLength) {
      break;
    }

    if (point.left_lane_border_type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      left_boundary_exist_virtual_type = true;
      break;
    }
  }

  // 判断右侧车道线类型
  bool right_boundary_exist_virtual_type = false;
  for (const auto& point : base_lane->lane_points()) {
    if (point.s < ego_s + kLaneLineSegmentLength) {
      continue;
    }
    if (point.s > ego_s + kDefaultConsiderVirtualLineLength) {
      break;
    }
    if (point.right_lane_border_type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      right_boundary_exist_virtual_type = true;
      break;
    }
  }

  if (lane_index == 0) {
    if (!right_boundary_exist_virtual_type) {
      virtual_lane_exist_virtual = false;
    }
  } else {
    if (!left_boundary_exist_virtual_type) {
      virtual_lane_exist_virtual = false;
    }
  }

  return;
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

  // 判断左侧车道线类型
  bool left_boundary_exist_virtual_type = false;
  double ego_s = base_lane->get_ego_longit_s();
  for (const auto& point : base_lane->lane_points()) {
    if (point.s < ego_s + kLaneLineSegmentLength) {
      continue;
    }
    if (point.s > ego_s + kDefaultConsiderVirtualLineLength) {
      break;
    }

    if (point.left_lane_border_type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      left_boundary_exist_virtual_type = true;
      break;
    }
  }

  // 判断右侧车道线类型
  bool right_boundary_exist_virtual_type = false;
  for (const auto& point : base_lane->lane_points()) {
    if (point.s < ego_s + kLaneLineSegmentLength) {
      continue;
    }
    if (point.s > ego_s + kDefaultConsiderVirtualLineLength) {
      break;
    }
    if (point.right_lane_border_type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      right_boundary_exist_virtual_type = true;
      break;
    }
  }

  if (!left_boundary_exist_virtual_type && !right_boundary_exist_virtual_type) {
    virtual_lane_exist_virtual = false;
  }
  return;
}

void EgoLaneTrackManger::ComputeIsSplitRegion(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const std::vector<int>& order_ids,
    const std::unordered_map<int, std::shared_ptr<VirtualLane>>&
        virtual_id_mapped_lane) {
  std::shared_ptr<VirtualLane> relative_left_lane = nullptr;
  std::shared_ptr<VirtualLane> relative_right_lane = nullptr;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_vel = ego_state->ego_v();
  sum_distance_from_ego_to_both_center_lines_ = 3.75;

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  const double default_lane_mapping_cost = 10.0;
  const int fix_lane_virtual_id =
      lane_change_decider_output.fix_lane_virtual_id;
  const int lc_state = lane_change_decider_output.curr_state;
  std::shared_ptr<VirtualLane> last_track_virtual_id_lane;
  bool is_split_region_include_fix_lane = true;
  int last_track_virtual_id = 0;
  const auto& plannig_init_point = ego_state->planning_init_point();
  ego_in_split_region_ = false;
  if (!virtual_id_mapped_lane.empty()) {
    for (const auto& virtual_id_lane : virtual_id_mapped_lane) {
      if (virtual_id_lane.second->get_virtual_id() == fix_lane_virtual_id) {
        last_track_virtual_id_lane = virtual_id_lane.second;
        last_track_virtual_id = virtual_id_lane.first;
      }
    }
  }

  Point2D ego_point = {ego_state->planning_init_point().x,
                       ego_state->planning_init_point().y};
  // if (order_ids.size() < 2) {
  //   ego_in_split_region_ = false;
  //   return;
  // }
  if (last_track_virtual_id_lane == nullptr) {
    return;
  }
  auto last_track_ego_lane_frenet_coord =
      last_track_virtual_id_lane->get_lane_frenet_coord();
  if (last_track_ego_lane_frenet_coord == nullptr) {
    return;
  }

  std::vector<int> relative_lane_order_ids;
  if (order_ids.size() > 2) {
    ego_in_split_region_ = true;
    return;
  } else {
    bool has_left_lane = false;
    bool has_right_lane = false;
    if (order_ids.size() == 2) {
      if (order_ids[0] < relative_id_lanes.size()) {
        relative_left_lane = relative_id_lanes[order_ids[0]];
        relative_lane_order_ids.emplace_back(
            relative_left_lane->get_order_id());
      }
      if (order_ids[1] < relative_id_lanes.size()) {
        relative_right_lane = relative_id_lanes[order_ids[1]];
        relative_lane_order_ids.emplace_back(
            relative_right_lane->get_order_id());
      }
    } else {
      double k_right_lane_lateral_distance =
          -std::numeric_limits<double>::max();
      double k_left_lane_lateral_distance = std::numeric_limits<double>::max();
      int left_lane_index = -1;
      int right_lane_index = -1;

      for (const auto& relative_id_lane : relative_id_lanes) {
        double ego_to_lane_lateral_distance =
            relative_id_lane->get_ego_lateral_offset();
        if (ego_to_lane_lateral_distance >= 0 &&
            ego_to_lane_lateral_distance < k_left_lane_lateral_distance) {
          k_left_lane_lateral_distance = ego_to_lane_lateral_distance;
          right_lane_index = relative_id_lane->get_order_id();
        }
        if (ego_to_lane_lateral_distance < 0 &&
            ego_to_lane_lateral_distance > k_right_lane_lateral_distance) {
          k_right_lane_lateral_distance = ego_to_lane_lateral_distance;
          left_lane_index = relative_id_lane->get_order_id();
        }
      }
      if (left_lane_index != -1 && left_lane_index < relative_id_lanes.size()) {
        relative_left_lane = relative_id_lanes[left_lane_index];
      }
      if (right_lane_index != -1 &&
          right_lane_index < relative_id_lanes.size()) {
        relative_right_lane = relative_id_lanes[right_lane_index];
      }
    }

    if (relative_left_lane != nullptr) {
      if (relative_left_lane->get_lane_frenet_coord() != nullptr) {
        has_left_lane = true;
        relative_lane_order_ids.emplace_back(
            relative_left_lane->get_order_id());
      }
    }
    if (relative_right_lane != nullptr) {
      if (relative_right_lane->get_lane_frenet_coord() != nullptr) {
        has_right_lane = true;
        relative_lane_order_ids.emplace_back(
            relative_right_lane->get_order_id());
      }
    }
    const auto& relative_left_lane_points = relative_left_lane->lane_points();
    if (!has_left_lane || !has_right_lane ||
        relative_left_lane_points.size() < 3) {
      return;
    }

    double clane_min_lateral_distance = std::numeric_limits<double>::max();
    if (!relative_lane_order_ids.empty()) {
      for (size_t i = 0; i < relative_lane_order_ids.size(); i++) {
        if (relative_id_lanes.size() > relative_lane_order_ids[i]) {
          std::shared_ptr<VirtualLane> relative_id_lane =
              relative_id_lanes[relative_lane_order_ids[i]];
          const auto& lane_points = relative_id_lane->lane_points();
          if (lane_points.size() <= 2) {
            continue;
          }
          auto cur_lane_frenet_coord =
              relative_id_lane->get_lane_frenet_coord();
          if (cur_lane_frenet_coord == nullptr) {
            continue;
          }
          double target_ego_s = 0.0;
          double target_ego_l = 0.0;
          Point2D ego_cart_target_frenet;
          if (!cur_lane_frenet_coord->XYToSL(ego_point,
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
          for (int i = 0; i < lane_points.size();
               i += select_lane_point_interval) {
            iflyauto::ReferencePoint point = lane_points[i];
            if (std::isnan(point.local_point.x) ||
                std::isnan(point.local_point.y)) {
              ILOG_ERROR << "update_lane_points: skip NaN point";
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
            total_lateral_offset += std::fabs(lateral_offset);
            point_nums += 1;
            if (point_nums >= kDefaultPointNums || s > target_ego_s + 50.0) {
              break;
            }
          }
          point_nums = std::max(1, point_nums);
          cumu_lat_dis_cost = std::fabs(total_lateral_offset / point_nums);
          if (cumu_lat_dis_cost < clane_min_lateral_distance) {
            clane_min_lateral_distance = cumu_lat_dis_cost;
          }
        }
      }
      if (clane_min_lateral_distance >= 1.5) {
        is_split_region_include_fix_lane = false;
      }
    }

    const auto& right_lane_frenet_crd =
        relative_right_lane->get_lane_frenet_coord();
    const auto& left_lane_frenet_crd =
        relative_left_lane->get_lane_frenet_coord();
    double ego_s_base_right = 0.0;
    double ego_l_base_right = 0.0;
    Point2D ego_cart_point(ego_point.x, ego_point.y);
    Point2D ego_frenet_right;
    Point2D right_lane_ego_cart;
    Point2D ego_frenet_left;
    if (!right_lane_frenet_crd->XYToSL(ego_cart_point, ego_frenet_right)) {
      ego_frenet_right.x = 50.0;
      ego_frenet_right.y = 0.0;
    }
    ego_frenet_right.y = 0.0;

    if (!right_lane_frenet_crd->SLToXY(ego_frenet_right, right_lane_ego_cart)) {
      right_lane_ego_cart = ego_cart_point;
    }
    if (!left_lane_frenet_crd->XYToSL(right_lane_ego_cart, ego_frenet_left)) {
      ego_frenet_left.y = 3.75;
    }
    sum_distance_from_ego_to_both_center_lines_ = std::fabs(ego_frenet_left.y);
    double near_average_l = 0.0;
    double behind_average_l = 0.0;
    double far_average_l = 0.0;
    int32_t near_pt_count = 0;
    int32_t behind_pt_count = 0;
    int32_t far_pt_count = 0;
    double near_pt_sum_l = 0.0;
    double far_pt_sum_l = 0.0;
    double behind_pt_sum_l = 0.0;
    for (const auto point : relative_left_lane_points) {
      if (std::isnan(point.local_point.x) || std::isnan(point.local_point.y)) {
        ILOG_ERROR << "update_lane_points: skip NaN point";
        continue;
      }
      double pt_s = 0.0;
      double pt_l = 0.0;
      right_lane_frenet_crd->XYToSL(point.local_point.x, point.local_point.y,
                                    &pt_s, &pt_l);
      const double pt_distance_to_ego = pt_s - ego_frenet_right.x;
      if (pt_l < 0.0 || pt_s > right_lane_frenet_crd->Length()) {
        continue;
      }
      if (pt_distance_to_ego > -kNearPreviewDistanceThd &&
          pt_distance_to_ego < 0.0) {
        ++near_pt_count;
        near_pt_sum_l += pt_l;
      }
      if (pt_distance_to_ego < -kNearPreviewDistanceThd) {
        ++behind_pt_count;
        behind_pt_sum_l += pt_l;
      }
      if (pt_distance_to_ego > kEgoPreviewTimeMinThd * ego_vel &&
          pt_distance_to_ego < kEgoPreviewTimeMaxThd * ego_vel) {
        ++far_pt_count;
        far_pt_sum_l += pt_l;
      }
    }
    near_pt_count = std::max(near_pt_count, 1);
    far_pt_count = std::max(far_pt_count, 1);
    behind_pt_count = std::max(behind_pt_count, 1);

    near_average_l = std::fabs(near_pt_sum_l / near_pt_count);
    far_average_l = std::fabs(far_pt_sum_l / far_pt_count);
    behind_average_l = std::fabs(behind_pt_sum_l / behind_pt_count);
    if (((far_average_l - kExistSplitLateralDisThd > near_average_l) &&
         near_average_l < kExistSplitEgoRearLateralDisThd) ||
        near_average_l < kCenterLineLateralDisThd ||
        ((near_average_l * 0.5 > behind_average_l) &&
         far_average_l - 1.0 > near_average_l)) {
      if (is_split_region_include_fix_lane) {
        ego_in_split_region_ = true;
      }
    }
  }

  return;
}

double EgoLaneTrackManger::CalculateLinearCost(double ttc) {
  const double k_ttc_max = 5.0;  // 安全TTC阈值（秒），超过则无代价
  const double k_ttc_low = 1.0;  // 低TTC阈值（秒），分段函数的分界点

  // 处理无穷大TTC（无碰撞）
  if (ttc > k_ttc_max) {
    return 0.0;
  }
  // TTC≤0时代价为1，否则线性计算
  double cost = 1.0 - (ttc / k_ttc_max);
  return std::clamp(cost, 0.0, 1.0);  // 限制代价在0~1之间
}

double EgoLaneTrackManger::Normalize(double value, double max_value) {
  if (max_value <= 0) return 0.0;
  double norm = value / max_value;
  // 限制在[0,1]，避免超出范围
  return std::clamp(norm, 0.0, 1.0);
}

double EgoLaneTrackManger::NormalizeCurvatureRadius(double radius) {
  // 限制半径范围：小于最小半径按最小算，大于最大半径按最大算
  double clamped_radius =
      std::clamp(radius, kMinCurvatureRadius, kMaxCurvatureRadius);
  // 转换为曲率程度（直道=0，最大曲率=1）
  double curv_degree = (kMaxCurvatureRadius - clamped_radius) /
                       (kMaxCurvatureRadius - kMinCurvatureRadius);
  return curv_degree;
}

double EgoLaneTrackManger::NormalizeCurvatureSign(double dis_to_ego) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double min_consider_dis = std::max(ego_state->ego_v(), 1.0) * 1.5;
  double max_consider_dis = std::max(ego_state->ego_v(), 1.0) * 4.0;
  double clamped_dis =
      std::clamp(dis_to_ego, min_consider_dis, max_consider_dis);
  // 转换为曲率符号程度（离自车距离越近 权重越大）
  double sign_degree =
      (max_consider_dis - clamped_dis) / (max_consider_dis - min_consider_dis);
  return sign_degree;
}

void EgoLaneTrackManger::ComputeEgoDistanceToRoadBorder(
    const std::shared_ptr<VirtualLane>& base_lane,
    double& dis_to_left_road_border, double& dis_to_right_road_border) {
  ReferencePathPoint sample_path_point{};
  const double cut_length = 1.4;
  const double sample_forward_distance = 1.0;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  if (!base_lane) {
    return;
  }
  const std::shared_ptr<planning_math::KDPath> base_frenet_coord =
      base_lane->get_lane_frenet_coord();
  if (!base_frenet_coord) {
    return;
  }
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const PlanningInitPoint planning_init_point =
      ego_state->planning_init_point();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point.lat_init_state.x(),
                         planning_init_point.lat_init_state.y()};
  double ego_s = 50.0;
  double ego_l = 0.0;
  if (base_frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
    ego_s = ego_frenet_point.x;
  }

  auto& lane_points = base_lane->lane_points();
  for (auto& refline_pt : lane_points) {
    if (refline_pt.s < ego_s) {
      continue;
    }
    if (refline_pt.s > ego_s + ego_state->ego_v() * 5.0) {
      break;
    }

    dis_to_left_road_border = std::fmin(refline_pt.distance_to_left_road_border,
                                        dis_to_left_road_border);
    dis_to_right_road_border = std::fmin(
        refline_pt.distance_to_right_road_border, dis_to_right_road_border);
  }
  dis_to_left_road_border -= ego_l;
  dis_to_right_road_border += ego_l;

  return;
}

void EgoLaneTrackManger::CalculateLaneCurvature(
    const std::shared_ptr<planning_math::KDPath>& lane_frenet_coord,
    std::vector<std::pair<int, double>>& lane_curv_info_set,
    LaneCurvInfo& lane_curv_info, double& max_road_radius,
    double& min_road_radius,
    const std::shared_ptr<VirtualLane>& relative_id_lane) {
  // 1. 内部获取ego_state（与原逻辑一致）
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();

  // 2. 初始化采样参数（与原逻辑一致）
  double k_ego_look_ahead_time = kEgoLookAheadTime;
  double k_sampling_step_i = kSamplingStepI;
  double k_sampling_step_j = kSamplingStepJ;
  if (ego_state->ego_v() < 8.34) {  // 车速<30kph
    k_ego_look_ahead_time = kEgoLookAheadTime * 0.5;
    k_sampling_step_i = 5.0;
  }

  Point2D cart_point(ego_state->planning_init_point().x,
                     ego_state->planning_init_point().y);
  Point2D ego_frenet_point(50.0, 0.0);
  if (!lane_frenet_coord->XYToSL(cart_point, ego_frenet_point)) {
    return;
  }

  const double initial_start_s =
      ego_frenet_point.x + ego_state->ego_v() * k_ego_look_ahead_time;
  double lane_curv = 0.0;
  int curv_sign = 0;
  double curv_sign_total = 0.0;

  // 内存优化：预留采样窗口空间
  const int max_curv_window_size = 2 * kSamplingRangeJ + 1;
  std::vector<double> curv_window_vec;
  curv_window_vec.reserve(max_curv_window_size);

  for (double i = 0; i <= 3; ++i) {
    const double start_s = initial_start_s + i * k_sampling_step_i;
    int current_lane_curv_sign = 0;
    double curv_sign_degree = 0.0;
    curv_window_vec.clear();

    for (int j = -kSamplingRangeJ; j <= kSamplingRangeJ; ++j) {
      const double sample_s = start_s + j * k_sampling_step_j;
      double curv = kDefaultCurvature;
      if (!lane_frenet_coord->GetKappaByS(sample_s, &curv)) {
        continue;
      }
      if (std::fabs(curv) > 0.0005) {
        current_lane_curv_sign = curv > 0 ? 1 : -1;
      }
      curv_sign_degree = NormalizeCurvatureSign(sample_s - ego_frenet_point.x);
      curv_sign_total += current_lane_curv_sign * curv_sign_degree;
      curv_window_vec.emplace_back(std::fabs(curv));
    }

    if (curv_window_vec.empty()) {
      continue;
    }
    const double curv_sum =
        std::accumulate(curv_window_vec.begin(), curv_window_vec.end(), 0.0);
    const double avg_curv = curv_sum / curv_window_vec.size();
    if (avg_curv > lane_curv) {
      lane_curv = avg_curv;
    }
  }

  if (std::fabs(curv_sign_total) > 1e-3) {
    curv_sign = curv_sign_total > 0 ? 1 : -1;
  }

  // 存储曲率结果
  lane_curv_info_set.emplace_back(relative_id_lane->get_order_id(), lane_curv);
  const double road_radius = 1.0 / std::max(lane_curv, kMinCurvature);
  lane_curv_info.curv_radius = road_radius;
  lane_curv_info.curv = lane_curv;
  lane_curv_info.curv_sign = curv_sign;

  // 4. 更新最大/最小车道半径
  if (lane_curv_info.curv_radius > max_road_radius) {
    max_road_radius = lane_curv_info.curv_radius;
  }
  if (lane_curv_info.curv_radius < min_road_radius) {
    min_road_radius = lane_curv_info.curv_radius;
  }
  return;
}

void EgoLaneTrackManger::CalculateDynamicCostWeights(
    double curv_degree, double& road_boundary_collision_cost_weight,
    double& relative_theta_diff_cost_weight, double& kappa_cost_weight) {
  // 步骤1：定义权重的两个端点（直道→最大曲率，与原逻辑完全一致）
  const double theta_weight_straight = 0.8;       // 直道夹角权重
  const double collision_weight_straight = 0.02;  // 直道碰撞权重
  const double kappa_weight_straight = 0.1;       // 直道曲率权重
  const double theta_weight_max_curv = 0.15;      // 最大曲率夹角权重
  const double collision_weight_max_curv = 0.6;   // 最大曲率碰撞权重
  const double kappa_weight_max_curv = 0.2;       // 最大曲率曲率权重
  const double lateral_dis_weight = 0.01;  // 横向距离权重（固定值）

  // 步骤2：线性插值计算当前曲率对应的权重（连续动态变化）
  relative_theta_diff_cost_weight =
      theta_weight_straight +
      curv_degree * (theta_weight_max_curv - theta_weight_straight);
  road_boundary_collision_cost_weight =
      collision_weight_straight +
      curv_degree * (collision_weight_max_curv - collision_weight_straight);
  kappa_cost_weight =
      kappa_weight_straight +
      curv_degree * (kappa_weight_max_curv - kappa_weight_straight);

  // 步骤3：权重归一化（确保总和为1，容错）
  double total_weight = kappa_cost_weight + relative_theta_diff_cost_weight +
                        road_boundary_collision_cost_weight +
                        lateral_dis_weight;

  if (std::fabs(total_weight - 1.0) > 1e-6) {
    relative_theta_diff_cost_weight /= total_weight;
    road_boundary_collision_cost_weight /= total_weight;
    kappa_cost_weight /= total_weight;
    // 横向距离权重也归一化（与原逻辑一致）
    // 注：原逻辑中lateral_dis_cost_weight是局部变量，无需返回，仅参与归一化
  }
}

void EgoLaneTrackManger::CalculateRoadBoundaryCollisionCost(
    const LaneCurvInfo& lane_curv_info, double& road_boundary_collision_cost,
    const std::shared_ptr<VirtualLane>& relative_id_lane) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  // 初始化输出参数为0（默认无风险）
  road_boundary_collision_cost = 0.0;

  // 初始化到左右路沿的距离（默认10米，与原逻辑一致）
  double dis_to_left_road_border = 10.0;
  double dis_to_right_road_border = 10.0;

  // 计算自车到道路边界的实际距离
  ComputeEgoDistanceToRoadBorder(relative_id_lane, dis_to_left_road_border,
                                 dis_to_right_road_border);

  // 初始化TTI（Time To Impact）为5.0
  double ttc = 5.5;
  // 取曲率绝对值（避免负号影响计算）
  double curv_abs = std::fabs(lane_curv_info.curv);

  // 根据曲率符号计算不同方向的TTI（与原逻辑完全一致）
  if (lane_curv_info.curv_sign == -1) {
    // 曲率为负：计算到左侧路沿的TTI
    double sqrt_2l = std::sqrt(2 * dis_to_left_road_border);
    double denominator = ego_state->ego_v() * std::sqrt(curv_abs);
    denominator = std::max(0.01, denominator);  // 避免除零
    ttc = sqrt_2l / denominator;
  } else if (lane_curv_info.curv_sign == 1) {
    // 曲率为正：计算到右侧路沿的TTI
    double sqrt_2l = std::sqrt(2 * dis_to_right_road_border);
    double denominator = ego_state->ego_v() * std::sqrt(curv_abs);
    denominator = std::max(0.01, denominator);  // 避免除零
    ttc = sqrt_2l / denominator;
  }

  // 根据TTI计算线性碰撞成本，赋值给输出参数
  road_boundary_collision_cost = CalculateLinearCost(ttc);
}

double EgoLaneTrackManger::EWMAFilter(double current_value, double alpha,
                                      double& filtered_history) {
  double filtered;
  if (std::fabs(filtered_history) > 1000.0) {
    // 首次滤波：无历史值，直接用当前值初始化
    filtered = current_value;
  } else {
    // EWMA核心公式
    filtered = alpha * current_value + (1 - alpha) * filtered_history;
  }
  // 更新历史值（供下一帧使用）
  filtered_history = filtered;
  return filtered;
}

void EgoLaneTrackManger::GenerateEgoFutureTrajectory(
    const std::shared_ptr<VirtualLane>& target_lane,
    const double lat_offset,
    TrajectoryPoints& ego_future_trajectory) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  ego_future_trajectory.clear();

  // 仿真参数
  const double dt = 0.2;                  // 步长0.2s
  const double max_simulate_time = 5.0;    // 最多仿真5秒
  const double max_sim_steps = static_cast<int>(max_simulate_time / dt);

  const auto& car_param = VehicleConfigurationContext::Instance()->get_vehicle_param();
  const PlanningInitPoint planning_init_point =
      ego_state->planning_init_point();
  const auto& ref_frenet_coord = target_lane->get_lane_frenet_coord();

  // 获取速度限制
  const auto& speed_limit_decider_output = session_->planning_context().speed_limit_decider_output();
  double speed_limit_ref = std::numeric_limits<double>::max();
  auto speed_limit_type_ref = SpeedLimitType::NONE;
  speed_limit_decider_output.GetSpeedLimit(&speed_limit_ref, &speed_limit_type_ref);

  // 初始化速度和减速度
  double current_v = planning_init_point.v;
  const double comfortable_decel = -2.0;  // 舒适减速度 -2m/s²
  const double wheel_base = car_param.wheel_base;

  // 纯追踪模型初始化
  BasicPurePursuitModel pp_model_;
  BasicPurePursuitModel::ModelState pp_model_state(
      planning_init_point.x,
      planning_init_point.y,
      planning_init_point.heading_angle,
      current_v);

  pp_model_.ProcessReferencePath(target_lane);

  // 起点
  TrajectoryPoint init_point;
  init_point.x = planning_init_point.x;
  init_point.y = planning_init_point.y;
  init_point.v = current_v;
  init_point.a = 0.0;                  // 匀速，无加速度
  init_point.heading_angle = planning_init_point.heading_angle;
  init_point.s = planning_init_point.frenet_state.s;
  init_point.l = planning_init_point.frenet_state.r;
  init_point.t = 0.0;
  init_point.delta = planning_init_point.delta;

  ego_future_trajectory.push_back(init_point);
  double last_s = init_point.s;
  const double min_s_inc = 0.01;

  for (int step = 0; step < max_sim_steps; ++step) {
    // 根据速度限制更新当前速度（舒适减速）
    if (current_v > speed_limit_ref) {
      // 计算减速后的速度: v = v0 + a*t
      double new_v = current_v + comfortable_decel * dt;
      // 确保不低于限速
      current_v = std::max(new_v, speed_limit_ref);
    }

    double ld = current_v * 1.2 + 1.0;  // 预瞄距离
    BasicPurePursuitModel::ModelParam pp_param(ld, wheel_base);
    pp_model_.set_model_state(pp_model_state);
    pp_model_.set_model_param(pp_param);
    pp_model_.CalculateDesiredDelta(lat_offset);

    double delta = pp_model_.get_delta();
    delta = std::clamp(delta, -0.5, 0.5);

    pnc::steerModel::VehicleSimulation sim;
    pnc::steerModel::VehicleState state{pp_model_state.x, pp_model_state.y, pp_model_state.theta};
    pnc::steerModel::VehicleControl ctrl{current_v, delta};
    sim.Init(state);
    sim.Update(ctrl, {});
    auto new_state = sim.GetState();

    double s = 0.0, l = 0.0;
    if (!ref_frenet_coord->XYToSL(new_state.x_, new_state.y_, &s, &l)) {
      return;
    }

    // 保证s单调
    if (s <= last_s) s = last_s + min_s_inc;
    last_s = s;

    // 计算实际加速度（用于记录）
    double actual_accel = 0.0;
    if (current_v > speed_limit_ref) {
      actual_accel = comfortable_decel;
    }

    // 构造轨迹点
    TrajectoryPoint pt;
    pt.x = new_state.x_;
    pt.y = new_state.y_;
    pt.heading_angle = new_state.phi_;
    pt.v = current_v;
    pt.a = actual_accel;
    pt.s = s;
    pt.l = l;
    pt.t = (step + 1) * dt;
    pt.delta = delta;

    ego_future_trajectory.push_back(pt);

    // 更新PP状态
    pp_model_state.x = new_state.x_;
    pp_model_state.y = new_state.y_;
    pp_model_state.theta = new_state.phi_;
    pp_model_state.vel = current_v;
  }

  return;
}

bool EgoLaneTrackManger::IsPathCollisionWithRoadEdge(
    const std::shared_ptr<VirtualLane>& last_lane,
    const std::shared_ptr<VirtualLane>& cur_lane,
    const TrajectoryPoints& path_points) {
  const double road_edge_buffer = 0.1;
  if (last_lane == nullptr) {
    return true;
  }
  const auto& origin_lane_points = last_lane->lane_points();
  if (origin_lane_points.empty()) {
    return true;
  }
  if (path_points.size() < 3) {
    return false;
  }
  const bool has_target_lane = cur_lane != nullptr;
  const std::vector<iflyauto::ReferencePoint>* target_lane_points_ptr =
      has_target_lane ? &cur_lane->lane_points() : nullptr;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_vehicle_width = vehicle_param.width * 0.5;

  for (size_t i = 0; i < path_points.size(); ++i) {
    const double pt_x = path_points[i].x;
    const double pt_y = path_points[i].y;

    // 在 last_lane上找最近点
    size_t origin_nearest_idx = 0;
    double origin_min_dist_sq = std::numeric_limits<double>::max();
    for (size_t j = 0; j < origin_lane_points.size(); ++j) {
      const double dx = origin_lane_points[j].enu_point.x - pt_x;
      const double dy = origin_lane_points[j].enu_point.y - pt_y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < origin_min_dist_sq) {
        origin_min_dist_sq = dist_sq;
        origin_nearest_idx = j;
      }
    }

    // 在 cur_lane 上找最近点
    size_t target_nearest_idx = 0;
    double target_min_dist_sq = std::numeric_limits<double>::max();
    const bool target_valid =
        has_target_lane && target_lane_points_ptr != nullptr &&
        !target_lane_points_ptr->empty();
    if (target_valid) {
      for (size_t j = 0; j < target_lane_points_ptr->size(); ++j) {
        const double dx = (*target_lane_points_ptr)[j].enu_point.x - pt_x;
        const double dy = (*target_lane_points_ptr)[j].enu_point.y - pt_y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < target_min_dist_sq) {
          target_min_dist_sq = dist_sq;
          target_nearest_idx = j;
        }
      }
    }

    // 选择距离更近的 lane
    const bool use_target =
        target_valid && target_min_dist_sq < origin_min_dist_sq;
    const auto& nearest_pt =
        use_target ? (*target_lane_points_ptr)[target_nearest_idx]
                   : origin_lane_points[origin_nearest_idx];
    const double ref_x = nearest_pt.enu_point.x;
    const double ref_y = nearest_pt.enu_point.y;
    const double ref_heading = nearest_pt.enu_heading;

    const double dx = pt_x - ref_x;
    const double dy = pt_y - ref_y;
    const double l = -dx * std::sin(ref_heading) + dy * std::cos(ref_heading);

    const double left_vehicle_edge = l + half_vehicle_width;
    const double right_vehicle_edge = l - half_vehicle_width;
    if (left_vehicle_edge >
        nearest_pt.distance_to_left_road_border - road_edge_buffer) {
      return true;
    }
    if (right_vehicle_edge <
        -nearest_pt.distance_to_right_road_border + road_edge_buffer) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
