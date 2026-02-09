#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common_platform_type_soc.h"
#include "define/geometry.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "longitudinal_motion_planner.pb.h"
#include "map_data.pb.h"
#include "mjson/mjson.hpp"
#include "planning_def.h"
#include "spline.h"
#include "trajectory1d/trajectory1d.h"
#include "utils/cartesian_coordinate_system.h"
#include "utils/frenet_coordinate_system.h"

#define DEFAULT_LANE_WIDTH (3.8)

namespace planning {

enum RequestType { NO_CHANGE, LEFT_CHANGE, RIGHT_CHANGE };

enum LooseBoundType { NONE_SIDE, LEFT_SIDE, RIGHT_SIDE, BOTH_SIDE };

enum class ErrorType {
  kSuccess = 0,
  kWrongStatus = 1,
  kIllegalInput = 2,
  kUnknown = 3
};

enum MarkingLineChangeType {
  NO_TYPE_CHANGE = 0,
  SOLID_TO_DASH = 1,
  DASH_TO_SOLID = 2,
};

// 匝道的方向
enum RampDirection {
  RAMP_NONE = 0,
  RAMP_ON_LEFT = 1,
  RAMP_ON_RIGHT = 2,
  RAMP_ON_MIDDLE = 3,
};

enum SplitDirection {
  SPLIT_NONE = 0,
  SPLIT_LEFT = 1,
  SPLIT_RIGHT = 2,
  SPLIT_MIDDLE = 3,
};

// 拥堵等级枚举
enum CongestionLevel {
  FREE_FLOW = 0,   // 自由流（畅通）
  CONGESTION = 1,  // 拥堵
  JAM = 2,         // 堵塞
};

// 拥堵状态检测结果
struct CongestionResult {
  CongestionLevel level;     // 拥堵等级
  double density;            // 密度（辆/公里）
  double avg_speed;          // 平均速度（m/s）
  double speed_deviation;    // 速度标准差（m/s）
  double avg_time_headway;   // 平均车头时距（秒）
  double avg_space_headway;  // 不安全车头时距比例
};

//分流的相对方向
enum SplitRelativeDirection {
  NO_SPLIT = 0,
  ON_LEFT = 1,
  ON_RIGHT = 2,
};

enum EgoStatusOnRoute {
  ON_MAIN = 0,
  NEARING_SPLIT = 1,
  NEARING_MERGE = 2,
  IN_EXCHANGE_AREA_FRONT = 3,
  IN_EXCHANGE_AREA_REAR = 4,
};

//导航变道请求类型
enum EgoMLCRequestType {
  None_MLC = 0,
  AVOIDE_MERGE = 1,
  AVOIDE_DIVERGE = 2,
  RAMP_TO_MAIN = 3,
  MAIN_TO_RAMP = 4,
  KEEP_LEFT = 5,
  OTHER_TYPE_MLC = 6,
};
struct EgoMLCRequestInfo {
  EgoMLCRequestType mlc_request_type = None_MLC;
  double distance_to_exchange_region = NL_NMAX;

  void reset() {
    mlc_request_type = None_MLC;
    distance_to_exchange_region = NL_NMAX;
  }
};
struct LaneChangeGapInfo {
  int front_node_id = -1;
  int rear_node_id = -1;
};

struct FeasibleLaneInfo {
  int total_lane_num = 0;
  std::vector<int> feasible_lane_sequence;
  FeasibleLaneInfo(int lane_num, const std::vector<int>& sequence)
      : total_lane_num(lane_num), feasible_lane_sequence(sequence) {}
};

struct FPPoint {
  uint64 link_id = 0;
  // 注：由于腾讯地图和百度地图的差异性，在基于百度地图计算的FPPoint信息中，
  // 只对fp_distance_to_split_point进行了有效赋值，其他皆没有赋值
  double fp_distance_to_split_point = 0.0;
  std::vector<uint64> lane_ids;
  std::optional<iflymapdata::sdpro::FeaturePoint> fp;

  FPPoint() = default;

  FPPoint(uint64 _link_id, double _fp_distance_to_split_point,
          const std::vector<uint64>& _lane_ids,
          const std::optional<iflymapdata::sdpro::FeaturePoint>& _fp)
      : link_id(_link_id),
        fp_distance_to_split_point(_fp_distance_to_split_point),
        lane_ids(_lane_ids) {
    if (_fp.has_value()) {
      fp = _fp;
    }
  }

  void reset() {
    link_id = 0;
    fp_distance_to_split_point = 0.0;
    lane_ids.clear();
  }

  bool isEmpty() const {
    return (link_id == 0) &&
           (fp_distance_to_split_point == 0.0) &&
           lane_ids.empty();
  }
  // FPPoint(int link_id, double fp_distance_to_split_point,
  //         std::vector<int> lane_ids)
  //     : link_id(link_id),
  //       fp_distance_to_split_point(fp_distance_to_split_point),
  //       lane_ids(lane_ids) {}
};

enum MergeType {
  NO_MERGE = 0,
  CONTINUE_MERGE = 1,
  LEFT_MERGE = 2,
  RIGHT_MERGE = 3,
  BOTH_MERGE = 4
};

enum MergeLaneType {
  NONE_MERGE = 0,
  MERGE_TO_LEFT = 1,
  MERGE_TO_RIGHT = 2
};

struct MapMergePointInfo {
  double dis_to_merge_fp = NL_NMAX;
  MergeLaneType merge_type = NONE_MERGE;
  int merge_lane_sequence = -1;
  uint64 merge_lane_link_id = 0;

  MapMergePointInfo() = default;

  MapMergePointInfo(double _dis_to_merge_fp, MergeLaneType _merge_type,
                    int _merge_lane_sequence, uint64 _merge_lane_link_id)
      : dis_to_merge_fp(_dis_to_merge_fp),
        merge_type(_merge_type),
        merge_lane_sequence(_merge_lane_sequence),
        merge_lane_link_id(_merge_lane_link_id) {

        };

  void reset() {
    dis_to_merge_fp = NL_NMAX;
    merge_type = NONE_MERGE;
    merge_lane_sequence = -1;
    merge_lane_link_id = -1;
  }
};

//后续把这个命名换成TencentSplitRegionInfo
struct NOASplitRegionInfo {
  bool is_valid = false;
  bool is_ramp_split = false;                  // split场景专用
  bool is_ramp_merge = false;                  // merge场景专用
  bool is_other_merge_to_road = false;         // merge场景专用
  MergeType merge_type = MergeType::NO_MERGE;  // merge场景专用
  int avoide_lane_num = -1;
  uint64 split_link_id = -1;
  double distance_to_split_point = NL_NMAX;
  FPPoint end_fp_point;
  FPPoint start_fp_point;
  SplitDirection split_direction = SplitDirection::SPLIT_NONE;
  std::vector<FeasibleLaneInfo> recommend_lane_num;

  void reset() {
    is_valid = false;
    is_ramp_split = false;
    is_ramp_merge = false;
    is_other_merge_to_road = false;
    merge_type = MergeType::NO_MERGE;
    split_link_id = -1;
    distance_to_split_point = NL_NMAX;
    end_fp_point.reset();
    start_fp_point.reset();
    split_direction = SplitDirection::SPLIT_NONE;
    recommend_lane_num.clear();
  }
  //车道数从左开始数
  // recommond_lane_num{交换区起点前的车道数，交换区内的车道数，交换区终点后route上的车道数，其他方向的车道数}
};

struct MapSplitRegionInfo {
  uint64 split_link_id = -1;
  double distance_to_split_point = NL_NMAX;
  SplitDirection split_direction = SplitDirection::SPLIT_NONE;
  FPPoint start_fp_point;
  FPPoint end_fp_point;
  bool is_ramp_split = false;

  MapSplitRegionInfo() = default;

  MapSplitRegionInfo(uint64 _link_id, double _distance_to_split_point,
                     SplitDirection _split_direction, FPPoint _start_fp_point,
                     FPPoint _end_fp_point, bool _is_ramp_split)
      : split_link_id(_link_id),
        distance_to_split_point(_distance_to_split_point),
        split_direction(_split_direction),
        start_fp_point(_start_fp_point),
        end_fp_point(_end_fp_point),
        is_ramp_split(_is_ramp_split) {}

  void reset() {
    split_link_id = -1;
    distance_to_split_point = NL_NMAX;
    split_direction = SplitDirection::SPLIT_NONE;
    start_fp_point.reset();
    end_fp_point.reset();
    is_ramp_split = false;
  }
};

struct MapMergeRegionInfo {
  uint64 merge_link_id = -1;
  double distance_to_merge_point = NL_NMAX;
  RampDirection merge_direction = RAMP_NONE;

  MapMergeRegionInfo() = default;

  MapMergeRegionInfo(uint64 link_id, double distance_to_merge_point,
                     RampDirection merge_direction)
      : merge_link_id(link_id),
        distance_to_merge_point(distance_to_merge_point),
        merge_direction(merge_direction) {}

  void reset() {
    merge_link_id = -1;
    distance_to_merge_point = NL_NMAX;
    merge_direction = RAMP_NONE;
  }
};

// struct EgoStatusOnRoute {
//   bool is_nearing_split_region = false;
//   bool is_nearing_merge_redion = false;
//   bool
// }
struct MLCDeciderRouteInfo {
  EgoStatusOnRoute ego_status_on_route = ON_MAIN;
  bool is_process_split = false;
  bool is_process_split_split = false;
  bool is_process_other_merge_split = false;
  bool is_process_other_merge = false;
  bool is_process_merge = false;
  double last_frame_dis_to_merge_start_point = NL_NMAX;
  double last_frame_dis_to_merge_point = NL_NMAX;
  double last_frame_dis_to_split = NL_NMAX;
  double end_fp_dis_to_split = 0;
  bool is_triggle_cal_dis_to_last_merge_point = false;
  NOASplitRegionInfo static_merge_region_info;
  NOASplitRegionInfo static_split_region_info;
  NOASplitRegionInfo first_static_split_region_info;
  std::vector<int> feasible_lane_sequence;

  void reset() {
    ego_status_on_route = ON_MAIN;
    is_process_split = false;
    is_process_split_split = false;
    is_process_other_merge_split = false;
    is_process_other_merge = false;
    is_process_merge = false;
    last_frame_dis_to_merge_start_point = NL_NMAX;
    last_frame_dis_to_merge_point = NL_NMAX;
    last_frame_dis_to_split = NL_NMAX;
    end_fp_dis_to_split = 0;
    is_triggle_cal_dis_to_last_merge_point = false;
    static_merge_region_info.reset();
    static_split_region_info.reset();
    first_static_split_region_info.reset();
    feasible_lane_sequence.clear();
  }
};

enum MLCSceneType {
  NONE_SCENE = 0,
  NORMAL_SCENE = 1,
  SPLIT_SCENE = 2,
  MERGE_SCENE = 3,
  AVOID_MERGE = 4,
  AVOID_SPLIT = 5
};
struct MLCDeciderSceneTypeInfo {
  MLCSceneType mlc_scene_type = NONE_SCENE;
  RampDirection route_lane_direction = RAMP_NONE;
  double dis_to_link_topo_change_point = 0.0;
  uint64 topo_change_link_id = 0;
  bool is_scene_info_valid = false;

  MLCDeciderSceneTypeInfo() = default;

  MLCDeciderSceneTypeInfo(MLCSceneType mlc_scene_type,
                               RampDirection route_lane_direction,
                               double dis_to_link_topo_change_point,
                               uint64 topo_change_link_id)
      : mlc_scene_type(mlc_scene_type),
        route_lane_direction(route_lane_direction),
        dis_to_link_topo_change_point(dis_to_link_topo_change_point),
        topo_change_link_id(topo_change_link_id) {
    is_scene_info_valid = info_valid(mlc_scene_type, route_lane_direction,
                          dis_to_link_topo_change_point);
  }

  void set_value(MLCSceneType scene_type, RampDirection lane_direction,
                 double dis, uint64 link_id) {
    mlc_scene_type = scene_type;
    route_lane_direction = lane_direction;
    dis_to_link_topo_change_point = dis;
    topo_change_link_id = link_id;
    is_scene_info_valid = info_valid(scene_type, lane_direction, dis);
  }

  void reset() {
    mlc_scene_type = NONE_SCENE;
    route_lane_direction = RAMP_NONE;
    dis_to_link_topo_change_point = 0.0;
    is_scene_info_valid = false;
    topo_change_link_id = 0;
  }

 private:
  bool info_valid(MLCSceneType mlc_scene_type,
                  RampDirection route_lane_direction,
                  double dis_to_link_topo_change_point) {
    if (mlc_scene_type != NONE_SCENE && route_lane_direction != RAMP_NONE &&
        dis_to_link_topo_change_point > 0.0) {
      return true;
    } else {
      return false;
    }
  }
};
struct HPPRouteInfoOutput {
  bool is_on_hpp_lane = false;
  bool is_reached_hpp_start_point = false;
  double sum_distance_driving = -1;
  double distance_to_target_slot = NL_NMAX;   // 自车当前位置到 HPP 巡航目标车位的距离
  double distance_to_target_dest = NL_NMAX;   // 自车当前位置到 HPP 巡航目标终点的距离
  double distance_to_next_speed_bump = NL_NMAX;
  ad_common::math::Vec2d target_dest_point;   // HPP 巡航目标终点

  void reset() {
    is_on_hpp_lane = false;
    is_reached_hpp_start_point = false;
    sum_distance_driving = -1;
    distance_to_target_slot = NL_NMAX;
    distance_to_target_dest = NL_NMAX;
    distance_to_next_speed_bump = NL_NMAX;
  }
};

struct GaodeRouteInfoOutput {
  int split_seg_forward_lane_nums = 0;
  int split_next_seg_forward_lane_nums = 0;
  int lc_nums_for_split = 0;
  int need_continue_lc_num_on_off_ramp_region = 0;
  int lane_num_except_emergency = 0;
  int merge_seg_forward_lane_nums = 0;
  int merge_last_seg_forward_lane_nums = 0;
  bool is_leaving_ramp = false;
  bool is_ramp_merge_to_road_on_expressway = false;
  bool is_road_merged_by_other_lane = false;
  bool is_ramp_merge_to_ramp_on_expressway = false;
  bool is_nearing_other_lane_merge_to_road_point = false;
  bool is_nearing_ramp = false;
  bool is_ego_on_split_region = false;
  double distance_to_second_road_merge = NL_NMAX;
  RampDirection second_merge_direction = RampDirection::RAMP_NONE;
  RampDirection last_split_seg_dir = RAMP_NONE;

  void reset() {
    split_seg_forward_lane_nums = 0;
    split_next_seg_forward_lane_nums = 0;
    lc_nums_for_split = 0;
    need_continue_lc_num_on_off_ramp_region = 0;
    lane_num_except_emergency = 0;
    merge_seg_forward_lane_nums = 0;
    merge_last_seg_forward_lane_nums = 0;
    is_leaving_ramp = false;
    is_ramp_merge_to_road_on_expressway = false;
    is_road_merged_by_other_lane = false;
    is_ramp_merge_to_ramp_on_expressway = false;
    is_nearing_other_lane_merge_to_road_point = false;
    is_nearing_ramp = false;
    is_ego_on_split_region = false;
    distance_to_second_road_merge = NL_NMAX;
    second_merge_direction = RampDirection::RAMP_NONE;
    last_split_seg_dir = RAMP_NONE;
  }
};
struct RouteInfoOutput {
  double lsl_length = 0.0;// 公供里面optional，注意汉杰调用的地方需要放在merge point里面输出给他使用
  std::vector<int> feasible_lane_sequence;
  MLCDeciderSceneTypeInfo mlc_decider_scene_type_info;
  // for NOA output
  // int split_seg_forward_lane_nums = 0;
  // int split_next_seg_forward_lane_nums = 0;
  // int lc_nums_for_split = 0;
  int cur_seg_forward_lane_num = 0;
  // int need_continue_lc_num_on_off_ramp_region = 0;
  // int lane_num_except_emergency = 0;
  // int merge_seg_forward_lane_nums = 0;
  // int merge_last_seg_forward_lane_nums = 0;
  int left_lane_num = 0;
  int right_lane_num = 0;
  int emergency_lane_num = 0;
  // int minVal_seq = 0; // 用feasible_lane_sequence来代替
  // int maxVal_seq = 0; // 用feasible_lane_sequence来代替
  int ego_seq = 0; // 表示自车seq的，可以根据左右车道数计算出来，需要考虑这个还要不要输出
  bool is_update_segment_success = false;
  bool is_on_ramp = false;
  bool is_in_sdmaproad = false;
  bool is_ego_on_expressway = false;
  // bool is_leaving_ramp = false;
  bool is_ego_on_city_expressway_hmi = false;
  bool is_ego_on_expressway_hmi = false;
  bool is_exist_toll_station = false;
  // bool is_ramp_merge_to_road_on_expressway = false;
  // bool is_road_merged_by_other_lane = false;
  // bool is_ramp_merge_to_ramp_on_expressway = false;
  // bool is_nearing_other_lane_merge_to_road_point = false;
  // bool is_on_highway = false;
  bool is_continuous_ramp = false;  // for jwliu23
  // bool is_nearing_ramp = false;
  // bool is_ego_on_split_region = false;
  // bool is_find_exc_fp = false;
  // bool is_miss_split_point = false;
  // 提供给纵向模块使用的，修改为纵向模块直接通过route_info_output计算得到
  // bool is_closing_merge = false; // 这个状态是怎么定义的？？？
  // bool is_closing_split = false; // 这个状态是怎么定义的？？？
  double dis_to_ramp = NL_NMAX; // 在E541中需要注意这个值是否与处理的ramp对齐
  double distance_to_first_road_merge = NL_NMAX;
  double distance_to_first_road_split = NL_NMAX;
  // // double distance_to_second_road_merge = NL_NMAX;
  // double distance_to_second_road_split = NL_NMAX;
  double distance_to_route_end = NL_NMAX;
  double sum_dis_to_last_link_merge_point = NL_NMAX;
  double sum_dis_to_last_link_split_point = NL_NMAX;
  double sum_dis_to_last_split_exchange_area_end_point = NL_NMAX;
  // double accumulate_dis_ego_to_last_split_point = NL_NMAX;
  // double sum_dis_to_last_split_point_on_ramp = NL_NMAX;
  double distance_to_toll_station = NL_NMAX;
  // double distance_to_exchange_area_end_point = -NL_NMAX;
  // double lsl_length = 0.0;
  double current_segment_passed_distance = 0.0;  // for xykuai

  // （for wangzhi17）目前只看到这两个对外只用作可视化信息，用virtual lane中的信息去更新，就不对外输出了
  // double left_lane_distance = 0.0; // 这个更新的方式没看懂
  // double right_lane_distance = 0.0; // 这个更新的方式没看懂


  // std::pair<SplitRelativeDirection, double>
  //     first_split_dir_dis_info;  // for xykuai
  // std::vector<std::pair<SplitRelativeDirection, double>>
  //     split_dir_dis_info_list;  // for xykuai


  RampDirection ramp_direction = RampDirection::RAMP_NONE;
  RampDirection first_split_direction = RampDirection::RAMP_NONE;
  RampDirection first_merge_direction = RampDirection::RAMP_NONE;
  // RampDirection second_split_direction = RampDirection::RAMP_NONE;
  // RampDirection second_merge_direction = RampDirection::RAMP_NONE;
  // RampDirection other_lane_merge_dir = RampDirection::RAMP_NONE;
  // RampDirection last_split_seg_dir = RAMP_NONE;
  // NOASplitRegionInfo ramp_region_info;

  // 重点解决！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
  // std::vector<NOASplitRegionInfo> split_region_info_list;// 已经完成第一版的翻译
  std::vector<MapSplitRegionInfo> map_split_region_info_list;

  // std::vector<NOASplitRegionInfo> merge_region_info_list; // 不需要输出这里面的fp，可以重新设计一个struct

  std::vector<MapMergeRegionInfo> map_merge_region_info_list;
  // EgoMLCRequestInfo mlc_request_type_route_info;
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  // EgoStatusOnRoute ego_status_on_route = EgoStatusOnRoute::ON_MAIN;

  iflymapdata::sdpro::MapVendorType map_vendor =
      iflymapdata::sdpro::MapVendorType::MAP_VENDOR_NONE;
  // MLCDeciderRouteInfo mlc_decider_route_info;
  // double dis_to_merge_fp = NL_NMAX;
  std::vector<MapMergePointInfo> map_merge_points_info; // 在virtual lane中 赋在车道线上输出给下游使用
  // MLCSceneType baidu_mlc_scene = NORMAL_SCENE;

  HPPRouteInfoOutput hpp_route_info_output;
  GaodeRouteInfoOutput gaode_route_info_output;

  void reset() {
    is_update_segment_success = false;
    is_on_ramp = false;
    dis_to_ramp = NL_NMAX;
    ramp_direction = RampDirection::RAMP_NONE;
    distance_to_first_road_merge = NL_NMAX;
    distance_to_first_road_split = NL_NMAX;
    distance_to_route_end = NL_NMAX;
    is_in_sdmaproad = false;
    is_ego_on_expressway = false;
    first_split_direction = RampDirection::RAMP_NONE;
    first_merge_direction = RampDirection::RAMP_NONE;
    sum_dis_to_last_link_merge_point = NL_NMAX;
    sum_dis_to_last_link_split_point = NL_NMAX;
    sum_dis_to_last_split_exchange_area_end_point = NL_NMAX;
    distance_to_toll_station = NL_NMAX;
    is_ego_on_city_expressway_hmi = false;
    is_ego_on_expressway_hmi = false;
    is_exist_toll_station = false;
    // is_on_highway = false;
    is_continuous_ramp = false;
    // first_split_dir_dis_info = std::make_pair(None, NL_NMAX);
    // split_dir_dis_info_list.clear();
    current_segment_passed_distance = 0.0;
    cur_seg_forward_lane_num = 0;
    map_split_region_info_list.clear();
    map_merge_region_info_list.clear();
    map_vendor = iflymapdata::sdpro::MapVendorType::MAP_VENDOR_NONE;
    map_merge_points_info.clear();
    left_lane_num = 0;
    right_lane_num = 0;
    ego_seq = 0;
    emergency_lane_num = 0;
    lsl_length = 0.0;
    // mlc_request_type_route_info.reset();
    // is_closing_merge = false;
    // is_closing_split = false;

    hpp_route_info_output.reset();
    gaode_route_info_output.reset();
    feasible_lane_sequence.clear();
    mlc_decider_scene_type_info.reset();
  }
};

// 转向灯拨杆
enum LeverStatus {
  LEVER_STATE_OFF,
  LEVER_STATE_LEFT,
  LEVER_STATE_RIGHT,
  LEVER_STATE_RESERVED1,
  LEVER_STATE_RESERVED2
};

typedef enum {
  UNKNOWN_POS = -100,
  LEFT_LEFT_POS = -2,
  LEFT_POS = -1,
  CURR_POS = 0,
  RIGHT_POS = 1,
  RIGHT_RIGHT_POS = 2
} LanePosition;
enum RequestSource {
  NO_REQUEST,
  INT_REQUEST,
  MAP_REQUEST,
  ACT_REQUEST,
  ROUTE_REQUEST,
  OVERTAKE_REQUEST,
  EMERGENCE_AVOID_REQUEST,
  CONE_REQUEST,
  MERGE_REQUEST
};

struct PointLLH {
  double Longitude;  // Longitude in degrees, ranging from -180 to 180,(度)
  double Latitude;   // Latitude in degrees, ranging from -90 to 90,(度)
  double height;     // WGS-84 ellipsoid height in meters,(米)
};

struct EulerAngle {
  double yaw;
  double pitch;
  double roll;
};
enum GapDriveStyle {
  NONESTYLE = 1,
  OnlyRearFasterCar = 2,
  OnlyRearSlowerCar = 3,
  RearCarFaster = 4,
  FrontCarFaster = 5,
  NoDecisionForBothCar = 6,
  OnlyFrontFasterCar = 7,
  OnlyFrontSlowerCar = 8,

  Free = 9,
};

struct CrossedLinePointInfo {
  Point2D crossed_line_point;
  bool valid = false;
};
struct GapSelectorPathSpline {
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  LonState start_state{0., 0., 0., 0., 0.};
  Point2D start_cart_point{0., 0.};
  Point2D start_frenet_point{0., 0.};
  Point2D quintic_p0{0., 0.};
  Point2D quintic_pe{0., 0.};
  Point2D stitched_p{0., 0.};
  CrossedLinePointInfo crossed_line_point_info{Point2D{0., 0.}, false};
  enum PathSplineStatus {
    NO_VALID = 0,
    LC_VALID = 1,
    LH_VALID = 2,
    LB_VALID = 3,
    LC_LANE_CROSS = 4,
    LC_PREMOVE = 5,
  };
  int path_spline_status{
      NO_VALID};  // 0--NO_VALID, 1--LC_VALID, 2--LH_VALID, 3--LB_VALID
};
typedef enum {
  BOTH_AVAILABLE = 0,
  LEFT_AVAILABLE,
  RIGHT_AVAILABLE,
  BOTH_MISSING
} LaneStatusEx;

enum FusionSource { CAMERA_ONLY = 1, RADAR_ONLY, FUSION };
typedef enum { LEFT = 0, RIGHT } LineDirection;
struct TrackInfo {
  TrackInfo() {}

  TrackInfo(int id, double drel, double vrel)
      : track_id(id), d_rel(drel), v_rel(vrel) {}

  TrackInfo(const TrackInfo &track_info) {
    track_id = track_info.track_id;
    d_rel = track_info.d_rel;
    v_rel = track_info.v_rel;
  }

  TrackInfo &operator=(const TrackInfo &track_info) {
    track_id = track_info.track_id;
    d_rel = track_info.d_rel;
    v_rel = track_info.v_rel;
    return *this;
  }

  void set_value(int id, double drel, double vrel) {
    track_id = id;
    d_rel = drel;
    v_rel = vrel;
  }

  void reset() {
    track_id = -10000;
    d_rel = 0.0;
    v_rel = 0.0;
  }

  int track_id = -10000;
  double d_rel = 0.0;
  double v_rel = 0.0;
};

typedef enum {
  NO_REJECTION,
  BIAS_L,
  BIAS_R,
  WIDE_REJECTION_L,
  WIDE_REJECTION_R,
  SHORT_REJECTION,
  NARROW_REJECTION
} RejectReason;

struct LatBehaviorInfo {
  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;
  bool flag_avd;
  int ncar_change;
  int avd_back_cnt;
  int avd_leadone;
  int pre_leadone_id;
  double dist_rblane;
  double final_y_rel;
};

enum ObstacleIntentType { COMMON = 0, CUT_IN = 1 };
struct VirtualPoint2D {
  double x = 0.0;
  double y = 0.0;

  VirtualPoint2D() = default;
  VirtualPoint2D(double xx, double yy) : x(xx), y(yy) {}
};

struct VirtualPoint3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  VirtualPoint3D() = default;
  VirtualPoint3D(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct TrajectoryPoint {
  // enu
  double x = 0;
  double y = 0;
  double heading_angle = 0;
  double curvature = 0;
  double dkappa = 0;
  double ddkappa = 0;
  double t = 0;
  double v = 0;
  double a = 0;
  double jerk = 0;
  double delta = 0.;

  // frenet
  double s = 0;
  double l = 0;
  bool frenet_valid = false;
};
using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct ObstaclePredicatedPoint {
  double x = 0.;
  double y = 0.;
  double heading_angle = 0.;
  double t = 0.;
  double s = 0.;
  double l = 0.;
};
using ObstaclePredicatedPoints = std::vector<ObstaclePredicatedPoint>;
struct ObstaclePredicatedInfo {
  ObstaclePredicatedPoints obstacle_pred_info;
  ~ObstaclePredicatedInfo() = default;
  // perception info
  double origin_x = 0.;
  double origin_y = 0.;
  double origin_heading_angle = 0.;
  double raw_vel = 0.;
  double cur_s = 0.;
  double cur_l = 0.;

  double length = 5.;
  double width = 2.1;
  std::vector<double> x_vec{};
  std::vector<double> y_vec{};
  std::vector<double> heading_angle_vec{};
};
// enum ScenarioStateEnum {
//   ROAD_NONE = 0,
//   ROAD_LC_LWAIT,
//   ROAD_LC_RWAIT,
//   ROAD_LC_LCHANGE,
//   ROAD_LC_RCHANGE,
//   ROAD_LC_LBACK,
//   ROAD_LC_RBACK,

//   ROAD_LB_LBORROW,
//   ROAD_LB_RBORROW,
//   ROAD_LB_LBACK,
//   ROAD_LB_RBACK,
//   ROAD_LB_LRETURN,
//   ROAD_LB_RRETURN,
//   ROAD_LB_LSUSPEND,
//   ROAD_LB_RSUSPEND,

//   INTER_GS_NONE,
//   INTER_GS_LC_LWAIT,
//   INTER_GS_LC_RWAIT,
//   INTER_GS_LC_LCHANGE,
//   INTER_GS_LC_RCHANGE,
//   INTER_GS_LC_LBACK,
//   INTER_GS_LC_RBACK,

//   INTER_GS_LB_LBORROW,
//   INTER_GS_LB_RBORROW,
//   INTER_GS_LB_LBACK,
//   INTER_GS_LB_RBACK,
//   INTER_GS_LB_LRETURN,
//   INTER_GS_LB_RRETURN,
//   INTER_GS_LB_LSUSPEND,
//   INTER_GS_LB_RSUSPEND,

//   INTER_TR_NONE,
//   INTER_TR_LC_LWAIT,
//   INTER_TR_LC_RWAIT,
//   INTER_TR_LC_LCHANGE,
//   INTER_TR_LC_RCHANGE,
//   INTER_TR_LC_LBACK,
//   INTER_TR_LC_RBACK,

//   INTER_TR_LB_LBORROW,
//   INTER_TR_LB_RBORROW,
//   INTER_TR_LB_LBACK,
//   INTER_TR_LB_RBACK,
//   INTER_TR_LB_LRETURN,
//   INTER_TR_LB_RRETURN,
//   INTER_TR_LB_LSUSPEND,
//   INTER_TR_LB_RSUSPEND,

//   INTER_TL_NONE,
//   INTER_TL_LC_LWAIT,
//   INTER_TL_LC_RWAIT,
//   INTER_TL_LC_LCHANGE,
//   INTER_TL_LC_RCHANGE,
//   INTER_TL_LC_LBACK,
//   INTER_TL_LC_RBACK,

//   INTER_TL_LB_LBORROW,
//   INTER_TL_LB_RBORROW,
//   INTER_TL_LB_LBACK,
//   INTER_TL_LB_RBACK,
//   INTER_TL_LB_LRETURN,
//   INTER_TL_LB_RRETURN,
//   INTER_TL_LB_LSUSPEND,
//   INTER_TL_LB_RSUSPEND,

//   INTER_UT_NONE
// };

enum StateMachineLaneChangeStatus {
  kLaneKeeping = 0,
  kLaneChangePropose,
  kLaneChangeExecution,
  kLaneChangeComplete,
  kLaneChangeCancel,
  kLaneChangeHold
};

enum MergeDirection {
  NONE_LANE_MERGE = 0,
  CUR_LANE_MERGE_TO_LEFT = 1,
  CUR_LANE_MERGE_TO_RIGHT = 2,
  RIGHT_LANE_MERGE_TO_CUR_LANE = 3,
  LEFT_LANE_MERGE_TO_CUR_LANE = 4,
};

enum ScenarioEnum { SCENARIO_CRUISE = 0, SCENARIO_LOW_SPEED };

enum FaultDiagnosisType {
  OFF_ROUTE = 0,
  OFF_MAP,
  LATERAL_DIFF,
  LINE_PRESSING_DRIVING,
  BORDER_PRESSING_DRIVING,
  LOCALIZATION_FAULT,
  CONTROL_FAULT
};

struct EgoPredictionTrajectory {
  TrajectoryPoints trajectory;
  double logit = 0;
  std::vector<std::uint64_t> track_ids;
};
using EgoPredictionTrajectorys = std::vector<EgoPredictionTrajectory>;

struct EgoPredictionObject {
  double timestamp_us = 0;
  double timestamp = 0;
  bool ego_prediction_valid = false;
  std::vector<EgoPredictionTrajectory> trajectory_array;
};

enum ResultTrajectoryType { REFERENCE_PATH = 0, RAW_TRAJ, REFINED_TRAJ };

struct CurvatureInfo {
  double max_curvature = 0.0;
  double curv_velocity_limit = 120.0 / 3.6;
};

struct LatDecisionInfo {
  int direction = 0;
  int id = 0;
  int type = 0;
};
using LatDecisionInfos = std::unordered_map<int, LatDecisionInfo>;

struct FaultDiagnosisInfo {
  bool able_to_auto{true};
  bool pre_able_to_auto{false};
  int fault_type = -1;
  double pre_x{0.0};
  double pre_y{0.0};
  bool localization_fault_state{false};
  double localization_fault_duration{0.0};
  bool localization_fault_disable{false};
};

struct AccInfo {
  bool enable_jerk_up{false};
  bool enable_jerk_down{false};
  bool enable_v_cost{false};
  bool enable_v_limit{false};
  double s_ref{1000.0};
  double s_bound{1000.0};
  double v_ref{1000.0};
  double v_bound{1000.0};
};

struct AdaptiveCruiseControlInfo {
  bool navi_speed_flag{false};
  bool navi_time_distance_flag{false};
  AccInfo navi_speed_control_info;
  AccInfo navi_time_distance_info;
};

struct MotionPlanningInfo {
  bool lat_enable_flag = false;
  bool lon_enable_flag = false;
  bool lat_init_flag = false;
  pnc::mathlib::spline ref_x_s_spline;
  pnc::mathlib::spline ref_y_s_spline;
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline theta_s_spline;
  pnc::mathlib::spline delta_s_spline;
  pnc::mathlib::spline omega_s_spline;
  pnc::mathlib::spline curv_s_spline;
  pnc::mathlib::spline d_curv_s_spline;

  pnc::mathlib::spline lateral_x_t_spline;
  pnc::mathlib::spline lateral_y_t_spline;
  pnc::mathlib::spline lateral_theta_t_spline;

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline v_t_spline;
  pnc::mathlib::spline a_t_spline;
  pnc::mathlib::spline j_t_spline;

  pnc::mathlib::spline x_t_spline;
  pnc::mathlib::spline y_t_spline;

  pnc::mathlib::spline ref_x_t_spline;
  pnc::mathlib::spline ref_y_t_spline;

  pnc::mathlib::spline theta_t_spline;
  pnc::mathlib::spline delta_t_spline;
  pnc::mathlib::spline omega_t_spline;

  std::vector<double> s_lat_vec;

  ilqr_solver::ControlVec u_vec;
};
struct GapSelectorInfo {
  bool lane_cross = false;
  bool lc_triggered = false;
  bool lb_triggered = false;
  bool lc_in = false;
  bool lb_in = false;
  double lc_pass_time = 0.;
  double lc_wait_time = 0.;
  bool path_requintic = false;
  bool lc_cancel = false;
  bool gs_skip = false;
  int lc_request = 0;
  GapSelectorPathSpline last_gap_selector_path_spline;
};
struct PlanningResult {
  planning::common::SceneType scene_type;
  int target_lane_id;
  // ScenarioStateEnum target_scenario_state = ROAD_NONE;
  TrajectoryPoints traj_points;
  TrajectoryPoints raw_traj_points;
  // MotionPlanningInfo motion_planning_info; // TODO: 从PlanningResult移出去
  RequestType turn_signal = NO_CHANGE;
  // CurvatureInfo curvature_info;
  int use_backup_cnt = 0;
  double timestamp = 0.0;
  // bool use_refined_reference_path = false;
  // int contingency_trigger_index = 0;
  // bool gap_selector_trustworthy = false;
  // std::string extra_json;
  mjson::Json extra_json = mjson::Json(mjson::Json::object());
  void Clear() {
    raw_traj_points.clear();
    traj_points.clear();
    extra_json = mjson::Json(mjson::Json::object());
  }
};

struct PlanningInitPoint {
  double x;
  double y;
  double heading_angle;
  double curvature;
  double dkappa;
  double v;
  double a;
  double jerk;
  double relative_time;
  double lon_pos_err;
  double delta;
  FrenetState frenet_state;

  planning::common::LateralInitState lat_init_state;
  planning::common::LongitudinalInitState lon_init_state;
};

class ReferencePath;

struct CarReferenceInfo {
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline k_s_spline;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> k_vec;
  std::vector<double> s_vec;
};
struct CoarsePlanningInfo {
  StateMachineLaneChangeStatus source_state;
  StateMachineLaneChangeStatus target_state;
  RequestSource lane_change_request_source;
  int source_lane_id;
  int target_lane_id;
  bool bind_end_state;
  int int_lane_change_cmd;
  std::shared_ptr<ReferencePath> reference_path = nullptr;
  TrajectoryPoints trajectory_points;
  // overtake_obstacles and yield_obstacles are used only under wait state
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  CarReferenceInfo cart_ref_info;
};

struct FrenetBoundary {
  double s_start;
  double s_end;
  double l_start;
  double l_end;
};

struct FrenetBoundaryCorners {
  double s_front_left;
  double l_front_left;
  double s_front_right;
  double l_front_right;
  double s_rear_left;
  double l_rear_left;
  double s_rear_right;
  double l_rear_right;
};

struct FrenetObstacleBoundary {
  double s_start{std::numeric_limits<double>::max()};
  double s_end{std::numeric_limits<double>::lowest()};
  double l_start{std::numeric_limits<double>::max()};
  double l_end{std::numeric_limits<double>::lowest()};
};

enum TrackType { SIDE_TRACK, FRONT_TRACK };

enum CurrentState {
  INIT = 0,
  LANE_KEEPING,
  APPROACH_STOPLINE_SLOW,
  PASS_INTERSECTION,
  RED_LIGHT_STOP,
  COVER_LIGHT,
  INTO_WAIT_ZONE
};

typedef enum { NORMAL_ROAD = 1, INTERSECT = 2 } AlgorithmScene;
typedef enum {
  LANE_CHANGE_LEFT = 1,
  LANE_CHANGE_RIGHT = 2,
  LANE_BORROW_LEFT = 4,
  LANE_BORROW_RIGHT = 8,
  INTERSECT_GO_STRAIGHT = 16,
  INTERSECT_TURN_LEFT = 32,
  INTERSECT_TURN_RIGHT = 64,
  INTERSECT_U_TURN = 128,
  LANE_BORROW_IN_NON_MOTORIZED_LANE = 256
} AlgorithmAction;
typedef enum {
  LANE_CHANGE_WAITING = 1,
  LANE_CHANGEING = 2,
  LANE_CHANGE_BACK = 4,
  LANE_BORROWING = 8,
  LANE_BORROW_BACK = 16,
  LANE_BORROW_RETURN = 32,
  LANE_BORROW_SUSPEND = 64
} AlgorithmStatus;
struct LatBehaviorStateMachineOutput {
  int scenario;
  int curr_state;
  int fix_lane_virtual_id;
  int origin_lane_virtual_id;
  int target_lane_virtual_id;
  std::string state_name;
  std::string lc_back_reason = "none";
  std::string lc_invalid_reason = "none";

  int turn_light;
  int map_turn_light;

  bool accident_back;
  bool accident_ahead;
  bool close_to_accident;
  bool should_premove;
  bool should_suspend;
  bool must_change_lane;

  int lc_request;
  int lc_request_source;
  int lc_turn_light;
  std::string act_request_source;

  TrackInfo lc_invalid_track;
  TrackInfo lc_back_track;

  bool is_lc_valid;
  int lc_valid_cnt;
  int lc_back_cnt;
  std::string lc_back_invalid_reason;
};

enum class LatObstacleType { LANE, ROAD, CAR };
struct LatDeciderOutput {
  planning::common::LateralInitState init_state;

  std::vector<std::pair<double, double>> enu_ref_path;  // <x, y>
  std::vector<std::pair<double, double>>
      last_enu_ref_path;  // pass it by planning context
  std::vector<std::pair<Point2D, Point2D>> path_bounds;  // <lower ,upper >
  std::vector<std::pair<Point2D, Point2D>> safe_bounds;  // <lower ,upper >
  std::vector<double> enu_ref_theta;
  std::vector<double> last_enu_ref_theta;
  double v_cruise;
  bool complete_follow = true;
  bool lane_change_scene = false;
};

struct LateralMotionPlanningOutput {
  std::vector<double> time_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> delta_vec;
  std::vector<double> omega_vec;
  std::vector<double> omega_dot_vec;
  std::vector<double> acc_vec;
  std::vector<double> jerk_vec;
  // const ilqr_solver::iLqr::iLqrSolverInfo *solver_info_ptr;
  // ilqr_solver::iLqr::iLqrSolverInfo solver_info;  // to be removed
};

struct CutinAgentTtcInfo {
  int32_t agent_id = -1;
  double ttc = -1.0;
};

struct AgentLongitudinalDeciderOutput {
  std::vector<int32_t> cutin_agent_ids;
  CutinAgentTtcInfo closest_cutin_ttc_info;
};
}  // namespace planning
