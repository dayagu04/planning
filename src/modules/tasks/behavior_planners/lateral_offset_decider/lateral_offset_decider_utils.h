#pragma once
#include <variant>
#include "lateral_obstacle.h"
#include "utils/hysteresis_decision.h"

constexpr double MAX_T_EXCEED_AVD_CAR = 6.0;
constexpr double kSafeDistance = 1.0;
constexpr double kDefaultLimitLateralDistance = 10.0;
constexpr int kDefaultLimitId = -1000;
constexpr int kDefaultSafeDistance = -1000;

namespace planning {
enum AvoidObstacleFlag {
  INVALID = kDefaultLimitId,
  LEAD_ONE = -1,
  NORMAL = 0,
  SIDE = 1
};
enum AvoidObstacleUpdateFlag { Update = 1, Past = 2 };
struct AvoidObstacleInfo {
  AvoidObstacleInfo() { Reset(); }
  AvoidObstacleInfo(AvoidObstacleFlag i_flag, double i_intersection_index,
                    double i_vs, double i_vs_lon_relative,
                    double i_predict_vs_lon_relative, double i_s_to_ego,
                    double i_tail_s_to_ego, double i_vs_lat_relative,
                    double i_min_l_to_ref, double i_max_l_to_ref,
                    double i_first_s_to_ego, double i_curr_time,
                    double i_t_exceed_avd_obstacle,
                    double i_allow_max_opposite_offset, uint i_track_id,
                    int i_type, AvoidObstacleUpdateFlag i_update_flag,
                    double i_length, int i_num_out_avd_area,
                    bool i_is_passive) {
    flag = i_flag;
    intersection_index = i_intersection_index;
    vs = i_vs;
    vs_lon_relative = i_vs_lon_relative;
    predict_vs_lon_relative = i_predict_vs_lon_relative;
    s_to_ego = i_s_to_ego;
    tail_s_to_ego = i_tail_s_to_ego;
    vs_lat_relative = i_vs_lat_relative;
    min_l_to_ref = i_min_l_to_ref;
    max_l_to_ref = i_max_l_to_ref;
    first_s_to_ego = i_first_s_to_ego;
    curr_time = i_curr_time;
    t_exceed_avd_obstacle = i_t_exceed_avd_obstacle;
    allow_max_opposite_offset = i_allow_max_opposite_offset;
    track_id = i_track_id;
    type = i_type;
    update_flag = i_update_flag;
    length = i_length;
    num_out_avd_area = i_num_out_avd_area;
    is_passive = i_is_passive;
  }

  void Assign(AvoidObstacleFlag i_flag, double i_intersection_index,
              double i_vs, double i_vs_lon_relative,
              double i_predict_vs_lon_relative, double i_s_to_ego,
              double i_tail_s_to_ego, double i_vs_lat_relative,
              double i_min_l_to_ref, double i_max_l_to_ref,
              double i_first_s_to_ego, double i_curr_time,
              double i_t_exceed_avd_obstacle,
              double i_allow_max_opposite_offset, uint i_track_id, int i_type,
              AvoidObstacleUpdateFlag i_update_flag, double i_length,
              int i_num_out_avd_area, bool i_is_passive) {
    flag = i_flag;
    intersection_index = i_intersection_index;
    vs = i_vs;
    vs_lon_relative = i_vs_lon_relative;
    predict_vs_lon_relative = i_predict_vs_lon_relative;
    s_to_ego = i_s_to_ego;
    tail_s_to_ego = i_tail_s_to_ego;
    vs_lat_relative = i_vs_lat_relative;
    min_l_to_ref = i_min_l_to_ref;
    max_l_to_ref = i_max_l_to_ref;
    first_s_to_ego = i_first_s_to_ego;
    curr_time = i_curr_time;
    t_exceed_avd_obstacle = i_t_exceed_avd_obstacle;
    allow_max_opposite_offset = i_allow_max_opposite_offset;
    track_id = i_track_id;
    type = i_type;
    update_flag = i_update_flag;
    length = i_length;
    num_out_avd_area = i_num_out_avd_area;
    is_passive = i_is_passive;
  }

  void Reset() {
    flag = AvoidObstacleFlag::INVALID;
    intersection_index = 0;
    vs = 0;
    vs_lon_relative = 0;
    predict_vs_lon_relative = 0;
    s_to_ego = 0;
    tail_s_to_ego = 0;
    vs_lat_relative = 0;
    min_l_to_ref = 0;
    max_l_to_ref = 0;
    first_s_to_ego = 0;  //第一次识别成avoid_car的时候的距离
    curr_time = 0;
    t_exceed_avd_obstacle = MAX_T_EXCEED_AVD_CAR;
    allow_max_opposite_offset = 0;
    track_id = 0;
    type = 0;
    update_flag = AvoidObstacleUpdateFlag::Update;
    length = 0;
    num_out_avd_area = 0;
    is_passive = false;
  };
  int flag = AvoidObstacleFlag::INVALID;
  double intersection_index;
  double vs;
  double vs_lon_relative;
  double predict_vs_lon_relative;  //根据自车的速度变化，预测障碍物相对速度
  double s_to_ego;
  double tail_s_to_ego;
  double vs_lat_relative;
  double min_l_to_ref;
  double max_l_to_ref;
  double first_s_to_ego;         //第一次识别成avoid_car的时候的距离
  double curr_time;              //单位s
  double t_exceed_avd_obstacle;  //单位s
  double allow_max_opposite_offset;
  uint track_id = 0;
  int type;
  int update_flag = AvoidObstacleUpdateFlag::Update;
  double length;
  int num_out_avd_area;  //出避让区域的次数
  bool is_passive = false;
};

enum class AvoidWay { None, Left, Right, Center };
enum class LimitType { None, Normal, Front, Side };

enum class HysteresisType {
  IsInConsiderLateralRangeHysteresis,
  IsObstacleConsideredHysteresis,
  EnoughSpaceHysteresis,
  AvoidWaySelect
};

struct AvoidInfo {
  void Reset() {
    normal_left_avoid_threshold = 0.0;
    normal_right_avoid_threshold = 0.0;
    desire_lat_offset = 0.0;
    lat_offset = 0.0;
    avoid_way = AvoidWay::None;
    allow_front_max_opposite_offset = kDefaultLimitLateralDistance;
    allow_front_max_opposite_offset_id = kDefaultLimitId;
    allow_side_max_opposite_offset = kDefaultLimitLateralDistance;
    allow_side_max_opposite_offset_id = kDefaultLimitId;
    is_use_ego_position = false;
  }
  void operator=(const AvoidInfo &avoid_info) {
    normal_left_avoid_threshold = avoid_info.normal_left_avoid_threshold;
    normal_right_avoid_threshold = avoid_info.normal_right_avoid_threshold;
    desire_lat_offset = avoid_info.desire_lat_offset;
    lat_offset = avoid_info.lat_offset;
    avoid_way = avoid_info.avoid_way;
    allow_front_max_opposite_offset =
        avoid_info.allow_front_max_opposite_offset;
    allow_front_max_opposite_offset_id =
        avoid_info.allow_front_max_opposite_offset_id;
    allow_side_max_opposite_offset = avoid_info.allow_side_max_opposite_offset;
    allow_side_max_opposite_offset_id =
        avoid_info.allow_side_max_opposite_offset_id;
    is_use_ego_position = avoid_info.is_use_ego_position;
  }

  double normal_left_avoid_threshold = kDefaultSafeDistance;
  double normal_right_avoid_threshold = kDefaultSafeDistance;
  double desire_lat_offset = 0.0;
  double lat_offset = 0.0;
  AvoidWay avoid_way = AvoidWay::None;
  bool is_use_ego_position = false;
  double allow_front_max_opposite_offset = 0.0;
  int allow_front_max_opposite_offset_id = kDefaultLimitId;
  double allow_side_max_opposite_offset = 0.0;
  int allow_side_max_opposite_offset_id = kDefaultLimitId;
};

namespace lateral_offset_decider {
const double kTruckMinLength = 6.5;
bool IsCameraObstacle(const TrackedObject &tr);
bool IsStaticObstacle(const AvoidObstacleInfo &avoid_obstacle);
bool IsInConsiderLateralRange();
bool IsFrontObstacleConsider(
    const framework::Session *session, const TrackedObject &tr, bool is_left,
    const AvoidInfo &avoid_info,
    std::map<HysteresisType,
             std::variant<std::map<int, HysteresisDecision>,
                          std::map<std::pair<int, int>, HysteresisDecision>>>
        &hysteresis_maps);
bool IsSideObstacleConsider(
    const framework::Session *session, const TrackedObject &tr, bool is_left,
    std::map<HysteresisType,
             std::variant<std::map<int, HysteresisDecision>,
                          std::map<std::pair<int, int>, HysteresisDecision>>>
        &hysteresis_maps);
bool AvoidWaySelectForTwoObstaclev2(const framework::Session *session,
                                    const AvoidObstacleInfo &avoid_obstacle,
                                    const TrackedObject &tr);
bool IsTruck(const AvoidObstacleInfo &avoid_obstacle);
bool IsVRU(const AvoidObstacleInfo &avoid_obstacle);
bool IsCone(const AvoidObstacleInfo &avoid_obstacle);
bool IsPassive(const AvoidObstacleInfo &avoid_obstacle);
bool HasEnoughSpace(const AvoidObstacleInfo &avoid_obstacle_1,
                    const AvoidObstacleInfo &avoid_obstacle_2);
bool HasOverlap(const framework::Session *session,
                const AvoidObstacleInfo &avoid_obstacle, double front_lon_buf,
                double rear_lon_buf);
}  // namespace lateral_offset_decider
}  // namespace planning