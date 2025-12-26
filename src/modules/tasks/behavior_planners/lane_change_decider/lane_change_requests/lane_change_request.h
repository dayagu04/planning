#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "adas_function/display_state_types.h"
#include "config/basic_type.h"
#include "ifly_time.h"
#include "interface/src/c/common_c.h"
#include "lane_change_lane_manager.h"
#include "lateral_obstacle.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {

struct ConePoint {
  double x, y;
  double s, l;
  double left_dist, right_dist;
  int32_t id;
  int cluster;
  bool visited;

  // Default constructor
  ConePoint()
      : x(0.0),
        y(0.0),
        s(0.0),
        l(0.0),
        left_dist(0.0),
        right_dist(0.0),
        id(0),
        cluster(-1),
        visited(false) {}
  // Parameterized constructor
  ConePoint(int32_t id, double x, double y, double s, double l,
            double left_dist, double right_dist)
      : x(x),
        y(y),
        s(s),
        l(l),
        left_dist(left_dist),
        right_dist(right_dist),
        id(id),
        cluster(-1),
        visited(false) {}
};
/// @brief 换道请求的基类，生成、结束换道请求等
class LaneChangeRequest {
 public:
  enum TurnSwitchState {
    NONE = 0,
    LEFT_FIRMLY_TOUCH = 1,
    RIGHT_FIRMLY_TOUCH = 2,
    LEFT_LIGHTLY_TOUCH = 3,
    RIGHT_LIGHTLY_TOUCH = 4,
    ERROR = 5,
  };
  LaneChangeRequest(
      planning::framework::Session* session,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~LaneChangeRequest() = default;

  void GenerateRequest(RequestType direction);
  void Finish();
  bool AggressiveChange() const;
  bool IsDashedLineEnough(RequestType direction, const double ego_vel,
                          std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
                          const StateMachineLaneChangeStatus& lc_status);

  RequestType request_type() const { return request_type_; }
  int target_lane_virtual_id() { return target_lane_virtual_id_; }
  int origin_lane_virtual_id() { return origin_lane_virtual_id_; }
  void set_target_lane_virtual_id(int target_lane_virtual_id) {
    target_lane_virtual_id_ = target_lane_virtual_id;
  }
  RequestType turn_signal() const { return turn_signal_; }
  double tstart() const { return tstart_; }
  double tfinish() const { return tfinish_; }
  bool ComputeLcValid(RequestType direction);
  bool IsDashEnoughForRepeatSegments(
      const RequestType& lc_request, const RequestSource& lc_request_source,
      int origin_lane_id, const StateMachineLaneChangeStatus& lc_status) const;
  iflyauto::LaneBoundaryType MakesureCurrentBoundaryType(
      const RequestType lc_request, const int origin_lane_id) const;
  bool IsRoadBorderSurpressLaneChange(const RequestType lc_request,
                                      const int origin_lane_id,
                                      const int target_lane_id);

  bool IsRoadBorderSurpressDuringLaneChange(const RequestType lc_direction,
                                            const int origin_lane_id,
                                            const int target_lane_id) const;

  double CalculatePressLineRatio(const int origin_lane_id,
                                 const RequestType& lc_request) const;
  double CalculatePressLineRatioByOrigin(const int origin_lane_id,
                                         const RequestType& lc_request) const;
  double CalculatePressLineRatioByTarget(int origin_lane_id,
                                         const RequestType& lc_request) const;
  double CalculatePressLineRatioByTwoLanes(const int origin_lane_id,
                                           const int target_lane_id,
                                           const RequestType& lc_request) const;
  double CalculateDynamicTTCtime(const int origin_lane_id,
                                 const RequestType& lc_request) const;
  bool EgoInIntersection();

  bool ConeSituationJudgement(const std::shared_ptr<VirtualLane>& target_lane);

  virtual void SetLaneChangeCmd(std::uint8_t lane_change_cmd) {
    lane_change_cmd_ = lane_change_cmd;
  }
  virtual void SetLaneChangeCancelFromTrigger(bool trigger_lane_change_cancel) {
    trigger_lane_change_cancel_ = trigger_lane_change_cancel;
  }
  virtual IntCancelReasonType lc_request_cancel_reason() {
    return lc_request_cancel_reason_;
  }

  bool ConeDistance(const ConePoint& a, const ConePoint& b, double eps_s,
                    double eps_l);

  void ExpandCluster(std::vector<ConePoint>& cone_points, int index, int c,
                     double eps_s, double eps_l, int minPts);

  void DbScan(std::vector<ConePoint>& cone_points, double eps_s, double eps_l,
              int minPts);

  double CalcClusterToBoundaryDist(const std::vector<ConePoint>& points,
                                   RequestType direction);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  double QueryLaneMinWidth(
      std::vector<ConePoint>& cone_points,
      const std::vector<std::pair<double, double>>& lane_s_width,
      const double target_s);

  bool GetLaneWidthByCone(const std::shared_ptr<VirtualLane> ego_lane,
                          const double cone_s, const double cone_l,
                          bool is_left, double* dist,
                          std::vector<std::pair<double, double>>& lane_s_width);

 protected:
  TrackInfo lc_invalid_track_;
  RequestType request_type_ = NO_CHANGE;
  int target_lane_virtual_id_ = -1000;  // invalid
  int origin_lane_virtual_id_ = -1000;
  int origin_lane_order_id_ = -1000;
  RequestType turn_signal_ = NO_CHANGE;
  int intersection_count_ = 0;
  double tstart_ = 0.0;
  double tfinish_ = 0.0;
  framework::Session* session_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr_;
  double aggressive_lane_change_distance_highway_{0.0};
  double aggressive_lane_change_distance_urban_{150.0};
  std::uint8_t lane_change_cmd_{0};
  bool trigger_lane_change_cancel_{false};
  IntCancelReasonType lc_request_cancel_reason_{NO_CANCEL};
  std::vector<ConePoint> target_lane_cone_points_;
  std::map<int, std::vector<ConePoint>> target_lane_cone_cluster_attribute_set_;
  std::vector<std::pair<double, double>> left_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>>
      right_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>> origin_lane_s_width_;
};

}  // namespace planning
