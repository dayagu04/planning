#pragma once

#include "config/basic_type.h"
#include "lane_change_request.h"
#include "tracked_object.h"

namespace planning {

/// @brief 避障换道请求
class ConeRequest : public LaneChangeRequest {
 public:
  ConeRequest(planning::framework::Session* session,
              std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
              std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~ConeRequest() = default;

  void Update(int lc_status);

  void Reset();

 private:
  // define cone point of cluster
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
  void UpdateConeSituation(int lc_status);

  void setLaneChangeRequestByCone();

  void GetTargetLaneWidthByCone(
      const std::vector<std::pair<double, double>> lane_s_width,
      const std::shared_ptr<VirtualLane> target_lane, const double cone_s,
      const double cone_l, bool is_left, double* dist);

  bool GetOriginLaneWidthByCone(const std::shared_ptr<VirtualLane> ego_lane,
                                const double cone_s, const double cone_l,
                                bool is_left, double* dist);

  bool ConeDistance(const ConePoint& a, const ConePoint& b, double eps_s,
                    double eps_l);

  void ExpandCluster(std::vector<ConePoint>& cone_points, int index, int c,
                     double eps_s, double eps_l, int minPts);

  void DbScan(std::vector<ConePoint>& cone_points, double eps_s, double eps_l,
              int minPts);

  double CalcClusterToBoundaryDist(const std::vector<ConePoint>& points,
                                   RequestType direction);

  void ConeDir();

  bool ConesDirection(RequestType& direction);

  bool CheckEgoLaneAvailable(bool is_left);

  bool CheckTargetLaneAvailable(bool is_left,
                                const std::shared_ptr<VirtualLane> lane);

  double ConeSpearmanRankCorrelation(const std::vector<ConePoint> points);

  double ConeComputeSlope(std::vector<ConePoint> points);

  std::vector<double> ConeRankify(std::vector<double>& arr);

  bool ConeMean(const std::vector<ConePoint>& points, double& s_mean,
                double& l_mean);

  bool ConeStddev(const std::vector<ConePoint>& points, double s_mean,
                  double l_mean, double& s_stddev, double& l_stddev);

  bool ConeStandardize(std::vector<ConePoint>& points);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::unordered_map<int, TrackedObject> tracks_map_;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  bool is_cone_lane_change_situation_ = false;
  int cone_alc_trigger_counter_;
  RequestType cone_lane_change_direction_ = NO_CHANGE;
  std::vector<ConePoint> cone_points_;
  std::map<int, std::vector<ConePoint>> cone_cluster_attribute_set_;
  std::map<int, ad_common::math::Polygon2d> out_cluster_;
  std::vector<int32_t> cone_cluster_size_;
  std::vector<ConePoint> cone_cluster_;
  std::vector<std::pair<double, double>> left_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>>
      right_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>> origin_lane_s_width_;
  // bool use_query_lane_width_ = false;
};

}  // namespace planning