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
  struct cone_point {
    double x, y;
    double s, l;
    double left_dist, right_dist;
    int32_t id;
    int cluster;
    bool visited;

    // Default constructor
    cone_point()
        : id(0),
          x(0.0),
          y(0.0),
          s(0.0),
          l(0.0),
          left_dist(0.0),
          right_dist(0.0),
          cluster(-1),
          visited(false) {}
    // Parameterized constructor
    cone_point(int32_t id, double x, double y, double s, double l,
               double left_dist, double right_dist)
        : id(id),
          x(x),
          y(y),
          s(s),
          l(l),
          left_dist(left_dist),
          right_dist(right_dist),
          cluster(-1),
          visited(false) {}
  };
  void updateConeSituation(int lc_status);

  void setLaneChangeRequestByCone();

  void GetTargetLaneWidthByCone(
      std::vector<std::pair<double, double>> lane_s_width,
      const std::shared_ptr<VirtualLane> target_lane, const double cone_s,
      const double cone_l, bool is_left, double* dist);

  bool GetOriginLaneWidthByCone(const std::shared_ptr<VirtualLane> ego_lane,
                                const double cone_s, const double cone_l,
                                bool is_left, double* dist);

  bool ConeDistance(const cone_point& a, const cone_point& b, double eps_s,
                    double eps_l);

  void ExpandCluster(std::vector<cone_point>& cone_points, int index, int c,
                     double eps_s, double eps_l, int minPts);

  void DbScan(std::vector<cone_point>& cone_points, double eps_s, double eps_l,
              int minPts);

  double CalcClusterToBoundaryDist(const std::vector<cone_point>& points,
                                   RequestType direction);

  void ConeDir();

  bool ConesDirection(RequestType& direction);

  bool CheckEgoLaneAvailable(bool is_left);

  bool CheckTargetLaneAvailable(
      bool is_left, const std::shared_ptr<VirtualLane> lane);

  double ConeSpearmanRankCorrelation(std::vector<cone_point> points);

  double ConeComputeSlope(std::vector<cone_point> points);

  std::vector<double> ConeRankify(std::vector<double>& arr);

  bool ConeMean(const std::vector<cone_point>& points, double& s_mean,
                double& l_mean);

  bool ConeStddev(const std::vector<cone_point>& points, double s_mean,
                  double l_mean, double& s_stddev, double& l_stddev);

  bool ConeStandardize(std::vector<cone_point>& points);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  std::shared_ptr<KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::unordered_map<int, TrackedObject> tracks_map_;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  bool is_cone_lane_change_situation_ = false;
  int cone_alc_trigger_counter_;
  RequestType cone_lane_change_direction_ = NO_CHANGE;
  std::vector<cone_point> cone_points_;
  std::map<int, std::vector<cone_point>> points_by_cluster_;
  std::map<int, ad_common::math::Polygon2d> out_cluster_;
  std::vector<int32_t> cone_cluster_size_;
  std::vector<cone_point> cone_cluster_;
  std::vector<std::pair<double, double>> left_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>>
      right_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>> origin_lane_s_width_;
  bool use_query_lane_width_ = false;
};

}  // namespace planning