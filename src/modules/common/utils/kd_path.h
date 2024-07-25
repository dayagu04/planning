#pragma once
#include <iostream>
#include <memory>
#include <vector>

#include "math/aaboxkdtree2d.h"
#include "math/geometry_object.h"
#include "math/line_segment2d.h"
#include "utils/path_point.h"
#include "vec2d.h"

using namespace planning;
using namespace planning::planning_math;
namespace planning {
namespace planning_math {
enum KDPathStatus { NORMAL = 0, EXCEED = 1, FALL = 2, ERROR = 3 };
constexpr static double KD_EPSILON = 1.0e-6;
constexpr static double KD_MAX = 1.0e6;
class KDPath {
 public:
  KDPath() = default;
  virtual ~KDPath() = default;
  explicit KDPath(std::vector<double>& x_vec, std::vector<double>& y_vec);
  explicit KDPath(std::vector<PathPoint>&& path_points);
  explicit KDPath(std::vector<PathPoint>&& path_points,
                  const bool need_reset_s);
  explicit KDPath(std::vector<PathPoint>&& path_points, const bool need_reset_s,
                  const double start_ref);
  bool InitData(const bool is_need_reset_s, bool head_mono = false,
                double start_ref = 0.0);

  double Length() const;

  bool KdtreeValid() const;

  PathPoint GetPathPointByS(const double path_s) const;

  const std::vector<PathPoint>& path_points() const;

  /**
   * @brief Compute (s, l) from (x, y) in 2-D.
   * if point is on the left side of the path,then l > 0. else l < 0.
   */
  bool XYToSL(const double x, const double y, double* const s,
              double* const l) const;

  bool SLToXY(const double s, const double l, double* const x,
              double* const y) const;

  KDPathStatus XYPointToSLPoint(const Point2D& cart_point,
                                Point2D& frenet_point);
  bool XYToSL(const Point2D& cart_point, Point2D& frenet_point);

  bool SLToXY(const Point2D& frenet_point, Point2D& cart_point);

  bool CartStateToFrenetState(const CartesianState& cart_state,
                              FrenetState& frenet_state);

  bool GetKappaByS(const double s, double* const kappa) const;

  double GetPathCurveHeading(double s) const;

  const LineSegment2d* GetNearestLineSegmentByXY(const double x,
                                                 const double y) const;

  const GeometryObject* GetNearestObjectByXY(const double x,
                                             const double y) const;

  bool NeighborXYToSL(const double x, const double y, const double theta,
                      const double radius, const double theta_tol,
                      double* const s, double* const l) const;

  std::vector<PathPoint>::const_iterator QueryLowerBound(
      const std::vector<PathPoint>& path_points, const double path_s) const;

  double DistanceTo(const Vec2d& point) const;

  double DistanceSquareTo(const Vec2d& point) const;

  double max_x() const { return max_x_; }

  double max_y() const { return max_y_; }

  double min_x() const { return min_x_; }

  double min_y() const { return min_y_; }

  bool IsPointIn(const Vec2d& point) const;

  const std::vector<const LineSegment2d*> GetLineSegments(
      const Vec2d& point, const double radius) const;

  const LineSegment2d* GetNearestLineSegment(const Vec2d& point) const;

  static double LimitAngle(double angle) {
    const double pi2 = 2.0 * M_PI;
    while (angle > M_PI) {
      angle -= pi2;
    }
    while (angle < -M_PI) {
      angle += pi2;
    }
    return angle;
  }
  // template <class T>
  static void ResetSCalculte(std::vector<PathPoint>& points) {
    double distance = 0.0;
    points.at(0).set_s(distance);
    for (size_t i = 1; i < points.size(); ++i) {
      double curr_x = points.at(i).x();
      double curr_y = points.at(i).y();
      double prev_x = points.at(i - 1).x();
      double prev_y = points.at(i - 1).y();
      distance += std::hypotf(prev_x - curr_x, prev_y - curr_y);
      points.at(i).set_s(distance);
    }
  }

  // template <class T>
  static bool PointsToPathPoints(const std::vector<Vec2d>& points,
                                 std::vector<PathPoint>& path_points) {
    path_points.resize(points.size());
    for (int32_t i = 0; i < points.size(); ++i) {
      path_points.at(i).set_x(points[i].x());
      path_points.at(i).set_y(points[i].y());
    }
    ResetCurveCalculate(path_points);
    return true;
  }

  // template <class T>
  static void ResetCurveCalculate(std::vector<PathPoint>& points) {
    if (points.size() < 2) {
      return;
    }
    // cal s
    double s = 0.0;
    ResetSCalculte(points);

    // cal heading
    for (int32_t i = 0; i < points.size(); ++i) {
      double theta = 0.0;
      double dx_theta = 0.0;
      double dy_theta = 0.0;
      if (i == 0) {
        dx_theta = points.at(i + 1).x() - points.at(i).x();
        dy_theta = points.at(i + 1).y() - points.at(i).y();
      } else if (i == points.size() - 1) {
        dx_theta = points.at(i).x() - points.at(i - 1).x();
        dy_theta = points.at(i).y() - points.at(i - 1).y();
      } else {
        dx_theta = points.at(i + 1).x() - points.at(i - 1).x();
        dy_theta = points.at(i + 1).y() - points.at(i - 1).y();
      }

      theta = std::atan2(dy_theta, dx_theta);
      points.at(i).set_theta(theta);
    }
    // calc kappa
    for (int32_t i = 0; i < points.size(); ++i) {
      double kappa = 0.0;
      double delta_s = 0.0;
      if (i == 0) {
        delta_s = points.at(i + 1).s() - points.at(i).s();
        kappa = LimitAngle(points.at(i + 1).theta() - points.at(i).theta()) /
                delta_s;
      } else if (i == points.size() - 1) {
        delta_s = points.at(i).s() - points.at(i - 1).s();
        kappa = LimitAngle(points.at(i).theta() - points.at(i - 1).theta()) /
                delta_s;
      } else {
        delta_s = points.at(i + 1).s() - points.at(i - 1).s();
        kappa =
            LimitAngle(points.at(i + 1).theta() - points.at(i - 1).theta()) /
            delta_s;
      }
      points.at(i).set_kappa(kappa);
    }
    // calc dkappa
    for (int32_t i = 0; i < points.size(); ++i) {
      double dkappa = 0.0;
      double delta_s = 0.0;
      if (i == 0) {
        delta_s = points.at(i + 1).s() - points.at(i).s();
        dkappa = (points.at(i + 1).kappa() - points.at(i).kappa()) / delta_s;
      } else if (i == points.size() - 1) {
        delta_s = points.at(i).s() - points.at(i - 1).s();
        dkappa = (points.at(i).kappa() - points.at(i - 1).kappa()) / delta_s;
      } else {
        delta_s = points.at(i + 1).s() - points.at(i - 1).s();
        dkappa =
            (points.at(i + 1).kappa() - points.at(i - 1).kappa()) / delta_s;
      }
      points.at(i).set_dkappa(dkappa);
    }
    // calc ddkappa
    for (int32_t i = 0; i < points.size(); ++i) {
      double ddkappa = 0.0;
      double delta_s = 0.0;
      if (i == 0) {
        delta_s = points.at(i + 1).s() - points.at(i).s();
        ddkappa = (points.at(i + 1).dkappa() - points.at(i).dkappa()) / delta_s;
      } else if (i == points.size() - 1) {
        delta_s = points.at(i).s() - points.at(i - 1).s();
        ddkappa = (points.at(i).dkappa() - points.at(i - 1).dkappa()) / delta_s;
      } else {
        delta_s = points.at(i + 1).s() - points.at(i - 1).s();
        ddkappa =
            (points.at(i + 1).dkappa() - points.at(i - 1).dkappa()) / delta_s;
      }
      points.at(i).set_ddkappa(ddkappa);
    }
  }

  // template <class T>
  static void RemoveDuplicates(std::vector<Vec2d>* raw_points,
                               const double threshold) {
    int count = 0;
    for (size_t i = 0; i < raw_points->size(); ++i) {
      if (count == 0 || (*raw_points)[i].DistanceSquareTo(
                            (*raw_points)[count - 1]) > threshold) {
        (*raw_points)[count++] = (*raw_points)[i];
      } else {
        (*raw_points)[count - 1] = (*raw_points)[i];
      }
    }
    raw_points->resize(count);
  }

  /*
   * @brief: reset segment heading to monotonic
   */
  bool MonoHeading(const double startref) {
    double startref_angle = startref;
    if (line_segments_.size() < 1) {
      return true;
    }
    for (int i = 0; i < line_segments_.size(); i++) {
      auto& seg = line_segments_[i];
      double delta_angle = LimitAngle(seg.heading() - startref_angle);
      seg.set_heading(startref_angle + delta_angle);
      startref_angle = seg.heading();
    }
    return true;
  }

  static inline double PerpendicularDistance(const PathPoint& line_pt_start,
                                             const PathPoint& line_pt_end,
                                             const PathPoint& cur_pt);
  static void DouglasPeuckerRecursion(const std::vector<PathPoint>& path_points,
                                      const int first_pt_index,
                                      const int last_pt_index,
                                      const double max_tolerance,
                                      std::vector<bool>* index_flags);

  static std::vector<PathPoint> DouglasPeuckerCondense(
      const std::vector<PathPoint>& path_points, const double max_tolerance,
      const double skip_dist = -1.0);

 private:
  bool BuildKDTree();
  void Project(const LineSegment2d& line, const double x, const double y,
               const double base_s, double* const s, double* const l) const;

 private:
  std::vector<double> x_vec_;
  std::vector<double> y_vec_;
  std::vector<PathPoint> path_points_;

  std::vector<GeometryObject> objects_;
  std::vector<LineSegment2d> line_segments_;
  std::shared_ptr<AABoxKDTree2d<GeometryObject>> kd_tree_;
  double length_ = 0.0;
  bool kdtree_valid_ = false;

  double min_x_ = 0.;
  double min_y_ = 0.;
  double max_x_ = 0.;
  double max_y_ = 0.;
};

}  // namespace planning_math
}  // namespace planning
