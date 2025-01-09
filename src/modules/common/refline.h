#pragma once

#include <float.h>
#include <limits.h>

#include <map>
#include <memory>

#include "utils/spline.h"
#include "utils_math.h"
// #include "worldmodel/lane.h"
#include "define/geometry.h"
#include "define/lateral_behavior_planner_output.h"
#include "src/modules/common/utils/path_point.h"
#include "utils/pose2d_utils.h"

namespace planning {

// typedef enum {
//   UNKNOWN_REFLINE = -100,
//   LEFT_LEFT_REFLINE = -2,
//   LEFT_REFLINE = -1,
//   CURR_REFLINE = 0,
//   RIGHT_REFLINE = 1,
//   RIGHT_RIGHT_REFLINE = 2
// } RefLinePosition;

typedef enum { LOCATION_NONE, LOCATION_ROAD, LOCATION_INTER } LocationEnum;

// struct PPath {
//   double u_min;
//   double u_max;
//   planning::planning_math::spline x_spline;
//   planning::planning_math::spline y_spline;

//   PPath(double min_u, double max_u, const planning::planning_math::spline
//   &spline_x,
//         const planning::planning_math::spline &spline_y)
//       : u_min(min_u), u_max(max_u), x_spline(spline_x), y_spline(spline_y) {}
// };

// struct LaneBoundaryPolylineDerived : MSDLaneBoundaryPolyline {
//   LaneBoundaryPolylineDerived() {}

//   LaneBoundaryPolylineDerived(const MSDLaneBoundaryPolyline &polyline) {
//     track_id = polyline.track_id;
//     poly_coefficient = polyline.poly_coefficient;
//     available_interval = polyline.available_interval;
//   }

//   LaneBoundaryPolylineDerived &operator=(const MSDLaneBoundaryPolyline
//   &polyline) {
//     track_id = polyline.track_id;
//     poly_coefficient = polyline.poly_coefficient;
//     available_interval = polyline.available_interval;

//     return *this;
//   }

//   LaneBoundaryPolylineDerived &operator=(const LaneBoundaryPolylineDerived
//   &polyline) {
//     track_id = polyline.track_id;
//     poly_coefficient = polyline.poly_coefficient;
//     available_interval = polyline.available_interval;
//     relative_id = polyline.relative_id;

//     return *this;
//   }

//   int relative_id = 99;
// };

// struct ReferenceLinePointDerived : MSDReferenceLinePoint {
//   ReferenceLinePointDerived() {}

//   ReferenceLinePointDerived(const MSDReferenceLinePoint &refline_point) {
//     car_point = refline_point.car_point;
//     enu_point = refline_point.enu_point;
//     curvature = refline_point.curvature;
//     yaw = refline_point.yaw;
//     distance_to_left_road_border =
//     refline_point.distance_to_left_road_border; distance_to_right_road_border
//     = refline_point.distance_to_right_road_border;
//     distance_to_left_lane_border =
//     refline_point.distance_to_left_lane_border; distance_to_right_lane_border
//     = refline_point.distance_to_right_lane_border; distance_to_left_obstacle
//     = refline_point.distance_to_left_obstacle; distance_to_right_obstacle =
//     refline_point.distance_to_right_obstacle; lane_width =
//     refline_point.lane_width; max_velocity = refline_point.max_velocity;
//     track_id = refline_point.track_id;
//     left_road_border_type = refline_point.left_road_border_type;
//     right_road_border_type = refline_point.right_road_border_type;
//   }

//   ReferenceLinePointDerived &operator=(const MSDReferenceLinePoint
//   &refline_point) {
//     car_point = refline_point.car_point;
//     enu_point = refline_point.enu_point;
//     curvature = refline_point.curvature;
//     yaw = refline_point.yaw;
//     distance_to_left_road_border =
//     refline_point.distance_to_left_road_border; distance_to_right_road_border
//     = refline_point.distance_to_right_road_border;
//     distance_to_left_lane_border =
//     refline_point.distance_to_left_lane_border; distance_to_right_lane_border
//     = refline_point.distance_to_right_lane_border; distance_to_left_obstacle
//     = refline_point.distance_to_left_obstacle; distance_to_right_obstacle =
//     refline_point.distance_to_right_obstacle; lane_width =
//     refline_point.lane_width; max_velocity = refline_point.max_velocity;
//     track_id = refline_point.track_id;
//     left_road_border_type = refline_point.left_road_border_type;
//     right_road_border_type = refline_point.right_road_border_type;

//     return *this;
//   }

//   ReferenceLinePointDerived &operator=(const ReferenceLinePointDerived
//   &refline_point) {
//     car_point = refline_point.car_point;
//     enu_point = refline_point.enu_point;
//     curvature = refline_point.curvature;
//     yaw = refline_point.yaw;
//     distance_to_left_road_border =
//     refline_point.distance_to_left_road_border; distance_to_right_road_border
//     = refline_point.distance_to_right_road_border;
//     distance_to_left_lane_border =
//     refline_point.distance_to_left_lane_border; distance_to_right_lane_border
//     = refline_point.distance_to_right_lane_border; distance_to_left_obstacle
//     = refline_point.distance_to_left_obstacle; distance_to_right_obstacle =
//     refline_point.distance_to_right_obstacle; lane_width =
//     refline_point.lane_width; max_velocity = refline_point.max_velocity;
//     track_id = refline_point.track_id;
//     direction = refline_point.direction;
//     in_intersection = refline_point.in_intersection;
//     left_road_border_type = refline_point.left_road_border_type;
//     right_road_border_type = refline_point.right_road_border_type;

//     return *this;
//   }

//   int direction = MSD_LANE_CONNECT_DIRECTION_NORMAL;
//   bool in_intersection = false;
// };

const double kInvalidDist = NL_NMAX;

bool equal_zero(double a);

void calc_cartesian_frenet(
    const std::vector<planning_math::PathPoint> &path_points, double x,
    double y, double &s, double &l, double &v_s, double &v_l, double &theta,
    bool get_theta, double *v = nullptr, double *yaw = nullptr);

void calc_frenet_cartesian(
    const std::vector<planning_math::PathPoint> &path_points, double s,
    double l, double &x, double &y);

// void discrete(double start, double end, double gap,
//               std::vector<double> &output) {
//   output.clear();
//   for (double value = start; value < end; value += gap) {
//     output.push_back(value);
//   }
// }

template <size_t SIZE>
double interp(double x, const std::array<double, SIZE> &xp,
              const std::array<double, SIZE> &fp) {
  int n = xp.size();

  int hi = 0;
  while (hi < n && x > xp[hi]) {
    hi++;
  }

  int low = hi - 1;
  if (hi == n && x > xp[low]) {
    return fp.back();
  } else {
    if (hi == 0) {
      return fp[0];
    } else {
      if (equal_zero(xp[hi] - xp[low])) {
        return 0;
      } else {
        return (x - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) +
               fp[low];
      }
    }
  }
}

// class RawRefLine {
// public:
//   RawRefLine(int position = RefLinePosition::UNKNOWN_REFLINE);

//   RawRefLine(const RawRefLine &raw_refline);

//   virtual ~RawRefLine() = default;

//   RawRefLine &operator=(const RawRefLine &raw_refline);

//   void update(const std::vector<ReferenceLinePointDerived> &waypoints);

//   double min_square_dist(double x = 0, double y = 0);

//   const std::vector<Point2D> &waypoints() const { return waypoints_; }

//   const std::map<std::string, bool> &point_ids() const { return point_ids_; }

//   int position() const { return position_; }

//   const std::array<RawRefLine *, 2> &neighbours() const { return neighbours_;
//   }

//   void set_neighbours(const std::array<RawRefLine *, 2> &neighbours) {
//     neighbours_ = neighbours;
//   }

//   void save_context(RawRefLineContext &context) const;
//   void restore_context(const RawRefLineContext &context);

//   void reset();

// private:
//   bool exist_;
//   int position_;
//   std::vector<Point2D> waypoints_;
//   std::map<std::string, bool> point_ids_;
//   std::array<RawRefLine *, 2> neighbours_;
// };

// class RefLine {
// public:
//   RefLine();
//   virtual ~RefLine() = default;

//   void update_pathpoints();

//   // void set_ppath(const std::vector<double> &u_points,
//   //                const planning::planning_math::spline &x_spline,
//   //                const planning::planning_math::spline &y_spline);

//   // void set_master(const RawRefLine *master) {
//   //   master_ = master;
//   //   if (master != nullptr) {
//   //     position_ = master->position();
//   //   }
//   // }

//   void cartesian_frenet(double x, double y, double &s, double &l, double
//   &theta,
//                         bool get_theta = false);

//   void frenet_cartesian(double s, double l, double &x, double &y);

//   // int position() const { return position_; }

//   const std::vector<PathPoint> &path_points() const { return path_points_; }

//   // void save_context(FixRefLineContext &context) const;
//   // void restore_context(const FixRefLineContext &context);

// private:
//   std::vector<PathPoint> path_points_;
//   // const RawRefLine *master_;
//   // std::unique_ptr<PPath> ppath_;
//   // int position_;
// };

}  // namespace planning
