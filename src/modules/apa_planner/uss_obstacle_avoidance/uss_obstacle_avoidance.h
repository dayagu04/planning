#ifndef __USS_OBSTACLE_AVOIDANCE_H__
#define __USS_OBSTACLE_AVOIDANCE_H__

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "dubins_lib/geometry_math.h"
#include "local_view.h"
#include "planning_plan.pb.h"
namespace planning {
class UssObstacleAvoidance {
  enum CarPointMode {
    LINE_MODE = 0,
    ARC_MODE = 1,
    COUNT,
  };
  struct Circle {
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double radius = 0.0;
    bool is_circle = true;
  };

  struct Arc {
    Circle circle_info;
    bool is_considered = true;
    Eigen::Vector2d pA;
    Eigen::Vector2d pB;
  };

  struct Paramters {
    double steer_ratio = 16.5;
    double arc_line_shift_steer_angle_deg = 2.5;
    double c1 = 0.3790;
    double max_remain_dist = 5.01;
    double lat_inflation = 0.1;
  };

  struct RemainDistInfo {
    double remain_dist = 100.0;
    size_t car_index = 0;
    size_t uss_index = 0;

    void Reset() {
      remain_dist = 100.0;
      car_index = 0;
      uss_index = 0;
    }
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset() {
    min_dist_car_arc_index_ = 0;
    min_dist_uss_arc_index_ = 0;
    remain_dist_ = 100.0;
  }

  void SetParam(Paramters &param) { param_ = param; }

  void Init() { InitVertexData(); }

  void SetLocalView(const LocalView *local_view_ptr) {
    local_view_ptr_ = local_view_ptr;
  }

  const size_t GetMinDistCarArcIndex() const { return min_dist_car_arc_index_; }

  const size_t GetMinDistUssArcIndex() const { return min_dist_uss_arc_index_; }

  const double GetRemainDist() const { return remain_dist_; }

  const std::vector<Eigen::Vector2d> GetCarVertex() const {
    return car_vertex_vec_;
  }

  const std::vector<Eigen::Vector2d> GetUssVertex() const {
    return uss_vertex_vec_;
  }

  const std::vector<double> GetUssRawDist() const { return uss_raw_dist_vec_; }

  const pnc::geometry_lib::LineSegment GetMinDistUssLine() const;

  bool Update(PlanningOutput::PlanningOutput *const planning_output);

 private:
  bool Preprocess();

  const bool CheckTwoCircleIntersection(const Circle &c1,
                                        const Circle &c2) const;

  const std::pair<Eigen::Vector2d, Eigen::Vector2d> GetTwoCircleIntersection(
      const Circle &c1, const Circle &c2) const;

  const bool CheckPointLiesOnArc(const Arc &arc,
                                 const Eigen::Vector2d &pC) const;

  const bool GetTwoArcIntersection(
      Eigen::Vector2d &intersection, const Arc &arc1,
      const Arc &arc2) const;  // note that assume just one interection

  const bool GetArcLineIntersection(
      Eigen::Vector2d &intersection, const Arc &arc1,
      const Arc &line2) const;  // use pA and pB of arc to fake line

  void InitVertexData();

  void GenCarArc();
  void GenCarLine();
  void GenUssArc();

  void CalRemainDist();

  std::vector<Eigen::Vector2d> car_vertex_vec_;
  std::vector<Eigen::Vector2d> uss_vertex_vec_;
  std::vector<double> uss_raw_dist_vec_;
  std::vector<double> uss_normal_angle_vec_;

  double steer_angle_ = 0.0;
  double heading_ = 0.0;
  bool reverse_flag_ = false;
  RemainDistInfo remain_dist_info_;

  Paramters param_;

  uint8_t car_point_mode_ = ARC_MODE;
  std::vector<Arc> car_arc_vec_;
  std::vector<Arc> uss_arc_vec_;

  size_t min_dist_car_arc_index_ = 0;
  size_t min_dist_uss_arc_index_ = 0;
  double remain_dist_ = 20.0;
  double rear_axle_center_radius_ = 1.0e4;
  Eigen::Vector2d turning_center_ = Eigen::Vector2d::Zero();

  const LocalView *local_view_ptr_ = nullptr;
  PlanningOutput::PlanningOutput *planning_output_;
};

};  // namespace planning

#endif
