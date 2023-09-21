#ifndef __USS_OBSTACLE_AVOIDANCE_H__
#define __USS_OBSTACLE_AVOIDANCE_H__

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "local_view.h"
#include "planning_plan.pb.h"

namespace planning {
class UssObstacleAvoidance {
  enum VehiclePointMode {
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
    Eigen::Vector2d pA;
    Eigen::Vector2d pB;
  };

  struct Paramters {
    double steer_ratio = 15.7;
    double arc_line_shift_steer_angle_deg = 2.5;
    double c1 = 0.3790;
    double max_remain_dist = 4.0;
    double lat_inflation = 0.1;
  };

  struct RemainDistInfo {
    double remain_dist = 100.0;
    size_t vehicle_index = 0;
    size_t uss_index = 0;

    void Reset() {
      remain_dist = 100.0;
      vehicle_index = 0;
      uss_index = 0;
    }
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void SetParam(Paramters &param) { param_ = param; }

  void Init() { InitVertexData(); }

  void SetLocalView(LocalView *local_view_ptr) {
    local_view_ptr_ = local_view_ptr;
  }

  bool Update(PlanningOutput::PlanningOutput *const planning_output);

 private:
  void Preprocess();

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

  void ConstructVehicleArc();
  void ConstructVehicleLine();
  void ConstructUssArc();

  void CalculateRemainDist();

  std::vector<Eigen::Vector2d> vehicle_vertex_vec_;
  std::vector<Eigen::Vector2d> uss_vertex_vec_;
  std::vector<double> uss_raw_dist_vec_;
  std::vector<double> uss_normal_angle_vec_;

  double steer_angle_ = 0.0;
  double heading_ = 0.0;
  bool reverse_flag_ = false;
  RemainDistInfo remain_dist_info_;

  Paramters param_;

  uint8_t vehicle_point_mode_ = ARC_MODE;
  std::vector<Arc> vehicle_arc_vec_;
  std::vector<Arc> uss_arc_vec_;

  size_t min_dist_vehicle_arc_index_ = 0;
  size_t min_dist_uss_arc_index_ = 0;
  double remain_dist_ = 20.0;
  double rear_axle_center_radius_ = 1.0e4;
  Eigen::Vector2d turning_center_ = Eigen::Vector2d::Zero();

  const LocalView *local_view_ptr_ = nullptr;
  PlanningOutput::PlanningOutput *planning_output_;
};

};  // namespace planning

#endif
