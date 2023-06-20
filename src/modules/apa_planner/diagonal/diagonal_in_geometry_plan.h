#pragma once

#include <vector>

#include "common/geometry_planning_io.h"
#include "common/vehicle_param_helper.h"
#include "math/line_segment2d.h"
#include "math/polygon2d.h"

namespace planning {
namespace apa_planner {

class DiagonalInGeometryPlan {
 public:
  DiagonalInGeometryPlan() = default;

  bool ABSegment(const PlanningPoint &point_a, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool ReverseABSegment(const PlanningPoint &point_a, bool is_start, bool is_rough_calc,
                        DiagonalSegmentsInfo *segments_info);
  bool BCSegment(const PlanningPoint &point_b, double len_ab, bool is_start, bool is_rough_calc,
                 DiagonalSegmentsInfo *segments_info);
  bool CDSegment(const PlanningPoint &point_c, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool CD1Segment(const PlanningPoint &point_c, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool CD2Segment(const PlanningPoint &point_c, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool DE1Segment(const PlanningPoint &point_d, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool EFSegment(const PlanningPoint &point_e, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool CD3Segment(const PlanningPoint &point_c, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool DESegment(const PlanningPoint &point_d, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool DE2Segment(const PlanningPoint &point_d, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);
  bool DE3Segment(const PlanningPoint &point_d, bool is_start, bool is_rough_calc, DiagonalSegmentsInfo *segments_info);

  void SetTargetPoint(const PlanningPoint &target_point);

  // TODO(xjli32): use obstacles rather than slot edges
  void SetObjectMap(const std::vector<planning_math::LineSegment2d> &objects_map_line_segments) {
    objects_map_line_segments_ = objects_map_line_segments;
  }

  void SetSlotType(int slot_sign) { slot_sign_ = slot_sign; }

  bool CheckSlotOpenSideWrong(const PlanningPoint &start_point);

 private:
  std::vector<double> GetXslotCenterLineVec(const double max_x_slot_central_line_bias,
                                            const double x_slot_central_line_step) const;

  bool CollideWithObjectsByBox(const PlanningPoint &veh_point, const double front_buffer, const double rear_buffer,
                               const double lat_buffer) const;
  bool CollideWithObjectsByBox(const PlanningPoint &veh_point1, const PlanningPoint &veh_point2, const double radius,
                               const double front_buffer, const double rear_buffer, const double lat_buffer) const;

  bool CollideWithObjectsByPolygon(const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
                                   const planning::planning_math::Polygon2d &init_ego_polygon) const;

  bool CollideWithObjectsByPolygon(const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
                                   const double radius,
                                   const planning::planning_math::Polygon2d &init_ego_polygon) const;

  bool CEndCollideCheck(const PlanningPoint &point_c, const double safe_dst) const;

  double CalPointxBiasCost(const double x_bias) const;
  double CalSegmentLengthCost(const double segment_len) const;
  double CalRadiusCost(const double radius, const double len) const;
  double CalEndThetaCost(const double theta) const;
  double CalBCThetaDiffCost(const double theta_diff) const;
  double CalPointDThetaCost(const double theta) const;
  void CalTargetXVec();
  double CalDESegmentLengthCost(const double segment_len) const;

 private:
  double min_turn_radius_ = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double width_veh_ = VehicleParamHelper::Instance()->GetParam().width();
  double half_width_veh_ = VehicleParamHelper::Instance()->GetParam().width() * 0.5;
  double front_edge_to_center_ = VehicleParamHelper::Instance()->GetParam().front_edge_to_center();
  double back_edge_to_center_ = VehicleParamHelper::Instance()->GetParam().back_edge_to_center();

  int slot_sign_ = 1;  // 1:Right,(default),-1:Left

  double sin_target_point_theta_ = 0.0;
  double cos_target_point_theta_ = 0.0;
  PlanningPoint target_point_;
  std::vector<planning_math::LineSegment2d> objects_map_line_segments_;
  std::vector<double> target_x_vec_;
};

}  // namespace apa_planner
}  // namespace planning