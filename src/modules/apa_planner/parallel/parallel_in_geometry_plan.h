#pragma once

#include <vector>

#include "apa_planner/common/geometry_planning_io.h"
#include "apa_planner/common/vehicle_param_helper.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"

namespace planning {
namespace apa_planner {

class ParallelInGeometryPlan {
 public:
  ParallelInGeometryPlan() = default;

  bool ABSegment(const PlanningPoint &point_a, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool BCSegment(const PlanningPoint &point_b, double len_ab, bool is_start,
      bool is_rough_calc, ParallelSegmentsInfo *segments_info);
  bool CDSegment(const PlanningPoint &point_c, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool DESegment(const PlanningPoint &point_d, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool EFSegment(const PlanningPoint &point_e, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool EF1Segment(const PlanningPoint &point_e, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool EF2Segment(const PlanningPoint &point_e, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool FGSegment(const PlanningPoint &point_f, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool EF3Segment(const PlanningPoint &point_e, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool FHSegment(const PlanningPoint &point_f, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool FH1Segment(const PlanningPoint &point_f, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool FH2Segment(const PlanningPoint &point_f, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool HISegment(const PlanningPoint &point_h, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);
  bool FH3Segment(const PlanningPoint &point_f, bool is_start, bool is_rough_calc,
      ParallelSegmentsInfo *segments_info);

  void SetTargetPoint(const PlanningPoint& target_point);

  const PlanningPoint& GetUpdatedTargetPoint() const {
    return updated_target_point_;
  }

  // TODO(xjli32): use obstacles rather than slot edges
  void SetObjectMap(
      const std::vector<planning_math::LineSegment2d>& objects_map_line_segments) {
    objects_map_line_segments_ = objects_map_line_segments;
  }

  void SetSlotType(int slot_sign) {
    slot_sign_ = slot_sign;
  }

 private:
  bool CheckSlotOpenSideWrong(const PlanningPoint &start_point);

  bool CollideWithObjectsByBox(const PlanningPoint &veh_point,
      const double front_buffer, const double rear_buffer,
      const double lat_buffer) const;
  bool CollideWithObjectsByBox(const PlanningPoint &veh_point1,
      const PlanningPoint &veh_point2, const double radius,
      const double front_buffer, const double rear_buffer,
      const double lat_buffer) const;

  bool CollideWithObjectsByPolygon(
      const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
      const planning::planning_math::Polygon2d& init_ego_polygon) const;

  bool CollideWithObjectsByPolygon(const PlanningPoint &veh_point1,
      const PlanningPoint &veh_point2, const double radius,
      const planning::planning_math::Polygon2d& init_ego_polygon) const;

  planning::planning_math::Polygon2d ConstructVehiclePolygonWithBuffer(
      const PlanningPoint &veh_point, const double front_buffer,
      const double rear_buffer, const double lat_buffer) const;

  bool CEndCollideCheck(const PlanningPoint &point_c,
      const double safe_dst) const;

  double CalPointYBiasCost(const double y_bias) const;
  double CalSegmentLengthCost(const double segment_len) const;
  double CalRadiusCost(const double radius,const double len) const;
  double CalEndXCost(const double x, const bool is_forward) const;
  double CalEndYCost(const double y) const;
  double CalEndThetaCost(const double theta) const;
  void CalTargetYVec();
  double CalFGSegmentCost(const PlanningPoint& point_f,
      const PlanningPoint& point_g, const double radius_fg,
      PlanningPoint* const updated_target_point) const;
  double CalEFSegmentCost(const PlanningPoint& point_e,
      const PlanningPoint& point_f, const double radius_ef,
      PlanningPoint* const updated_target_point) const;

 private:
  double min_turn_radius_ =
      VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double min_segment_len_ =
      VehicleParamHelper::Instance()->GetParam().min_segment_len_for_parallel();
  double width_veh_ = VehicleParamHelper::Instance()->GetParam().width();
  double half_width_veh_ =
      VehicleParamHelper::Instance()->GetParam().width() * 0.5;
  double front_edge_to_center_ =
      VehicleParamHelper::Instance()->GetParam().front_edge_to_center();
  double back_edge_to_center_ =
      VehicleParamHelper::Instance()->GetParam().back_edge_to_center();

  int slot_sign_ = 1;  // 1:Right,(default),-1:Left

  double sin_target_point_theta_ = 0.0;
  double cos_target_point_theta_ = 0.0;
  PlanningPoint target_point_;
  PlanningPoint updated_target_point_;
  std::vector<planning_math::LineSegment2d> objects_map_line_segments_;
  std::vector<double> target_y_vec_;

  double front_shrink_dis_ =
      VehicleParamHelper::Instance()->GetParam().front_shrink_dis();
  double front_side_shrink_dis_ =
      VehicleParamHelper::Instance()->GetParam().front_side_shrink_dis();
  double rear_shrink_dis_ =
      VehicleParamHelper::Instance()->GetParam().rear_shrink_dis();
  double rear_side_shrink_dis_ =
      VehicleParamHelper::Instance()->GetParam().rear_side_shrink_dis();
};

} // namespace apa_planner
} // namespace planning