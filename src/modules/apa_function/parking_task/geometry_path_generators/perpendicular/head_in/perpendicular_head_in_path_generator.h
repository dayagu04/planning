#ifndef __PERPENDICULAR_PATH_HEADING_IN_PLANNER_H__
#define __PERPENDICULAR_PATH_HEADING_IN_PLANNER_H__

#include "perpendicular_path_generator.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathHeadingInPlanner : public PerpendicularPathGenerator {
 public:
  struct PlannerParams {
    bool is_left_side = true;
    double slot_side_sgn = 1.0;

    bool multi_plan = false;
    bool can_insert_line = true;
    bool origin_prepare_plan_success = true;

    double turn_radius = 5.5;

    size_t adjust_fail_count = 0;

    pnc::geometry_lib::LineSegment target_line;

    pnc::geometry_lib::Circle mono_safe_circle;
    pnc::geometry_lib::Circle multi_safe_circle;

    Eigen::Vector2d pt_inside;

    pnc::geometry_lib::PathPoint safe_circle_tang_pt;
    bool cal_tang_pt_success = false;
    bool directly_use_ego_pose = false;
    uint8_t first_path_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

    // headin params
    double search_length_arc_first_method = 0.0;
    bool use_origin_multiplan = false;
    bool use_line_arc = false;
    bool is_outside_occupied = false;
    bool is_inside_occupied = false;
    double target_line_y_offset_sign = 0.0;
    bool verify_mono_circle_headin = false;
    bool use_mono_circle_headin = false;
    pnc::geometry_lib::PathPoint second_prepareplan_input;
    bool use_adjust = false;
    int prepare_point_condition = 0;

    bool use_mono_tang = false;
    bool use_multi_tang = false;
    pnc::geometry_lib::LineSegment prepare_line;  // pA is tag point
    Eigen::Vector2d pre_line_tangent_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d pre_line_normal_vec = Eigen::Vector2d::Zero();

    void Reset() {
      is_left_side = true;
      slot_side_sgn = 1.0;

      turn_radius = 5.5;

      adjust_fail_count = 0;

      multi_plan = false;
      can_insert_line = true;
      origin_prepare_plan_success = true;

      use_mono_tang = false;
      use_multi_tang = false;
      first_path_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

      // headin
      search_length_arc_first_method = 0.0;
      use_origin_multiplan = false;
      target_line_y_offset_sign = 0.0;
      use_line_arc = false;
      is_outside_occupied = false;
      is_inside_occupied = false;
      verify_mono_circle_headin = false;
      use_mono_circle_headin = false;
      second_prepareplan_input.Reset();
      use_adjust = false;
      prepare_point_condition = 0;  // reverse

      pt_inside.setZero();

      target_line.Reset();

      mono_safe_circle.Reset();
      multi_safe_circle.Reset();

      safe_circle_tang_pt.Reset();
      cal_tang_pt_success = false;
      directly_use_ego_pose = false;

      pre_line_tangent_vec.setZero();
      pre_line_normal_vec.setZero();
    }
  };

  enum class PathColDetRes : uint8_t {
    INVALID,
    NORMAL,
    SINGLE_PLAN_AGAIN,
    COMPLETE_PLAN_AGAIN,
    SHORTEN,
    COUNT,
  };

 public:
  virtual void Reset() override;
  virtual const bool Update() override;
  const bool Update(const std::shared_ptr<CollisionDetector>
                        &collision_detector_ptr) override;
  void InsertLineSegAfterCurrentFollowLastPath(double extend_distance);
  // simulation
  void PreprocessForSimu();
  const bool UpdatePb();
  const bool PreparePlanPybind();
  const bool GenPathOutputByDubinsPybind();
  const bool MultiPlanPybind();
  const bool MultiLineArcPlanPybind();
  const bool AdjustPlanPybind();
  const PerpendicularPathHeadingInPlanner::PlannerParams &GetCalcParams();
  const bool CheckReachTargetPosePybind();

 private:
  virtual void Preprocess() override;
  // prepare plan
  const bool TurnIntoSlotFirst();
  const bool PreparePlanArcFirst();
  const bool PreparePlanArcFirstTry(
      const pnc::geometry_lib::PathPoint start_pose);

  const bool PreparePlan();
  const bool ComputePreparePointSecond(
      pnc::dubins_lib::DubinsLibrary::Input &input,
      const pnc::geometry_lib::PathPoint &target_pose);

  const bool PreparePlanOnce(const double &x_offset,
                             const double &heading_offset,
                             const double &radius);

  const bool MonoPreparePlan(Eigen::Vector2d &tag_point);
  void CalMonoSafeCircle();
  const bool CheckMonoIsFeasible();

  const bool MultiPreparePlan(Eigen::Vector2d &tag_point);
  const bool CalMultiSafeCircle();

  // multi plan
  const bool MultiPlan();
  const bool CheckMultiPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose,
      const double &slot_occupied_ratio);

  const bool CalSinglePathInMulti(
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const uint8_t current_arc_steer,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const double turn_radius, size_t i);

  const bool CheckReachTargetPose(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool OneLinePlan(
      pnc::geometry_lib::LineSegment &line,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear);

  const bool TwoArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool CheckArcOrLineAvailable(const pnc::geometry_lib::Arc &arc);

  const bool OneArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool LineArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool CheckArcOrLineAvailable(
      const pnc::geometry_lib::LineSegment &line);

  // multi line arc plan
  const bool MultiLineArcPlan();

  const bool CheckAdjustPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose);

  // adjust plan
  const bool AdjustPlan();
  const bool CalSinglePathInAdjust(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t &current_gear, const double &steer_change_ratio,
      const double &steer_change_radius, const size_t &i);

  const bool OneArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool LineArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const double turn_radius);

  const bool AlignBodyPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool STurnParallelPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const double steer_change_ratio1,
      const double steer_change_radius);

  // some utils
  const bool CheckReachTargetPose();
  const double CalOccupiedRatio(
      const pnc::geometry_lib::PathPoint &current_pose);

  // collison detection
  const PathColDetRes TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg);

  const PathColDetRes TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg, const double safe_dist);

  // path postprocess
  const bool GenPathOutputByDubins();

 private:
  PlannerParams calc_params_;
};

}  // namespace apa_planner
}  // namespace planning

#endif