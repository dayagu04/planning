#ifndef __PERPENDICULAR_PATH_OUT_PLANNER_H__
#define __PERPENDICULAR_PATH_OUT_PLANNER_H__

#include "perpendicular_path_generator.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathOutPlanner : public PerpendicularPathGenerator {
 public:
  enum class PrePlanCase : uint8_t {
    FAIL,
    EGO_POSE,
    FIRST_MID_POINT,
    SECOND_MID_POINT,
    MOVE_TO_TARGET_LINE,
    TURN_AROUND,
    COUNT,
  };

  enum class PathColDetRes : uint8_t {
    INVALID,
    NORMAL,
    INSIDE_STUCK,
    SHORTEN,
    COUNT,
  };

  enum class PathPlanMethod {
    INVALID,
    ONE_ARC,
    TWO_ARC,
    LINE_ARC,
    COUNT,
  };
  struct PlannerParams {
    bool is_left_side = true;
    double slot_side_sgn = 1.0;

    PrePlanCase pre_plan_case = PrePlanCase::FAIL;

    bool should_prepare_second = false;
    bool should_prepare_third = false;
    bool first_multi_plan = true;
    bool complete_plan_again = false;
    bool single_plan_again = false;
    bool multi_plan = false;
    bool can_insert_line = true;

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

      pre_plan_case = PrePlanCase::FAIL;
      should_prepare_second = false;
      should_prepare_third = false;
      first_multi_plan = true;
      complete_plan_again = false;
      single_plan_again = false;
      multi_plan = false;
      can_insert_line = true;

      use_mono_tang = false;
      use_multi_tang = false;
      first_path_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

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

  // for debug
  struct DebugInfo {
    Eigen::Vector2d tag_point;
    double headingB = 0.0;

    std::vector<Eigen::Vector2d> center_vec;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_pt_vec;
  };

 public:
  virtual void Reset() override;
  virtual const bool Update() override;

  virtual const bool UpdateByPrePlan() override;

  const bool UpdatePyband(
      const Input &input,
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);

  const PlannerParams &GetCalcParams();

 private:
  virtual void Preprocess() override;

  // for park out simulation

  const bool PreparePlan();
  const bool PreparePlanOnce(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const double &y_offset, const double &radius, const uint8_t current_gear,
      const uint8_t current_arc_steer,
      pnc::geometry_lib::PathPoint current_pose);
  const bool AdjustPlan();
  const bool AdjustPlanOnce(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      pnc::geometry_lib::PathPoint &current_pose, const double safe_dist,
      const uint8_t current_arc_steer, const uint8_t current_gear);

  const bool STurnParallelPlan();
  const bool STurnParallelPlanOnce(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const double offset_y, const double radius,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const uint8_t current_arc_steer,
      const double steer_change_radius);

  const double CalOccupiedRatio(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckReachTargetPose(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckReachTargetPose();

  const bool CheckArcOrLineAvailable(
      const pnc::geometry_lib::LineSegment &line);

  const bool CheckArcOrLineAvailable(const pnc::geometry_lib::Arc &arc);

  // collision detect start
  const PathColDetRes TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg);
  const PathColDetRes TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg, const double safe_dist);
  // collision detect end
 private:
  PlannerParams calc_params_;
};

}  // namespace apa_planner
}  // namespace planning

#endif