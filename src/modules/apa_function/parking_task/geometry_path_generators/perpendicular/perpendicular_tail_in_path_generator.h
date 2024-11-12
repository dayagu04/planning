#ifndef __PERPENDICULAR_PATH_IN_PLANNER_H__
#define __PERPENDICULAR_PATH_IN_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "collision_detection/collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "perpendicular_path_generator.h"
#include "planning_plan_c.h"

namespace planning {
namespace apa_planner {
using namespace pnc;
class PerpendicularTailInPathGenerator : public PerpendicularPathGenerator {
 public:
  enum class PrePlanCase : uint8_t {
    FAIL,
    EGO_POSE,
    MID_POINT,
    FIRST_MID_POINT,
    SECOND_MID_POINT,
    MOVE_TO_TARGET_LINE,
    TURN_AROUND,
    COUNT,
  };

  enum class PathColDetRes : uint8_t {
    INVALID,
    NORMAL,
    SINGLE_PLAN_AGAIN,
    COMPLETE_PLAN_AGAIN,
    SHORTEN,
    COUNT,
  };

  enum class PathPlanMethod {
    INVALID,
    ONE_ARC,
    TWO_ARC,
    LINE_ARC,
    ONE_LINE,
    COUNT,
  };

  enum class DubinsPlanResult {
    SUCCESS,
    PATH_COLLISION,
    NO_PATH,
  };

  enum class PlanRequest {
    ROUGH_PATH,
    ONE_STEP_PATH,
    OPTIMAL_PATH,
  };

  struct PlannerParams {
    bool is_left_side = true;
    double slot_side_sgn = 1.0;

    double strict_car_lat_inflation = 0.0;
    double strict_col_lon_safe_dist = 0.0;

    bool is_searching_stage = false;

    std::vector<geometry_lib::PathPoint> tange_pose_vec;

    PrePlanCase pre_plan_case = PrePlanCase::FAIL;

    PlanRequest plan_request;

    bool should_prepare_second = false;
    bool should_prepare_third = false;
    bool first_multi_plan = true;
    bool complete_plan_again = false;
    bool single_plan_again = false;
    bool multi_plan = false;
    bool can_insert_line = true;

    double col_det_time = 0.0;

    double dubins_plan_time = 0.0;
    double rough_plan_time = 0.0;

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
      is_searching_stage = false;
      is_left_side = true;
      slot_side_sgn = 1.0;

      strict_car_lat_inflation = 0.0;
      strict_col_lon_safe_dist = 0.0;

      col_det_time = 0.0;
      dubins_plan_time = 0.0;
      rough_plan_time = 0.0;

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

      tange_pose_vec.clear();
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
  virtual const bool CheckCurrentGearLength() override;

  void ExtendCurrentFollowLastPath(double extend_distance);
  void InsertLineSegAfterCurrentFollowLastPath(double extend_distance);

  const std::vector<double> GetMinSafeCircle() const;

  // for simulation
  const bool UpdatePb(
      const Input &input,
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);

  const bool ItervativeUpdatePb(
      const Input &input,
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);

  const bool PreparePlanPb();

  const bool PreparePlanSecondPb();

  const bool GenPathOutputByDubinsPb();

  const bool MultiPlanPb();

  const bool AdjustPlanPb();

  const bool CheckReachTargetPosePb();

  const PlannerParams &GetCalcParams();

 private:
  // member function
  virtual void Preprocess() override;

  // new prepare plan
  const bool PreparePathPlan();

  const bool PrepareSinglePathPlan(
      const pnc::geometry_lib::PathPoint &cur_pose,
      std::vector<geometry_lib::GeometryPath> &geometry_path_vec);

  const bool PreparePathPlanSecond();

  const bool UpdatePath();

  const DubinsPlanResult DubinsPathPlan(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose, const double turn_radius,
      const double min_length, const bool need_col_det,
      geometry_lib::GeometryPath &geometry_path);

  const bool IsPathSafe(const pnc::geometry_lib::PathSegment &path_seg,
                        const double lat_inflation, const double lon_safe_dist);

  const bool IsGeometryPathSafe(
      const pnc::geometry_lib::GeometryPath &geometry_path,
      const double lat_inflation, const double lon_safe_dist);

  const PathColDetRes TrimPathByObs(geometry_lib::PathSegment &path_seg,
                                    const double lat_inflation,
                                    const double lon_safe_dist,
                                    const bool enable_log = true);

  // prepare plan start
  const bool PreparePlan();
  const bool PreparePlanOnce(const double x_offset, const double heading_offset,
                             const double radius);

  const bool CalTurnAroundPose();

  const bool TurnAround();

  const bool DubinsPlan(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose, const double turn_radius,
      const double min_length, const bool need_col_det,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool PreparePlanSecond();

  const bool GenPathOutputByDubins();
  const bool MonoPreparePlan(Eigen::Vector2d &tag_point);
  void CalMonoSafeCircle();
  const bool CheckMonoIsFeasible();
  const bool MultiPreparePlan(Eigen::Vector2d &tag_point);
  bool CalMultiSafeCircle();
  // prepare plan end

  const bool NewUpdatePathPlan();

  const bool NewPreparePathPlan();

  const bool NewPreparePathSecondPlan();

  const bool NewPrepareSinglePathPlan(
      const pnc::geometry_lib::PathPoint &cur_pose,
      std::vector<
          std::pair<geometry_lib::GeometryPath, geometry_lib::GeometryPath>>
          &pair_geometry_path_vec);

  const DubinsPlanResult NewDubinsPathPlan(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose, const double turn_radius,
      const double min_length, const uint8_t max_gear_change_count,
      const uint8_t ref_gear, const bool need_col_det,
      geometry_lib::GeometryPath &geometry_path);

  const DubinsPlanResult RSPathPlan(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose, const double turn_radius,
      const double min_length, const uint8_t max_gear_change_count,
      const bool need_col_det, geometry_lib::GeometryPath &geometry_path);

  const bool MultiAdjustPathPlan(const geometry_lib::PathPoint &pose,
                                 const uint8_t ref_gear,
                                 const PlanRequest plan_request);

  const bool RoughMultiAdjustPathPlan(const geometry_lib::PathPoint &pose,
                                      const uint8_t ref_gear,
                                      geometry_lib::GeometryPath &ahead_path,
                                      const bool enable_log = false);

  const bool OneStepMultiAdjustPathPlan(const geometry_lib::PathPoint &pose,
                                        const uint8_t ref_gear,
                                        geometry_lib::GeometryPath &ahead_path,
                                        const bool enable_log = false);

  const bool OptimalMultiAdjustPathPlan(const geometry_lib::PathPoint &pose,
                                        const uint8_t ref_gear,
                                        geometry_lib::GeometryPath &ahead_path);

  const bool SingleMultiAdjustPathPlan(
      const geometry_lib::PathPoint &pose, const uint8_t ref_gear,
      const double lat_buffer, const double lon_buffer,
      std::vector<geometry_lib::GeometryPath> &geometry_path_vec,
      const bool enable_log = true);

  const bool OneArcPathPlan(const geometry_lib::PathPoint &pose,
                            const uint8_t ref_gear, const double lat_buffer,
                            const double lon_buffer,
                            geometry_lib::GeometryPath &geometry_path,
                            const bool enable_log = true);

  const bool LineArcPathPlan(const geometry_lib::PathPoint &pose,
                             const uint8_t ref_gear, const double lat_buffer,
                             const double lon_buffer,
                             geometry_lib::GeometryPath &geometry_path,
                             const bool same_gear = true,
                             const bool enable_log = true);

  const bool TwoArcPathPlan(const geometry_lib::PathPoint &pose,
                            const uint8_t ref_gear, const double lat_buffer,
                            const double lon_buffer,
                            geometry_lib::GeometryPath &geometry_path,
                            const bool same_gear = false,
                            const bool enable_log = true);

  const bool AlignAndSTurnPathPlan(const geometry_lib::PathPoint &pose,
                                   const uint8_t ref_gear,
                                   const double lat_buffer,
                                   const double lon_buffer,
                                   geometry_lib::GeometryPath &geometry_path,
                                   const bool enable_log = true);

  const bool OneLinePathPlan(const geometry_lib::PathPoint &pose,
                             const uint8_t gear, const double lat_buffer,
                             const double lon_buffer,
                             const double last_seg_reverse_length,
                             geometry_lib::GeometryPath &geometry_path,
                             const bool enable_log = true);

  const bool InsertLineInGeometryPath(const double lat_buffer,
                                      const double lon_buffer,
                                      const uint8_t ref_gear,
                                      const double insert_length,
                                      geometry_lib::GeometryPath &geometry_path,
                                      const bool enable_log = true);

  const bool ConstructReverseVaildPathSeg(geometry_lib::PathSegment &seg1,
                                          geometry_lib::PathSegment &seg2,
                                          const double lat_buffer,
                                          const double lon_buffer,
                                          const bool enable_log = true);

  const bool OneArcPathPlan(
      const pnc::geometry_lib::PathPoint &pose,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);
  //   const bool LineArcPathPlan();
  const bool TwoArcPathPlan(
      const pnc::geometry_lib::PathPoint &pose,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);
  //   const bool OneLinePathPlan();

  // multi plan start
  const bool CheckMultiPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose,
      const double &slot_occupied_ratio);

  const bool MultiPlan();
  const bool CalSinglePathInMulti(
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const uint8_t current_arc_steer,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const double turn_radius, size_t i);

  const bool OneArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool TwoArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool LineArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool OneLinePlan(
      pnc::geometry_lib::LineSegment &line,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear);
  // multi plan end

  // adjust plan start
  const bool CheckAdjustPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose,
      const double slot_occupied_ratio = 0.0);

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
  // adjust plan end

  // collision detect start
  const PathColDetRes TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg,
      CollisionDetector::CollisionResult *pcol_res = nullptr);
  const PathColDetRes TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg, const double safe_dist,
      CollisionDetector::CollisionResult *pcol_res = nullptr);
  // collision detect end

  const bool CheckArcOrLineAvailable(const pnc::geometry_lib::Arc &arc);
  const bool CheckArcOrLineAvailable(
      const pnc::geometry_lib::LineSegment &line);

  const bool CheckPathIsNormal(const pnc::geometry_lib::PathSegment &path_seg);

  const bool CheckReachTargetPose(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckReachTargetPose();

  const double CalOccupiedRatio(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckTwoPoseInCircle(const Eigen::Vector2d &ego_pos0,
                                  const double ego_heading0,
                                  const Eigen::Vector2d &ego_pos1,
                                  const double ego_heading1,
                                  const Eigen::Vector2d &center) const;

  const bool IsRightCircle(const pnc::geometry_lib::PathPoint &ego_pose,
                           const Eigen::Vector2d &center) const;

  const bool IsRightCircle(const Eigen::Vector2d &ego_pos,
                           const double ego_heading,
                           const Eigen::Vector2d &center) const;

  PlannerParams calc_params_;
};

}  // namespace apa_planner
}  // namespace planning

#endif