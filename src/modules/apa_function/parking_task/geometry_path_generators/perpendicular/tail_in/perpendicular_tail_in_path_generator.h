#ifndef __PERPENDICULAR_PATH_IN_PLANNER_H__
#define __PERPENDICULAR_PATH_IN_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "collision_detection/collision_detector_interface.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
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

  enum class DubinsPlanResult : uint8_t {
    SUCCESS,
    PATH_COLLISION,
    NO_PATH,
  };

  enum class PlanRequest : uint8_t {
    ROUGH_PATH,
    ONE_STEP_PATH,
    OPTIMAL_PATH,
  };

  enum class PathColDetRes : uint8_t {
    INVALID,
    NORMAL,
    SHORTEN,
    COUNT,
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
    bool multi_plan = false;

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
    uint8_t first_path_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

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
      multi_plan = false;

      first_path_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

      pt_inside.setZero();

      target_line.Reset();

      mono_safe_circle.Reset();
      multi_safe_circle.Reset();

      safe_circle_tang_pt.Reset();

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
  virtual const bool CheckCurrentGearLength() override;

  // for simulation
  const bool ItervativeUpdatePb(
      const GeometryPathInput &input,
      const std::shared_ptr<CollisionDetectorInterface>
          &collision_detector_interface_ptr);

  const PlannerParams &GetCalcParams();

 private:
  // member function
  virtual void Preprocess() override;

  const PathColDetRes TrimPathByObs(geometry_lib::PathSegment &path_seg,
                                    const double lat_inflation,
                                    const double lon_safe_dist,
                                    const bool enable_log = true,
                                    const bool need_cal_obs_dist = false,
                                    const bool use_edt_col = true);

  const bool CalTurnAroundPose();

  const bool TurnAround();

  const bool MonoPreparePlan(Eigen::Vector2d &tag_point);
  void CalMonoSafeCircle();
  const bool CheckMonoIsFeasible();
  const bool MultiPreparePlan(Eigen::Vector2d &tag_point);
  bool CalMultiSafeCircle();
  // prepare plan end

  const bool PreparePathPlan();

  const bool PreparePathSecondPlan();

  const bool PrepareSinglePathPlan(
      const pnc::geometry_lib::PathPoint &cur_pose,
      std::vector<
          std::pair<geometry_lib::GeometryPath, geometry_lib::GeometryPath>>
          &pair_geometry_path_vec);

  const DubinsPlanResult DubinsPathPlan(
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
                                      const bool enable_log = false,
                                      const bool ego_pose_flag = false);

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

  const bool DubinsPathPlan(const geometry_lib::PathPoint &pose,
                            const uint8_t ref_gear, const double lat_buffer,
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

  const double CalOccupiedRatio(const geometry_lib::PathPoint &current_pose);

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

  const bool CheckReachTargetPose(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckReachTargetPose();

  PlannerParams calc_params_;
};

}  // namespace apa_planner
}  // namespace planning

#endif