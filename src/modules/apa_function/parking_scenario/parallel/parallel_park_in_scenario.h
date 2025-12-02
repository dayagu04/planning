#ifndef __PARALLEL_PARK_IN_PLANNER_H__
#define __PARALLEL_PARK_IN_PLANNER_H__

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include "apa_world.h"
#include "dubins_lib.h"
#include "parallel_path_generator.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "src/modules/apa_function/apa_common/relative_loc_observer_manager.h"

namespace planning {
namespace apa_planner {

struct AngleResult {
  double ang;
  bool res;

  AngleResult() = default;
  AngleResult(double ang_, bool res_) : ang(ang_), res(res_) {}

  bool operator<(const AngleResult& other) const { return ang < other.ang; }
};

class ParallelParkInScenario : public ParkingScenario {
 public:
  ParallelParkInScenario() = default;
  ParallelParkInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }

  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); };
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const bool GenTlane() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool CheckFinished() override;
  virtual const bool GenObstacles() override;
  virtual void ExcutePathPlanningTask() override;
  virtual void ScenarioTry() override;
  virtual void Log() const override;

  const double CalcSlotOccupiedRatio(const Eigen::Vector2d& terminal_err,
                                     const double slot_width,
                                     const bool is_right_side) const;
  const double CalcSlotOccupiedRatio(
      const pnc::geometry_lib::PathPoint start_pose) const;

  void GenTBoundaryObstacles();

  const Tlane& GetTlane() { return t_lane_; }

  virtual const double CalRealTimeBrakeDist() override;
  const bool IsPointInOrientedRectangle(const Eigen::Vector2d& point,
                                        const Eigen::Vector2d& rect_center,
                                        const double rect_heading,
                                        const double rect_length,
                                        const double rect_width);
  SlotCoord AlignAndMoveSlotToLine(const SlotCoord& slot,
                                   const Eigen::Vector2d& ego_pos,
                                   const Eigen::Vector2d& ego_head, double k,
                                   double b, double slot_width,
                                   double distance = 0.2);
  const bool LineFittingWithRansac(Eigen::Vector2d& line_coeffs,
                                   const std::vector<Eigen::Vector2d>& points,
                                   std::vector<bool>& inliers);
  const bool GetOffsetLineCoeffsWithEgoPose(
      Eigen::Vector2d& line_coeffs_out, const Eigen::Vector2d& line_coeffs_in,
      const std::vector<Eigen::Vector2d>& points,
      const std::vector<bool>& inliers, const Eigen::Vector2d& ego_pose);

  const bool GeneralPASlot();
  const bool ParkInTry(const ApaSlot& slot);
  const bool CheckPAFinished();

  const bool UpdatePASlotInfo();
  void ExtractLongestLineSegmentPointsPCA(
      const std::vector<Eigen::Vector2d>& obs_pos,
      std::vector<Eigen::Vector2d>& pa_curb_obs, Eigen::Vector2d& line_coeffs,
      double distance_threshold = 0.2);
  void EnablePAPark() {
    enable_pa_park_ = true;
    return;
  }
  void RecordDebugPaInfo(const Eigen::Vector2d& line_coeffs,
                         const SlotCoord& slot,
                         const std::vector<Eigen::Vector2d>& pa_curb_obs);
  void UpdatePARemainDistance();
  SlotCoord slot_new_;
  Eigen::Vector2d line_coeffs_out_;

 private:
  // virtual func
  void CalStaticBufferInDiffSteps(double& lat_buffer,
                                  double& safe_uss_remain_dist) const;

  void CalDynamicBufferInDiffSteps(double& dynaminc_lat_buffer,
                                   double& dynamic_lon_buffer) const;

  const bool CheckOneReverseToSlot();
  bool CheckReplanParallel();
  const GeometryPathOutput& SuitablePathReplan();
  void CheckEgoPoseWhenPlanFaild(ParkingFailReason reason);

  const double UpdateRemainDistObs(const double remain_dist_path,
                                   const double remain_dist_obs);
  const bool PostProcessPathPara();

  Tlane t_lane_;
  std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_pt_local_vec_;
  ParallelPathGenerator parallel_path_planner_;
  ParallelPathGenerator previous_parallel_path_planner_;
  int parallel_replan_again_ = 0;
  std::vector<pnc::geometry_lib::PathPoint>
      previous_current_path_point_global_vec_;
  std::deque<double> previous_remain_dist_obs;
  bool enable_pa_park_ = false;
  ApaSlot first_plan_slot;
  Eigen::Vector2d first_line_coeffs_;
  double slot2curb_dist_;
  pnc::geometry_lib::PathPoint first_plan_cur_pos;
  std::unordered_map<size_t, std::vector<pnc::geometry_lib::PathPoint>>
      multi_parkin_path_vec_;

  RelativeLocObserverManager relative_loc_observer_manager_;
  std::unordered_map<size_t, std::set<AngleResult>> try_bound_map_;
  bool delay_check_finish_ = false;
};

}  // namespace apa_planner
}  // namespace planning

#endif
