#pragma once

#include <memory>
// #include <string>
#include <vector>

#include "common/ifly_time.h"

#include "modules/context/environmental_model.h"
#include "modules/scenario/scenario_manager.h"
#include "common/utils_math.h"

#include "framework/session.h"
#include "framework/scheduler.h"
#include "modules/common/local_view.h"

#include "modules/common/define/debug_output.h"
#include "../res/include/proto/planning_plan.pb.h"

namespace planning {

class GeneralPlanning {

 public:
  GeneralPlanning();
  virtual ~GeneralPlanning();

  void Init();
  bool RunOnce(const LocalView& local_view,
               PlanningOutput::PlanningOutput* const planning_output,
               DebugOutput &debug_info,
               PlanningHMI::PlanningHMIOutputInfoStr &planning_hmi_Info);
//   void ResetState() override;

 private:
  // 解析障碍物
  void FillPredictionTrajectoryPoint(
      const std::vector<Prediction::PredictionTrajectoryPoint> &input,
      Prediction::PredictionTrajectory &output);

  void FillPredictionTrajectory(
      const std::vector<Prediction::PredictionTrajectory> &input,
      Prediction::PredictionObject &output);

  void FillPredictionObjectInfo(const std::shared_ptr<Prediction::PredictionResult> &prediction_result_raw,
                                const std::shared_ptr<FusionObjects::FusionObjectsInfo> &fusion_objects_info_raw,
                                const std::shared_ptr<RadarPerceptionObjects::RadarPerceptionObjectsInfo> &radar_objects_raw,
                                Prediction::PredictionResult &output_prediction_ojects);

  void UpdateChassisReport(double current_time);
  void UpdateWheelReport(double current_time);
  void UpdateUodyReport(double current_time);
  void UpdateEgoPose(double current_time);
  void UpdateVehicleStatus(double current_time);
  
  void FillEgoPlanningTrajectoryPoint(
      const std::vector<Prediction::PredictionTrajectoryPoint> &input,
      std::vector<planning::common::TrajectoryPoint> &output);

  void FillEgoPlanningTrajectory(
      const Prediction::PredictionTrajectory &pred_traj,
      planning::common::EgoPredictionTrajectory &ego_prediction_traj);
  
  void FillEgoPlanningInfo(
      const std::shared_ptr<Prediction::PredictionObject>
          &prediction_object,
      planning::common::EgoPredictionObject &ego_prediction_info);

  void FillControlInfo();
  void FillFusionRoadInfo();
  void UpdatePredictionInfo(double current_time);
  void UpdateFusionObjectInfo(double current_time);

  void FillPlanningTrajectory(double start_time, PlanningOutput::PlanningOutput *const planning_output);

  void GenerateStopTrajectory(double start_time, PlanningOutput::PlanningOutput *const planning_output);

  void FillPlanningDebugInfo(double start_time, DebugOutput &debug_info);
  void FillPlanningHmiInfo(double start_timestamp, PlanningHMI::PlanningHMIOutputInfoStr &planning_hmi_Info);
 private:
//   std::shared_ptr<EnvironmentalModel> environmental_model_ = nullptr;
//   std::shared_ptr<ScenarioManager> scenario_manager_;
  // planning::framework::Frame *frame_ = nullptr;
  planning::framework::Session session_;
  planning::framework::Scheduler scheduler_;
  bool hdmap_valid_ = false;
  bool reset_pnc_{false};
  bool last_can_run_{false};
  int hdmap_valid_count_;

 protected:
  enum FeedType {
    FEED_VEHICLE_DBW_STATUS = 0,
    FEED_EGO_VEL,
    FEED_EGO_STEER_ANGLE,
    FEED_EGO_ENU,
    FEED_WHEEL_SPEED_REPORT,
    FEED_EGO_ACC,
    FEED_MISC_REPORT,
    FEED_MAP_INFO,
    FEED_FUSION_INFO,
    FEED_PREDICTION_INFO,
    FEED_FUSION_LANES_INFO,
    FEED_TYPE_MAX,
  };

 protected:
  LocalView local_view_;
  double last_feed_time_[FEED_TYPE_MAX]{};
};

}  // namespace planning