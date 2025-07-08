#pragma once

#include <cstdint>

#include "adas_function.h"
#include "apa_function/apa_function.h"
#include "common/define/debug_output.h"
#include "common/local_view.h"
#include "common_platform_type_soc.h"
#include "config_context.h"
#include "environmental_model_manager.h"
#include "hpp_function/hpp_function.h"
#include "noa_function/noa_function.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "rads_function/rads_function.h"
#include "scc_function/scc_function.h"
#include "session.h"
namespace planning {

class PlanningScheduler {
 public:
  PlanningScheduler(const LocalView *const local_view,
                    const common::EngineConfiguration *const engine_config_ptr);
  virtual ~PlanningScheduler();

  void Init(const common::EngineConfiguration *const engine_config_ptr);
  bool RunOnce(iflyauto::PlanningOutput *const planning_output,
               iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info);

  void SyncParameters(planning::common::SceneType scene_type);
  //   void ResetState() override;
  planning::framework::Session *MutableSession() { return &session_; }

  uint64_t FaultCode();

 private:
  // 解析障碍物
  void FillPredictionTrajectoryPoint(
      const std::vector<iflyauto::PredictionTrajectoryPoint> &input,
      iflyauto::PredictionTrajectory &output);

  void FillPredictionTrajectory(
      const std::vector<iflyauto::PredictionTrajectory> &input,
      iflyauto::PredictionObject &output);

  void FillPredictionObjectInfo(
      const std::shared_ptr<iflyauto::PredictionResult> &prediction_result_raw,
      const std::shared_ptr<iflyauto::FusionObjectsInfo>
          &fusion_objects_info_raw,
      iflyauto::PredictionResult &output_prediction_ojects);

  void UpdateChassisReport(double current_time);
  void UpdateWheelReport(double current_time);
  void UpdateUodyReport(double current_time);
  void UpdateEgoPose(double current_time);
  void UpdateVehicleStatus(double current_time);

  void FillEgoPlanningTrajectoryPoint(
      const std::vector<iflyauto::PredictionTrajectoryPoint> &input,
      std::vector<planning::common::TrajectoryPoint> &output);

  void FillEgoPlanningTrajectory(
      const iflyauto::PredictionTrajectory &pred_traj,
      planning::common::EgoPredictionTrajectory &ego_prediction_traj);

  void FillEgoPlanningInfo(
      const std::shared_ptr<iflyauto::PredictionObject> &prediction_object,
      planning::common::EgoPredictionObject &ego_prediction_info);

  void FillControlInfo();
  void FillFusionRoadInfo();
  void UpdatePredictionInfo(double current_time);
  void UpdateFusionObjectInfo(double current_time);

  void FillPlanningTrajectory(double start_time,
                              iflyauto::PlanningOutput *planning_output);

  double ComputeBoundOfReferenceIntercept();

  void GenerateStopTrajectory(double start_time,
                              iflyauto::PlanningOutput *planning_output);

  void FillPlanningHmiInfo(
      double start_timestamp,
      iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info);

  void FillPlanningRequest(iflyauto::RequestLevel request,
                           iflyauto::PlanningOutput *const planning_output);

  void ClearParkingInfo(iflyauto::PlanningOutput *planning_output);
  bool IsUndefinedScene(const iflyauto::FunctionalState &current_state);
  bool IsValidHppState(const iflyauto::FunctionalState &current_state);
  bool IsValidRadsState(const iflyauto::FunctionalState &current_state);

  void InitSccFunction();

  void interpolate_with_last_trajectory_points();

  bool UpdateFailedPlanningResult();

  bool UpdateSuccessfulPlanningResult();

  // hpp routing state: need apa search slot
  // hpp searching state: need apa search slot.
  bool IsHppSlotSearchingByDistance();

  planning::common::SceneType DetermineSceneType(const iflyauto::FuncStateMachine &func_state_machine);

  // parking function: APA, RPA, HPP, AVP
  const bool ExcuteParkingFunction(const common::SceneType function_type,
      iflyauto::PlanningOutput *const planning_output);

  // Navigation function: NOA, SCC, LCC, HPP
  const bool ExcuteNavigationFunction(
      const common::SceneType function_type, const double start_timestamp,
      iflyauto::PlanningOutput *const planning_output,
      iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info);

 private:
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

  double last_feed_time_[FEED_TYPE_MAX]{};

  planning::framework::Session session_;
  GeneralPlanningConfig config_;
  bool hdmap_valid_ = false;
  bool last_can_run_{false};
  int hdmap_valid_count_;

  std::unique_ptr<BaseFunction> hpp_function_ = nullptr;
  std::unique_ptr<BaseFunction> noa_function_ = nullptr;
  std::unique_ptr<BaseFunction> scc_function_ = nullptr;
  std::unique_ptr<BaseFunction> apa_function_ = nullptr;
  std::unique_ptr<BaseFunction> rads_function_;
  std::unique_ptr<BaseFunction> adas_function_ = nullptr;

  planner::EnvironmentalModelManager environmental_model_manager_;

  const LocalView *const local_view_;
};

}  // namespace planning