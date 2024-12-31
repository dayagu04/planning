#pragma once

#include <functional>

#include "camera_preception_groundline_c.h"
#include "camera_preception_tsr_c.h"
#include "component_intf.h"
#include "control_command_c.h"
#include "ehr.pb.h"
#include "ehr_sdmap.pb.h"
#include "fm_info_c.h"
#include "func_state_machine_c.h"
#include "fusion_deceler_c.h"
#include "fusion_groundline_c.h"
#include "fusion_objects_c.h"
#include "fusion_occupancy_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "fusion_road_c.h"
#include "ifly_localization_c.h"
#include "struct_container.hpp"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "prediction_c.h"
#include "uss_perception_info_c.h"
#include "uss_wave_info_c.h"
#include "vehicle_service_c.h"

namespace iflyauto {
namespace interface {
class PlanningInterface : public ComponentInterface {
 public:
  PlanningInterface() = default;
  virtual ~PlanningInterface() = default;

  virtual bool Init() = 0;
  virtual bool Proc() = 0;

  virtual void Feed_IflytekControlControlCommand(
      const iflyauto::ControlOutput& data) = 0;
  virtual void Feed_IflytekFusionObjects(
      const iflyauto::FusionObjectsInfo& data) = 0;
  virtual void Feed_IflytekFusionOccupancyObjects(
      const iflyauto::FusionOccupancyObjectsInfo& data) = 0;
  virtual void Feed_IflytekFusionRoadFusion(const iflyauto::RoadInfo& data) = 0;
  virtual void Feed_IflytekFusionGroundLine(
      const iflyauto::FusionGroundLineInfo& data) = 0;
  virtual void Feed_IflytekFusionSpeedBump(
      const iflyauto::FusionDecelerInfo& data) = 0;
  virtual void Feed_IflytekLocalizationEgomotion(
      const iflyauto::IFLYLocalization& data) = 0;
  virtual void Feed_IflytekPredictionPredictionResult(
      const iflyauto::PredictionResult& data) = 0;
  virtual void Feed_IflytekVehicleService(
      const iflyauto::VehicleServiceOutputInfo& data) = 0;
  virtual void Feed_IflytekFusionParkingSlot(
      const iflyauto::ParkingFusionInfo& data) = 0;
  virtual void Feed_IflytekCameraPerceptionTrafficSignRecognition(
      const iflyauto::CameraPerceptionTsrInfo& data) = 0;
  virtual void Feed_IflytekFsmSocState(
      const iflyauto::FuncStateMachine& data) = 0;
  virtual void Feed_IflytekUssUsswaveInfo(
      const iflyauto::UssWaveInfo& data) = 0;
  virtual void Feed_IflytekUssUssPerceptionInfo(
      const iflyauto::UssPerceptInfo& data) = 0;
  virtual void Feed_IflytekEhrSdmapInfo(const SdMapSwtx::SdMap& data) = 0;
  virtual void Feed_IflytekEhrStaticMap(const Map::StaticMap& data) = 0;

  virtual void RegWriter_IflytekPlanningPlan(
      const std::function<void(const iflyauto::PlanningOutput&)>& writer) = 0;
  virtual void RegWriter_IflytekPlanningHmi(
      const std::function<void(const iflyauto::PlanningHMIOutputInfoStr&)>&
          writer) = 0;
  virtual void RegWriter_IflytekPlanningDebugInfo(
      const std::function<void(const iflyauto::StructContainer &)>&
          writer) = 0;

  virtual void RegFmWriter_IflytekAlarmInfoPlanning(
      const std::function<void(const iflyauto::FmInfo&)>& fm_writer) = 0;
};
}  // namespace interface
}  // namespace iflyauto
