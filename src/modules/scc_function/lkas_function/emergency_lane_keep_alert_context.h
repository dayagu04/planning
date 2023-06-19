#ifndef EMERGENCY_LANE_KEEP_ALERT_CONTEXT_H_
#define EMERGENCY_LANE_KEEP_ALERT_CONTEXT_H_
#include "lane_keep_assist_type.h"

namespace planning {
class EmergencyLaneKeepAlert {
 public:
  void Init(planning::LkasInput *lkas_input, framework::Session *session);
  void Update();
  uint8 RunOnce();
  boolean LeftAlertJudgeFL();
  boolean LeftAlertJudgeRL();
  boolean RightAlertJudgeFR();
  boolean RightAlertJudgeRR();
  boolean LeftAlertJudge();
  boolean RightAlertJudge();
  uint8 ObjSelect(planning::RadarObjData *radar);
  boolean ObjBsdFL(planning::RadarObjData *radar);
  boolean ObjLcaFL(planning::RadarObjData *radar);
  boolean ObjBsdFR(planning::RadarObjData *radar);
  boolean ObjLcaFR(planning::RadarObjData *radar);
  boolean ObjBsdRL(planning::RadarObjData *radar);
  boolean ObjLcaRL(planning::RadarObjData *radar);
  boolean ObjBsdRR(planning::RadarObjData *radar);
  boolean ObjLcaRR(planning::RadarObjData *radar);
  uint8 ObjBsdConditionF(planning::RadarObjData *radar);
  uint8 ObjBsdConditionR(planning::RadarObjData *radar);
  uint8 ObjLcaConditionF(planning::RadarObjData *radar);
  uint8 ObjLcaAlertF(planning::RadarObjData *radar);
  uint8 ObjLcaConditionR(planning::RadarObjData *radar);
  uint8 ObjLcaAlertR(planning::RadarObjData *radar);

  ~EmergencyLaneKeepAlert() = default;

 private:
  framework::Session *session_ = nullptr;
  // MeasurementPoint measurement_str;
  // CalibrationParameter calribration_str;
  planning::LkasInput *lkas_input_ = nullptr;
  planning::AeraVel f_bsd_aera_vel_;
  planning::AeraVel r_bsd_aera_vel_;
  planning::AeraVel f_lca_aera_vel_;
  planning::AeraVel r_lca_aera_vel_;
  // LkasInput *lkas_output;
};

}  // namespace planning
#endif