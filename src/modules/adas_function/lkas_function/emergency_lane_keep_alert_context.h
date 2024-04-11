#ifndef EMERGENCY_LANE_KEEP_ALERT_CONTEXT_H_
#define EMERGENCY_LANE_KEEP_ALERT_CONTEXT_H_
#include "lane_keep_assist_type.h"

namespace planning {
class EmergencyLaneKeepAlert {
 public:
  void Init(planning::LkasInput *lkas_input, framework::Session *session);
  uint8 RunOnce();
  ~EmergencyLaneKeepAlert() = default;

 private:
  void Update();
  bool LeftAlertJudgeFL();
  bool LeftAlertJudgeRL();
  bool RightAlertJudgeFR();
  bool RightAlertJudgeRR();
  bool LeftAlertJudge();
  bool RightAlertJudge();
  uint8 ObjSelect(planning::RadarObjData *radar);
  bool ObjBsdFL(planning::RadarObjData *radar);
  bool ObjLcaFL(planning::RadarObjData *radar);
  bool ObjBsdFR(planning::RadarObjData *radar);
  bool ObjLcaFR(planning::RadarObjData *radar);
  bool ObjBsdRL(planning::RadarObjData *radar);
  bool ObjLcaRL(planning::RadarObjData *radar);
  bool ObjBsdRR(planning::RadarObjData *radar);
  bool ObjLcaRR(planning::RadarObjData *radar);
  uint8 ObjBsdConditionF(planning::RadarObjData *radar);
  uint8 ObjBsdConditionR(planning::RadarObjData *radar);
  uint8 ObjLcaConditionF(planning::RadarObjData *radar);
  uint8 ObjLcaAlertF(planning::RadarObjData *radar);
  uint8 ObjLcaConditionR(planning::RadarObjData *radar);
  uint8 ObjLcaAlertR(planning::RadarObjData *radar);

 private:
  framework::Session *session_ = nullptr;
  // MeasurementPoint measurement_str;
  // CalibrationParameter calribration_str;
  planning::LkasInput *lkas_input_ = nullptr;
  planning::AeraVel f_bsd_aera_vel_;
  planning::AeraVel r_bsd_aera_vel_;
  planning::AeraVel f_lca_aera_vel_;
  planning::AeraVel r_lca_aera_vel_;
  double ego_curvature;
};

}  // namespace planning
#endif