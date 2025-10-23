#ifndef MEB_CORE_H_
#define MEB_CORE_H_

#include "adas_function_context.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "meb_box_collision_lib.h"
using namespace planning;
namespace adas_function {
namespace meb_core {

struct LineSegMent {
  Eigen::Vector2d pA = Eigen::Vector2d::Zero();
  Eigen::Vector2d pB = Eigen::Vector2d::Zero();
  double length = 0.0;
};

struct ArcSegMent {
  Eigen::Vector2d pA = Eigen::Vector2d::Zero();
  Eigen::Vector2d pB = Eigen::Vector2d::Zero();
  Eigen::Vector2d circle_center = Eigen::Vector2d::Zero();
  double circle_radius = 0.0;
  double length = 0.0;
};

struct MebResult {
  Eigen::Vector2d nearest_point = {0.0, 0.0};
  bool collision_flag = false;
  double remain_dist = 10.0;
  double remain_obstacle_dist = 10.0;
  double remain_path_distance = 10.0;
  double relative_distance_min = 10.0;
  double acc_min = 10.0;
  double ttc_min = 10.0;
};
struct MebParameters {
  double enable_vehspd_display_min =
      2.0 / 3.6;  // 激活的最小仪表车速，单位：m/s
  double enable_vehspd_display_max =
      10.0 / 3.6;  // 激活的最大仪表车速，单位：m/s
  double disable_vehspd_display_min =
      1.0 / 3.6;  // 退出的最小仪表车速，单位：m/s
  double disable_vehspd_display_max =
      12.0 / 3.6;  // 退出的最大仪表车速，单位：m/s
  double predict_t = 1.0;
  // double earliest_warning_line = 1.5;  // 触发的最早报警线，单位：m
  // double latest_warning_line = -0.3;   // 触发的最晚报警线，单位：m
  // double reset_warning_line = 0.15;    // 触发的报警重置线，单位：m
  // double supp_turn_light_recovery_time = 2.0;  // 转向灯抑制恢复时长，单位：s
  // double warning_time_min = 1.0;  // 单次最大报警时长，单位：s
  // double warning_time_max = 2.0;  // 单次最大报警时长，单位：s
  double brake_pedal_pos = 20.0;

  /*
      /-1----2----3----4-\
     /                    \
    0                      5
    |                      |
    |                      |
    |                      |
    |                      |
    11                     6
     \                    /
      \-10---9----8----7-/

  */

  std::vector<double> uss_radar_x_vector = {3.245,  3.58,   3.725, 3.725,
                                            3.58,   3.245,  -0.5,  -0.96,
                                            -1.064, -1.064, -0.96, -0.5};
  std::vector<double> uss_radar_y_vector = {0.941,  0.694,  0.331,  -0.331,
                                            -0.694, -0.941, -0.912, -0.731,
                                            -0.328, 0.328,  0.731,  0.912};
  double uss_rabar_angle_range_rad = 0.5;
  std::vector<int> uss_pos_index_vector = {0, 1, 2, 3, 4,  5,
                                           6, 7, 8, 9, 10, 11};
  bool dynamic_uss_collision_switch = true;
  double dynamic_uss_collision_ttc = 1.5;
  // double meb_dis_buffer = 0.35;
};

enum MebInnerState {
  MEB_STATE_INIT = 0,     // Init
  MEB_STATE_OFF = 1,      // Off
  MEB_STATE_STANDBY = 2,  // Standby
  MEB_STATE_PASSIVE = 3,  // passive
  MEB_STATE_ACTIVE = 4,   // Active
  MEB_STATE_FAULT = 5,    // Fault
};


/*
TODO:
1.是否判断后视镜展开；
TODO:
2.是否限定一定时间内触发次数20s；
TODO:
3.meb触发后保压时间是否限定，Tmax = 2s
*/
class MebCore {
 public:
  void RunOnce(void);
  MebCore() { Init(); }

 private:
  void Init(void);
  bool UpdateMebMainSwitch(void);
  uint32 UpdateMebEnableCode(void);
  uint32 UpdateMebDisableCode(void);
  uint32 UpdateMebFaultCode(void);
  bool UpdateIntervention(void);
  uint32 UpdateMebKickdownCode(void);
  MebInnerState MebStateMachine(void);
  void SetMebOutputInfo(void);
  void Log(void);

  const bool IsPointInZone(Eigen::Vector2d point);
  void UpdateObstacles(void);
  void CollisionCalculate(void);
  void UpdateUssDistanceFunction();
  double CollisionCalculateAcc(double remain_dist);
  void CollisionOdBoxCalculate();

  const MebResult LineCollisionUpdateCommon(const LineSegMent& line_seg,
                                        const double heading_start,std::vector<Eigen::Vector2d> & obs_vec);
  const MebResult ArcCollisionUpdateCommon(const ArcSegMent& arc,
                                       const double heading_start,std::vector<Eigen::Vector2d> & obs_vec);
  const MebResult CollisionUpdateUss();

  MebInnerState meb_inner_state_;
  iflyauto::MEBFunctionFSMWorkState meb_state_;
  bool meb_main_switch_ = false;
  uint32 meb_enable_code_ = 255;
  uint32 meb_disable_code_ = 255;
  uint32 meb_fault_code_ = 255;
  uint32 meb_kickdown_code_ = 255;

  bool meb_intervention_flag_ = false;
  double meb_obs_collision_flag_ = false;
  // uint32 meb_request_status_;
  // double meb_request_value_;  // 单位:m/ss
  double meb_radius_;
  BoxCollisonLib box_collision_;
  MebParameters meb_param_;
  MebResult od_meb_result_;
  MebResult occ_meb_result_;
  MebResult uss_meb_result_;
  bool od_box_collision_flag_ = false;
  bool meb_state_machine_init_flag_ = false;
  Eigen::Vector2d predict_point_;
  std::vector<Eigen::Vector2d> occ_obs_vec_;  // 自车坐标系下的障碍物
  std::vector<Eigen::Vector2d> od_obs_vec_;   // 自车坐标系下的障碍物
  std::vector<Eigen::Vector2d> uss_obs_vec_;  // 自车坐标系下的障碍物
  std::vector<double> uss_distance_vec_;      // 每个超声波扇形距离
  std::vector<double> uss_acc_vec_;           // 每个超声波扇形距离
  std::vector<Eigen::Vector2d> car_with_mirror_polygon_vertex_;
  std::vector<pnc::geometry_lib::LineSegment> car_line_local_vec_;
  double meb_zone_x1_ = 10.0;
  double meb_zone_x2_ = 10.0;
  double meb_zone_y1_ = 10.0;
  double meb_zone_y2_ = 10.0;
};

}  // namespace meb_core
}  // namespace adas_function
#endif