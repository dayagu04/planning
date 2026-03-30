#ifndef MEB_BOX_COLLISION_LIB_H_
#define MEB_BOX_COLLISION_LIB_H_
#include <vector>
#include "adas_function_context.h"
#include "adas_function_lib.h"

using namespace planning;
namespace adas_function {

/*box collision calculate*/
typedef struct CalculateETTCInputStr {
  double ego_width;  // 本车的宽度(左右轮胎外侧相距的距离) 单位：m
  double ego_length;  // 本车的长度（前保至后保之间的距离） 单位：m
  double ego_backshaft_2_fbumper;  // 本车后轴中心与前保中心之间的距离 单位:m
  double ego_v_x;                  // 本车实际车速 单位:m/s
  double ego_a_x;                  // 本车实际加速度 单位:m/ss
  double ego_radius;  // 本车行驶的弯道半径 单位:m 方向:左正右负
  double obj_x;       // 目标物几何中心的x坐标 单位:m
  double obj_y;       // 目标物几何中心的y坐标 单位:m
  double obj_v_x;     // 目标物沿x轴方向的绝对速度 单位:m/s
  double obj_v_y;     // 目标物沿y轴方向的绝对速度 单位:m/s
  double obj_a_x;     // 目标物沿x轴方向的绝对加速度 单位:m/ss
  double obj_a_y;     // 目标物沿y轴方向的绝对加速度 单位:m/ss
  double obj_width;   // 目标物的宽度 单位：m
  double obj_length;  // 目标物的长度 单位：m
  double obj_heading_angle;  // 目标物的航向角(与x轴正方向的夹角) 单位:rad
                             // 方向:左正右负
} CalculateETTCInputStr;

struct DebugFrame {
  double time;
  double ego_x;
  double ego_y;
  double ego_heading;
  double ego_v;
  double ego_a;
  double obj_x;
  double obj_y;
  double obj_heading;
  double obj_vx;
  double obj_vy;
  bool collision;
};

class BoxCollisonLib {
 public:
  BoxCollisonLib() { Init(); };
  CalculateETTCInputStr boxs_info_;
  double t_start;
  double t_end;
  double dec_request;
  double time_dealy;
  double time_step;

  std::vector<DebugFrame> debug_trace_;

  bool GetCollisionResultBySimEgoDec(CalculateETTCInputStr &info,
                                     double time_t0, double time_end,
                                     double sim_step_time,
                                     double time_before_aeb_dec,
                                     double aeb_dec_acc);

  // 1.假定障碍物维持当前运动状态做匀速直线运动
  // 2.假定本车维持当前加速度
  /*
  3.输入:
  (1)ego_steer_angle:当前时刻本车方向盘转角 单位:rad 方向:左正右负
  (2)ego_steer_angle_speed:假定本车转向避撞维持的方向盘转速
  单位:rad/s 方向:左正右负
  (3)double ego_steer_angle_max_abs:方向盘最大转角
  */
  // 4.输出:
  // (1)按照假定的本车与障碍物的运动趋势,判断是否会发生碰撞
  bool GetCollisionResultBySimEgoSteerAngleSpeed(
      CalculateETTCInputStr &info, double time_t0, double time_end,
      double sim_step_time, double steer_ratio, double wheelbase,
      double ego_steer_angle, double ego_steer_angle_speed,
      double ego_steer_angle_max_abs);

 private:
  void Init();
  bool BoxCollisionDetection(Box2DStr *box1, Box2DStr *box2);
};

}  // namespace adas_function
#endif