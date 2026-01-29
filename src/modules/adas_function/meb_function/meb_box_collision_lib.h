#ifndef MEB_BOX_COLLISION_LIB_H_
#define MEB_BOX_COLLISION_LIB_H_
#include "adas_function_context.h"
#include "adas_function_lib.h"

using namespace planning;
namespace adas_function {

/*box collision calculate*/
typedef struct CalculateETTCInputStr {
  double ego_width; // 本车的宽度(左右轮胎外侧相距的距离) 单位：m
  double ego_length; // 本车的长度（前保至后保之间的距离） 单位：m
  double ego_backshaft_2_fbumper; // 本车后轴中心与前保中心之间的距离 单位:m
  double ego_v_x;                 // 本车实际车速 单位:m/s
  double ego_a_x;                 // 本车实际加速度 单位:m/ss
  double ego_radius; // 本车行驶的弯道半径 单位:m 方向:左正右负
  double obj_x;      // 目标物几何中心的x坐标 单位:m
  double obj_y;      // 目标物几何中心的y坐标 单位:m
  double obj_v_x;    // 目标物沿x轴方向的绝对速度 单位:m/s
  double obj_v_y;    // 目标物沿y轴方向的绝对速度 单位:m/s
  double obj_a_x;    // 目标物沿x轴方向的绝对加速度 单位:m/ss
  double obj_a_y;    // 目标物沿y轴方向的绝对加速度 单位:m/ss
  double obj_width;  // 目标物的宽度 单位：m
  double obj_length; // 目标物的长度 单位：m
  double obj_heading_angle; // 目标物的航向角(与x轴正方向的夹角) 单位:rad
                             // 方向:左正右负
} CalculateETTCInputStr;

// typedef struct Box2DInputStr {
//   float32 heading_angle; // 航向角(与x轴正方向的夹角) 单位:rad 方向:左正右负
//   float32 sin_heading_angle; // sin(heading_angle)
//   float32 cos_heading_angle; // cos(heading_angle)
//   float32 length;            // box的长度 单位：m
//   float32 width;             // box的宽度 单位：m
//   float32 x;                 // box几何中心的x坐标
//   float32 y;                 // box几何中心的y坐标
// } Box2DInputStr;
// typedef struct Box2DStateStr {
//   float32 fl_x;  // 前左角点的x坐标
//   float32 fl_y;  // 前左角点的y坐标
//   float32 fr_x;  // 前右角点的x坐标
//   float32 fr_y;  // 前右角点的y坐标
//   float32 rl_x;  // 后左角点的x坐标
//   float32 rl_y;  // 后左角点的y坐标
//   float32 rr_x;  // 后右角点的x坐标
//   float32 rr_y;  // 后右角点的y坐标
//   float32 max_x; // 所有角点的x坐标最大值
//   float32 min_x; // 所有角点的x坐标最小值
//   float32 max_y; // 所有角点的y坐标最大值
//   float32 min_y; // 所有角点的y坐标最小值
// } Box2DStateStr;
// typedef struct Box2DStr {
//   Box2DInputStr input;
//   Box2DStateStr state;
// } Box2DStr;


class BoxCollisonLib{
public:
BoxCollisonLib() {Init();};
CalculateETTCInputStr boxs_info_;
double t_start;
double t_end;
double dec_request;
double time_dealy;
double time_step;
bool GetCollisionResultBySimEgoDec(CalculateETTCInputStr &info,
                                      double time_t0,
                                      double time_end,
                                      double sim_step_time,
                                      double time_before_aeb_dec,
                                      double aeb_dec_acc);
private:
void Init();
bool BoxCollisionDetection(Box2DStr *box1, Box2DStr *box2);
float32 MyCosRad(float32 angle_rad);
float32 MySinRad(float32 angle_rad);


};

}  // namespace adas_function
#endif