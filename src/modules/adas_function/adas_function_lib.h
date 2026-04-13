#ifndef ADAS_FUNCTION_LIB_H_
#define ADAS_FUNCTION_LIB_H_
#include <iostream>

#include "Platform_Types.h"
#include "adas_function_context.h"
#include "math.h"
#include "math_lib.h"

namespace adas_function {
// 1. 定义一个专门的结构体来描述“关键障碍物”

enum class OdObjGroup { kPeople, kCar, kMotor, kDefault };

struct MebTempObj {
  float rel_x;
  float rel_y;
  float rel_heading_angle;
  float rel_vx;
  float rel_vy;
  float abs_v;
  float abs_vx;
  float rel_acc_x;
  float rel_acc_y;
  float width;
  float length;
  float height;
  float conf;
  float age;
  uint16_t track_id;
  uint16_t index;
  int fusion_source;
  OdObjGroup type_for_meb;
  int type;
  int interest_code;
  bool is_collision;
  uint64_t suppe_code;
  double stop_distance_buffer;
  double an_avoid_by_steering;
  double ay_avoid_by_accelerating;
  double collision_point_y;  // 障碍物横穿到 自车前or后时，障碍物中心的横向位置
};

struct InterestObjInfo {
  int valid_num;
  std::vector<MebTempObj> interest_obj_vec_;
};

struct WarningLevelInfo {
  bool alert_level;
  float alert_level_ettc_threshold;
  int alert_level_supp_code;
};

extern uint16 uint16_bit[16];
extern uint32 uint32_bit[32];

/*二维矩形结构体定义
坐标系:
(1)原点为本车后轴中心
(2)本车前进方向为x轴正方向
(3)本车前进方向左侧为y轴正方向*/
typedef struct Box2DInputStr {
  float32 heading_angle;  // 航向角(与x轴正方向的夹角) 单位:rad 方向:左正右负
  float32 sin_heading_angle;  // sin(heading_angle)
  float32 cos_heading_angle;  // cos(heading_angle)
  float32 length;             // box的长度 单位：m
  float32 width;              // box的宽度 单位：m
  float32 x;                  // box几何中心的x坐标
  float32 y;                  // box几何中心的y坐标
} Box2DInputStr;
typedef struct Box2DStateStr {
  float32 fl_x;   // 前左角点的x坐标
  float32 fl_y;   // 前左角点的y坐标
  float32 fr_x;   // 前右角点的x坐标
  float32 fr_y;   // 前右角点的y坐标
  float32 rl_x;   // 后左角点的x坐标
  float32 rl_y;   // 后左角点的y坐标
  float32 rr_x;   // 后右角点的x坐标
  float32 rr_y;   // 后右角点的y坐标
  float32 max_x;  // 所有角点的x坐标最大值
  float32 min_x;  // 所有角点的x坐标最小值
  float32 max_y;  // 所有角点的y坐标最大值
  float32 min_y;  // 所有角点的y坐标最小值
} Box2DStateStr;
typedef struct Box2DStr {
  Box2DInputStr input;
  Box2DStateStr state;
} Box2DStr;

extern void BoxCornersCoordinateUpdate(Box2DStr *box);

extern void CalProjectionPointByNewtonIteration(
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline, const double s_start,
    const double s_end, const Eigen::Vector2d &x, double &s_proj);

extern double LkasLineLeftIntervention(double tlc_to_line_threshold);

extern double LkasLineRightIntervention(double tlc_to_line_threshold);

extern double LkasRoadedgeLeftIntervention(double tlc_to_line_threshold,
                                           double roadedge_offset);

extern double LkasRoadedgeRightIntervention(double tlc_to_line_threshold,
                                            double roadedge_offset);

extern void PreviewEgoPosisation(double tlc_to_line_threshold,
                                 std::vector<double> &ego_box_predict_vector);

extern void CalProjectionPointForLong(const pnc::mathlib::spline &x_s_spline,
                                      const pnc::mathlib::spline &y_s_spline,
                                      const double s_start, const double s_end,
                                      const Eigen::Vector2d &x, double &s_proj);

extern std::vector<double> ObjCornersCalculate(
    const context::FusionObjExtractInfo &obj);

extern double integer_power(const double x, int order);

extern void leastSquareFitting(const std::vector<double>& points_x_vec,
                               const std::vector<double>& points_y_vec,
                               const int order,
                               adas_function::context::LineInfo* line_info_ptr);

extern void leastSquareFittingForRoadedge(
    const std::vector<double>& points_x_vec,
    const std::vector<double>& points_y_vec, const int order,
    adas_function::context::RoadedgeInfo* line_info_ptr);

extern OdObjGroup GetOdObjGroup(const iflyauto::ObjectType type);

extern int32 min_int32(int32 x, int32 y);

extern float32 min_float(float32 x, float32 y);

extern int32 max_int32(int32 x, int32 y);

extern float32 max_float(float32 x, float32 y);

extern float32 GetEgoCurvature(float32 v, float32 yaw_rate, float32 steer_angle,
                               float32 steer_ratio, float32 wheel_base);

extern float32 MySinRad(float32 angle_rad);

extern float32 MyCosRad(float32 angle_rad);

}  // namespace adas_function

#endif