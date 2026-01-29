#include "meb_box_collision_lib.h"

#include <cmath>

using namespace planning;
namespace adas_function {

void BoxCollisonLib::Init() { return; }

bool BoxCollisonLib::GetCollisionResultBySimEgoDec(
    CalculateETTCInputStr& info, double time_t0, double time_end,
    double sim_step_time, double time_before_aeb_dec, double aeb_dec_acc) {
  // 外部输入的信息(time=0时刻)
  float32 ego_width =
      info.ego_width;  // 本车的宽度(左右轮胎外侧相距的距离) 单位：m
  float32 ego_length =
      info.ego_length;  // 本车的长度（前保至后保之间的距离） 单位：m
  float32 ego_backshaft_2_fbumper =
      info.ego_backshaft_2_fbumper;  // 本车后轴中心与前保中心之间的距离 单位:m
  float32 ego_v_x_t0 = info.ego_v_x;  // 本车实际车速 单位:m/s
  float32 ego_a_x_t0 = info.ego_a_x;  // 本车实际加速度 单位:m/ss
  float32 ego_radius =
      info.ego_radius;  // 本车行驶的弯道半径 单位:m 方向:左正右负
  float32 obj_x_t0 = info.obj_x;  // 目标物几何中心的x坐标 单位:m
  float32 obj_y_t0 = info.obj_y;  // 目标物几何中心的y坐标 单位:m
  float32 obj_v_x_t0 = info.obj_v_x;  // 目标物沿x轴方向的速度 单位:m/s
  float32 obj_v_y_t0 = info.obj_v_y;  // 目标物沿y轴方向的速度 单位:m/s
  float32 obj_a_x_t0 = info.obj_a_x;  // 目标物沿x轴方向的加速度 单位:m/ss
  float32 obj_a_y_t0 = info.obj_a_y;  // 目标物沿y轴方向的加速度 单位:m/ss
  float32 obj_width = info.obj_width;    // 目标物的宽度 单位：m
  float32 obj_length = info.obj_length;  // 目标物的长度 单位：m
  float32 obj_heading_angle_t0 =
      info.obj_heading_angle;  // 目标物的航向角(与x轴正方向的夹角) 单位:rad
                               // 方向:左正右负

  float32 time = time_t0;  // 仿真时间 单位:s
  if (time < 0.0) {
    time = 0.0;
  } else {
    // do nothing
  }
  float32 time_max = time_end;  // 最大仿真时间 单位:s
  if (time_max < 0.1) {
    time_max = 0.1;
  } else if (time_max > 7.0) {
    time_max = 7.0;
  } else {
    // do nothing
  }

  float32 ego_x = 0.0F;     // 本车后轴中心的x坐标(time时刻) 单位:m
  float32 ego_y = 0.0F;     // 本车后轴中心的y坐标(time时刻) 单位:m
  float32 ego_move = 0.0F;  // 本车移动的欧式距离(time时刻) 单位：m
  Box2DStr ego_box;         // 本车的二维矩形结构体定义

  float32 obj_x = obj_x_t0;  // 目标物几何中心的x坐标(time时刻) 单位:m
  float32 obj_y = obj_y_t0;  // 目标物几何中心的y坐标(time时刻) 单位:m
  boolean obj_direction;  // 目标物的运动方向 FALSE:与本车同向  TRUE:与本车对向
  if (obj_v_x_t0 < 0.0F) {
    obj_direction = TRUE;
  } else {
    obj_direction = FALSE;
  }
  float32 obj_move_time_max = time_max;  // 目标物处于运动状态的最大时间 单位：s
  if ((obj_a_x_t0 < 0.0F) && (fabs(obj_v_x_t0 / obj_a_x_t0) < time_max) &&
      obj_direction == FALSE) {
    obj_move_time_max = fabs(obj_v_x_t0 / obj_a_x_t0);
  }
  Box2DStr obj_box;  // 目标物的二维矩形结构体定义
  obj_box.input.heading_angle = info.obj_heading_angle;
  obj_box.input.sin_heading_angle =
      MySinRad(obj_box.input.heading_angle);  // sin(heading_angle)
  obj_box.input.cos_heading_angle =
      MyCosRad(obj_box.input.heading_angle);  // cos(heading_angle)
  obj_box.input.length = info.obj_length;     // box的长度 单位：m
  obj_box.input.width = info.obj_width;       // box的宽度 单位：m
  obj_box.input.x = info.obj_x;               // box几何中心的x坐标
  obj_box.input.y = info.obj_y;               // box几何中心的y坐标
  BoxCornersCoordinateUpdate(&obj_box);       // 更新box角点坐标

  boolean collision_flag = FALSE;  // FALSE:未发生碰撞 TRUE:发生碰撞(检测到过)

  // 设置搜索的time范围
  float32 ettc;
  int sim_count_max = time_max / sim_step_time;

  float32 sim_step_ego_v = ego_v_x_t0;  // 当前仿真时刻本车速度
  bool update_ego_move_flag = true;
  for (int i = 0; i < sim_count_max; i++) {
    // 本车所处的位置(time时刻)
    if (time < time_before_aeb_dec) {
      sim_step_ego_v += ego_a_x_t0 * sim_step_time;
    } else {
      sim_step_ego_v += aeb_dec_acc * sim_step_time;
    }
    // 修正本车车速
    if (sim_step_ego_v * ego_v_x_t0 < 0.0) {
      update_ego_move_flag = false;
    }
    if (time < time_before_aeb_dec) {
      ego_move += 0.5F * ego_a_x_t0 * sim_step_time * sim_step_time +
                  sim_step_ego_v * sim_step_time;
    } else {
      if (update_ego_move_flag == true) {
        ego_move += 0.5F * aeb_dec_acc * sim_step_time * sim_step_time +
                    sim_step_ego_v * sim_step_time;
      } else {
        ego_move = 0.0;
      }
    }
    ego_box.input.heading_angle =
        ego_move /
        ego_radius;  // 航向角(与x轴正方向的夹角) 单位:rad 方向:左正右负
    ego_box.input.sin_heading_angle =
        MySinRad(ego_box.input.heading_angle);  // sin(heading_angle)
    ego_box.input.cos_heading_angle =
        MyCosRad(ego_box.input.heading_angle);  // cos(heading_angle)
    ego_box.input.length = ego_length;          // box的长度 单位：m
    ego_box.input.width = ego_width;            // box的宽度 单位：m
    ego_x =
        ego_radius *
        ego_box.input
            .sin_heading_angle;  // 依据转弯半径,计算本车后轴中心所处的纵向坐标(time时刻),单位：m
    ego_y =
        ego_radius *
        (1.0F -
         ego_box.input
             .cos_heading_angle);  // 依据转弯半径,计算本车后轴中心所处的横向坐标(time时刻),单位：m
    ego_box.input.x =
        ego_x + (ego_backshaft_2_fbumper - 0.5F * ego_box.input.length) *
                    ego_box.input.cos_heading_angle;  // box几何中心的x坐标
    ego_box.input.y =
        ego_y + (ego_backshaft_2_fbumper - 0.5F * ego_box.input.length) *
                    ego_box.input.sin_heading_angle;  // box几何中心的y坐标

    // 目标物所处的位置(time时刻)
    if (time <= obj_move_time_max) {
      obj_x = obj_x_t0 + 0.5F * obj_a_x_t0 * time * time +
              obj_v_x_t0 *
                  time;  // 计算目标物几何中心所处的纵向坐标(time时刻),单位：m
      // obj_y = obj_y_t0 + 0.5F * obj_a_y_t0 * time * time + obj_v_y_t0 * time;
      // // 计算目标物几何中心所处的横向坐标(time时刻),单位：m
    } else {
      obj_x =
          obj_x_t0 + 0.5F * obj_a_x_t0 * obj_move_time_max * obj_move_time_max +
          obj_v_x_t0 *
              obj_move_time_max;  // 计算目标物几何中心所处的纵向坐标(time时刻),单位：m
      // obj_y = obj_y_t0 + 0.5F * obj_a_y_t0 * obj_move_time_max *
      // obj_move_time_max + obj_v_y_t0 * obj_move_time_max; //
      // 计算目标物几何中心所处的横向坐标(time时刻),单位：m
    }
    obj_y = obj_y_t0 + 0.5F * obj_a_y_t0 * time * time +
            obj_v_y_t0 *
                time;  // 计算目标物几何中心所处的横向坐标(time时刻),单位：m
    obj_box.input.heading_angle =
        obj_heading_angle_t0;  // 航向角(与x轴正方向的夹角) 单位:rad
                               // 方向:左正右负
    obj_box.input.sin_heading_angle =
        MySinRad(obj_box.input.heading_angle);  // sin(heading_angle)
    obj_box.input.cos_heading_angle =
        MyCosRad(obj_box.input.heading_angle);  // cos(heading_angle)
    obj_box.input.length = obj_length;          // box的长度 单位：m
    obj_box.input.width = obj_width;            // box的宽度 单位：m
    obj_box.input.x = obj_x;                    // box几何中心的x坐标
    obj_box.input.y = obj_y;                    // box几何中心的y坐标

    // 碰撞检测
    collision_flag = BoxCollisionDetection(&ego_box, &obj_box);

    // 判断是否检测到碰撞发生
    if (collision_flag) {
      break;  // 检测到碰撞发生,结束循环
    } else {
      // do nothing
    }

    // 更新下一次循环的time
    time = time + sim_step_time;

    // 判断下一次循环的time是否在搜索范围内
    if (time > time_max) {
      break;  // 检测到time超出搜索范围,结束循环
    } else {
      // do nothing
    }
  }

  // 循环结束
  return collision_flag;

  return false;
}

// 函数功能：碰撞检测。依据超平面分离定理,判断两个box是否相交
// 输出接口:FALSE:不相交 TRUE:相交
bool BoxCollisonLib::BoxCollisionDetection(Box2DStr* box1, Box2DStr* box2) {
  // 更新box1的四个角点坐标信息
  BoxCornersCoordinateUpdate(box1);

  // 更新box2的四个角点坐标信息
  BoxCornersCoordinateUpdate(box2);

  // 使用AABB box判断两个box是否相交
  if ((box1->state.max_x < box2->state.min_x) ||
      (box1->state.max_y < box2->state.min_y) ||
      (box1->state.min_x > box2->state.max_x) ||
      (box1->state.min_y > box2->state.max_y)) {
    return FALSE;
  }

  // 使用分离轴判断两个box是否相交
  float32 shift_x = box2->input.x - box1->input.x;
  float32 shift_y = box2->input.y - box1->input.y;
  float32 dx1 = 0.5F * box1->input.length * box1->input.cos_heading_angle;
  float32 dy1 = 0.5F * box1->input.length * box1->input.sin_heading_angle;
  float32 dx2 = 0.5F * box1->input.width * box1->input.sin_heading_angle;
  float32 dy2 = 0.5F * box1->input.width * box1->input.cos_heading_angle;
  float32 dx3 = 0.5F * box2->input.length * box2->input.cos_heading_angle;
  float32 dy3 = 0.5F * box2->input.length * box2->input.sin_heading_angle;
  float32 dx4 = 0.5F * box2->input.width * box2->input.sin_heading_angle;
  float32 dy4 = 0.5F * box2->input.width * box2->input.cos_heading_angle;
  bool box_collision_flag = false;
  box_collision_flag = fabs(shift_x * box1->input.cos_heading_angle +
                            shift_y * box1->input.sin_heading_angle) <=
                           fabs(dx3 * box1->input.cos_heading_angle +
                                dy3 * box1->input.sin_heading_angle) +
                               fabs(dx4 * box1->input.cos_heading_angle -
                                    dy4 * box1->input.sin_heading_angle) +
                               0.5F * box1->input.length &&
                       fabs(shift_x * box1->input.sin_heading_angle -
                            shift_y * box1->input.cos_heading_angle) <=
                           fabs(dx3 * box1->input.sin_heading_angle -
                                dy3 * box1->input.cos_heading_angle) +
                               fabs(dx4 * box1->input.sin_heading_angle +
                                    dy4 * box1->input.cos_heading_angle) +
                               0.5F * box1->input.width &&
                       fabs(shift_x * box2->input.cos_heading_angle +
                            shift_y * box2->input.sin_heading_angle) <=
                           fabs(dx1 * box2->input.cos_heading_angle +
                                dy1 * box2->input.sin_heading_angle) +
                               fabs(dx2 * box2->input.cos_heading_angle -
                                    dy2 * box2->input.sin_heading_angle) +
                               0.5F * box2->input.length &&
                       fabs(shift_x * box2->input.sin_heading_angle -
                            shift_y * box2->input.cos_heading_angle) <=
                           fabs(dx1 * box2->input.sin_heading_angle -
                                dy1 * box2->input.cos_heading_angle) +
                               fabs(dx2 * box2->input.sin_heading_angle +
                                    dy2 * box2->input.cos_heading_angle) +
                               0.5F * box2->input.width;
  return box_collision_flag;
}

/*MySinRad函数功能:
使用泰勒展开法替代<math.h>中的sin函数，解决计算量太大造成程序卡死问题
输入接口：
(1)angle_rad:角度 单位:rad 范围:-4pi 到 4pi
输出接口：
sin(angle_rad)
*/
float32 BoxCollisonLib::MySinRad(float32 angle_rad) {
  float32 x = angle_rad;
  float32 pi = 3.14159265358979323846F;
  float32 two_pi = 2.0F * pi;

  // 将x值等效至[-4.0*pi,4.0*pi]之间
  float32 two_pi_times;  // angle_rad/(2*pi)取整的结果
  two_pi_times = (int)(angle_rad / two_pi + 0.5);
  x = angle_rad - two_pi_times * two_pi;

  if ((x > 4.0 * pi) && (x < -4.0 * pi)) {
    return 0.0F;
  }

  if (isnan(x)) {
    return 0.0F;
  }

  // 将x值的范围控制在[0,2*pi]之间
  while (x > two_pi)  // 大于360度
  {
    x = x - two_pi;
  }
  while (x < 0.0F)  // 小于0度
  {
    x = x + two_pi;
  }

  // 泰勒展开公式(x有效输入范围为[-0.5*pi,0.5*pi])
  // sin x = x - x^3/3! + x^5/5! - x^7/7! +……+(-1)^(k-1)*(x^(2k-1))/(2k-1)!
  // cos x = 1 - x^2/2! + x^4/4! - x^6/6! +……+(-1)^(k)*(x^(2k))/(2k)!
  float32 result_sign = 1.0F;
  if ((x >= 0.0F) && (x <= (pi / 2.0F))) {
    //[0,90]deg
    result_sign = 1.0F;
  } else if (x <= (pi * 3.0F / 2.0F)) {
    //(90,270]deg
    // sin(pi+x) = -sin(x)
    x = x - pi;  //(-90,90]
    result_sign = -1.0F;
  } else {
    //(270,360]deg
    // sin(two_pi+x) = sin(x)
    x = x - two_pi;  //(-90,0]
    result_sign = 1.0F;
  }

  float32 x_2 = x * x;
  float32 x_3 = x_2 * x;
  float32 x_5 = x_3 * x_2;
  float32 x_7 = x_5 * x_2;
  float32 result = 0.0F;
  float32 result_temp = 0.0F;
  result_temp = x - x_3 / 6.0F + x_5 / 120.0F - x_7 / 5040.0F;
  result = result_temp * result_sign;
  return result;
}

/*MyCosRad函数功能:
使用泰勒展开法替代<math.h>中的cos函数，解决计算量太大造成程序卡死问题
输入接口：
(1)angle_rad:角度 单位:rad 范围:-4pi 到 4pi
输出接口：
sin(angle_rad)
*/
float32 BoxCollisonLib::MyCosRad(float32 angle_rad) {
  float32 x = angle_rad;
  float32 pi = 3.14159265358979323846F;
  float32 two_pi = 2.0F * pi;

  // 将x值等效至[-4.0*pi,4.0*pi]之间
  float32 two_pi_times;  // angle_rad/(2*pi)取整的结果
  two_pi_times = (int)(angle_rad / two_pi + 0.5);
  x = angle_rad - two_pi_times * two_pi;

  if ((x > 4.0 * pi) && (x < -4.0 * pi)) {
    return 0.0F;
  }

  if (isnan(x)) {
    return 0.0F;
  }

  // 将x值的范围控制在[0,2*pi]之间
  while (x > two_pi)  // 大于360度
  {
    x = x - two_pi;
  }
  while (x < 0.0F)  // 小于0度
  {
    x = x + two_pi;
  }

  // 泰勒展开公式(x有效输入范围为[-0.5*pi,0.5*pi])
  // sin x = x - x^3/3! + x^5/5! - x^7/7! +……+(-1)^(k-1)*(x^(2k-1))/(2k-1)!
  // cos x = 1 - x^2/2! + x^4/4! - x^6/6! +……+(-1)^(k)*(x^(2k))/(2k)!
  float32 result_sign = 1.0F;
  if ((x >= 0.0F) && (x <= (pi / 2.0F))) {
    //[0,90]deg
    result_sign = 1.0F;
  } else if (x <= (pi * 3.0F / 2.0F)) {
    //(90,270]deg
    // cos(pi+x) = -cos(x)
    x = x - pi;  //(-90,90]
    result_sign = -1.0F;
  } else {
    //(270,360]deg
    // cos(two_pi+x) = cos(x)
    x = x - two_pi;  //(-90,0]
    result_sign = 1.0F;
  }

  float32 x_2 = x * x;
  float32 x_4 = x_2 * x_2;
  float32 x_6 = x_4 * x_2;
  float32 x_8 = x_6 * x_2;
  float32 result = 0.0F;
  float32 result_temp = 0.0F;
  result_temp = 1.0F - x_2 / 2.0F + x_4 / 24.0F - x_6 / 720.0F + x_8 / 40320.0F;
  result = result_temp * result_sign;
  return result;
}

}  // namespace adas_function