#include "jerk_limited_trajectory.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>

Point jerk_limited_trajectory::velocityTargetSolver(const extremum &extrem,
                                                    const double &v_0,
                                                    const double &a_0,
                                                    const double &v_des) {
  extremum_ = extrem;
  Point Point_;
  double v_end = 0.0;
  double a_cruise = 0.0;
  int dir_a = 1;
  if (a_0 >= 0) {
    v_end = v_0 + a_0 * std::fabs(a_0 / extremum_.j_min) /
                      2;  // a_0以匀jerk方式降到0的过程中，v的变化量
  } else {
    v_end = v_0 + a_0 * std::fabs(a_0 / extremum_.j_max) / 2;
  }

  // compare v_des with v_end
  // if v_des > v_end : we need a positive cruise direction for acceleration;
  // otherwise, a negative cruise direction is needed
  ((v_des - v_end) > 0) ? dir_a = 1
                        : (((v_des - v_end) < 0) ? dir_a = -1 : dir_a = 0);

  (dir_a == 1) ? a_cruise = extremum_.a_max
               : ((dir_a == -1) ? a_cruise = extremum_.a_min : a_cruise = 0);

  double delt_a = a_cruise - a_0;
  double t1 = 0.0;
  double delt_v_1 = 0.0;
  if (delt_a >= 0) {
    t1 = delt_a / extremum_.j_max;
    Point_.j1 = extremum_.j_max;
  } else {
    t1 = delt_a / extremum_.j_min;
    Point_.j1 = extremum_.j_min;
  }
  delt_v_1 = v_0 + a_0 * t1 + Point_.j1 * std::pow(t1, 2) / 2;

  double t3 = 0.0;
  if (-a_cruise >= 0) {
    t3 = -a_cruise / extremum_.j_max;
    Point_.j3 = extremum_.j_max;
  } else {
    t3 = -a_cruise / extremum_.j_min;
    Point_.j3 = extremum_.j_min;
  }

  double delt_v_3 =
      (a_cruise * t3 + Point_.j3 * std::pow(t3, 2) / 2);  // 符号待定
  double delt_v_2 = v_des - delt_v_1 - delt_v_3;

  double t2 = 0.0;
  if (dir_a == 0) {
    t2 = 0.0;
  } else {
    t2 = delt_v_2 / a_cruise;
  }

  if (t2 < 0) {
    if (dir_a == 1) {
      double a_n =
          sqrt((2 * (v_des - v_0) + std::pow(a_0, 2) / extremum_.j_max) /
               (1.0 / extremum_.j_max - 1.0 / extremum_.j_min));
      t1 = (a_n - a_0) / extremum_.j_max;
      t2 = 0.0;
      t3 = -a_n / extremum_.j_min;
    } else if (dir_a == -1) {
      double a_n =
          -sqrt((2 * (v_des - v_0) + std::pow(a_0, 2) / extremum_.j_min) /
                (1.0 / extremum_.j_min - 1.0 / extremum_.j_max));
      t1 = (a_n - a_0) / extremum_.j_min;
      t2 = 0.0;
      t3 = -a_n / extremum_.j_max;
    }
  }

  Point_.T1 = t1;
  Point_.T2 = t2 + t1;
  Point_.T3 = t3 + t2 + t1;

  return Point_;
}

Point_State jerk_limited_trajectory::updateTrajectory(const double &p_0,
                                                      const double &v_0,
                                                      const double &a_0,
                                                      const double &t,
                                                      const Point &Point_) {
  Point_State Point_State_;
  double t_remain = t;
  double t1_ = std::min(t, Point_.T1);
  Point_State_ = updatePoint(p_0, v_0, a_0, Point_.j1, t1_);

  t_remain = t - t1_;
  if (t_remain > 0) {
    double t2_ = std::min(t_remain, (Point_.T2 - Point_.T1));
    Point_State_ =
        updatePoint(Point_State_.p_, Point_State_.v_, Point_State_.a_, 0, t2_);
    t_remain = t_remain - t2_;
  }

  if (t_remain > 0) {
    double t3_ = std::min(t_remain, (Point_.T3 - Point_.T2));
    Point_State_ = updatePoint(Point_State_.p_, Point_State_.v_,
                               Point_State_.a_, Point_.j3, t3_);
    t_remain = t_remain - t3_;
  }

  if (t_remain > 0) {
    Point_State_ =
        updatePoint(Point_State_.p_, Point_State_.v_, 0, 0, t_remain);
  }

  return Point_State_;
}

Point_State jerk_limited_trajectory::updatePoint(const double &x,
                                                 const double &v,
                                                 const double &a,
                                                 const double &j,
                                                 const double &t) {
  Point_State Ps1;
  Ps1.a_ = a + j * t;
  Ps1.v_ = v + a * t + 0.5 * j * std::pow(t, 2);
  Ps1.p_ = x + v * t + 0.5 * a * std::pow(t, 2) + 1.0 / 6 * j * std::pow(t, 3);
  return Ps1;
}

void jerk_limited_trajectory::positionTargetSolver(const extremum &extrem,
                                                   const double &p_0,
                                                   const double &v_0,
                                                   const double &a_0,
                                                   const double &p_des) {
  Point P_sp;
  Point_State PS_sp;
  P_sp = velocityTargetSolver(extrem, v_0, a_0, 0);
  PS_sp = updateTrajectory(p_0, v_0, a_0, P_sp.T3, P_sp);

  int dir_p = 1;
  double v_cruise = 0.0;

  ((p_des - PS_sp.p_) > 0)
      ? dir_p = 1
      : (((p_des - PS_sp.p_) < 0) ? dir_p = -1 : dir_p = 0);

  (dir_p == 1) ? v_cruise = extrem.v_max
               : ((dir_p == -1) ? v_cruise = extrem.v_min : v_cruise = 0);

  Point P_a;
  Point P_b;
  Point P_pb;
  Point_State PS_fa;
  Point_State PS_fb;
  Point_State PS_pb;

  P_a = velocityTargetSolver(extrem, v_0, a_0, v_cruise);
  PS_fa = updateTrajectory(p_0, v_0, a_0, P_a.T3, P_a);
  P_b = velocityTargetSolver(extrem, v_cruise, 0, 0);
  PS_fb = updateTrajectory(PS_fa.p_, v_cruise, 0, P_b.T3, P_b);
  double delt_p = (PS_fb.p_ - p_des);

  double t_cruise = 0.0;
  int dir_pf = 1;
  double t_pb = 0.0;
  (delt_p > 0) ? dir_pf = 1 : ((delt_p < 0) ? dir_pf = -1 : dir_pf = 0);

  //可以达到v_cruise, 并计算巡航时间t_c, 反之t_c为0,
  //计算切换时间t_pb(加速度开始减小的时间, 二分法求解)
  if (dir_pf * dir_p <= 0) {
    t_cruise = -delt_p / v_cruise;
    t_pb = P_a.T3;
    t_s_ = t_pb;
    P_b_ = P_b;
  } else {
    t_cruise = 0.0;
    double t_H = P_a.T3;
    double t_L = 0;

    //这里N和e的取值论文未提及，目前暂取为100和0.001
    for (int i = 0; i < 100; ++i) {
      t_pb = (t_H + t_L) / 2;
      PS_pb = updateTrajectory(p_0, v_0, a_0, t_pb, P_a);
      P_pb = velocityTargetSolver(extrem, PS_pb.v_, PS_pb.a_, 0);
      PS_fb = updateTrajectory(PS_pb.p_, PS_pb.v_, PS_pb.a_, P_pb.T3, P_pb);
      double delt_p = (PS_fb.p_ - p_des);

      (delt_p > 0) ? dir_pf = 1 : ((delt_p < 0) ? dir_pf = -1 : dir_pf = 0);

      if (dir_pf * dir_p < 0) {
        t_L = t_pb;
      } else {
        t_H = t_pb;
      }
      if (std::fabs(delt_p) < 0.001) {
        t_s_ = t_pb;
        P_b_ = P_pb;
        break;
      }
    }
  }
  P_a_ = P_a;
  t_c_ = t_cruise;
  v_c_ = v_cruise;
}

void jerk_limited_trajectory::positionTargetSolver(
    const extremum &extrem, const double &p_0, const double &v_0,
    const double &a_0, const double &p_des, const double &v_des) {
  Point P_sp;
  Point_State PS_sp;
  P_sp = velocityTargetSolver(extrem, v_0, a_0, v_des);
  PS_sp = updateTrajectory(p_0, v_0, a_0, P_sp.T3, P_sp);

  int dir_p = 1;
  double v_cruise = 0.0;

  ((p_des - PS_sp.p_) > 0)
      ? dir_p = 1
      : (((p_des - PS_sp.p_) < 0) ? dir_p = -1 : dir_p = 0);

  (dir_p == 1) ? v_cruise = extrem.v_max
               : ((dir_p == -1) ? v_cruise = extrem.v_min : v_cruise = 0);

  Point P_a;
  Point P_b;
  Point P_pb;
  Point_State PS_fa;
  Point_State PS_fb;
  Point_State PS_pb;

  P_a = velocityTargetSolver(extrem, v_0, a_0, v_cruise);
  PS_fa = updateTrajectory(p_0, v_0, a_0, P_a.T3, P_a);
  P_b = velocityTargetSolver(extrem, v_cruise, 0, v_des);
  PS_fb = updateTrajectory(PS_fa.p_, v_cruise, 0, P_b.T3, P_b);
  double delt_p = (PS_fb.p_ - p_des);

  double t_cruise = 0.0;
  int dir_pf = 1;
  double t_pb = 0.0;
  (delt_p > 0) ? dir_pf = 1 : ((delt_p < 0) ? dir_pf = -1 : dir_pf = 0);

  //可以达到v_cruise, 并计算t_c, 反之t_c为0,
  //计算切换时间t_pb(加速度开始减小的时间, 二分法求解)
  if (dir_pf * dir_p <= 0) {
    t_cruise = -delt_p / v_cruise;
    t_pb = P_a.T3;
    t_s_ = t_pb;
    P_b_ = P_b;
  } else {
    t_cruise = 0.0;
    double t_H = P_a.T3;
    double t_L = 0;

    //这里N和e的取值论文未提及，目前暂取为100和0.001
    for (int i = 0; i < 100; ++i) {
      t_pb = (t_H + t_L) / 2;
      PS_pb = updateTrajectory(p_0, v_0, a_0, t_pb, P_a);
      P_pb = velocityTargetSolver(extrem, PS_pb.v_, PS_pb.a_, v_des);
      PS_fb = updateTrajectory(PS_pb.p_, PS_pb.v_, PS_pb.a_, P_pb.T3, P_pb);
      double delt_p = (PS_fb.p_ - p_des);

      (delt_p > 0) ? dir_pf = 1 : ((delt_p < 0) ? dir_pf = -1 : dir_pf = 0);

      if (dir_pf * dir_p < 0) {
        t_L = t_pb;
      } else {
        t_H = t_pb;
      }
      if (std::fabs(delt_p) < 0.001) {
        t_s_ = t_pb;
        P_b_ = P_pb;
        break;
      }
    }
  }
  P_a_ = P_a;
  t_c_ = t_cruise;
  v_c_ = v_cruise;
}

Point_State jerk_limited_trajectory::updateFinalTrajectory(const double &p_0,
                                                           const double &v_0,
                                                           const double &a_0,
                                                           double &ts) {
  Point_State Point_State_;

  if (ts >= 0 && ts < t_s_) {
    Point_State_ = updateTrajectory(p_0, v_0, a_0, ts, P_a_);
  } else if (ts >= t_s_ && ts < (t_s_ + t_c_)) {
    Point_State_ = updateTrajectory(p_0, v_0, a_0, P_a_.T3, P_a_);
    Point_State_.p_ = Point_State_.p_ + v_c_ * (ts - P_a_.T3);
  } else if (ts >= (t_s_ + t_c_)) {
    if (t_c_ > 0) {
      Point_State_ = updateTrajectory(p_0, v_0, a_0, t_s_, P_a_);
      double p_c = Point_State_.p_ + v_c_ * t_c_;
      Point_State_ = updateTrajectory(p_c, v_c_, 0, ts - (t_s_ + t_c_), P_b_);
    } else {
      Point_State_ = updateTrajectory(p_0, v_0, a_0, t_s_, P_a_);
      Point_State_ =
          updateTrajectory(Point_State_.p_, Point_State_.v_, Point_State_.a_,
                           ts - (t_s_ + t_c_), P_b_);
    }
  }
  return Point_State_;
}
