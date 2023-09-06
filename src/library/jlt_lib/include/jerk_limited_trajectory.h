
/**
 * @file
 * @brief 用于快速生成存在jerk,v约束的轨迹ref
 */
#ifndef __JERK_LIMITED_TRAJECTORY__
#define __JERK_LIMITED_TRAJECTORY__

struct extremum {
    double v_max = 20.0;
    double v_min = 0.0;
    double a_max = 3.0;
    double a_min = -4.0;
    double j_max = 7.0;
    double j_min = -7.0;
};

struct Point {
    double j1 = 0.0;
    double j2 = 0.0;
    double j3 = 0.0;
    double T1 = 0.0;
    double T2 = 0.0;
    double T3 = 0.0;
};

struct Point_State {
    double p_ = 0.0;
    double v_ = 0.0;
    double a_ = 0.0;
};

class jerk_limited_trajectory {   
 public:
    Point getPositionA() {return P_a_;};
    Point getPositionB() {return P_b_;};
    double getTimeSwitch() {return t_s_;};
    double getTimeCruise() {return t_c_;};    
    double getVelocityCruise() {return v_c_;};
    void positionTargetSolver(const extremum &extrem, const double &p_0, const double &v_0, 
                              const double &a_0, const double &p_f);
    void positionTargetSolver(const extremum &extrem, const double &p_0, const double &v_0, 
                              const double &a_0, const double &p_f, const double &v_f);
    Point_State updateFinalTrajectory(const double &p_0, const double &v_0, const double &a_0, 
                                      double &ts);    

 private:
    Point velocityTargetSolver(const extremum &extrem, const double &v_0, const double &a_0, 
                               const double &v_des);
    Point_State updatePoint(const double &x, const double &v, const double &a, 
                            const double &j, const double &t);
    Point_State updateTrajectory(const double &p_0, const double &v_0, const double &a_0, 
                                 const double &t, const Point &Point_);                              
    int calculateDireciton(const extremum &extrem, const double &v_0, const double &a_0, 
                           const double &v_des);

 private:
    extremum extremum_;
    double delta_t_ = 0.1;
    Point P_a_;
    Point P_b_;
    double t_c_;
    double t_s_;
    double v_c_;
    // double pos_;
    // double vel_;
    // double acc_;
};

#endif