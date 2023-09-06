#include "/root/code/planning/src/library/jlt_lib/include/jerk_limited_trajectory.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include "gtest/gtest.h"

jerk_limited_trajectory JLT;
extremum extrem_;
Point_State PS_0 = {2.0, 1.0, 0.2};
// Point_State PS_f = {0.0, 0.0, 0.0};


TEST(TestJerkLimitedTrajectory, jerk_limited_trajectory) {
    //init
    // double total_time = 25.0;
    double p_desire = 10.0;
    double v_desire = 0.5;
    double delta_t = 0.1;
    double time_stamp = 0.0;
    std::vector<double> s_ref;
    std::vector<double> v_ref;
    std::vector<double> a_ref;
    JLT.positionTargetSolver(extrem_, PS_0.p_, PS_0.v_, PS_0.a_, p_desire, v_desire);

    Point _P_a = JLT.getPositionA();
    Point _P_b = JLT.getPositionB();
    double _t_s_ = JLT.getTimeSwitch();
    double _t_c_ = JLT.getTimeCruise();
    double _v_c_ = JLT.getVelocityCruise(); 

    double total_time  = _P_a.T3 + _P_b.T3 + _t_c_;
    s_ref.reserve(total_time / delta_t + 1);
    v_ref.reserve(total_time / delta_t + 1);
    a_ref.reserve(total_time / delta_t + 1);
    std::cout << "P_a_ = " << _P_a.T3 << " "
              << "P_b_ = " << _P_b.T3 << " " << std::endl;
    std::cout << "t_s_ = " << _t_s_ << " "
              << "t_c_ = " << _t_c_ << " " 
              << "v_c_ = " << _v_c_ << " " << std::endl;             
    for (int i = 0; i < total_time / delta_t; ++i) {
        Point_State PS_f =  JLT.updateFinalTrajectory(PS_0.p_, PS_0.v_, PS_0.a_, time_stamp);
        time_stamp = time_stamp + delta_t;
        s_ref.emplace_back(PS_f.p_);
        v_ref.emplace_back(PS_f.v_);
        a_ref.emplace_back(PS_f.a_);
        std::cout << "position = " << s_ref[i] << " "
                  << "velocity = " << v_ref[i] << " "
                  << "acc = " << a_ref[i] << std::endl;
    } 
    EXPECT_TRUE(s_ref.back() == 0);
    EXPECT_TRUE(v_ref.back() == 0);
    EXPECT_TRUE(a_ref.back() == 0);  
}
