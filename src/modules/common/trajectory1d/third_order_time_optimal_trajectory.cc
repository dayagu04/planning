#include "third_order_time_optimal_trajectory.h"
#include <math.h>

#include <cmath>
#include <iostream>
#include <limits>

#include "status/status.h"

namespace planning {

namespace {

constexpr int32_t kMaxSearchTimes = 50;

inline int GetSign(double num) {
  if (std::fabs(num) < std::numeric_limits<double>::epsilon()) {
    return 0;
  }
  if (num > 0.0) {
    return 1;
  }
  return -1;
}

}  // namespace

ThirdOrderTimeOptimalTrajectory::ThirdOrderTimeOptimalTrajectory(
    const double p0, const double v0, const double a0, const double p_end,
    const double v_min, const double v_max, const double acc_min,
    const double acc_max, const double jerk_min, const double jerk_max,
    const double p_precision) {
  const bool is_input_valid =
      (v_min < 0.0 && v_max > 0.0 && acc_min < 0.0 && acc_max > 0.0 &&
       jerk_min < 0.0 && jerk_max > 0.0);
  if (!is_input_valid) {
    // throw SpeedStatus(
    //     StatusCode::COMMON_ERROR,
    //     "invalid input limit for third order time optimal trajectory");
  }

  p_precision_ = p_precision;
  init_state_.p = p0;
  init_state_.v = v0;
  init_state_.a = a0;

  state_limit_.p_end = p_end;
  state_limit_.v_min = v_min;
  state_limit_.v_max = v_max;
  state_limit_.a_min = acc_min;
  state_limit_.a_max = acc_max;
  state_limit_.j_min = jerk_min;
  state_limit_.j_max = jerk_max;

  state_limit_pa_ = state_limit_;
  state_limit_pb_ = state_limit_;

  third_order_param_ = PositionTargetSolver(init_state_, state_limit_pa_,
                                            state_limit_pb_, p_precision_);
}

ThirdOrderTimeOptimalTrajectory::ThirdOrderTimeOptimalTrajectory(
    const LonState& init_state, const StateLimit& state_limit,
    const double p_precision)
    : init_state_(init_state),
      state_limit_(state_limit),
      state_limit_pa_(state_limit),
      state_limit_pb_(state_limit),
      p_precision_(p_precision) {
  const bool is_input_valid =
      (state_limit_.v_min < 0.0 && state_limit_.v_max > 0.0 &&
       state_limit_.a_min < 0.0 && state_limit_.a_max > 0.0 &&
       state_limit_.j_min < 0.0 && state_limit_.j_max > 0.0);
  if (!is_input_valid) {
    // throw SpeedStatus(
    //     StatusCode::COMMON_ERROR,
    //     "invalid input limit for third order time optimal trajectory");
  }

  third_order_param_ = PositionTargetSolver(init_state_, state_limit_pa_,
                                            state_limit_pb_, p_precision_);
}

ThirdOrderTimeOptimalTrajectory::ThirdOrderTimeOptimalTrajectory(
    const LonState& init_state, const StateLimit& state_limit_pa,
    const StateLimit& state_limit_pb, const double p_precision)
    : init_state_(init_state),
      state_limit_(state_limit_pa),
      state_limit_pa_(state_limit_pa),
      state_limit_pb_(state_limit_pb),
      p_precision_(p_precision) {
  const bool is_input_valid =
      (state_limit_.v_min < 0.0 && state_limit_.v_max > 0.0 &&
       state_limit_.a_min < 0.0 && state_limit_.a_max > 0.0 &&
       state_limit_.j_min < 0.0 && state_limit_.j_max > 0.0);
  if (!is_input_valid) {
    // std::cout << "invalid input limit for third order time optimal
    // trajectory" << std::endl;
    // throw SpeedStatus(
    //     StatusCode::COMMON_ERROR,
    //     "invalid input limit for third order time optimal trajectory");
  }

  third_order_param_ = PositionTargetSolver(init_state_, state_limit_pa_,
                                            state_limit_pb_, p_precision_);
}

double ThirdOrderTimeOptimalTrajectory::Evaluate(const int32_t order,
                                                 const double param) const {
  switch (order) {
    case 0:
      if (param > ParamLength()) {
        return state_limit_.p_end;
      }
      return GetThirdOrderTrajectoryState(init_state_, third_order_param_,
                                          param)
          .p;
    case 1:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetThirdOrderTrajectoryState(init_state_, third_order_param_,
                                          param)
          .v;
    case 2:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetThirdOrderTrajectoryState(init_state_, third_order_param_,
                                          param)
          .a;
    case 3:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetThirdOrderTrajectoryState(init_state_, third_order_param_,
                                          param)
          .j;
    default:
      return 0.0;
  }
}

double ThirdOrderTimeOptimalTrajectory::ParamLength() const {
  return third_order_param_.tpb + third_order_param_.tc +
         third_order_param_.Pb.t3;
}

ThirdOrderParam ThirdOrderTimeOptimalTrajectory::PositionTargetSolver(
    const LonState& init_state, const StateLimit& state_limit_pa,
    const StateLimit& state_limit_pb, const double p_precision) {
  // std::cout << "== start to solve position target" << std::endl;
  ThirdOrderParam third_order_param;

  SecondOrderParam P_sp =
      SecondOrderTimeOptimalTrajectory::VelocityTargetSolver(init_state,
                                                             state_limit_pa);
  LonState state_sp =
      SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
          init_state, P_sp, P_sp.t3);

  int dp = GetSign(state_limit_pa.p_end - state_sp.p);

  double vc = 0.0;
  if (dp == 1) {
    vc = state_limit_pa.v_max;
  } else if (dp == -1) {
    vc = state_limit_pa.v_min;
  }

  // std::cout << "vc: " << vc << std::endl;

  StateLimit state_limit_vc = state_limit_pa;
  state_limit_vc.v_end = vc;
  SecondOrderParam Pa = SecondOrderTimeOptimalTrajectory::VelocityTargetSolver(
      init_state, state_limit_vc);
  LonState state_pa =
      SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
          init_state, Pa, Pa.t3);
  double pfa = state_pa.p;

  // std::cout << "print Pa result: " << std::endl;
  // std::cout << "\tt1: " << Pa.t1 << " t2: " << Pa.t2 << " t3: " << Pa.t3 <<
  // std::endl; std::cout << "\tj1: " << Pa.j1 << " j2: " << Pa.j2 << " j3: " <<
  // Pa.j3 << std::endl; std::cout << "pfa: " << pfa << std::endl;

  LonState init_state_pb;
  init_state_pb.p = pfa;
  init_state_pb.v = vc;
  init_state_pb.a = 0.0;
  SecondOrderParam Pb = SecondOrderTimeOptimalTrajectory::VelocityTargetSolver(
      init_state_pb, state_limit_pb);
  LonState state_pb =
      SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
          init_state_pb, Pb, Pb.t3);
  double pfb = state_pb.p;

  // std::cout << "print Pb result: " << std::endl;
  // std::cout << "\tt1: " << Pb.t1 << " t2: " << Pb.t2 << " t3: " << Pb.t3 <<
  // std::endl; std::cout << "\tj1: " << Pb.j1 << " j2: " << Pb.j2 << " j3: " <<
  // Pb.j3 << std::endl; std::cout << "pfb: " << pfb << std::endl;

  double tc = 0.0;
  double tpb = 0.0;

  if (GetSign(pfb - state_limit_pb.p_end) * dp <= 0) {
    constexpr double kEpsilon = 1e-10;
    if (fabs(vc) < kEpsilon) {
      tc = (state_limit_pb.p_end - pfb) / (vc + kEpsilon);
    } else {
      tc = (state_limit_pb.p_end - pfb) / vc;  // incase vc == 0.0;
    }
    tpb = Pa.t3;
  } else {
    tc = 0.0;
    double th = Pa.t3;
    double tl = 0.0;
    int count = 0;
    double cur_pfb = pfb;
    // std::cout << "p_precision: " << p_precision << std::endl;
    // std::cout << "cur_pfb: " << cur_pfb << std::endl;
    // std::cout << "start to run while loop, th: " << th << " tl: " << tl <<
    // std::endl;
    while ((std::fabs(cur_pfb - state_limit_pb.p_end) > p_precision) &&
           count <= kMaxSearchTimes) {
      count++;
      tpb = (th + tl) * 0.5;

      LonState init_state_tpb =
          SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
              init_state, Pa, tpb);
      Pb = SecondOrderTimeOptimalTrajectory::VelocityTargetSolver(
          init_state_tpb, state_limit_pb);

      LonState cur_state_pb =
          SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
              init_state_tpb, Pb, Pb.t3);
      cur_pfb = cur_state_pb.p;

      // std::cout << "\n\tcount: " << count << std::endl;
      // std::cout << "\t\ttpb: " << tpb << std::endl;
      // std::cout << "\t\tinit_state_tpb, p: " << init_state_tpb.p << " v: " <<
      // init_state_tpb.v
      //           << " a: " << init_state_tpb.a << " j: " << init_state_tpb.j
      //           << " t: " << init_state_tpb.t << std::endl;
      // std::cout << "\t\tPb, t1: " << Pb.t1 << " t2: " << Pb.t2 << " t3: " <<
      // Pb.t3
      //           << " j1: " << Pb.j1 << " j2: " << Pb.j2 << " j3: " << Pb.j3
      //           << std::endl;
      // std::cout << "\t\tcur_state_pb, p: " << cur_state_pb.p << " v: " <<
      // cur_state_pb.v
      //           << " a: " << cur_state_pb.a << " j: " << cur_state_pb.j << "
      //           t: " << cur_state_pb.t
      //           << std::endl;
      // std::cout << "\t\tcur_pfb: " << cur_pfb << std::endl;
      // std::cout << "\t\terror: " << cur_pfb - state_limit_pb.p_end <<
      // std::endl;

      // if (std::fabs(cur_pfb - state_limit_pb.p_end) < p_precision) {
      //   std::cout << "reach p within precision" << std::endl;
      //   break;
      // }

      if (GetSign(cur_pfb - state_limit_pb.p_end) * dp < 0.0) {
        tl = tpb;
      } else {
        th = tpb;
      }

      // std::cout << "\tafter update: " << std::endl;
      // std::cout << "\t\tth: " << th << std::endl;
      // std::cout << "\t\ttl: " << tl << std::endl;
    }
  }

  // std::cout << "tc: " << tc << std::endl;
  // std::cout << "tpb: " << tpb << std::endl;

  third_order_param.Pa = Pa;
  third_order_param.Pb = Pb;
  third_order_param.tc = tc;
  third_order_param.tpb = tpb;
  third_order_param.vc = vc;

  return third_order_param;
}

LonState ThirdOrderTimeOptimalTrajectory::GetThirdOrderTrajectoryState(
    const LonState& init_state, const ThirdOrderParam& third_order_param,
    const double t) {
  if (t <= 0.0) {
    return init_state;
  }

  LonState result_state;
  double t_remain = t;

  double ta = std::min(t_remain, third_order_param.tpb);
  result_state =
      SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
          init_state, third_order_param.Pa, ta);
  t_remain = t_remain - third_order_param.tpb;

  if (t_remain > 0.0) {
    double tc = std::min(t_remain, third_order_param.tc);
    result_state.p = result_state.p + third_order_param.vc * tc;
    result_state.j = 0.0;

    t_remain = t_remain - third_order_param.tc;
  }

  if (t_remain > 0.0) {
    result_state =
        SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
            result_state, third_order_param.Pb, t_remain);
  }

  result_state.t = t;
  return result_state;
}

}  // namespace planning
