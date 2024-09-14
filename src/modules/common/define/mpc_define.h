#ifndef MSQUARE_PLANNING_DEFINE_MPC_DEFINE_H_
#define MSQUARE_PLANNING_DEFINE_MPC_DEFINE_H_

#define MPC_N 25

struct state_t {
  double s_frenet, r_frenet, theta_error, vel, acc, omega;
};

struct log_t {
  double s_frenet[MPC_N + 1];
  double r_frenet[MPC_N + 1];
  double theta_error[MPC_N + 1];
  double vel[MPC_N + 1];
  double acc[MPC_N + 1];
  double omega[MPC_N + 1];
};

struct control_out_t {
  double omega_rate[MPC_N];
  double jerk[MPC_N];
};

struct speed_mpc_online_data {
  double sr[MPC_N + 1];
  double vr[MPC_N + 1];
  double s_upper[MPC_N + 1];
  double s_lower[MPC_N + 1];
  double curvature[MPC_N + 1];
  double a_upper[MPC_N + 1];
  double a_lower[MPC_N + 1];
};

#endif
