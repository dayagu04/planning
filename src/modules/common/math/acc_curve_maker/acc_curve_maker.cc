#include "acc_curve_maker.h"

namespace planning {

namespace {
constexpr double kFollowHighWay = 1.2;
constexpr double kMinFollowDistance = 4.0;
static const std::vector<double> A_MIN_VALUE_PT = {-4.4, -3.08};
static const std::vector<double> A_MIN_SPEED_PT = {5, 20};
static constexpr double deadband_region_ratio = 0.5;
}  // namespace

AccCurveMaker::AccCurveMaker(const std::array<double, 3>& ego_state,
                             const double cipv_s, const double cipv_vel)
    : ego_state_(ego_state), cipv_s_(cipv_s), cipv_vel_(cipv_vel) {}

bool AccCurveMaker::Run() {
  const double ds = cipv_s_;
  ego_dv_ = ego_state_[1] - cipv_vel_;
  dv_safe_ = MakeDvSafe();
  MakeDvComfortable(&dv_curve_);
  return true;
}

double AccCurveMaker::ego_dv() const { return ego_dv_; }

double AccCurveMaker::dv_safe() const { return dv_safe_; }

double AccCurveMaker::dv_comfortalble() const { return dv_curve_; }

double AccCurveMaker::MakeDvSafe() {
  const double ego_vel = ego_state_[1];
  // TODO: 按照现在执行器能够响应的减速度修改标定值
  double ax_min_limit = interpolation(ego_vel, A_MIN_SPEED_PT, A_MIN_VALUE_PT);
  return smooth_sqrt(2.0 * std::fabs(ax_min_limit) * (cipv_s_));
}

double AccCurveMaker::MakeFollowDist() {
  return kFollowHighWay * cipv_vel_ + kMinFollowDistance;
}

bool AccCurveMaker::MakeDvComfortable(double* const dv_curve) {
  double da_curve = 0.0;
  const double ds_offset = cipv_s_;
  double ds_follow_offset = MakeFollowDist();
  // acc curve is designed with 3-segment piecewise function
  // seg1 is linear, seg3 is const decel, seg2 is a blending linear function
  // const double &seg1_start_dv = -4.0;
  const double& seg1_start_dv = -std::fmin(8.0, std::fmax(cipv_vel_ * 0.5, 4));
  // const double &seg1_end_dv = 0.4;
  const double& seg1_end_dv = -0.1 * seg1_start_dv;
  const double& seg2_end_dv = 1.4;
  // TODO: design seg2 slope 0.3 at 0mps, slope 0.8 at 30mps
  const double seg2_slope = 0.5;
  const double seg1_end_s = 1.1 * ds_follow_offset;
  const double seg2_end_s =
      seg1_end_s + (seg2_end_dv - seg1_end_dv) / seg2_slope;

  // compute acc curve in 3-segments
  if (ds_offset < seg1_end_s) {
    *dv_curve = seg1_start_dv - ds_offset / ds_follow_offset * seg1_start_dv;
    // a = dvdt = dvds*dsdt = dvds*v
    da_curve = -*dv_curve / ds_follow_offset * seg1_start_dv;
  } else if (ds_offset < seg2_end_s) {
    *dv_curve = seg1_end_dv + (ds_offset - seg1_end_s) * seg2_slope;
    // a = dvdt = dvds*dsdt = dvds*v
    da_curve = *dv_curve * seg2_slope;
  } else {
    // 0mps is -0.8mps2, 30mps is -0.4mps2
    double seg3_decel_abs = std::fmax(0.4, 0.8 - ego_state_[1] * 0.4 / 30.0);
    double quadratic_zero_s =
        seg2_end_s - (seg2_end_dv * seg2_end_dv) / 2.0 / seg3_decel_abs;
    *dv_curve =
        smooth_sqrt(2.0 * seg3_decel_abs * (ds_offset - quadratic_zero_s));
    da_curve = seg3_decel_abs;
  }

  double deadband_dv = *dv_curve > 2.0
                           ? deadband_region_ratio * (*dv_curve - 2.0) + 2.0
                           : *dv_curve;
  return true;
}

}  // namespace planning