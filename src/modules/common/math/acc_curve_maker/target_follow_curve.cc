#include "target_follow_curve.h"

#include <vector>

#include "dv_spline.h"

namespace planning {

namespace {
const int num_of_knots = 26;
const double t_cycle = 0.2;
}  // namespace

TargetFollowCurve::TargetFollowCurve(DvPoint target_pt, const double& ego_s,
                                     const double& cipv_s,
                                     const double& cipv_v) {
  target_point_ = target_pt;
  if (target_point_.v < obj_stop_v_thr_) {
    target_point_.v = 0.0;
  }
  generate_target_dv_curve();
  update_target_st_curve(cipv_s - ego_s, cipv_v, cipv_s);
}

void TargetFollowCurve::generate_target_dv_curve() {
  target_dv_curve_.get_mutable_dv_points()->clear();
  DvPoint pdv = target_point_;
  const double ref_traj_len = 200.0;
  target_dv_curve_.get_mutable_dv_points()->push_back(pdv);

  for (int i = 0;; ++i) {
    double t = i * t_cycle;
    pdv.d = target_point_.d - 0.5 * target_st_dec_ * t * t;
    pdv.v = target_point_.v - target_st_dec_ * t;
    target_dv_curve_.get_mutable_dv_points()->push_back(pdv);
    if (pdv.d > ref_traj_len) {
      break;
    }
  }
  for (int i = 0;; ++i) {
    double t = i * t_cycle;
    pdv.d = target_point_.d - 0.5 * target_st_acc_ * t * t;
    pdv.v = target_point_.v - target_st_acc_ * t;
    target_dv_curve_.get_mutable_dv_points()->push_back(pdv);
    if (pdv.d <= 0.0) {
      break;
    }
  }
  target_dv_curve_.sort_by_d();
}

void TargetFollowCurve::update_target_st_curve(const double dist,
                                               const double& cipv_v,
                                               const double& cipv_s) {
  auto* st_points = target_st_curve_.get_mutable_st_points();
  const auto& dv_points = target_dv_curve_.get_dv_points();
  StPoint pst;
  int start_idx = 0;
  int head = 0, tail = dv_points.size() - 1, mid = 0;
  for (int i = 0; i < dv_points.size(); ++i) {
    mid = (head + tail) / 2;
    if (tail > head + 1) {
      if (dv_points[mid].d > dist) {
        tail = mid;
      } else {
        head = mid;
      }
    } else {
      if (dv_points[tail].d - dist < dist - dv_points[head].d) {
        start_idx = tail;
      } else {
        start_idx = head;
      }
      break;
    }
  }
  if (dist < target_point_.d) {
    // dist < set_dist
    int cnt = 0;
    for (int i = start_idx; i > 0; ++i) {
      auto pdv = dv_points[i];
      pst.s = cipv_s - pdv.d + cnt * t_cycle * cipv_v;
      pst.v = pdv.v;
      pst.t = cnt * t_cycle;
      pst.a = target_st_acc_;
      st_points->push_back(pst);
      if (i < dv_points.size() - 1 &&
          dv_points[i + 1].d > target_point_.d + 0.01) {
        break;
      }
      ++cnt;
    }
  } else {
    // dist > set_dist
    int cnt = 0;
    for (int i = start_idx; i < dv_points.size(); --i) {
      auto pdv = dv_points[i];
      pst.s = cipv_s - pdv.d + cnt * t_cycle * cipv_v;
      pst.v = pdv.v;
      pst.t = cnt * t_cycle;
      pst.a = target_st_dec_;
      st_points->push_back(pst);
      if (i > 0 && dv_points[i - 1].d < target_point_.d - 0.01) {
        break;
      }
      ++cnt;
    }
  }
  if (st_points->size() < num_of_knots) {
    for (int i = st_points->size(); i < num_of_knots; ++i) {
      pst.s = cipv_s - target_point_.d + i * t_cycle * cipv_v;
      pst.v = target_point_.v;
      pst.t = (double)i * t_cycle;
      pst.a = 0;
      st_points->push_back(pst);
    }
  }
}

}  // namespace planning