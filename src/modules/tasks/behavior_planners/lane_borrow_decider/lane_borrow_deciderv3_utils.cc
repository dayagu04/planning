#include "lane_borrow_deciderv3_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <ostream>
#include <utility>
#include <vector>

#include "basic_types.pb.h"
#include "config/message_type.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "ifly_time.h"
#include "library/lc_idm_lib/include/basic_intelligent_driver_model.h"
#include "log.h"
#include "math/curve1d/quintic_polynomial_curve1d.h"
#include "math_lib.h"
#include "modules/context/planning_context.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
namespace {
constexpr double kMaxLateralRange = 5.0;
constexpr double kMaxLongitRange = 70.0;
constexpr double kMinLongitRange = 25.0;
constexpr double kMinSDelta = 1e-4;  // 路径点 s 值的最小增量阈值
constexpr double kStaticOtherMaxExtraLateralBuffer = 0.5;

constexpr double kBlockedBorrowObstaclesRatioThreshold = 0.5;
constexpr double kHardBuffer2Road = 0.4;  // 参考横向避让路沿的hard buffer
constexpr double kScanRoadRange = 10.0;  // 扫描路沿的范围 m
constexpr double kScanRoadStep = 1.0;    // 扫描路沿的步长 m
constexpr double kEpsilon = 1e-6;
constexpr double kClearanceHysteresis = 0.15;  // [m]

};  // namespace

namespace planning {
namespace lane_borrow_deciderv3_utils {

namespace {

struct SLPair {
  double s = 0.0;
  double l = 0.0;
};

double InterpLAtS(const std::vector<SLPair>& sl, const double query_s,
                  size_t* idx_hint) {
  if (sl.empty()) {
    return 0.0;
  }
  if (sl.size() == 1) {
    return sl.front().l;
  }
  size_t i = (idx_hint != nullptr) ? *idx_hint : 0U;
  if (i >= sl.size()) {
    i = 0U;
  }
  while (i + 1 < sl.size() && sl[i + 1].s < query_s) {
    ++i;
  }
  while (i > 0 && sl[i].s > query_s) {
    --i;
  }
  if (idx_hint != nullptr) {
    *idx_hint = i;
  }
  if (query_s <= sl.front().s) {
    return sl.front().l;
  }
  if (query_s >= sl.back().s) {
    return sl.back().l;
  }
  const auto& p0 = sl[i];
  const auto& p1 = sl[std::min(i + 1, sl.size() - 1)];
  const double s0 = p0.s;
  const double s1 = p1.s;
  if (s1 <= s0 + 1e-6) {
    return p0.l;
  }
  const double w = (query_s - s0) / (s1 - s0);
  return p0.l + w * (p1.l - p0.l);
}

struct RefSLPoint {
  double s = 0.0;
  double l = 0.0;
  double x = 0.0;
  double y = 0.0;
};

std::vector<RefSLPoint> BuildRefSlFromRefinedPaths(
    const std::vector<planning_math::PathPoint>& refined_paths,
    const std::shared_ptr<planning_math::KDPath>& current_frenet) {
  std::vector<RefSLPoint> out;
  out.reserve(refined_paths.size());
  if (current_frenet == nullptr) {
    return out;
  }
  for (const auto& p : refined_paths) {
    double s = 0.0, l = 0.0;
    // Reproject to ensure same (s,l) reference frame.
    if (!current_frenet->XYToSL(p.x(), p.y(), &s, &l)) {
      continue;
    }
    out.push_back({s, l, p.x(), p.y()});
  }
  std::sort(out.begin(), out.end(),
            [](const RefSLPoint& a, const RefSLPoint& b) { return a.s < b.s; });
  // Dedup close s to keep monotonic.
  std::vector<RefSLPoint> uniq;
  uniq.reserve(out.size());
  constexpr double kDedupS = 1e-3;
  for (const auto& p : out) {
    if (uniq.empty() || p.s > uniq.back().s + kDedupS) {
      uniq.push_back(p);
    } else {
      uniq.back() = p;
    }
  }
  return uniq;
}

double InterpRefLAtS(const std::vector<RefSLPoint>& ref, const double query_s,
                     size_t* idx_hint) {
  if (ref.empty()) {
    return 0.0;
  }
  if (ref.size() == 1) {
    return ref.front().l;
  }
  size_t i = (idx_hint != nullptr) ? *idx_hint : 0U;
  if (i >= ref.size()) {
    i = 0U;
  }
  while (i + 1 < ref.size() && ref[i + 1].s < query_s) {
    ++i;
  }
  while (i > 0 && ref[i].s > query_s) {
    --i;
  }
  if (idx_hint != nullptr) {
    *idx_hint = i;
  }
  if (query_s <= ref.front().s) {
    return ref.front().l;
  }
  if (query_s >= ref.back().s) {
    return ref.back().l;
  }
  const auto& p0 = ref[i];
  const auto& p1 = ref[std::min(i + 1, ref.size() - 1)];
  const double s0 = p0.s;
  const double s1 = p1.s;
  if (s1 <= s0 + 1e-6) {
    return p0.l;
  }
  const double w = (query_s - s0) / (s1 - s0);
  return p0.l + w * (p1.l - p0.l);
}

double ClearanceToLRange(const double ref_l, const double l_min,
                         const double l_max) {
  if (ref_l < l_min) {
    return l_min - ref_l;
  }
  if (ref_l > l_max) {
    return ref_l - l_max;
  }
  return 0.0;
}

double CosineWindow01(const double u01) {
  const double u = std::clamp(u01, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * u);
}

void AddDeltaLWindow(std::vector<double>& delta_l,
                     const std::vector<RefSLPoint>& ref, const double s0,
                     const double s1, const double delta) {
  if (delta_l.size() != ref.size() || ref.empty() || s1 <= s0 + 1e-3) {
    return;
  }
  for (size_t i = 0; i < ref.size(); ++i) {
    const double s = ref[i].s;
    if (s < s0 || s > s1) {
      continue;
    }
    const double w = CosineWindow01((s - s0) / (s1 - s0));
    delta_l[i] += delta * w;
  }
}

double InterpDeltaAtS(const std::vector<RefSLPoint>& ref,
                      const std::vector<double>& delta, const double query_s,
                      size_t* idx_hint) {
  if (ref.empty() || delta.size() != ref.size()) {
    return 0.0;
  }
  if (ref.size() == 1) {
    return delta.front();
  }
  size_t i = (idx_hint != nullptr) ? *idx_hint : 0U;
  if (i >= ref.size()) {
    i = 0U;
  }
  while (i + 1 < ref.size() && ref[i + 1].s < query_s) {
    ++i;
  }
  while (i > 0 && ref[i].s > query_s) {
    --i;
  }
  if (idx_hint != nullptr) {
    *idx_hint = i;
  }
  if (query_s <= ref.front().s) {
    return delta.front();
  }
  if (query_s >= ref.back().s) {
    return delta.back();
  }
  const double s0 = ref[i].s;
  const size_t i1 = std::min(i + 1, ref.size() - 1);
  const double s1 = ref[i1].s;
  if (s1 <= s0 + 1e-6) {
    return delta[i];
  }
  const double w = (query_s - s0) / (s1 - s0);
  return delta[i] + w * (delta[i1] - delta[i]);
}

// Compute minimal clearance between ref l(s) and obstacle [l_min,l_max] over s
double ComputeMinClearanceOverS(const std::vector<RefSLPoint>& ref,
                                const double s_start, const double s_end,
                                const double l_min, const double l_max) {
  if (ref.size() < 2) {
    return std::numeric_limits<double>::max();
  }
  const double s0 = std::min(s_start, s_end);
  const double s1 = std::max(s_start, s_end);
  constexpr double kSampleStep = 1.0;  // m
  double min_clear = std::numeric_limits<double>::max();
  size_t idx_hint = 0U;
  for (double s = s0; s <= s1 + 1e-6; s += kSampleStep) {
    const double ref_l = InterpRefLAtS(ref, s, &idx_hint);
    min_clear = std::min(min_clear, ClearanceToLRange(ref_l, l_min, l_max));
  }
  // also include endpoints
  {
    size_t h = 0U;
    min_clear = std::min(
        min_clear, ClearanceToLRange(InterpRefLAtS(ref, s0, &h), l_min, l_max));
    min_clear = std::min(
        min_clear, ClearanceToLRange(InterpRefLAtS(ref, s1, &h), l_min, l_max));
  }
  return min_clear;
}
}  // namespace

void LaneBorrowDeciderV3Utils::StabilizeRefinedPaths() {
  if (refined_paths_.empty() || last_frame_paths_.size() < 3 ||
      current_reference_path_ptr_ == nullptr) {
    return;
  }

  // 借道障碍物列表有变化，跳过稳定化，让路径快速响应新障碍物
  auto sorted_ids = static_blocked_obj_id_vec_;
  auto sorted_last_ids = last_static_blocked_obj_id_vec_;
  std::sort(sorted_ids.begin(), sorted_ids.end());
  std::sort(sorted_last_ids.begin(), sorted_last_ids.end());
  if (sorted_ids != sorted_last_ids) {
    return;
  }

  // 障碍物 ID 相同，但静/动态属性变化，也跳过滞回
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  bool static_attr_changed = false;
  for (const int obs_id : static_blocked_obj_id_vec_) {
    auto it = std::find_if(obstacles.begin(), obstacles.end(),
                           [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                             return o != nullptr && o->obstacle() != nullptr &&
                                    o->obstacle()->id() == obs_id;
                           });
    if (it != obstacles.end()) {
      const bool is_static_now = (*it)->obstacle()->is_static();
      auto last_it = last_obs_static_map_.find(obs_id);
      if (last_it != last_obs_static_map_.end() &&
          last_it->second != is_static_now) {
        static_attr_changed = true;
      }
    }
  }
  if (static_attr_changed) {
    return;
  }

  const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }

  // Reproject last-frame path into current frenet coordinate system (XY->SL),
  // ensuring s/l comparison is in the same reference frame.
  std::vector<SLPair> last_sl;
  last_sl.reserve(last_frame_paths_.size());
  for (const auto& p : last_frame_paths_) {
    double s = 0.0, l = 0.0;
    if (!frenet_coord->XYToSL(p.x(), p.y(), &s, &l)) {
      continue;
    }
    last_sl.push_back({s, l});
  }
  if (last_sl.size() < 3) {
    return;
  }
  std::sort(last_sl.begin(), last_sl.end(),
            [](const SLPair& a, const SLPair& b) { return a.s < b.s; });
  // Dedup near-equal s to keep monotonic for interpolation.
  std::vector<SLPair> last_sl_uniq;
  last_sl_uniq.reserve(last_sl.size());
  constexpr double kDedupS = 1e-3;
  for (const auto& p : last_sl) {
    if (last_sl_uniq.empty() || p.s > last_sl_uniq.back().s + kDedupS) {
      last_sl_uniq.push_back(p);
    } else {
      last_sl_uniq.back().l = p.l;
    }
  }
  if (last_sl_uniq.size() < 3) {
    return;
  }
  const double last_s_min = last_sl_uniq.front().s;
  const double last_s_max = last_sl_uniq.back().s;

  constexpr double kMaxDeltaLPerFrame = 0.1;  // m
  constexpr double kEmaAlpha = 0.5;

  size_t idx_hint = 0U;
  for (auto& pt : refined_paths_) {
    const double s = pt.s();
    if (s < last_s_min || s > last_s_max) {
      continue;  // keep current-frame raw point outside overlap
    }
    const double last_l = InterpLAtS(last_sl_uniq, s, &idx_hint);
    const double raw_l = pt.l();
    const double dl = raw_l - last_l;
    const double limited_l =
        last_l + std::clamp(dl, -kMaxDeltaLPerFrame, kMaxDeltaLPerFrame);
    const double smooth_l = kEmaAlpha * limited_l + (1.0 - kEmaAlpha) * last_l;

    double x = 0.0, y = 0.0;
    if (!frenet_coord->SLToXY(s, smooth_l, &x, &y)) {
      continue;
    }
    const auto ref_pt = frenet_coord->GetPathPointByS(s);
    pt = planning_math::PathPoint(x, y, s, smooth_l, ref_pt.theta(),
                                  ref_pt.kappa(), ref_pt.dkappa(),
                                  ref_pt.ddkappa());
  }
}

void LaneBorrowDeciderV3Utils::AdjustRefinedPathsByNonBorrowObstacles(
    const LaneBorrowDeciderOutput& lane_borrow_output,
    const std::vector<std::shared_ptr<FrenetObstacle>>& obstacles,
    const std::function<planning_math::Box2d(const Obstacle*, double)>&
        predict_box,
    std::unordered_map<uint32_t, LatObstacleDecisionType>&
        lat_obstacle_decision,
    const double lon_min_s, const double lon_max_s,
    std::shared_ptr<FrenetObstacle>& nearest_static_non_borrow_obstacle,
    std::shared_ptr<FrenetObstacle>& nearest_dynamic_non_borrow_obstacle) {
  const auto ref_sl = BuildRefSlFromRefinedPaths(
      refined_paths_, lane_borrow_output.dp_path_coord);

  if (ref_sl.empty() || lane_borrow_output.borrow_direction == NO_BORROW) {
    return;
  }

  constexpr double kMinNonBorrowClearance = 0.5;  // m
  constexpr double kMinBlockedClearance = 0.5;    // m
  constexpr double kObsWindowPre = 5.0;           // m
  constexpr double kObsWindowPost = 10.0;         // m
  constexpr double kMaxTotalDeltaL = 1.0;         // m, avoid extreme jump

  const double sign = (lane_borrow_output.borrow_direction == LEFT_BORROW)
                          ? -1.0
                          : 1.0;  // reverse direction

  const double tail_s_begin = ref_sl.back().s + init_sl_point_.s - 20.0;
  // 收集对ref产生影响的障碍物
  struct AdjustReq {
    int32_t obs_id = 0;
    double s0 = 0.0;
    double s1 = 0.0;
    double delta_need = 0.0;
    bool is_dynamic = false;
  };
  std::vector<AdjustReq> reqs;
  reqs.reserve(obstacles.size());

  for (const auto& obstacle : obstacles) {
    // 筛选需要对ref产生影响的障碍物
    if (obstacle == nullptr || obstacle->obstacle() == nullptr ||
        !obstacle->b_frenet_valid()) {
      continue;
    }
    const int32_t obs_id = obstacle->obstacle()->id();
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  obs_id) != static_blocked_obj_id_vec_.end()) {
      continue;
    }
    const auto it = lat_obstacle_decision.find(obs_id);
    if (it == lat_obstacle_decision.end()) {
      continue;
    }
    if (it->second != LatObstacleDecisionType::IGNORE &&
        it->second != LatObstacleDecisionType::FOLLOW) {
      continue;
    }
    const auto& obs_sl = obstacle->frenet_obstacle_boundary();
    const double s0 = std::max(lon_min_s, obs_sl.s_start - kObsWindowPre);
    const double s1 = std::min(lon_max_s, obs_sl.s_end + kObsWindowPost);
    const double l_min = std::min(obs_sl.l_start, obs_sl.l_end);
    const double l_max = std::max(obs_sl.l_start, obs_sl.l_end);
    const double min_clear = ComputeMinClearanceOverS(
        ref_sl, s0 - init_sl_point_.s, s1 - init_sl_point_.s, l_min, l_max);
    if (min_clear <= 0.1) {
      continue;
    }
    // if (min_clear >= kMinNonBorrowClearance) {
    //   continue;
    // }
    AdjustReq r;
    r.obs_id = obs_id;
    r.s0 = s0;
    r.s1 = s1;
    r.delta_need = (kMinNonBorrowClearance - min_clear);
    r.is_dynamic = !obstacle->obstacle()->is_static();
    if (r.delta_need > 0.0) {
      reqs.emplace_back(std::move(r));
    }
  }

  auto build_delta_profile = [&](double scale) {
    std::vector<double> delta_l(ref_sl.size(), 0.0);
    for (const auto& r : reqs) {
      const double apply_s0 = std::min(r.s0, tail_s_begin);
      const double apply_s1 = std::min(r.s1, tail_s_begin);
      if (apply_s1 <= apply_s0 + 1e-3) {
        continue;
      }
      AddDeltaLWindow(delta_l, ref_sl, apply_s0 - init_sl_point_.s,
                      apply_s1 - init_sl_point_.s,
                      sign * std::min(kMaxTotalDeltaL, r.delta_need) * scale);
    }
    // Clamp total delta for safety.
    for (auto& d : delta_l) {
      d = std::clamp(d, -kMaxTotalDeltaL, kMaxTotalDeltaL);
    }
    // Force tail to converge (no delta) near the end.
    for (size_t i = 0; i < ref_sl.size(); ++i) {
      if (ref_sl[i].s + init_sl_point_.s >= tail_s_begin) {
        delta_l[i] = 0.0;
      }
    }
    return delta_l;
  };

  auto check_blocked_clearance_ok = [&](const std::vector<double>& delta_l) {
    // Check blocked obstacles clearance after applying delta on ref.
    // If any blocked obstacle becomes too close, reject.
    for (const int blocked_id : static_blocked_obj_id_vec_) {
      auto it_obs = std::find_if(
          obstacles.begin(), obstacles.end(),
          [blocked_id](const std::shared_ptr<FrenetObstacle>& o) {
            return o != nullptr && o->obstacle() != nullptr &&
                   o->obstacle()->id() == blocked_id && o->b_frenet_valid();
          });
      if (it_obs == obstacles.end()) {
        continue;
      }
      const auto& sl = (*it_obs)->frenet_obstacle_boundary();
      const double s0 = sl.s_start - 2.0;
      const double s1 = sl.s_end + 2.0;
      const double l_min = std::min(sl.l_start, sl.l_end);
      const double l_max = std::max(sl.l_start, sl.l_end);
      // sample on ref_sl
      double min_clear = std::numeric_limits<double>::max();
      for (size_t i = 0; i < ref_sl.size(); ++i) {
        const double s = ref_sl[i].s + init_sl_point_.s;
        if (s < s0 || s > s1) {
          continue;
        }
        const double ref_l = ref_sl[i].l + delta_l[i];
        min_clear = std::min(min_clear, ClearanceToLRange(ref_l, l_min, l_max));
      }
      if (min_clear < kMinBlockedClearance) {
        return false;
      }
    }
    return true;
  };

  std::vector<double> chosen_delta;
  double scale = 1.0;
  bool ok = false;
  for (int iter = 0; iter < 4; ++iter) {
    auto delta = build_delta_profile(scale);
    if (check_blocked_clearance_ok(delta)) {
      chosen_delta = std::move(delta);
      ok = true;
      break;
    }
    scale *= 0.5;
  }

  auto apply_delta_to_refined_paths = [&](const std::vector<double>& delta) {
    const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
    if (frenet_coord == nullptr) {
      return;
    }
    size_t idx_hint = 0U;
    for (auto& pt : refined_paths_) {
      double s = 0.0, l = 0.0;
      if (!frenet_coord->XYToSL(pt.x(), pt.y(), &s, &l)) {
        continue;
      }
      const double dl =
          InterpDeltaAtS(ref_sl, delta, s - init_sl_point_.s, &idx_hint);
      const double l_new = l + dl;
      double x = 0.0, y = 0.0;
      if (!frenet_coord->SLToXY(s, l_new, &x, &y)) {
        continue;
      }
      const auto ref_pt = frenet_coord->GetPathPointByS(s);
      pt = planning_math::PathPoint(x, y, s, l_new, ref_pt.theta(),
                                    ref_pt.kappa(), ref_pt.dkappa(),
                                    ref_pt.ddkappa());
    }
  };

  struct EgoSLAtT {
    double t = 0.0;
    double s = 0.0;
    double l = 0.0;
  };
  auto build_ego_traj_by_idm = [&]() -> std::vector<EgoSLAtT> {
    std::vector<EgoSLAtT> ego_traj;
    constexpr double kTimeStep = 0.2;
    constexpr double kTotalTime = 5.0;
    const int total_steps = static_cast<int>(kTotalTime / kTimeStep);

    // Lead: compare the two candidates, whoever is nearer.
    auto compute_ds = [&](const std::shared_ptr<FrenetObstacle>& o) {
      if (o == nullptr || o->obstacle() == nullptr || !o->b_frenet_valid()) {
        return std::numeric_limits<double>::max();
      }
      const auto& sl = o->frenet_obstacle_boundary();
      if (sl.s_start <= ego_frenet_boundary_.s_end) {
        return std::numeric_limits<double>::max();
      }
      return sl.s_start - ego_frenet_boundary_.s_end;
    };
    const double dyn_ds = compute_ds(nearest_dynamic_non_borrow_obstacle);
    const double sta_ds = compute_ds(nearest_static_non_borrow_obstacle);
    std::shared_ptr<FrenetObstacle> lead_obs =
        (dyn_ds <= sta_ds) ? nearest_dynamic_non_borrow_obstacle
                           : nearest_static_non_borrow_obstacle;

    const agent::Agent* lead_agent =
        (lead_obs != nullptr && lead_obs->obstacle() != nullptr)
            ? session_->environmental_model().get_agent_manager()->GetAgent(
                  lead_obs->obstacle()->id())
            : nullptr;
    const auto* lead_traj =
        (lead_agent != nullptr &&
         !lead_agent->trajectories_used_by_st_graph().empty() &&
         !lead_agent->trajectories_used_by_st_graph()[0].empty())
            ? &lead_agent->trajectories_used_by_st_graph()[0]
            : nullptr;

    BasicIntelligentDriverModel idm;
    BasicIntelligentDriverModel::ModelParam idm_param;
    idm_param.kDesiredVelocity = std::max(ego_v_, 0.1);
    idm_param.kVehicleLength =
        VehicleConfigurationContext::Instance()->get_vehicle_param().length;

    double ego_s = init_sl_point_.s;
    double ego_v = std::max(ego_v_, 0.0);
    size_t ref_hint = 0U;

    ego_traj.reserve(static_cast<size_t>(total_steps) + 1U);
    for (int step = 0; step <= total_steps; ++step) {
      const double t = step * kTimeStep;
      const double ego_l =
          InterpRefLAtS(ref_sl, ego_s - init_sl_point_.s, &ref_hint);
      ego_traj.push_back({t, ego_s, ego_l});

      double lead_s = ego_s + 1e6;
      double lead_v = ego_v;
      if (lead_traj != nullptr) {
        const int idx = std::min(step, static_cast<int>(lead_traj->size() - 1));
        const auto& lead_pt = (*lead_traj)[idx];
        double s = 0.0, l = 0.0;
        if (lane_borrow_output.dp_path_coord->XYToSL(lead_pt.x(), lead_pt.y(),
                                                     &s, &l)) {
          lead_s = s;
          lead_v = lead_pt.vel();
        }
      }
      BasicIntelligentDriverModel::ModelState state(ego_s, ego_v, lead_s,
                                                    lead_v);
      double acc = 0.0;
      idm.GetIIdmDesiredAcceleration(idm_param, state, &acc);
      ego_s += ego_v * kTimeStep + 0.5 * acc * kTimeStep * kTimeStep;
      ego_v = std::max(0.0, ego_v + acc * kTimeStep);
    }
    return ego_traj;
  };

  auto interaction_check_with_ego_traj =
      [&](const std::vector<EgoSLAtT>& ego_traj,
          const std::vector<int32_t>& req_ids) -> std::vector<int32_t> {
    std::vector<int32_t> unsafe_ids;
    if (ego_traj.empty() || req_ids.empty()) {
      return unsafe_ids;
    }
    constexpr double kMinInteractClearanceStatic = 0.5;   // m
    constexpr double kMinInteractClearanceDynamic = 1.0;  // m
    std::vector<int32_t> check_ids = req_ids;
    for (const auto& ego : ego_traj) {
      for (const int32_t obs_id : check_ids) {
        auto obstacle_it =
            std::find_if(obstacles.begin(), obstacles.end(),
                         [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                           return o != nullptr && o->obstacle() != nullptr &&
                                  o->obstacle()->id() == obs_id;
                         });
        const std::shared_ptr<FrenetObstacle> obstacle =
            (obstacle_it == obstacles.end()) ? nullptr : *obstacle_it;
        const Obstacle* obs =
            (obstacle != nullptr) ? obstacle->obstacle() : nullptr;
        if (obs == nullptr) {
          continue;
        }
        const planning_math::Box2d pred_box = predict_box(obs, ego.t);
        const auto corners = pred_box.GetAllCorners();
        double l_min = std::numeric_limits<double>::max();
        double l_max = std::numeric_limits<double>::lowest();
        for (const auto& p : corners) {
          double s = 0.0, l = 0.0;
          if (!lane_borrow_output.dp_path_coord->XYToSL(p.x(), p.y(), &s, &l)) {
            continue;
          }
          l_min = std::min(l_min, l);
          l_max = std::max(l_max, l);
        }
        if (l_min > l_max) {
          continue;
        }
        const bool is_static =
            (obstacle != nullptr && obstacle->obstacle() != nullptr)
                ? obstacle->obstacle()->is_static()
                : false;
        const double clearance_thr = is_static ? kMinInteractClearanceStatic
                                               : kMinInteractClearanceDynamic;
        if (ClearanceToLRange(ego.l, l_min, l_max) < clearance_thr) {
          unsafe_ids.push_back(obs_id);
        }
      }
    }
    std::sort(unsafe_ids.begin(), unsafe_ids.end());
    unsafe_ids.erase(std::unique(unsafe_ids.begin(), unsafe_ids.end()),
                     unsafe_ids.end());
    return unsafe_ids;
  };

  if (ok && chosen_delta.size() == ref_sl.size()) {
    // Validate obstacles (static & dynamic) that actually triggered adjustment.
    std::vector<int32_t> req_ids;
    req_ids.reserve(reqs.size());
    for (const auto& r : reqs) {
      req_ids.push_back(r.obs_id);
    }
    const auto ego_traj = build_ego_traj_by_idm();
    const auto unsafe_ids = interaction_check_with_ego_traj(ego_traj, req_ids);
    // For unsafe obstacles: force FOLLOW. For safe ones: mark as avoid
    // (LEFT/RIGHT).
    for (const int32_t id : unsafe_ids) {
      lat_obstacle_decision[static_cast<uint32_t>(id)] =
          LatObstacleDecisionType::FOLLOW;
    }
    for (const int32_t id : req_ids) {
      if (std::find(unsafe_ids.begin(), unsafe_ids.end(), id) !=
          unsafe_ids.end()) {
        continue;
      }
      // safe -> avoidance direction
      auto it_obs = std::find_if(
          obstacles.begin(), obstacles.end(),
          [id](const std::shared_ptr<FrenetObstacle>& o) {
            return o != nullptr && o->obstacle() != nullptr &&
                   o->obstacle()->id() == id && o->b_frenet_valid();
          });
      if (it_obs == obstacles.end()) {
        continue;
      }
      const auto& obs_sl = (*it_obs)->frenet_obstacle_boundary();
      const double obstacle_center_l = (obs_sl.l_start + obs_sl.l_end) / 2.0;
      const double obs_center_s = 0.5 * (obs_sl.s_start + obs_sl.s_end);
      size_t ref_hint = 0U;
      const double ref_l =
          InterpRefLAtS(ref_sl, obs_center_s - init_sl_point_.s, &ref_hint);
      const double rel_l = obstacle_center_l - ref_l;
      const auto avoid = (rel_l > 0.0) ? LatObstacleDecisionType::RIGHT
                                       : LatObstacleDecisionType::LEFT;
      lat_obstacle_decision[static_cast<uint32_t>(id)] = avoid;
    }

    if (!unsafe_ids.empty()) {
      reqs.erase(std::remove_if(
                     reqs.begin(), reqs.end(),
                     [&](const AdjustReq& r) {
                       return std::find(unsafe_ids.begin(), unsafe_ids.end(),
                                        r.obs_id) != unsafe_ids.end();
                     }),
                 reqs.end());
      ok = false;
      scale = 1.0;
      for (int iter = 0; iter < 4; ++iter) {
        auto delta = build_delta_profile(scale);
        if (check_blocked_clearance_ok(delta)) {
          chosen_delta = std::move(delta);
          ok = true;
          break;
        }
        scale *= 0.5;
      }
    }
    if (ok && chosen_delta.size() == ref_sl.size()) {
      apply_delta_to_refined_paths(chosen_delta);
    }
  }
}

bool LaneBorrowDeciderV3Utils::Execute() { return true; }

bool LaneBorrowDeciderV3Utils::ProcessEnvInfos(
    const LaneBorrowDeciderOutput* lane_borrow_output,
    const LaneBorrowStatus lane_borrow_status,
    const std::vector<int>& static_blocked_obj_id_vec) {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& agents = dynamic_world->agent_manager()->GetAllCurrentAgents();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();
  if (current_lane_ptr_ == nullptr || agents.empty()) {
    last_back_origin_valid_ = false;
    return false;
  }
  current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();
  if (current_reference_path_ptr_ == nullptr) {
    return false;
  }
  const auto& current_frenet_coord =
      current_reference_path_ptr_->get_frenet_coord();
  ego_frenet_state_ = session_->environmental_model()
                          .get_reference_path_manager()
                          ->get_reference_path_by_current_lane()
                          ->get_frenet_ego_state();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  ego_s_ = ego_frenet_state_.s();
  ego_l_ = ego_frenet_state_.l();
  ego_v_ = ego_frenet_state_.velocity();
  ego_cartes_state.x =
      session_->environmental_model().get_ego_state_manager()->ego_carte().x;
  ego_cartes_state.y =
      session_->environmental_model().get_ego_state_manager()->ego_carte().y;
  v_cruise_ =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
  vehicle_width_ =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  vehicle_length_ =
      VehicleConfigurationContext::Instance()->get_vehicle_param().length;
  const auto& planning_init_point =
      current_reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  init_sl_point_.l = planning_init_point.frenet_state.r;
  init_sl_point_.s = planning_init_point.frenet_state.s;
  lane_borrow_status_ = lane_borrow_status;
  last_static_blocked_obj_id_vec_ = static_blocked_obj_id_vec_;
  static_blocked_obj_id_vec_ = static_blocked_obj_id_vec;
  return true;
}

bool LaneBorrowDeciderV3Utils::GenerateBackOriginLaneRulePath() {
  // From planning init (s,l) on the reference centerline frame: quintic l(s)
  // merges lateral offset to l=0, with longitudinal span derived from lateral
  // jerk bound 0.6 m/s^3 (same minimum-jerk time bound as borrow transitions).
  if (current_reference_path_ptr_ == nullptr) {
    last_back_origin_valid_ = false;
    return false;
  }
  const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    last_back_origin_valid_ = false;
    return false;
  }

  const double init_s = init_sl_point_.s;
  const double init_l = init_sl_point_.l;

  constexpr double kLatJerkMax = 0.8;  // m/s^3
  constexpr double kMinTransS = 15.0;
  constexpr double kMaxTransS = 35.0;
  constexpr double kSResolution = 0.5;
  constexpr double kTailCenterlineTotalLength = 80.0;
  constexpr double kMinRemainTransS = 5.0;  // 剩余过渡距离下限

  const double ego_v = std::max(ego_v_, 0.1);
  const double L = std::max(std::fabs(init_l), 1e-3);
  const double T = std::cbrt((60.0 * L) / kLatJerkMax);
  const double fresh_trans_s =
      std::min(kMaxTransS, std::max(kMinTransS, ego_v * T));

  double s_merge_end;
  if (last_back_origin_valid_) {
    double last_s = 0.0, last_l = 0.0;
    if (frenet_coord->XYToSL(last_back_origin_merge_end_x_,
                             last_back_origin_merge_end_y_, &last_s, &last_l) &&
        last_s > init_s) {
      s_merge_end = std::min(last_s, init_s + fresh_trans_s);
    } else {
      s_merge_end = init_s + fresh_trans_s;
    }
  } else {
    s_merge_end = init_s + fresh_trans_s;
  }
  // 保证剩余过渡距离不会太小，避免极端曲率
  s_merge_end = std::max(s_merge_end, init_s + kMinRemainTransS);
  double end_x = 0.0, end_y = 0.0;
  if (frenet_coord->SLToXY(s_merge_end, 0.0, &end_x, &end_y)) {
    last_back_origin_merge_end_x_ = end_x;
    last_back_origin_merge_end_y_ = end_y;
    last_back_origin_valid_ = true;
  }
  const double trans_s = s_merge_end - init_s;
  double s_end =
      std::max(s_merge_end + 5.0, init_s + kTailCenterlineTotalLength);
  s_end = std::max(s_end, kTailCenterlineTotalLength + ego_s_);

  refined_paths_.clear();
  refined_paths_.reserve(static_cast<size_t>((s_end - init_s) / kSResolution) +
                         10U);

  auto emit_point = [&](double s, double l) {
    double x = 0.0, y = 0.0;
    if (!frenet_coord->SLToXY(s, l, &x, &y)) {
      return;
    }
    const auto ref_pt = frenet_coord->GetPathPointByS(s);
    refined_paths_.emplace_back(
        planning_math::PathPoint(x, y, s, l, ref_pt.theta(), ref_pt.kappa(),
                                 ref_pt.dkappa(), ref_pt.ddkappa()));
  };

  const double ds_total = trans_s;
  if (ds_total <= kMinSDelta) {
    return false;
  }

  planning_math::QuinticPolynomialCurve1d curve(
      init_l, ego_frenet_state_.planning_init_point().frenet_state.dr_ds,
      ego_frenet_state_.planning_init_point().frenet_state.ddr_dsds, 0.0, 0.0,
      0.0, ds_total);
  for (double s = init_s; s < s_merge_end - kMinSDelta; s += kSResolution) {
    const double ds = s - init_s;
    const double l = curve.Evaluate(0, ds);
    if (refined_paths_.empty() || s > refined_paths_.back().s() + kMinSDelta) {
      emit_point(s, l);
    }
  }
  emit_point(s_merge_end, 0.0);

  for (double s = s_merge_end + kSResolution; s < s_end - kMinSDelta;
       s += kSResolution) {
    if (s > refined_paths_.back().s() + kMinSDelta) {
      emit_point(s, 0.0);
    }
  }
  emit_point(s_end, 0.0);

  return refined_paths_.size() >= 3U;
}

bool LaneBorrowDeciderV3Utils::GenerateOriginRulePath(
    const std::unordered_map<int, std::pair<BorrowDirection, int>>&
        obs_direction_map) {
  last_back_origin_valid_ = false;
  if (static_blocked_obj_id_vec_.empty()) {
    return false;
  }
  const double init_s = init_sl_point_.s;
  const double init_l = init_sl_point_.l;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  // 参考DesireLateralOffsetSideWay函数，计算每个借道障碍物期望的横向移动距离，用map存储
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  constexpr double kBaseDistance = 0.7;

  std::unordered_map<int, double> obs_desire_lat_offset_map;
  obs_desire_lat_offset_map.reserve(static_blocked_obj_id_vec_.size());
  double speed_extra_buffer = 0.2;
  // interp(ego_cart_state_manager_->ego_v() * 3.6,
  //        config_.lateral_offset_obstacle_nudge_buffer_v_bp,
  //        config_.lateral_offset_nudge_buffer);

  for (const int obs_id : static_blocked_obj_id_vec_) {
    // 上游保证了static_blocked_obj_id_vec_里面的借道障碍物借道方向同向！
    auto it =
        std::find_if(obstacles.begin(), obstacles.end(),
                     [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                       return o != nullptr && o->obstacle()->id() == obs_id;
                     });
    if (it == obstacles.end()) {
      continue;
    }
    const auto& obs_sl = (*it)->frenet_obstacle_boundary();
    // nearest_l_to_ref: obstacle edge closest to reference line (l=0)
    double extra_buffer = 0.0;
    if ((*it)->obstacle()->is_VRU()) {
      extra_buffer = 0.15;
    } else if ((*it)->obstacle()->is_oversize_vehicle()) {
      extra_buffer = 0.3;
    }
    BorrowDirection dir = NO_BORROW;
    const auto dir_it = obs_direction_map.find(obs_id);
    if (dir_it != obs_direction_map.end()) {
      dir = dir_it->second.first;
    } else {
      continue;
    }
    const double l_to_ref =
        (dir == LEFT_BORROW) ? obs_sl.l_end : obs_sl.l_start;
    const double clearance =
        half_ego_width + kBaseDistance + extra_buffer + speed_extra_buffer;
    obs_desire_lat_offset_map[obs_id] =
        (dir == LEFT_BORROW) ? (l_to_ref + clearance) : (l_to_ref - clearance);
  }
  if (obs_desire_lat_offset_map.empty()) {
    return false;
  }
  struct OffsetSegment {
    double enter_s = 0.0;       // 开始偏移
    double peak_start_s = 0.0;  // （到达目标偏移）
    double peak_end_s = 0.0;    // （保持目标偏移）
    double exit_s = 0.0;        // （回到 0）
    double target_l = 0.0;
  };

  std::vector<OffsetSegment> segments;
  segments.reserve(obs_desire_lat_offset_map.size());

  // Lateral jerk bound (hyper-parameter), used to compute a reasonable
  // longitudinal distance for l(s) transition under constant-speed assumption.
  // Minimum-jerk profile: l(t)=L(10u^3-15u^4+6u^5), max|jerk| = 60|L|/T^3.
  constexpr double kLatJerkMax = 0.6;     // m/s^3
  constexpr double kMinTransS = 15.0;     // m 避免自车速度较低
  constexpr double kMaxTransS = 50.0;     // m (avoid overly long transition)
  constexpr double kCareDistance = 10.0;  // m
  constexpr double kDynPeakLookaheadTime = 5.0;        // s
  constexpr double kDynPeakMinExtraS = 10.0;           // m
  constexpr double kDynPeakMaxExtraS = 45.0;           // m
  constexpr double kOncomingEarlyLookaheadTime = 3.5;  // s
  constexpr double kOncomingEarlyMinS = 10.0;          // m
  constexpr double kOncomingEarlyMaxS = 20.0;          // m
  constexpr double kDynPeakStartMaxShiftS = 15.0;      // m

  const double ego_v = std::max(ego_v_, 0.1);  // m/s, guard zero speed

  const auto calc_trans_s = [&](double delta_l) {
    const double L = std::max(std::fabs(delta_l), 1e-3);
    const double T = std::cbrt((60.0 * L) / kLatJerkMax);
    const double S = ego_v * T;
    return std::min(kMaxTransS, std::max(kMinTransS, S));
  };

  for (const auto& kv : obs_desire_lat_offset_map) {
    const int obs_id = kv.first;
    const double target_l = kv.second;
    auto it =
        std::find_if(obstacles.begin(), obstacles.end(),
                     [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                       return o != nullptr && o->obstacle()->id() == obs_id;
                     });
    if (it == obstacles.end()) {
      continue;
    }
    const auto& obs_ptr = *it;
    const auto& sl = obs_ptr->frenet_obstacle_boundary();
    const double trans_s = calc_trans_s(target_l);
    OffsetSegment seg;
    const bool is_static =
        obs_ptr->obstacle() != nullptr && obs_ptr->obstacle()->is_static();
    const bool oncoming_dynamic = !is_static &&
                                  obs_ptr->obstacle() != nullptr &&
                                  (obs_ptr->obstacle()->is_reverse());
    double early_s = 0.0;
    if (oncoming_dynamic) {
      const double v_close =
          std::max(0.1, std::fabs(obs_ptr->frenet_velocity_s()));
      early_s = std::min(
          kOncomingEarlyMaxS,
          std::max(kOncomingEarlyMinS, v_close * kOncomingEarlyLookaheadTime));
    }
    double dyn_shift_s = 0.0;
    double dyn_t_reach = 0.0;
    if (!is_static && !oncoming_dynamic) {
      // Meet-time should be based on relative closing speed, not ego speed
      // only. Use ego head s (boundary.s_end) to be consistent with obstacle
      // s_start.
      const double ds_ahead =
          std::max(0.0, sl.s_start - ego_frenet_boundary_.s_end);
      const double vs = std::max(0.0, obs_ptr->frenet_velocity_s());
      // Assume ego accelerates with constant a (1.5 m/s^2) to estimate
      // catch-up. Solve: ds_ahead = (ego_v - vs) * t + 0.5 * a * t^2
      constexpr double kEgoAssumedAcc = 1.5;  // m/s^2
      const double v_rel0 = ego_v - vs;
      if (ds_ahead <= 1e-6) {
        dyn_t_reach = 0.0;
      } else {
        const double disc = v_rel0 * v_rel0 + 2.0 * kEgoAssumedAcc * ds_ahead;
        dyn_t_reach =
            (-v_rel0 + std::sqrt(std::max(0.0, disc))) / kEgoAssumedAcc;
      }
      dyn_t_reach = std::clamp(dyn_t_reach, 0.0, kDynPeakLookaheadTime);
      dyn_shift_s = std::min(kDynPeakStartMaxShiftS, vs * dyn_t_reach);
    }
    seg.peak_start_s = sl.s_start - 2.0 - early_s + dyn_shift_s;
    double peak_end_s = sl.s_end + 2.0;
    if (!is_static && obs_ptr->obstacle() != nullptr && !oncoming_dynamic) {
      const double vs = std::max(0.0, obs_ptr->frenet_velocity_s());
      const double extra_s =
          std::min(kDynPeakMaxExtraS,
                   std::max(kDynPeakMinExtraS, vs * kDynPeakLookaheadTime));
      peak_end_s = seg.peak_start_s + extra_s;
    }
    seg.peak_end_s = peak_end_s;
    seg.enter_s = sl.s_start - trans_s - early_s + dyn_shift_s;
    seg.exit_s = peak_end_s + trans_s;
    seg.target_l = target_l;
    segments.emplace_back(seg);
  }
  if (segments.empty()) {
    return false;
  }

  std::sort(segments.begin(), segments.end(),
            [](const OffsetSegment& a, const OffsetSegment& b) {
              return a.peak_start_s < b.peak_start_s;
            });

  // Merge overlapping segments and keep the larger |target_l| (prefer same
  // sign).
  std::vector<OffsetSegment> merged;
  merged.reserve(segments.size());
  for (const auto& seg : segments) {
    if (merged.empty() || seg.enter_s > merged.back().exit_s) {
      merged.emplace_back(seg);
      continue;
    }
    auto& cur = merged.back();
    cur.enter_s = std::min(cur.enter_s, seg.enter_s);
    cur.peak_start_s = std::min(cur.peak_start_s, seg.peak_start_s);
    cur.peak_end_s = std::max(cur.peak_end_s, seg.peak_end_s);
    cur.exit_s = std::max(cur.exit_s, seg.exit_s);
    if (std::fabs(seg.target_l) > std::fabs(cur.target_l)) {
      cur.target_l = seg.target_l;
    } else if (std::fabs(seg.target_l) == std::fabs(cur.target_l) &&
               (cur.target_l * seg.target_l) < 0.0) {
      // same magnitude but opposite sign: keep current (do nothing)
    }
  }

  const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return false;
  }

  constexpr double kSResolution = 0.5;  // s方向的步长
  // Make refined path start from planning_init_point, so the output path can be
  // stitched naturally with planner init state in the first frame.
  const double s_begin = init_s;
  // Extend the tail along centerline (l=0) to avoid too-short refined_paths_
  // in some corner cases (e.g. short obstacle segment or early exit).
  constexpr double kTailCenterlineTotalLength = 60;  // m
  double s_end = std::max(merged.back().exit_s, s_begin + 5.0);
  s_end = std::max(s_end, kTailCenterlineTotalLength + s_begin);

  struct Anchor {
    double s = 0.0;
    double l = 0.0;
  };
  std::vector<Anchor> anchors;
  anchors.reserve(merged.size() * 4 + 2);
  anchors.push_back({s_begin, init_l});
  for (const auto& seg : merged) {
    // Only add anchors genuinely ahead of s_begin; clamped anchors would
    // pile up at s_begin and corrupt the init point during dedup.
    if (seg.enter_s > s_begin) {
      anchors.push_back({seg.enter_s, init_l});
    }
    // If ego is already inside the peak zone, choose the l with larger
    // magnitude between init_l and target_l to avoid pulling ego back
    // when it is further from centerline than target_l.
    const bool is_reach = std::fabs(init_l) > std::fabs(seg.target_l);
    const double effective_l =
        (is_reach && init_l * seg.target_l >= 0.0) ? init_l : seg.target_l;
    if (seg.peak_start_s > s_begin) {
      anchors.push_back({seg.peak_start_s, effective_l});
    }
    if (seg.peak_end_s > s_begin) {
      anchors.push_back({seg.peak_end_s, effective_l});
    }
    if (seg.exit_s > s_begin) {
      anchors.push_back({seg.exit_s, 0.0});
    }
  }
  anchors.push_back({s_end, 0.0});

  // Deduplicate & ensure strictly increasing s.
  std::sort(anchors.begin(), anchors.end(),
            [](const Anchor& a, const Anchor& b) { return a.s < b.s; });
  std::vector<Anchor> uniq;
  uniq.reserve(anchors.size());
  for (const auto& a : anchors) {
    if (uniq.empty() || a.s > uniq.back().s + kMinSDelta) {
      uniq.push_back(a);
    } else {
      // same/very close s: keep larger |l|
      if (std::fabs(a.l) > std::fabs(uniq.back().l)) {
        uniq.back().l = a.l;
      }
    }
  }
  if (uniq.size() < 2) {
    return false;
  }

  // Ensure the first segment has at least 5m to allow smooth lateral transition
  // from init_l. If the second anchor is too close, push it forward.
  constexpr double kMinFirstSegmentLength = 5.0;  // m
  if (uniq.size() >= 2 && uniq[1].s < uniq[0].s + kMinFirstSegmentLength) {
    uniq[1].s = uniq[0].s + kMinFirstSegmentLength;
    // Propagate: ensure each subsequent segment also has at least
    // kMinFirstSegmentLength to keep lateral transitions smooth.
    for (size_t i = 2; i < uniq.size(); ++i) {
      if (uniq[i].s < uniq[i - 1].s + kMinFirstSegmentLength) {
        uniq[i].s = uniq[i - 1].s + kMinFirstSegmentLength;
      }
    }
  }

  refined_paths_.clear();
  refined_paths_.reserve(static_cast<size_t>((s_end - s_begin) / kSResolution) +
                         10);

  auto emit_point = [&](double s, double l) {
    double x = 0.0, y = 0.0;
    if (!frenet_coord->SLToXY(s, l, &x, &y)) {
      return;
    }
    const auto ref_pt = frenet_coord->GetPathPointByS(s);
    refined_paths_.emplace_back(
        planning_math::PathPoint(x, y, s, l, ref_pt.theta(), ref_pt.kappa(),
                                 ref_pt.dkappa(), ref_pt.ddkappa()));
  };

  // Build piecewise quintic l(s) between anchors and sample.
  // 第一段起点导数从自车当前状态继承
  const double init_dl_ds =
      ego_frenet_state_.planning_init_point().frenet_state.dr_ds;
  const double init_ddl_dsds =
      ego_frenet_state_.planning_init_point().frenet_state.ddr_dsds;
  for (size_t i = 0; i + 1 < uniq.size(); ++i) {
    const double s0 = uniq[i].s;
    const double s1 = uniq[i + 1].s;
    const double l0 = uniq[i].l;
    const double l1 = uniq[i + 1].l;
    if (s1 <= s0 + kMinSDelta) {
      continue;
    }

    const double ds_total = s1 - s0;
    // 仅第一段使用自车起点导数，后续段起止导数均为 0（锚点处切线平行参考线）
    const double seg_dl0 = (i == 0) ? init_dl_ds : 0.0;
    const double seg_ddl0 = (i == 0) ? init_ddl_dsds : 0.0;
    planning_math::QuinticPolynomialCurve1d curve(l0, seg_dl0, seg_ddl0, l1,
                                                  0.0, 0.0, ds_total);
    for (double s = s0; s < s1 - kMinSDelta; s += kSResolution) {
      const double ds = s - s0;
      const double l = curve.Evaluate(0, ds);
      if (refined_paths_.empty() ||
          s > refined_paths_.back().s() + kMinSDelta) {
        emit_point(s, l);
      }
    }
  }
  // Ensure last point.
  emit_point(uniq.back().s, uniq.back().l);

  // Ensure enough points for later spline usage.
  return refined_paths_.size() >= 3;
}

bool LaneBorrowDeciderV3Utils::CartSpline(
    LaneBorrowDeciderOutput* lane_borrow_decider_output) {
  std::vector<double> s_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  if (refined_paths_.size() < 2) {
    ref_path_curve_.x_vec.clear();
    ref_path_curve_.y_vec.clear();
    ref_path_curve_.s_vec.clear();
    ref_path_curve_.k_vec.clear();
    return false;
  }
  if (lane_borrow_decider_output->lane_borrow_failed_reason ==
          NEARBY_OBSTACLE_TOO_CLOSE ||
      lane_borrow_decider_output->lane_borrow_failed_reason ==
          BACKWARD_OBSTACLE_TOO_CLOSE ||
      lane_borrow_decider_output->lane_borrow_failed_reason ==
          AHEAD_COMING_OBS) {
    last_frame_paths_.clear();
    // 修饰后的不能滞回
  } else {
    last_frame_paths_ =
        refined_paths_;  // last paths is not  updated until CartSpline
  }
  std::unordered_map<int, bool> cur_obs_static_map;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  for (const int obs_id : static_blocked_obj_id_vec_) {
    auto it = std::find_if(obstacles.begin(), obstacles.end(),
                           [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                             return o != nullptr && o->obstacle() != nullptr &&
                                    o->obstacle()->id() == obs_id;
                           });
    if (it != obstacles.end()) {
      cur_obs_static_map[obs_id] = (*it)->obstacle()->is_static();
    }
  }
  last_obs_static_map_ = std::move(cur_obs_static_map);
  // s 的增量必须不小于最小阈值
  // 如果当前 s 值增量小于阈值，则调整为 last_s + kMinSDelta
  double last_s = std::numeric_limits<double>::lowest();
  for (const auto& path_point : refined_paths_) {
    double cur_s = path_point.s();
    // 取当前 s 和 (上一个 s + 最小增量) 的较大值，确保严格递增且增量不小于阈值
    double adjusted_s = std::max(cur_s, last_s + kMinSDelta);
    s_vec.emplace_back(adjusted_s);
    x_vec.emplace_back(path_point.x());
    y_vec.emplace_back(path_point.y());
    last_s = adjusted_s;
  }

  // 检查过滤后的点数是否足够（spline 至少需要 3 个点）
  if (s_vec.size() < 3) {
    ref_path_curve_.x_vec.clear();
    ref_path_curve_.y_vec.clear();
    ref_path_curve_.s_vec.clear();
    ref_path_curve_.k_vec.clear();
    return false;
  }

  ref_path_curve_.x_vec = x_vec;
  ref_path_curve_.y_vec = y_vec;
  ref_path_curve_.s_vec = s_vec;
  ref_path_curve_.x_s_spline.set_points(s_vec, x_vec);
  ref_path_curve_.y_s_spline.set_points(s_vec, y_vec);

  lane_borrow_decider_output->dp_path_coord =
      ConstructLaneBorrowKDPath(x_vec, y_vec);
  lane_borrow_decider_output->dp_path_ref = ref_path_curve_;
  return true;
}

std::shared_ptr<planning_math::KDPath>
LaneBorrowDeciderV3Utils::ConstructLaneBorrowKDPath(
    const std::vector<double>& x_vec, const std::vector<double>& y_vec) {
  if (x_vec.empty() || x_vec.size() != y_vec.size()) {
    return nullptr;
  }
  std::vector<planning_math::PathPoint> rules_path_points;
  rules_path_points.reserve(x_vec.size());
  constexpr double kDuplicateThreshold = 1e-3;
  for (size_t i = 0; i < x_vec.size(); ++i) {
    if (std::isnan(x_vec[i]) || std::isnan(y_vec[i])) {
      ILOG_ERROR << "skip NaN point";
      continue;
    }
    planning_math::PathPoint path_point{x_vec[i], y_vec[i]};
    // Check duplicate before adding
    if (!rules_path_points.empty()) {
      const auto& last_pt = rules_path_points.back();
      if (planning_math::Vec2d(last_pt.x() - path_point.x(),
                               last_pt.y() - path_point.y())
              .Length() < kDuplicateThreshold) {
        continue;
      }
    }
    rules_path_points.emplace_back(path_point);
  }
  if (rules_path_points.size() <=
      planning_math::KDPath::kKDPathMinPathPointSize) {
    return nullptr;
  }
  return std::make_shared<planning_math::KDPath>(std::move(rules_path_points));
}

void LaneBorrowDeciderV3Utils::LogDebugInfo() {
  auto dp_road_pb_info =
      DebugInfoManager::GetInstance().GetDebugInfoPb()->mutable_dp_road_info();
  dp_road_pb_info->Clear();
  // fined path
  for (const auto& path_point : refined_paths_) {
    auto* fined_points_pb = dp_road_pb_info->mutable_dp_result_path()
                                ->mutable_fined_points()
                                ->Add();
    dp_road_pb_info->mutable_dp_result_path()->mutable_fined_xs()->Add(
        path_point.x());
    dp_road_pb_info->mutable_dp_result_path()->mutable_fined_ys()->Add(
        path_point.y());
    fined_points_pb->set_s(path_point.s());
    fined_points_pb->set_l(path_point.l());
  }
}

void LaneBorrowDeciderV3Utils::UpdateObstacleLateralDecisionBaseRulePath(
    const LaneBorrowDeciderOutput& lane_borrow_output,
    std::shared_ptr<FrenetObstacle>& nearest_static_non_borrow_obstacle,
    std::shared_ptr<FrenetObstacle>& nearest_dynamic_non_borrow_obstacle,
    std::unordered_map<int32_t, LatObstacleDecisionType>*
        pending_lat_decisions) {
  if (lane_borrow_output.dp_path_coord == nullptr ||
      lane_borrow_output.lane_borrow_failed_reason ==
          NEARBY_OBSTACLE_TOO_CLOSE ||
      lane_borrow_output.lane_borrow_failed_reason ==
          BACKWARD_OBSTACLE_TOO_CLOSE ||
      lane_borrow_output.lane_borrow_failed_reason == AHEAD_COMING_OBS) {
    rule_lat_decision_hysteresis_map_.clear();
    nearest_static_non_borrow_obstacle = nullptr;
    nearest_dynamic_non_borrow_obstacle = nullptr;
    return;
  }
  pending_lat_decisions->clear();

  constexpr double kPredHorizon = 5.0;
  constexpr double kPredStep = 1.0;
  constexpr int kSwitchHoldFrames = 3;
  constexpr double kLatSafeEnterIgnoreMargin = 0.2;
  constexpr double kLatSafeReleaseIgnoreMargin = 0.2;
  constexpr double dynamic_base_lat_safe_distance = 1.0;
  constexpr double static_base_lat_safe_distance = 0.6;
  constexpr double kDefaultLaneWidth = 3.75;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (current_reference_path_ptr_ == nullptr) {
    rule_lat_decision_hysteresis_map_.clear();
    nearest_static_non_borrow_obstacle = nullptr;
    nearest_dynamic_non_borrow_obstacle = nullptr;
    return;
  }
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const double lon_min_s = ego_frenet_boundary_.s_start;
  double lon_max_s = ego_frenet_boundary_.s_end + 80.0;
  if (lane_borrow_output.dp_path_coord != nullptr) {
    lon_max_s =
        ego_frenet_boundary_.s_end + lane_borrow_output.dp_path_coord->Length();
  }

  const auto predict_box = [](const Obstacle* obstacle, double delta_t) {
    if (obstacle->is_static() || !obstacle->trajectory_valid() ||
        obstacle->trajectory().empty()) {
      return planning_math::Box2d(
          planning_math::Vec2d(obstacle->x_center(), obstacle->y_center()),
          obstacle->heading_angle(), obstacle->length(), obstacle->width());
    }
    const auto pred_point = obstacle->get_point_at_time(delta_t);
    return planning_math::Box2d(planning_math::Vec2d(pred_point.path_point.x(),
                                                     pred_point.path_point.y()),
                                pred_point.path_point.theta(),
                                obstacle->length(), obstacle->width());
  };

  // purge disappeared obstacles
  for (auto it = rule_lat_decision_hysteresis_map_.begin();
       it != rule_lat_decision_hysteresis_map_.end();) {
    const int32_t obs_id = it->first;
    if (std::find_if(obstacles.begin(), obstacles.end(),
                     [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                       return o != nullptr && o->obstacle()->id() == obs_id;
                     }) == obstacles.end()) {
      it = rule_lat_decision_hysteresis_map_.erase(it);
    } else {
      ++it;
    }
  }

  std::unordered_map<int32_t, LatObstacleDecisionType>
      obs_lat_avoid_flag_at_current_pose;

  for (const auto& obstacle : obstacles) {
    if (obstacle == nullptr || obstacle->obstacle() == nullptr) {
      continue;
    }
    const int32_t obs_id = obstacle->obstacle()->id();
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  obs_id) != static_blocked_obj_id_vec_.end()) {
      continue;  // skip lane-borrow blocked obstacles
    }

    const auto& agent = agent_mgr->GetAgent(obs_id);
    if (agent == nullptr) {
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {
      continue;
    }

    const auto& obs_sl = obstacle->frenet_obstacle_boundary();
    if (obs_sl.s_end < lon_min_s || obs_sl.s_start > lon_max_s) {
      continue;
    }

    double lat_safe_distance = obstacle->obstacle()->is_static()
                                   ? static_base_lat_safe_distance
                                   : dynamic_base_lat_safe_distance;
    if (obstacle->obstacle()->is_VRU() ||
        obstacle->obstacle()->is_oversize_vehicle()) {
      lat_safe_distance += 0.3;
    }

    // lat_safe_distance hysteresis based on stable decision
    const auto hist_it = rule_lat_decision_hysteresis_map_.find(obs_id);
    if (hist_it != rule_lat_decision_hysteresis_map_.end()) {
      const auto stable_decision = hist_it->second.first;
      if (stable_decision == LatObstacleDecisionType::IGNORE) {
        lat_safe_distance += kLatSafeReleaseIgnoreMargin;
      } else if (stable_decision == LatObstacleDecisionType::LEFT ||
                 stable_decision == LatObstacleDecisionType::RIGHT) {
        lat_safe_distance =
            std::max(0.0, lat_safe_distance - kLatSafeEnterIgnoreMargin);
      }
    }

    LatObstacleDecisionType candidate = LatObstacleDecisionType::IGNORE;
    // Decide avoidance direction based on obstacle position relative to current
    // ref.
    const double obstacle_center_l = (obs_sl.l_start + obs_sl.l_end) / 2.0;
    const double obs_center_s = 0.5 * (obs_sl.s_start + obs_sl.s_end);
    size_t ref_hint = 0U;
    double s = 0.0, l = 0.0;
    if (lane_borrow_output.dp_path_coord->XYToSL(
            obstacle->obstacle()->x_center(), obstacle->obstacle()->y_center(),
            &s, &l)) {
      candidate = (l > 0.0) ? LatObstacleDecisionType::RIGHT
                            : LatObstacleDecisionType::LEFT;
    } else {
      candidate = LatObstacleDecisionType::IGNORE;
    }

    for (double t = 0.0; t <= kPredHorizon + 1e-6; t += kPredStep) {
      const planning_math::Box2d pred_box =
          predict_box(obstacle->obstacle(), t);
      const auto corners = pred_box.GetAllCorners();

      FrenetObstacleBoundary dp_sl_bd;
      dp_sl_bd.s_start = std::numeric_limits<double>::max();
      dp_sl_bd.s_end = std::numeric_limits<double>::lowest();
      dp_sl_bd.l_start = std::numeric_limits<double>::max();
      dp_sl_bd.l_end = std::numeric_limits<double>::lowest();

      for (const auto& p : corners) {
        double s = 0.0, l = 0.0;
        if (!lane_borrow_output.dp_path_coord->XYToSL(p.x(), p.y(), &s, &l)) {
          continue;
        }
        dp_sl_bd.s_start = std::min(dp_sl_bd.s_start, s);
        dp_sl_bd.s_end = std::max(dp_sl_bd.s_end, s);
        dp_sl_bd.l_start = std::min(dp_sl_bd.l_start, l);
        dp_sl_bd.l_end = std::max(dp_sl_bd.l_end, l);
      }

      const double min_lat_dist_to_path =
          (dp_sl_bd.l_start <= 0.0 && dp_sl_bd.l_end >= 0.0)
              ? 0.0
              : std::min(std::fabs(dp_sl_bd.l_start),
                         std::fabs(dp_sl_bd.l_end));
      if (min_lat_dist_to_path < lat_safe_distance) {
        candidate = LatObstacleDecisionType::IGNORE;
        if (t <= 1e-3) {
          obs_lat_avoid_flag_at_current_pose[obs_id] = candidate;
        }
        break;
      } else {
        if (t <= 1e-3) {
          obs_lat_avoid_flag_at_current_pose[obs_id] = candidate;
        }
      }
    }

    // Apply internal hysteresis (do not rely on previous planning_context).
    auto& hist = rule_lat_decision_hysteresis_map_[obs_id];
    if (hist.second == 0) {
      hist.first = candidate;
      hist.second = 0;
    }

    LatObstacleDecisionType stable = hist.first;
    if (candidate == stable) {
      hist.second = 0;
    } else {
      hist.second += 1;
      if (hist.second >= kSwitchHoldFrames) {
        hist.first = candidate;
        hist.second = 0;
        stable = candidate;
      }
    }

    (*pending_lat_decisions)[obs_id] = stable;
  }

  std::shared_ptr<FrenetObstacle> nearest_static = nullptr;
  std::shared_ptr<FrenetObstacle> nearest_dynamic = nullptr;
  double nearest_static_ds = std::numeric_limits<double>::max();
  double nearest_dynamic_ds = std::numeric_limits<double>::max();

  for (const auto& obstacle : obstacles) {
    if (obstacle == nullptr || obstacle->obstacle() == nullptr ||
        !obstacle->b_frenet_valid()) {
      continue;
    }
    const int32_t obs_id = obstacle->obstacle()->id();
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  obs_id) != static_blocked_obj_id_vec_.end()) {
      continue;
    }
    const auto& obs_sl = obstacle->frenet_obstacle_boundary();
    if (obs_sl.s_start <= ego_frenet_boundary_.s_end) {
      continue;
    }
    if (obs_sl.s_end < lon_min_s || obs_sl.s_start > lon_max_s) {
      continue;
    }
    const auto it = pending_lat_decisions->find(obs_id);
    if (it == pending_lat_decisions->end()) {
      continue;
    }
    if (it->second != LatObstacleDecisionType::IGNORE &&
        it->second != LatObstacleDecisionType::FOLLOW) {
      continue;
    }
    const auto it_lat_avoid_flag =
        obs_lat_avoid_flag_at_current_pose.find(obs_id);
    if (it_lat_avoid_flag == obs_lat_avoid_flag_at_current_pose.end()) {
      continue;
    }
    if (it_lat_avoid_flag->second != LatObstacleDecisionType::IGNORE &&
        it_lat_avoid_flag->second != LatObstacleDecisionType::FOLLOW) {
      continue;
    }
    const double ds = obs_sl.s_start - ego_frenet_boundary_.s_end;
    if (obstacle->obstacle()->is_static()) {
      if (ds < nearest_static_ds) {
        nearest_static_ds = ds;
        nearest_static = obstacle;
      }
    } else {
      if (obstacle->obstacle()->is_reverse()) {
        continue;
      }
      if (ds < nearest_dynamic_ds) {
        nearest_dynamic_ds = ds;
        nearest_dynamic = obstacle;
      }
    }
  }
  nearest_static_non_borrow_obstacle = nearest_static;
  nearest_dynamic_non_borrow_obstacle = nearest_dynamic;

  // AdjustRefinedPathsByNonBorrowObstacles(
  //     lane_borrow_output, obstacles, predict_box, lat_obstacle_decision,
  //     lon_min_s, lon_max_s, nearest_static_non_borrow_obstacle,
  //     nearest_dynamic_non_borrow_obstacle);
}

void LaneBorrowDeciderV3Utils::ClearInfo() {
  refined_paths_.clear();
  ref_path_curve_.x_vec.clear();
  ref_path_curve_.y_vec.clear();
  ref_path_curve_.s_vec.clear();
  ref_path_curve_.k_vec.clear();
  rule_lat_decision_hysteresis_map_.clear();
  need_virtual_obs_hysteresis_map_.clear();
  last_frame_paths_.clear();
  last_static_blocked_obj_id_vec_.clear();
  last_obs_static_map_.clear();
  static_blocked_obj_id_vec_.clear();
  last_back_origin_valid_ = false;
}

void LaneBorrowDeciderV3Utils::AddLaneBorrowVirtualObstacle(
    double obs_inner_l, double obs_start_s, double speed, bool is_reverse,
    int borrow_id, bool& is_hold_reset_path) {
  const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  is_hold_reset_path = false;
  if (static_blocked_obj_id_vec_.size() == 1) {
    const int target_obs_id = static_blocked_obj_id_vec_[0];
    const auto obs_it =
        std::find_if(obstacles.begin(), obstacles.end(),
                     [target_obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                       return o != nullptr && o->obstacle() != nullptr &&
                              o->obstacle()->id() == target_obs_id;
                     });
    if (obs_it == obstacles.end() || *obs_it == nullptr ||
        !(*obs_it)->b_frenet_valid()) {
      is_hold_reset_path = false;
    } else {
      const auto& obs_sl = (*obs_it)->frenet_obstacle_boundary();
      is_hold_reset_path = obs_sl.s_end <= ego_frenet_boundary_.s_end;
    }
  }
  // center line
  double distance_to_blocking = obs_start_s - ego_s_;
  double mini_gap = 5.0;
  double virtual_length = 1.0;  // 1+6 = 7
  double center_virtual_s = obs_start_s - mini_gap + virtual_length * 0.5;
  if (center_virtual_s - ego_s_ <
      vehicle_param.front_edge_to_rear_axle + virtual_length * 0.5) {
    double target_l = obs_inner_l > 0 ? std::max(obs_inner_l, ego_l_)
                                      : std::min(obs_inner_l, ego_l_);
    SetPullOverPath(obs_start_s, target_l);
    return;
  }
  SetPullOverPath(obs_start_s, obs_inner_l);
  // 判断是否需要添加虚拟障碍物，如果当前路径已经能够保证无恐慌的通过借道障碍物，即横向较远
  // 就不需要添加虚拟障碍物
  const double target_l = obs_inner_l;
  bool no_need_virtual_obs = true;
  std::unordered_map<int, bool> curr_need_virtual_obs_map;
  for (const int obs_id : static_blocked_obj_id_vec_) {
    double lat_buffer_to_obstacle = kStaticOtherMaxExtraLateralBuffer;
    const auto obs_it =
        std::find_if(obstacles.begin(), obstacles.end(),
                     [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                       return o != nullptr && o->obstacle() != nullptr &&
                              o->obstacle()->id() == obs_id;
                     });
    if (obs_it == obstacles.end() || !(*obs_it)->b_frenet_valid()) {
      continue;
    }
    if ((*obs_it)->obstacle()->is_VRU() ||
        (*obs_it)->obstacle()->is_oversize_vehicle() ||
        !((*obs_it)->obstacle()->is_static())) {
      lat_buffer_to_obstacle += 0.3;
    }
    const auto& obs_sl = (*obs_it)->frenet_obstacle_boundary();
    const double l_min = std::min(obs_sl.l_start, obs_sl.l_end);
    const double l_max = std::max(obs_sl.l_start, obs_sl.l_end);
    const double clearance =
        (target_l < l_min) ? (l_min - target_l)
                           : (target_l > l_max) ? (target_l - l_max) : 0.0;
    const double min_no_panic_clearance =
        vehicle_width_ * 0.5 + lat_buffer_to_obstacle;
    // 上一帧该障碍物是否被判定为需要虚拟障碍物
    const auto hist_it = need_virtual_obs_hysteresis_map_.find(obs_id);
    const bool last_need = (hist_it != need_virtual_obs_hysteresis_map_.end())
                               ? hist_it->second
                               : false;
    // 有滞回的阈值：上一帧 need=false 时，
    // clearance 需要比阈值小 kClearanceHysteresis 才切换为 need=true。
    const double effective_threshold =
        last_need ? min_no_panic_clearance
                  : (min_no_panic_clearance - kClearanceHysteresis);
    const bool curr_need = (clearance < effective_threshold);
    curr_need_virtual_obs_map[obs_id] = curr_need;
    if (curr_need) {
      no_need_virtual_obs = false;
    }
  }
  // 更新滞回状态：仅保留本帧仍在 static_blocked_obj_id_vec_ 中的条目，避免膨胀
  need_virtual_obs_hysteresis_map_ = std::move(curr_need_virtual_obs_map);
  if (no_need_virtual_obs) {
    return;
  }

  double virtual_l = obs_inner_l;
  // if (center_virtual_s < 60.0 || center_virtual_s - virtual_length* 0.5 -
  // vehicle_param.front_edge_to_rear_axle - 1.0 < ego_s_){
  //   return;
  // }
  Point2D end_sl_point(center_virtual_s, virtual_l);
  Point2D cart_point;
  frenet_coord->SLToXY(end_sl_point, cart_point);
  // ReferencePathPoint refpath_pt;
  // current_reference_path_ptr_->get_reference_point_by_lon(center_virtual_s,
  // refpath_pt); double virtual_obs_x = refpath_pt.path_point.x(); double
  // virtual_obs_y = refpath_pt.path_point.y();
  double virtual_obs_x = cart_point.x;
  double virtual_obs_y = cart_point.y;
  double path_heading = frenet_coord->GetPathCurveHeading(obs_start_s);
  double virtual_obs_theta = is_reverse ? (path_heading + M_PI) : path_heading;
  // construct trajectory for virtual obs
  std::vector<trajectory::Trajectory> trajectories;
  trajectories.reserve(1);
  trajectory::Trajectory trajectory;
  if (is_reverse) {
    const auto& agent = agent_mgr->GetAgent(borrow_id);
    const bool has_valid_lead_trajectory =
        (agent != nullptr) && !agent->trajectories_used_by_st_graph().empty() &&
        !agent->trajectories_used_by_st_graph()[0].empty();
    const int traj_len =
        has_valid_lead_trajectory
            ? static_cast<int>(agent->trajectories_used_by_st_graph()[0].size())
            : 25;
    for (int i = 0; i < traj_len; ++i) {
      double traj_s = center_virtual_s;
      if (has_valid_lead_trajectory) {
        const auto& lead_trajectory = agent->trajectories_used_by_st_graph()[0];
        Point2D cart_point_tmp(lead_trajectory[i].x(), lead_trajectory[i].y());
        Point2D sl_point;
        frenet_coord->XYToSL(cart_point_tmp, sl_point);
        traj_s = sl_point.x;
      }
      Point2D end_sl_point(traj_s, virtual_l);
      Point2D cart_point;
      frenet_coord->SLToXY(end_sl_point, cart_point);
      auto point = trajectory::TrajectoryPoint(cart_point.x, cart_point.y,
                                               virtual_obs_theta, speed, 0.0,
                                               i * 0.2, 0.0, 0.0, traj_s, 0.0);
      trajectory.emplace_back(point);
    }
  } else {
    for (int i = 0; i < 25; ++i) {
      double traj_s = center_virtual_s + speed * 0.2 * i;
      Point2D end_sl_point(traj_s, virtual_l);
      Point2D cart_point;
      frenet_coord->SLToXY(end_sl_point, cart_point);
      auto point = trajectory::TrajectoryPoint(cart_point.x, cart_point.y,
                                               virtual_obs_theta, speed, 0.0,
                                               i * 0.2, 0.0, 0.0, traj_s, 0.0);
      trajectory.emplace_back(point);
    }
  }
  trajectories.emplace_back(trajectory);

  // build virtual obs
  planning::agent::Agent virtual_agent;
  virtual_agent.set_agent_id(999999);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_lane_borrow_virtual_obs(true);
  virtual_agent.set_x(virtual_obs_x);  // 几何中心
  virtual_agent.set_y(virtual_obs_y);
  virtual_agent.set_length(virtual_length);
  virtual_agent.set_width(7.0);
  virtual_agent.set_fusion_source(1);
  bool is_static = (speed < 0.3) ? true : false;
  virtual_agent.set_is_static(is_static);
  virtual_agent.set_is_reverse(is_reverse);

  virtual_agent.set_speed(speed);
  virtual_agent.set_theta(virtual_obs_theta);
  virtual_agent.set_accel(0.0);
  virtual_agent.set_time_range({0.0, 5.0});
  virtual_agent.set_trajectories(trajectories);
  virtual_agent.set_trajectories_used_by_st_graph(std::move(trajectories));

  planning::planning_math::Box2d box(
      planning::planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
      virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());

  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);
  auto* agent_manager = session_->environmental_model()
                            .get_dynamic_world()
                            ->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);

  // reset path
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_x", virtual_agent.x())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_y", virtual_agent.y())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_theta",
                   virtual_agent.theta())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_id",
                   virtual_agent.agent_id())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_width",
                   virtual_agent.width())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_length",
                   virtual_agent.length())
}

void LaneBorrowDeciderV3Utils::SetPullOverPath(
    double end_s,
    double end_l) {  // re-set refined_paths_
  // 记录原有长度
  double path_s_end = ego_s_ + std::fmax(ego_v_ * 6.0,  // 6s
                                         55.0);
  const auto& current_frenet_coord = current_lane_ptr_->get_lane_frenet_coord();
  refined_paths_.clear();
  if (end_s - init_sl_point_.s < 0) {
    double path_s_length = path_s_end - init_sl_point_.s;
    double path_resolution = std::min(1.0, (path_s_length / 10));
    double current_s = 0.0;
    while (current_s + path_resolution * 0.5 < path_s_length) {
      const double l = end_l;
      const double dl = 0;
      const double ddl = 0;
      const double dddl = 0;
      const double theta = 0;  // 计算heading角度 不是全局的
      const double kappa = 0;  // 获取曲率
      const double dkappa = 0;
      const double ddkappa = 0.;
      const SLPoint sl_point(init_sl_point_.s + current_s, l);
      Point2D sl_cart(0, 0);
      current_frenet_coord->SLToXY(init_sl_point_.s + current_s, l, &sl_cart.x,
                                   &sl_cart.y);
      planning_math::PathPoint path_point{sl_cart.x,  sl_cart.y, sl_point.s,
                                          sl_point.l, theta,     kappa,
                                          dkappa,     ddkappa};
      refined_paths_.emplace_back(path_point);
      current_s += path_resolution;
    }
    return;
  }

  planning_math::QuinticPolynomialCurve1d curve(
      init_sl_point_.l, 0, 0, end_l, 0.0, 0.0, end_s - init_sl_point_.s);
  double path_s_length = end_s - init_sl_point_.s;
  double path_resolution = std::min(1.0, (path_s_length / 5));
  double current_s = 0.0;
  while (current_s + path_resolution * 0.5 < path_s_length) {
    // current_s += path_resolution;
    const double l = curve.Evaluate(0, current_s);
    const double dl = curve.Evaluate(1, current_s);
    const double ddl = curve.Evaluate(2, current_s);
    const double dddl = curve.Evaluate(3, current_s);
    const double theta = std::atan2(dl, 1);  // 计算heading角度 不是全局的
    const double kappa = curve.EvaluateKappa(current_s);  // 获取曲率
    const double denominator = std::pow(1 + dl * dl, 1.5);
    const double numerator1 = dddl;
    const double numerator2 = 3 * ddl * dl * ddl / std::pow(1 + dl * dl, 2.5);
    const double dkappa = numerator1 / denominator - numerator2;
    const double ddkappa = 0.;

    const SLPoint sl_point(init_sl_point_.s + current_s, l);

    Point2D sl_cart(0, 0);  // nonsene
    // donnot delete, discard for pybind
    current_frenet_coord->SLToXY(init_sl_point_.s + current_s, l, &sl_cart.x,
                                 &sl_cart.y);
    planning_math::PathPoint path_point{sl_cart.x,  sl_cart.y, sl_point.s,
                                        sl_point.l, theta,     kappa,
                                        dkappa,     ddkappa};
    refined_paths_.emplace_back(path_point);
    current_s += path_resolution;
  }
  // 已有 path 剩余长度为直线部分
  while (current_s + path_resolution * 0.5 < path_s_end - init_sl_point_.s) {
    const double l = end_l;
    const double dl = 0;
    const double ddl = 0;
    const double dddl = 0;
    const double theta = 0;  // 计算heading角度 不是全局的
    const double kappa = 0;  // 获取曲率
    const double dkappa = 0;
    const double ddkappa = 0.;

    const SLPoint sl_point(init_sl_point_.s + current_s, l);

    Point2D sl_cart(0, 0);  // nonsene
    // donnot delete, discard for pybind
    current_frenet_coord->SLToXY(init_sl_point_.s + current_s, l, &sl_cart.x,
                                 &sl_cart.y);
    planning_math::PathPoint path_point{sl_cart.x,  sl_cart.y, sl_point.s,
                                        sl_point.l, theta,     kappa,
                                        dkappa,     ddkappa};
    refined_paths_.emplace_back(path_point);
    current_s += path_resolution;
  }
}

}  // namespace lane_borrow_deciderv3_utils
}  // namespace planning