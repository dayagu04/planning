#include <cyber/binary.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "ad_common/math/linear_interpolation.h"
#include "ad_common/math/math_utils.h"
#include "apa_param_config.h"
#include "apa_plan_interface.h"
#include "collision_detection/collision_detection.h"
#include "common.h"
#include "future_path_decider.h"
#include "hybrid_a_star.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_interface.h"
#include "log_glog.h"
#include "math_lib.h"
#include "narrow_space_scenario.h"
#include "park_speed_limit_decider.h"
#include "parking_stop_decider.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "pwj_qp_speed_optimizer/piecewise_jerk_qp_speed_optimizer.h"
#include "reeds_shepp.h"
#include "rs_path_interpolate.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/hybrid_astar_lib/hybrid_a_star.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_thread.h"
#include "src/library/occupancy_grid_map/euler_distance_transform.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "src/library/reeds_shepp/reeds_shepp_interface.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "src/modules/apa_function/parking_task/optimizers/sv_dp_optimizer/dp_speed_optimizer.h"
#include "transform2d.h"
#include "virtual_wall_decider.h"
#include "jerk_limited_traj_optimizer/jerk_limited_traj_optimizer.h"

namespace py = pybind11;
using namespace planning::apa_planner;
using namespace planning;
using namespace pnc::geometry_lib;

// test item:
// pwj qp speed planning
// sv dynamic programming speed planning
// jerk limited trajectory speed planning
// lattice speed planning;
static std::shared_ptr<planning::HybridAStarInterface> hybrid_astar_interface_;
static planning::apa_planner::ApaPlanInterface *parking_interface = nullptr;
std::vector<Eigen::Vector3d> global_path_;
SpeedData dp_speed_profile_;
SpeedData qp_speed_profile_;
SpeedData jlt_speed_profile_;
double delta_time = 0.1;

void Init() {
  FilePath::SetName("dp_speed_optimizer");
  InitGlog(FilePath::GetName().c_str());

  ILOG_INFO << "log init finish";

  // parking
  parking_interface = new planning::apa_planner::ApaPlanInterface();

  parking_interface->Init();

  std::shared_ptr<apa_planner::ParkingScenario> planner =
      parking_interface->GetPlannerByType(
          ParkingScenarioType::SCENARIO_NARROW_SPACE);

  std::shared_ptr<apa_planner::NarrowSpaceScenario> hybrid_astar_park_ =
      std::dynamic_pointer_cast<apa_planner::NarrowSpaceScenario>(planner);

  HybridAStarThreadSolver *thread = hybrid_astar_park_->GetThread();
  hybrid_astar_interface_ = thread->GetHybridAStarInterface();

  if (hybrid_astar_interface_ == nullptr) {
    ILOG_INFO << "hybrid_astar_interface_ is null";
  }

  return;
}

void StopPybind() {
  // StopGlog();
  return;
}

void UpdatePathInfo(std::vector<pnc::geometry_lib::PathPoint> &path,
                    const double radius) {
  if (path.size() < 1) {
    return;
  }

  double accumulated_s = 0.0;
  auto last_x = path.front().pos.x();
  auto last_y = path.front().pos.y();
  double x_diff;
  double y_diff;
  double kappa;
  if (std::fabs(radius) < 1e-1) {
    kappa = 100.0;
  } else {
    kappa = 1 / radius;
  }

  for (size_t i = 0; i < path.size(); ++i) {
    x_diff = path[i].pos.x() - last_x;
    y_diff = path[i].pos.y() - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    path[i].s = accumulated_s;
    path[i].kappa = kappa;

    last_x = path[i].pos.x();
    last_y = path[i].pos.y();

    path[i].dist_to_obs = 10.0;

    // ILOG_INFO << "i = " << i << ",s = " << accumulated_s
    //           << ", kappa = " << kappa;
  }

  return;
}

void UpdatePathObsDistance(std::vector<pnc::geometry_lib::PathPoint> &path,
                           double obs_s, double dist_to_obs) {
  if (path.size() < 1) {
    return;
  }
  for (size_t i = 0; i < path.size(); ++i) {

    path[i].dist_to_obs = 10.0;
    if (std::fabs(path[i].s - obs_s) < 0.15) {
      path[i].dist_to_obs = dist_to_obs;
    }

    // ILOG_INFO << "i = " << i << ",s = " << accumulated_s
    //           << ", kappa = " << kappa;
  }

  return;
}

std::vector<Eigen::Vector3d> Update(Eigen::Vector3d ego_pose,
                                    double path_length, double path_radius,
                                    double ego_v, double ego_acc, double obs_s,
                                    double dist_to_obs, double max_cruise_speed,
                                    double jlt_acc_lower) {
  // generate path
  global_path_.clear();

  apa_param.SetPram().speed_config.default_cruise_speed = max_cruise_speed;

  const ApaParameters &param = apa_param.GetParam();

  Pose2D start_pose(ego_pose[0], ego_pose[1], ego_pose[2]);
  std::vector<Pose2D> path;

  FuturePathDecider future_path_decider;

  double radius;
  if (path_radius > 0) {
    radius = std::max(param.min_turn_radius, path_radius);
  } else {
    radius = std::min(-param.min_turn_radius, path_radius);
  }
  future_path_decider.GetPathByRadius(&start_pose, path_length, radius, true,
                                      &path);

  std::vector<pnc::geometry_lib::PathPoint> path2;
  for (auto &point : path) {
    global_path_.push_back(Eigen::Vector3d(point.x, point.y, point.theta));
    path2.push_back(pnc::geometry_lib::PathPoint(
        Eigen::Vector2d(point.x, point.y), point.theta));
  }
  UpdatePathInfo(path2, radius);

  // update stop decision
  std::shared_ptr<ApaObstacleManager> obstacles =
      std::make_shared<ApaObstacleManager>();
  obstacles->Reset();

  std::shared_ptr<apa_planner::ApaMeasureDataManager> localization_ptr =
      std::make_shared<apa_planner::ApaMeasureDataManager>();
  localization_ptr->SetPose(Eigen::Vector2d(start_pose.x, start_pose.y),
                       start_pose.theta);

  std::shared_ptr<apa_planner::ApaPredictPathManager> predict_path =
      std::make_shared<ApaPredictPathManager>();

  std::shared_ptr<apa_planner::CollisionDetectorInterface>
      col_det_interface_ptr =
          std::make_shared<apa_planner::CollisionDetectorInterface>(
              obstacles, localization_ptr, predict_path);

  // collision check
  SpeedDecisions speed_decisions;
  ParkingStopDecider stop_decider =
      ParkingStopDecider(col_det_interface_ptr, localization_ptr);
  stop_decider.Process(100, path2, &speed_decisions);

  // use boken obs dist, need retire
  UpdatePathObsDistance(path2, obs_s, dist_to_obs);
  stop_decider.AddDebugInfo(path2);

  // update speed limit decision
  ParkSpeedLimitDecider speed_limit_decider = ParkSpeedLimitDecider();
  speed_limit_decider.Process(path2, &speed_decisions);
  const SpeedLimitProfile &speed_limit =
      speed_limit_decider.GetSpeedLimitProfile();

  // generate dp speed
  DpSpeedOptimizer dp_speed_optimizer;
  dp_speed_optimizer.Init();
  dp_speed_optimizer.Excute(path2, start_pose, ego_v, ego_acc, &speed_decisions,
                         &speed_limit);

  dp_speed_profile_ = dp_speed_optimizer.SpeedProfile();

  PiecewiseJerkSpeedQPOptimizer qp_speed_optimizer;
  SVPoint init_point;
  init_point.s = 0.0;
  init_point.v = ego_v;
  init_point.acc = ego_acc;
  init_point.t = 0.0;

  qp_speed_optimizer.Execute(init_point, &speed_limit, dp_speed_profile_);
  qp_speed_profile_ = qp_speed_optimizer.GetSpeedData();

  JerkLimitedTrajOptimizer jlt_optimizer;
  jlt_optimizer.Execute(init_point, path_length);
  jlt_speed_profile_ = jlt_optimizer.GetSpeedData();

  return global_path_;
}

std::vector<Eigen::VectorXd> GetDpSpeedConstraints() {
  std::vector<Eigen::VectorXd> speed_debug_data;
  Eigen::VectorXd v(7);

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_debug_data.emplace_back(v);
    return speed_debug_data;
  }

  int size = 0;
  if (speed_debug->has_dp_speed_constraint()) {
    size = speed_debug->dp_speed_constraint().s_size();
  }

  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->dp_speed_constraint().s(i);

    if (i < speed_debug->dp_speed_constraint().obs_dist_size()) {
      v[1] = speed_debug->dp_speed_constraint().obs_dist(i);
    }

    if (i < speed_debug->dp_speed_constraint().v_upper_bound_size()) {
      v[2] = speed_debug->dp_speed_constraint().v_upper_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().a_upper_bound_size()) {
      v[3] = speed_debug->dp_speed_constraint().a_upper_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().a_lower_bound_size()) {
      v[4] = speed_debug->dp_speed_constraint().a_lower_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().jerk_upper_bound_size()) {
      v[5] = speed_debug->dp_speed_constraint().jerk_upper_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().jerk_lower_bound_size()) {
      v[6] = speed_debug->dp_speed_constraint().jerk_lower_bound(i);
    }

    speed_debug_data.emplace_back(v);
  }

  if (speed_debug_data.size() == 0) {
    speed_debug_data.emplace_back(v);
  }

  return speed_debug_data;
}

std::vector<Eigen::Vector2d> GetQPSpeedConstraints() {
  std::vector<Eigen::Vector2d> speed_debug_data;
  Eigen::Vector2d v;

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_debug_data.emplace_back(v);
    return speed_debug_data;
  }

  int size = 0;
  if (speed_debug->has_qp_speed_constraint()) {
    size = speed_debug->qp_speed_constraint().s_size();
  }

  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->qp_speed_constraint().s(i);
    if (i < speed_debug->qp_speed_constraint().v_upper_bound_size()) {
      v[1] = speed_debug->qp_speed_constraint().v_upper_bound(i);
    }

    speed_debug_data.emplace_back(v);
  }

  if (speed_debug_data.size() == 0) {
    speed_debug_data.emplace_back(v);
  }

  return speed_debug_data;
}

const double GetRefCruiseSpeed() {
  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }
  if (speed_debug == nullptr) {
    return 0.0;
  }

  double speed = 0.0;
  if (speed_debug->has_ref_cruise_speed()) {
    speed = speed_debug->ref_cruise_speed();
  }

  return speed;
}

std::vector<Eigen::VectorXd> GetDPSpeedOptimizationData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  for (int i = 0; i < dp_speed_profile_.size(); i++) {
    v[0] = dp_speed_profile_[i].s;
    v[1] = dp_speed_profile_[i].t;
    v[2] = dp_speed_profile_[i].v;
    v[3] = dp_speed_profile_[i].a;
    v[4] = dp_speed_profile_[i].da;

    speed_profile.push_back(v);
  }

  return speed_profile;
}

std::vector<Eigen::VectorXd> GetQPSpeedOptimizationData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  for (int i = 0; i < qp_speed_profile_.size(); i++) {
    v[0] = qp_speed_profile_[i].s;
    v[1] = qp_speed_profile_[i].t;
    v[2] = qp_speed_profile_[i].v;
    v[3] = qp_speed_profile_[i].a;
    v[4] = qp_speed_profile_[i].da;

    speed_profile.push_back(v);
  }

  return speed_profile;
}

std::vector<Eigen::VectorXd> GetJLTSpeedData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  for (int i = 0; i < jlt_speed_profile_.size(); i++) {
    v[0] = jlt_speed_profile_[i].s;
    v[1] = jlt_speed_profile_[i].t;
    v[2] = jlt_speed_profile_[i].v;
    v[3] = jlt_speed_profile_[i].a;
    v[4] = jlt_speed_profile_[i].da;

    speed_profile.push_back(v);
  }

  return speed_profile;
}

PYBIND11_MODULE(dp_speed_optimizer_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetRefCruiseSpeed", &GetRefCruiseSpeed)
      .def("GetDPSpeedOptimizationData", &GetDPSpeedOptimizationData)
      .def("GetQPSpeedOptimizationData", &GetQPSpeedOptimizationData)
      .def("GetJLTSpeedData", &GetJLTSpeedData)
      .def("GetQPSpeedConstraints", &GetQPSpeedConstraints)
      .def("GetDpSpeedConstraints", &GetDpSpeedConstraints);
}