#include "apa_lon_util.h"
#include "pose2d.h"
#include "debug_info_log.h"
#include "transform2d.h"

namespace planning {

std::vector<std::vector<Eigen::Vector2d>> GetODTraj() {
  std::vector<std::vector<Eigen::Vector2d>> trajs;
  trajs.clear();

  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    return trajs;
  }

  if (!speed_debug->has_predict_traj_set()) {
    return trajs;
  }

  int size = speed_debug->predict_traj_set().trajs_size();
  if (size <= 0) {
    return trajs;
  }

  std::vector<Eigen::Vector2d> traj;
  for (int i = 0; i < size; i++) {
    const common::ParkPredictTraj &proto_traj =
        speed_debug->mutable_predict_traj_set()->trajs(i);

    traj.clear();
    for (int j = 0; j < proto_traj.point_size(); j++) {
      traj.emplace_back(
          Eigen::Vector2d(proto_traj.point(j).x(), proto_traj.point(j).y()));
    }

    trajs.emplace_back(traj);
  }

  return trajs;
}

const bool IsODVeh(const iflyauto::ObjectType type) {
  if (type >= iflyauto::OBJECT_TYPE_PICKUP &&
      type <= iflyauto::OBJECT_TYPE_ENGINEERING_VEHICLE) {
    return true;
  }
  if (type >= iflyauto::OBJECT_TYPE_COUPE &&
      type <= iflyauto::OBJECT_TYPE_TRICYCLE) {
    return true;
  }
  if (type >= iflyauto::OBJECT_TYPE_CYCLE_RIDING &&
      type <= iflyauto::OBJECT_TYPE_TRICYCLE_RIDING) {
    return true;
  }

  return false;
}

const bool IsODSpecificationer(const iflyauto::ObjectType type) {
  if (type == iflyauto::OBJECT_TYPE_SPECIFICATIONER) {
    return true;
  }

  return false;
}

const bool IsODLivingThings(const iflyauto::ObjectType type) {
  if (type == iflyauto::OBJECT_TYPE_PEDESTRIAN) {
    return true;
  }
  if (type == iflyauto::OBJECT_TYPE_ANIMAL) {
    return true;
  }
  if (type == iflyauto::OBJECT_TYPE_ADULT) {
    return true;
  }
  if (type == iflyauto::OBJECT_TYPE_CHILD) {
    return true;
  }
  if (type == iflyauto::OBJECT_TYPE_TRAFFIC_POLICE) {
    return true;
  }

  return false;
}

const bool IsDynamicLivingThings(
                                 const iflyauto::ObjectType type) {
  if (IsODLivingThings(type)) {
    return true;
  }

  return false;
}

const bool IsDynamicODVeh(const double v,
                          const iflyauto::ObjectType type) {
  if (IsODVeh(type)) {
    return true;
  }

  return false;
}

std::vector<Eigen::VectorXd> TransformDpSpeedConstraints() {
  std::vector<Eigen::VectorXd> speed_debug_data;
  Eigen::VectorXd v(7);

  auto &debug = planning::DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
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

std::vector<Eigen::Vector2d> TransformQPSpeedConstraints() {
  std::vector<Eigen::Vector2d> speed_debug_data;
  Eigen::Vector2d v;
  v.setZero();

  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
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

const double GetDebugRefCruiseSpeed() {
  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
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

std::vector<Eigen::VectorXd> TransformDPSpeedOptimizationData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_profile.emplace_back(v);
    return speed_profile;
  }

  int size = speed_debug->dp_profile_size();
  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->dp_profile(i).s();
    v[1] = speed_debug->dp_profile(i).t();
    v[2] = speed_debug->dp_profile(i).vel();
    v[3] = speed_debug->dp_profile(i).acc();
    v[4] = speed_debug->dp_profile(i).jerk();

    speed_profile.push_back(v);
  }

  return speed_profile;
}

std::vector<Eigen::VectorXd> TransformQPSpeedOptimizationData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_profile.emplace_back(v);
    return speed_profile;
  }

  int size = speed_debug->qp_profile_size();
  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->qp_profile(i).s();
    v[1] = speed_debug->qp_profile(i).t();
    v[2] = speed_debug->qp_profile(i).vel();
    v[3] = speed_debug->qp_profile(i).acc();
    v[4] = speed_debug->qp_profile(i).jerk();

    speed_profile.push_back(v);
  }

  return speed_profile;
}

std::vector<Eigen::VectorXd> TransformStopSigns() {
  std::vector<Eigen::VectorXd> stop_signs;
  stop_signs.clear();

  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    return stop_signs;
  }

  int size = speed_debug->stop_signs_size();
  if (size <= 0) {
    return stop_signs;
  }

  Eigen::VectorXd stop_sign(4);
  Transform2d tf;
  Position2D local;
  Position2D global;

  for (int i = 0; i < size; i++) {
    tf.SetBasePose(Pose2D(speed_debug->stop_signs(i).stop_pose().x(),
                          speed_debug->stop_signs(i).stop_pose().y(),
                          speed_debug->stop_signs(i).stop_pose().theta()));

    local.x = 0;
    local.y = -1.5;
    tf.ULFLocalPointToGlobal(&global, local);
    stop_sign[0] = global.x;
    stop_sign[1] = global.y;

    local.x = 0;
    local.y = 1.5;
    tf.ULFLocalPointToGlobal(&global, local);
    stop_sign[2] = global.x;
    stop_sign[3] = global.y;

    stop_signs.push_back(stop_sign);
  }

  return stop_signs;
}

std::vector<Eigen::VectorXd> TransformJLTSpeedData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd point(5);

  auto &debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug->has_apa_speed_debug()) {
    speed_debug = debug->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_profile.emplace_back(point);
    return speed_profile;
  }

  int size = speed_debug->jlt_profile_size();
  for (int i = 0; i < size; i++) {
    point[0] = speed_debug->jlt_profile(i).s();
    point[1] = speed_debug->jlt_profile(i).t();
    point[2] = speed_debug->jlt_profile(i).vel();
    point[3] = speed_debug->jlt_profile(i).acc();
    point[4] = speed_debug->jlt_profile(i).jerk();

    speed_profile.push_back(point);
  }

  return speed_profile;
}

}  // namespace planning