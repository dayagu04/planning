#pragma once
#include "behavior_planners/lane_change_decider/lane_change_joint_decision_generator/src/joint_decision_planning_model.h"
#include "ilqr_cost.h"
#include "utils/kd_path.h"
#include "vec2d.h"
namespace pnc {
namespace lane_change_joint_decision {

enum iLqrCostconfigId {
  EGO_REF_X,
  EGO_REF_Y,
  EGO_REF_THETA,
  EGO_REF_DELTA,
  EGO_REF_VEL,
  EGO_REF_ACC,
  W_EGO_REF_X,
  W_EGO_REF_Y,
  W_EGO_REF_THETA,
  W_EGO_REF_DELTA,
  W_EGO_REF_VEL,
  W_EGO_REF_ACC,
  W_OBS_REF_X,
  W_OBS_REF_Y,
  W_OBS_REF_THETA,
  W_OBS_REF_DELTA,
  W_OBS_REF_VEL,
  W_OBS_REF_ACC,
  CURV_FACTOR,
  EGO_LENGTH,
  EGO_WIDTH,
  EGO_FRONT_EDGE_TO_REAR_AXLE,
  EGO_WHEEL_BASE,
  OBS_NUM,
  THREE_DISC_SAFE_DIST,
  W_THREE_DISC_SAFE_DIST_WEIGHT,
  ROAD_BOUNDARY_SAFE_DIST,
  W_ROAD_BOUNDARY,
  W_EGO_ACC,
  W_EGO_JERK,
  W_EGO_OMEGA,
  W_EGO_DELTA,
  W_OBS_JERK,
  W_OBS_OMEGA,
  W_EGO_ACC_BOUND,
  EGO_ACC_MAX,
  EGO_ACC_MIN,
  W_EGO_JERK_BOUND,
  EGO_JERK_MAX,
  EGO_JERK_MIN,
  W_HARD_HALFPLANE,
  HARD_HALFPLANE_DIST,
  HALFPLANE_COST_ALLOCATION_RATIO,
  W_SOFT_HALFPLANE,
  SOFT_HALFPLANE_S0,
  SOFT_HALFPLANE_TAU,
  SOFT_HALFPLANE_COST_ALLOCATION_RATIO,
  OBS_CONFIG_START,
};

// int config_size = OBS_CONFIG_START +
//                   obs_num +                  // 曲率因子
//                   obs_num +                  // 长度
//                   obs_num +                  // 宽度
//                   obs_num * OBS_STATE_SIZE + // 参考状态
//                   obs_num +                  // ego对obs的纵向标签
inline int GetObsCurvFactorIdx(int obs_idx) {
  return OBS_CONFIG_START + obs_idx;
}

inline int GetObsLengthIdx(int obs_idx, int obs_num) {
  return OBS_CONFIG_START + obs_num + obs_idx;
}

inline int GetObsWidthIdx(int obs_idx, int obs_num) {
  return OBS_CONFIG_START + 2 * obs_num + obs_idx;
}

inline int GetObsRefStateIdx(int obs_idx, int obs_num, int state_id) {
  return OBS_CONFIG_START + 3 * obs_num + obs_idx * OBS_STATE_SIZE + state_id;
}

inline int GetObsLongitudinalLabelIdx(int obs_idx, int obs_num) {
  return OBS_CONFIG_START + 3 * obs_num + obs_num * OBS_STATE_SIZE + obs_idx;
}

enum iLqrCostId {
  EGO_REFERENCE_COST,
  OBS_REFERENCE_COST,
  EGO_THREE_DISC_SAFE_COST,
  EGO_ROAD_BOUNDARY_COST,
  EGO_ACC_COST,
  EGO_JERK_COST,
  EGO_OMEGA_COST,
  EGO_DELTA_COST,
  OBS_JERK_COST,
  OBS_OMEGA_COST,
  EGO_ACC_BOUND_COST,
  EGO_JERK_BOUND_COST,
  HARD_HALFPLANE_COST,
  SOFT_HALFPLANE_COST,
  COST_SIZE,
};
enum EgoStateId {
  EGO_X = 0,
  EGO_Y = 1,
  EGO_THETA = 2,
  EGO_DELTA = 3,
  EGO_VEL = 4,
  EGO_ACC = 5,
};

enum ObsStateId {
  OBS_X = 0,
  OBS_Y = 1,
  OBS_THETA = 2,
  OBS_DELTA = 3,
  OBS_VEL = 4,
  OBS_ACC = 5,
};

enum ObsControlId {
  OBS_OMEGA = 0,
  OBS_JERK = 1,
};

enum EgoControlId {
  EGO_OMEGA = 0,
  EGO_JERK = 1,
};

inline int GetObsStateIdx(int obs_idx, int state_offset) {
  return EGO_STATE_SIZE + obs_idx * OBS_STATE_SIZE + state_offset;
}

inline int GetObsControlIdx(int obs_idx, int control_offset) {
  return EGO_CONTROL_SIZE + obs_idx * OBS_CONTROL_SIZE + control_offset;
}
class EgoReferenceCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoReferenceCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_REFERENCE_COST; }
};
class ObsReferenceCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  ObsReferenceCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return OBS_REFERENCE_COST; }
};
class EgoThreeDiscSafeCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoThreeDiscSafeCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_THREE_DISC_SAFE_COST; }

 private:
  struct ThreeDiscResult {
    double min_dist;
    double min_obs_x;
    double min_obs_y;
    double min_obs_radius;
    int closest_obs_index;
    double closest_ego_disc_x;
    double closest_ego_disc_y;
    double closest_obs_disc_x;
    double closest_obs_disc_y;
    int closest_ego_disc_index;
  };
  ThreeDiscResult CalculateThreeDiscDistances(const ilqr_solver::State &x);
};
class EgoRoadBoundaryCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoRoadBoundaryCostTerm() = default;
  EgoRoadBoundaryCostTerm(
      std::shared_ptr<planning::planning_math::KDPath> road_left,
      std::shared_ptr<planning::planning_math::KDPath> road_right)
      : road_left_boundary_path_(road_left),
        road_right_boundary_path_(road_right) {}
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &lxu,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_ROAD_BOUNDARY_COST; }
  void UpdateBoundaryPaths(
      std::shared_ptr<planning::planning_math::KDPath> road_left,
      std::shared_ptr<planning::planning_math::KDPath> road_right) {
    road_left_boundary_path_ = road_left;
    road_right_boundary_path_ = road_right;
  }

 private:
  std::shared_ptr<planning::planning_math::KDPath> road_left_boundary_path_;
  std::shared_ptr<planning::planning_math::KDPath> road_right_boundary_path_;
  struct BoundaryDistResult {
    double min_dist_to_left;
    double min_dist_to_right;
    bool left_front_closer;
    bool right_front_closer;
    planning::planning_math::Vec2d front_center;
    planning::planning_math::Vec2d rear_center;
    bool ego_left_valid;
    planning::planning_math::Vec2d ego_left_unit_vector;
    bool ego_right_valid;
    planning::planning_math::Vec2d ego_right_unit_vector;
  };
  BoundaryDistResult dist_result_;
  void CalculateBoundaryDistancesInfo(const ilqr_solver::State &x);
};

class EgoAccCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoAccCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_ACC_COST; }
};

class EgoJerkCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoJerkCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_JERK_COST; }
};

class EgoOmegaCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoOmegaCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_OMEGA_COST; }
};

class EgoDeltaCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoDeltaCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_DELTA_COST; }
};

class ObsJerkCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  ObsJerkCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return OBS_JERK_COST; }
};

class ObsOmegaCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  ObsOmegaCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return OBS_OMEGA_COST; }
};

class EgoAccBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoAccBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return EGO_ACC_BOUND_COST; }
};

class EgoJerkBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EgoJerkBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return W_EGO_JERK_BOUND; }
};

class SoftHalfplaneCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  SoftHalfplaneCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &lxu,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return SOFT_HALFPLANE_COST; }

 private:
  struct SoftHalfplaneResult {
    int obs_index;     // 障碍物索引
    int label_type;    // 标签类型：1=OVERTAKE, 2=YIELD, 3=EGO_OVERTAKE, 4=HALF_YIELD
    double s_current;  // 当前纵向距离
    double s_target;   // 目标安全距离
    double normal_x;   // 参考朝向法向量x
    double normal_y;   // 参考朝向法向量y
  };
  std::vector<SoftHalfplaneResult> CalculateSoftHalfplane(
      const ilqr_solver::State &x);
};

class HardHalfplaneCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  HardHalfplaneCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &lxu,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return HARD_HALFPLANE_COST; }

 private:
  struct HardHalfplaneResult {
    int obs_index;      // 障碍物索引
    int label_type;     // 标签类型：1=OVERTAKE, 2=YIELD
    double plane_dist;  // 半平面距离
    double normal_x;    // 参考朝向法向量x
    double normal_y;    // 参考朝向法向量y
  };
  std::vector<HardHalfplaneResult> CalculateObsHardHalfplane(
      const ilqr_solver::State &x);
};

}  // namespace lane_change_joint_decision
}  // namespace pnc
