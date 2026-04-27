#pragma once

#include "ilqr_cost.h"
#include "../problem_solver/solver_define.h"
#include "euler_distance_transform.h"

namespace pnc {
namespace lateral_planning {

class EdtDistanceCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  EdtDistanceCostTerm() = default;

  EdtDistanceCostTerm(planning::Transform2f* ego_base_ptr, planning::EulerDistanceTransform* edt_ptr);

  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;

  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

  void SetEgoBase(planning::Transform2f* ego_base) {
    ego_base_ptr_ = ego_base;
  }

  void SetEulerDistanceTransform(planning::EulerDistanceTransform* edt_ptr) {
    edt_ptr_ = edt_ptr;
  }

 private:
  uint8_t cost_id_ = EDT_DISTANCE_COST;

  planning::Transform2f* ego_base_ptr_ = nullptr;
  planning::EulerDistanceTransform* edt_ptr_ = nullptr;
};

}  // namespace lateral_planning
}  // namespace pnc