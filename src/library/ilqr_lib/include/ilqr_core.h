#ifndef __ILQR_CORE_H__
#define __ILQR_CORE_H__

#include <chrono>
#include <utility>
#include <vector>

#include "ilqr_define.h"
#include "ilqr_model.h"
namespace ilqr_solver {
class iLqr {
 public:
  // ilqr solver condition of result
  enum iLqrSolveCondition {
    INIT,                     // solver init
    NORMAL_TERMINATE,         // normal terminate, dcost < cost_tol
    CONST_CONTROL_TERMINATE,  // small control u update, du_norm < du_tol
    MAX_ITER_TERMINATE,       // max iteration
    LINESEARCH_TERMINATE,     // linesearch terminate, lambda is too big
    INIT_TERMINATE,           // one step iteration, init is the minimum
    BACKWARD_PASS_FAIL,       // backward pass failed, non-positive definite Quu
    NON_POSITIVE_EXPECT,      // non-positive expect, should not happen
    FAULT_INPUT_SIZE,         // input size is error
  };

  // solver info for each iteration
  struct IterationInfo {
    bool linesearch_success = false;
    size_t linesearch_count = 0;
    size_t backward_pass_count = 0;
    double lambda = 0.0;
    double cost = 0.0;
    double dcost = 0.0;
    double expect = 0.0;
    double du_norm = 0.0;
    StateVec x_vec;
    ControlVec u_vec;
  };

  // solver info for all iterations
  struct iLqrSolverInfo {
    uint8_t solver_condition = NORMAL_TERMINATE;
    size_t cost_size = 0;
    size_t iter_count = 0;
    double init_cost = 0.0;
    std::vector<IterationInfo> iteration_info_vec;

    // each cost of cost term for all iterations and all horizons
    std::vector<ILqrCostMap> cost_map_vec;

    // each cost of cost term for every iteration
    std::vector<ILqrCostVec> cost_iter_vec;
  };

  struct TimeInfo {
    void Reset() {
      t_init_guess_ms = 0.0;
      t_compute_deriv_ms = 0.0;
      t_backward_pass_ms = 0.0;
      t_forward_pass_ms = 0.0;
      t_one_step_ms = 0.0;
    }

    double GetElapsed(std::chrono::time_point<std::chrono::system_clock> &start,
                      const bool is_overlay) {
      auto now = std::chrono::system_clock::now();
      auto elapsed =
          std::chrono::duration_cast<std::chrono::nanoseconds>(now - start)
              .count();

      if (is_overlay) {
        start = now;
      }

      return elapsed * 1e-6;
    }

    void UpdateAllStart() { all_start = std::chrono::system_clock::now(); }

    void UpdateStart() { start = std::chrono::system_clock::now(); }

    std::chrono::time_point<std::chrono::system_clock> all_start;
    std::chrono::time_point<std::chrono::system_clock> start;
    double t_init_guess_ms = 0.0;
    double t_compute_deriv_ms = 0.0;
    double t_backward_pass_ms = 0.0;
    double t_forward_pass_ms = 0.0;
    double t_one_step_ms = 0.0;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  iLqr() = default;
  ~iLqr() = default;
  void Init(const std::shared_ptr<iLqrModel> ilqr_model,
            const iLqrSolverConfig &ilqr_sovler_config);

  void Init(const std::shared_ptr<iLqrModel> ilqr_model);

  // usually do not need init cost
  void Solve(const State &x0);
  void Solve(const State &x0, const ControlVec &u_vec);
  // simulation dynamics with given init state x0 and input u_vec
  void Simulation(const State &x0, const ControlVec &u_vec);

  // void InputFeasibilityCheck(const StateVec &xk, const
  // ControlVec &uk);

  void SetWarmStart(bool flag) { solver_config_ptr_->warm_start_enable = flag; }

  void SetMaxIter(size_t max_iter);

  void GetOutput(StateVec &xk, ControlVec &uk) const {
    uk = uk_vec_;
    xk = xk_vec_;
  }

  const StateVec *GetStateResultPtr() const { return &xk_vec_; }

  const ControlVec *GetControlResultPtr() const { return &uk_vec_; }

  const iLqrSolverInfo *GetSolverInfoPtr() { return &solver_info_; }

  void PrintSolverInfo();
  void PrintCostInfo();
  void PrintTimeInfo();

  void UpdateDynamicsDerivatives();

  void AddCost(std::shared_ptr<BaseCostTerm> cost_term);

  void SetSolverConfig(const iLqrSolverConfig &ilqr_sovler_config) {
    *solver_config_ptr_ = ilqr_sovler_config;
  }

  void SetCostConfig(const std::vector<IlqrCostConfig> &cost_config) {
    ilqr_model_ptr_->SetCostConfig(cost_config);
  }

  void InitSolverConfig();

  std::shared_ptr<iLqrModel> GetiLqrModelPtr() const { return ilqr_model_ptr_; }

  std::shared_ptr<iLqrSolverConfig> GetSolverConfigPtr() {
    return solver_config_ptr_;
  }

  // init cost map
  void InitAdvancedInfo();

  // update cost_vec
  void UpdateAdvancedInfo(size_t iter);

  // reset solver
  void Reset();

 protected:
  virtual bool ForwardPass(double &new_cost, double &expected,
                           const size_t &iter);

  bool BackwardPass();
  bool iLqrIteration();

  void IncreaseLambda();
  void DecreaseLambda();
  bool PSDCheck(Eigen::MatrixXd &Q);

  StateVec xk_vec_;
  ControlVec uk_vec_;
  StateVec xk_new_vec_;
  ControlVec uk_new_vec_;

  double cost_ = 0.0;

  LxMTVec lx_vec_;    // n * (CONTROL_HORIZON)
  LuMTVec lu_vec_;    // m * (CONTROL_HORIZON)
  LxxMTVec lxx_vec_;  // n * n * (CONTROL_HORIZON)
  LxuMTVec lxu_vec_;  // n * m * (CONTROL_HORIZON)
  LuuMTVec luu_vec_;  // m * m * (CONTROL_HORIZON)

  FxMTVec fx_vec_;  // n * n * (CONTROL_HORIZON)
  FuMTVec fu_vec_;  // n * m * (CONTROL_HORIZON)

  std::array<double, 2> dV_{};  // 2 * 1
  kMTVec k_vec_;                // m * (CONTROL_HORIZON + 1)
  KMTVec K_vec_;                // m * n * (CONTROL_HORIZON + 1)

  Eigen::VectorXd Qu_, Qx_, k_i_;
  Eigen::MatrixXd Quu_, Qxx_, Qux_, Quuf_, K_i_;
  Eigen::MatrixXd Quu_eye_;

  double lambda_ = 1.0;
  double lambda_gain_ = 1.0;

  std::vector<double> du_norm_vec_;

  // model
  std::shared_ptr<iLqrModel> ilqr_model_ptr_;

  // solver config
  std::shared_ptr<iLqrSolverConfig> solver_config_ptr_;

  // solver info
  iLqrSolverInfo solver_info_;

  // time debug info
  TimeInfo time_info_;
};
}  // namespace ilqr_solver

#endif
