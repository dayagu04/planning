#ifndef __AL_ILQR_CORE_H__
#define __AL_ILQR_CORE_H__

#include <chrono>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "al_ilqr_define.h"
#include "al_ilqr_model.h"
namespace al_ilqr_solver {

// Augmented Lagrangian iLQR (AL-iLQR) solver.
//
// Extends standard iLQR with an outer Augmented Lagrangian loop
// to handle inequality/equality constraints on states and controls.
//
// Algorithm overview (Howell et al. 2019, ALTRO):
//   outer loop: update dual variables mu, penalty weights rho
//     inner loop: standard iLQR with augmented cost
//       L_AL = J(x,u) + sum_k [ mu_k * c_k + (rho_k/2) * c_k^2 ]
//       for active constraints where c_k + mu_k/rho_k > 0
//     check KKT: constraint_violation < tol AND dcost < tol
//     update: mu_new = max(0, mu + rho * c),  rho_new = phi * rho
class AliLqr {
 public:
  // AL-iLQR solver termination condition
  enum AliLqrSolveCondition {
    INIT,                     // solver init
    NORMAL_TERMINATE,         // normal terminate, dcost < cost_tol
    CONST_CONTROL_TERMINATE,  // small control u update, du_norm < du_tol
    MAX_ITER_TERMINATE,       // max inner iteration
    LINESEARCH_TERMINATE,     // linesearch terminate, lambda is too big
    INIT_TERMINATE,           // one step iteration, init is the minimum
    MAX_OUTERITER_TERMINATE,  // al-ilqr max outer iteration
    KKT_TERMINATE,            // al-ilqr KKT satisfied
    BACKWARD_PASS_FAIL,       // backward pass failed, non-PD Quu
    NON_POSITIVE_EXPECT,      // non-positive expect, should not happen
    FAULT_INPUT_SIZE,         // input size error
    TIME_LIMIT_TERMINATE,     // wall-clock time limit exceeded
  };

  struct CostInfo {
    size_t id = 0;
    double cost = 0.0;

    CostInfo(const size_t id_, const double cost_) : id(id_), cost(cost_) {}
  };

  // solver info for each inner iteration
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
    std::vector<CostInfo> cost_vec;
  };

  // AL outer loop iteration record for longitudinal constraints
  struct AlIterationInfo {
    // Lagrange multiplier mu and penalty rho for state constraints
    Eigen::MatrixXd mu_k_iteration;
    Eigen::MatrixXd rho_k_iteration;
    // Lagrange multiplier mu and penalty rho for control constraints
    Eigen::MatrixXd mu_u_iteration;
    Eigen::MatrixXd rho_u_iteration;
    // Raw constraint values for each constraint at each horizon step
    Eigen::MatrixXd constraint_data_iteration;
  };

  // AL outer loop iteration record for lateral constraints
  struct LateralAlIterationInfo {
    Eigen::MatrixXd mu_hard_acc_iteration;
    Eigen::MatrixXd rho_hard_acc_iteration;
    Eigen::MatrixXd mu_hard_jerk_iteration;
    Eigen::MatrixXd rho_hard_jerk_iteration;
    Eigen::MatrixXd mu_hard_upper_pos_iteration;
    Eigen::MatrixXd rho_hard_upper_pos_iteration;
    Eigen::MatrixXd mu_hard_lower_pos_iteration;
    Eigen::MatrixXd rho_hard_lower_pos_iteration;
    Eigen::MatrixXd constraint_data_iteration;
  };

  // Per-outer-iteration record: captures inner solve snapshot
  struct OuterIterRecord {
    size_t inner_iter_count = 0;
    double init_cost = 0.0;
    double final_cost = 0.0;
    double constraint_violation = 0.0;
    double dcost_outer = 0.0;
    std::vector<IterationInfo> inner_iter_info_vec;
    std::vector<AliLqrCostVec> inner_cost_iter_vec;
  };

  // solver info for all iterations
  struct AliLqrSolverInfo {
    uint8_t solver_condition = NORMAL_TERMINATE;
    size_t cost_size = 0;
    size_t iter_count = 0;
    double init_cost = 0.0;
    std::vector<IterationInfo> iteration_info_vec;

    // each cost of cost term for all iterations and all horizons
    std::vector<AliLqrCostMap> cost_map_vec;

    // each cost of cost term for every iteration
    std::vector<AliLqrCostVec> cost_iter_vec;

    // outer AL iteration info
    size_t outer_iter_count = 0;
    // Cost change between outer iterations
    double dcost_outer = 0.0;
    // Maximum constraint violation across all steps
    double constraint_violation = 0.0;
    AlIterationInfo al_iteration_info;
    LateralAlIterationInfo lat_al_iteration_info;

    // Accumulated record across all outer iterations
    std::vector<OuterIterRecord> outer_iter_records;
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
  AliLqr() = default;
  virtual ~AliLqr() = default;
  void Init(const std::shared_ptr<AliLqrModel> al_ilqr_model,
            const AliLqrSolverConfig &al_ilqr_solver_config);

  void Init(const std::shared_ptr<AliLqrModel> al_ilqr_model);

  // Standard iLQR solve (without constraint handling)
  void Solve(const State &x0);
  void Solve(const State &x0, const ControlVec &u_vec);
  // Simulation dynamics with given init state x0 and input u_vec
  void Simulation(const State &x0, const ControlVec &u_vec);

  void SetWarmStart(bool flag) {
    solver_config_ptr_->warm_start_enable = flag;
  }

  void SetDuTol(double du_tol) { solver_config_ptr_->du_tol = du_tol; }

  void SetMaxIter(size_t max_iter);

  void SetStateAndInputSize(size_t state_size, size_t input_size) {
    solver_config_ptr_->state_size = state_size;
    solver_config_ptr_->input_size = input_size;
    InitSolverConfig(state_size, input_size);
  }

  void GetOutput(StateVec &xk, ControlVec &uk) const {
    uk = uk_vec_;
    xk = xk_vec_;
  }

  const StateVec *GetStateResultPtr() const { return &xk_vec_; }

  const ControlVec *GetControlResultPtr() const { return &uk_vec_; }

  const AliLqrSolverInfo *GetSolverInfoPtr() { return &solver_info_; }

  void PrintSolverInfo();
  void PrintAlSolverInfo();
  virtual void PrintAlParamInfo();
  void PrintCostInfo();
  void PrintTimeInfo();
  virtual void PrintAlParamInfoAfter();

  void UpdateDynamicsDerivatives();

  void AddCost(std::shared_ptr<AliLqrBaseCostTerm> cost_term);

  void ClearCost();

  void SetSolverConfig(const AliLqrSolverConfig &al_ilqr_solver_config) {
    *solver_config_ptr_ = al_ilqr_solver_config;
  }

  void SetConstraintConfig(
      const std::vector<AliLqrConstraintConfig> &constraint_config) {
    al_ilqr_model_ptr_->SetConstraintConfig(constraint_config);
  }

  void SetCostConfig(const std::vector<AliLqrCostConfig> &cost_config) {
    al_ilqr_model_ptr_->SetCostConfig(cost_config);
  }

  void InitSolverConfig();

  void InitSolverConfig(size_t state_size, size_t input_size);

  std::shared_ptr<AliLqrModel> GetModelPtr() const {
    return al_ilqr_model_ptr_;
  }

  std::shared_ptr<AliLqrSolverConfig> GetSolverConfigPtr() {
    return solver_config_ptr_;
  }

  // init cost map
  void InitAdvancedInfo();

  // update cost_vec
  void UpdateAdvancedInfo(size_t iter);

  // reset solver
  void Reset();

  // --------- AL-iLQR specific interface ---------
  // Initialize inner solver config at each outer AL iteration
  void InitAliLqrSolverConfig();

  // Main entry: solve constrained problem via AL outer + iLQR inner
  void SolveForAliLqr(const State &x0);

  // Outer AL iteration loop.
  // Derived classes override to customize constraint-specific behavior.
  // Default implementation: standard AL outer loop with
  //   iLQR inner solve -> check KKT -> update (mu, rho)
  virtual void AliLqrIteration();

  // Update Lagrange multipliers (mu) and penalty weights (rho).
  //   mu_new = max(0, mu + rho * c)   for inequality c(x,u) <= 0
  //   rho_new = phi * rho             when constraint not yet satisfied
  // Derived classes override with problem-specific AL parameter layout.
  virtual void UpdateAugmentedLagragian();

  // Compute maximum constraint violation across all horizon steps.
  //   max_k max_i |max(0, c_i(x_k, u_k))|
  // Derived classes override with problem-specific constraint evaluation.
  // @return Maximum constraint violation (0 means all constraints satisfied)
  virtual double MaxConstraintViolation();

  // Compute cost deviation metric between outer iterations.
  // Used as stationarity indicator in KKT convergence check.
  // @return Cost deviation value
  virtual double MaxDerivationValue();

 protected:
  virtual bool ForwardPass(double &new_cost, double &expected,
                           const size_t &iter);

  virtual bool BackwardPass();
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
  std::shared_ptr<AliLqrModel> al_ilqr_model_ptr_;

  // AL constraint parameters shared with model
  std::shared_ptr<std::vector<AliLqrConstraintConfig>>
      constraint_config_vec_ptr_;
  std::shared_ptr<std::vector<AliLqrCostConfig>> cost_config_vec_ptr_;

  // solver config
  std::shared_ptr<AliLqrSolverConfig> solver_config_ptr_;

  // solver info
  AliLqrSolverInfo solver_info_;

  // time debug info
  TimeInfo time_info_;

  // Record cost from previous outer iteration for convergence check
  double prev_outer_cost_ = 0.0;
};
}  // namespace al_ilqr_solver

#endif  // __AL_ILQR_CORE_H__
