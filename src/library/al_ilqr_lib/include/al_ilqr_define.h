#ifndef __AL_ILQR_DEFINE_H__
#define __AL_ILQR_DEFINE_H__

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <unordered_map>
#include <vector>

// for offline debug
#define __AL_ILQR_DEBUG__

// for offline info print
// #define __AL_ILQR_PRINT__

// for offline timer
// #define __AL_ILQR_TIMER__

// size of cost config array
#ifndef MAX_COST_CONFIG_SIZE
#define MAX_COST_CONFIG_SIZE (100)
#endif

// size of CONSTRAINT COST CONFIG array
#ifndef MAX_CONSTRAINT_COST_CONFIG_SIZE
#define MAX_CONSTRAINT_COST_CONFIG_SIZE (28)
#endif

// how many costs we can have
#ifndef MAX_COST_SIZE
#define MAX_COST_SIZE (20)
#endif
namespace al_ilqr_solver {
// state of model
typedef Eigen::VectorXd State;

// control of model
typedef Eigen::VectorXd Control;

// definition of ilqr matrix for each iteration
typedef Eigen::VectorXd XkMT;
typedef Eigen::VectorXd UkMT;
typedef Eigen::VectorXd LxMT;  // lx size is equal to xk
typedef Eigen::VectorXd LuMT;  // lu size is equal to uk
typedef Eigen::MatrixXd LxxMT;
typedef Eigen::MatrixXd LxuMT;
typedef Eigen::MatrixXd LuuMT;
typedef Eigen::MatrixXd FxMT;
typedef Eigen::MatrixXd FuMT;
typedef Eigen::VectorXd kMT;
typedef Eigen::MatrixXd KMT;

// vector of definition of ilqr matrix for all iteration
typedef std::vector<Eigen::VectorXd> StateVec;
typedef std::vector<Eigen::VectorXd> ControlVec;
typedef std::vector<Eigen::VectorXd> LxMTVec;
typedef std::vector<Eigen::VectorXd> LuMTVec;
typedef std::vector<Eigen::MatrixXd> LxxMTVec;
typedef std::vector<Eigen::MatrixXd> LxuMTVec;
typedef std::vector<Eigen::MatrixXd> LuuMTVec;
typedef std::vector<Eigen::MatrixXd> FxMTVec;
typedef std::vector<Eigen::MatrixXd> FuMTVec;
typedef std::vector<Eigen::VectorXd> kMTVec;
typedef std::vector<Eigen::MatrixXd> KMTVec;

// al-ilqr cost config: use array instead of vector
typedef std::array<double, MAX_COST_CONFIG_SIZE> AliLqrCostConfig;

// al-ilqr constraint weight config: use array
// Layout defined by derived cost classes:
//   stores Lagrange multiplier (mu) and penalty weight (rho) per constraint
typedef std::array<double, MAX_CONSTRAINT_COST_CONFIG_SIZE> AliLqrConstraintConfig;

// al-ilqr cost info for init and each iteration: unordered map
typedef std::unordered_map<uint8_t, std::vector<double>> AliLqrCostMap;

// al-ilqr cost info for init and each iteration: vector
typedef std::vector<double> AliLqrCostVec;

// al-ilqr solver config for solver parameters
struct AliLqrSolverConfig {
  size_t horizon = 25;
  size_t state_size = 3;
  size_t input_size = 1;
  size_t max_iter = 10;
  size_t max_backward_pass_count = 5;
  bool warm_start_enable = false;
  // Enable augmented Lagrangian constraint handling
  bool al_ilqr_enable = false;
  double lambda_factor = 2.0;
  double lambda_max = 1e4;
  double lambda_min = 1e-5;
  double lambda_start = 20.0;
  double lambda_fix = 0.1;
  double cost_tol = 1e-4;
  double du_tol = 2.5e-5;
  double z_min = 0.0;
  double model_dt = 0.0;
  double loop_dt = 0.0;
  std::vector<double> alpha_vec = {
      1.0000, 0.6180, 0.3819, 0.2360, 0.1458,
      0.0901, 0.0557, 0.0344, 0.01};  // fixed linesearch step

  // outer al-ilqr parameters
  // Inner cost tolerance scale factor per outer iteration:
  //   cost_tol *= cost_scale each outer step to tighten convergence
  double cost_scale = 0.95;
  // Maximum number of outer AL iterations
  size_t max_al_iter = 10;
  // Number of constraint types
  size_t constraint_num = 2;
  // Outer loop cost change tolerance for KKT check
  double cost_tolerance_tol = 1e-4;        // 1e2 ; 1e-8
  // Outer loop constraint violation tolerance for KKT check
  double constraint_tolerance_tol = 1e-3;  // 1e2 ; 1e-8
  // Maximum wall-clock time for the entire AL solve (ms)
  double max_al_solve_time = 30.0;  // ms
  // Penalty scaling factor: rho_new = penalty_scaling * rho
  double penalty_scaling = 10.0;
  // Initial penalty weight
  double penalty_init = 1.0;
  // Maximum penalty weight to avoid ill-conditioning
  double penalty_max = 1e6;
};

}  // namespace al_ilqr_solver
#endif  // __AL_ILQR_DEFINE_H__
