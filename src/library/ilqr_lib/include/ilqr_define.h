#ifndef __ILQR_DEFINE_H__
#define __ILQR_DEFINE_H__

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <unordered_map>
#include <vector>

// for offline debug
#define __ILQR_DEBUG__

// for offline info print
// #define __ILQR_PRINT__

// for offline timer
// #define __ILQR_TIMER__

// size of cost config array
#ifndef MAX_COST_CONFIG_SIZE
#define MAX_COST_CONFIG_SIZE (50)
#endif

// size of CONSTRAINT COST CONFIG array
#ifndef MAX_CONSTRAINT_COST_CONFIG_SIZE
#define MAX_CONSTRAINT_COST_CONFIG_SIZE (28)
#endif

// how many costs we can have
#ifndef MAX_COST_SIZE
#define MAX_COST_SIZE (20)
#endif
namespace ilqr_solver {
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

// ilqr cost config: use array instead of vector
typedef std::array<double, MAX_COST_CONFIG_SIZE> IlqrCostConfig;

// al-ilqr constraint weight config: use array
typedef std::array<double, MAX_CONSTRAINT_COST_CONFIG_SIZE> AliLqrConfig;

// ilqr cost info for init and each iteration: unordered map
typedef std::unordered_map<uint8_t, std::vector<double>> ILqrCostMap;

// ilqr cost info for init and each iteration: vecotr
typedef std::vector<double> ILqrCostVec;

// ilqr solver config for solver parameters

struct iLqrSolverConfig {
  size_t horizon = 25;
  size_t state_size = 3;
  size_t input_size = 1;
  size_t max_iter = 10;
  size_t max_backward_pass_count = 5;
  bool warm_start_enable = false;
  bool c_ilqr_enable = false;
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
  double cost_scale = 0.95;
  size_t max_al_iter = 10;
  size_t constraint_num = 2;
  double cost_tolerance_tol = 1e-4;        // 1e2 ; 1e-8
  double constraint_tolerance_tol = 1e-3;  // 1e2 ; 1e-8
};

}  // namespace ilqr_solver
#endif
