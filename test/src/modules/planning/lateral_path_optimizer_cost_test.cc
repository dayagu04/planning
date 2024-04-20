#include "apa_function/src/lateral_path_optimizer/src/lateral_path_optimizer_cost.h"

#include <math.h>

#include <Eigen/Dense>
#include <random>

#include "gtest/gtest.h"
#define DEBUG

using namespace std;
using namespace planning::apa_planner;

namespace planning {
namespace apa_planner {
#define EPS (1E-3)

ilqr_solver::State x;
ilqr_solver::Control u;

ilqr_solver::LxMT lx, lx_dif;
ilqr_solver::LuMT lu, lu_dif;
ilqr_solver::LxxMT lxx, lxx_dif;
ilqr_solver::LxuMT lxu, lxu_dif;
ilqr_solver::LuuMT luu, luu_dif;

ilqr_solver::IlqrCostConfig cost_config;
ilqr_solver::AliLqrConfig alilqr_config_vec;
void clear() {
  x.resize(0);
  u.resize(0);
  lx.resize(0);
  lu.resize(0);
  lxx.resize(0, 0);
  lxu.resize(0, 0);
  luu.resize(0, 0);

  lx_dif.resize(0);
  lu_dif.resize(0);
  lxx_dif.resize(0, 0);
  lxu_dif.resize(0, 0);
  luu_dif.resize(0, 0);
}

void Init() {
#ifdef DEBUG

  cout << "--------------------------------------------------------------------"
          "-------------------"
       << endl;
#endif
  x.resize(STATE_SIZE);
  u.resize(INPUT_SIZE);
  lx.resize(STATE_SIZE);
  lu.resize(INPUT_SIZE);
  lxx.resize(STATE_SIZE, STATE_SIZE);
  lxu.resize(STATE_SIZE, INPUT_SIZE);
  luu.resize(INPUT_SIZE, INPUT_SIZE);
  x.setZero();
  u.setZero();
  lx.setZero();
  lu.setZero();
  lxx.setZero();
  lxu.setZero();
  luu.setZero();

  lx_dif.resize(STATE_SIZE);
  lu_dif.resize(INPUT_SIZE);
  lxx_dif.resize(STATE_SIZE, STATE_SIZE);
  lxu_dif.resize(STATE_SIZE, INPUT_SIZE);
  luu_dif.resize(INPUT_SIZE, INPUT_SIZE);
  lx_dif.setZero();
  lu_dif.setZero();
  lxx_dif.setZero();
  lxu_dif.setZero();
  luu_dif.setZero();

  cost_config[REF_X] = 1.0;
  cost_config[REF_Y] = 1.0;
  cost_config[REF_THETA] = 1.0;
  cost_config[TERMINAL_THETA] = 0.1;
  cost_config[TERMINAL_X] = 0.1;
  cost_config[TERMINAL_Y] = 0.1;
  cost_config[K_MAX] = 1.0 / 6;
  cost_config[U_MAX] = 0.3 * 400 / 57.3 / 15 / 1.688;

  cost_config[W_REF_X] = 100.0;
  cost_config[W_REF_Y] = 100.0;
  cost_config[W_REF_THETA] = 100.0;
  cost_config[W_TERMINAL_X] = 9000.0;
  cost_config[W_TERMINAL_Y] = 9000.0;
  cost_config[W_TERMINAL_THETA] = 9000.0;
  cost_config[W_U] = 10.0;
  cost_config[W_K] = 10.0;
  alilqr_config_vec[W_K_HARDBOUND] = 75.0;
  alilqr_config_vec[W_U_HARDBOUND] = 75.0;
  alilqr_config_vec[L_K_HARDBOUND] = 0.0;
  alilqr_config_vec[L_U_HARDBOUND] = 0.05412;
}

void SetRandomMat(Eigen::MatrixXd &A) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      A(i, j) = dist(gen);
    }
  }
}

void SetRandomVec(Eigen::VectorXd &A) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(-5.0, 5.0);

  for (int i = 0; i < A.size(); i++) {
    A(i) = dist(gen);
  }
}

double GetRandomDouble(double lb, double ub) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(lb, ub);

  return dist(gen);
}

void Generate() {
  std::random_device rd;
  std::mt19937 gen(rd());
  // SetRandomVec(x);
  // SetRandomVec(u);

  x << GetRandomDouble(-5.0, 5.0), GetRandomDouble(-5.0, 5.0),
      GetRandomDouble(-2.0, 2.0),
      GetRandomDouble(0.0, (cost_config[K_MAX] + 0.0));

  u << GetRandomDouble(-1.0, 1.0) * (cost_config[U_MAX] + 0.0);

  if (std::abs(std::abs(x(3)) - cost_config[K_MAX]) < 0.02) {
    x(3) += 0.2;
  }
  if (std::abs(std::abs(u(0)) - cost_config[U_MAX]) < 0.02) {
    u(0) += 0.1;
  }

  // u.setRandom(gen);

  Eigen::MatrixXd A;
  A.resize(3, 3);
#ifdef DEBUG
  cout << "x = \n[" << x << "]" << endl;
  cout << "u = \n[" << u << "]" << endl;
#endif
}

void Print(string str) {
#ifdef DEBUG
  cout << "------" << str << "------" << endl;
  cout << "lx = \n[" << lx << "]" << endl;
  cout << "lu = \n[" << lu << "]" << endl;
  cout << "lxu = \n[" << lxu << "]" << endl;
  cout << "lxx = \n[" << lxx << "]" << endl;
  cout << "luu = \n[" << luu << "]" << endl;
  cout << "-----------------------" << endl;
#endif
}

void Print_dif(string str) {
#ifdef DEBUG
  cout << "------" << str << "------" << endl;
  cout << "lx_dif = \n[" << lx_dif << "]" << endl;
  cout << "lu_dif = \n[" << lu_dif << "]" << endl;
  cout << "lxu_dif = \n[" << lxu_dif << "]" << endl;
  cout << "lxx_dif = \n[" << lxx_dif << "]" << endl;
  cout << "luu_dif = \n[" << luu_dif << "]" << endl;
  cout << "-----------------------" << endl;
#endif
}

void CompareMatrix(Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
  for (int i = 0; i < A.rows(); ++i)
    for (int j = 0; j < A.cols(); ++j) {
      EXPECT_NEAR(A(i, j), B(i, j), EPS);
    }
}

void CompareMatrix(Eigen::VectorXd &A, Eigen::VectorXd &B) {
  for (int i = 0; i < A.rows(); ++i) {
    EXPECT_NEAR(A(i), B(i), EPS);
  }
}

void Compare() {
  CompareMatrix(lx, lx_dif);
  CompareMatrix(lu, lu_dif);
  CompareMatrix(lxx, lxx_dif);
  CompareMatrix(lxu, lxu_dif);
  CompareMatrix(luu, luu_dif);
}

TEST(ReferenceCostTerm, ref_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> ref_cost_term_ptr =
      std::make_shared<ReferenceCostTerm>();
  clear();
  Init();
  Generate();
  ref_cost_term_ptr->SetConfig(&cost_config);
  ref_cost_term_ptr->SetAliLqrConfig(&alilqr_config_vec);
  ref_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("ref_cost_term");

  ref_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif,
                                            lxu_dif, luu_dif);
  Print_dif("ref_cost_term_diff");
  Compare();
}

TEST(TernimalCostTerm, terminal_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> terminal_cost_term_ptr =
      std::make_shared<TerminalCostTerm>();
  clear();
  Init();
  Generate();

  terminal_cost_term_ptr->SetConfig(&cost_config);
  terminal_cost_term_ptr->SetAliLqrConfig(&alilqr_config_vec);
  terminal_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("terminal_cost_term_ptr");

  terminal_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif,
                                                 lxu_dif, luu_dif);

  Print_dif("terminal_cost_term_diff");

  Compare();
}

TEST(KBoundCostTerm, kbound_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> kbound_cost_term_ptr =
      std::make_shared<KBoundCostTerm>();
  clear();
  Init();
  Generate();
  kbound_cost_term_ptr->SetConfig(&cost_config);
  kbound_cost_term_ptr->SetAliLqrConfig(&alilqr_config_vec);
  kbound_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("kbound_cost_term");

  kbound_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif,
                                               lxu_dif, luu_dif);

  Print_dif("kbound_cost_term_diff");

  Compare();
}

TEST(UCostTerm, u_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> u_cost_term_ptr =
      std::make_shared<UCostTerm>();
  clear();
  Init();
  Generate();
  u_cost_term_ptr->SetConfig(&cost_config);
  u_cost_term_ptr->SetAliLqrConfig(&alilqr_config_vec);
  u_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("u_cost_term");

  u_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif,
                                          lxu_dif, luu_dif);

  Print_dif("u_cost_term_diff");

  Compare();
}

TEST(UBoundCostTerm, u_bound_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> ubound_cost_term_ptr =
      std::make_shared<UBoundCostTerm>();
  clear();
  Init();
  Generate();
  ubound_cost_term_ptr->SetConfig(&cost_config);
  ubound_cost_term_ptr->SetAliLqrConfig(&alilqr_config_vec);
  ubound_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("ubound_cost_term");

  ubound_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif,
                                               lxu_dif, luu_dif);

  Print_dif("ubound_cost_term_diff");

  Compare();
}

}  // namespace apa_planner
}  // namespace planning