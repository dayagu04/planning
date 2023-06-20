#include <Eigen/Dense>
#include <random>

#include "gtest/gtest.h"
#include "motion_planners/lateral_motion_planner/src/lateral_motion_planning_cost.h"

#define DEBUG

using namespace std;
using namespace pnc::lateral_planning;
namespace pnc {
namespace lateral_planning {

#define EPS (1E-3)

State x;
Control u;

LxMT lx, lx_dif;
LuMT lu, lu_dif;
LxxMT lxx, lxx_dif;
LxuMT lxu, lxu_dif;
LuuMT luu, luu_dif;

IlqrCostConfig cost_config;

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
  cost_config[CONTINUITY_X] = 1.0;
  cost_config[CONTINUITY_Y] = 1.0;
  cost_config[CONTINUITY_THETA] = 1.0;
  cost_config[DELTA_BOUND] = 0.1;
  cost_config[OMEGA_BOUND] = 0.1;

  cost_config[SOFT_UPPER_BOUND_X0] = 0.0;
  cost_config[SOFT_UPPER_BOUND_Y0] = 1.0;

  cost_config[SOFT_UPPER_BOUND_X1] = 1.0;
  cost_config[SOFT_UPPER_BOUND_Y1] = 1.0;

  cost_config[SOFT_LOWER_BOUND_X0] = 0.0;
  cost_config[SOFT_LOWER_BOUND_Y0] = 0.0;

  cost_config[SOFT_LOWER_BOUND_X1] = 1.0;
  cost_config[SOFT_LOWER_BOUND_Y1] = 0.0;

  cost_config[W_REF_X] = 2.0;
  cost_config[W_REF_Y] = 2.0;
  cost_config[W_REF_THETA] = 3.0;
  cost_config[W_CONTINUITY_X] = 5.0;
  cost_config[W_CONTINUITY_Y] = 2.0;
  cost_config[W_CONTINUITY_THETA] = 3.0;
  cost_config[W_ACC] = 5.0;
  cost_config[W_SNAP] = 2.0;
  cost_config[W_JERK] = 2.0;
  cost_config[W_ACC_BOUND] = 100.0;
  cost_config[W_JERK_BOUND] = 100.0;
  cost_config[W_SOFT_CORRIDOR] = 1000.0;
  cost_config[REF_VEL] = 20.0;
  cost_config[CURV_FACTOR] = 0.2;
  cost_config[TERMINAL_FLAG] = 0.0;
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

  x << GetRandomDouble(-5.0, 5.0), GetRandomDouble(-5.0, 5.0), GetRandomDouble(-0.8, 0.8),
      GetRandomDouble(-1.0, 1.0) * 200.0 / 57.3 / 14.5, u << GetRandomDouble(-1.0, 1.0) * 1;

  if (std::abs(std::abs(x(3)) - cost_config[DELTA_BOUND]) < 0.02) {
    x(3) += 0.05;
  }
  if (std::abs(std::abs(u(0)) - cost_config[OMEGA_BOUND]) < 0.02) {
    u(0) += 0.1;
  }

  cout << "lat_acc = " << std::tan(x(3)) * cost_config[CURV_FACTOR] * (cost_config[REF_VEL] * cost_config[REF_VEL])
       << endl;
  cout << "lat_jerk = " << u(0) * cost_config[CURV_FACTOR] * (cost_config[REF_VEL] * cost_config[REF_VEL]) << endl;
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
  std::shared_ptr<ilqr_solver::BaseCostTerm> ref_cost_term_ptr = std::make_shared<ReferenceCostTerm>();
  clear();
  Init();
  Generate();
  ref_cost_term_ptr->SetConfig(&cost_config);
  ref_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("ref_cost_term");

  ref_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);
  Print_dif("ref_cost_term_diff");
  Compare();
}

TEST(ContinuityCostTerm, continuity_ref_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> continuity_ref_cost_term_ptr = std::make_shared<ContinuityCostTerm>();
  clear();
  Init();
  Generate();

  continuity_ref_cost_term_ptr->SetConfig(&cost_config);
  continuity_ref_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("continuity_ref_cost_term");

  continuity_ref_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);

  Print_dif("ref_cost_term_diff");

  Compare();
}

TEST(LatAccCostTerm, lat_acc_cost_term) {
  std::shared_ptr<ilqr_solver::BaseCostTerm> lat_acc_cost_term_ptr = std::make_shared<LatAccCostTerm>();
  clear();
  Init();
  Generate();
  lat_acc_cost_term_ptr->SetConfig(&cost_config);
  lat_acc_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("lat_acc_cost_term");

  lat_acc_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);

  Print_dif("lat_acc_cost_term_diff");

  Compare();
}

TEST(LatJerkCostTerm, lat_jerk_cost_term) {
  clear();
  Init();
  Generate();
  std::shared_ptr<ilqr_solver::BaseCostTerm> lat_jerk_cost_term_ptr = std::make_shared<LatJerkCostTerm>();

  lat_jerk_cost_term_ptr->SetConfig(&cost_config);
  lat_jerk_cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("lat_jerk_cost_term");

  lat_jerk_cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);

  Print_dif("lat_jerk_cost_term_diff");
  Compare();
}

TEST(LatAccBoundCostTerm, lat_acc_bound_cost_term) {
  clear();
  Init();
  Generate();
  std::shared_ptr<ilqr_solver::BaseCostTerm> lat_acc_bound_cost_term = std::make_shared<LatAccBoundCostTerm>();

  lat_acc_bound_cost_term->SetConfig(&cost_config);
  lat_acc_bound_cost_term->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("lat_acc_bound_cost_term");

  lat_acc_bound_cost_term->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);

  Print_dif("lat_acc_bound_cost_term_diff");

  Compare();
}

TEST(LatJerkBoundCostTerm, lat_jerk_bound_cost_term) {
  clear();
  Init();
  Generate();
  std::shared_ptr<ilqr_solver::BaseCostTerm> lat_jerk_bound_cost_term = std::make_shared<LatJerkBoundCostTerm>();

  lat_jerk_bound_cost_term->SetConfig(&cost_config);
  lat_jerk_bound_cost_term->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("lat_jerk_bound_cost_term");

  lat_jerk_bound_cost_term->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);

  Print_dif("lat_jerk_bound_cost_term_diff");

  Compare();
}

TEST(PathSoftCorridorCostTerm, path_soft_corridor_cost_term) {
  clear();
  Init();
  Generate();
  std::shared_ptr<ilqr_solver::BaseCostTerm> path_soft_corridor_cost_term =
      std::make_shared<PathSoftCorridorCostTerm>();

  path_soft_corridor_cost_term->SetConfig(&cost_config);
  path_soft_corridor_cost_term->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print("path_soft_corridor_cost_term");

  path_soft_corridor_cost_term->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif, luu_dif);

  Print_dif("path_soft_corridor_cost_term");

  Compare();
}

// int main(int argc, char **argv) {
//   printf("Testing google test!");
//   static const int N = 1000;
//   testing::InitGoogleTest(&argc, argv);
//   int out = 0;
//   int i = 0;
//   for (i = 0; i < N; ++i) {
//     cout << "--------------------------------- i = " << i
//          << "-----------------------------------" << endl;
//     out = RUN_ALL_TESTS();
//     if (out) {
//       cout << "wocao wocao wocao wocao wocao" << endl;
//       break;
//     }
//     cout << "out = " << out << endl;
//   }

//   cout << "out = " << out << ", i = " << i << endl;
//   return out;
// }
}  // namespace lateral_planning
}  // namespace pnc