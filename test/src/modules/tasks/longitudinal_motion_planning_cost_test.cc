#include <Eigen/Dense>
#include <cstddef>
#include <random>

#include "motion_planners/longitudinal_motion_planner/src/longitudinal_motion_planning_cost.h"
#include "gtest/gtest.h"

using namespace std;
using namespace pnc::longitudinal_planning;
namespace pnc {
namespace longitudinal_planning {

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

  cost_config[REF_POS] = 5.0;
  cost_config[REF_VEL] = 5.0;

  cost_config[S_STOP] = 5.0;

  cost_config[POS_MAX] = 5.0 * 1.0;
  cost_config[POS_MIN] = -5.0 * 1.0;
  cost_config[VEL_MAX] = 5.0 * 1.0;
  cost_config[VEL_MIN] = -5.0 * 1.0;
  cost_config[ACC_MAX] = 5.0 * 1.0;
  cost_config[ACC_MIN] = -5.0 * 1.0;
  cost_config[JERK_MAX] = 5.0 * 1.0;
  cost_config[JERK_MIN] = -5.0 * 1.0;

  cost_config[W_REF_POS] = 2.5;
  cost_config[W_REF_VEL] = 2.2;
  cost_config[W_ACC] = 3.2;
  cost_config[W_JERK] = 1.5;
  cost_config[W_SNAP] = 0.6;

  cost_config[W_POS_BOUND] = 2.2;
  cost_config[W_VEL_BOUND] = 2.5;
  cost_config[W_ACC_BOUND] = 1.2;
  cost_config[W_JERK_BOUND] = 5.0;

  cost_config[W_S_STOP] = 500.0;

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

  x << GetRandomDouble(-10, 10.0), GetRandomDouble(-10, 10.0),
      GetRandomDouble(-10.0, 10.0), GetRandomDouble(-10.0, 10.0);
  u << GetRandomDouble(-10.0, 10.0);

  if (fabs(fabs(x(POS)) - 5.0) < 0.02) {
    x(POS) += 10.0;
  }

  if (fabs(fabs(x(VEL)) - 5.0) < 0.02) {
    x(VEL) += 10.0;
  }

  if (fabs(fabs(x(ACC)) - 5.0) < 0.02) {
    x(ACC) += 10.0;
  }

  if (fabs(fabs(x(JERK)) - 5.0) < 0.02) {
    x(JERK) += 10.0;
  }

  Eigen::MatrixXd A;
  A.resize(4, 4);
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
  cout << "------" << str << "_dif"
       << "------" << endl;
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

void ApplyCostTest(std::shared_ptr<ilqr_solver::BaseCostTerm> cost_term_ptr) {
  clear();
  Init();
  Generate();
  cost_term_ptr->SetConfig(&cost_config);
  cost_term_ptr->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);

  Print(cost_term_ptr->GetCostString());

  cost_term_ptr->GetDiffGradientHessian(x, u, lx_dif, lu_dif, lxx_dif, lxu_dif,
                                        luu_dif);
  Print_dif(cost_term_ptr->GetCostString());
  Compare();
}

TEST(ReferenceCostTerm, ReferenceCostTerm) {
  ApplyCostTest(std::make_shared<ReferenceCostTerm>());
}

TEST(LonAccCostTerm, LonAccCostTerm) {
  ApplyCostTest(std::make_shared<LonAccCostTerm>());
}

TEST(LonJerkCostTerm, LonJerkCostTerm) {
  ApplyCostTest(std::make_shared<LonJerkCostTerm>());
}

TEST(LonSnapCostTerm, LonSnapCostTerm) {
  ApplyCostTest(std::make_shared<LonSnapCostTerm>());
}

TEST(LonPosBoundCostTerm, LonPosBoundCostTerm) {
  ApplyCostTest(std::make_shared<LonPosBoundCostTerm>());
}

TEST(LonVelBoundCostTerm, LonVelBoundCostTerm) {
  ApplyCostTest(std::make_shared<LonVelBoundCostTerm>());
}

TEST(LonAccBoundCostTerm, LonAccBoundCostTerm) {
  ApplyCostTest(std::make_shared<LonAccBoundCostTerm>());
}

TEST(LonJerkBoundCostTerm, LonJerkBoundCostTerm) {
  ApplyCostTest(std::make_shared<LonJerkBoundCostTerm>());
}

TEST(LonStopPointCost, LonStopPointCost) {
  ApplyCostTest(std::make_shared<LonStopPointCost>());
}

int main(int argc, char **argv) {
  printf("Testing google test!");
  static const int N = 1000;
  testing::InitGoogleTest(&argc, argv);
  int out = 0;
  int i = 0;
  for (i = 0; i < N; ++i) {
    cout << "--------------------------------- i = " << i
         << "-----------------------------------" << endl;
    out = RUN_ALL_TESTS();
    if (out) {
      cout << "wocao wocao wocao wocao wocao" << endl;
      break;
    }
    cout << "out = " << out << endl;
  }

  cout << "out = " << out << ", i = " << i << endl;
  return out;
}
} // namespace longitudinal_planning
} // namespace pnc