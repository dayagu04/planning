#include "utils/kd_path.h"

#include <math.h>

#include <array>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <variant>
#include <vector>

#include "ctime"
#include "gtest/gtest.h"
#include "log.h"
#include "longitudinal_motion_planner.pb.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/path_point.h"
#include "vec2d.h"

using namespace planning;
using namespace planning_math;
namespace planning {
namespace planning_math {
constexpr double sqrt_2 = 1.4142;
std::vector<PathPoint> path_points_line = {
    {0.0, 0.0},    {1.0, 1.0},   {2.0, 2.0},   {3.0, 3.0},   {4.0, 4.0},
    {5.0, 5.0},    {6.0, 6.0},   {7.0, 7.0},   {8.0, 8.0},   {9.0, 9.0},
    {10.0, 10.0},  {11.0, 11.0}, {12.0, 12.0}, {13.0, 13.0}, {14.0, 14.0},
    {15.0, 15.0},  {16.0, 16.0}, {17.0, 17.0}, {18.0, 18.0}, {19.0, 19.0},
    {20.0, 20.0},  {21.0, 21.0}, {22.0, 22.0}, {23.0, 23.0}, {24.0, 24.0},
    {25.0, 25.0},  {26.0, 26.0}, {27.0, 27.0}, {28.0, 28.0}, {29.0, 29.0},
    {30.0, 30.0},  {31.0, 31.0}, {32.0, 32.0}, {33.0, 33.0}, {34.0, 34.0},
    {35.0, 35.0},  {36.0, 36.0}, {37.0, 37.0}, {38.0, 38.0}, {39.0, 39.0},
    {40.0, 40.0},  {41.0, 41.0}, {42.0, 42.0}, {43.0, 43.0}, {44.0, 44.0},
    {45.0, 45.0},  {46.0, 46.0}, {47.0, 47.0}, {48.0, 48.0}, {49.0, 49.0},
    {50.0, 50.0},  {51.0, 51.0}, {52.0, 52.0}, {53.0, 53.0}, {54.0, 54.0},
    {55.0, 55.0},  {56.0, 56.0}, {57.0, 57.0}, {58.0, 58.0}, {59.0, 59.0},
    {60.0, 60.0},  {61.0, 61.0}, {62.0, 62.0}, {63.0, 63.0}, {64.0, 64.0},
    {65.0, 65.0},  {66.0, 66.0}, {67.0, 67.0}, {68.0, 68.0}, {69.0, 69.0},
    {70.0, 70.0},  {71.0, 71.0}, {72.0, 72.0}, {73.0, 73.0}, {74.0, 74.0},
    {75.0, 75.0},  {76.0, 76.0}, {77.0, 77.0}, {78.0, 78.0}, {79.0, 79.0},
    {80.0, 80.0},  {81.0, 81.0}, {82.0, 82.0}, {83.0, 83.0}, {84.0, 84.0},
    {85.0, 85.0},  {86.0, 86.0}, {87.0, 87.0}, {88.0, 88.0}, {89.0, 89.0},
    {90.0, 90.0},  {91.0, 91.0}, {92.0, 92.0}, {93.0, 93.0}, {94.0, 94.0},
    {95.0, 95.0},  {96.0, 96.0}, {97.0, 97.0}, {98.0, 98.0}, {99.0, 99.0},
    {100.0, 100.0}

};

// One quarter of a circle
std::vector<PathPoint> path_points_circle = {
    {0, 1.14239e-12},    {0.0123368, 1.57073}, {0.049344, 3.14108},
    {0.111013, 4.71065}, {0.197327, 6.27905},  {0.308267, 7.84591},
    {0.443804, 9.41083}, {0.603904, 10.9734},  {0.78853, 12.5333},
    {0.997634, 14.0901}, {1.23117, 15.6434},   {1.48907, 17.1929},
    {1.77127, 18.7381},  {2.07772, 20.2787},   {2.40832, 21.8143},
    {2.76301, 23.3445},  {3.14168, 24.869},    {3.54426, 26.3873},
    {3.97063, 27.8991},  {4.4207, 29.404},     {4.89435, 30.9017},
    {5.39146, 32.3917},  {5.91192, 33.8738},   {6.4556, 35.3475},
    {7.02235, 36.8125},  {7.61205, 38.2683},   {8.22454, 39.7148},
    {8.85967, 41.1514},  {9.51729, 42.5779},   {10.1972, 43.9939},
    {10.8993, 45.399},   {11.6234, 46.793},    {12.3693, 48.1754},
    {13.1368, 49.5459},  {13.9258, 50.9041},   {14.736, 52.2499},
    {15.5672, 53.5827},  {16.4193, 54.9023},   {17.2919, 56.2083},
    {18.185, 57.5005},   {19.0983, 58.7785},   {20.0315, 60.042},
    {20.9845, 61.2907},  {21.957, 62.5243},    {22.9487, 63.7424},
    {23.9594, 64.9448},  {24.9889, 66.1312},   {26.0369, 67.3013},
    {27.1031, 68.4547},  {28.1874, 69.5913},   {29.2893, 70.7107},
    {30.4087, 71.8126},  {31.5453, 72.8969},   {32.6987, 73.9631},
    {33.8688, 75.0111},  {35.0552, 76.0406},   {36.2576, 77.0513},
    {37.4757, 78.043},   {38.7093, 79.0155},   {39.958, 79.9685},
    {41.2215, 80.9017},  {42.4995, 81.815},    {43.7917, 82.7081},
    {45.0977, 83.5807},  {46.4173, 84.4328},   {47.7501, 85.264},
    {49.0959, 86.0742},  {50.4541, 86.8632},   {51.8246, 87.6307},
    {53.207, 88.3766},   {54.601, 89.1007},    {56.0061, 89.8028},
    {57.4221, 90.4827},  {58.8486, 91.1403},   {60.2852, 91.7755},
    {61.7317, 92.388},   {63.1875, 92.9776},   {64.6525, 93.5444},
    {66.1262, 94.0881},  {67.6083, 94.6085},   {69.0983, 95.1057},
    {70.596, 95.5793},   {72.1009, 96.0294},   {73.6127, 96.4557},
    {75.131, 96.8583},   {76.6555, 97.237},    {78.1857, 97.5917},
    {79.7213, 97.9223},  {81.2619, 98.2287},   {82.8071, 98.5109},
    {84.3566, 98.7688},  {85.9099, 99.0024},   {87.4667, 99.2115},
    {89.0266, 99.3961},  {90.5892, 99.5562},   {92.1541, 99.6917},
    {93.7209, 99.8027},  {95.2894, 99.889},    {96.8589, 99.9507},
    {98.4293, 99.9877},  {100, 100},           {101.571, 99.9877},
    {103.141, 99.9507},  {104.711, 99.889},    {106.279, 99.8027},
    {107.846, 99.6917},  {109.411, 99.5562},   {110.973, 99.3961}};

struct GroundTruthPoint {
  double x;
  double y;
  double s;
  double l;
};

TEST(TEST_KD_PATH, KDPath) {
  // line
  // std::vector<PathPoint> path_points_test = path_points;
  // // generate ground truth values
  // std::vector<GroundTruthPoint> ground_truth_points;
  // ground_truth_points.resize(path_points_test.size());
  // for (auto i = 0; i < ground_truth_points.size(); i++) {
  //   ground_truth_points[i].x = path_points_test[i].x();
  //   ground_truth_points[i].y =
  //       path_points_test[i].y() + 1;  // paralle the ref line
  //   ground_truth_points[i].s = (i + 0.5) * sqrt_2;
  //   ground_truth_points[i].l = 0.5 * sqrt_2;
  // }

  // One quarter of a circle
  std::vector<PathPoint> path_points_test = path_points_circle;
  // generate ground truth values
  // path_points为原点(100,0),半径100的四分之一圆，ground_truth_points的构造为原点(100,0)，半径101的圆的一部分；
  std::vector<GroundTruthPoint> ground_truth_points_circle;
  ground_truth_points_circle.resize(path_points_test.size());
  for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
    ground_truth_points_circle[i].x = path_points_test[i].x();
    ground_truth_points_circle[i].y =
        sqrt(pow(101, 2) -
             pow(path_points_test[i].x() - 100, 2));  // paralle the ref line
    ground_truth_points_circle[i].l = 1;
  }
  // ground_truth_points[0].s = acos(100/101) * M_PI/180 * 100;
  for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
    ground_truth_points_circle[i].s =
        acos((100 - path_points_test[i].x()) / 101) * 100;
  }

  // ------------------------------------------------KD_PATH_CoordinateSystem
  path_points_test[0].set_s(0);
  for (auto i = 1; i < path_points_test.size();
       i++) {  // init s info of path point
    path_points_test[i].set_s(
        path_points_test[i - 1].s() +
        std::hypot(path_points_test[i].x() - path_points_test[i - 1].x(),
                   path_points_test[i].y() - path_points_test[i - 1].y()));
  }

  // log consturct kd frenet crood time
  clock_t start_consturct_kd_frenet_crood_time,
      end_consturct_kd_frenet_crood_time;
  start_consturct_kd_frenet_crood_time = clock();

  KDPath kd_path(std::move(path_points_test));

  end_consturct_kd_frenet_crood_time = clock();
  std::cout << "\n KD consturct time: "
            << (double)(end_consturct_kd_frenet_crood_time -
                        start_consturct_kd_frenet_crood_time) /
                   CLOCKS_PER_SEC
            << std::endl;

  FrenetCoordinateSystemParameters frenet_parameters{0.1, 0.01, 0.3,
                                                     1.0, 0.5,  15};
  std::vector<Point2D> frenet_coord_points;
  frenet_coord_points.resize(ground_truth_points_circle.size());
  for (auto i = 0; i < frenet_coord_points.size(); i++) {
    frenet_coord_points[i] = Point2D{ground_truth_points_circle[i].x,
                                     ground_truth_points_circle[i].y};
  }

  // log consturct frenet crood system time
  clock_t start_consturct_frenet_crood_time, end_consturct_frenet_crood_time;
  start_consturct_frenet_crood_time = clock();

  FrenetCoordinateSystem frenet_coord_system(frenet_coord_points,
                                             frenet_parameters);

  end_consturct_frenet_crood_time = clock();
  std::cout << "\n consturct frenet crood system time: "
            << (double)(end_consturct_frenet_crood_time -
                        start_consturct_frenet_crood_time) /
                   CLOCKS_PER_SEC
            << std::endl;

  int loop_num = 100;
  // log transform refpath points (xy -> sl) time
  clock_t start_transform_refpath_xy2sl_time, end_transform_refpath_xy2sl_time;
  start_transform_refpath_xy2sl_time = clock();

  for (auto iter = 0; iter < loop_num; iter++) {
    for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
      double s;
      double l;
      kd_path.XYToSL(ground_truth_points_circle[i].x,
                     ground_truth_points_circle[i].y, &s, &l);
    }
  }

  end_transform_refpath_xy2sl_time = clock();
  std::cout << "\n KD transform refpath points (xy -> sl) time: "
            << (double)(end_transform_refpath_xy2sl_time -
                        start_transform_refpath_xy2sl_time) /
                   CLOCKS_PER_SEC
            << std::endl;

  // log frenet transform points (xy -> sl) time
  start_transform_refpath_xy2sl_time = clock();
  for (auto iter = 0; iter < loop_num; iter++) {
    for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
      Point2D temp_cart_point{ground_truth_points_circle[i].x,
                              ground_truth_points_circle[i].y};
      Point2D temp_frenet_point;
      frenet_coord_system.CartCoord2FrenetCoord(temp_cart_point,
                                                temp_frenet_point);
    }
  }
  end_transform_refpath_xy2sl_time = clock();
  std::cout << "\n  frenet system transform (xy -> sl) time: "
            << (double)(end_transform_refpath_xy2sl_time -
                        start_transform_refpath_xy2sl_time) /
                   CLOCKS_PER_SEC
            << std::endl;

  // log transform refpath points (sl -> xy) time
  clock_t start_transform_refpath_sl2xy_time, end_transform_refpath_sl2xy_time;
  start_transform_refpath_sl2xy_time = clock();

  for (auto iter = 0; iter < loop_num; iter++) {
    for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
      double x;
      double y;
      kd_path.SLToXY(ground_truth_points_circle[i].s,
                     ground_truth_points_circle[i].l, &x, &y);
    }
  }

  end_transform_refpath_sl2xy_time = clock();
  std::cout << "\n KD transform refpath points (sl -> xy) time: "
            << (double)(end_transform_refpath_sl2xy_time -
                        start_transform_refpath_sl2xy_time) /
                   CLOCKS_PER_SEC
            << std::endl;

  // log frenet transform points (sl -> xy) time
  start_transform_refpath_sl2xy_time = clock();
  for (auto iter = 0; iter < loop_num; iter++) {
    for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
      Point2D temp_frenet_point{ground_truth_points_circle[i].s,
                                ground_truth_points_circle[i].l};
      Point2D temp_cart_point;
      frenet_coord_system.FrenetCoord2CartCoord(temp_frenet_point,
                                                temp_cart_point);
    }
  }

  end_transform_refpath_sl2xy_time = clock();
  std::cout << "\n  frenet system transform (sl -> xy) time: "
            << (double)(end_transform_refpath_sl2xy_time -
                        start_transform_refpath_sl2xy_time) /
                   CLOCKS_PER_SEC
            << std::endl;

  // log transform refpath points (xy -> sl) accuracy
  std::ofstream xy2sl_kd_errors("xy2sl_kd_errors.csv");
  xy2sl_kd_errors << "s error(%)"
                  << ","
                  << "l error(%)" << std::endl;
  for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
    double s;
    double l;
    kd_path.XYToSL(ground_truth_points_circle[i].x,
                   ground_truth_points_circle[i].y, &s, &l);
    double kd_relative_error_s =
        std::fabs(s / ground_truth_points_circle[i].s - 1) * 100;
    double kd_relative_error_l =
        std::fabs(l / ground_truth_points_circle[i].l - 1) * 100;
    // kd_relative_errors_s.emplace_back(kd_relative_error_s);
    // kd_relative_errors_l.emplace_back(kd_relative_error_l);
    xy2sl_kd_errors << kd_relative_error_s << "," << kd_relative_error_l
                    << std::endl;
  }
  xy2sl_kd_errors.close();

  // log transform refpath points (sl -> xy) accuracy
  std::ofstream sl2xy_kd_errors("sl2xy_kd_errors.csv");
  sl2xy_kd_errors << "x error(%)"
                  << ","
                  << "y error(%)" << std::endl;
  for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
    double x;
    double y;
    kd_path.SLToXY(ground_truth_points_circle[i].s,
                   ground_truth_points_circle[i].l, &x, &y);
    double kd_relative_error_x =
        std::fabs(x / ground_truth_points_circle[i].x - 1) * 100;
    double kd_relative_error_y =
        std::fabs(y / ground_truth_points_circle[i].y - 1) * 100;
    sl2xy_kd_errors << kd_relative_error_x << "," << kd_relative_error_y
                    << std::endl;
  }
  sl2xy_kd_errors.close();

  // log frenet transform points (xy -> sl) accuracy
  // std::ofstream xy2sl_frenet_errors("xy2sl_frenet_errors.csv");
  // xy2sl_frenet_errors << "s error(%)"
  //                     << ","
  //                     << "l error(%)" << std::endl;
  // for (auto i = 0; i < ground_truth_points_circle.size(); i++) {
  //   Point2D temp_cart_point{ground_truth_points_circle[i].x,
  //                           ground_truth_points_circle[i].y};
  //   Point2D temp_frenet_point;
  //   frenet_coord_system.CartCoord2FrenetCoord(temp_cart_point,
  //                                             temp_frenet_point);
  //   double ft_relative_error_s =
  //       std::fabs(temp_frenet_point.x / ground_truth_points_circle[i].s - 1)
  //       * 100;
  //   double ft_relative_error_l =
  //       std::fabs(temp_frenet_point.y / ground_truth_points_circle[i].l - 1)
  //       * 100;

  //   xy2sl_frenet_errors << ft_relative_error_s << "," << ft_relative_error_l
  //                       << std::endl;
  // }
};

}  // namespace planning_math
}  // namespace planning