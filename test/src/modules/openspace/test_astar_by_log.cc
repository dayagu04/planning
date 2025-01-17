
#include "./matplotlibcpp.h"
#include "ad_common/math/vec2d.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/common/planning_gflags.h"
#include "src/library/hybrid_astar_lib/hybrid_a_star.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_config.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_request.h"
#include "src/library/hybrid_astar_lib/node3d.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "src/library/reeds_shepp/reeds_shepp_interface.h"
#include "src/modules/common/utils/file.h"
#include "src/modules/context/vehicle_config_context.h"

// 导入window ip地址,ipconfig可以查询
// export DISPLAY=10.5.166.104:0.0
// echo $DISPLAY
// apt-get install xarclock
// xarclock 可以查询是否生效

using namespace ad_common::math;
namespace plt = matplotlibcpp;

void plot_astar_result(const planning::HybridAStarResult &coarse_traj,
                       const Eigen::MatrixXd &smooth_traj,
                       const std::vector<planning::Polygon2D> &obs_list,
                       const planning::VehicleParam &vehicle) {
  // plt::figure(1);
  // plt::figure_size(1200, 1200);

  // obstacle
  for (int i = 0; i < obs_list.size(); ++i) {
    const auto &obs = obs_list.at(i);

    std::vector<double> obs_x;
    std::vector<double> obs_y;

    for (int i = 0; i < obs.vertex_num; i++) {
      obs_x.push_back(obs.vertexes[i].x);
      obs_y.push_back(obs.vertexes[i].y);
    }

    obs_x.push_back(obs.vertexes[0].x);
    obs_y.push_back(obs.vertexes[0].y);

    plt::plot(obs_x, obs_y, "black");
  }

  // path
  std::vector<double> coarse_x;
  std::vector<double> coarse_y;

  for (size_t i(0); i < coarse_traj.x.size(); ++i) {
    coarse_x.push_back(coarse_traj.x[i]);
    coarse_y.push_back(coarse_traj.y[i]);

    auto box = planning::Node3d::GetBoundingBox(
        vehicle, coarse_traj.x[i], coarse_traj.y[i], coarse_traj.phi[i]);

    std::vector<double> corner_x;
    std::vector<double> corner_y;

    for (const auto &corner : box.GetAllCorners()) {
      corner_x.push_back(corner.x());
      corner_y.push_back(corner.y());
    }

    corner_x.push_back(corner_x.front());
    corner_y.push_back(corner_y.front());

    plt::plot(corner_x, corner_y, "green");
  }

  plt::plot(coarse_traj.x, coarse_traj.y, "blue");

  plt::axis("equal");
  plt::show();
}

int main(int argc, char *argv[]) {
  FilePath::SetName("test_astar_log");

  FLAGS_log_dir = "/asw/planning/log";

  // Init glog
  InitGlog(FilePath::GetName().c_str());

  // test apollo cyber log api
  // ILOG_INFO << "log init finish";
  ILOG_ERROR << "log init finish";
  ILOG_WARN << "log init finish";
  ILOG_DEBUG << "log init finish";

  // test google log api
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream()
      << "LEFT_BRACKET << module << RIGHT_BRACKET";

  LOG(INFO) << "Hello,GOOGLE!";

  // test astar
  // read configurature from file

  planning::PlannerOpenSpaceConfig config;
  config.InitConfig();

  // read vehicle param from file
  planning::VehicleParam vehicle_param;

  planning::VehicleConfigurationContext::Instance()->set_vehicle_param(
      vehicle_param);

  // planning::apa_planner::ApaPlanInterfaceparking_interface;
  // parking_interface.Init();

  // planning::apa_planner::ApaParameters parking_param;
  // parking_param = apa_param.GetParam();

  // ILOG_INFO << "l " << vehicle_param.length;
  // ILOG_INFO << "w " << vehicle_param.width;
  // ILOG_INFO << "base " << vehicle_param.wheel_base;

  /// set scene:
  planning::Polygon2D polygon;
  std::vector<planning::Polygon2D> obs_list;

  std::vector<Vec2d> obs1 = {
      ad_common::math::Vec2d(13, 8), ad_common::math::Vec2d(1.5, 8),
      ad_common::math::Vec2d(1.5, 0), ad_common::math::Vec2d(13, 0)};

  GeneratePolygonByPoints(&polygon, obs1);
  obs_list.emplace_back(polygon);

  std::vector<Vec2d> obs2 = {
      ad_common::math::Vec2d(-1.5, 8), ad_common::math::Vec2d(-13, 8),
      ad_common::math::Vec2d(-13, 0), ad_common::math::Vec2d(-1.5, 0)};

  GeneratePolygonByPoints(&polygon, obs2);
  obs_list.emplace_back(polygon);

  std::vector<Vec2d> obs3 = {
      ad_common::math::Vec2d(1.5, 2), ad_common::math::Vec2d(-1.5, 2),
      ad_common::math::Vec2d(-1.5, 0), ad_common::math::Vec2d(1.5, 0)};

  GeneratePolygonByPoints(&polygon, obs3);
  obs_list.emplace_back(polygon);

  planning::ParkObstacleList park_obs;

  planning::MapBound map_bounds = {-10, 10, -20, 20};

  /// Hybrid Astar search a coarse path:
  planning::HybridAStar hybrid_astar(config, vehicle_param);
  hybrid_astar.Init();
  planning::HybridAStarResult coarse_traj;

  // y
  // ^
  // |
  // |
  // |
  // ----> x
  Pose2D initial_state = Pose2D(-4, 10, 0);
  Pose2D goal_state = Pose2D(0, 3.5, M_PI_2);

  planning::AstarRequest request;

  hybrid_astar.Plan(initial_state, goal_state, map_bounds, park_obs, request,
                    &coarse_traj);

  Eigen::MatrixXd state;

  // plot

  ap_traj_set_t traj_set;

  bool is_connected_to_goal;

  Pose2D start = initial_state;
  Pose2D end = goal_state;
  GeneShortestRSPath(&traj_set, &is_connected_to_goal, &start, &end, 5.0, true, true);

  coarse_traj.x.clear();
  coarse_traj.y.clear();
  coarse_traj.phi.clear();
  for (int i = 0; i < traj_set.size; i++) {
    ap_traj_t *traj = &traj_set.trajs[i];

    for (int j = 0; j < traj->size; j++) {
      coarse_traj.x.emplace_back(traj->points[j].x);
      coarse_traj.y.emplace_back(traj->points[j].y);
      coarse_traj.phi.emplace_back(traj->points[j].theta);
    }
  }

  plot_astar_result(coarse_traj, state, obs_list, vehicle_param);

  // stop
  // async_logger->Stop();
  // google::ShutdownGoogleLogging();

  return 0;
}