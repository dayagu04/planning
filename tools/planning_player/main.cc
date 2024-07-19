#include <getopt.h>

#include <chrono>
#include <string>

// #include "modules_register.h"
#include "planning_player.h"

int run_planning_player(const std::string &bag_path, const std::string &out_bag,
                        bool is_close_loop, double auto_time_sec,
                        const std::string &scene_type,
                        const std::string mileage_path, bool no_debug,
                        bool no_interface_check) {
  planning::planning_player::PlanningPlayer player;

  if (!player.LoadRosBag(bag_path, out_bag, is_close_loop, no_debug,
                         no_interface_check)) {
    return -1;
  }
  player.Init(is_close_loop, auto_time_sec, scene_type, no_debug);
  if (no_debug) {
    player.NoDebugInfoMode(is_close_loop);
  } else {
    player.PlayAllFrames(is_close_loop);
  }
  player.GenMileage(mileage_path);
  player.StoreRosBag(out_bag);
  return 0;
}

int main(int argc, char **argv) {
  std::string bag_path, out_bag, log_file;
  bool is_close_loop = false;
  bool no_debug = false;
  bool no_interface_check = false;
  std::string mileage_path = "";
  double auto_time_sec = 1.5;
  std::string scene_type = "scc";

  int opt, lopt, loidx;
  const char *optstring = "";
  const struct option long_options[] = {
      {"help", no_argument, &lopt, 1},
      {"play", required_argument, &lopt, 2},
      {"close-loop", no_argument, &lopt, 3},
      {"out-bag", required_argument, &lopt, 4},
      {"auto-time", required_argument, &lopt, 5},
      {"scene-type", required_argument, &lopt, 6},
      {"mileage-path", required_argument, &lopt, 7},
      {"no-debug", no_argument, &lopt, 8},
      {"no-interface-check", no_argument, &lopt, 9}};

  while ((opt = getopt_long(argc, argv, optstring, long_options, &loidx)) !=
         -1) {
    if (opt == 0) opt = lopt;
    switch (opt) {
      case 1:
        std::cout << "--help             print this message" << std::endl;
        std::cout << "--play [bag path]  origin bag path, required"
                  << std::endl;
        std::cout << "--close-loop       run close loop, "
                     "default false"
                  << std::endl;
        std::cout << "--out-bag          generated bag path, default [bag "
                     "path].[timestamp].plan"
                  << std::endl;
        std::cout << "--log-file         generated log path, default [bag "
                     "paht].[timestamp].log"
                  << std::endl;
        std::cout << "--auto-time        time when enter auto, default 1.5"
                  << std::endl;
        std::cout << "--scene-type       acc/apa" << std::endl;
        std::cout << "--no-debug         play without planning debug info"
                  << std::endl;
        std::cout << "--no-interface-check   do not check interface version"
                  << std::endl;
        break;
      case 2:
        bag_path = std::string(optarg);
        if (bag_path.empty()) {
          std::cerr << "empty bag path" << std::endl;
          return -1;
        }
        break;
      case 3:
        is_close_loop = true;
        break;
      case 4:
        out_bag = std::string(optarg);
        break;
      case 5:
        auto_time_sec = std::stod(optarg);
        break;
      case 6:
        scene_type = std::string(optarg);
        break;
      case 7:
        mileage_path = std::string(optarg);
        break;
      case 8:
        no_debug = true;
        break;
      case 9:
        no_interface_check = true;
        break;
      default:
        std::cerr << "unknown option " << opt << std::endl;
        return -1;
    }
  }

  auto tp = std::chrono::time_point_cast<std::chrono::seconds>(
                std::chrono::system_clock::now())
                .time_since_epoch()
                .count();
  if (out_bag.empty()) {
    if (is_close_loop) {
      out_bag = bag_path + "." + std::to_string(tp) + ".close-loop.plan";
    } else {
      out_bag = bag_path + "." + std::to_string(tp) + ".open-loop.plan";
    }
  }

  return run_planning_player(bag_path, out_bag, is_close_loop, auto_time_sec,
                             scene_type, mileage_path, no_debug,
                             no_interface_check);
}
