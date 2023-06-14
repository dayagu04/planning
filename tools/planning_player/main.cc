#include <getopt.h>

#include <chrono>
#include <string>

#include "modules_register.h"
#include "planning_player.h"

int run_open_loop(const std::string &bag_path, const std::string &out_bag) {
  planning::planning_player::PlanningPlayer player;
  player.Init();

  player.Clear();
  if (!player.LoadCyberBag(bag_path)) {
    return -1;
  }
  player.PlayAllFrames();
  player.StoreCyberBag(out_bag);
  return 0;
}

int main(int argc, char **argv) {
  std::string bag_path, out_bag, log_file;
  bool is_close_loop = false;

  int opt, lopt, loidx;
  const char *optstring = "";
  const struct option long_options[] = {{"help", no_argument, &lopt, 1},
                                        {"play", required_argument, &lopt, 2},
                                        {"close-loop", optional_argument, &lopt, 3},
                                        {"out-bag", optional_argument, &lopt, 4}};

  while ((opt = getopt_long(argc, argv, optstring, long_options, &loidx)) != -1) {
    if (opt == 0) opt = lopt;
    switch (opt) {
      case 1:
        std::cout << "--help             print this message" << std::endl;
        std::cout << "--play [bag path]  origin bag path, required" << std::endl;
        std::cout << "--close-loop       run close loop(not implemented), default false" << std::endl;
        std::cout << "--out-bag          generated bag path, default [bag path].[timestamp].plan" << std::endl;
        std::cout << "--log-file         generated log path, default [bag paht].[timestamp].log" << std::endl;
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
      default:
        std::cerr << "unknown option " << opt << std::endl;
        return -1;
    }
  }

  auto tp =
      std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
  if (out_bag.empty()) {
    out_bag = bag_path + "." + std::to_string(tp) + ".plan";
  }

  if (!is_close_loop) {
    return run_open_loop(bag_path, out_bag);
  } else {
    std::cerr << "closed loop not implemented" << std::endl;
    return -1;
  }
}
