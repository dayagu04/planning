#include "planning_gflags.h"

// scenario related
DEFINE_string(vehicle_param_config_file,
    "/asw/planning/res/conf/vehicle_param_aionlx.pb.txt", "");
DEFINE_int32(planning_loop_rate,
    10, "Loop rate for planning");