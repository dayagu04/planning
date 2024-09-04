
#pragma once

#include <iostream>
#include <memory>

#include "debug_mode.h"
#include "display_config.h"
#include "opencv_viz.h"
#include "system_state.h"

namespace planning {

void main_window2d_init(viz2d_color background_color);

void hmap_window2d_init();

int viz2d_window_read_config(std::string config_dir);

int viz2d_window_read_config();

DisplayConfig *get_windows2d_config();

viz2d_image *get_main_window2d();

viz2d_image *get_hmap_window2d();

viz2d_image *get_control_window2d();

void control_viz2d_init();

int viz2d_draw_system_state(viz2d_image *viz2d, const SystemState &states,
                            const Pose2D *base_pose);



// use radian
int viz2d_draw_front_wheel_state(viz2d_image *viz2d, const Pose2D *veh_pose,
                                 double wheel_base, double steering,
                                 viz2d_color text_color_index);

int viz2d_draw_pause_state(viz2d_image *viz2d, const RunningTimeDebug &debug);

int viz2d_draw_run_mode(viz2d_image *viz2d, const ApolloRunMode &mode);

int viz2d_draw_localization_time(viz2d_image *viz2d, double time_stamp);

int viz2d_draw_replay_info(viz2d_image *viz2d, int64_t ratio);

}  // namespace planning
