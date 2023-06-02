#!/usr/bin/python
# encoding=utf-8

from plot_planning_info import PlotPlanningInfo

if __name__ == '__main__':
    data_path = "" # change to log file path
    plot_all_frame = PlotPlanningInfo(data_path)
    plot_all_frame.read_file()
    plot_all_frame.plot_pos_data()
    plot_all_frame.plot_time_diff()
    # plot_all_frame.plot_smoothed_speed()