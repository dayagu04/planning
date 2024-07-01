#!/usr/bin/env python3

"""
在主函数中填入两个包的路径(bag1_path, bag2_path), 程序将提取两个包中长时规划的结果(201个轨迹点坐标), 并对比同时刻下两个包结果的差距
"""

import sys
import os
import datetime
import pandas as pd
import rosbag
import rospy

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, '../jupyter_pybind/python_proto/'))
from planning_debug_info_pb2 import *

def dump_trajectory_points(bag_path, bag_trajectory_points, input_frame_num):
    try:
        bag = rosbag.Bag(bag_path)
    except Exception:
        print('Cannot open bag file %s' % bag_path)
    else:
        index_map = {}
        first_flag = True
        for topic_name, msg, t in bag.read_messages("/iflytek/planning/debug_info"):
            planning_debug_output = PlanningDebugInfo()
            planning_debug_output.ParseFromString(msg.debug_info)
            if first_flag:
                start_index = planning_debug_output.frame_info.frame_num
                first_flag = False
            index_map[planning_debug_output.timestamp] = planning_debug_output.frame_info.frame_num

        frame_num = 0
        for topic_name, msg, t in bag.read_messages("/iflytek/planning/plan"):
            # 舍弃前几帧，因为刚进自驾，可能存在误差
            if frame_num < input_frame_num:
                frame_num+=1
                continue
            if len(msg.trajectory.trajectory_points) == 201:
                index = index_map[msg.meta.msg_header.timestamp] - start_index
                bag_trajectory_points["frame_" + str(index) + "_x"] = []
                bag_trajectory_points["frame_" + str(index) + "_y"] = []
                for i in range(len(msg.trajectory.trajectory_points)):
                    bag_trajectory_points["frame_" + str(index) + "_x"].append(msg.trajectory.trajectory_points[i].x)
                    bag_trajectory_points["frame_" + str(index) + "_y"].append(msg.trajectory.trajectory_points[i].y)

if __name__ == '__main__':
    bag1_path = "/data_cold/autoupload/jac_s811_35kw2/trigger/20240520/20240520-14-35-53/split.bag"
    bag2_path = "/data_cold/autoupload/jac_s811_35kw2/trigger/20240520/20240520-14-35-53/split.bag.1716262964.plan"
    bag1_trajectory_points = {}
    bag2_trajectory_points = {}
    average_error = {}
    drop_frame_num = 30
    dump_trajectory_points(bag1_path, bag1_trajectory_points, drop_frame_num)
    dump_trajectory_points(bag2_path, bag2_trajectory_points, drop_frame_num)
    # print(len(bag1_trajectory_points), len(bag2_trajectory_points))

    for key, value in bag1_trajectory_points.items():
        if "_x" in key:
            if key in bag2_trajectory_points.keys():
                error = 0
                for i, e in enumerate(value):
                    error_x = (e - bag2_trajectory_points[key][i])**2
                    key_y = key[:-1] + 'y'
                    error_y = (bag1_trajectory_points[key_y][i] - bag2_trajectory_points[key_y][i])**2
                    error += (error_x + error_y)**0.5
                average_error[key[:-2]] = error / len(value)

    total_error = 0
    for key, value in average_error.items():
        total_error += value
    average_error["平均值"] = sum(average_error.values()) / len(average_error)
    average_error["最大值"] = max(average_error.values())
    average_error["最小值"] = min(average_error.values())

    now = datetime.datetime.now()
    formatted_date = now.strftime("%Y-%m-%d-%H-%M-%S")
    writer = pd.ExcelWriter('./dump_trajectory_points_' + str(formatted_date) + '.xlsx')
    bag1_data = pd.DataFrame(bag1_trajectory_points)
    bag2_data = pd.DataFrame(bag2_trajectory_points)
    error_data = pd.DataFrame(average_error, index=[0]).T
    bag1_data.to_excel(writer, sheet_name="bag1_data")
    bag2_data.to_excel(writer, sheet_name="bag2_data")
    error_data.to_excel(writer, sheet_name="error_data")
    writer.save()