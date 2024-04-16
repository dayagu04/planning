#!/usr/bin/env python3

"""
在主函数中填入两个包的路径(bag1_path, bag2_path), 程序将提取两个包中实时规划的结果(v, a, poly), 并对比同时刻下两个包结果的差距
"""

import sys
import datetime
import pandas as pd
from cyber_record.record import Record

def dump_realtime_planning(bag_path, bag_trajectory_points, input_frame_num):
    try:
        freader = Record(bag_path)
    except Exception:
        print('Cannot open record file %s' % bag_path)
    else:
        frame_num = 0
        for topic_name, msg, t in freader.read_messages("/iflytek/planning/plan"):
            # 舍弃前n帧，因为刚进自驾，可能存在误差
            if frame_num < input_frame_num:
                frame_num+=1
                continue

            index = msg.meta.header.timestamp
            bag_trajectory_points[index] = []
            bag_trajectory_points[index].append(msg.trajectory.target_reference.target_velocity)
            bag_trajectory_points[index].append(msg.trajectory.target_reference.acceleration_range_limit.min_a)
            bag_trajectory_points[index].append(msg.trajectory.target_reference.acceleration_range_limit.max_a)
            bag_trajectory_points[index].append(msg.trajectory.target_reference.polynomial[0])
            bag_trajectory_points[index].append(msg.trajectory.target_reference.polynomial[1])
            bag_trajectory_points[index].append(msg.trajectory.target_reference.polynomial[2])
            bag_trajectory_points[index].append(msg.trajectory.target_reference.polynomial[3])


if __name__ == '__main__':
    bag1_path = "/data_cold/abu_zone/hpp/test/scc_data/394314.record"
    bag2_path = "/data_cold/abu_zone/hpp/test/scc_data/394314.record.1706238939.plan"
    bag1_trajectory_points = {}
    bag2_trajectory_points = {}
    error = {}
    average_error = {}
    drop_frame_num = 40
    dump_realtime_planning(bag1_path, bag1_trajectory_points, drop_frame_num)
    dump_realtime_planning(bag2_path, bag2_trajectory_points, drop_frame_num)
    print(len(bag1_trajectory_points), len(bag2_trajectory_points))

    list_error_v, list_error_min_a, list_error_max_a, list_error_polynomial_1, list_error_polynomial_2, list_error_polynomial_3, list_error_polynomial_4 = [], [], [], [], [], [], []
    for key, value in bag1_trajectory_points.items():
        if key in bag2_trajectory_points.keys():
            error_v = abs(value[0] - bag2_trajectory_points[key][0])
            error_min_a = abs(value[1] - bag2_trajectory_points[key][1])
            error_max_a = abs(value[2] - bag2_trajectory_points[key][2])
            error_polynomial_1 = abs(value[3] - bag2_trajectory_points[key][3])
            error_polynomial_2 = abs(value[4] - bag2_trajectory_points[key][4])
            error_polynomial_3 = abs(value[5] - bag2_trajectory_points[key][5])
            error_polynomial_4 = abs(value[6] - bag2_trajectory_points[key][6])
            error[key] = [error_v, error_min_a, error_max_a, error_polynomial_1, error_polynomial_2, error_polynomial_3, error_polynomial_4]

            list_error_v.append(error_v)
            list_error_min_a.append(error_min_a)
            list_error_max_a.append(error_max_a)
            list_error_polynomial_1.append(error_polynomial_1)
            list_error_polynomial_2.append(error_polynomial_2)
            list_error_polynomial_3.append(error_polynomial_3)
            list_error_polynomial_4.append(error_polynomial_4)

    average_error["速度差-平均值"] = sum(list_error_v) / len(list_error_v)
    average_error["速度差-最大值"] = max(list_error_v)
    average_error["速度差-最小值"] = min(list_error_v)
    average_error["加速度最小值差-平均值"] = sum(list_error_min_a) / len(list_error_min_a)
    average_error["加速度最小值差-最大值"] = max(list_error_min_a)
    average_error["加速度最小值差-最小值"] = min(list_error_min_a)
    average_error["加速度最大值差-平均值"] = sum(list_error_max_a) / len(list_error_max_a)
    average_error["加速度最大值差-最大值"] = max(list_error_max_a)
    average_error["加速度最大值差-最小值"] = min(list_error_max_a)
    average_error["多项式1差-平均值"] = sum(list_error_polynomial_1) / len(list_error_polynomial_1)
    average_error["多项式1差-最大值"] = max(list_error_polynomial_1)
    average_error["多项式1差-最小值"] = min(list_error_polynomial_1)
    average_error["多项式2差-平均值"] = sum(list_error_polynomial_2) / len(list_error_polynomial_2)
    average_error["多项式2差-最大值"] = max(list_error_polynomial_2)
    average_error["多项式2差-最小值"] = min(list_error_polynomial_2)
    average_error["多项式3差-平均值"] = sum(list_error_polynomial_3) / len(list_error_polynomial_3)
    average_error["多项式3差-最大值"] = max(list_error_polynomial_3)
    average_error["多项式3差-最小值"] = min(list_error_polynomial_3)
    average_error["多项式4差-平均值"] = sum(list_error_polynomial_4) / len(list_error_polynomial_4)
    average_error["多项式4差-最大值"] = max(list_error_polynomial_4)
    average_error["多项式4差-最小值"] = min(list_error_polynomial_4)

    now = datetime.datetime.now()
    formatted_date = now.strftime("%Y-%m-%d-%H-%M-%S")
    writer = pd.ExcelWriter('./dump_realtime_planning_' + str(formatted_date) + '.xlsx')
    bag1_data = pd.DataFrame(bag1_trajectory_points, index=["v", "min_a", "max_a", "ploy_1", "ploy_2", "ploy_3", "ploy_4"])
    bag2_data = pd.DataFrame(bag2_trajectory_points, index=["v", "min_a", "max_a", "ploy_1", "ploy_2", "ploy_3", "ploy_4"])
    error_data = pd.DataFrame(error, index=["v", "min_a", "max_a", "ploy_1", "ploy_2", "ploy_3", "ploy_4"])
    average_error_data = pd.DataFrame(average_error, index=[0]).T
    bag1_data.to_excel(writer, sheet_name="bag1_data")
    bag2_data.to_excel(writer, sheet_name="bag2_data")
    error_data.to_excel(writer, sheet_name="error_data")
    average_error_data.to_excel(writer, sheet_name="average_error_data")

    writer.save()