#!/usr/bin/env python3

"""
在主函数中填入两个包的路径(bag1_path, bag2_path), 并选择要使用的定位topic(默认/iflytek/localization/ego_pose)
程序将提取两个包中定位的结果, 并对比同时刻下bag1与bag2之间结果的差距
"""

# from scipy.spatial.distance import euclidean
# from fastdtw import fastdtw
import sys
import datetime
import numpy as np
import pandas as pd
from cyber_record.record import Record
import math

TOPIC_LOCALIZATION = "/iflytek/localization/egomotion"
TOPIC_LOCALIZATION_ESTIMATE = "/iflytek/localization/ego_pose"

def dump_localization_points(bag_path, bag_loc_point, topic_name):
    try:
        freader = Record(bag_path)
    except Exception:
        print('Cannot open record file %s' % bag_path)
    else:
        bag_loc_point["x"] = []
        bag_loc_point["y"] = []
        for topic_name, msg, t in freader.read_messages(topic_name):
            if topic_name == TOPIC_LOCALIZATION:
                bag_loc_point["x"].append(msg.position.position_boot.x)
                bag_loc_point["y"].append(msg.position.position_boot.y)
            elif topic_name == TOPIC_LOCALIZATION_ESTIMATE:
                bag_loc_point["x"].append(msg.pose.local_position.x)
                bag_loc_point["y"].append(msg.pose.local_position.y)
            else:
                print("unkown localization topic")


if __name__ == '__main__':

    bag1_path = "/data_cold/abu_zone/hpp/test/check_0226/lane_keeping_split.record"
    bag2_path = "/data_cold/abu_zone/hpp/test/check_0226/lane_keeping_split.record.1709088015.plan"

    bag1_loc_points = {}
    bag2_loc_points = {}

    localization_topic = TOPIC_LOCALIZATION_ESTIMATE
    dump_localization_points(bag1_path, bag1_loc_points, localization_topic)
    dump_localization_points(bag2_path, bag2_loc_points, localization_topic)

    dis_error = {} 
    dis_error["Bag_2"] = []
    for i in range(len(bag1_loc_points["x"])):
        dis_error["Bag_2"].append(math.hypot((bag1_loc_points["x"][i] - bag2_loc_points["x"][i]), 
                                        (bag1_loc_points["y"][i] - bag2_loc_points["y"][i])))

    result = {}
    result["Bag_2_平均值"] = sum(dis_error["Bag_2"]) / len(dis_error["Bag_2"])
    result["Bag_2_最大值"] = max(dis_error["Bag_2"])
    result["Bag_2_最小值"] = min(dis_error["Bag_2"])

    x = np.array([[bag1_loc_points["x"][0], bag1_loc_points["y"][0]]])
    y1 = np.array([[bag2_loc_points["x"][0], bag2_loc_points["y"][0]]])

    # print(bag1_loc_points["x"][0])
    for i in range(1, len(bag1_loc_points["x"])):
        x = np.append(x, np.array([[bag1_loc_points["x"][i], bag1_loc_points["y"][i]]]), 0)
        y1 = np.append(y1, np.array([[bag2_loc_points["x"][i], bag2_loc_points["y"][i]]]), 0)


    # distance1, path1 = fastdtw(x, y1, dist=euclidean)
    # result["Bag_2_DTW距离"] = distance1/len(bag1_loc_points["x"])
    # print("Bag_2DTW距离 ", distance1/len(bag1_loc_points["x"]))

    now = datetime.datetime.now()
    formatted_date = now.strftime("%Y-%m-%d-%H-%M-%S")
    writer = pd.ExcelWriter('./定位点距离差_' + str(formatted_date) + '.xlsx')

    bag1_data = pd.DataFrame(bag1_loc_points)
    bag2_data = pd.DataFrame(bag2_loc_points)
    error_data = pd.DataFrame(dis_error)
    result_data = pd.DataFrame(result, index=[0]).T
    bag1_data.to_excel(writer, sheet_name="Bag_1")
    bag2_data.to_excel(writer, sheet_name="Bag_2")
    error_data.to_excel(writer, sheet_name="距离差")
    result_data.to_excel(writer, sheet_name="计算结果")
    writer.save()

