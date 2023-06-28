#!/usr/bin/python
# encoding=utf-8

import matplotlib.pyplot as plt
import math

import numpy as np


class PlotPlanningInfo(object):
    def __init__(self, data_path):
        self.data_path = data_path
        self.init_time = None
        self.time_list = []
        self.all_time_diff_list = []
        self.rel_time_list = []
        self.pre_time = None
        self.x_ego_list = []
        self.y_ego_list = []
        self.theta_ego_list = []
        self.x_traj_list = []
        self.y_traj_list = []
        self.theta_traj_list = []
        self.smooth_v_list = []
        self.smoothed_s_list = []
        self.v_limit_list = []
        self.gear_list = []
        self.veh_spd_mps_list = []
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.raw_slot0_x = None
        self.raw_slot0_y = None
        self.raw_slot1_x = None
        self.raw_slot1_y = None
        self.raw_slot2_x = None
        self.raw_slot2_y = None
        self.raw_slot3_x = None
        self.raw_slot3_y = None
        self.raw_slot0_x_list = []
        self.raw_slot0_y_list = []
        self.raw_slot1_x_list = []
        self.raw_slot1_y_list = []
        self.raw_slot2_x_list = []
        self.raw_slot2_y_list = []
        self.raw_slot3_x_list = []
        self.raw_slot3_y_list = []
        self.slot0_x = None
        self.slot0_y = None
        self.slot1_x = None
        self.slot1_y = None
        self.slot2_x = None
        self.slot2_y = None
        self.slot3_x = None
        self.slot3_y = None
        self.slot0_x_list = []
        self.slot0_y_list = []
        self.slot1_x_list = []
        self.slot1_y_list = []
        self.slot2_x_list = []
        self.slot2_y_list = []
        self.slot3_x_list = []
        self.slot3_y_list = []
        self.mocked_obj_x0 = None
        self.mocked_obj_y0 = None
        self.mocked_obj_x1 = None
        self.mocked_obj_y1 = None

    def get_values(self, line):
        if line.startswith("current time(ms)"):
            strs = line.split(":")
            t = int(strs[1])
            if self.init_time is None:
                self.init_time = t
            rel_time = (t - self.init_time) * 0.001
            self.rel_time_list.append(rel_time)
            if self.pre_time is None:
                self.all_time_diff_list.append(0)
            else:
                self.all_time_diff_list.append(t - self.pre_time)
            self.pre_time = t
        elif line.startswith("cur_pos_in_odom_ x"):
            strs1 = line.split(",")
            strs2 = strs1[0].split(":")
            self.x_ego_list.append(float(strs2[1]))
            strs2 = strs1[1].split(":")
            self.y_ego_list.append(float(strs2[1]))
            strs2 = strs1[2].split(":")
            self.theta_ego_list.append(float(strs2[1]))
        elif line.startswith("gear"):
            strs = line.split(":")
            self.gear_list.append(int(strs[1]))
        elif line.startswith("veh_spd"):
            strs = line.split(":")
            self.veh_spd_mps_list.append(float(strs[1]))
        elif line.startswith("seg traj pt"):
            strs1 = line.split(",")
            strs2 = strs1[1].split(":")
            self.x_traj_list.append(float(strs2[1]))
            strs2 = strs1[2].split(":")
            self.y_traj_list.append(float(strs2[1]))
            strs2 = strs1[3].split(":")
            self.theta_traj_list.append(float(strs2[1]))
        elif line.startswith("cur_pos_in_odom_ x"):
            strs1 = line.split(",")
            strs2 = strs1[0].split(":")
            self.x_ego_list.append(float(strs2[1]))
            strs2 = strs1[1].split(":")
            self.y_ego_list.append(float(strs2[1]))
            strs2 = strs1[2].split(":")
            self.theta_ego_list.append(float(strs2[1]))
        elif line.startswith("target_point_in_odom_ x"):
            strs1 = line.split(",")
            strs2 = strs1[0].split(":")
            self.target_x = float(strs2[1])
            strs2 = strs1[1].split(":")
            self.target_y = float(strs2[1])
            strs2 = strs1[2].split(":")
            self.target_theta = float(strs2[1])
        elif line.startswith("mocked obj"):
            strs1 = line.split(",")
            strs2 = strs1[0].split(":")
            self.mocked_obj_x0 = float(strs2[1])
            strs2 = strs1[1].split(":")
            self.mocked_obj_y0 = float(strs2[1])
            strs2 = strs1[2].split(":")
            self.mocked_obj_x1 = float(strs2[1])
            strs2 = strs1[3].split(":")
            self.mocked_obj_y1 = float(strs2[1])
        elif line.startswith("raw slot_points_in_m_"):
            strs1 = line.split(",")
            strs2 = strs1[0].split(":")
            self.raw_slot0_x = float(strs2[1])
            strs2 = strs1[1].split(":")
            self.raw_slot0_y = float(strs2[1])
            strs2 = strs1[2].split(":")
            self.raw_slot1_x = float(strs2[1])
            strs2 = strs1[3].split(":")
            self.raw_slot1_y = float(strs2[1])
            strs2 = strs1[4].split(":")
            self.raw_slot2_x = float(strs2[1])
            strs2 = strs1[5].split(":")
            self.raw_slot2_y = float(strs2[1])
            strs2 = strs1[6].split(":")
            self.raw_slot3_x = float(strs2[1])
            strs2 = strs1[7].split(":")
            self.raw_slot3_y = float(strs2[1])
        elif line.startswith("slot_points_in_m_"):
            strs1 = line.split(",")
            strs2 = strs1[0].split(":")
            self.slot0_x = float(strs2[1])
            self.slot0_x_list.append(self.slot0_x)
            strs2 = strs1[1].split(":")
            self.slot0_y = float(strs2[1])
            self.slot0_y_list.append(self.slot0_y)
            strs2 = strs1[2].split(":")
            self.slot1_x = float(strs2[1])
            self.slot1_x_list.append(self.slot1_x)
            strs2 = strs1[3].split(":")
            self.slot1_y = float(strs2[1])
            self.slot1_y_list.append(self.slot1_y)
            strs2 = strs1[4].split(":")
            self.slot2_x = float(strs2[1])
            self.slot2_x_list.append(self.slot2_x)
            strs2 = strs1[5].split(":")
            self.slot2_y = float(strs2[1])
            self.slot2_y_list.append(self.slot2_y)
            strs2 = strs1[6].split(":")
            self.slot3_x = float(strs2[1])
            self.slot3_x_list.append(self.slot3_x)
            strs2 = strs1[7].split(":")
            self.slot3_y = float(strs2[1])
            self.slot3_y_list.append(self.slot3_y)
        elif line.startswith("smoothed traj pt"):
            strs1 = line.split(",")
            strs2 = strs1[5].split(":")
            self.smooth_v_list.append(float(strs2[1]))
            strs2 = strs1[6].split(":")
            self.v_limit_list.append(float(strs2[1]))
            strs2 = strs1[7].split(":")
            self.smoothed_s_list.append(float(strs2[1]))

    def read_file(self):
        file_object = open(self.data_path, 'r')
        lines = file_object.readlines()
        is_glog = lines[0].startswith("Log file created at")
        if is_glog:
            for line in lines[3:]:
                splited_line = line.split(']', 1)[1].strip()
                self.get_values(splited_line)
        else:
            for line in lines:
                self.get_values(line)
        file_object.close()

    def get_closed_veh_box(self, x, y, theta):
        # params for E40X
        # length = 4.41
        # width = 1.8
        # shift_dis = 1.325

        # params for AIONLX
        length = 4.786
        width = 1.935
        shift_dis = (3.846 - 0.94) * 0.5

        half_length = length * 0.5
        half_width = width * 0.5
        cos_ego_start_theta = math.cos(theta)
        sin_ego_start_theta = math.sin(theta)
        shift_ego_start_x = x + shift_dis * cos_ego_start_theta
        shift_ego_start_y = y + shift_dis * sin_ego_start_theta
        dx1 = cos_ego_start_theta * half_length
        dy1 = sin_ego_start_theta * half_length
        dx2 = sin_ego_start_theta * half_width
        dy2 = -cos_ego_start_theta * half_width
        pt0_x = shift_ego_start_x + dx1 + dx2
        pt0_y = shift_ego_start_y + dy1 + dy2
        pt1_x = shift_ego_start_x + dx1 - dx2
        pt1_y = shift_ego_start_y + dy1 - dy2
        pt2_x = shift_ego_start_x - dx1 - dx2
        pt2_y = shift_ego_start_y - dy1 - dy2
        pt3_x = shift_ego_start_x - dx1 + dx2
        pt3_y = shift_ego_start_y - dy1 + dy2
        return [[pt0_x, pt1_x, pt2_x, pt3_x, pt0_x], [pt0_y, pt1_y, pt2_y, pt3_y, pt0_y]]

    def plot_pos_data(self):
        fig = plt.figure(num="parking info")
        fig.subplots(1, 1, sharex=True, sharey=False)
        plt.subplot(1, 1, 1)
        plt.plot(self.x_ego_list, self.y_ego_list, label="ego pos", c='b')
        plt.plot(self.x_traj_list, self.y_traj_list, label="traj pos", c='g')
        # plt.plot([self.raw_slot0_x, self.raw_slot2_x, self.raw_slot3_x, self.raw_slot1_x],
        #          [self.raw_slot0_y, self.raw_slot2_y, self.raw_slot3_y, self.raw_slot1_y], label="raw slot")
        # plt.plot([self.slot0_x, self.slot2_x, self.slot3_x, self.slot1_x],
        #          [self.slot0_y, self.slot2_y, self.slot3_y, self.slot1_y], label="slot")
        replan_num = len(self.slot0_x_list)
        for i in np.arange(replan_num):
            line_width = 0.5
            if i == replan_num - 1:
                line_width = 1.5
            plt.plot([self.slot0_x_list[i], self.slot2_x_list[i], self.slot3_x_list[i], self.slot1_x_list[i]],
                     [self.slot0_y_list[i], self.slot2_y_list[i], self.slot3_y_list[i], self.slot1_y_list[i]],
                     linewidth=line_width, label="slot at seg "+str(i), marker=".")
        # plt.plot([self.mocked_obj_x0, self.mocked_obj_x1],
        #          [self.mocked_obj_y0, self.mocked_obj_y1], label="mocked obj")
        traj_len = len(self.x_traj_list)
        for i in range(traj_len):
            x = self.x_traj_list[i]
            y = self.y_traj_list[i]
            theta = self.theta_traj_list[i]
            traj_box = self.get_closed_veh_box(x, y, theta)
            plt.plot(traj_box[0], traj_box[1], c='g', linewidth=0.2)
        ego_len = len(self.x_ego_list)
        for i in range(ego_len):
            x = self.x_ego_list[i]
            y = self.y_ego_list[i]
            theta = self.theta_ego_list[i]
            ego_box = self.get_closed_veh_box(x, y, theta)
            plt.plot(ego_box[0], ego_box[1], c='b', linewidth=0.2)
        if len(self.x_traj_list) != 0:
            traj_x0 = self.x_traj_list[0]
            traj_y0 = self.y_traj_list[0]
            traj_theta0 = self.theta_traj_list[0]
            traj_box0 = self.get_closed_veh_box(traj_x0, traj_y0, traj_theta0)
            plt.plot(traj_box0[0], traj_box0[1], c='r', linewidth=1.5, label="first traj box")
            traj_x_1 = self.x_traj_list[-1]
            traj_y_1 = self.y_traj_list[-1]
            traj_theta_1 = self.theta_traj_list[-1]
            traj_box_1 = self.get_closed_veh_box(traj_x_1, traj_y_1, traj_theta_1)
            plt.plot(traj_box_1[0], traj_box_1[1], c='r', linewidth=1.5, label="last traj box")
        if len(self.x_ego_list) != 0:
            ego_x_1 = self.x_ego_list[-1]
            ego_y_1 = self.y_ego_list[-1]
            ego_theta_1 = self.theta_ego_list[-1]
            ego_box_1 = self.get_closed_veh_box(ego_x_1, ego_y_1, ego_theta_1)
            plt.plot(ego_box_1[0], ego_box_1[1], c='y', linewidth=1.5, label="last ego box")
            plt.plot(self.x_ego_list[0], self.y_ego_list[0], marker="x", label="ego start point")
        plt.plot(self.target_x, self.target_y, marker="x", label="target point")
        plt.axis('equal')
        plt.legend()
        plt.grid()

    def plot_time_diff(self):
        fig = plt.figure(num="time diff")
        fig.subplots(1, 1, sharex=True, sharey=False)

        plt.subplot(1, 1, 1)
        plt.plot(self.rel_time_list, self.all_time_diff_list, label="time diff")
        plt.legend()
        plt.grid()
        plt.show()

    def plot_smoothed_speed(self):
        fig = plt.figure(num="smoothed speed")
        fig.subplots(1, 1, sharex=True, sharey=False)

        plt.subplot(1, 1, 1)
        delta_s = 0.0
        smoothed_s_list = [0]
        for i in range(0, len(self.v_limit_list)):
            if i == 0:
                delta_s = self.smoothed_s_list[1] - self.smoothed_s_list[0]
            elif self.v_limit_list[i] * self.v_limit_list[i - 1] < 0.0:
                delta_s = self.smoothed_s_list[i + 1] - self.smoothed_s_list[i]
            else:
                delta_s = self.smoothed_s_list[i] - self.smoothed_s_list[i - 1]
            if i != 0:
                smoothed_s_list.append(smoothed_s_list[-1] + delta_s)
        plt.plot(smoothed_s_list, self.smooth_v_list, label="smoothed speed", marker='.')
        plt.plot(smoothed_s_list, self.v_limit_list, label="speed limit", marker='.')
        plt.legend()
        plt.grid()
        plt.show()