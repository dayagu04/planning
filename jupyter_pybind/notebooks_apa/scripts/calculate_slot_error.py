## 这个模块的目的是统计平行泊车以不同角度倒车入库时车位误差，
## 输入：
##      1. 数据bag（时间戳、定位、车位，终点处的车位，）、
##      2. 真实车位尺寸、
##      3. 最终泊车位姿相对车位的距离，前后轮与外侧车位线的横向距离，前后轮（或前后保险杠）与前后车位线的纵向距离
import json
import sys, copy
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

sys.path.append("..")

from lib.load_local_view_parking import (
    LoadCyberbag,
    load_local_view_figure_parking,
    update_local_view_data_parking,
)


def rotate(x, y, theta):
    x_rotated = x * math.cos(theta) - y * math.sin(theta)
    y_rotated = x * math.sin(theta) + y * math.cos(theta)
    return x_rotated, y_rotated


def local2global(x, y, ox, oy, otheta):
    tx, ty = rotate(x, y, otheta)
    return (tx + ox, ty + oy)


def load_ego_car_box(ego_x, ego_y, ego_heading, car_xb, car_yb):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_x, ego_y, ego_heading)
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    return car_xn, car_yn


def round_values_in_dict(data, precision=4):
    # 如果是字典，递归处理每个键值对
    if isinstance(data, dict):
        return {
            key: round_values_in_dict(value, precision) for key, value in data.items()
        }
    # 如果是列表，递归处理每个元素
    elif isinstance(data, list):
        return [round_values_in_dict(item, precision) for item in data]
    # 如果是浮点数，四舍五入
    elif isinstance(data, float):
        return round(data, precision)
    else:
        return data  # 如果既不是字典、列表或浮点数，直接返回原值


## 先实现的功能：以真实车位坐标系为基准，计算泊车终点的相对位置，把所有定位全局转局部坐标系（真实车位坐标系）
class ParkingSlotEvaluator:
    def __init__(self):
        self.true_slot_width_ = 2.2
        self.true_slot_length_ = 6.0

        self.ego_vertex_x_vec_ = [
            3.5815,
            3.8330,
            3.9250,
            3.9250,
            3.8330,
            3.5815,
            2.2350,
            2.2350,
            2.0350,
            2.0350,
            -0.4690,
            -0.8960,
            -1.0250,
            -1.0250,
            -0.8960,
            -0.4690,
            2.0350,
            2.0350,
            2.2350,
            2.2350,
        ]
        self.ego_vertex_y_vec_ = [
            0.9875,
            0.6755,
            0.2545,
            -0.2545,
            -0.6755,
            -0.9875,
            -0.9875,
            -1.1145,
            -1.1145,
            -0.9875,
            -0.9875,
            -0.8617,
            -0.4696,
            0.4696,
            0.8617,
            0.9875,
            0.9875,
            1.1145,
            1.1145,
            0.9875,
        ]
        self.wheel_base_ = 3.0

        self.ego_half_width_ = 1.0

        self.measured_terminal_lat_error_ = [0.0, 0.0]  ## 外侧前后轮与外侧车位内线的横向相对距离，向内为正
        self.measured_terminal_lon_error_ = [0.0, 0.0]  ## 保险杠与前后车位线的相对距离 向内为正

        self.measured_terminal_x_offset = [0.0, 0.0]
        self.measured_terminal_y_offset = [0.0, 0.0]

        self.terminal_pose_in_true_slot_ = [0.0, 0.0, 0.0]  ## true slot坐标系
        self.terminal_pose_in_bag_ = [0.0, 0.0, 0.0]

        self.slot_message_info_ = []  ## in boot sys
        self.ego_pose_message_info_ = []  ## in boot sys
        self.planning_message_info_ = []
        self.occ_obstacle_message_info_ = []  ## in boot sys

        self.slot_message_true_slot_ = []
        self.ego_pose_message_true_slot_ = []
        self.planning_message_true_slot_ = []
        self.occ_obstacle_message_true_slot_ = []

        self.cos_dtheta = 0.0
        self.sin_dtheta = 0.0

    def set_measured_error(
        self, front_lat_err, rear_lat_err, front_lon_err, rear_lon_err
    ):
        self.measured_terminal_lat_error_ = [front_lat_err, rear_lat_err]
        self.measured_terminal_lon_error_ = [front_lon_err, rear_lon_err]

        self.measured_terminal_x_offset = [-front_lon_err, rear_lon_err]
        self.measured_terminal_y_offset = [-front_lat_err, -rear_lat_err]

    def set_true_slot_info(self, slot_length, slot_width):
        self.true_slot_width_ = slot_width
        self.true_slot_length_ = slot_length

    def set_ego_info(self, ego_x_vec, ego_y_vec, wheel_base):
        self.ego_vertex_x_vec_ = ego_x_vec
        self.ego_vertex_y_vec_ = ego_y_vec
        self.wheel_base_ = wheel_base

    def set_ego_pos_message(self, ego_pose_message_info):
        self.ego_pose_message_info_ = ego_pose_message_info

    def set_planning_message(self, planning_message_info):
        self.planning_message_info_ = planning_message_info

    def set_slot_message(self, slot_message_info):
        self.slot_message_info_ = slot_message_info

    def set_occ_obstacle_info(self, occ_obs_vec):
        self.occ_obstacle_message_info_ = occ_obs_vec

    def calc_terminal_pose_in_true_slot(self):
        self.ego_half_width_ = self.ego_vertex_y_vec_[0]

        if (
            len(self.measured_terminal_x_offset) != 2
            or len(self.measured_terminal_y_offset) != 2
        ):
            return False

        self.terminal_pose_in_true_slot_ = [0.0, 0.0, 0.0]
        # calc relative heading
        dy = self.measured_terminal_y_offset[0] - self.measured_terminal_y_offset[1]
        dx = math.sqrt(self.wheel_base_ * self.wheel_base_ - dy * dy)
        self.terminal_pose_in_true_slot_[2] = math.atan(dy / dx)
        print("heading offset = ", np.rad2deg(self.terminal_pose_in_true_slot_[2]))

        rear_outer_wheel_pos = [0.0, self.ego_half_width_]
        rear_outer_wheel_pos = local2global(
            rear_outer_wheel_pos[0],
            rear_outer_wheel_pos[1],
            0.0,
            0.0,
            self.terminal_pose_in_true_slot_[2],
        )

        self.terminal_pose_in_true_slot_[1] = (
            self.true_slot_width_
            - rear_outer_wheel_pos[1]
            + self.measured_terminal_y_offset[1]
        )

        tmp_ego_x_vec, tmp_ego_y_vec = load_ego_car_box(
            0.0,
            0.0,
            self.terminal_pose_in_true_slot_[2],
            self.ego_vertex_x_vec_,
            self.ego_vertex_y_vec_,
        )

        self.terminal_pose_in_true_slot_[0] = (
            self.true_slot_length_
            - max(tmp_ego_x_vec)
            + self.measured_terminal_x_offset[0]
        )

    def transfer_pose_to_true_slot_sys(self, pose_in_bag):
        x_bag, y_bag, theta_bag = pose_in_bag
        terminal_x_bag, terminal_y_bag, terminal_theta_bag = self.terminal_pose_in_bag_
        (
            terminal_x_true_slot,
            terminal_y_true_slot,
            terminal_theta_true_slot,
        ) = self.terminal_pose_in_true_slot_

        dx = x_bag - terminal_x_bag
        dy = y_bag - terminal_y_bag

        x_prime = terminal_x_true_slot + self.cos_dtheta * dx - self.sin_dtheta * dy
        y_prime = terminal_y_true_slot + self.sin_dtheta * dx + self.cos_dtheta * dy

        theta_prime = terminal_theta_true_slot + (theta_bag - terminal_theta_bag)
        return [x_prime, y_prime, theta_prime]

    def transfer_pos_to_true_slot_sys(self, pos_in_bag):
        x_bag, y_bag = pos_in_bag
        terminal_x_bag, terminal_y_bag, terminal_theta_bag = self.terminal_pose_in_bag_
        (
            terminal_x_true_slot,
            terminal_y_true_slot,
            terminal_theta_true_slot,
        ) = self.terminal_pose_in_true_slot_

        dx = x_bag - terminal_x_bag
        dy = y_bag - terminal_y_bag

        x_prime = terminal_x_true_slot + self.cos_dtheta * dx - self.sin_dtheta * dy
        y_prime = terminal_y_true_slot + self.sin_dtheta * dx + self.cos_dtheta * dy

        return [x_prime, y_prime]

    def transfer_all_message_to_true_slot_sys(self):
        self.slot_message_true_slot_ = []
        self.ego_pose_message_true_slot_ = []
        self.occ_obstacle_message_true_slot_ = []
        self.planning_message_true_slot_ = []

        self.terminal_pose_in_bag_ = self.ego_pose_message_info_[-1]

        dtheta = self.terminal_pose_in_true_slot_[2] - self.terminal_pose_in_bag_[2]
        self.cos_dtheta = math.cos(dtheta)
        self.sin_dtheta = math.sin(dtheta)

        ## localization
        for pose in self.ego_pose_message_info_:
            self.ego_pose_message_true_slot_.append(
                self.transfer_pose_to_true_slot_sys(pose)
            )
        ## occ
        for pos_vec in self.occ_obstacle_message_info_:
            obs_vec_true_slot = []
            for pos in pos_vec:
                obs_vec_true_slot.append(self.transfer_pos_to_true_slot_sys(pos))
            self.occ_obstacle_message_true_slot_.append(obs_vec_true_slot)

        for planning_path in self.planning_message_info_:
            path_pt_vec = []
            for planning_pt in planning_path:
                path_pt_vec.append(self.transfer_pos_to_true_slot_sys(planning_pt))
            self.planning_message_true_slot_.append(path_pt_vec)

        ## vis slot
        for slot_pt_vec in self.slot_message_info_:
            slot_pt_vec_true_slot = []
            all_zero = all(point == [0, 0] for point in slot_pt_vec)

            if all_zero:
                slot_pt_vec_true_slot.append(slot_pt_vec)
            else:
                for pt in slot_pt_vec:
                    slot_pt_vec_true_slot.append(self.transfer_pos_to_true_slot_sys(pt))

                slot_center_x = sum(x_i for x_i, y_i in slot_pt_vec_true_slot) / 4.0
                slot_center_y = sum(y_i for x_i, y_i in slot_pt_vec_true_slot) / 4.0
                center = np.array([slot_center_x, slot_center_y])

                corrected_slot_pt_vec_true_slot = [[0, 0] for _ in range(4)]
                for pt in slot_pt_vec_true_slot:
                    v_oa = np.array(pt) - center
                    if v_oa[0] > 0.0 and v_oa[1] > 0.0:
                        corrected_slot_pt_vec_true_slot[0] = pt
                    elif v_oa[0] < 0.0 and v_oa[1] > 0.0:
                        corrected_slot_pt_vec_true_slot[1] = pt
                    elif v_oa[0] > 0.0 and v_oa[1] < 0.0:
                        corrected_slot_pt_vec_true_slot[2] = pt
                    elif v_oa[0] < 0.0 and v_oa[1] < 0.0:
                        corrected_slot_pt_vec_true_slot[3] = pt
                self.slot_message_true_slot_.append(corrected_slot_pt_vec_true_slot)

    def set_input(
        self,
        error_vec,
        slot_info,
        ego_pose_vec,
        vis_slot_vec,
        occ_obstacle_vec,
        planning_path_vec,
    ):
        if len(error_vec) != 4:
            return False
        if len(slot_info) != 2:
            return False

        self.set_measured_error(error_vec[0], error_vec[1], error_vec[2], error_vec[3])
        self.set_true_slot_info(slot_info[0], slot_info[1])

        self.set_ego_pos_message(ego_pose_vec)

        self.set_slot_message(vis_slot_vec)

        self.set_occ_obstacle_info(occ_obstacle_vec)

        self.set_planning_message(planning_path_vec)

    def update(self):
        self.calc_terminal_pose_in_true_slot()
        self.transfer_all_message_to_true_slot_sys()

        ideal_points = [
            [self.true_slot_length_, self.true_slot_width_],
            [0.0, self.true_slot_width_],
            [self.true_slot_length_, 0.0],
            [0.0, 0.0]
        ]

        start_idx = 0
        ego_pose_vec = np.array(self.ego_pose_message_true_slot_)
        for i in range(1, len(ego_pose_vec) -1):
            pos_0 = ego_pose_vec[i - 1][:2]
            pos_1 = ego_pose_vec[i][:2]
            pos_2 = ego_pose_vec[i + 1][:2]

            v_01 = pos_1 - pos_0
            v_12 = pos_2 - pos_1

            if np.dot(v_01, v_12) > 0.0 and v_01[0] < 0.0 and v_01[1] < 0.0 and pos_1[1] < self.true_slot_width_ + 2:
                start_idx = i
                break
        if start_idx == 0:
            print("find start idx failed")
            return False


        lat_err_vec = []
        lon_err_vec = []
        heading_deg_err_vec = []
        ego_heading_deg_vec = []

        # start_idx = 0

        for i in range(start_idx, len(ego_pose_vec)):
            ego_heading_deg_vec.append(math.degrees(ego_pose_vec[i][2]))

            slot_points = np.array(self.slot_message_true_slot_[i])
            err = slot_points - ideal_points

            lat_err_vec.append([err[0][1], err[1][1]])
            lon_err_vec.append([err[0][0], err[1][0]])

            v_01 = slot_points[0] - slot_points[1]
            heading_deg = np.degrees(np.arctan2(v_01[1], v_01[0]))
            heading_deg_err_vec.append(heading_deg)

        print("heading = ", max(heading_deg_err_vec), min(heading_deg_err_vec), np.mean(heading_deg_err_vec))

        ## plot
        ego_heading_deg_vec = np.array(ego_heading_deg_vec)       # (N,)
        lat_err_vec = np.array(lat_err_vec)                       # (N,4)
        lon_err_vec = np.array(lon_err_vec)                       # (N,4)
        heading_deg_err_vec = np.array(heading_deg_err_vec)       # (N,)

        N = len(ego_heading_deg_vec)

        # 展平成一维，便于画散点图
        ego_heading_for_err = np.repeat(ego_heading_deg_vec, 2)   # 每个角点都对应同一帧heading
        lat_err_flat = lat_err_vec.flatten()
        lon_err_flat = lon_err_vec.flatten()

        plt.figure(figsize=(16, 4))

        plt.subplot(1, 3, 1)
        plt.scatter(ego_heading_for_err, lat_err_flat, s=8, alpha=0.7)
        plt.xlabel('Ego Heading (deg)')
        plt.ylabel('Lat Error (m)')
        plt.title('Ego Heading vs. Lat Error')

        plt.subplot(1, 3, 2)
        plt.scatter(ego_heading_for_err, lon_err_flat, s=8, alpha=0.7)
        plt.xlabel('Ego Heading (deg)')
        plt.ylabel('Lon Error (m)')
        plt.title('Ego Heading vs. Lon Error')

        plt.subplot(1, 3, 3)
        plt.scatter(ego_heading_deg_vec, heading_deg_err_vec, s=8, alpha=0.7)
        plt.xlabel('Ego Heading (deg)')
        plt.ylabel('Heading Error (deg)')
        plt.title('Ego Heading vs. Heading Error')

        plt.tight_layout()
        plt.savefig('checker.svg')
        plt.close()

























    def process_rectangle_points(self, corner_points):
        # 计算矩形四个角点的中心
        center_x = sum(point[0] for point in corner_points) / 4
        center_y = sum(point[1] for point in corner_points) / 4
        center = [center_x, center_y]

        # 按照每个点相对于中心的角度进行排序
        sorted_points = sorted(
            corner_points,
            key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x),
        )

        # 返回排序后的角点和中心点
        final_fus_slot = [[point[0], point[1]] for point in sorted_points]

        return center, final_fus_slot

    def record_bags(
        self,
        dt = 0.2,
        bag_path="/data_cold/abu_zone/autoparse/chery_e0y_20267/trigger/20250620/20250620-16-29-47/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2025-06-20-16-29-48_no_camera.bag",
    ):
        start_time = time.time()
        bag_loader = LoadCyberbag(bag_path, True)
        load_bag_end_time = time.time()
        print(f"load bag time: {load_bag_end_time - start_time:.6f} seconds")

        max_time = bag_loader.load_all_data()

        load_all_data_end_time = time.time()
        print(
            f"load all data time: {load_all_data_end_time - load_bag_end_time:.6f} seconds"
        )

        count = 0.0
        apa_duration_time = 0.0

        ego_pos_vec = []
        planning_path_vec = []
        last_gear = 0
        gear_vec = []
        final_fus_slot = []
        vis_slot_vec = []
        occ_obstacles_vec = []

        success = False
        for bag_time in np.arange(max_time, 0.0, -dt):
            index_map = bag_loader.get_msg_index(bag_time)
            fus_msg = bag_loader.fus_parking_msg["data"][
                index_map["fus_parking_msg_idx"]
            ]
            plan_msg = bag_loader.plan_msg["data"][index_map["plan_msg_idx"]]
            apa_plan_status = plan_msg.planning_status.apa_planning_status

            if apa_plan_status != 1:
                continue

            select_slot_id = fus_msg.select_slot_id
            if select_slot_id == 0:
                continue

            fus_slot_lists = fus_msg.parking_fusion_slot_lists
            for i in range(fus_msg.parking_fusion_slot_lists_size):
                slot = fus_slot_lists[i]
                if select_slot_id == slot.id:
                    success = True
                    final_fus_slot = [
                        [point.x, point.y] for point in slot.corner_points
                    ]

                    break
            if success:
                break
        print("final_fus_slot = ", final_fus_slot)

        final_fus_slot_center, final_fus_slot = self.process_rectangle_points(
            final_fus_slot
        )

        for bag_time in np.arange(0.0, max_time, dt):
            index_map = bag_loader.get_msg_index(bag_time)

            soc_state_msg = bag_loader.soc_state_msg["data"][
                index_map["soc_state_msg_idx"]
            ]

            plan_msg = bag_loader.plan_msg["data"][index_map["plan_msg_idx"]]
            plan_debug_json = bag_loader.plan_debug_msg["json"][
                index_map["plan_debug_msg_idx"]
            ]

            vis_msg = bag_loader.vis_parking_msg["data"][
                index_map["vis_parking_msg_idx"]
            ]
            occ_msg = bag_loader.fus_occupancy_objects_msg["data"][
                index_map["fus_occupancy_objects_msg_idx"]
            ]

            current_state = soc_state_msg.current_state
            apa_plan_status = plan_msg.planning_status.apa_planning_status
            trajectory_points = plan_msg.trajectory.trajectory_points

            # print("apa_plan_status= ", apa_plan_status)

            if apa_plan_status == 1:
                count = count + 1
                apa_duration_time += dt

                if bag_loader.loc_msg["enable"] == True:
                    loc_msg = copy.deepcopy(
                        bag_loader.loc_msg["data"][index_map["loc_msg_idx"]]
                    )
                    current_ego_x = loc_msg.position.position_boot.x
                    current_ego_y = loc_msg.position.position_boot.y
                    current_ego_heading = loc_msg.orientation.euler_boot.yaw
                    ego_pos_vec.append(
                        [current_ego_x, current_ego_y, current_ego_heading]
                    )

                gear_command_value = plan_msg.gear_command.gear_command_value
                if gear_command_value in (2, 4) and gear_command_value != last_gear:
                    gear_vec.append(gear_command_value)

                    planning_path = [[pt.x, pt.y] for pt in trajectory_points]
                    planning_path_vec.append(planning_path)

                    obs_pt_vec = []
                    for k in range(occ_msg.fusion_object_size):
                        obj = occ_msg.fusion_object[k]

                        polygon_points = obj.additional_occupancy_info.polygon_points
                        for n in range(
                            obj.additional_occupancy_info.polygon_points_size
                        ):
                            x = polygon_points[n].x
                            y = polygon_points[n].y
                            obs_pt_vec.append([x, y])
                    print("leng obs_pt_vec", len(obs_pt_vec))
                    occ_obstacles_vec.append(obs_pt_vec)

                last_gear = gear_command_value

                parking_slots = vis_msg.parking_slots
                parking_slots_size = vis_msg.parking_slots_size
                # attention: vision slots and limiters are based on vehicle system, needed to be transferred into global system
                # 1. updatge slot points
                matched_slot_x_vec = []
                matched_slot_y_vec = []
                min_dist = 100

                for j in range(parking_slots_size):
                    slot = parking_slots[j]

                    single_slot_x_vec, single_slot_y_vec = zip(
                        *[
                            local2global(
                                slot.corner_points[k].x,
                                slot.corner_points[k].y,
                                current_ego_x,
                                current_ego_y,
                                current_ego_heading,
                            )
                            for k in range(4)
                        ]
                    )

                    single_slot_x_vec = np.array(single_slot_x_vec)
                    single_slot_y_vec = np.array(single_slot_y_vec)
                    single_slot_center = np.array(
                        [np.mean(single_slot_x_vec), np.mean(single_slot_y_vec)]
                    )

                    dist = np.linalg.norm(single_slot_center - final_fus_slot_center)
                    if dist < 3.0 and dist < min_dist:
                        min_dist = dist
                        matched_slot_x_vec = single_slot_x_vec
                        matched_slot_y_vec = single_slot_y_vec

                if len(matched_slot_x_vec) > 0:
                    matched_slot_coords = [
                        [x, y] for x, y in zip(matched_slot_x_vec, matched_slot_y_vec)
                    ]
                    _, matched_slot_coords = self.process_rectangle_points(
                        matched_slot_coords
                    )
                    vis_slot_vec.append(matched_slot_coords)
                else:
                    vis_slot_vec.append([(0, 0), [0, 0], [0, 0], [0, 0]])

        result = {
            "dt": dt,
            "ego_pose_vec": ego_pos_vec,
            "planning_path_vec": planning_path_vec,
            "gear_vec": gear_vec,
            "final_fus_slot": final_fus_slot,
            "vis_slot": vis_slot_vec,
            "occ_obstacle": occ_obstacles_vec,
        }

        result = round_values_in_dict(result, precision=3)
        with open("data.json", "w") as f:
            json.dump(result, f)

        print("gear_vec = ", gear_vec)
        print("len planning_path_vec = ", len(planning_path_vec))

    def draw_messages(
        self,
        ego_pose_vec,
        planning_path_vec,
        final_fus_slot,
        vis_slot_vec,
        occ_obs_vec,
        save_path="corrected_data.svg",
    ):
        plt.rcParams.update(
            {
                "font.family": "Times New Roman",
                "font.size": 9,
                "axes.labelsize": 9,
                "axes.titlesize": 9,
                "legend.fontsize": 9,
                "xtick.labelsize": 9,
                "ytick.labelsize": 9,
                "lines.linewidth": 1.2,
            }
        )

        fig, ax = plt.subplots(figsize=(10, 6), dpi=800)

        if len(vis_slot_vec) > 0:
            vis_rectangles = np.array(vis_slot_vec)
            vis_x_vec, vis_y_vec = vis_rectangles[:, :, 0], vis_rectangles[:, :, 1]
            vis_x_vec = np.concatenate(
                [vis_x_vec, vis_x_vec[:, [0]]], axis=1
            )  # 添加第一个点到最后，确保闭合
            vis_y_vec = np.concatenate(
                [vis_y_vec, vis_y_vec[:, [0]]], axis=1
            )  # 同样添加第一个点到最后，确保闭合
            ax.plot(
                vis_x_vec.T,
                vis_y_vec.T,
                linestyle="-",
                color="gray",
                linewidth=0.5,
            )

        if len(occ_obs_vec) > 0:
            occ_obs_x, occ_obs_y = zip(*occ_obs_vec)
            ax.scatter(occ_obs_x, occ_obs_y, color="gray", marker="o", s=3)

        if len(final_fus_slot) > 0:
            final_fus_slot_x_vec, final_fus_slot_y_vec = zip(*final_fus_slot)
            final_fus_slot_x_vec += (final_fus_slot_x_vec[0],)
            final_fus_slot_y_vec += (final_fus_slot_y_vec[0],)
            ax.plot(
                final_fus_slot_x_vec,
                final_fus_slot_y_vec,
                linestyle="-",
                color="black",
                linewidth=1.2,
                label="final slot",
            )

        if len(ego_pose_vec) > 0:
            control_x_vec = [p[0] for p in ego_pose_vec]
            control_y_vec = [p[1] for p in ego_pose_vec]

            ax.plot(
                control_x_vec,
                control_y_vec,
                linestyle="--",
                color="black",
                linewidth=1.2,
                label="Control",
            )

        if len(planning_path_vec) > 0:
            lc = LineCollection(
                planning_path_vec, colors="red", linewidths=1.0, label="Planning"
            )

            ax.add_collection(lc)



        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(
            loc="best",
            frameon=False,
            handlelength=1.5,
        )
        ax.grid(False)

        ax.set_aspect("equal", adjustable="box")

        # plt.tight_layout(pad=0.2)
        plt.savefig(save_path, bbox_inches="tight", pad_inches=0.01)
        plt.close(fig)  # 推荐画完关闭，防止内存泄露（jupyter尤甚）

    def draw_points(self, result, save_path="ieee_trajectory.svg"):
        ego_pose_vec = result["ego_pose_vec"]
        planning_path_vec = result["planning_path_vec"]

        final_fus_slot_vec = result["final_fus_slot"]
        vis_slot_vec = result["vis_slot"]

        occ_obs_vec = result["occ_obstacle"][0]

        control_x_vec = [p[0] for p in ego_pose_vec]
        control_y_vec = [p[1] for p in ego_pose_vec]

        final_fus_slot_x_vec, final_fus_slot_y_vec = zip(*final_fus_slot_vec)
        final_fus_slot_x_vec += (final_fus_slot_x_vec[0],)
        final_fus_slot_y_vec += (final_fus_slot_y_vec[0],)

        occ_obs_x, occ_obs_y = zip(*occ_obs_vec)

        vis_rectangles = np.array(vis_slot_vec)
        vis_x_vec, vis_y_vec = vis_rectangles[:, :, 0], vis_rectangles[:, :, 1]
        vis_x_vec = np.concatenate(
            [vis_x_vec, vis_x_vec[:, [0]]], axis=1
        )  # 添加第一个点到最后，确保闭合
        vis_y_vec = np.concatenate(
            [vis_y_vec, vis_y_vec[:, [0]]], axis=1
        )  # 同样添加第一个点到最后，确保闭合

        plt.rcParams.update(
            {
                "font.family": "Times New Roman",
                "font.size": 9,
                "axes.labelsize": 9,
                "axes.titlesize": 9,
                "legend.fontsize": 9,
                "xtick.labelsize": 9,
                "ytick.labelsize": 9,
                "lines.linewidth": 1.2,
            }
        )

        fig, ax = plt.subplots(figsize=(10, 6), dpi=800)

        lc = LineCollection(
            planning_path_vec, colors="red", linewidths=1.0, label="Planning"
        )

        ax.add_collection(lc)

        ax.plot(
            control_x_vec,
            control_y_vec,
            linestyle="--",
            color="black",
            linewidth=1.2,
            label="Control",
        )

        ax.plot(
            final_fus_slot_x_vec,
            final_fus_slot_y_vec,
            linestyle="-",
            color="black",
            linewidth=1.2,
            label="terminal fus slot",
        )

        ax.scatter(
            occ_obs_x, occ_obs_y, color="gray", marker="o", s=3
        )  # s=50 表示圆点的大小，color='b' 表示蓝色

        ax.plot(
            vis_x_vec.T,
            vis_y_vec.T,
            linestyle="-",
            color="gray",
            linewidth=0.5,
        )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        ax.legend(
            loc="best",
            frameon=False,
            handlelength=1.5,
        )
        ax.grid(False)

        ax.set_aspect("equal", adjustable="box")

        # plt.tight_layout(pad=0.2)
        plt.savefig(save_path, bbox_inches="tight", pad_inches=0.01)
        plt.close(fig)  # 推荐画完关闭，防止内存泄露（jupyter尤甚）


if __name__ == "__main__":
    parking_slot_eva = ParkingSlotEvaluator()

    parking_slot_eva.record_bags(dt = 0.2)

    with open("data.json", "r") as f:
        result = json.load(f)
    # parking_slot_eva.draw_points(result, "my_points.svg")

    error_vec = [0.12, 0.15, 0.6, 1.1]
    slot_info = [6.2, 2.4]

    parking_slot_eva.set_input(
        error_vec,
        slot_info,
        result["ego_pose_vec"],
        result["vis_slot"],
        result["occ_obstacle"],
        result["planning_path_vec"],
    )

    parking_slot_eva.update()

    slot_length = parking_slot_eva.true_slot_length_
    slot_width = parking_slot_eva.true_slot_width_

    final_fus_slot = [[slot_length, slot_width], [0.0, slot_width],  [0.0, 0.0], [slot_length, 0.0]]

    parking_slot_eva.draw_messages(
        parking_slot_eva.ego_pose_message_true_slot_,
        parking_slot_eva.planning_message_true_slot_,
        final_fus_slot,
        parking_slot_eva.slot_message_true_slot_,
        parking_slot_eva.occ_obstacle_message_true_slot_[0],
        save_path="corrected_data.svg",
    )

    print("len(slot) = ", len(parking_slot_eva.slot_message_true_slot_))
    print("len(ego pose vec) = ", len(parking_slot_eva.ego_pose_message_true_slot_))

    ## evaluation
