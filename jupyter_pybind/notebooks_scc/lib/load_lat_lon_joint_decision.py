
from lib.load_rotate import *
import numpy as np
import warnings
import logging

warnings.filterwarnings('ignore', category=UserWarning, module='bokeh')
warnings.filterwarnings('ignore', category=UserWarning, message='.*ColumnDataSource.*')
logging.getLogger('bokeh').setLevel(logging.ERROR)

from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import (
    WheelZoomTool, HoverTool,
    Label, LabelSet, DataTable, DateFormatter, TableColumn,
    Panel, Tabs, CustomJS, CustomJSHover, Legend,
    Range1d, LinearAxis
)
import ipywidgets as widgets
from ipywidgets import Button, HBox
from IPython.display import clear_output, display
import time, threading
from collections import namedtuple
from functools import partial
import bokeh.plotting as bkp
from bokeh.plotting import ColumnDataSource
from lib.load_json import *
from lib.load_struct import *
import math


# Coordinate transformer (global ↔ local)
coord_tf = coord_transformer()

# --- Constants ---
RAD_TO_DEG = 57.2957795  # 180 / math.pi


def generate_circle_points(center_x, center_y, radius, num_points=50):
    """Generate sampling points on a circle."""
    angles = np.linspace(0, 2 * np.pi, num_points)
    x_points = center_x + radius * np.cos(angles)
    y_points = center_y + radius * np.sin(angles)
    return x_points.tolist(), y_points.tolist()


def calc_three_centers(x, y, theta, length, is_ego=True):
    """Calculate center coordinates for three-disc model."""
    L = length / 6.0
    backcenter = L if is_ego else 3 * L
    offsets = [L - backcenter, 3 * L - backcenter, 5 * L - backcenter]

    centers = []
    for offset in offsets:
        center_x = x + offset * math.cos(theta)
        center_y = y + offset * math.sin(theta)
        centers.append((center_x, center_y))

    return centers


def calc_radius(length, width):
    """Calculate disc radius."""
    return math.sqrt((length / 6.0) ** 2 + (width / 2.0) ** 2)


def update_joint_plan_data(bag_loader, bag_time, local_view_data, joint_plan_data):
    """Update joint planner input/output into Bokeh data sources."""
    
    # 定义创建适合Bokeh数据源的轨迹段辅助函数
    def create_bokeh_segments(data_x, data_y, time_vec_length, num_segments=5):
        """创建适合Bokeh ColumnDataSource的轨迹段数据"""
        segments = {}
        
        if len(data_x) > 0 and time_vec_length > 0:
            # 将轨迹点分成5段
            points_per_segment = len(data_x) // num_segments
            remainder = len(data_x) % num_segments
            
            current_idx = 0
            for i in range(num_segments):
                # 计算当前段的起止索引
                segment_size = points_per_segment + (1 if i < remainder else 0)
                end_idx = min(current_idx + segment_size, len(data_x))
                
                # 提取当前段的数据
                if current_idx < len(data_x):
                    seg_x = data_x[current_idx:end_idx]
                    seg_y = data_y[current_idx:end_idx]
                else:
                    seg_x = []
                    seg_y = []
                
                # 为了匹配时间序列长度，我们需要创建标记数组
                # 用NaN填充非分段点，只在分段点填入实际值
                full_x = [float('nan')] * time_vec_length
                full_y = [float('nan')] * time_vec_length
                
                # 在对应的时间点填入分段数据
                if len(seg_x) > 0:
                    # 计算在时间序列中的对应位置
                    time_start = current_idx
                    for j, (x, y) in enumerate(zip(seg_x, seg_y)):
                        time_idx = min(time_start + j, time_vec_length - 1)
                        full_x[time_idx] = x
                        full_y[time_idx] = y
                
                segments[f'seg_{i}_x'] = full_x
                segments[f'seg_{i}_y'] = full_y
                
                current_idx = end_idx
        else:
            # 空数据情况
            for i in range(num_segments):
                segments[f'seg_{i}_x'] = [float('nan')] * time_vec_length
                segments[f'seg_{i}_y'] = [float('nan')] * time_vec_length
        
        return segments
    
    plan_debug = local_view_data['data_msg']['plan_debug_msg']
    planner_json = local_view_data['data_msg']['plan_debug_json_msg']

    # Update coordinate transformer
    if bag_loader.loc_msg['enable']:
        loc = local_view_data['data_msg']['loc_msg']
        coord_tf.set_info(
            loc.position.position_boot.x,
            loc.position.position_boot.y,
            loc.orientation.euler_boot.yaw)

    jp_in = plan_debug.joint_decision_input
    jp_out = plan_debug.joint_decision_output

    # Update road boundary data
    if jp_in.HasField('left_road_boundary') and jp_in.HasField('right_road_boundary'):
        left_x_local, left_y_local = coord_tf.global_to_local(
            jp_in.left_road_boundary.x_vec,
            jp_in.left_road_boundary.y_vec)
        right_x_local, right_y_local = coord_tf.global_to_local(
            jp_in.right_road_boundary.x_vec,
            jp_in.right_road_boundary.y_vec)

        joint_plan_data['data_road_boundary'].data.update({
            'left_x': left_x_local,
            'left_y': left_y_local,
            'right_x': right_x_local,
            'right_y': right_y_local
        })

    # Extract planning info from JSON
    planning_json_value_list = ["joint_lead_one_id", "joint_key_agent_ids", "joint_limit_speed"]
    vision_lon_attr_vec = []
    for attr in planning_json_value_list:
        if attr == "joint_key_agent_ids":
            # joint_key_agent_ids is a list, convert to string for display
            joint_key_agent_ids = planner_json.get(attr, [])
            if isinstance(joint_key_agent_ids, list):
                vision_lon_attr_vec.append(str(joint_key_agent_ids))
            else:
                vision_lon_attr_vec.append(str(joint_key_agent_ids))
        else:
            vision_lon_attr_vec.append(planner_json.get(attr, 0))
    
    # Extract lane change state
    lane_change_state_dict = {
        0: "IDLE",
        1: "READY",
        2: "EXECUTION",
        3: "COMPLETE",
        5: "HOLD",
        4: "CANCEL"
    }
    joint_lane_change_state = planner_json.get("joint_lane_change_state", 0)
    lane_change_state_name = lane_change_state_dict.get(joint_lane_change_state, f"UNKNOWN_{joint_lane_change_state}")
    #lane id in table
    target_lane_virtual_id = planner_json.get("target_lane_virtual_id", -99)
    origin_lane_virtual_id = planner_json.get("origin_lane_virtual_id", -99)
    gap_front_agent_id = planner_json.get("lc_gap_front_agent_id", -99)
    gap_rear_agent_id = planner_json.get("lc_gap_rear_agent_id", -99)
    # Edge positions for safety check
    ego_front_edge = planner_json.get("ego_front_edge", -99)
    ego_rear_edge = planner_json.get("ego_rear_edge", -99)
    front_agent_front_edge = planner_json.get("front_agent_front_edge", -99)
    front_agent_rear_edge = planner_json.get("front_agent_rear_edge", -99)
    rear_agent_front_edge = planner_json.get("rear_agent_front_edge", -99)
    rear_agent_rear_edge = planner_json.get("rear_agent_rear_edge", -99)
    # Extract obstacle information for display
    joint_lead_one_id = planner_json.get("joint_lead_one_id", 0)
    joint_key_agent_ids = planner_json.get("joint_key_agent_ids", [])
    joint_limit_speed = planner_json.get("joint_limit_speed", 0)
    obs_num = jp_in.obs_num

    ego_length = jp_in.ego_length if jp_in.HasField('ego_length') else 4.95
    ego_width = jp_in.ego_width if jp_in.HasField('ego_width') else 2.0

    # Extract ego reference trajectory
    ref_x_vec = list(jp_in.ref_x_vec)
    ref_y_vec = list(jp_in.ref_y_vec)
    ref_theta_vec = list(jp_in.ref_theta_vec)
    ref_delta_vec = list(jp_in.ref_delta_vec)
    ref_vel_vec = list(jp_in.ref_vel_vec)
    ref_acc_vec = list(jp_in.ref_acc_vec)
    ref_s_vec = list(jp_in.ref_s_vec)
    
    obs_num = jp_in.obs_num

    # Extract ego output trajectory
    time_vec = list(jp_out.time_vec)
    
    # Create velocity bounds vectors
    vel_max_bound_vec = [joint_limit_speed] * len(time_vec)
    vel_min_bound_vec = [0.0] * len(time_vec)
    x_vec = list(jp_out.x_vec)
    y_vec = list(jp_out.y_vec)
    theta_vec = list(jp_out.theta_vec)
    delta_vec = list(jp_out.delta_vec)
    vel_vec = list(jp_out.vel_vec)
    acc_vec = list(jp_out.acc_vec)
    jerk_vec = list(jp_out.jerk_vec)
    omega_vec = list(jp_out.omega_vec)

    # Calculate ego three-disc data
    ego_three_disc_data = {
        'center_x': [], 'center_y': [], 'center_x_local': [], 'center_y_local': [],
        'radius': [],
        'circle_x': [], 'circle_y': [], 'circle_x_local': [], 'circle_y_local': [],
        'decision_label': []
    }

    if len(x_vec) > 0 and len(y_vec) > 0 and len(theta_vec) > 0:
        ego_x, ego_y, ego_theta = x_vec[0], y_vec[0], theta_vec[0]
        ego_centers = calc_three_centers(ego_x, ego_y, ego_theta, ego_length, is_ego=True)
        ego_radius = calc_radius(ego_length, ego_width)

        for i, (center_x, center_y) in enumerate(ego_centers):
            center_x_local, center_y_local = coord_tf.global_to_local([center_x], [center_y])
            circle_x_global, circle_y_global = generate_circle_points(center_x, center_y, ego_radius)
            circle_x_local, circle_y_local = coord_tf.global_to_local(circle_x_global, circle_y_global)

            ego_three_disc_data['center_x'].append(center_x)
            ego_three_disc_data['center_y'].append(center_y)
            ego_three_disc_data['center_x_local'].append(center_x_local[0])
            ego_three_disc_data['center_y_local'].append(center_y_local[0])
            ego_three_disc_data['radius'].append(ego_radius)
            ego_three_disc_data['circle_x'].append(circle_x_global)
            ego_three_disc_data['circle_y'].append(circle_y_global)
            ego_three_disc_data['circle_x_local'].append(circle_x_local)
            ego_three_disc_data['circle_y_local'].append(circle_y_local)
            ego_three_disc_data['decision_label'].append('ego')

    joint_plan_data['data_ego_three_disc'].data.update(ego_three_disc_data)

    # Update obstacle reference trajectories
    obs_ref_trajectories = jp_in.obs_ref_trajectory
    for i in range(10):
        if i < len(obs_ref_trajectories):
            obs_traj = obs_ref_trajectories[i]
            ref_x_vec_local, ref_y_vec_local = coord_tf.global_to_local(
                obs_traj.ref_x_vec, obs_traj.ref_y_vec)


            # Extract longitudinal_label if available
            longitudinal_label = obs_traj.longitudinal_label if obs_traj.HasField('longitudinal_label') else -1
            
            joint_plan_data[f'data_obs_ref_{i}'].data.update({
                'time_vec': [round(t, 2) for t in time_vec],
                'ref_x_vec': list(obs_traj.ref_x_vec),
                'ref_y_vec': list(obs_traj.ref_y_vec),
                'ref_x_local_vec': ref_x_vec_local,
                'ref_y_local_vec': ref_y_vec_local,
                'ref_theta_vec': list(obs_traj.ref_theta_vec),
                'ref_delta_vec': list(obs_traj.ref_delta_vec),
                'ref_vel_vec': list(obs_traj.ref_vel_vec),
                'ref_acc_vec': list(obs_traj.ref_acc_vec),
                'ref_s_vec': list(obs_traj.ref_s_vec),
                'obs_id': [obs_traj.obs_id] * len(time_vec),
                'longitudinal_label': [longitudinal_label] * len(time_vec)
            })
        else:
            # 为空数据创建空字段
            joint_plan_data[f'data_obs_ref_{i}'].data.update({
                'time_vec': [], 'ref_x_vec': [], 'ref_y_vec': [],
                'ref_x_local_vec': [], 'ref_y_local_vec': [],
                'ref_theta_vec': [], 'ref_delta_vec': [],
                'ref_vel_vec': [], 'ref_acc_vec': [], 'ref_s_vec': [],
                'obs_id': [], 'longitudinal_label': []
            })

    # Update obstacle opt trajectories
    obs_opt_trajectory = jp_out.obs_opt_trajectory
    for i in range(10):
        if i < len(obs_opt_trajectory):
            obs_traj = obs_opt_trajectory[i]
            x_vec_local, y_vec_local = coord_tf.global_to_local(obs_traj.x_vec, obs_traj.y_vec)

            # 为障碍物优化轨迹创建分段数据
            obs_opt_segments = create_bokeh_segments(x_vec_local, y_vec_local, len(time_vec))

            joint_plan_data[f'data_obs_opt_{i}'].data.update({
                'time_vec': [round(t, 2) for t in time_vec],
                'x_vec': list(obs_traj.x_vec),
                'y_vec': list(obs_traj.y_vec),
                'x_local_vec': x_vec_local,
                'y_local_vec': y_vec_local,
                'theta_vec': list(obs_traj.theta_vec),
                'delta_vec': list(obs_traj.delta_vec),
                'omega_vec': list(obs_traj.omega_vec),
                'vel_vec': list(obs_traj.vel_vec),
                'acc_vec': list(obs_traj.acc_vec),
                'jerk_vec': list(obs_traj.jerk_vec),
                's_vec': list(obs_traj.s_vec),
                'obs_id': [obs_ref_trajectories[i].obs_id] * len(time_vec),
                # 添加分段轨迹数据 - 障碍物opt轨迹
                'obs_opt_seg0_x': obs_opt_segments['seg_0_x'],
                'obs_opt_seg0_y': obs_opt_segments['seg_0_y'],
                'obs_opt_seg1_x': obs_opt_segments['seg_1_x'],
                'obs_opt_seg1_y': obs_opt_segments['seg_1_y'],
                'obs_opt_seg2_x': obs_opt_segments['seg_2_x'],
                'obs_opt_seg2_y': obs_opt_segments['seg_2_y'],
                'obs_opt_seg3_x': obs_opt_segments['seg_3_x'],
                'obs_opt_seg3_y': obs_opt_segments['seg_3_y'],
                'obs_opt_seg4_x': obs_opt_segments['seg_4_x'],
                'obs_opt_seg4_y': obs_opt_segments['seg_4_y']
            })

            # Calculate obstacle three-disc data
            obs_three_disc_data = {
                'center_x': [], 'center_y': [], 'center_x_local': [], 'center_y_local': [],
                'radius': [], 'obs_id': [],
                'circle_x': [], 'circle_y': [], 'circle_x_local': [], 'circle_y_local': [],
                'decision_label': []
            }

            if len(obs_traj.x_vec) > 0 and len(obs_traj.y_vec) > 0 and len(obs_traj.theta_vec) > 0:
                obs_x, obs_y, obs_theta = obs_traj.x_vec[0], obs_traj.y_vec[0], obs_traj.theta_vec[0]
                if i < len(obs_ref_trajectories):
                    obs_length = obs_ref_trajectories[i].length
                    obs_width = obs_ref_trajectories[i].width
                else:
                    obs_length = 4.5
                    obs_width = 2.0

                obs_centers = calc_three_centers(obs_x, obs_y, obs_theta, obs_length, is_ego=False)
                obs_radius = calc_radius(obs_length, obs_width)

                for j, (center_x, center_y) in enumerate(obs_centers):
                    center_x_local, center_y_local = coord_tf.global_to_local([center_x], [center_y])
                    circle_x_global, circle_y_global = generate_circle_points(center_x, center_y, obs_radius)
                    circle_x_local, circle_y_local = coord_tf.global_to_local(circle_x_global, circle_y_global)

                    obs_three_disc_data['center_x'].append(center_x)
                    obs_three_disc_data['center_y'].append(center_y)
                    obs_three_disc_data['center_x_local'].append(center_x_local[0])
                    obs_three_disc_data['center_y_local'].append(center_y_local[0])
                    obs_three_disc_data['radius'].append(obs_radius)
                    obs_three_disc_data['obs_id'].append(obs_ref_trajectories[i].obs_id)
                    obs_three_disc_data['circle_x'].append(circle_x_global)
                    obs_three_disc_data['circle_y'].append(circle_y_global)
                    obs_three_disc_data['circle_x_local'].append(circle_x_local)
                    obs_three_disc_data['circle_y_local'].append(circle_y_local)
                    # Use longitudinal_label from obs_ref_trajectory if available
                    if i < len(obs_ref_trajectories) and obs_ref_trajectories[i].HasField('longitudinal_label'):
                        longitudinal_label = obs_ref_trajectories[i].longitudinal_label
                        # Convert longitudinal_label to meaningful text
                        if longitudinal_label == 0:
                            label_text = 'IGNORE'
                        elif longitudinal_label == 1:
                            label_text = 'OVERTAKE'
                        elif longitudinal_label == 2:
                            label_text = 'YIELD'
                        elif longitudinal_label == 3:
                            label_text = 'EGO_OVERTAKE'
                        else:
                            label_text = f'UNKNOWN_{longitudinal_label}'
                        obs_three_disc_data['decision_label'].append(label_text)
                    else:
                        obs_three_disc_data['decision_label'].append(f'obstacle_{i}')

            joint_plan_data[f'data_obs_three_disc_{i}'].data.update(obs_three_disc_data)
        else:
            # 为空数据创建分段字段
            empty_segments = create_bokeh_segments([], [], 0)
            joint_plan_data[f'data_obs_opt_{i}'].data.update({
                'time_vec': [], 'x_vec': [], 'y_vec': [], 'x_local_vec': [], 'y_local_vec': [],
                'theta_vec': [], 'delta_vec': [], 'omega_vec': [],
                'vel_vec': [], 'acc_vec': [], 'jerk_vec': [], 's_vec': [], 'obs_id': [],
                # 添加空的分段数据字段
                'obs_opt_seg0_x': empty_segments['seg_0_x'],
                'obs_opt_seg0_y': empty_segments['seg_0_y'],
                'obs_opt_seg1_x': empty_segments['seg_1_x'],
                'obs_opt_seg1_y': empty_segments['seg_1_y'],
                'obs_opt_seg2_x': empty_segments['seg_2_x'],
                'obs_opt_seg2_y': empty_segments['seg_2_y'],
                'obs_opt_seg3_x': empty_segments['seg_3_x'],
                'obs_opt_seg3_y': empty_segments['seg_3_y'],
                'obs_opt_seg4_x': empty_segments['seg_4_x'],
                'obs_opt_seg4_y': empty_segments['seg_4_y']
            })
            joint_plan_data[f'data_obs_three_disc_{i}'].data.update({
                'center_x': [], 'center_y': [], 'center_x_local': [], 'center_y_local': [],
                'radius': [], 'obs_id': [],
                'circle_x': [], 'circle_y': [], 'circle_x_local': [], 'circle_y_local': [],
                'decision_label': []
            })

    # Update key agents data
    key_agent_x_local = []
    key_agent_y_local = []
    key_agent_ids = []
    
    # Extract key agent information from planning_json
    if 'joint_key_agent_ids' in planner_json and planner_json['joint_key_agent_ids']:
        joint_key_agent_ids = planner_json['joint_key_agent_ids']
        if isinstance(joint_key_agent_ids, list):
            # Find the agent positions from obstacle data
            for key_agent_id in joint_key_agent_ids:
                for i, obs_traj in enumerate(jp_out.obs_opt_trajectory):
                    if i < len(jp_in.obs_init_state) and jp_in.obs_init_state[i].agent_id == key_agent_id:
                        if len(obs_traj.x_vec) > 0 and len(obs_traj.y_vec) > 0:
                            x_local, y_local = coord_tf.global_to_local([obs_traj.x_vec[0]], [obs_traj.y_vec[0]])
                            key_agent_x_local.append(x_local[0])
                            key_agent_y_local.append(y_local[0])
                            key_agent_ids.append(f"Key_{key_agent_id}")
                        break
    
    joint_plan_data['data_key_agents'].data.update({
        'x_local': key_agent_x_local,
        'y_local': key_agent_y_local,
        'agent_id': key_agent_ids
    })

    # Print solver costs (optional)
    solver_info = jp_out.solver_info
    iter_count = solver_info.iter_count
    cost_size = solver_info.cost_size
    cost_vec = solver_info.cost_vec
    lists = [cost_vec[i * cost_size : (i + 1) * cost_size] for i in range(iter_count)]
    cost_list = ["EgoReferenceCost", "EgoThreeDiscSafeCost", "HardHalfplaneCost", "SoftHalfplaneCost",
                 "EgoRoadBoundaryCost", "EgoAccCost", "EgoJerkCost", "EgoDeltaCost", 
                 "EgoOmegaCost", "EgoAccBoundCost", "EgoJerkBoundCost", "ObsReferenceCost", "ObsJerkCost", "ObsOmegaCost"]
    
    # Get solver condition and determine planning success
    solver_condition = solver_info.solver_condition
    solver_condition_names = {
        0: "INIT",
        1: "NORMAL_TERMINATE", 
        2: "CONST_CONTROL_TERMINATE",
        3: "MAX_ITER_TERMINATE",
        4: "LINESEARCH_TERMINATE",
        5: "INIT_TERMINATE",
        6: "MAX_OUTERITER_TERMINATE",
        7: "KKT_TERMINATE",
        8: "BACKWARD_PASS_FAIL",
        9: "NON_POSITIVE_EXPECT",
        10: "FAULT_INPUT_SIZE"
    }
    
    # Planning is successful if solver_condition < 8 (BACKWARD_PASS_FAIL)
    planning_success = solver_condition < 8
    solver_condition_name = solver_condition_names.get(solver_condition, f"UNKNOWN_{solver_condition}")
    
    print(cost_list)
    for i, sub_list in enumerate(lists):
        if i == 0:
            print(f"Cost init: {sub_list}")
        else:
            print(f"Cost {i}: {sub_list}")
    
    obstacle_selection_time = planner_json.get("JointDecisionObstacleSelectionTime", 0.0)
    optimization_time = planner_json.get("JointDecisionOptimizationTime", 0.0)
    lat_lon_joint_planner_time = planner_json.get('LatLonJointDecisionTime', 0.0)
    
    # Extract rear agent longitudinal label from JSON
    rear_agent_label = planner_json.get("rear_agent_longitudinal_label", -1)
    label_dict = {
        0: "IGNORE",
        1: "OVERTAKE",
        2: "YIELD",
        3: "EGO_OVERTAKE"
    }
    rear_agent_label_name = label_dict.get(rear_agent_label, f"UNKNOWN_{rear_agent_label}")
    rear_agent_confidence = planner_json.get("rear_agent_confidence", None)
    rear_agent_confidence_str = (
        f"{rear_agent_confidence:.3f}" if isinstance(rear_agent_confidence, (int, float)) else "N/A"
    )
    
    # Update planning info data for display - 仅优化器相关信息
    planning_info_data = {
        'labels': ['Total Decision Time', 'Obstacle Selection Time', 'Optimization Time',
                   'Solver Condition', 'Planning Condition', 'Rear Agent Label', 'Rear Agent Confidence'],
        'values': [f"{round(lat_lon_joint_planner_time, 2)}ms", 
                   f"{round(obstacle_selection_time, 2)}ms", f"{round(optimization_time, 2)}ms",
                   solver_condition_name, "SUCCESS" if planning_success else "FAILED",
                   rear_agent_label_name, rear_agent_confidence_str]
    }
    joint_plan_data['data_planning_info'].data.update(planning_info_data)

    # Update ego trajectory data
    ref_x_vec_local, ref_y_vec_local = coord_tf.global_to_local(ref_x_vec, ref_y_vec)
    x_vec_local, y_vec_local = coord_tf.global_to_local(x_vec, y_vec)

    # 为自车轨迹创建分段数据
    ego_act_segments = create_bokeh_segments(x_vec_local, y_vec_local, len(time_vec))

    # 获取加速度边界参数（数组形式）
    if len(jp_in.ego_acc_max) > 0:
        acc_max_bound_vec = list(jp_in.ego_acc_max)[:len(time_vec)]
        # 如果数组长度不够，用最后一个值填充
        while len(acc_max_bound_vec) < len(time_vec):
            acc_max_bound_vec.append(acc_max_bound_vec[-1] if acc_max_bound_vec else 1.35)
    else:
        acc_max_bound_vec = [1.35] * len(time_vec)
    
    if len(jp_in.ego_acc_min) > 0:
        acc_min_bound_vec = list(jp_in.ego_acc_min)[:len(time_vec)]
        while len(acc_min_bound_vec) < len(time_vec):
            acc_min_bound_vec.append(acc_min_bound_vec[-1] if acc_min_bound_vec else -4.5)
    else:
        acc_min_bound_vec = [-4.5] * len(time_vec)

    # 获取加加速度边界参数（数组形式）
    if len(jp_in.ego_jerk_max) > 0:
        jerk_max_bound_vec = list(jp_in.ego_jerk_max)[:len(time_vec)]
        while len(jerk_max_bound_vec) < len(time_vec):
            jerk_max_bound_vec.append(jerk_max_bound_vec[-1] if jerk_max_bound_vec else 5.0)
    else:
        jerk_max_bound_vec = [5.0] * len(time_vec)
    
    if len(jp_in.ego_jerk_min) > 0:
        jerk_min_bound_vec = list(jp_in.ego_jerk_min)[:len(time_vec)]
        while len(jerk_min_bound_vec) < len(time_vec):
            jerk_min_bound_vec.append(jerk_min_bound_vec[-1] if jerk_min_bound_vec else -6.0)
    else:
        jerk_min_bound_vec = [-6.0] * len(time_vec)

    joint_plan_data['data_joint_motion_plan'].data.update({
        'time_vec': time_vec, 'x_vec': x_vec, 'y_vec': y_vec,
        'x_local_vec': x_vec_local, 'y_local_vec': y_vec_local,
        'theta_vec': theta_vec,
        'delta_vec': delta_vec,
        'vel_vec': vel_vec, 'acc_vec': acc_vec, 'jerk_vec': jerk_vec,
        'omega_vec': omega_vec,
        's_vec': list(jp_out.s_vec),
        'ref_x_vec': ref_x_vec, 'ref_y_vec': ref_y_vec,
        'ref_x_local_vec': ref_x_vec_local, 'ref_y_local_vec': ref_y_vec_local,
        'ref_theta_vec': ref_theta_vec,
        'ref_delta_vec': ref_delta_vec,
        'ref_vel_vec': ref_vel_vec, 'ref_acc_vec': ref_acc_vec,
        'ref_s_vec': ref_s_vec,
        # 添加加速度边界数据
        'acc_max_bound': acc_max_bound_vec,
        'acc_min_bound': acc_min_bound_vec,
        'acc_max_bound_vec': acc_max_bound_vec,
        'acc_min_bound_vec': acc_min_bound_vec,
        # 添加速度边界数据
        'vel_max_bound_vec': vel_max_bound_vec,
        'vel_min_bound_vec': vel_min_bound_vec,
        # 添加jerk边界数据
        'jerk_max_bound_vec': jerk_max_bound_vec,
        'jerk_min_bound_vec': jerk_min_bound_vec,
        # 添加分段轨迹数据 - 自车act轨迹
        'ego_act_seg0_x': ego_act_segments['seg_0_x'],
        'ego_act_seg0_y': ego_act_segments['seg_0_y'],
        'ego_act_seg1_x': ego_act_segments['seg_1_x'],
        'ego_act_seg1_y': ego_act_segments['seg_1_y'],
        'ego_act_seg2_x': ego_act_segments['seg_2_x'],
        'ego_act_seg2_y': ego_act_segments['seg_2_y'],
        'ego_act_seg3_x': ego_act_segments['seg_3_x'],
        'ego_act_seg3_y': ego_act_segments['seg_3_y'],
        'ego_act_seg4_x': ego_act_segments['seg_4_x'],
        'ego_act_seg4_y': ego_act_segments['seg_4_y']
    })


    joint_plan_data['data_text'].data.update({
        'VisionLonAttr': planning_json_value_list,
        'VisionLonVal': vision_lon_attr_vec
    })


def load_joint_planner_cost_time_figure(bag_loader):
    """Create cost time figure for joint decision pipeline."""
    cost_time_fig = bkp.figure(
        title='Joint Decision Pipeline Cost Time',
        x_axis_label='time/s',
        y_axis_label='time cost/(ms)',
        width=1200, height=280)

    # Initialize cost vectors - 5 dimensions like Joint Planner
    joint_motion_planner_cost_vec = []
    key_agent_num_vec = []
    lat_motion_cost_vec = []
    lon_motion_cost_vec = []
    environmental_cost_vec = []
    dynamic_world_cost_vec = []

    for ind in range(len(bag_loader.plan_debug_msg['json'])):
        plan_json = bag_loader.plan_debug_msg['json'][ind]
        joint_motion_planner_cost_vec.append(
            round(plan_json.get('LatLonJointDecisionTime', 0.0), 2))
        key_agent_num_vec.append(plan_json.get('key_agent_num', 0))
        environmental_cost_vec.append(
            round(plan_json.get('EnvironmentalModelManagerCost', 0.0), 2))
        dynamic_world_cost_vec.append(
            round(plan_json.get('dynamic_world_cost', 0.0), 2))
        lat_motion_cost_vec.append(
            round(plan_json.get('LateralMotionCostTime', 0.0), 2))
        lon_motion_cost_vec.append(
            round(plan_json.get('SccLonMotionCostTime', 0.0), 2))

    t_plan_debug = bag_loader.plan_debug_msg['t']

    # Calculate and print average cost times
    if len(t_plan_debug) > 0:
        lat_motion_average_cost = sum(lat_motion_cost_vec) / len(t_plan_debug)
        lon_motion_average_cost = sum(lon_motion_cost_vec) / len(t_plan_debug)
        lat_lon_decision_average_cost = sum(joint_motion_planner_cost_vec) / len(t_plan_debug)
        environmental_average_cost = sum(environmental_cost_vec) / len(t_plan_debug)
        dynamic_world_average_cost = sum(dynamic_world_cost_vec) / len(t_plan_debug)
        
        print('lat_motion_average_cost', lat_motion_average_cost)
        print('lon_motion_average_cost', lon_motion_average_cost)
        print('lat_lon_decision_average_cost', lat_lon_decision_average_cost)
        print('Environmental_average_cost', environmental_average_cost)
        print('dynamic_world_average_cost', dynamic_world_average_cost)

    cost_time_data_source = ColumnDataSource(data={
        't_plan_debug': t_plan_debug,
        'joint_motion_planner_cost': joint_motion_planner_cost_vec,
        'key_agent_num': key_agent_num_vec
    })

    if len(t_plan_debug) > 0:
        cost_time_fig.x_range.start = t_plan_debug[0]
        cost_time_fig.x_range.end = t_plan_debug[-1]

    # Plot joint planner cost
    total_cost_renderer = cost_time_fig.line(
        't_plan_debug', 'joint_motion_planner_cost',
        source=cost_time_data_source, line_width=3, line_color='black',
        legend_label='JointDecision')

    # Plot key agent number
    key_agent_num_renderer = cost_time_fig.line(
        't_plan_debug', 'key_agent_num',
        source=cost_time_data_source, line_width=2, line_color='magenta',
        line_dash='dotted', y_range_name='key_agent_num',
        legend_label='KeyAgentNum')

    # Add secondary y-axis for key agent number
    cost_time_fig.extra_y_ranges = {'key_agent_num': Range1d(start=0, end=5)}
    cost_time_fig.add_layout(
        LinearAxis(y_range_name='key_agent_num', axis_label='Key Agent Number'), 'right')

    # Add statistics lines
    if joint_motion_planner_cost_vec:
        max_total_cost = max(joint_motion_planner_cost_vec)
        avg_total_cost = sum(joint_motion_planner_cost_vec) / len(joint_motion_planner_cost_vec)

        cost_time_fig.line(
            [t_plan_debug[0], t_plan_debug[-1]], [max_total_cost, max_total_cost],
            line_width=2, line_color='red', line_dash='dashed', line_alpha=0.8,
            legend_label=f'Max({max_total_cost:.1f}ms)')

        cost_time_fig.line(
            [t_plan_debug[0], t_plan_debug[-1]], [avg_total_cost, avg_total_cost],
            line_width=2, line_color='blue', line_dash='dotted', line_alpha=0.8,
            legend_label=f'Avg({avg_total_cost:.1f}ms)')

    cost_time_fig.legend.click_policy = 'hide'
    cost_time_fig.legend.location = "top_left"
    cost_time_fig.legend.orientation = "horizontal"
    cost_time_fig.legend.label_text_font_size = '10pt'

    # Add hover tool
    hover_total_cost = HoverTool(
        tooltips=[
            ('Time', '@t_plan_debug{0.00}s'),
            ('JointDecision', '@joint_motion_planner_cost{0.00}ms'),
            ('KeyAgentNum', '@key_agent_num')
        ],
        mode='vline',
        renderers=[total_cost_renderer])
    cost_time_fig.add_tools(hover_total_cost)

    cost_time_fig.toolbar.active_scroll = cost_time_fig.select_one(WheelZoomTool)

    return cost_time_fig, cost_time_data_source


def load_joint_plan_figure(fig1, bag_loader):
    """Build figures and data sources for joint planner visualization."""

    # Initialize data sources
    data_text = ColumnDataSource(data={'VisionLonAttr': [], 'VisionLonVal': []})
    data_road_boundary = ColumnDataSource(data={
        'left_x': [], 'left_y': [], 'right_x': [], 'right_y': []})
    data_ego_three_disc = ColumnDataSource(data={
        'center_x': [], 'center_y': [], 'center_x_local': [], 'center_y_local': [],
        'radius': [],
        'circle_x': [], 'circle_y': [], 'circle_x_local': [], 'circle_y_local': [],
        'decision_label': []})

    # Initialize ego trajectory data source
    base_fields = [
        ('time', []), ('x', []), ('y', []), ('theta', []), ('delta', []),
        ('vel', []), ('acc', []), ('jerk', []), ('omega', []), ('s', []),
        ('ref_x', []), ('ref_y', []), ('ref_theta', []), ('ref_delta', []),
        ('ref_x_local', []), ('ref_y_local', []), ('x_local', []), ('y_local', []),
        ('ref_vel', []), ('ref_acc', []), ('ref_s', []),
        ('acc_max_bound', []), ('acc_min_bound', []),
        ('vel_max_bound', []), ('vel_min_bound', []),
        ('jerk_max_bound', []), ('jerk_min_bound', []),
    ]

    data_joint = {f'{k}_vec': v.copy() for k, v in base_fields}
    
    
    # 添加五段圆绘制的分段数据字段（自车实际轨迹）
    for i in range(5):
        data_joint[f'ego_act_seg{i}_x'] = []
        data_joint[f'ego_act_seg{i}_y'] = []
    
    data_joint_motion_plan = ColumnDataSource(data=data_joint)

    # Initialize obstacle data sources
    data_obs_ref = []
    data_obs_opt = []
    data_obs_three_disc = []
    for i in range(10):
        # 障碍物参考轨迹数据源
        obs_ref_data = {
            'time_vec': [], 'ref_x_vec': [], 'ref_y_vec': [],
            'ref_x_local_vec': [], 'ref_y_local_vec': [],
            'ref_theta_vec': [], 'ref_delta_vec': [],
            'ref_vel_vec': [], 'ref_acc_vec': [], 'ref_s_vec': [],
            'obs_id': [], 'longitudinal_label': []
        }
        data_obs_ref.append(ColumnDataSource(data=obs_ref_data))

        # 障碍物输出轨迹数据源
        obs_opt_data = {
            'time_vec': [], 'x_vec': [], 'y_vec': [],
            'x_local_vec': [], 'y_local_vec': [],
            'theta_vec': [], 'delta_vec': [], 'omega_vec': [],
            'vel_vec': [], 'acc_vec': [], 'jerk_vec': [], 's_vec': [],
            'obs_id': []
        }
        # 为障碍物实际轨迹添加五段圆分段字段
        for j in range(5):
            obs_opt_data[f'obs_opt_seg{j}_x'] = []
            obs_opt_data[f'obs_opt_seg{j}_y'] = []
        data_obs_opt.append(ColumnDataSource(data=obs_opt_data))

        data_obs_three_disc.append(ColumnDataSource(data={
            'center_x': [], 'center_y': [], 'center_x_local': [], 'center_y_local': [],
            'radius': [], 'obs_id': [],
            'circle_x': [], 'circle_y': [], 'circle_x_local': [], 'circle_y_local': [],
            'decision_label': []
        }))

    joint_plan_data = {
        'data_joint_motion_plan': data_joint_motion_plan,
        'data_text': data_text,
        'data_road_boundary': data_road_boundary,
        'data_ego_three_disc': data_ego_three_disc,
    }

    # Add obstacle data sources to joint_plan_data
    for i in range(10):
        joint_plan_data[f'data_obs_ref_{i}'] = data_obs_ref[i]
        joint_plan_data[f'data_obs_opt_{i}'] = data_obs_opt[i]
        joint_plan_data[f'data_obs_three_disc_{i}'] = data_obs_three_disc[i]

    # Add key agents data source
    data_key_agents = ColumnDataSource(data={
        'x_local': [], 'y_local': [], 'agent_id': []
    })
    joint_plan_data['data_key_agents'] = data_key_agents

    # Add planning info data source - vertical layout with labels and values (仅优化器相关)
    data_planning_info = ColumnDataSource(data={
        'labels': ['Total Decision Time', 'Obstacle Selection Time', 'Optimization Time',
                   'Solver Condition', 'Planning Condition', 'Rear Agent Label', 'Rear Agent Confidence'],
        'values': ['', '', '', '', '', '', '']
    })
    joint_plan_data['data_planning_info'] = data_planning_info

    # Plot on fig1 (local view)
    fig1.line('left_y', 'left_x', source=data_road_boundary,
              line_width=4, line_color="darkorange", line_dash='solid', line_alpha=0.7,
              legend_label='left_road_boundary')
    fig1.line('right_y', 'right_x', source=data_road_boundary,
              line_width=4, line_color="maroon", line_dash='solid', line_alpha=0.7,
              legend_label='right_road_boundary')


    # 自车参考轨迹 - 使用线条绘制（实线）
    ego_ref_renderer = fig1.line('ref_y_local_vec', 'ref_x_local_vec', source=data_joint_motion_plan,
                     line_width=5, line_color='purple', line_alpha=0.6,
                     line_dash='solid', legend_label='ego_ref_traj', visible=False)  # 初始不可见

    # 使用五段空心圆绘制自车实际轨迹（实线边框）- 多色系高对比度
    ego_act_colors = ['purple', 'magenta', 'orange', 'green', 'cyan']
    ego_act_circle_renderers = []
    for i in range(5):
        # 所有段都使用相同的图例标签，空心圆样式，实线边框
        act_renderer = fig1.circle(f'ego_act_seg{i}_y', f'ego_act_seg{i}_x', source=data_joint_motion_plan,
                      radius=0.4, color=ego_act_colors[i], alpha=0.9, line_width=2,
                      fill_alpha=0,  # 空心圆
                      line_dash='solid',  # 实线边框
                      legend_label='ego_opt_traj')
        ego_act_circle_renderers.append(act_renderer)

    # Plot key agents
    key_agent_labels = fig1.text('x_local', 'y_local', text='agent_id',
                                 source=data_key_agents, text_font_size='12pt',
                                 text_color='black', text_align='center',
                                 text_baseline='middle')

    # Plot ego three-disc
    ego_circle_renderer = fig1.multi_line('circle_y_local', 'circle_x_local',
                                         source=data_ego_three_disc,
                                         line_width=2, line_color='red', line_alpha=0.6,
                                         legend_label='ego_three_disc')

    ego_hover = HoverTool(
        tooltips=[('Center X', '@center_x_local{0.00}'), ('Center Y', '@center_y_local{0.00}')],
        renderers=[ego_circle_renderer],
        name='ego_disc_hover')
    fig1.add_tools(ego_hover)

    # Plot obstacle trajectories and three-discs
    colors = ['purple', 'orange', 'cyan', 'magenta', 'brown', 'pink', 'gray', 'yellow', 'navy', 'lime']
    obs_circle_renderers = []

    for i in range(10):
        circle_renderer = fig1.multi_line('circle_y_local', 'circle_x_local',
                                         source=data_obs_three_disc[i],
                                         line_width=2, line_color=colors[i], line_alpha=0.6,
                                         legend_label='obs_multi_disc')
        obs_circle_renderers.append(circle_renderer)

        obs_disc_hover = HoverTool(
            tooltips=[('Obs ID', '@obs_id'),
                     ('Center X', '@center_x_local{0.00}'), ('Center Y', '@center_y_local{0.00}'),
                     ('Longitudinal Label', '@decision_label')],
            renderers=[circle_renderer],
            name=f'obs_{i}_disc_hover')
        fig1.add_tools(obs_disc_hover)

    # 为每个障碍物创建单独的渲染器
    obs_ref_renderers = []
    obs_opt_renderers = []

    for i in range(10):
        # 障碍物参考轨迹 - 使用线条绘制（虚线）
        ref_renderer = fig1.line('ref_y_local_vec', 'ref_x_local_vec',
                                source=data_obs_ref[i],
                                line_width=5, line_color=colors[i], line_dash='dotted', line_alpha=0.6,
                                legend_label='obs_ref_traj')
        obs_ref_renderers.append(ref_renderer)

        # 障碍物输出轨迹 - 使用五段空心圆绘制，所有段都使用相同图例标签（实线边框）
        obs_opt_segment_renderers = []
        for j in range(5):
            opt_segment_renderer = fig1.circle(f'obs_opt_seg{j}_y', f'obs_opt_seg{j}_x',
                     source=data_obs_opt[i],
                     radius=0.4, color=colors[i], alpha=0.9, line_width=2,
                     fill_alpha=0,  # 空心圆
                     line_dash='solid',  # 实线边框
                     legend_label='obs_opt_traj')
            obs_opt_segment_renderers.append(opt_segment_renderer)
        obs_opt_renderers.append(obs_opt_segment_renderers)

        # 为每个障碍物创建独立的hover tool
        obs_hover = HoverTool(
            tooltips=[
                ('obs_id', '@obs_id')
            ],
            mode='vline',
            renderers=[ref_renderer],  # 使用参考轨迹渲染器作为hover目标
            name=f'obs_{i}_hover')
        fig1.add_tools(obs_hover)



    # Create sub-figures
    fig3 = bkp.figure(x_axis_label='time', y_axis_label='theta',
                      x_range=[-0.1, 5.5], width=525, height=330)

    fig4 = bkp.figure(x_axis_label='time', y_axis_label='delta',
                      x_range=fig3.x_range, y_range=[-0.1, 0.1], width=525, height=330)

    fig5 = bkp.figure(x_axis_label='time', y_axis_label='omega',
                      x_range=fig4.x_range, y_range=[-0.2, 0.2], width=525, height=330)

    fig6 = bkp.figure(x_axis_label='time', y_axis_label='vel',
                      x_range=fig5.x_range, width=525, height=330)

    fig7 = bkp.figure(x_axis_label='time', y_axis_label='acc',
                      x_range=fig6.x_range, width=525, height=330)

    fig8 = bkp.figure(x_axis_label='time', y_axis_label='jerk',
                      x_range=fig7.x_range, width=525, height=330)

    fig9 = bkp.figure(x_axis_label='time', y_axis_label='s',
                      x_range=fig8.x_range, width=525, height=330)

    # Create planning info table - vertical layout with labels and values (仅优化器相关)
    planning_info_table = DataTable(
        source=data_planning_info,
        columns=[
            TableColumn(field='labels', title='Label', width=180),
            TableColumn(field='values', title='Value', width=340)
        ],
        width=520, height=220, index_position=None,  # 调整高度适应6行数据
        fit_columns=True,
        sortable=False,
        reorderable=False
    )



    # Plot theta on fig3
    theta_plan_renderer = fig3.line('time_vec', 'theta_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='blue', line_dash='solid')
    theta_ref_renderer = fig3.line('time_vec', 'ref_theta_vec', source=data_joint_motion_plan,
                                  line_width=2, line_color='red', line_dash='dashed')

    fig3.legend.visible = False

    obs_ref_renderers = []
    obs_opt_renderers = []
    for i in range(10):
        ref_theta_renderer = fig3.line('time_vec', 'ref_theta_vec',
                                      source=data_obs_ref[i],
                                      line_width=2, line_color=colors[i], line_dash='dotted')
        obs_ref_renderers.append(ref_theta_renderer)

        opt_theta_renderer = fig3.line('time_vec', 'theta_vec',
                                         source=data_obs_opt[i],
                                         line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_renderers.append(opt_theta_renderer)

        theta_hover = HoverTool(
            tooltips=[('obs_id', '@obs_id')],
            mode='vline',
            renderers=[ref_theta_renderer],
            name=f'obs_{i}_theta_hover')
        fig3.add_tools(theta_hover)

    # Combined legend for fig3
    fig3_combined_legend = Legend(items=[
        ('ego_theta_opt', [theta_plan_renderer]),
        ('ego_theta_ref', [theta_ref_renderer]),
        ('obs_theta_ref', obs_ref_renderers),
        ('obs_theta_opt', obs_opt_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig3.add_layout(fig3_combined_legend)
    fig3_combined_legend.click_policy = 'hide'

    # Plot delta on fig4
    delta_plan_renderer = fig4.line('time_vec', 'delta_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='blue', line_dash='solid')
    delta_ref_renderer = fig4.line('time_vec', 'ref_delta_vec', source=data_joint_motion_plan,
                                  line_width=2, line_color='red', line_dash='dashed')

    fig4.legend.visible = False

    obs_ref_renderers = []
    obs_opt_renderers = []
    for i in range(10):
        ref_delta_renderer = fig4.line('time_vec', 'ref_delta_vec',
                                      source=data_obs_ref[i],
                                      line_width=2, line_color=colors[i], line_dash='dotted')
        obs_ref_renderers.append(ref_delta_renderer)

        opt_delta_renderer = fig4.line('time_vec', 'delta_vec',
                                       source=data_obs_opt[i],
                                       line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_renderers.append(opt_delta_renderer)

        delta_hover = HoverTool(
            tooltips=[('obs_id', '@obs_id')],
            mode='vline',
            renderers=[ref_delta_renderer],
            name=f'obs_{i}_delta_hover')
        fig4.add_tools(delta_hover)

    # Combined legend for fig4
    fig4_combined_legend = Legend(items=[
        ('ego_delta_opt', [delta_plan_renderer]),
        ('ego_delta_ref', [delta_ref_renderer]),
        ('obs_delta_ref', obs_ref_renderers),
        ('obs_delta_opt', obs_opt_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig4.add_layout(fig4_combined_legend)
    fig4_combined_legend.click_policy = 'hide'

    # Plot omega on fig5
    omega_plan_renderer = fig5.line('time_vec', 'omega_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='blue', line_dash='solid')

    fig5.legend.visible = False

    obs_opt_renderers = []
    for i in range(10):
        opt_omega_renderer = fig5.line('time_vec', 'omega_vec',
                                       source=data_obs_opt[i],
                                       line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_renderers.append(opt_omega_renderer)

        omega_hover = HoverTool(
            tooltips=[('obs_id', '@obs_id')],
            mode='vline',
            renderers=[opt_omega_renderer],
            name=f'obs_{i}_omega_hover')
        fig5.add_tools(omega_hover)

    # Combined legend for fig5
    fig5_combined_legend = Legend(items=[
        ('ego_omega_opt', [omega_plan_renderer]),
        ('obs_omega_opt', obs_opt_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig5.add_layout(fig5_combined_legend)
    fig5_combined_legend.click_policy = 'hide'

    # Plot velocity on fig6
    ego_ref_vel_renderer = fig6.line('time_vec', 'ref_vel_vec', source=data_joint_motion_plan,
                                     line_width=2, line_color='red', line_dash='dashed')
    plan_vel_renderer = fig6.line('time_vec', 'vel_vec', source=data_joint_motion_plan,
                                  line_width=2, line_color='blue', line_dash='solid')
    
    # Plot velocity bounds with lines and triangles
    vel_max_bound_line = fig6.line('time_vec', 'vel_max_bound_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='gray', line_dash='dotted',
                                   line_alpha=0.8)
    
    vel_min_bound_line = fig6.line('time_vec', 'vel_min_bound_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='gray', line_dash='dotted',
                                   line_alpha=0.8)
    
    vel_max_bound_renderer = fig6.inverted_triangle('time_vec', 'vel_max_bound_vec', source=data_joint_motion_plan,
                                                    size=8, color='gray', alpha=0.7,
                                                    legend_label='vel max bound')

    vel_min_bound_renderer = fig6.triangle('time_vec', 'vel_min_bound_vec', source=data_joint_motion_plan,
                                           size=8, color='gray', alpha=0.7,
                                           legend_label='vel min bound')

    fig6.legend.visible = False

    obs_ref_renderers = []
    obs_opt_renderers = []
    for i in range(10):
        ref_vel_renderer = fig6.line('time_vec', 'ref_vel_vec',
                                    source=data_obs_ref[i],
                                    line_width=2, line_color=colors[i], line_dash='dotted')
        obs_ref_renderers.append(ref_vel_renderer)

        opt_vel_renderer = fig6.line('time_vec', 'vel_vec',
                                       source=data_obs_opt[i],
                                       line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_renderers.append(opt_vel_renderer)

        vel_hover = HoverTool(
            tooltips=[('obs_id', '@obs_id')],
            mode='vline',
            renderers=[ref_vel_renderer],
            name=f'obs_{i}_vel_hover')
        fig6.add_tools(vel_hover)

    # Combined legend for fig6
    fig6_combined_legend = Legend(items=[
        ('ego_vel_ref', [ego_ref_vel_renderer]),
        ('ego_vel_opt', [plan_vel_renderer]),
        ('vel_max_bound', [vel_max_bound_renderer]),
        ('vel_min_bound', [vel_min_bound_renderer]),
        ('obs_vel_ref', obs_ref_renderers),
        ('obs_vel_opt', obs_opt_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig6.add_layout(fig6_combined_legend)
    fig6_combined_legend.click_policy = 'hide'

    # Plot acceleration on fig7
    ego_ref_acc_renderer = fig7.line('time_vec', 'ref_acc_vec', source=data_joint_motion_plan,
                                     line_width=2, line_color='red', line_dash='dashed')
    plan_acc_renderer = fig7.line('time_vec', 'acc_vec', source=data_joint_motion_plan,
                                  line_width=2, line_color='blue', line_dash='solid')
    
    # Plot acceleration bounds with lines and triangles
    acc_max_bound_line = fig7.line('time_vec', 'acc_max_bound_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='gray', line_dash='dotted',
                                   line_alpha=0.8)
    
    acc_min_bound_line = fig7.line('time_vec', 'acc_min_bound_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='gray', line_dash='dotted',
                                   line_alpha=0.8)
    
    acc_max_bound_renderer = fig7.inverted_triangle('time_vec', 'acc_max_bound_vec', source=data_joint_motion_plan,
                                                    size=8, color='gray', alpha=0.7,
                                                    legend_label='acc max bound')

    acc_min_bound_renderer = fig7.triangle('time_vec', 'acc_min_bound_vec', source=data_joint_motion_plan,
                                           size=8, color='gray', alpha=0.7,
                                           legend_label='acc min bound')
    

    fig7.legend.visible = False

    obs_ref_renderers = []
    obs_opt_renderers = []
    for i in range(10):
        ref_acc_renderer = fig7.line('time_vec', 'ref_acc_vec',
                                    source=data_obs_ref[i],
                                    line_width=2, line_color=colors[i], line_dash='dotted')
        obs_ref_renderers.append(ref_acc_renderer)

        opt_acc_renderer = fig7.line('time_vec', 'acc_vec',
                                       source=data_obs_opt[i],
                                       line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_renderers.append(opt_acc_renderer)

    # Combined legend for fig7
    fig7_combined_legend = Legend(items=[
        ('ego_acc_ref', [ego_ref_acc_renderer]),
        ('ego_acc_opt', [plan_acc_renderer]),
        ('acc_max_bound', [acc_max_bound_renderer]),
        ('acc_min_bound', [acc_min_bound_renderer]),
        ('obs_acc_ref', obs_ref_renderers),
        ('obs_acc_opt', obs_opt_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig7.add_layout(fig7_combined_legend)
    fig7_combined_legend.click_policy = 'hide'

    # Plot jerk on fig8
    jerk_plan_renderer = fig8.line('time_vec', 'jerk_vec', source=data_joint_motion_plan,
                                  line_width=2, line_color='blue', line_dash='solid')

    # Plot jerk bounds with lines and triangles
    jerk_max_bound_line = fig8.line('time_vec', 'jerk_max_bound_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='gray', line_dash='dotted',
                                   line_alpha=0.8)
    
    jerk_min_bound_line = fig8.line('time_vec', 'jerk_min_bound_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='gray', line_dash='dotted',
                                   line_alpha=0.8)
    
    jerk_max_bound_renderer = fig8.inverted_triangle('time_vec', 'jerk_max_bound_vec', source=data_joint_motion_plan,
                                                    size=8, color='gray', alpha=0.7,
                                                    legend_label='jerk max bound')

    jerk_min_bound_renderer = fig8.triangle('time_vec', 'jerk_min_bound_vec', source=data_joint_motion_plan,
                                           size=8, color='gray', alpha=0.7,
                                           legend_label='jerk min bound')

    fig8.legend.visible = False

    obs_opt_renderers = []
    for i in range(10):
        opt_jerk_renderer = fig8.line('time_vec', 'jerk_vec',
                                      source=data_obs_opt[i],
                                      line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_renderers.append(opt_jerk_renderer)

        jerk_hover = HoverTool(
            tooltips=[('obs_id', '@obs_id')],
            mode='vline',
            renderers=[opt_jerk_renderer],
            name=f'obs_{i}_jerk_hover')
        fig8.add_tools(jerk_hover)

    # Combined legend for fig8
    fig8_combined_legend = Legend(items=[
        ('ego_jerk_opt', [jerk_plan_renderer]),
        ('jerk_max_bound', [jerk_max_bound_renderer]),
        ('jerk_min_bound', [jerk_min_bound_renderer]),
        ('obs_jerk_opt', obs_opt_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig8.add_layout(fig8_combined_legend)
    fig8_combined_legend.click_policy = 'hide'

    # Plot s-t on fig9
    ego_ref_s_renderer = fig9.line('time_vec', 'ref_s_vec', source=data_joint_motion_plan,
                                   line_width=2, line_color='red', line_dash='dashed')
    plan_s_renderer = fig9.line('time_vec', 's_vec', source=data_joint_motion_plan,
                                line_width=2, line_color='blue', line_dash='solid')

    fig9.legend.visible = False

    obs_ref_s_renderers = []
    obs_opt_s_renderers = []
    for i in range(10):
        ref_s_renderer = fig9.line('time_vec', 'ref_s_vec',
                                   source=data_obs_ref[i],
                                   line_width=2, line_color=colors[i], line_dash='dotted')
        obs_ref_s_renderers.append(ref_s_renderer)

        opt_s_renderer = fig9.line('time_vec', 's_vec',
                                   source=data_obs_opt[i],
                                   line_width=2, line_color=colors[i], line_dash='solid')
        obs_opt_s_renderers.append(opt_s_renderer)

        s_hover = HoverTool(
            tooltips=[('obs_id', '@obs_id')],
            mode='vline',
            renderers=[ref_s_renderer],
            name=f'obs_{i}_s_hover')
        fig9.add_tools(s_hover)

    # Combined legend for fig9
    fig9_combined_legend = Legend(items=[
        ('ego_s_ref', [ego_ref_s_renderer]),
        ('ego_s_opt', [plan_s_renderer]),
        ('obs_s_ref', obs_ref_s_renderers),
        ('obs_s_opt', obs_opt_s_renderers)
    ], location="top_right", orientation="vertical",
       label_text_font_size='10pt', label_text_align='left',
       glyph_height=10, glyph_width=18)
    fig9.add_layout(fig9_combined_legend)
    fig9_combined_legend.click_policy = 'hide'


    # Set toolbar and legend behavior
    for f in (fig3, fig4, fig5, fig6, fig7, fig8, fig9):
        f.toolbar.active_scroll = f.select_one(WheelZoomTool)
        f.legend.click_policy = 'hide'

    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'

    # 移除 omega、delta、theta 图表，保留 vel、acc、jerk、s 和优化器信息表
    pan1 = Panel(child=row(column(fig9, fig6, fig7),
                              column(fig8, planning_info_table)),
                title="Longtime")

    pans = Tabs(tabs=[pan1])

    return pans, joint_plan_data


