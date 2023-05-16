import sys
import numpy as np
import math

def load_car_params_patch():
    car_x = [4.015, 4.015, -1.083, -1.083, 4.015]
    car_y = [0.98, -0.98, -0.98, 0.98, 0.98]
    return car_x, car_y

def load_silder_params(kwargs, mpc_parameters):
    mpc_parameters.curv_factor_gain_ = kwargs["curv_factor_gain"]
    mpc_parameters.lat_q0_ = kwargs["lat_q0"]
    mpc_parameters.lat_q1_ = kwargs["lat_q1"]
    mpc_parameters.lat_r_ = kwargs["lat_r"]

def load_mpc_input(RefOutput, mpc_state, mpc_input):
    mpc_input.q_s_ = 0.1
    mpc_input.q_y_ = 1.0
    mpc_input.q_phi_ = 1.0
    mpc_input.q_v_ = 10.0
    mpc_input.q_jerk_ = 0.5
    mpc_input.q_omega_ = 1.0

    mpc_input.jerk_min_ = -5.0
    mpc_input.jerk_max_ = 5.0
    mpc_input.v_min_ = -10.0
    mpc_input.v_max_ = 33.3
    mpc_input.delta_limit_ = 1.0
    mpc_input.omega_limit_ = 1.0
    mpc_input.dx_max_ = 2000.0
    mpc_input.dy_min_ = -2000.0
    mpc_input.dy_max_ = 2000.0

    mpc_input.init_state_ = mpc_state
    mpc_input.dx_ref_mpc_vec_   = RefOutput.dx_ref_mpc_vec_
    mpc_input.dy_ref_mpc_vec_   = RefOutput.dy_ref_mpc_vec_
    mpc_input.dphi_ref_mpc_vec_ = RefOutput.dphi_ref_mpc_vec_
    mpc_input.vel_ref_mpc_vec_  = RefOutput.vel_ref_mpc_vec_
    mpc_input.acc_ref_mpc_vec_  = RefOutput.acc_ref_mpc_vec_

def load_mpc_state(mpc_state):
    mpc_state.dx_ = 0.0
    mpc_state.dy_ = 0.0
    mpc_state.dphi_ = 0.0
    mpc_state.delta_ = 0.0
    mpc_state.v_ = 10.0
    mpc_state.a_ = 0.5


# def load_mpc_state(config_data, mpc_state):

#     mpc_state.Ad_ = np.array([[1.0, 1.0, 0.125], [0.0, 1.0, 0.25], [0.0, 0.0, 1.0]])
#     mpc_state.Bd_ = np.array([[0.0], [0.0], [0.1]])
#     mpc_state.mpc_horizon_ = config_data["mpc_horizon"]
#     mpc_state.mpc_fs_ = config_data["mpc_fs"]

#     mpc_state.Q_ = np.array([config_data["lat_q0"], config_data["lat_q1"], 0.0])
#     mpc_state.R_ = np.array([config_data["lat_r"]])

#     mpc_state.x_max_ = np.array([config_data["lat_dy_limit"], config_data["lat_dphi_limit"], config_data["lat_steering_angle_limit"] / 14.8])
#     mpc_state.x_min_ = np.array([-config_data["lat_dy_limit"], -config_data["lat_dphi_limit"], -config_data["lat_steering_angle_limit"] / 14.8])

#     mpc_state.u_max_ = np.array([config_data["lat_steering_angle_rate_limit"] / 14.8])
#     mpc_state.u_min_ = np.array([-config_data["lat_steering_angle_rate_limit"] / 14.8])

#     mpc_state.x0_ = np.array([0.0, 0.0, 0.0])
#     mpc_state.u0_ = np.array([0.0])

#     time_vec_mpc = []
#     for i in range(mpc_state.mpc_horizon_ + 1):
#         time_vec_mpc.append(i * 1.0 / mpc_state.mpc_fs_)
#     mpc_state.time_vec_mpc_ = time_vec_mpc

#     return True
def load_ref_params(bag_data, mpc_reference):
    mpc_reference.dx_ref_vec_ = bag_data["dx_ref_vec"]
    mpc_reference.dy_ref_vec_ = bag_data["dy_ref_vec"]
    mpc_reference.dphi_ref_vec_ = bag_data["dphi_ref_vec"]

def load_mpc_parameters(config_data, bag_data, mpc_parameters):
    mpc_parameters.curv_factor_k1_ = config_data["curv_factor_k1"]
    mpc_parameters.curv_factor_k2_ = config_data["curv_factor_k2"]
    mpc_parameters.lat_mpc_vel_min_ = config_data["vel_min"]
    mpc_parameters.lat_acc_y_limit_ = config_data["lat_acc_y_limit"]
    mpc_parameters.lat_jerk_y_limit_ = config_data["lat_jerk_y_limit"]
    mpc_parameters.steer_ratio_ = config_data["steer_ratio"]
    mpc_parameters.lat_dy_limit_ = config_data["lat_dy_limit"]
    mpc_parameters.lat_dphi_limit_ = config_data["lat_dphi_limit"]
    mpc_parameters.lat_steering_angle_limit_ = config_data["lat_steering_angle_limit"]
    mpc_parameters.lat_steering_angle_rate_limit_ = config_data["lat_steering_angle_rate_limit"]
    mpc_parameters.lat_actuator_delay_ = config_data["lat_actuator_delay"]
    mpc_parameters.fs_ = config_data["control_fs"]
    mpc_parameters.lat_ctrl_index_ = config_data["lat_ctrl_index"]

    mpc_parameters.vel_ = bag_data["vel"]
    mpc_parameters.wheel_angle_ = bag_data["wheel_angle"]
    mpc_parameters.vel_ref_ = bag_data["vel_ref"]
    mpc_parameters.reverse_flag_ = bag_data["reverse_flag"]
    mpc_parameters.wheel_angle_cmd_ =  bag_data["wheel_angle_cmd"]
    mpc_parameters.curv_factor_gain_ = config_data["curv_factor_gain"]
    mpc_parameters.lat_q0_ = config_data["lat_q0"]
    mpc_parameters.lat_q1_ = config_data["lat_q1"]
    mpc_parameters.lat_r_ = config_data["lat_r"]
    mpc_parameters.weight_keep_flag_ = False;
    mpc_parameters.low_curv_thresh_ = config_data["low_curv_thresh"];
    mpc_parameters.high_curv_thresh_ = config_data["high_curv_thresh"];
    mpc_parameters.q0_high_curv_gain_ = config_data["q0_high_curv_gain"];
    mpc_parameters.q1_high_curv_gain_ = config_data["q1_high_curv_gain"];
    mpc_parameters.q0_large_dy_gain_ = config_data["q0_large_dy_gain"];
    mpc_parameters.R_high_curv_gain_ = config_data["R_high_curv_gain"];
    mpc_parameters.q0_high_vel_gain_ = config_data["q0_high_vel_gain"];
    mpc_parameters.q1_high_vel_gain_ = config_data["q1_high_vel_gain"];
    mpc_parameters.q1_low_vel_gain_ = config_data["q1_low_vel_gain"];
    mpc_parameters.q0_max_ = config_data["q0_max"];
    mpc_parameters.low_speed_jerk_y_limit_ = config_data["low_speed_jerk_y_limit"];

def load_obstacle_params(obstacle_list):
    obs_info = {'obstacles_x': [],
                'obstacles_y': [],
                'pos_x': [],
                'pos_y': [],
                'obstacles_vel': [],
                'obstacles_acc': [],
                'obstacles_tid': [],
                'is_cipv': [],
                'obs_label':[]
                }
    obs_num = len(obstacle_list)
    num = 0
    for i in range(obs_num):
      if obstacle_list[i].additional_info.fusion_source == 2:
        continue
      if obstacle_list[i].common_info.relative_position.x == 0 and obstacle_list[i].common_info.relative_position.y == 0:
        continue
      elif obstacle_list[i].common_info.shape.width == 0 or obstacle_list[i].common_info.shape.length == 0:
        continue
      else:
        long_pos = obstacle_list[i].common_info.relative_position.x
        lat_pos = obstacle_list[i].common_info.relative_position.y
        theta = obstacle_list[i].common_info.relative_heading_angle
        half_width = obstacle_list[i].common_info.shape.width /2
        half_length = obstacle_list[i].common_info.shape.length / 2
        
        cos_heading = math.cos(theta)
        sin_heading = math.sin(theta)
        dx1 = cos_heading * half_length
        dy1 = sin_heading * half_length
        dx2 = sin_heading * half_width
        dy2 = -cos_heading * half_width
        
        obs_x = [long_pos + dx1 + dx2,
                 long_pos + dx1 - dx2,
                 long_pos - dx1 - dx2,
                 long_pos - dx1 + dx2,
                 long_pos + dx1 + dx2]
        obs_y = [lat_pos + dy1 + dy2,
                 lat_pos + dy1 - dy2,
                 lat_pos - dy1 - dy2,
                 lat_pos - dy1 + dy2,
                 lat_pos + dy1 + dy2]
        
        num = num + 1
        # for ind in range(len(obs_x)):
        obs_info['obstacles_x'].append(obs_x)
        # for ind in range(len(obs_y)):
        obs_info['obstacles_y'].append(obs_y)
        obs_info['pos_x'].append(long_pos)
        obs_info['pos_y'].append(lat_pos)
        obs_info['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
        obs_info['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
        obs_info['obstacles_tid'].append(obstacle_list[i].common_info.id)
#             obs_info['is_cipv'].append(obstacle_list[i].target_selection_type)
        obs_info['obs_label'].append(str(obstacle_list[i].common_info.id) + ',v=' + str(round(obstacle_list[i].common_info.relative_velocity.x, 2)))
    return obs_info

def gen_line(c0, c1, c2, c3, start, end):
  points_x = []
  points_y = []

  for x in np.linspace(start, end, 50):
      y = c0 + c1 * x + c2 * x * x + c3 * x * x* x
      points_x.append([x])
      points_y.append([y])

  return points_x, points_y
  
def load_lane_lines(lanes):
  line_info_list = []

  for i in range(6):
    lane_info_l = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
    if i< len(lanes):
        lane = lanes[i]
        left_line = lane.left_lane_boundary
        left_line_coef = left_line.poly_coefficient
        line_x, line_y = gen_line(left_line_coef[0], left_line_coef[1], left_line_coef[2], left_line_coef[3], \
          left_line.begin, left_line.end)

        lane_info_l['line_x_vec'] = line_x
        lane_info_l['line_y_vec'] = line_y
        lane_info_l['type'] = left_line.segment[0].type

        line_info_list.append(lane_info_l)

        lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
        right_line = lane.right_lane_boundary
        right_line_coef = right_line.poly_coefficient
        line_x, line_y = gen_line(right_line_coef[0], right_line_coef[1], right_line_coef[2], right_line_coef[3], \
          right_line.begin, right_line.end)
        lane_info_r['line_x_vec'] = line_x
        lane_info_r['line_y_vec'] = line_y
        lane_info_r['type'] = left_line.segment[0].type
        line_info_list.append(lane_info_r)
    else:
        line_x, line_y = gen_line(0,0,0,0,0,0)
        lane_info_l['line_x_vec'] = line_x
        lane_info_l['line_y_vec'] = line_y
        lane_info_l['type'] = []
        line_info_list.append(lane_info_l)
        line_info_list.append(lane_info_l)

  return line_info_list  