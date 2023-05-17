import sys
import numpy as np
import math

def load_car_params_patch():
  car_x = [4.015, 4.015, -1.083, -1.083, 4.015]
  car_y = [0.98, -0.98, -0.98, 0.98, 0.98]
  return car_x, car_y

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
      lane_info_r['type'] = right_line.segment[0].type
      line_info_list.append(lane_info_r)
    else:
      line_x, line_y = gen_line(0,0,0,0,0,0)
      lane_info_l['line_x_vec'] = line_x
      lane_info_l['line_y_vec'] = line_y
      lane_info_l['type'] = []
      line_info_list.append(lane_info_l)
      line_info_list.append(lane_info_l)

  return line_info_list

def load_obstacle_params(obstacle_list):
  obs_info = {
    'obstacles_x': [],
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
      obs_info['obs_label'].append('v(' + str(obstacle_list[i].common_info.id) + ')=' \
          + str(round(obstacle_list[i].common_info.relative_velocity.x, 2)))
  return obs_info

def gen_line(c0, c1, c2, c3, start, end):
  points_x = []
  points_y = []

  for x in np.linspace(start, end, 50):
      y = c0 + c1 * x + c2 * x * x + c3 * x * x* x
      points_x.append([x])
      points_y.append([y])

  return points_x, points_y
