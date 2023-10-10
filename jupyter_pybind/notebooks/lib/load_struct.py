import sys
import numpy as np
import math
from lib.load_rotate import *

def load_car_params_patch():
  # car_x = [3.624, 3.624, -0.947, -0.947, 3.624]
  # car_y = [1.89*0.5, -1.89*0.5, -1.89*0.5, 1.89*0.5, 1.89*0.5]
  # return car_x, car_y
  car_x = [3.187342, 3.424531, 3.593071,  3.593071,  3.424531,  3.187342,   2.177994,  1.916421,  1.96496, -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357, 1.96496, 1.916421, 2.177994]
  car_y = [0.887956, 0.681712, 0.334651, -0.334651, -0.681712, -0.887956,  -0.887956, -1.06715, -0.887956, -0.887956, -0.706505, -0.334845,  0.334845,  0.706505,  0.887956, 0.887956, 1.06715, 0.887956]
  return car_x, car_y

def load_car_circle_coord():
  # circle_x = [3.3, 3.3, 2.0, -0.6, -0.6, 2.0, 2.7, 1.8, 0.9, 0.0]
  # circle_y = [-0.55, 0.55, 0.85, 0.55, -0.55, -0.85, 0.0, 0.0, 0.0, 0.0]
  # circle_r = [0.35, 0.35, 0.25, 0.35, 0.35, 0.25, 0.95, 0.95, 0.95, 0.95]
  # circle_x = [1.35, 3.2, 3.2, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0]
  # circle_y = [0.0, -0.5, 0.5, 0.95, 0.5, -0.5, -0.95, 0.0, 0.0, 0.0, 0.0]
  # circle_r = [2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95]
  
  circle_x = [1.35, 3.3, 3.3, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0]
  circle_y = [0.0, -0.55, 0.55, 0.95, 0.5, -0.5, -0.95, 0.0, 0.0, 0.0, 0.0]
  circle_r = [2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95]
  return circle_x, circle_y, circle_r

def load_car_uss_patch():
  apa_x = [3.187342, 3.424531, 3.593071,  3.593071,  3.424531,  3.187342, -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357]
  apa_y = [0.887956, 0.681712, 0.334651, -0.334651, -0.681712, -0.887956, -0.887956, -0.706505, -0.334845,  0.334845,  0.706505,  0.887956]
  return apa_x, apa_y

def load_uss_angle_patch():
   uss_angle = [170, 130, 92, 88, 50, 8, 352, 298, 275, 264, 242, 187]
   return uss_angle

def one_echo_text_local(old_x, old_y, radian, distance):
    new_x = old_x + distance * math.cos(radian)
    new_y = old_y + distance * math.sin(radian)
    return new_x, new_y

def load_lane_lines(lanes):
  line_info_list = []

  for i in range(10):
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

  return line_info_list

def load_lane_center_lines(lanes):
  line_info_list = []

  for i in range(5):
    lane_info = {'line_x_vec':[], 'line_y_vec':[], 'relative_id':[],'type':[]}
    if i< len(lanes):
      lane = lanes[i]
      virtual_lane_refline_points = lane.lane_reference_line.virtual_lane_refline_points
      line_x = []
      line_y = []
      for virtual_lane_refline_point in virtual_lane_refline_points:
        line_x.append(virtual_lane_refline_point.car_point.x)
        line_y.append(virtual_lane_refline_point.car_point.y)

      lane_info['line_x_vec'] = line_x
      lane_info['line_y_vec'] = line_y
      lane_info['relative_id'] = lane.relative_id
      lane_info['type'] = 0

      line_info_list.append(lane_info)
    else:
      line_x, line_y = gen_line(0,0,0,0,0,0)
      lane_info['line_x_vec'] = line_x
      lane_info['line_y_vec'] = line_y
      lane_info['relative_id'] = 1000
      lane_info['type'] = 0
      line_info_list.append(lane_info)

  return line_info_list

def load_obstacle_paramsV1(obstacle_list):

  obs_info_all = dict()

  obs_num = len(obstacle_list)
  num = 0
  for i in range(obs_num):
    source = obstacle_list[i].additional_info.fusion_source
    if source & 0x01: #相机融合障碍物
      source = 1
    elif (obstacle_list[i].common_info.relative_center_position.x > 0 and \
      math.tan(25) > math.fabs(obstacle_list[i].common_info.relative_center_position.y / obstacle_list[i].common_info.relative_center_position.x)) or \
      math.fabs(obstacle_list[i].common_info.relative_center_position.y) > 10:
      continue
    else:
      source = 4
    if (source in obs_info_all.keys()) == False:
      obs_info = {
        'obstacles_x_rel': [],
        'obstacles_y_rel': [],
        'obstacles_x': [],
        'obstacles_y': [],
        'pos_x_rel': [],
        'pos_y_rel': [],
        'pos_x': [],
        'pos_y': [],
        'obstacles_vel': [],
        'obstacles_acc': [],
        'obstacles_tid': [],
        'is_cipv': [],
        'obs_label':[]
      }
      obs_info_all[source] = obs_info

    long_pos_rel = obstacle_list[i].common_info.relative_center_position.x
    lat_pos_rel = obstacle_list[i].common_info.relative_center_position.y
    theta = obstacle_list[i].common_info.relative_heading_angle
    if theta == 255:
      theta = 0
    half_width = obstacle_list[i].common_info.shape.width /2
    half_length = obstacle_list[i].common_info.shape.length / 2
    # if half_width == 0 or half_length == 0:
    #   continue
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width

    obs_x_rel = [long_pos_rel + dx1 + dx2,
              long_pos_rel + dx1 - dx2,
              long_pos_rel- dx1 - dx2,
              long_pos_rel - dx1 + dx2,
              long_pos_rel + dx1 + dx2]
    obs_y_rel = [lat_pos_rel + dy1 + dy2,
              lat_pos_rel + dy1 - dy2,
              lat_pos_rel - dy1 - dy2,
              lat_pos_rel - dy1 + dy2,
              lat_pos_rel + dy1 + dy2]

    # 绝对坐标系下的数据
    long_pos = obstacle_list[i].common_info.center_position.x
    lat_pos = obstacle_list[i].common_info.center_position.y
    theta = obstacle_list[i].common_info.heading_angle
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
    obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all[source]['pos_x_rel'].append(long_pos_rel)
    obs_info_all[source]['pos_y_rel'].append(lat_pos_rel)
    obs_info_all[source]['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
    obs_info_all[source]['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
    obs_info_all[source]['obstacles_tid'].append(obstacle_list[i].additional_info.track_id)
#             fusion_obs_info['is_cipv'].append(obstacle_list[i].target_selection_type)
    obs_info_all[source]['obs_label'].append('v(' + str(obstacle_list[i].additional_info.track_id) + ')=' \
        + str(round(obstacle_list[i].common_info.relative_velocity.x, 2))+','+ str(round(obstacle_list[i].common_info.relative_velocity.y, 2)))
    obs_info_all[source]['obstacles_x'].append(obs_x)
    # for ind in range(len(obs_y)):
    obs_info_all[source]['obstacles_y'].append(obs_y)
    obs_info_all[source]['pos_x'].append(long_pos)
    obs_info_all[source]['pos_y'].append(lat_pos)

  return obs_info_all

def load_obstacle_me(obstacle_list):

  obs_info_all = dict()

  obs_num = len(obstacle_list)
  num = 0
  for i in range(obs_num):
    source = 1#obstacle_list[i].additional_info.sensor_type
    # if source & 0x01: #相机融合障碍物
    #   source = 1
    # elif (obstacle_list[i].common_info.relative_center_position.x > 0 and \
    #   math.tan(25) > math.fabs(obstacle_list[i].common_info.relative_center_position.y / obstacle_list[i].common_info.relative_center_position.x)) or \
    #   math.fabs(obstacle_list[i].common_info.relative_center_position.y) > 10:
    #   continue
    # else:
    #   source = 4
    if (source in obs_info_all.keys()) == False:
      obs_info = {
        'obstacles_x_rel': [],
        'obstacles_y_rel': [],
        'obstacles_x': [],
        'obstacles_y': [],
        'pos_x_rel': [],
        'pos_y_rel': [],
        'pos_x': [],
        'pos_y': [],
        'obstacles_vel': [],
        'obstacles_acc': [],
        'obstacles_tid': [],
        'is_cipv': [],
        'obs_label':[]
      }
    obs_info_all[source] = obs_info

    long_pos_rel = obstacle_list[i].common_info.relative_position.x
    # print(long_pos_rel)
    lat_pos_rel = obstacle_list[i].common_info.relative_position.y
    theta = obstacle_list[i].common_info.relative_heading_angle
    if theta == 255:
      theta = 0
    half_width = obstacle_list[i].common_info.shape.width /2
    half_length = obstacle_list[i].common_info.shape.length / 2
    # if half_width == 0 or half_length == 0:
    #   continue
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width

    obs_x_rel = [long_pos_rel + dx1 + dx2,
              long_pos_rel + dx1 - dx2,
              long_pos_rel- dx1 - dx2,
              long_pos_rel - dx1 + dx2,
              long_pos_rel + dx1 + dx2]
    obs_y_rel = [lat_pos_rel + dy1 + dy2,
              lat_pos_rel + dy1 - dy2,
              lat_pos_rel - dy1 - dy2,
              lat_pos_rel - dy1 + dy2,
              lat_pos_rel + dy1 + dy2]
    # print(obs_x_rel)
    # 绝对坐标系下的数据
    long_pos = obstacle_list[i].common_info.center_position.x
    lat_pos = obstacle_list[i].common_info.center_position.y
    theta = obstacle_list[i].common_info.heading_angle
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
    obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all[source]['pos_x_rel'].append(long_pos_rel)
    obs_info_all[source]['pos_y_rel'].append(lat_pos_rel)
    obs_info_all[source]['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
    obs_info_all[source]['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
    obs_info_all[source]['obstacles_tid'].append(obstacle_list[i].common_info.id)#contour不太确定
#             fusion_obs_info['is_cipv'].append(obstacle_list[i].target_selection_type)
    obs_info_all[source]['obs_label'].append('v(' + str(obstacle_list[i].common_info.id) + ')=' \
        + str(round(obstacle_list[i].common_info.relative_velocity.x, 2))+','+ str(round(obstacle_list[i].common_info.relative_velocity.y, 2)))
    obs_info_all[source]['obstacles_x'].append(obs_x)
    # for ind in range(len(obs_y)):
    obs_info_all[source]['obstacles_y'].append(obs_y)
    obs_info_all[source]['pos_x'].append(long_pos_rel)
    obs_info_all[source]['pos_y'].append(lat_pos_rel)
    # print("me_message:(",obstacle_list[i].common_info.relative_position.x-10200,",",obstacle_list[i].common_info.relative_position.y+5000,")")

  return obs_info_all

def load_obstacle_radar(obstacle_list,type):
  obs_info_all = dict()

  obs_num = len(obstacle_list)
  # print("obs_num:",obs_num)
  # print(obs_num)
  num = 0
  if type == 0:
    source = 11
  elif type == 1:
    source = 12
  elif type == 2:
    source = 13
  elif type == 3:
    source = 14
  elif type == 4:
    source = 15
  # print("source:",source)
  for i in range(obs_num):
    # source = obstacle_list[i].additional_info.fusion_source
    # if source & 0x01: #相机融合障碍物
    #   source = 1
    # elif (obstacle_list[i].common_info.relative_center_position.x > 0 and \
    #   math.tan(25) > math.fabs(obstacle_list[i].common_info.relative_center_position.y / obstacle_list[i].common_info.relative_center_position.x)) or \
    #   math.fabs(obstacle_list[i].common_info.relative_center_position.y) > 10:
    #   continue
    # else:

    if (source in obs_info_all.keys()) == False:
      obs_info = {
        'obstacles_x_rel': [],
        'obstacles_y_rel': [],
        'obstacles_x': [],
        'obstacles_y': [],
        'pos_x_rel': [],
        'pos_y_rel': [],
        'pos_x': [],
        'pos_y': [],
        'obstacles_vel': [],
        'obstacles_acc': [],
        'obstacles_tid': [],
        'is_cipv': [],
        'obs_label':[]
      }
      obs_info_all[source] = obs_info

    long_pos_rel = obstacle_list[i].relative_position.x
    lat_pos_rel = obstacle_list[i].relative_position.y
    theta = obstacle_list[i].relative_heading_angle
    if theta == 255:
      theta = 0
    half_width = obstacle_list[i].shape.width /2
    half_length = obstacle_list[i].shape.length / 2
    # if half_width == 0 or half_length == 0:
    #   continue
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width

    obs_x_rel = [long_pos_rel + dx1 + dx2,
              long_pos_rel + dx1 - dx2,
              long_pos_rel- dx1 - dx2,
              long_pos_rel - dx1 + dx2,
              long_pos_rel + dx1 + dx2]
    obs_y_rel = [lat_pos_rel + dy1 + dy2,
              lat_pos_rel + dy1 - dy2,
              lat_pos_rel - dy1 - dy2,
              lat_pos_rel - dy1 + dy2,
              lat_pos_rel + dy1 + dy2]

    #print(obs_x_rel)
    # 绝对坐标系下的数据
    long_pos = obstacle_list[i].relative_position.x
    lat_pos = obstacle_list[i].relative_position.y
    theta = 0 #obstacle_list[i].heading_angle
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
    obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all[source]['pos_x_rel'].append(long_pos_rel)
    obs_info_all[source]['pos_y_rel'].append(lat_pos_rel)
    obs_info_all[source]['obstacles_vel'].append(obstacle_list[i].relative_velocity.x)
    obs_info_all[source]['obstacles_acc'].append(obstacle_list[i].relative_acceleration.x)
    obs_info_all[source]['obstacles_tid'].append(obstacle_list[i].id)
#             fusion_obs_info['is_cipv'].append(obstacle_list[i].target_selection_type)
    obs_info_all[source]['obs_label'].append('v(' + str(obstacle_list[i].id) + ')=' \
        + str(round(obstacle_list[i].relative_velocity.x, 2))+','+ str(round(obstacle_list[i].relative_velocity.y, 2)))
    obs_info_all[source]['obstacles_x'].append(obs_x)
    # for ind in range(len(obs_y)):
    obs_info_all[source]['obstacles_y'].append(obs_y)
    obs_info_all[source]['pos_x'].append(long_pos)
    obs_info_all[source]['pos_y'].append(lat_pos)

  return obs_info_all

def gen_line(c0, c1, c2, c3, start, end):
  points_x = []
  points_y = []

  for x in np.linspace(start, end, 50):
      y = c0 + c1 * x + c2 * x * x + c3 * x * x* x
      points_x.append(x)
      points_y.append(y)

  return points_x, points_y

def load_prediction_objects(obstacle_list, localization_info):
    obs_info = {'obstacles_x': [],
                'obstacles_y': [],
                'pos_x': [],
                'pos_y': [],
                'loc_x': [],
                'loc_y': [],
                'obstacles_vel': [],
                'obstacles_acc': [],
                'obstacles_tid': [],
                'is_cipv': [],
                'obs_label':[]
                }
    localization_x = 0
    localization_y = 0
    if localization_info.pose.type == 1:
      localization_x = localization_info.pose.enu_position.x
      localization_y = localization_info.pose.enu_position.y
    elif localization_info.pose.type == 2:
      localization_x = localization_info.pose.llh_position.x
      localization_y = localization_info.pose.llh_position.y
    elif localization_info.pose.type == 3:
      localization_x = localization_info.pose.local_position.x
      localization_y = localization_info.pose.local_position.y
    elif localization_info.pose.type == 0:
      localization_x = localization_info.pose.local_position.x
      localization_y = localization_info.pose.local_position.y
    linear_velocity_from_wheel = localization_info.pose.linear_velocity_from_wheel
    localization_theta = localization_info.pose.euler_angles.yaw

    trajectory_info = {'x':[],'y':[]}
    obs_num = len(obstacle_list)
    num = len(obstacle_list[0].trajectory[0].trajectory_point)
    for i in range(obs_num):
      if len(obstacle_list[i].trajectory[0].trajectory_point) == 0:
        # print("No data")
        continue
      elif obstacle_list[i].fusion_obstacle.common_info.shape.width == 0 or obstacle_list[i].fusion_obstacle.common_info.shape.length == 0:
        continue
      else:
        long_pos = obstacle_list[i].trajectory[0].trajectory_point[0].relative_position.x
        lat_pos = obstacle_list[i].trajectory[0].trajectory_point[0].relative_position.y
        theta = obstacle_list[i].fusion_obstacle.common_info.relative_heading_angle
        half_width = obstacle_list[i].fusion_obstacle.common_info.shape.width /2
        length = obstacle_list[i].fusion_obstacle.common_info.shape.length
        track_id = obstacle_list[i].fusion_obstacle.additional_info.track_id

        # print(f"long_pos = {long_pos}\tlat_pos = {lat_pos}\ttheta = {theta}\thalf_width = {half_width}\tlength = {length}\n")

        obs_x = [long_pos+half_width*math.sin(theta), \
                  long_pos+half_width*math.sin(theta)+length*math.cos(theta), \
                  long_pos-half_width*math.sin(theta)+length*math.cos(theta), \
                  long_pos-half_width*math.sin(theta), \
                  long_pos+half_width*math.sin(theta)]

        obs_y = [lat_pos-half_width*math.cos(theta), \
                  lat_pos-half_width*math.cos(theta)+ length*math.sin(theta), \
                  lat_pos+half_width*math.cos(theta)+ length*math.sin(theta), \
                  lat_pos+half_width*math.cos(theta),
                  lat_pos-half_width*math.cos(theta)]
        num = num + 1
        obs_info['obstacles_x'].append(obs_x)
        obs_info['obstacles_y'].append(obs_y)
        obs_info['pos_x'].append(lat_pos)
        obs_info['pos_y'].append(long_pos)
        obs_info['obstacles_vel'].append(obstacle_list[i].fusion_obstacle.common_info.relative_velocity.x)
        obs_info['obstacles_acc'].append(obstacle_list[i].fusion_obstacle.common_info.relative_acceleration.x)
        obs_info['obstacles_tid'].append(obstacle_list[i].fusion_obstacle.common_info.id)
        obs_info['obs_label'].append(str(obstacle_list[i].fusion_obstacle.common_info.id) + ',v=' + str(round(obstacle_list[i].fusion_obstacle.common_info.relative_velocity.x, 2)))

        p_x = []
        p_y = []
        for j in range(len(obstacle_list[i].trajectory[0].trajectory_point)):
          # local_x = obstacle_list[i].trajectory[0].trajectory_point[j].relative_position.x
          # local_y = obstacle_list[i].trajectory[0].trajectory_point[j].relative_position.y
          global_x = obstacle_list[i].trajectory[0].trajectory_point[j].position.x
          global_y = obstacle_list[i].trajectory[0].trajectory_point[j].position.y
          local_x, local_y = global2local(global_x, global_y, localization_x, localization_y, localization_theta)
          p_x.append(local_x)
          p_y.append(local_y)
        # trajectory_info[track_id] = [p_x, p_y]
      trajectory_info['x'].append(p_x)
      trajectory_info['y'].append(p_y)

    return obs_info, trajectory_info