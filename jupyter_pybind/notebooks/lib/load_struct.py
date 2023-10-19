import sys
import numpy as np
import math
from lib.load_rotate import *

def load_car_params_patch():
  car_x = [4.015, 4.015, -1.083, -1.083, 4.015]
  car_y = [0.98, -0.98, -0.98, 0.98, 0.98]
  return car_x, car_y

def ehr_load_center_lane_lines(lanes,x,y,yaw,Max_line_size):
  ehr_line_info_list = []
  for i in range(Max_line_size):
    ehr_lane_info = {'ehr_line_x_vec':[], 'ehr_line_y_vec':[],'ehr_relative_id':[], 'ehr_type':[]}
    if i < len(lanes):
      lane = lanes[i]
      line_x = []
      line_y = []
      cur_line_first_point = lane.points_on_central_line[0]
      cur_line_last_point = lane.points_on_central_line[-1]
      first_point_to_cur_dis = math.sqrt((cur_line_first_point.x - x)**2 + (cur_line_first_point.y - y)**2)
      last_point_to_cur_dis = math.sqrt((cur_line_last_point.x - x)**2 + (cur_line_last_point.y - y)**2)
      # if ((first_point_to_cur_dis > 1000) & (last_point_to_cur_dis>1000)):
      #   continue
      for point in lane.points_on_central_line:
        ehr_x = point.x
        ehr_y = point.y
        car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
        # print("x:",ehr_x)
        # print("y:",ehr_y)
        line_x.append(car_x)
        line_y.append(car_y)
      ehr_lane_info['ehr_line_x_vec'] = line_x
      ehr_lane_info['ehr_line_y_vec'] = line_y
      ehr_lane_info['ehr_relative_id'] = lane.lane_id
      ehr_lane_info['ehr_type'] = 0
      ehr_line_info_list.append(ehr_lane_info)
    else:
      line_x, line_y = gen_line(0,0,0,0,0,0)
      ehr_lane_info['ehr_line_x_vec'] = line_x
      ehr_lane_info['ehr_line_y_vec'] = line_y
      ehr_lane_info['ehr_relative_id'] = 1000
      ehr_lane_info['ehr_type'] = 0
      ehr_line_info_list.append(ehr_lane_info)
  return ehr_line_info_list

def ehr_load_road_boundary_lines(road_boundaries,x,y,yaw,Road_boundary_max_line_size):
  ehr_road_boundary_info_list = []
  for i in range(Road_boundary_max_line_size):
    ehr_road_boundary_info = {'ehr_road_boundary_x_vec':[], 'ehr_road_boundary_y_vec':[],'ehr_road_boundary_relative_id':[], 'ehr_type':[]}
    if i < len(road_boundaries):
      road_boundary = road_boundaries[i]
      line_x = []
      line_y = []
      # cur_line_first_point = road_boundary.points_on_central_line[0]
      # cur_line_last_point = road_boundary.points_on_central_line[-1]
      # first_point_to_cur_dis = math.sqrt((cur_line_first_point.x - x)**2 + (cur_line_first_point.y - y)**2)
      # last_point_to_cur_dis = math.sqrt((cur_line_last_point.x - x)**2 + (cur_line_last_point.y - y)**2)
      # # if ((first_point_to_cur_dis > 1000) & (last_point_to_cur_dis>1000)):
      # #   continue
      for doundary_attribute in road_boundary.boundary_attributes:
        for  point in doundary_attribute.points:
          ehr_x = point.x
          ehr_y = point.y
          car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
          # print("x:",ehr_x)
          # print("y:",ehr_y)
          line_x.append(car_x)
          line_y.append(car_y)
      ehr_road_boundary_info['ehr_road_boundary_x_vec'] = line_x
      ehr_road_boundary_info['ehr_road_boundary_y_vec'] = line_y
      ehr_road_boundary_info['ehr_road_boundary_relative_id'] = road_boundary.boundary_id
      ehr_road_boundary_info['ehr_type'] = 0
      ehr_road_boundary_info_list.append(ehr_road_boundary_info)
    else:
      line_x, line_y = gen_line(0,0,0,0,0,0)
      ehr_road_boundary_info['ehr_road_boundary_x_vec'] = line_x
      ehr_road_boundary_info['ehr_road_boundary_y_vec'] = line_y
      ehr_road_boundary_info['ehr_road_boundary_relative_id'] = 1000
      ehr_road_boundary_info['ehr_type'] = 0
      ehr_road_boundary_info_list.append(ehr_road_boundary_info)
  return ehr_road_boundary_info_list

def ehr_load_lane_boundary_lines(lane_boundaries,x,y,yaw,Lane_boundary_max_line_size):
  ehr_lane_boundary_info_list = []

  for i in range(Lane_boundary_max_line_size):
    ehr_lane_boundary_info = {'ehr_lane_boundary_x_vec':[], 'ehr_lane_boundary_y_vec':[],'ehr_lane_boundary_relative_id':[], 'ehr_type':[]}
    if i < len(lane_boundaries):
      lane_boundary = lane_boundaries[i]
      line_x = []
      line_y = []
      for boundary_attribute in lane_boundary.boundary_attributes:
        for  point in boundary_attribute.points:
          ehr_x = point.x
          ehr_y = point.y
          car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
          # print("x:",ehr_x)
          # print("y:",ehr_y)
          line_x.append(car_x)
          line_y.append(car_y)
      ehr_lane_boundary_info['ehr_lane_boundary_x_vec'] = line_x
      ehr_lane_boundary_info['ehr_lane_boundary_y_vec'] = line_y
      ehr_lane_boundary_info['ehr_lane_boundary_relative_id'] = lane_boundary.boundary_id
      ehr_lane_boundary_info['ehr_type'] = 0
      ehr_lane_boundary_info_list.append(ehr_lane_boundary_info)
    else:
      line_x, line_y = gen_line(0,0,0,0,0,0)
      ehr_lane_boundary_info['ehr_lane_boundary_x_vec'] = line_x
      ehr_lane_boundary_info['ehr_lane_boundary_y_vec'] = line_y
      ehr_lane_boundary_info['ehr_lane_boundary_relative_id'] = 1000
      ehr_lane_boundary_info['ehr_type'] = 0
      ehr_lane_boundary_info_list.append(ehr_lane_boundary_info)
  return ehr_lane_boundary_info_list

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
      lane_info_l['type'] = left_line.type_segments[0].type

      line_info_list.append(lane_info_l)

      lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
      right_line = lane.right_lane_boundary
      right_line_coef = right_line.poly_coefficient
      line_x, line_y = gen_line(right_line_coef[0], right_line_coef[1], right_line_coef[2], right_line_coef[3], \
        right_line.begin, right_line.end)

      lane_info_r['line_x_vec'] = line_x
      lane_info_r['line_y_vec'] = line_y
      lane_info_r['type'] = right_line.type_segments[0].type
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

  for i in range(10):
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