import sys
import numpy as np
import math
from lib.load_rotate import *

kMapRange = 500
only_display_map_in_route = False
load_center_line_in_poly = False

def load_car_params_patch():
  # car_x = [3.624, 3.624, -0.947, -0.947, 3.624]
  # car_y = [1.89*0.5, -1.89*0.5, -1.89*0.5, 1.89*0.5, 1.89*0.5]
  # return car_x, car_y
  car_x = [3.187342, 3.424531, 3.593071,  3.593071,  3.424531,  3.187342,   2.177994,  1.916421,  1.96496, -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357, 1.96496, 1.916421, 2.177994]
  car_y = [0.887956, 0.681712, 0.334651, -0.334651, -0.681712, -0.887956,  -0.887956, -1.06715, -0.887956, -0.887956, -0.706505, -0.334845,  0.334845,  0.706505,  0.887956, 0.887956, 1.06715, 0.887956]
  return car_x, car_y

JAC_S811 = 'JAC_S811'
CHERY_T26 = 'CHERY_T26'
CHERY_E0X = 'CHERY_E0X'

def load_car_params_patch_parking(vehicle_type = JAC_S811):

  if vehicle_type == JAC_S811:
    # for JAC_S811
    car_x = [3.424, 3.624, 3.624, 3.424, 2.177, 2.177, 1.916, 1.916, -0.747, -0.947, -0.947, -0.747, 1.916, 1.916, 2.177, 2.177]
    car_y = [0.945, 0.745, -0.745, -0.945, -0.945, -1.055, -1.055, -0.945, -0.945, -0.745, 0.745, 0.945, 0.945, 1.055, 1.055, 0.945]
  elif vehicle_type == CHERY_T26:
    # for CHERY_T26
    car_x = [3.518, 3.718, 3.718, 3.518, 2.092, 2.092, 1.906, 1.906, -0.885, -1.085, -1.085, -0.885, 1.906, 1.906, 2.092, 2.092]
    car_y = [0.9595, 0.7595, -0.7595, -0.9595, -0.9595, -1.092, -1.092, -0.9595, -0.9595, -0.7595, 0.7595, 0.9595, 0.9595, 1.092, 1.092, 0.9595]
  elif vehicle_type == CHERY_E0X:
    # for CHERY_E0X
    car_x = [3.724, 3.924, 3.924, 3.724, 2.234, 2.234, 2.034, 2.034, -0.825, -1.025, -1.025, -0.825, 2.034, 2.034, 2.234, 2.234]
    car_y = [0.9875, 0.7875, -0.7875, -0.9875, -0.9875, -1.1375, -1.1375, -0.9875, -0.9875, -0.7875, 0.7875, 0.9875, 0.9875, 1.1375, 1.1375, 0.9875]

  car_lat_inflation = 0.0986
  for i in range(len(car_x)):
    if car_y[i] > 0.0:
      car_y[i] = car_y[i] + car_lat_inflation
    else:
      car_y[i] = car_y[i] - car_lat_inflation

  return car_x, car_y

def load_car_circle_coord():
  # circle_x = [3.3, 3.3, 2.0, -0.6, -0.6, 2.0, 2.7, 1.8, 0.9, 0.0]
  # circle_y = [-0.55, 0.55, 0.85, 0.55, -0.55, -0.85, 0.0, 0.0, 0.0, 0.0]
  # circle_r = [0.35, 0.35, 0.25, 0.35, 0.35, 0.25, 0.95, 0.95, 0.95, 0.95]
  # circle_x = [1.35, 3.2, 3.2, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0]
  # circle_y = [0.0, -0.5, 0.5, 0.95, 0.5, -0.5, -0.95, 0.0, 0.0, 0.0, 0.0]
  # circle_r = [2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95]

  circle_x = [1.35, 3.3, 3.3, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0]
  circle_y = [0.0, 0.55, -0.55, -0.95, -0.5, 0.5, 0.95, 0.0, 0.0, 0.0, 0.0]
  circle_r = [2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95]

  return circle_x, circle_y, circle_r

def load_car_uss_patch(vehicle_type = JAC_S811):
  if vehicle_type == JAC_S811:
    # for JAC_S811
    apa_x = [3.187342, 3.424531, 3.593071, 3.593071, 3.424531, 3.187342,
            -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357]
    apa_y = [0.887956, 0.681712, 0.334651, -0.334651, -0.681712, -0.887956,
            -0.887956, -0.706505, -0.334845, 0.334845, 0.706505, 0.887956]
  elif vehicle_type == CHERY_T26:
   # for CHERY_T26
    apa_x = [3.298241, 3.580141, 3.667435, 3.667435, 3.580141, 3.298241,
            -0.602483, -0.997449, -1.06219, -1.06219, -0.997449, -0.602483]
    apa_y = [0.935328, 0.680863, 0.334976, -0.334976, -0.680863, -0.935328,
            -0.935328, -0.669815, -0.299949, 0.299949, 0.699815, 0.935328]
  elif vehicle_type == CHERY_E0X:
    apa_x = [3.4655,  3.7711,  3.8742, 3.8742, 3.7711,  3.4655,
            -0.5150, -0.8653, -1.0115, -1.0115, -0.8653, -0.5150]
    apa_y = [0.97620,  0.69918,  0.32000,  -0.32000, -0.66918, -0.97620,
            -0.9583, -0.8314, -0.3250, -0.3250,  0.8314,  0.9583]

  return apa_x, apa_y

def load_uss_angle_patch(vehicle_type = JAC_S811):
  if vehicle_type == JAC_S811:
    # for JAC_S811
    uss_angle = [170, 130, 92, 88, 50, 8, 352, 298, 275, 264, 242, 187]
  elif vehicle_type == CHERY_T26:
    # for CHERY_T26
    uss_angle = [169.998, 125.019, 97.046, 82.954, 54.981, 10.002,354.78, 298.086, 277.369, 262.631, 241.9914, 185.22]
  elif vehicle_type == CHERY_E0X:
    # for CHERY_E0X
    uss_angle = [169.998, 125.019, 97.046, 82.954, 54.981, 10.002,354.78, 298.086, 277.369, 262.631, 241.9914, 185.22]

  return uss_angle

def one_echo_text_local(old_x, old_y, radian, distance):
    new_x = old_x + distance * math.cos(radian)
    new_y = old_y + distance * math.sin(radian)
    return new_x, new_y
def ehr_load_center_lane_lines(lanes,x,y,yaw,Max_line_size,lane_id_in_route_set):
  ehr_line_info_list = []
  for i in range(Max_line_size):
    ehr_lane_info = {'ehr_line_x_vec':[], 'ehr_line_y_vec':[],'ehr_relative_id':[], 'ehr_type':[]}
    if i < len(lanes):
      if only_display_map_in_route and lanes[i].lane_id not in lane_id_in_route_set:
        continue
      lane = lanes[i]
      line_x = []
      line_y = []
      cur_line_first_point = lane.points_on_central_line[0]
      cur_line_last_point = lane.points_on_central_line[-1]
      first_point_to_cur_dis = math.sqrt((cur_line_first_point.x - x)**2 + (cur_line_first_point.y - y)**2)
      last_point_to_cur_dis = math.sqrt((cur_line_last_point.x - x)**2 + (cur_line_last_point.y - y)**2)
      if first_point_to_cur_dis > kMapRange and last_point_to_cur_dis > kMapRange:
        continue
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
      ehr_lane_info['ehr_line_x_vec'] = []
      ehr_lane_info['ehr_line_y_vec'] = []
      ehr_lane_info['ehr_relative_id'] = []
      ehr_lane_info['ehr_type'] = []
      ehr_line_info_list.append(ehr_lane_info)
  return ehr_line_info_list

def ehr_load_road_boundary_lines(road_boundaries,x,y,yaw,Road_boundary_max_line_size,road_boundary_id_in_route_set):
  ehr_road_boundary_info_list = []
  for i in range(Road_boundary_max_line_size):
    ehr_road_boundary_info = {'ehr_road_boundary_x_vec':[], 'ehr_road_boundary_y_vec':[],'ehr_road_boundary_relative_id':[], 'ehr_type':[]}
    if i < len(road_boundaries):
      if only_display_map_in_route and road_boundaries[i].boundary_id not in road_boundary_id_in_route_set:
        continue
      road_boundary = road_boundaries[i]
      line_x = []
      line_y = []
      cur_line_first_point = road_boundary.boundary_attributes[0].points[0]
      cur_line_last_point = road_boundary.boundary_attributes[-1].points[-1]
      first_point_to_cur_dis = math.sqrt((cur_line_first_point.x - x)**2 + (cur_line_first_point.y - y)**2)
      last_point_to_cur_dis = math.sqrt((cur_line_last_point.x - x)**2 + (cur_line_last_point.y - y)**2)
      if first_point_to_cur_dis > kMapRange and last_point_to_cur_dis > kMapRange:
        continue
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
      ehr_road_boundary_info['ehr_road_boundary_x_vec'] = []
      ehr_road_boundary_info['ehr_road_boundary_y_vec'] = []
      ehr_road_boundary_info['ehr_road_boundary_relative_id'] = []
      ehr_road_boundary_info['ehr_type'] = []
      ehr_road_boundary_info_list.append(ehr_road_boundary_info)
  return ehr_road_boundary_info_list

def ehr_load_lane_boundary_lines(lane_boundaries,x,y,yaw,Lane_boundary_max_line_size, lane_boundary_id_in_route_set):
  ehr_lane_boundary_info_list = []

  for i in range(Lane_boundary_max_line_size):
    ehr_lane_boundary_info = {'ehr_lane_boundary_x_vec':[], 'ehr_lane_boundary_y_vec':[],'ehr_lane_boundary_relative_id':[], 'ehr_type':[]}
    if i < len(lane_boundaries):
      if only_display_map_in_route and lane_boundaries[i].boundary_id not in lane_boundary_id_in_route_set:
        continue
      lane_boundary = lane_boundaries[i]
      cur_line_first_point = lane_boundary.boundary_attributes[0].points[0]
      cur_line_last_point = lane_boundary.boundary_attributes[-1].points[-1]
      first_point_to_cur_dis = math.sqrt((cur_line_first_point.x - x)**2 + (cur_line_first_point.y - y)**2)
      last_point_to_cur_dis = math.sqrt((cur_line_last_point.x - x)**2 + (cur_line_last_point.y - y)**2)
      if first_point_to_cur_dis > kMapRange and last_point_to_cur_dis > kMapRange:
        continue
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
      ehr_lane_boundary_info['ehr_lane_boundary_x_vec'] = []
      ehr_lane_boundary_info['ehr_lane_boundary_y_vec'] = []
      ehr_lane_boundary_info['ehr_lane_boundary_relative_id'] = []
      ehr_lane_boundary_info['ehr_type'] = []
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
      try:
        line_x, line_y = gen_line(left_line_coef[0], left_line_coef[1], left_line_coef[2], left_line_coef[3], \
          left_line.begin, left_line.end)
        lane_info_l['line_x_vec'] = line_x
        lane_info_l['line_y_vec'] = line_y
        try:
          tp = left_line.type_segments[0].type
        except:
          print("旧格式：左车道线类型")
          tp = left_line.segment[0].type
        if tp == 0 or tp == 1 or tp == 3 or tp == 4:
          lane_info_l['type'] = ['dashed']
        else:
          lane_info_l['type'] = ['solid']
      except:
        print("旧格式：左车道线信息")
        line_x, line_y = gen_line(0,0,0,0,0,0)
        lane_info_l['line_x_vec'] = line_x
        lane_info_l['line_y_vec'] = line_y
        lane_info_l['type'] = ['dashed']

      line_info_list.append(lane_info_l)

      lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
      right_line = lane.right_lane_boundary
      right_line_coef = right_line.poly_coefficient
      try:
        line_x, line_y = gen_line(right_line_coef[0], right_line_coef[1], right_line_coef[2], right_line_coef[3], \
          right_line.begin, right_line.end)
        lane_info_r['line_x_vec'] = line_x
        lane_info_r['line_y_vec'] = line_y
        try:
          tp = left_line.type_segments[0].type
        except:
          print("旧格式：右车道线类型")
          tp = left_line.segment[0].type
        if tp == 0 or tp == 1 or tp == 3 or tp == 4:
          lane_info_r['type'] = ['dashed']
        else:
          lane_info_r['type'] = ['solid']
      except:
        line_x, line_y = gen_line(0,0,0,0,0,0)
        lane_info_r['line_x_vec'] = line_x
        lane_info_r['line_y_vec'] = line_y
        lane_info_r['type'] = ['dashed']
        print("旧格式：右车道线信息")
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
      if not load_center_line_in_poly:
        for virtual_lane_refline_point in virtual_lane_refline_points:
          line_x.append(virtual_lane_refline_point.car_point.x)
          line_y.append(virtual_lane_refline_point.car_point.y)
      else:
        line_x, line_y = gen_line(lane.lane_reference_line.poly_coefficient_car[0], \
                                  lane.lane_reference_line.poly_coefficient_car[1], \
                                  lane.lane_reference_line.poly_coefficient_car[2], \
                                  lane.lane_reference_line.poly_coefficient_car[3], 0, 50)

      lane_info['line_x_vec'] = line_x
      lane_info['line_y_vec'] = line_y
      lane_info['relative_id'] = lane.relative_id
      lane_info['type'] = 0

      line_info_list.append(lane_info)
    else:
      lane_info['line_x_vec'] = []
      lane_info['line_y_vec'] = []
      lane_info['relative_id'] = []
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