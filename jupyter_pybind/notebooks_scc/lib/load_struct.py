import sys
import numpy as np
import math
from scipy.interpolate import interp1d
from scipy.misc import derivative
from lib.load_rotate import *
sys.path.append('../../python_proto')
from ehr_sdmap_pb2 import *
from bokeh.models import TextInput
import ipywidgets

def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False
def find(msg, t):
  if msg['enable']  == True:
    t_list = msg['timestamp']
    ind = binary_search(t_list, t)
    if(ind == -1):
      return None
    else:
      return msg['data'][ind]
  else:
    return None
def findbyseq(msg, t):
  try:
    if msg['enable']  == True:
      t_list = msg['seq']
      ind = binary_search(t_list, t)
      if(ind == -1):
        return None
      else:
        return msg['data'][ind]
    else:
      return None
  except:
    return None
def binary_search(in_list, target):
  low = 0
  high = len(in_list) - 1
  while low <= high:
      mid = (low + high) // 2
      if in_list[mid] == target:
          return mid
      elif in_list[mid] > target:
          high = mid - 1
      else:
          low = mid + 1
  return -1

def find_nearest(msg, bag_time, find_json = False):
  try:
    if msg['enable']  == True:
      msg_idx = 0
      while msg['t'][msg_idx] <= bag_time and msg_idx < (len(msg['t'])-2):
        msg_idx = msg_idx + 1
      if find_json:
        return msg['json'][msg_idx]
      else:
        return msg['data'][msg_idx]
    else:
      return None
  except:
    print("find nearest error!")
    return None

def load_car_params_patch(car_type = 'E0Y'):
  # car_x = [3.624, 3.624, -0.947, -0.947, 3.624]
  # car_y = [1.89*0.5, -1.89*0.5, -1.89*0.5, 1.89*0.5, 1.89*0.5]
  # return car_x, car_y
  if car_type == 'S811':
    car_x = [3.187342, 3.424531, 3.593071,  3.593071,  3.424531,  3.187342,   2.177994,  1.916421,  1.96496, -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357, 1.96496, 1.916421, 2.177994]
    car_y = [0.887956, 0.681712, 0.334651, -0.334651, -0.681712, -0.887956,  -0.887956, -1.06715, -0.887956, -0.887956, -0.706505, -0.334845,  0.334845,  0.706505,  0.887956, 0.887956, 1.06715, 0.887956]
  elif car_type == 'T26':
    car_x = [3.518, 3.718, 3.718, 3.518, 2.092, 2.092, 1.906, 1.906, -0.885, -1.085, -1.085, -0.885, 1.906, 1.906, 2.092, 2.092]
    car_y = [0.9595, 0.7595, -0.7595, -0.9595, -0.9595, -1.1095, -1.1095, -0.9595, -0.9595, -0.7595, 0.7595, 0.9595, 0.9595, 1.1095, 1.1095, 0.9595]
  elif car_type == 'E0Y':
    car_x = [3.825, 4.025, 4.025, 3.825, 2.399, 2.399, 2.213, 2.213, -0.725, -0.925, -0.925, -0.725, 2.213, 2.213, 2.399, 2.399]
    car_y = [0.9875, 0.7875, -0.7875, -0.9875, -0.9875, -1.1145, -1.1145, -.9875, -.9875, -0.7875, 0.7875, 0.9875, 0.9875, 1.1145, 1.1145, 0.9875]
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

def ehr_load_center_lane_lines(lanes, x, y, yaw, Max_line_size, g_is_display_enu = False):
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
        if g_is_display_enu:
          ehr_x, ehr_y = point.x, point.y
        else:
          ehr_x, ehr_y = global2local(point.x, point.y, x, y, yaw)
        # print("x:",ehr_x)
        # print("y:",ehr_y)
        line_x.append(ehr_x)
        line_y.append(ehr_y)
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

def load_sd_map_segments(segments,x,y,yaw,Max_sdmap_segment_size):
  sdmap_road_line_info = {'sdmap_road_line_x_vec':[], 'sdmap_road_line_y_vec':[],
                          'sdmap_ramp_line_x_vec':[], 'sdmap_ramp_line_y_vec':[],
                          'sdmap_expressway_line_x_vec':[], 'sdmap_expressway_line_y_vec':[],
                          'inlinek_x_vec':[], 'inlinek_y_vec':[],
                          'outlinek_x_vec':[], 'outlinek_y_vec':[],}
  line_x = []
  line_y = []
  ramp_line_x = []
  ramp_line_y = []
  expressway_line_x = []
  expressway_line_y = []
  inlink_x = []
  inlink_y = []
  outlink_x = []
  outlink_y = []
  route_ids = set([seg.id for seg in segments])
  for segment in segments:
    for point in segment.enu_points:
      ehr_x = point.x
      ehr_y = point.y
      car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
      line_x.append(car_x)
      line_y.append(car_y)
    if (segment.usage != RoadUsage.RAMP and segment.usage != RoadUsage.ENTRANCE_EXIT):
      for point in segment.enu_points:
        ehr_x = point.x
        ehr_y = point.y
        car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
        expressway_line_x.append(car_x)
        expressway_line_y.append(car_y)
    if (segment.usage == RoadUsage.RAMP or segment.usage == RoadUsage.ENTRANCE_EXIT):
      for point in segment.enu_points:
        ehr_x = point.x
        ehr_y = point.y
        car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
        ramp_line_x.append(car_x)
        ramp_line_y.append(car_y)
    for inlink in segment.in_link:
      if inlink.id in route_ids:
        continue
      for point in inlink.enu_points:
        ehr_x = point.x
        ehr_y = point.y
        car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
        inlink_x.append(car_x)
        inlink_y.append(car_y)
    for outlink in segment.out_link:
      if outlink.id in route_ids:
        continue
      for point in outlink.enu_points:
        ehr_x = point.x
        ehr_y = point.y
        car_x, car_y= global2local(ehr_x, ehr_y, x, y, yaw)
        outlink_x.append(car_x)
        outlink_y.append(car_y)

  sdmap_road_line_info['sdmap_road_line_x_vec'] = line_x
  sdmap_road_line_info['sdmap_road_line_y_vec'] = line_y
  sdmap_road_line_info['sdmap_ramp_line_x_vec'] = ramp_line_x
  sdmap_road_line_info['sdmap_ramp_line_y_vec'] = ramp_line_y
  sdmap_road_line_info['sdmap_expressway_line_x_vec'] = expressway_line_x
  sdmap_road_line_info['sdmap_expressway_line_y_vec'] = expressway_line_y
  sdmap_road_line_info['inlinek_x_vec'] = inlink_x
  sdmap_road_line_info['inlinek_y_vec'] = inlink_y
  sdmap_road_line_info['outlinek_x_vec'] = outlink_x
  sdmap_road_line_info['outlinek_y_vec'] = outlink_y
  return sdmap_road_line_info

def ehr_load_road_boundary_lines(road_boundaries, x, y, yaw, Road_boundary_max_line_size, g_is_display_enu = False):
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
          if g_is_display_enu:
            ehr_x, ehr_y = point.x, point.y
          else:
            ehr_x, ehr_y= global2local(point.x, point.y, x, y, yaw)
          # print("x:",ehr_x)
          # print("y:",ehr_y)
          line_x.append(ehr_x)
          line_y.append(ehr_y)
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

def ehr_load_lane_boundary_lines(lane_boundaries, x, y, yaw, Lane_boundary_max_line_size, g_is_display_enu = False):
  ehr_lane_boundary_info_list = []

  for i in range(Lane_boundary_max_line_size):
    ehr_lane_boundary_info = {'ehr_lane_boundary_x_vec':[], 'ehr_lane_boundary_y_vec':[],'ehr_lane_boundary_relative_id':[], 'ehr_type':[]}
    if i < len(lane_boundaries):
      lane_boundary = lane_boundaries[i]
      line_x = []
      line_y = []
      for boundary_attribute in lane_boundary.boundary_attributes:
        for  point in boundary_attribute.points:
          if g_is_display_enu:
            ehr_x, ehr_y = point.x, point.y
          else:
            ehr_x, ehr_y = global2local(point.x, point.y, x, y, yaw)
          # print("x:",ehr_x)
          # print("y:",ehr_y)
          line_x.append(ehr_x)
          line_y.append(ehr_y)
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

# 加载车道boundary
def load_lane_boundary_lines(road_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  line_info_list = []
  reference_line_msg = road_msg.reference_line_msg
  reference_line_msg_size = road_msg.reference_line_msg_size
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  lane_line_info = {'line_x_vec':[], 'line_y_vec':[], 'type_vec':[], 'relative_id_vec':[]}
  dash_line_x, dash_line_y, dash_line_id = [], [], []
  solid_line_x, solid_line_y, solid_line_id = [], [], []
  dot_line_x, dot_line_y, dot_line_id = [], [], []
  for i in range(10):
    if i < reference_line_msg_size:
      lane = reference_line_msg[i]
      relative_id = lane.relative_id
      left_line = lane.left_lane_boundary
      left_length = 0.0
      try:
        line_x, line_y = [], []
        if g_is_display_enu:
          local_points = left_line.enu_points
          point_num = left_line.enu_points_size
          type_num = left_line.type_segments_size
          k = 0
          for j in range(point_num):
            line_x.append(local_points[j].x)
            line_y.append(local_points[j].y)
            if j < (point_num - 1):
              left_length += math.sqrt((local_points[j + 1].x - local_points[j].x)**2 + (local_points[j + 1].y - local_points[j].y)**2)
            if left_length > left_line.type_segments[k].length or j == (point_num - 1):
              left_length = 0
              tp = left_line.type_segments[k].type
              k += 1
              k = max(min(k, type_num - 1), 0)
              if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                dash_line_x.append(line_x)
                dash_line_y.append(line_y)
                dash_line_id.append(relative_id)
              elif tp == 9:
                dot_line_x.append(line_x)
                dot_line_y.append(line_y)
                dot_line_id.append(relative_id)
              else:
                solid_line_x.append(line_x)
                solid_line_y.append(line_y)
                solid_line_id.append(relative_id)
              line_x, line_y = [], []
              line_x.append(local_points[j].x)
              line_y.append(local_points[j].y)
        else :
          if is_enu_to_car:
            coord_tf = coord_transformer()
            if loc_msg != None: # 长时轨迹
              cur_pos_xn = loc_msg.position.position_boot.x
              cur_pos_yn = loc_msg.position.position_boot.y
              cur_yaw = loc_msg.orientation.euler_boot.yaw
              coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
              local_points = left_line.enu_points
              point_num = left_line.enu_points_size
              type_num = left_line.type_segments_size
              k = 0
              for j in range(point_num):
                line_x.append(local_points[j].x)
                line_y.append(local_points[j].y)
                if j < (point_num - 1):
                  left_length += math.sqrt((local_points[j + 1].x - local_points[j].x)**2 + (local_points[j + 1].y - local_points[j].y)**2)
                if left_length > left_line.type_segments[k].length or j == (point_num - 1):
                  left_length = 0
                  tp = left_line.type_segments[k].type
                  k += 1
                  k = max(min(k, type_num - 1), 0)
                  line_x, line_y = coord_tf.global_to_local(line_x, line_y)
                  if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                    dash_line_x.append(line_x)
                    dash_line_y.append(line_y)
                    dash_line_id.append(relative_id)
                  elif tp == 9:
                    dot_line_x.append(line_x)
                    dot_line_y.append(line_y)
                    dot_line_id.append(relative_id)
                  else:
                    solid_line_x.append(line_x)
                    solid_line_y.append(line_y)
                    solid_line_id.append(relative_id)
                  line_x, line_y = [], []
                  line_x.append(local_points[j].x)
                  line_y.append(local_points[j].y)
            else:
              dash_line_x.append(default_line_x)
              dash_line_y.append(default_line_y)
              dash_line_id.append(relative_id)
          else:
            car_points = left_line.car_points
            point_num = left_line.car_points_size
            type_num = left_line.type_segments_size
            k = 0
            for j in range(point_num):
              line_x.append(car_points[j].x)
              line_y.append(car_points[j].y)
              if j < (point_num - 1):
                left_length += math.sqrt((car_points[j + 1].x - car_points[j].x)**2 + (car_points[j + 1].y - car_points[j].y)**2)
              if left_length > left_line.type_segments[k].length or j == (point_num - 1):
                left_length = 0
                tp = left_line.type_segments[k].type
                k += 1
                k = max(min(k, type_num - 1), 0)
                if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                  dash_line_x.append(line_x)
                  dash_line_y.append(line_y)
                  dash_line_id.append(relative_id)
                elif tp == 9:
                  dot_line_x.append(line_x)
                  dot_line_y.append(line_y)
                  dot_line_id.append(relative_id)
                else:
                  solid_line_x.append(line_x)
                  solid_line_y.append(line_y)
                  solid_line_id.append(relative_id)
                line_x, line_y = [], []
                line_x.append(car_points[j].x)
                line_y.append(car_points[j].y)
      except:
        dash_line_x.append(default_line_x)
        dash_line_y.append(default_line_y)
        dash_line_id.append(relative_id)

      right_line = lane.right_lane_boundary
      right_length = 0.0
      try:
        line_x, line_y = [], []
        if g_is_display_enu:
          local_points = right_line.enu_points
          point_num = right_line.enu_points_size
          type_num = right_line.type_segments_size
          k = 0
          for j in range(point_num):
            line_x.append(local_points[j].x)
            line_y.append(local_points[j].y)
            if j < (point_num - 1):
              right_length += math.sqrt((local_points[j + 1].x - local_points[j].x)**2 + (local_points[j + 1].y - local_points[j].y)**2)
            if right_length > right_line.type_segments[k].length or j == (point_num - 1):
              right_length = 0
              tp = right_line.type_segments[k].type
              k += 1
              k = max(min(k, type_num - 1), 0)
              if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                dash_line_x.append(line_x)
                dash_line_y.append(line_y)
                dash_line_id.append(relative_id)
              elif tp == 9:
                dot_line_x.append(line_x)
                dot_line_y.append(line_y)
                dot_line_id.append(relative_id)
              else:
                solid_line_x.append(line_x)
                solid_line_y.append(line_y)
                solid_line_id.append(relative_id)
              line_x, line_y = [], []
              line_x.append(local_points[j].x)
              line_y.append(local_points[j].y)
        else :
          if is_enu_to_car:
            coord_tf = coord_transformer()
            if loc_msg != None: # 长时轨迹
              cur_pos_xn = loc_msg.position.position_boot.x
              cur_pos_yn = loc_msg.position.position_boot.y
              cur_yaw = loc_msg.orientation.euler_boot.yaw
              coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
              local_points = right_line.enu_points
              point_num = right_line.enu_points_size
              type_num = right_line.type_segments_size
              k = 0
              for j in range(point_num):
                line_x.append(local_points[j].x)
                line_y.append(local_points[j].y)
                if j < (point_num - 1):
                  right_length += math.sqrt((local_points[j + 1].x - local_points[j].x)**2 + (local_points[j + 1].y - local_points[j].y)**2)
                if right_length > right_line.type_segments[k].length or j == (point_num - 1):
                  right_length = 0
                  tp = right_line.type_segments[k].type
                  k += 1
                  k = max(min(k, type_num - 1), 0)
                  line_x, line_y = coord_tf.global_to_local(line_x, line_y)
                  if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                    dash_line_x.append(line_x)
                    dash_line_y.append(line_y)
                    dash_line_id.append(relative_id)
                  elif tp == 9:
                    dot_line_x.append(line_x)
                    dot_line_y.append(line_y)
                    dot_line_id.append(relative_id)
                  else:
                    solid_line_x.append(line_x)
                    solid_line_y.append(line_y)
                    solid_line_id.append(relative_id)
                  line_x, line_y = [], []
                  line_x.append(local_points[j].x)
                  line_y.append(local_points[j].y)
            else:
              dash_line_x.append(default_line_x)
              dash_line_y.append(default_line_y)
              dash_line_id.append(relative_id)
          else:
            car_points = right_line.car_points
            point_num = right_line.car_points_size
            type_num = right_line.type_segments_size
            k = 0
            for j in range(point_num):
              line_x.append(car_points[j].x)
              line_y.append(car_points[j].y)
              if j < (point_num - 1):
                right_length += math.sqrt((car_points[j + 1].x - car_points[j].x)**2 + (car_points[j + 1].y - car_points[j].y)**2)
              if right_length > right_line.type_segments[k].length or j == (point_num - 1):
                right_length = 0
                tp = right_line.type_segments[k].type
                k += 1
                k = max(min(k, type_num - 1), 0)
                if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                  dash_line_x.append(line_x)
                  dash_line_y.append(line_y)
                  dash_line_id.append(relative_id)
                elif tp == 9:
                  dot_line_x.append(line_x)
                  dot_line_y.append(line_y)
                  dot_line_id.append(relative_id)
                else:
                  solid_line_x.append(line_x)
                  solid_line_y.append(line_y)
                  solid_line_id.append(relative_id)
                line_x, line_y = [], []
                line_x.append(car_points[j].x)
                line_y.append(car_points[j].y)
      except:
        dash_line_x.append(default_line_x)
        dash_line_y.append(default_line_y)
        dash_line_id.append(relative_id)
    else:
      dash_line_x.append(default_line_x)
      dash_line_y.append(default_line_y)
      dash_line_id.append(relative_id)
  line_x_vec, line_y_vec, type_vec, relative_id_vec = [], [], [], []
  line_x_vec.extend([dash_line_x, solid_line_x, dot_line_x])
  line_y_vec.extend([dash_line_y, solid_line_y, dot_line_y])
  type_vec.extend(['dashed_line', 'solid_line', 'virtual_line'])
  relative_id_vec.extend([dash_line_id, solid_line_id, dot_line_id])
  lane_line_info['line_x_vec'] = line_x_vec
  lane_line_info['line_y_vec'] = line_y_vec
  lane_line_info['type_vec'] = type_vec
  lane_line_info['relative_id_vec'] = relative_id_vec
  return lane_line_info

# 加载车道boundary
def load_lane_lines(road_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  line_info_list = []
  reference_line_msg = road_msg.reference_line_msg
  reference_line_msg_size = road_msg.reference_line_msg_size
  #print("reference_line_msg_size ", reference_line_msg_size)
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(10):
    if i< reference_line_msg_size:
      lane = reference_line_msg[i]
      lane_info_l = {'line_x_vec':[], 'line_y_vec':[], 'type_vec':[]}
      left_line = lane.left_lane_boundary
      # left_line_coef = left_line.poly_coefficient
      try:
        line_x, line_y = [], []
        if g_is_display_enu:
          # print(left_line.enu_points)
          local_points = left_line.enu_points
          point_num = left_line.enu_points_size
          line_x = [local_points[j].x for j in range(point_num)]
          line_y = [local_points[j].y for j in range(point_num)]
        else :
          if is_enu_to_car:
            coord_tf = coord_transformer()
            if loc_msg != None: # 长时轨迹
              cur_pos_xn = loc_msg.position.position_boot.x
              cur_pos_yn = loc_msg.position.position_boot.y
              cur_yaw = loc_msg.orientation.euler_boot.yaw
              coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
              local_points = left_line.enu_points
              point_num = left_line.enu_points_size
              line_x = [local_points[j].x for j in range(point_num)]
              line_y = [local_points[j].y for j in range(point_num)]
              line_x, line_y = coord_tf.global_to_local(line_x, line_y)
            else:
              line_x = default_line_x
              line_y = default_line_y
            #pt_vec = local_points[:point_num]
            #line_x, line_y = coord_tf.global_to_local(pt_vec)
          else:
            car_points = left_line.car_points
            point_num = left_line.car_points_size
            line_x = [car_points[j].x for j in range(point_num)]
            line_y = [car_points[j].y for j in range(point_num)]
        # line_x, line_y = gen_line(left_line_coef[0], left_line_coef[1], left_line_coef[2], left_line_coef[3], \
        #   left_line.begin, left_line.end)
        lane_info_l['line_y_vec'] = line_y
        lane_info_l['line_x_vec'] = line_x

        left_line_type_vec = []
        if (len(left_line.type_segments) == 0):
          left_line_type_vec.append(['dashed'])
        for j in range(len(left_line.type_segments)):
          tp = left_line.type_segments[j].type
          if tp == 0 or tp == 1 or tp == 3 or tp == 4:
            left_line_type_vec.append(['dashed'])
          elif tp == 9:
            left_line_type_vec.append(['dashdot'])
          else:
            left_line_type_vec.append(['solid'])
        lane_info_l['type_vec'] = left_line_type_vec
      except:
        lane_info_l['line_x_vec'] = default_line_x
        lane_info_l['line_y_vec'] = default_line_y
        lane_info_l['type_vec'] = ['dashed']

      line_info_list.append(lane_info_l)

      lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type_vec':[]}
      right_line = lane.right_lane_boundary
      # right_line_coef = right_line.poly_coefficient
      try:
        line_x, line_y = [], []
        if g_is_display_enu:
          # print(right_line.enu_points)
          local_points = right_line.enu_points
          point_num = right_line.enu_points_size
          line_x = [local_points[j].x for j in range(point_num)]
          line_y = [local_points[j].y for j in range(point_num)]
        else :
          if is_enu_to_car:
            coord_tf = coord_transformer()
            if loc_msg != None: # 长时轨迹
              cur_pos_xn = loc_msg.position.position_boot.x
              cur_pos_yn = loc_msg.position.position_boot.y
              cur_yaw = loc_msg.orientation.euler_boot.yaw
              coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
              local_points = right_line.enu_points
              point_num = right_line.enu_points_size
              line_x = [local_points[j].x for j in range(point_num)]
              line_y = [local_points[j].y for j in range(point_num)]
              line_x, line_y = coord_tf.global_to_local(line_x, line_y)
            #pt_vec = local_points[:point_num]
            #line_x, line_y = coord_tf.global_to_local(pt_vec)
          else:
            car_points = right_line.car_points
            point_num = right_line.car_points_size
            line_x = [car_points[j].x for j in range(point_num)]
            line_y = [car_points[j].y for j in range(point_num)]
        # line_x, line_y = gen_line(right_line_coef[0], right_line_coef[1], right_line_coef[2], right_line_coef[3], \
        #   right_line.begin, right_line.end)
        lane_info_r['line_x_vec'] = line_x
        lane_info_r['line_y_vec'] = line_y

        right_line_type_vec = []
        if (len(right_line.type_segments) == 0):
          right_line_type_vec.append(['dashed'])
        for j in range(len(right_line.type_segments)):
          tp = right_line.type_segments[j].type
          if tp == 0 or tp == 1 or tp == 3 or tp == 4:
            right_line_type_vec.append(['dashed'])
          elif tp == 9:
            right_line_type_vec.append(['dashdot'])
          else:
            right_line_type_vec.append(['solid'])
        lane_info_r['type_vec'] = right_line_type_vec
      except:
        lane_info_r['line_x_vec'] = default_line_x
        lane_info_r['line_y_vec'] = default_line_y
        lane_info_r['type_vec'] = ['dashed']

      line_info_list.append(lane_info_r)
    else:
      lane_info_l = {'line_x_vec':[], 'line_y_vec':[], 'type_vec':[]}
      lane_info_l['line_x_vec'] = default_line_x
      lane_info_l['line_y_vec'] = default_line_y
      lane_info_l['type_vec'] = ['dashed']
      lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type_vec':[]}
      lane_info_r['line_x_vec'] = default_line_x
      lane_info_r['line_y_vec'] = default_line_y
      lane_info_r['type_vec'] = ['dashed']
      line_info_list.append(lane_info_l)
      line_info_list.append(lane_info_r)
  return line_info_list

#加载topo车道线
def load_lane_topo_lines(lane_topo_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  line_topo_info_list = []
  line_topo_msg = lane_topo_msg.lane_line
  line_topo_msg_size = lane_topo_msg.lane_line_size

  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(20):
    lane_info = {'line_x_topo':[], 'line_y_topo':[],'type':[]}
    if i< line_topo_msg_size:
      lane_line = line_topo_msg[i]
      lane_line_topo_refline_points = lane_line.lane_points_attr_set
      lane_line_topo_refline_points_size = lane_line.lane_points_attr_set_size
      line_x = []
      line_y = []
      line_x = [lane_line_topo_refline_points[j].lane_point_coordinate.x for j in range(lane_line_topo_refline_points_size)]
      line_y = [lane_line_topo_refline_points[j].lane_point_coordinate.y for j in range(lane_line_topo_refline_points_size)]

      lane_info['line_x_topo'] = line_x
      lane_info['line_y_topo'] = line_y

      line_topo_info_list.append(lane_info)

      tp = lane_line.type
      if tp == 0 or tp == 1 or tp == 3 or tp == 4:
        lane_info['type'] = ['dashed']
      else:
        lane_info['type'] = ['solid']
    else:
      lane_info['line_x_topo'] = default_line_x
      lane_info['line_y_topo'] = default_line_y
      lane_info['type'] = ['dashed']
      line_topo_info_list.append(lane_info)

  return line_topo_info_list

# 加载中心线
def load_lane_center_lines(road_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  line_info_list = []
  reference_line_msg = road_msg.reference_line_msg
  reference_line_msg_size = road_msg.reference_line_msg_size
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(10):
    lane_info = {'line_x_vec':[], 'line_y_vec':[], 'relative_id':[],'type':[], 'line_s_vec':[], 'curvature_vec':[], 'd_poly_curvature_vec':[], 'lane_mark_vec':[],
                 'lane_mark_point_x':[], 'lane_mark_point_y':[], 'lane_mark_loc_x':[], 'lane_mark_loc_y':[]}
    if i< reference_line_msg_size:
      lane = reference_line_msg[i]
      virtual_lane_refline_points = lane.lane_reference_line.virtual_lane_refline_points
      virtual_lane_refline_points_size = lane.lane_reference_line.virtual_lane_refline_points_size
      d_poly = lane.lane_reference_line.poly_coefficient_car
      virtual_lane_marks_size = lane.lane_marks_size
      lane_mark_s_vec = [lane.lane_marks[j].end for j in range(virtual_lane_marks_size)]
      lane_mark_s_begin_vec = [lane.lane_marks[j].begin for j in range(virtual_lane_marks_size)]
      lane_info['lane_mark_vec'] = [lane.lane_marks[j].lane_mark for j in range(virtual_lane_marks_size)]

      lane_mark_point_x = []
      lane_mark_point_y = []
      lane_mark_loc_x = []
      lane_mark_loc_y = []
      line_x = []
      line_y = []
      line_curvature = []
      d_poly_line_curvature = []
      line_s = []
      if g_is_display_enu:
        line_x = [virtual_lane_refline_points[j].local_point.x for j in range(virtual_lane_refline_points_size)]
        line_y = [virtual_lane_refline_points[j].local_point.y for j in range(virtual_lane_refline_points_size)]
      else:
        if is_enu_to_car:
          coord_tf = coord_transformer()
          if loc_msg != None: # 长时轨迹
            cur_pos_xn = loc_msg.position.position_boot.x
            cur_pos_yn = loc_msg.position.position_boot.y
            cur_yaw = loc_msg.orientation.euler_boot.yaw
            coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
            line_x = [virtual_lane_refline_points[j].local_point.x for j in range(virtual_lane_refline_points_size)]
            line_y = [virtual_lane_refline_points[j].local_point.y for j in range(virtual_lane_refline_points_size)]
            line_x, line_y = coord_tf.global_to_local(line_x, line_y)
            """ for virtual_lane_refline_point in virtual_lane_refline_points:
              car_point_x, car_point_y = coord_tf.global_to_local([virtual_lane_refline_point.enu_point.x], [virtual_lane_refline_point.enu_point.y])
              line_x.append(car_point_x[0])
              line_y.append(car_point_y[0]) """
          else:
            line_x = default_line_x
            line_y = default_line_y
        else:
          line_x = [virtual_lane_refline_points[j].car_point.x for j in range(virtual_lane_refline_points_size)]
          line_y = [virtual_lane_refline_points[j].car_point.y for j in range(virtual_lane_refline_points_size)]

      line_s = [virtual_lane_refline_points[j].s for j in range(virtual_lane_refline_points_size)]
      line_curvature = [max(min(1.0 / (virtual_lane_refline_points[j].curvature + 1e-6), 10000.0), -10000.0) for j in range(virtual_lane_refline_points_size)]
      try:
        d_poly_line_curvature = [max(min(1.0 / ((math.fabs(2 * d_poly[3] * (virtual_lane_refline_points[j].s - 50.0) + d_poly[2]) / math.pow(math.pow(2 * d_poly[3] * (virtual_lane_refline_points[j].s - 50.0) + d_poly[2], 2) + 1,1.5)) + 1e-6), 10000.0), -10000.0) for j in range(virtual_lane_refline_points_size)]
      except:
        print("d_poly error!")

      lane_info['line_x_vec'] = line_x
      lane_info['line_y_vec'] = line_y
      lane_info['relative_id'] = lane.relative_id
      lane_info['type'] = 0
      lane_info['line_s_vec'] = line_s
      lane_info['curvature_vec'] = line_curvature
      lane_info['d_poly_curvature_vec'] = d_poly_line_curvature

      try:
        for i in range(10):
          if i < virtual_lane_marks_size:
            is_no_lm_point = True
            for j in range(virtual_lane_refline_points_size):
              if i == virtual_lane_marks_size - 1:
                lane_mark_point_x.append(line_x[-1])
                lane_mark_point_y.append(line_y[-1])
                break
              if line_s[j] >= lane_mark_s_vec[i]:
                lane_mark_point_x.append(line_x[j])
                lane_mark_point_y.append(line_y[j])
                is_no_lm_point = False
                break
            if is_no_lm_point and (virtual_lane_refline_points_size > 0):
              lane_mark_point_x.append(line_x[virtual_lane_refline_points_size - 1])
              lane_mark_point_y.append(line_y[virtual_lane_refline_points_size - 1])
          else:
            break
      except:
        pass

      try:
        for i in range(10):
          if i < virtual_lane_marks_size:
            is_no_lm_point = True
            for j in range(virtual_lane_refline_points_size):
              if line_s[j] >= (lane_mark_s_vec[i] + lane_mark_s_begin_vec[i]) / 2:
                lane_mark_loc_x.append(line_x[j])
                lane_mark_loc_y.append(line_y[j])
                is_no_lm_point = False
                break
            if is_no_lm_point and (virtual_lane_refline_points_size > 0):
              lane_mark_loc_x.append(line_x[virtual_lane_refline_points_size - 1])
              lane_mark_loc_y.append(line_y[virtual_lane_refline_points_size - 1])
          else:
            break
      except:
        pass

    else:
      lane_mark_point_x = []
      lane_mark_point_y = []
      lane_mark_loc_x = []
      lane_mark_loc_y = []
      lane_info['line_x_vec'] = default_line_x
      lane_info['line_y_vec'] = default_line_y
      lane_info['relative_id'] = 1000
      lane_info['type'] = 0
      lane_info['line_s_vec'] = 0
      lane_info['curvature_vec'] = 0
      lane_info['d_poly_curvature_vec'] = 0
      lane_info['lane_mark_vec'] = []

    lane_info['lane_mark_point_x'] = lane_mark_point_x
    lane_info['lane_mark_point_y'] = lane_mark_point_y
    lane_info['lane_mark_loc_x'] = lane_mark_loc_x
    lane_info['lane_mark_loc_y'] = lane_mark_loc_y
    line_info_list.append(lane_info)

  return line_info_list

# 加载topo中心线
def load_lane_topo_center_lines(lane_topo_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  center_line_topo_info_list = []
  lane_topo_msg_msg = lane_topo_msg.lanes
  lane_topo_size = lane_topo_msg.lane_line_size
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(10):
    lane_info = {'center_line_x_topo':[], 'center_line_y_topo':[]}
    if i< lane_topo_size:
      lane = lane_topo_msg_msg[i]
      center_lane_topo_refline_points = lane.central_line.lane_points_attr_set
      center_lane_topo_refline_points_size = lane.central_line.lane_points_attr_set_size
      line_x = []
      line_y = []
      line_x = [center_lane_topo_refline_points[j].lane_point_coordinate.x for j in range(center_lane_topo_refline_points_size)]
      line_y = [center_lane_topo_refline_points[j].lane_point_coordinate.y for j in range(center_lane_topo_refline_points_size)]

      lane_info['center_line_x_topo'] = line_x
      lane_info['center_line_y_topo'] = line_y

      center_line_topo_info_list.append(lane_info)
    else:
      lane_info['center_line_x_topo'] = default_line_x
      lane_info['center_line_y_topo'] = default_line_y
      center_line_topo_info_list.append(lane_info)

  return center_line_topo_info_list

def load_rdg_lane_lines(road_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  line_info_list = []
  lane_line = road_msg.lane_line
  lane_line_size = road_msg.lane_line_size
  #print("reference_line_msg_size ", reference_line_msg_size)
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)

  for i in range(20):
    if i< lane_line_size:
      lane = lane_line[i]
      lane_info = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}

      lane_points_attr_set = lane.lane_points_attr_set
      point_num = lane.lane_points_attr_set_size
      if g_is_display_enu:
        if loc_msg != None: # 长时轨迹
          coord_tf = coord_transformer()
          cur_pos_xn = loc_msg.position.position_boot.x
          cur_pos_yn = loc_msg.position.position_boot.y
          cur_yaw = loc_msg.orientation.euler_boot.yaw
          coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        line_x = [lane_points_attr_set[j].lane_point_coordinate.x for j in range(point_num)]
        line_y = [lane_points_attr_set[j].lane_point_coordinate.y for j in range(point_num)]
        line_x, line_y = coord_tf.local_to_global(line_x, line_y)
      else:
        line_x = [lane_points_attr_set[j].lane_point_coordinate.x for j in range(point_num)]
        line_y = [lane_points_attr_set[j].lane_point_coordinate.y for j in range(point_num)]

      lane_info['line_x_vec'] = line_x
      lane_info['line_y_vec'] = line_y
      if( lane.marking_segments_size != 0 ):
        tp = lane.marking_segments[0].marking
        if tp == 0 or tp == 1 or tp == 3 or tp == 4:
          lane_info['type'] = ['dashed']
        else:
          lane_info['type'] = ['solid']
      else:
        lane_info['type'] = ['solid']

      if (lane.type == 2):
        print('lane_info curb')
        lane_info['type'] = ['curb']
      line_info_list.append(lane_info)
    else:
      lane_info = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
      lane_info['line_x_vec'] = default_line_x
      lane_info['line_y_vec'] = default_line_y
      lane_info['type'] = ['dashed']
      line_info_list.append(lane_info)
  return line_info_list

def load_rdg_parking_lane_line(rdg_parking_lane_line_msg, loc_msg):
  ground_mark_info = {
    'ground_mark_x_rel': [],
    'ground_mark_y_rel': [],
    'ground_mark_x': [],
    'ground_mark_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'ground_mark_label':[]
  }
  LaneDrivableDirection = ['未知', '直行', '直行加右转', '右转', '直行加左转', '左转', '调头加左转', '调头加右转',
                           '直行加左转加右转', '直行加左调头', '直行加右调头', '直行加左转加左掉头', '左调头', '右调头',
                           '禁止调头', '左转加右转', '左侧汇流', '右侧汇流', '人行横道']
  try:
    lane_ground_markings_size = rdg_parking_lane_line_msg.lane_ground_markings_size
    for j in range(lane_ground_markings_size):
      lane_ground_marking = rdg_parking_lane_line_msg.lane_ground_markings[j]
      turn_type = lane_ground_marking.turn_type
      ground_marking_points_set = lane_ground_marking.ground_marking_points_set
      pos_x_rel = 0.0
      pos_y_rel = 0.0
      ground_mark_x_rel, ground_mark_y_rel = [], []
      for index, points in enumerate(ground_marking_points_set):
        ground_mark_x_rel.append(points.x)
        ground_mark_y_rel.append(points.y)
        pos_x_rel += points.x
        pos_y_rel += points.y

      pos_x_rel /= 4.0
      pos_y_rel /= 4.0

      pos_x, pos_y= [], []
      ground_mark_x, ground_mark_y = [], []
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        ground_mark_x, ground_mark_y = coord_tf.local_to_global(ground_mark_x_rel, ground_mark_y_rel)
        pos_x, pos_y = coord_tf.local_to_global([pos_x_rel], [pos_y_rel])

      ground_mark_info['ground_mark_x_rel'].append(ground_mark_x_rel)
      ground_mark_info['ground_mark_y_rel'].append(ground_mark_y_rel)
      ground_mark_info['pos_x_rel'].append(pos_x_rel)
      ground_mark_info['pos_y_rel'].append(pos_y_rel)
      ground_mark_info['ground_mark_x'].append(ground_mark_x)
      ground_mark_info['ground_mark_y'].append(ground_mark_y)
      ground_mark_info['pos_x'].append([pos_x])
      ground_mark_info['pos_y'].append([pos_y])
      ground_mark_info['ground_mark_label'].append([LaneDrivableDirection[turn_type]])
  except:
    print("rdg_parking_lane_line_msg error")
  return ground_mark_info

def load_stop_lines(rdg_lane_lines_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  stop_line_info_list = []
  stop_line_msg = rdg_lane_lines_msg.stop_line
  stop_line_msg_size = rdg_lane_lines_msg.stop_line_size
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(5):
    lane_info = {'stop_line_x':[], 'stop_line_y':[]}
    if i< stop_line_msg_size:
      stop_line = stop_line_msg[i]
      stop_line_points = stop_line.lane_points_attr_set
      stop_line_points_size = stop_line.lane_points_attr_set_size
      line_x = []
      line_y = []
      line_x = [stop_line_points[j].lane_point_coordinate.x for j in range(stop_line_points_size)]
      line_y = [stop_line_points[j].lane_point_coordinate.y for j in range(stop_line_points_size)]

      lane_info['stop_line_x'] = line_x
      lane_info['stop_line_y'] = line_y

      stop_line_info_list.append(lane_info)

    else:
      lane_info['stop_line_x'] = default_line_x
      lane_info['stop_line_y'] = default_line_y
      stop_line_info_list.append(lane_info)

  return stop_line_info_list

def load_zebra_crossing_lines(rdg_lane_lines_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  zebra_crossing_lines_info_list = []
  zebra_crossing_lines_msg = rdg_lane_lines_msg.lane_ground_markings
  zebra_crossing_lines_size = rdg_lane_lines_msg.lane_ground_markings_size
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(20):
    zebra_crossing_info = {'zebra_crossing_line_x':[], 'zebra_crossing_line_y':[]}
    if i< zebra_crossing_lines_size and zebra_crossing_lines_msg[i].turn_type == 18 :
      zebra_crossing_line = zebra_crossing_lines_msg[i]
      zebra_crossing_line_points = zebra_crossing_line.ground_marking_points_set
      zebra_crossing_line_points_size = zebra_crossing_line.ground_marking_points_set_size
      line_x = []
      line_y = []
      line_x = [zebra_crossing_line_points[j].x for j in range(zebra_crossing_line_points_size)]
      line_y = [zebra_crossing_line_points[j].y for j in range(zebra_crossing_line_points_size)]

      zebra_crossing_info['zebra_crossing_line_x'] = line_x
      zebra_crossing_info['zebra_crossing_line_y'] = line_y

      zebra_crossing_lines_info_list.append(zebra_crossing_info)
    else:
      zebra_crossing_info['zebra_crossing_line_x'] = default_line_x
      zebra_crossing_info['zebra_crossing_line_y'] = default_line_y

      zebra_crossing_lines_info_list.append(zebra_crossing_info)

  return zebra_crossing_lines_info_list



def load_intersection_generated_refline(plan_gen_refline, is_enu_to_car = False, loc_msg = [], g_is_display_enu = False):
  virtual_lane_refline_points = plan_gen_refline.virtual_lane_refline_points
  line_x = []
  line_y = []
  coord_tf = coord_transformer()
  cur_pos_xn = loc_msg.position.position_boot.x
  cur_pos_yn = loc_msg.position.position_boot.y
  cur_yaw = loc_msg.orientation.euler_boot.yaw
  coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
  for virtual_lane_refline_point in virtual_lane_refline_points:
    car_point_x, car_point_y = coord_tf.global_to_local([virtual_lane_refline_point.local_point.x], [virtual_lane_refline_point.local_point.y])
    line_x.append(car_point_x[0])
    line_y.append(car_point_y[0])
  return line_x, line_y

def load_obstacle_params(fus_msg, is_enu_to_car = False, loc_msg = None, environment_model_info = None):
  obs_info_all = dict()
  fusion_object_size = fus_msg.fusion_object_size
  obstacle_list = fus_msg.fusion_object
  local_point_valid = fus_msg.local_point_valid  # 障碍物中用到的绝对坐标是否有效
  # print("obj local_point_valid: ", local_point_valid)
  num = 0
  for i in range(fusion_object_size):
    obs_id = obstacle_list[i].additional_info.track_id
    source = obstacle_list[i].additional_info.fusion_source
    type = obstacle_list[i].common_info.type
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
        'polygon_x_rel': [],
        'polygon_y_rel': [],
        'polygon_x': [],
        'polygon_y': [],
        'obstacles_id': [],
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
    polygon_x = []
    polygon_y = []
    try:
      frenet_vs, frenet_vl = 255, 255
      lat_decision = "None"
      is_static = ""
      obs_polygon = []
      for obstacle in environment_model_info.obstacle:
        if obstacle.id == obs_id:
          obs_polygon = obstacle.polygon_points
          frenet_vs, frenet_vl = obstacle.vs_lon_relative, obstacle.vs_lat_relative
          if (0 == obstacle.lat_decision):
            lat_decision = "LEFT"
          elif (1 == obstacle.lat_decision):
            lat_decision = "RIGHT"
          elif (2 == obstacle.lat_decision):
            lat_decision = "IGNORE"
          if obstacle.is_static:
            is_static = "Static"
          break
      for point in obs_polygon:
        polygon_x.append(point.x)
        polygon_y.append(point.y)
    except:
      pass
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
    # obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
    # obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
    # obs_info_all[source]['pos_x_rel'].append(long_pos_rel)
    # obs_info_all[source]['pos_y_rel'].append(lat_pos_rel)

    polygon_x_rel, polygon_y_rel = [], []
    if is_enu_to_car:
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        obstacles_x_rel, obstacles_y_rel = coord_tf.global_to_local(obs_x, obs_y)
        pos_x_rel, pos_y_rel = coord_tf.global_to_local([long_pos], [lat_pos])
        polygon_x_rel, polygon_y_rel = coord_tf.global_to_local(polygon_x, polygon_y)
      else:
        obstacles_x_rel, obstacles_y_rel = obs_x_rel, obs_y_rel
        pos_x_rel, pos_y_rel = long_pos_rel, lat_pos_rel
      obs_info_all[source]['obstacles_x_rel'].append(obstacles_x_rel)
      obs_info_all[source]['obstacles_y_rel'].append(obstacles_y_rel)
      obs_info_all[source]['polygon_x_rel'].append(polygon_x_rel)
      obs_info_all[source]['polygon_y_rel'].append(polygon_y_rel)
      obs_info_all[source]['pos_x_rel'].append(pos_x_rel)
      obs_info_all[source]['pos_y_rel'].append(pos_y_rel)
    else:
      obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
      obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
      obs_info_all[source]['polygon_x_rel'].append(polygon_x_rel)
      obs_info_all[source]['polygon_y_rel'].append(polygon_y_rel)
      obs_info_all[source]['pos_x_rel'].append(long_pos_rel)
      obs_info_all[source]['pos_y_rel'].append(lat_pos_rel)

    obs_info_all[source]['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
    obs_info_all[source]['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
    obs_info_all[source]['obstacles_tid'].append(obs_id)
#             fusion_obs_info['is_cipv'].append(obstacle_list[i].target_selection_type)
    if frenet_vs == 255 and  frenet_vl == 255:
      obs_info_all[source]['obs_label'].append('v(' + str(obs_id) + ')=' \
          + str(round(obstacle_list[i].common_info.relative_velocity.x, 2))+','+ str(round(obstacle_list[i].common_info.relative_velocity.y, 4))+','+ str(type)+'\n'\
          + str(round(obstacle_list[i].common_info.velocity.x, 2))+','+ str(round(obstacle_list[i].common_info.velocity.y, 4))+'\n'\
          + lat_decision + '\n' + is_static)
    else:
      obs_info_all[source]['obs_label'].append('vs(' + str(obs_id) + ')=' \
          + str(round(frenet_vs, 2))+', '+ str(round(frenet_vl, 2))+', '+str(type)+'\n' +'rel_v: '\
          + str(round(obstacle_list[i].common_info.relative_velocity.x, 2))+', '+ str(round(obstacle_list[i].common_info.relative_velocity.y, 2))+'\n'\
          +'abs_v: '+ str(round(obstacle_list[i].common_info.velocity.x, 2))+', '+ str(round(obstacle_list[i].common_info.velocity.y, 2))+'\n'\
          + lat_decision + '\n' + is_static)
    obs_info_all[source]['obstacles_x'].append(obs_x)
    # for ind in range(len(obs_y)):
    obs_info_all[source]['obstacles_y'].append(obs_y)
    obs_info_all[source]['polygon_x'].append(polygon_x)
    obs_info_all[source]['polygon_y'].append(polygon_y)
    obs_info_all[source]['obstacles_id'].append(obs_id)
    obs_info_all[source]['pos_x'].append(long_pos)
    obs_info_all[source]['pos_y'].append(lat_pos)
    # print(long_pos_rel, lat_pos_rel, long_pos, lat_pos)

  obs_info = {
    'obstacles_x_rel': [],
    'obstacles_y_rel': [],
    'obstacles_x': [],
    'obstacles_y': [],
    'polygon_x_rel': [],
    'polygon_y_rel': [],
    'polygon_x': [],
    'polygon_y': [],
    'obstacles_id': [],
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
  if (1 in obs_info_all.keys()) == False:
    obs_info_all[1] = obs_info
  if (4 in obs_info_all.keys()) == False:
    obs_info_all[4] = obs_info

  return obs_info_all


def load_occupancy_obstacle(fus_occ_obj_msg, loc_msg = None, environment_model_info = None):
  obs_info_all = dict()
  fusion_object_size = fus_occ_obj_msg.fusion_object_size
  obstacle_list = fus_occ_obj_msg.fusion_object
  local_point_valid = fus_occ_obj_msg.local_point_valid  # 障碍物中用到的绝对坐标是否有效
  # print("occ obj local_point_valid: ", local_point_valid)
  num = 0
  plan_obs_id = 7000000
  for i in range(fusion_object_size):
    plan_obs_id += 1
    common_info = obstacle_list[i].common_occupancy_info
    additional_info = obstacle_list[i].additional_occupancy_info
    obs_id = additional_info.track_id + 7000000
    source = additional_info.fusion_source
    source = 0  # hack
    type = common_info.type
    # if source & 0x01: #相机融合障碍物
    #   source = 1
    # elif (common_info.relative_center_position.x > 0 and \
    #   math.tan(25) > math.fabs(common_info.relative_center_position.y / common_info.relative_center_position.x)) or \
    #   math.fabs(common_info.relative_center_position.y) > 10:
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
        'polygon_x': [],
        'polygon_y': [],
        'polygon_x_rel': [],
        'polygon_y_rel': [],
        'obstacles_id': [],
        'obstacles_vel': [],
        'obstacles_acc': [],
        'obstacles_tid': [],
        'is_cipv': [],
        'obs_label':[]
      }
      obs_info_all[source] = obs_info
    polygon_x = []
    polygon_y = []
    try:
      frenet_vs, frenet_vl = 255, 255
      lat_decision = "None"
      is_static = ""
      obs_polygon = []
      for obstacle in environment_model_info.obstacle:
        if obstacle.id == obs_id:
        # if obstacle.id == plan_obs_id:
          obs_polygon = obstacle.polygon_points
          frenet_vs, frenet_vl = obstacle.vs_lon_relative, obstacle.vs_lat_relative
          if (0 == obstacle.lat_decision):
            lat_decision = "LEFT"
          elif (1 == obstacle.lat_decision):
            lat_decision = "RIGHT"
          elif (2 == obstacle.lat_decision):
            lat_decision = "IGNORE"
          if obstacle.is_static:
            is_static = "Static"
          break
      for point in obs_polygon:
        polygon_x.append(point.x)
        polygon_y.append(point.y)
    except:
      pass
    # pos_x_rel = common_info.relative_center_position.x
    # pos_y_rel = common_info.relative_center_position.y
    pos_x = common_info.relative_center_position.x
    pos_y = common_info.relative_center_position.y
    polygon_points_size = additional_info.polygon_points_size
    polygon_points = additional_info.polygon_points
    obs_x, obs_y = [], []
    for index, points in enumerate(polygon_points):
      obs_x.append(points.x)
      obs_y.append(points.y)

    num = num + 1

    pos_x_rel, pos_y_rel = [], []
    obs_x_rel, obs_y_rel = [], []
    polygon_x_rel, polygon_y_rel = [], []
    coord_tf = coord_transformer()
    if loc_msg != None: # 长时轨迹
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
      obs_x_rel, obs_y_rel = coord_tf.global_to_local(obs_x, obs_y)
      pos_x_rel, pos_y_rel = coord_tf.global_to_local([pos_x], [pos_y])
      polygon_x_rel, polygon_y_rel = coord_tf.global_to_local(polygon_x, polygon_y)

    obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all[source]['pos_x_rel'].append(pos_x_rel)
    obs_info_all[source]['pos_y_rel'].append(pos_y_rel)
    obs_info_all[source]['obstacles_x'].append(obs_x)
    obs_info_all[source]['obstacles_y'].append(obs_y)
    obs_info_all[source]['pos_x'].append(pos_x)
    obs_info_all[source]['pos_y'].append(pos_y)
    obs_info_all[source]['polygon_x'].append(polygon_x)
    obs_info_all[source]['polygon_y'].append(polygon_y)
    obs_info_all[source]['polygon_x_rel'].append(polygon_x_rel)
    obs_info_all[source]['polygon_y_rel'].append(polygon_y_rel)
    obs_info_all[source]['obstacles_id'].append(obs_id)
    obs_info_all[source]['obstacles_vel'].append(common_info.relative_velocity.x)
    obs_info_all[source]['obstacles_acc'].append(common_info.relative_acceleration.x)
    obs_info_all[source]['obstacles_tid'].append(additional_info.track_id)

    if frenet_vs == 255 and  frenet_vl == 255:
      obs_info_all[source]['obs_label'].append('v(' + str(obs_id) + ')=' \
          + str(type))
    else:
      obs_info_all[source]['obs_label'].append('vs(' + str(obs_id) + ')=' \
          + str(type)+'\n'\
          + lat_decision + '\n' + is_static)

  if fusion_object_size == 0:
    obs_info = {
      'obstacles_x_rel': [],
      'obstacles_y_rel': [],
      'obstacles_x': [],
      'obstacles_y': [],
      'pos_x_rel': [],
      'pos_y_rel': [],
      'pos_x': [],
      'pos_y': [],
      'polygon_x': [],
      'polygon_y': [],
      'polygon_x_rel': [],
      'polygon_y_rel': [],
      'obstacles_id': [],
      'obstacles_vel': [],
      'obstacles_acc': [],
      'obstacles_tid': [],
      'is_cipv': [],
      'obs_label':[]
    }
    obs_info_all[0] = obs_info

  return obs_info_all


def load_obstacle_me(camera_msg,is_rdg):

  obs_info_all = dict()
  obstacle_list = camera_msg.camera_perception_objects
  obs_num = camera_msg.camera_perception_objects_size
  num = 0
  for i in range(obs_num):
    source = 1
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
    long_pos_rel = 0
    if is_rdg == False:
      long_pos_rel = obstacle_list[i].common_info.relative_position.x + obstacle_list[i].common_info.shape.length / 2
    else:
      long_pos_rel = obstacle_list[i].common_info.relative_center_position.x
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


def load_rdg_general_obstacle(camera_msg, loc_msg = None):
  obs_info_all = {
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
  obstacle_list = camera_msg.camera_perception_objects
  obs_num = camera_msg.camera_perception_objects_size
  num = 0
  for i in range(obs_num):
    #remove obstacle
    type = obstacle_list[i].common_info.type
    if type > 22 and type != 27 :
      continue

    long_pos_rel = obstacle_list[i].common_info.relative_center_position.x
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
    # 绝对坐标系下的数据
    obs_x, obs_y = [], []
    long_pos, lat_pos = [], []
    if loc_msg != None:
      coord_tf = coord_transformer()
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
      obs_x, obs_y = coord_tf.local_to_global(obs_x_rel, obs_y_rel)
      long_pos, lat_pos = coord_tf.local_to_global([long_pos_rel], [lat_pos_rel])
    else:
      print("no loc_msg in load_rdg_general_obstacle")
    obs_info_all['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all['pos_x_rel'].append(long_pos_rel)
    obs_info_all['pos_y_rel'].append(lat_pos_rel)
    obs_info_all['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
    obs_info_all['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
    obs_info_all['obstacles_tid'].append(obstacle_list[i].common_info.id)#contour不太确定
    # obs_info_all['is_cipv'].append(obstacle_list[i].target_selection_type)
    obs_info_all['obs_label'].append('v(' + str(obstacle_list[i].common_info.id) + ')=' \
        + str(round(obstacle_list[i].common_info.relative_velocity.x, 2)) + ',' \
        + str(round(obstacle_list[i].common_info.relative_velocity.y, 2)) + '\n' \
        + str(round(obstacle_list[i].common_info.velocity.x, 2))+ ',' \
        + str(round(obstacle_list[i].common_info.velocity.y, 4)) + ',' \
        + str(type))
    obs_info_all['obstacles_x'].append(obs_x)
    obs_info_all['obstacles_y'].append(obs_y)
    obs_info_all['pos_x'].append(long_pos)
    obs_info_all['pos_y'].append(lat_pos)
    num = num + 1

  return obs_info_all


def load_rdg_occupancy_obstacle(rdg_occ_obj_msg, loc_msg = None):
  obs_info_all = {
    'obstacles_x_rel': [],
    'obstacles_y_rel': [],
    'obstacles_x': [],
    'obstacles_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'obs_label':[]
  }
  perception_objects_size = rdg_occ_obj_msg.camera_perception_objects_size
  perception_objects = rdg_occ_obj_msg.camera_perception_objects
  num = 0
  for i in range(perception_objects_size):
    num = num + 1
    id = perception_objects[i].id
    type = perception_objects[i].type
    # visable_seg_num = perception_objects[i].type
    # contour_points_size = perception_objects[i].contour_points_size
    contour_points = perception_objects[i].contour_points

    pos_x_rel, pos_y_rel = [], []
    obs_x_rel, obs_y_rel = [], []
    for index, points in enumerate(contour_points):
      obs_x_rel.append(points.x)
      obs_y_rel.append(points.y)
      if index > 0:
        continue
      pos_x_rel.append(points.x)
      pos_y_rel.append(points.y)

    pos_x, pos_y = [], []
    obs_x, obs_y = [], []
    coord_tf = coord_transformer()
    if loc_msg != None: # 长时轨迹
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
      obs_x, obs_y = coord_tf.local_to_global(obs_x_rel, obs_y_rel)
      pos_x, pos_y = coord_tf.local_to_global(pos_x_rel, pos_y_rel)

    obs_info_all['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all['pos_x_rel'].append(pos_x_rel)
    obs_info_all['pos_y_rel'].append(pos_y_rel)
    obs_info_all['obstacles_x'].append(obs_x)
    obs_info_all['obstacles_y'].append(obs_y)
    obs_info_all['pos_x'].append(pos_x)
    obs_info_all['pos_y'].append(pos_y)
    obs_info_all['obs_label'].append('t(' + str(id) + ')=' \
        + str(type))

  return obs_info_all


def load_obstacle_radar(obstacle_list,type,num):
  obs_info_all = dict()
  obs_num = num
  # print("obs_num:",obs_num)
  #print(obs_num)
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

def load_prediction_objects(obstacle_list, prediction_obs_id, localization_info, g_is_display_enu = False):
  prediction_dict = {0: {'x': [], 'y': [], 'obs_x': [], 'obs_y': []},
                     1: {'x': [], 'y': [], 'obs_x': [], 'obs_y': []},
                     2: {'x': [], 'y': [], 'obs_x': [], 'obs_y': []},
                     3: {'x': [], 'y': [], 'obs_x': [], 'obs_y': []},
                     4: {'x': [], 'y': [], 'obs_x': [], 'obs_y': []}}
  # obs_info = {'obstacles_x': [],
  #             'obstacles_y': [],
  #             'pos_x': [],
  #             'pos_y': [],
  #             'loc_x': [],
  #             'loc_y': [],
  #             'obstacles_vel': [],
  #             'obstacles_acc': [],
  #             'obstacles_tid': [],
  #             'is_cipv': [],
  #             'obs_label':[]
  #             }
  # localization_x = 0
  # localization_y = 0
  # if localization_info.pose.type == 1:
  #   localization_x = localization_info.pose.enu_position.x
  #   localization_y = localization_info.pose.enu_position.y
  # elif localization_info.pose.type == 2:
  #   localization_x = localization_info.pose.llh_position.x
  #   localization_y = localization_info.pose.llh_position.y
  # elif localization_info.pose.type == 3:
  #   localization_x = localization_info.pose.local_position.x
  #   localization_y = localization_info.pose.local_position.y
  # elif localization_info.pose.type == 0:
  #   localization_x = localization_info.pose.local_position.x
  #   localization_y = localization_info.pose.local_position.y
  # localization_x = localization_info.position.position_boot.x
  # localization_y = localization_info.position.position_boot.y
  # linear_velocity_from_wheel = localization_info.pose.linear_velocity_from_wheel
  # localization_theta = localization_info.orientation.euler_boot.yaw

  trajectory_info = {'x':[],'y':[], 'obs_x': [], 'obs_y': [], 'r':[]}
  p_x = []
  p_y = []
  p_obs_x = []
  p_obs_y = []
  obs_id_num = len(prediction_obs_id)
  obs_num = len(obstacle_list)
  # num = len(obstacle_list[0].trajectory.trajectory_point)
  # print("num", num)
  for i in range(obs_num):
    obs_id = obstacle_list[i].fusion_obstacle.additional_info.track_id
    if (obs_id_num > 0) and (obs_id not in prediction_obs_id):
      continue
    if (obstacle_list[i].fusion_obstacle.additional_info.fusion_source & 1) == 0:
      continue
    if len(obstacle_list[i].trajectory.trajectory_point) == 0:
      # print("No data")
      continue
    elif obstacle_list[i].fusion_obstacle.common_info.shape.width == 0 or obstacle_list[i].fusion_obstacle.common_info.shape.length == 0:
      continue
    else:
      # long_pos = obstacle_list[i].trajectory.trajectory_point[0].center_position.x
      # lat_pos = obstacle_list[i].trajectory.trajectory_point[0].center_position.y
      # theta = obstacle_list[i].fusion_obstacle.common_info.heading_angle
      half_width = obstacle_list[i].fusion_obstacle.common_info.shape.width / 2
      half_length = obstacle_list[i].fusion_obstacle.common_info.shape.length / 2
      # track_id = obstacle_list[i].fusion_obstacle.additional_info.track_id

      # print(f"long_pos = {long_pos}\tlat_pos = {lat_pos}\ttheta = {theta}\thalf_width = {half_width}\tlength = {length}\n")

      # obs_x = [long_pos+half_width*math.sin(theta), \
      #           long_pos+half_width*math.sin(theta)+length*math.cos(theta), \
      #           long_pos-half_width*math.sin(theta)+length*math.cos(theta), \
      #           long_pos-half_width*math.sin(theta), \
      #           long_pos+half_width*math.sin(theta)]

      # obs_y = [lat_pos-half_width*math.cos(theta), \
      #           lat_pos-half_width*math.cos(theta)+ length*math.sin(theta), \
      #           lat_pos+half_width*math.cos(theta)+ length*math.sin(theta), \
      #           lat_pos+half_width*math.cos(theta),
      #           lat_pos-half_width*math.cos(theta)]
      # num = num + 1
      # obs_info['obstacles_x'].append(obs_x)
      # obs_info['obstacles_y'].append(obs_y)
      # obs_info['pos_x'].append(lat_pos)
      # obs_info['pos_y'].append(long_pos)
      # obs_info['obstacles_vel'].append(obstacle_list[i].fusion_obstacle.common_info.relative_velocity.x)
      # obs_info['obstacles_acc'].append(obstacle_list[i].fusion_obstacle.common_info.relative_acceleration.x)
      # obs_info['obstacles_tid'].append(obstacle_list[i].fusion_obstacle.common_info.id)
      # obs_info['obs_label'].append(str(obstacle_list[i].fusion_obstacle.common_info.id) + ',v=' + str(round(obstacle_list[i].fusion_obstacle.common_info.relative_velocity.x, 2)))

      for j in range(len(obstacle_list[i].trajectory.trajectory_point)):
        # local_x = obstacle_list[i].trajectory.trajectory_point[j].relative_position.x
        # local_y = obstacle_list[i].trajectory.trajectory_point[j].relative_position.y
        global_x = obstacle_list[i].trajectory.trajectory_point[j].position.x
        global_y = obstacle_list[i].trajectory.trajectory_point[j].position.y
        global_yaw = obstacle_list[i].trajectory.trajectory_point[j].yaw
        cos_heading = math.cos(global_yaw)
        sin_heading = math.sin(global_yaw)
        dx1 = cos_heading * half_length
        dy1 = sin_heading * half_length
        dx2 = sin_heading * half_width
        dy2 = -cos_heading * half_width
        obs_x = [global_x + dx1 + dx2,
                 global_x + dx1 - dx2,
                 global_x - dx1 - dx2,
                 global_x - dx1 + dx2,
                 global_x + dx1 + dx2]
        obs_y = [global_y + dy1 + dy2,
                 global_y + dy1 - dy2,
                 global_y - dy1 - dy2,
                 global_y - dy1 + dy2,
                 global_y + dy1 + dy2]
        # print(global_x, global_y)
        if g_is_display_enu:
          p_x.append(global_x)
          p_y.append(global_y)
          p_obs_x.append(obs_x)
          p_obs_y.append(obs_y)
        else:
          coord_tf = coord_transformer()
          if localization_info != None:
            cur_pos_xn = localization_info.position.position_boot.x
            cur_pos_yn = localization_info.position.position_boot.y
            cur_yaw = localization_info.orientation.euler_boot.yaw
            coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
            local_x, local_y = coord_tf.global_to_local(global_x, global_y)
            p_x.append(local_x)
            p_y.append(local_y)
            obs_x_rel, obs_y_rel = coord_tf.global_to_local(obs_x, obs_y)
            p_obs_x.append(obs_x_rel)
            p_obs_y.append(obs_y_rel)
          else:
            p_x.append(global_x)
            p_y.append(global_y)
            p_obs_x.append(obs_x)
            p_obs_y.append(obs_y)
            print("load_prediction_objects: localization_info error!")

      # trajectory_info[track_id] = [p_x, p_y]
  trajectory_info['x']=p_x
  trajectory_info['y']=p_y
  trajectory_info['obs_x']=p_obs_x
  trajectory_info['obs_y']=p_obs_y

  for i in range(len(trajectory_info['x'])):
    index_object = (int)(i / 40)
    index = (int)((i - index_object * 40) / 8)
    if index > 4:
      break
    prediction_dict[index]['x'].append(trajectory_info['x'][i])
    prediction_dict[index]['y'].append(trajectory_info['y'][i])
    if obs_id_num > 0:
      prediction_dict[index]['obs_x'].append(trajectory_info['obs_x'][i])
      prediction_dict[index]['obs_y'].append(trajectory_info['obs_y'][i])
    else:
      prediction_dict[index]['obs_x'].append([])
      prediction_dict[index]['obs_y'].append([])

  return prediction_dict

def generate_planning_trajectory(trajectory, loc_msg = None, g_is_display_enu = False):
  plan_dict = {0: {'x': [], 'y': []},
               1: {'x': [], 'y': []},
               2: {'x': [], 'y': []},
               3: {'x': [], 'y': []},
               4: {'x': [], 'y': []}}
  plan_x = []
  plan_y = []
  plan_theta = []
  try:
    for i in range(len(trajectory.trajectory_points)):
      plan_x.append(trajectory.trajectory_points[i].x)
      plan_y.append(trajectory.trajectory_points[i].y)
      plan_theta.append(trajectory.trajectory_points[i].heading_yaw)

    if trajectory.target_reference.lateral_maneuver_gear == 2:
      pass
    else:
      if g_is_display_enu:
        for i in range(len(plan_x)):
          index = (int)(i / 40)
          if index > 4:
            break
          plan_dict[index]['x'].append(plan_x[i])
          plan_dict[index]['y'].append(plan_y[i])
      else:
        coord_tf = coord_transformer()
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw

        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        plan_x, plan_y = coord_tf.global_to_local(plan_x, plan_y)
        for i in range(len(plan_x)):
          index = (int)(i / 40)
          if index > 4:
            break
          plan_dict[index]['x'].append(plan_x[i])
          plan_dict[index]['y'].append(plan_y[i])
        for i in range(len(plan_theta)):
          tmp_theta = plan_theta[i] - cur_yaw
          plan_theta[i] = tmp_theta
  except:
    print('generate_planning_trajectory error')
  return plan_x, plan_y, plan_theta, plan_dict

def generate_ehr_static_map(ehr_static_map_msg, loc_msg = None, environment_model_info = None, g_is_display_enu = False):
  parking_space_boxes_x = []
  parking_space_boxes_y = []
  road_mark_boxes_x = []
  road_mark_boxes_y = []
  road_obstacle_x_vec = []
  road_obstacle_y_vec = []
  polygon_obstacle_x_vec = []
  polygon_obstacle_y_vec = []
  polygon_obstacle_label_vec = []
  polygon_x_vec = []
  polygon_y_vec = []
  polygon_id_vec = []
  polygon_obstacle_id = 8000000
  try:
    parking_assist_info = ehr_static_map_msg.parking_assist_info
    # print("parking_assist_info:", parking_assist_info)
    parking_spaces = parking_assist_info.parking_spaces
    road_obstacles = parking_assist_info.road_obstacles
    polygon_obstacles = []
    try:
      polygon_obstacles = parking_assist_info.polygon_obstacles
    except:
      pass
    # 车道级
    lanes = ehr_static_map_msg.road_map.lanes
    # print("lanes: ", lanes)
    # 道路级
    lane_groups = ehr_static_map_msg.road_map.lane_groups
    # print("lane_groups: ", lane_groups)
    if g_is_display_enu:
      for parking_space in parking_spaces:
        parking_space_box_x = []
        parking_space_box_y = []
        for shape in parking_space.shape:
          x = shape.x
          y = shape.y
          parking_space_box_x.append(x)
          parking_space_box_y.append(y)
        parking_space_boxes_x.append(parking_space_box_x)
        parking_space_boxes_y.append(parking_space_box_y)

      for lane in lane_groups:
        for road_mark in lane.road_marks:
          road_mark_box_x = []
          road_mark_box_y = []
          for shape in road_mark.shape:
            x = shape.x
            y = shape.y
            road_mark_box_x.append(x)
            road_mark_box_y.append(y)
          road_mark_boxes_x.append(road_mark_box_x)
          road_mark_boxes_y.append(road_mark_box_y)

      for road_obstacle in road_obstacles:
        road_obstacle_x = []
        road_obstacle_y = []
        for shape in road_obstacle.shape:
          x = shape.x
          y = shape.y
          road_obstacle_x.append(x)
          road_obstacle_y.append(y)
        road_obstacle_x_vec.append(road_obstacle_x)
        road_obstacle_y_vec.append(road_obstacle_y)

      for polygon_obstacle in polygon_obstacles:
        polygon_obstacle_id += 1
        polygon_id_vec.append(polygon_obstacle_id)
        polygon_obstacle_x = []
        polygon_obstacle_y = []
        for shape in polygon_obstacle.shape:
          x = shape.x
          y = shape.y
          polygon_obstacle_x.append(x)
          polygon_obstacle_y.append(y)
        polygon_obstacle_x_vec.append(polygon_obstacle_x)
        polygon_obstacle_y_vec.append(polygon_obstacle_y)
        polygon_x = []
        polygon_y = []
        lat_decision = "None"
        is_static = ""
        try:
          obs_polygon = []
          for obstacle in environment_model_info.obstacle:
            if obstacle.id == polygon_obstacle_id:
              obs_polygon = obstacle.polygon_points
              if (0 == obstacle.lat_decision):
                lat_decision = "LEFT"
              elif (1 == obstacle.lat_decision):
                lat_decision = "RIGHT"
              elif (2 == obstacle.lat_decision):
                lat_decision = "IGNORE"
              if obstacle.is_static:
                is_static = "Static"
              break
          for point in obs_polygon:
            polygon_x.append(point.x)
            polygon_y.append(point.y)
        except:
          pass
        polygon_x_vec.append(polygon_x)
        polygon_y_vec.append(polygon_y)
        polygon_obstacle_label_vec.append('(id)='\
            + str(polygon_obstacle_id) + '\n' \
            + lat_decision + '\n' + is_static)
    else:
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        for parking_space in parking_spaces:
          parking_space_box_x = []
          parking_space_box_y = []
          for shape in parking_space.shape:
            x = shape.x
            y = shape.y
            local_x, local_y = coord_tf.global_to_local([x], [y])
            parking_space_box_x.append(local_x)
            parking_space_box_y.append(local_y)
          parking_space_boxes_x.append(parking_space_box_x)
          parking_space_boxes_y.append(parking_space_box_y)

        for lane in lane_groups:
          for road_mark in lane.road_marks:
            road_mark_box_x = []
            road_mark_box_y = []
            for shape in road_mark.shape:
              x = shape.x
              y = shape.y
              local_x, local_y = coord_tf.global_to_local([x], [y])
              road_mark_box_x.append(local_x)
              road_mark_box_y.append(local_y)
            road_mark_boxes_x.append(road_mark_box_x)
            road_mark_boxes_y.append(road_mark_box_y)

        for road_obstacle in road_obstacles:
          road_obstacle_x = []
          road_obstacle_y = []
          for shape in road_obstacle.shape:
            x = shape.x
            y = shape.y
            local_x, local_y = coord_tf.global_to_local([x], [y])
            road_obstacle_x.append(local_x)
            road_obstacle_y.append(local_y)
          road_obstacle_x_vec.append(road_obstacle_x)
          road_obstacle_y_vec.append(road_obstacle_y)

        for polygon_obstacle in polygon_obstacles:
          polygon_obstacle_id += 1
          polygon_id_vec.append(polygon_obstacle_id)
          polygon_obstacle_x = []
          polygon_obstacle_y = []
          for shape in polygon_obstacle.shape:
            x = shape.x
            y = shape.y
            local_x, local_y = coord_tf.global_to_local([x], [y])
            polygon_obstacle_x.append(local_x)
            polygon_obstacle_y.append(local_y)
          polygon_obstacle_x_vec.append(polygon_obstacle_x)
          polygon_obstacle_y_vec.append(polygon_obstacle_y)
          polygon_x = []
          polygon_y = []
          lat_decision = "None"
          is_static = ""
          try:
            obs_polygon = []
            for obstacle in environment_model_info.obstacle:
              if obstacle.id == polygon_obstacle_id:
                obs_polygon = obstacle.polygon_points
                if (0 == obstacle.lat_decision):
                  lat_decision = "LEFT"
                elif (1 == obstacle.lat_decision):
                  lat_decision = "RIGHT"
                elif (2 == obstacle.lat_decision):
                  lat_decision = "IGNORE"
                if obstacle.is_static:
                  is_static = "Static"
                break
            for point in obs_polygon:
              local_x, local_y = coord_tf.global_to_local([point.x], [point.y])
              polygon_x.append(local_x)
              polygon_y.append(local_y)
          except:
            pass
          polygon_obstacle_label_vec.append('(id)='\
              + str(polygon_obstacle_id) + '\n' \
              + lat_decision + '\n' + is_static)
          polygon_x_vec.append(polygon_x)
          polygon_y_vec.append(polygon_y)
  except:
    print('generate_ehr_static_map error')
  return parking_space_boxes_x, parking_space_boxes_y, \
         road_mark_boxes_x, road_mark_boxes_y, \
         road_obstacle_x_vec, road_obstacle_y_vec, \
         polygon_obstacle_x_vec, polygon_obstacle_y_vec, polygon_obstacle_label_vec, \
         polygon_x_vec, polygon_y_vec, polygon_id_vec

def generate_ground_line(ground_line_msg, loc_msg = None, environment_model_info = None, g_is_display_enu = False):
  groundline_x_vec = []
  groundline_y_vec = []
  groundline_id_vec = []
  pos_x_vec = []
  pos_y_vec = []
  ground_line_label_vec = []
  polygon_x_vec = []
  polygon_y_vec = []
  try:
    groundline_size = ground_line_msg.groundline_size
    ground_lines = ground_line_msg.groundline
    ground_line_id = 5000000
    if g_is_display_enu:
      for j in range(groundline_size):
        groundline = ground_lines[j]
        type = groundline.type
        resource_type = groundline.resource_type
        single_groundline_x_vec = []
        single_groundline_y_vec = []
        groundline_point_size = groundline.groundline_point_size
        for k in range(groundline_point_size):
          ground_x_enu = groundline.shape[k].x
          ground_y_enu = groundline.shape[k].y
          single_groundline_x_vec.append(ground_x_enu)
          single_groundline_y_vec.append(ground_y_enu)
        groundline_x_vec.append(single_groundline_x_vec)
        groundline_y_vec.append(single_groundline_y_vec)
        polygon_x = []
        polygon_y = []
        lat_decision = "None"
        is_static = ""
        ground_line_id = ground_line_id + 1
        groundline_id_vec.append(ground_line_id)
        try:
          obs_polygon = []
          for obstacle in environment_model_info.obstacle:
            if obstacle.id == ground_line_id:
              obs_polygon = obstacle.polygon_points
              if (0 == obstacle.lat_decision):
                lat_decision = "LEFT"
              elif (1 == obstacle.lat_decision):
                lat_decision = "RIGHT"
              elif (2 == obstacle.lat_decision):
                lat_decision = "IGNORE"
              if obstacle.is_static:
                is_static = "Static"
              break
          for point in obs_polygon:
            polygon_x.append(point.x)
            polygon_y.append(point.y)
        except:
          pass
        polygon_x_vec.append(polygon_x)
        polygon_y_vec.append(polygon_y)
        pos_x_vec.append(single_groundline_x_vec[0])
        pos_y_vec.append(single_groundline_y_vec[0])
        ground_line_label_vec.append('(id,type,resource)=' + '\n'\
            + str(ground_line_id) + ',' + str(type) + ',' + str(resource_type) + '\n'\
            + lat_decision + '\n' + is_static)
    else:
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        for j in range(groundline_size):
          groundline = ground_lines[j]
          type = groundline.type
          resource_type = groundline.resource_type
          single_groundline_x_vec = []
          single_groundline_y_vec = []
          groundline_point_size = groundline.groundline_point_size
          for k in range(groundline_point_size):
            ground_x_enu = groundline.shape[k].x
            ground_y_enu = groundline.shape[k].y
            local_x, local_y = coord_tf.global_to_local([ground_x_enu], [ground_y_enu])
            single_groundline_x_vec.append(local_x)
            single_groundline_y_vec.append(local_y)
          groundline_x_vec.append(single_groundline_x_vec)
          groundline_y_vec.append(single_groundline_y_vec)
          polygon_x = []
          polygon_y = []
          lat_decision = "None"
          is_static = ""
          ground_line_id = ground_line_id + 1
          groundline_id_vec.append(ground_line_id)
          try:
            obs_polygon = []
            for obstacle in environment_model_info.obstacle:
              if obstacle.id == ground_line_id:
                obs_polygon = obstacle.polygon_points
                if (0 == obstacle.lat_decision):
                  lat_decision = "LEFT"
                elif (1 == obstacle.lat_decision):
                  lat_decision = "RIGHT"
                elif (2 == obstacle.lat_decision):
                  lat_decision = "IGNORE"
                if obstacle.is_static:
                  is_static = "Static"
                break
            for point in obs_polygon:
              local_x, local_y = coord_tf.global_to_local([point.x], [point.y])
              polygon_x.append(local_x)
              polygon_y.append(local_y)
          except:
            pass
          pos_x_vec.append(single_groundline_x_vec[0])
          pos_y_vec.append(single_groundline_y_vec[0])
          ground_line_label_vec.append('(id,type,resource)=' + '\n'\
              + str(ground_line_id) + ',' + str(type) + ',' + str(resource_type) + '\n'\
              + lat_decision + '\n' + is_static)
          polygon_x_vec.append(polygon_x)
          polygon_y_vec.append(polygon_y)
  except:
    print('groundline error')
  return groundline_x_vec, groundline_y_vec, groundline_id_vec, pos_x_vec, pos_y_vec, ground_line_label_vec, polygon_x_vec, polygon_y_vec

def generate_rdg_ground_line(ground_line_msg, loc_msg = None, g_is_display_enu = False):
  groundline_x_vec = []
  groundline_y_vec = []
  try:
    groundline_size = ground_line_msg.ground_lines_size
    ground_lines = ground_line_msg.ground_lines
    if not g_is_display_enu:
      for j in range(groundline_size):
        groundline = ground_lines[j]
        single_groundline_x_vec = []
        single_groundline_y_vec = []
        groundline_point_size = groundline.points_2d_size
        for k in range(groundline_point_size):
          ground_x_enu = groundline.points_2d[k].x
          ground_y_enu = groundline.points_2d[k].y
          single_groundline_x_vec.append(ground_x_enu)
          single_groundline_y_vec.append(ground_y_enu)
        groundline_x_vec.append(single_groundline_x_vec)
        groundline_y_vec.append(single_groundline_y_vec)
    else:
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        for j in range(groundline_size):
          groundline = ground_lines[j]
          single_groundline_x_vec = []
          single_groundline_y_vec = []
          groundline_point_size = groundline.points_2d_size
          for k in range(groundline_point_size):
            ground_x_enu = groundline.points_2d[k].x
            ground_y_enu = groundline.points_2d[k].y
            # local_x, local_y = coord_tf.global_to_local([ground_x_enu], [ground_y_enu])
            local_x, local_y = coord_tf.local_to_global([ground_x_enu], [ground_y_enu])
            single_groundline_x_vec.append(local_x)
            single_groundline_y_vec.append(local_y)
          groundline_x_vec.append(single_groundline_x_vec)
          groundline_y_vec.append(single_groundline_y_vec)
  except:
    print('rdg ground line error')
  return groundline_x_vec, groundline_y_vec

def generate_control(control_msg, loc_msg = None, g_is_display_enu = False):
  mpc_dx = []
  mpc_dy = []
  mpc_dtheta = []
  if control_msg.control_status.available:
    control_result_points_size = control_msg.control_trajectory.control_result_points_size
    control_result_points = control_msg.control_trajectory.control_result_points
    #print("real control_result_points size: ", control_result_points_size)
    for i in range(control_result_points_size):
      mpc_dx.append(control_result_points[i].x)
      mpc_dy.append(control_result_points[i].y)
    if len(mpc_dx) > 0:
      f1 = interp1d(mpc_dx, mpc_dy, kind='cubic')
      try:
        for i in range(len(mpc_dx) - 1):
          mpc_dtheta.append(math.atan(derivative(f1, mpc_dx[i] + 1e-6, dx = 1e-6)))
        mpc_dtheta.append(math.atan(derivative(f1, mpc_dx[len(mpc_dx) - 1] - 1e-6, dx = 1e-6)))
      except:
        mpc_dtheta = len(mpc_dx) * [0]
    if g_is_display_enu:
      if loc_msg == None:
        mpc_dx = len(mpc_dx) * [0]
        mpc_dy = len(mpc_dy) * [0]
        return mpc_dx, mpc_dy, mpc_dtheta
      coord_tf = coord_transformer()
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
      mpc_dx, mpc_dy = coord_tf.local_to_global(mpc_dx, mpc_dy)
      for i in range(len(mpc_dtheta)):
        mpc_dtheta[i] = mpc_dtheta[i] + cur_yaw
  else:
    # print("control_status available is false")
    pass
  return mpc_dx, mpc_dy, mpc_dtheta

# GroundLinePoint & GroundLineDecider & generate_ground_line_clusters must be consistent with C++ code
class GroundLinePoint:
  class Status:
      UNCLASSIFIED = 0
      CLASSIFIED = 1
      NOISE = 2

  def __init__(self, point, status):
      self.point = point
      self.status = status

  def __eq__(self, other):
      return self.point == other.point and self.status == other.status

  def __ne__(self, other):
      return not (self.point == other.point and self.status == other.status)

class GroundLineDecider:
  min_pts = 3
  eps = 0.5

  def update_params(min_pts, eps):
    GroundLineDecider.min_pts = min_pts
    GroundLineDecider.eps = eps

  def execute(ground_line_points):
    result = []
    points = ground_line_points
    for point in points:
      if point.status == GroundLinePoint.Status.UNCLASSIFIED:
        cluster = []
        GroundLineDecider.expand_cluster(point, points, cluster)
        if cluster:
          result.append(cluster)
    return result

  def calc_cluster(point, points, cluster_index):
    for i in range(len(points)):
      if point != points[i] and math.sqrt((point.point[0] - points[i].point[0])**2 + (point.point[1] - points[i].point[1])**2) <= GroundLineDecider.eps:
        cluster_index.append(i)

  def expand_cluster(point, points, result):
    result.clear()
    cluster = []
    GroundLineDecider.calc_cluster(point, points, cluster)
    if len(cluster) < GroundLineDecider.min_pts:
      point.status = GroundLinePoint.Status.NOISE
      return

    point.status = GroundLinePoint.Status.CLASSIFIED
    result.append(point.point)
    index = 0
    for c_index in cluster:
      index += 1
      cluster_exp = []
      GroundLineDecider.calc_cluster(points[c_index], points, cluster_exp)
      points[c_index].status = GroundLinePoint.Status.CLASSIFIED
      result.append(points[c_index].point)

      if len(cluster_exp) >= GroundLineDecider.min_pts:
        for j in range(len(cluster_exp)):
          if points[cluster_exp[j]].status == GroundLinePoint.Status.UNCLASSIFIED:
            if cluster_exp[j] not in cluster:
              cluster.append(cluster_exp[j])
          elif points[cluster_exp[j]].status == GroundLinePoint.Status.NOISE:
            points[cluster_exp[j]].status = GroundLinePoint.Status.CLASSIFIED
            result.append(points[cluster_exp[j]].point)

def generate_ground_line_clusters(ground_line_msg, loc_msg = None, g_is_display_enu = False):
  groundline_point_x_vec = []
  groundline_point_y_vec = []
  groundline_points = []
  try:
    ground_lines = ground_line_msg.groundline
    groundline_size = ground_line_msg.groundline_size
    if g_is_display_enu:
      for j in range(groundline_size):
        groundline = ground_lines[j]
        groundline_point_size = groundline.groundline_point_size
        for k in range(groundline_point_size):
          ground_x_enu = groundline.shape[k].x
          ground_y_enu = groundline.shape[k].y
          groundline_point_x_vec.append(ground_x_enu)
          groundline_point_y_vec.append(ground_y_enu)
          groundline_point = GroundLinePoint([ground_x_enu, ground_y_enu], GroundLinePoint.Status.UNCLASSIFIED)
          groundline_points.append(groundline_point)

    else:
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        for j in range(groundline_size):
          groundline = ground_lines[j]
          groundline_point_size = groundline.groundline_point_size
          for k in range(groundline_point_size):
            ground_x_enu = groundline.shape[k].x
            ground_y_enu = groundline.shape[k].y
            local_x, local_y = coord_tf.global_to_local([ground_x_enu], [ground_y_enu])
            groundline_point_x_vec.append(local_x)
            groundline_point_y_vec.append(local_y)
            groundline_point = GroundLinePoint([ground_x_enu, ground_y_enu], GroundLinePoint.Status.UNCLASSIFIED)
            groundline_points.append(groundline_point)
  except:
    print('groundline error')

  clusters = GroundLineDecider.execute(groundline_points)
  groundline_x_vec = []
  groundline_y_vec = []
  groundline_id_vec = []
  kGroundLineIdOffset = 5000000
  for cluster in clusters:
    single_groundline_x_vec = []
    single_groundline_y_vec = []
    single_groundline_id_vec = []
    kGroundLineIdOffset += 1
    single_groundline_id_vec.append(kGroundLineIdOffset)
    for groundline_point in cluster:
      single_groundline_x_vec.append(groundline_point[0])
      single_groundline_y_vec.append(groundline_point[1])
    groundline_x_vec.append(single_groundline_x_vec)
    groundline_y_vec.append(single_groundline_y_vec)
    groundline_id_vec.append(single_groundline_id_vec)
  return groundline_point_x_vec, groundline_point_y_vec, groundline_x_vec, groundline_y_vec, groundline_id_vec

# ParkingSlotManager & hpp_generate_ehr_static_map must be consistent with C++ code
class ParkingSlotManager:
  kMaxDistanceY = 5
  kMaxDistanceFrontX = 40
  kMaxDistanceBackX = 30

  def update(parking_info, loc_info, is_fusion = False):
    points_ = []
    if is_fusion:
      parking_fusion_slot_lists_size = parking_info.parking_fusion_slot_lists_size
      for i in range(parking_fusion_slot_lists_size):
        park_space = parking_info.parking_fusion_slot_lists[i]
        corner_points = park_space.corner_points
        slot_point = []
        if len(corner_points) != 4:
          continue
        min_x, min_y, max_x, max_y = float('inf'), float('inf'), float('-inf'), float('-inf')
        for lot_point in corner_points:
          v = np.array([[lot_point.x], [lot_point.y], [0]])
          park_space_point_car = ParkingSlotManager.enu2car_matrix(loc_info, v, True)
          min_x = min(min_x, park_space_point_car[0])
          min_y = min(min_y, park_space_point_car[1])
          max_x = max(max_x, park_space_point_car[0])
          max_y = max(max_y, park_space_point_car[1])
          slot_point.append([lot_point.x, lot_point.y])
        if ((min_y > 0 and min_y < ParkingSlotManager.kMaxDistanceY) or (max_y < 0 and max_y > -ParkingSlotManager.kMaxDistanceY) or (min_y <= 0 and max_y >=0)) and min_x < ParkingSlotManager.kMaxDistanceFrontX and max_x > -ParkingSlotManager.kMaxDistanceBackX:
          points_.append(slot_point)
    else:
      park_spaces = parking_info.parking_assist_info.parking_spaces
      for park_space in park_spaces:
        slot_point = []
        if len(park_space.shape) != 4:
          continue
        min_x, min_y, max_x, max_y = float('inf'), float('inf'), float('-inf'), float('-inf')
        for lot_point in park_space.shape:
          v = np.array([[lot_point.x], [lot_point.y], [lot_point.z]])
          park_space_point_car = ParkingSlotManager.enu2car_matrix(loc_info, v)
          min_x = min(min_x, park_space_point_car[0])
          min_y = min(min_y, park_space_point_car[1])
          max_x = max(max_x, park_space_point_car[0])
          max_y = max(max_y, park_space_point_car[1])
          slot_point.append([lot_point.x, lot_point.y])
        if ((min_y > 0 and min_y < ParkingSlotManager.kMaxDistanceY) or (max_y < 0 and max_y > -ParkingSlotManager.kMaxDistanceY) or (min_y <= 0 and max_y >=0)) and min_x < ParkingSlotManager.kMaxDistanceFrontX and max_x > -ParkingSlotManager.kMaxDistanceBackX:
          points_.append(slot_point)
    return points_

  def enu2car_matrix(loc_info, v, is_fusion = False):
    o = loc_info.orientation.quaternion_boot
    p = loc_info.position.position_boot
    if is_fusion:
      m_basis = ParkingSlotManager.calculate_m_basis(np.array([o.x, o.y, o.z, o.w]))
      m_origin = np.array([[p.x], [p.y], [0]])
      enu2car_q = m_basis.T
      enu2car_v = enu2car_q.dot(-1 * m_origin)
      return enu2car_q.dot(v) + enu2car_v
    else:
      m_basis = ParkingSlotManager.calculate_m_basis(np.array([o.x, o.y, o.z, o.w]))
      m_origin = np.array([[p.x], [p.y], [p.z]])
      enu2car_q = m_basis.T
      enu2car_v = enu2car_q.dot(-1 * m_origin)
      return enu2car_q.dot(v) + enu2car_v

  def calculate_m_basis(q):
    d = q.dot(q)
    k_epsilon = 1.0e-10
    if d >= 0.0:
      d = max(d, k_epsilon)
    else:
      d = min(d, -k_epsilon)
    s = 2.0 / d
    xs = q[0] * s
    ys = q[1] * s
    zs = q[2] * s
    wx = q[3] * xs
    wy = q[3] * ys
    wz = q[3] * zs
    xx = q[0] * xs
    xy = q[0] * ys
    xz = q[0] * zs
    yy = q[1] * ys
    yz = q[1] * zs
    zz = q[2] * zs
    return np.array([[(1.0 - (yy + zz)), (xy - wz), (xz + wy)],
                     [(xy + wz), (1.0 - (xx + zz)), (yz - wx)],
                     [(xz - wy), (yz + wx), (1.0 - (xx + yy))]])

def hpp_generate_ehr_static_map(ehr_static_map_msg, loc_msg, g_is_display_enu = False):
  # road_mark_boxes_x = []
  # road_mark_boxes_y = []
  parking_space_boxes_x = []
  parking_space_boxes_y = []
  parking_space_boxes_id = []
  kParkingSlotIdOffset = 6000000
  try:
    parking_slot_points = ParkingSlotManager.update(ehr_static_map_msg, loc_msg)
    # road_mark_info = ehr_static_map_msg.
    if g_is_display_enu:
      for parking_slot_point in parking_slot_points:
        if len(parking_slot_point) != 4:
          continue
        kParkingSlotIdOffset += 1
        parking_space_box_x = []
        parking_space_box_y = []
        for points in parking_slot_point:
          parking_space_box_x.append(points[0])
          parking_space_box_y.append(points[1])
        parking_space_boxes_x.append(parking_space_box_x)
        parking_space_boxes_y.append(parking_space_box_y)
        parking_space_boxes_id.append(kParkingSlotIdOffset)

      # for road_mark in road_mark_info:
        # road_mark_box_x = []
        # road_mark_box_y = []
        # for shape in road_mark.shape:
          # x = shape.boot.x
          # y = shape.boot.y
          # road_mark_box_x.append(x)
          # road_mark_box_y.append(y)
        # road_mark_boxes_x.append(road_mark_box_x)
        # road_mark_boxes_y.append(road_mark_box_y)
    else:
      coord_tf = coord_transformer()
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
      for parking_slot_point in parking_slot_points:
        if len(parking_slot_point) != 4:
          continue
        kParkingSlotIdOffset += 1
        parking_space_box_x = []
        parking_space_box_y = []
        for points in parking_slot_point:
          local_x, local_y = coord_tf.global_to_local(points[0], points[1])
          parking_space_box_x.append(local_x)
          parking_space_box_y.append(local_y)
        parking_space_boxes_x.append(parking_space_box_x)
        parking_space_boxes_y.append(parking_space_box_y)
        parking_space_boxes_id.append(kParkingSlotIdOffset)

      # for road_mark in road_mark_info:
        # road_mark_box_x = []
        # road_mark_box_y = []
        # for shape in road_mark.shape:
          # x = shape.boot.x
          # y = shape.boot.y
          # local_x, local_y = coord_tf.global_to_local([x], [y])
          # road_mark_box_x.append(local_x)
          # road_mark_box_y.append(local_y)
        # road_mark_boxes_x.append(road_mark_box_x)
        # road_mark_boxes_y.append(road_mark_box_y)
  except:
    print('hpp_generate_ehr_static_map error')
  return parking_space_boxes_x, parking_space_boxes_y, parking_space_boxes_id  # , road_mark_boxes_x, road_mark_boxes_y

def hpp_generate_parking_slot(fus_parking_msg, loc_msg, g_is_display_enu = False):
  # road_mark_boxes_x = []
  # road_mark_boxes_y = []
  parking_space_boxes_x = []
  parking_space_boxes_y = []
  parking_space_boxes_id = []
  kParkingSlotIdOffset = 6000000
  try:
    parking_slot_points = ParkingSlotManager.update(fus_parking_msg, loc_msg, True)
    if g_is_display_enu:
      for parking_slot_point in parking_slot_points:
        if len(parking_slot_point) != 4:
          continue
        kParkingSlotIdOffset += 1
        parking_space_box_x = []
        parking_space_box_y = []
        for points in parking_slot_point:
          parking_space_box_x.append(points[0])
          parking_space_box_y.append(points[1])
        parking_space_boxes_x.append(parking_space_box_x)
        parking_space_boxes_y.append(parking_space_box_y)
        parking_space_boxes_id.append(kParkingSlotIdOffset)
    else:
      coord_tf = coord_transformer()
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
      for parking_slot_point in parking_slot_points:
        if len(parking_slot_point) != 4:
          continue
        kParkingSlotIdOffset += 1
        parking_space_box_x = []
        parking_space_box_y = []
        for points in parking_slot_point:
          local_x, local_y = coord_tf.global_to_local(points[0], points[1])
          parking_space_box_x.append(local_x)
          parking_space_box_y.append(local_y)
        parking_space_boxes_x.append(parking_space_box_x)
        parking_space_boxes_y.append(parking_space_box_y)
        parking_space_boxes_id.append(kParkingSlotIdOffset)
  except:
    print('hpp_generate_parking_slot error')
  return parking_space_boxes_x, parking_space_boxes_y, parking_space_boxes_id

def generate_parking_slot(fus_parking_msg, loc_msg, release_slot_id_list):
  parking_slot_info = {
    'parking_slot_x_rel': [],
    'parking_slot_y_rel': [],
    'parking_slot_x': [],
    'parking_slot_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'parking_slot_label':[]
  }
  release_slot_info = {
    'parking_slot_x_rel': [],
    'parking_slot_y_rel': [],
    'parking_slot_x': [],
    'parking_slot_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'parking_slot_label':[]
  }
  plan_release_slot_info = {
    'parking_slot_x_rel': [],
    'parking_slot_y_rel': [],
    'parking_slot_x': [],
    'parking_slot_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'parking_slot_label':[]
  }
  select_parking_slot_info = {
    'parking_slot_x_rel': [],
    'parking_slot_y_rel': [],
    'parking_slot_x': [],
    'parking_slot_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'parking_slot_label':[]
  }
  try:
    select_slot_id = fus_parking_msg.select_slot_id
    print("select_slot_id: ", select_slot_id)
    parking_fusion_slot_lists_size = fus_parking_msg.parking_fusion_slot_lists_size
    for j in range(parking_fusion_slot_lists_size):
      parking_fusion_slot_lists = fus_parking_msg.parking_fusion_slot_lists[j]
      id = parking_fusion_slot_lists.id
      is_release = False
      for release_id in release_slot_id_list:
        if id == release_id.id:
          is_release = True
          break
      # uss_id = parking_fusion_slot_lists.uss_id
      resource_type = parking_fusion_slot_lists.resource_type
      fusion_source = parking_fusion_slot_lists.fusion_source
      type = parking_fusion_slot_lists.type
      allow_parking = parking_fusion_slot_lists.allow_parking

      corner_points = parking_fusion_slot_lists.corner_points
      pos_x = 0.0
      pos_y = 0.0
      parking_slot_x, parking_slot_y = [], []
      for index, points in enumerate(corner_points):
        parking_slot_x.append(points.x)
        parking_slot_y.append(points.y)
        pos_x += points.x
        pos_y += points.y

      pos_x /= 4.0
      pos_y /= 4.0

      pos_x_rel, pos_y_rel = [], []
      parking_slot_x_rel, parking_slot_y_rel = [], []
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        parking_slot_x_rel, parking_slot_y_rel = coord_tf.global_to_local(parking_slot_x, parking_slot_y)
        pos_x_rel, pos_y_rel = coord_tf.global_to_local([pos_x], [pos_y])

      if id == select_slot_id:
        select_parking_slot_info['parking_slot_x_rel'].append(parking_slot_x_rel)
        select_parking_slot_info['parking_slot_y_rel'].append(parking_slot_y_rel)
        select_parking_slot_info['pos_x_rel'].append(pos_x_rel)
        select_parking_slot_info['pos_y_rel'].append(pos_y_rel)
        select_parking_slot_info['parking_slot_x'].append(parking_slot_x)
        select_parking_slot_info['parking_slot_y'].append(parking_slot_y)
        select_parking_slot_info['pos_x'].append([pos_x])
        select_parking_slot_info['pos_y'].append([pos_y])
        select_parking_slot_info['parking_slot_label'].append(['(' + str(id) + ',' + str(resource_type) + ',' +  str(allow_parking) +')'])
      elif is_release:
        plan_release_slot_info['parking_slot_x_rel'].append(parking_slot_x_rel)
        plan_release_slot_info['parking_slot_y_rel'].append(parking_slot_y_rel)
        plan_release_slot_info['pos_x_rel'].append(pos_x_rel)
        plan_release_slot_info['pos_y_rel'].append(pos_y_rel)
        plan_release_slot_info['parking_slot_x'].append(parking_slot_x)
        plan_release_slot_info['parking_slot_y'].append(parking_slot_y)
        plan_release_slot_info['pos_x'].append([pos_x])
        plan_release_slot_info['pos_y'].append([pos_y])
        plan_release_slot_info['parking_slot_label'].append(['(' + str(id) + ',' + str(resource_type) + ')'])
      elif allow_parking == 1:
        release_slot_info['parking_slot_x_rel'].append(parking_slot_x_rel)
        release_slot_info['parking_slot_y_rel'].append(parking_slot_y_rel)
        release_slot_info['pos_x_rel'].append(pos_x_rel)
        release_slot_info['pos_y_rel'].append(pos_y_rel)
        release_slot_info['parking_slot_x'].append(parking_slot_x)
        release_slot_info['parking_slot_y'].append(parking_slot_y)
        release_slot_info['pos_x'].append([pos_x])
        release_slot_info['pos_y'].append([pos_y])
        release_slot_info['parking_slot_label'].append(['(' + str(id) + ',' + str(resource_type) + ')'])
      else:
        parking_slot_info['parking_slot_x_rel'].append(parking_slot_x_rel)
        parking_slot_info['parking_slot_y_rel'].append(parking_slot_y_rel)
        parking_slot_info['pos_x_rel'].append(pos_x_rel)
        parking_slot_info['pos_y_rel'].append(pos_y_rel)
        parking_slot_info['parking_slot_x'].append(parking_slot_x)
        parking_slot_info['parking_slot_y'].append(parking_slot_y)
        parking_slot_info['pos_x'].append([pos_x])
        parking_slot_info['pos_y'].append([pos_y])
        parking_slot_info['parking_slot_label'].append(['(' + str(id) + ',' + str(resource_type) + ')'])
  except:
    print("fus_parking_msg error")
  return parking_slot_info, release_slot_info, plan_release_slot_info, select_parking_slot_info


def generate_rdg_parking_slot(rdg_parking_msg, loc_msg):
  parking_slot_info = {
    'parking_slot_x_rel': [],
    'parking_slot_y_rel': [],
    'parking_slot_x': [],
    'parking_slot_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'parking_slot_label':[]
  }
  try:
    parking_slots_size = rdg_parking_msg.parking_slots_size
    for j in range(parking_slots_size):
      parking_camera_slot = rdg_parking_msg.parking_slots[j]
      id = parking_camera_slot.id
      type = parking_camera_slot.type
      allow_parking = parking_camera_slot.allow_parking
      corner_points = parking_camera_slot.corner_points
      pos_x_rel = 0.0
      pos_y_rel = 0.0
      parking_slot_x_rel, parking_slot_y_rel = [], []
      for index, points in enumerate(corner_points):
        parking_slot_x_rel.append(points.x)
        parking_slot_y_rel.append(points.y)
        pos_x_rel += points.x
        pos_y_rel += points.y

      pos_x_rel /= 4.0
      pos_y_rel /= 4.0

      pos_x, pos_y= [], []
      parking_slot_x, parking_slot_y = [], []
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        parking_slot_x, parking_slot_y = coord_tf.local_to_global(parking_slot_x_rel, parking_slot_y_rel)
        pos_x, pos_y = coord_tf.local_to_global([pos_x_rel], [pos_y_rel])

      parking_slot_info['parking_slot_x_rel'].append(parking_slot_x_rel)
      parking_slot_info['parking_slot_y_rel'].append(parking_slot_y_rel)
      parking_slot_info['pos_x_rel'].append(pos_x_rel)
      parking_slot_info['pos_y_rel'].append(pos_y_rel)
      parking_slot_info['parking_slot_x'].append(parking_slot_x)
      parking_slot_info['parking_slot_y'].append(parking_slot_y)
      parking_slot_info['pos_x'].append([pos_x])
      parking_slot_info['pos_y'].append([pos_y])
      parking_slot_info['parking_slot_label'].append(['(' + str(id) + ',' + str(allow_parking) + ',' + str(type) + ')'])
  except:
    print("rdg_parking_msg error")
  return parking_slot_info


def generate_speed_bump(fus_speed_bump_msg, loc_msg):
  speed_bump_info = {
    'speed_bump_x_rel': [],
    'speed_bump_y_rel': [],
    'speed_bump_x': [],
    'speed_bump_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'speed_bump_label':[]
  }
  try:
    decelers_size = fus_speed_bump_msg.decelers_size
    for j in range(decelers_size):
      decelers = fus_speed_bump_msg.decelers[j]
      id = decelers.id
      # resource_type = decelers.resource_type
      deceler_points = decelers.deceler_points
      pos_x = 0.0
      pos_y = 0.0
      speed_bump_x, speed_bump_y = [], []
      for index, points in enumerate(deceler_points):
        speed_bump_x.append(points.x)
        speed_bump_y.append(points.y)
        pos_x += points.x
        pos_y += points.y

      pos_x /= 4.0
      pos_y /= 4.0

      pos_x_rel, pos_y_rel = [], []
      speed_bump_x_rel, speed_bump_y_rel = [], []
      coord_tf = coord_transformer()
      if loc_msg != None: # 长时轨迹
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
        speed_bump_x_rel, speed_bump_y_rel = coord_tf.global_to_local(speed_bump_x, speed_bump_y)
        pos_x_rel, pos_y_rel = coord_tf.global_to_local([pos_x], [pos_y])

      speed_bump_info['speed_bump_x_rel'].append(speed_bump_x_rel)
      speed_bump_info['speed_bump_y_rel'].append(speed_bump_y_rel)
      speed_bump_info['pos_x_rel'].append(pos_x_rel)
      speed_bump_info['pos_y_rel'].append(pos_y_rel)
      speed_bump_info['speed_bump_x'].append(speed_bump_x)
      speed_bump_info['speed_bump_y'].append(speed_bump_y)
      speed_bump_info['pos_x'].append(pos_x)
      speed_bump_info['pos_y'].append(pos_y)
      speed_bump_info['speed_bump_label'].append(str(id))
  except:
    print("fus_speed_bump_msg error")
  return speed_bump_info


def load_lat_common(plan_debug, planning_json):
  vo_lat_motion_plan = plan_debug.vo_lat_motion_plan
  vo_lat_behavior_plan = plan_debug.vo_lat_behavior_plan
  lat_behavior_common = plan_debug.lat_behavior_common
  vars = ['fix_lane_virtual_id','target_lane_virtual_id','origin_lane_virtual_id',\
          'lc_request','lc_request_source','turn_light','map_turn_light','lc_turn_light','act_request_source','lc_back_invalid_reason','lc_status',\
            'is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',\
      'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',\
        'v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',\
          'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',\
            'is_side_borrow_bicycle_lane','is_side_borrow_lane','has_origin_lane',\
              'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason', ]
  # 'near_car_ids_origin','near_car_ids_target', 'left_alc_car_ids','right_alc_car_ids', ,'avoid_car_ids','avoid_car_allow_max_opposite_offset'
  data_dict1 = {}
  for name in vars:
    try:
      data_dict1[name] = getattr(lat_behavior_common,name)
    except:
      pass

  data_dict2 = {}
  try:
  # 横向运动规划offset 可视化
    basic_dpoly = vo_lat_motion_plan.basic_dpoly
    data_dict2['premove_dpoly_c0'] = vo_lat_motion_plan.premove_dpoly_c0 - basic_dpoly[3]
    data_dict2['avoid_dpoly_c0'] = vo_lat_motion_plan.avoid_dpoly_c0 - basic_dpoly[3]
  except:
    pass

  avoid_car_id_str = ""
  for avoid_car_id in vo_lat_behavior_plan.avoid_car_ids:
    avoid_car_id_str = avoid_car_id_str + str(avoid_car_id) + ' '
  left_alc_car_id_str = ""
  right_alc_car_id_str = ""
  for left_alc_car_id in lat_behavior_common.left_alc_car_ids:
    left_alc_car_id_str = left_alc_car_id_str + str(left_alc_car_id) + ' '
  for right_alc_car_id in lat_behavior_common.right_alc_car_ids:
    right_alc_car_id_str = right_alc_car_id_str + str(right_alc_car_id) + ' '
  # 添加可视化left_alc_car_ids、right_alc_car_ids可视化

  avoid_debug_key = ["avoid_car_id", "avoid_car_ids_1", "avoid_car_ids_2", \
                        "select_avoid_car_ids_1", "select_avoid_car_ids_2","lat_offset", "smooth_lateral_offset", "lane_width", "normal_right_avoid_threshold","normal_left_avoid_threshold", "avoid_way", "is_use_ego_position",\
                        "allow_side_max_opposite_offset", "allow_side_max_opposite_offset_id", \
                        "allow_front_max_opposite_offset", "allow_front_max_opposite_offset_id", "ego_l"]
  for key in avoid_debug_key:
    try:
      data_dict2[key] = planning_json[key]
    except:
      pass

  try:
    data_dict2['lateral_offset'] = planning_json["lateral_offset"]
  except:
    pass
  try:
    data_dict2['avoid_way'] = planning_json["avoid_way"]
  except:
    pass
  return data_dict1, data_dict2

def load_avoid(plan_debug, planning_json):

  data_dict2 = {}
  avoid_debug_key = ["avoid_car_id", "avoid_car_ids_1", "avoid_car_ids_2", \
                        "select_avoid_car_ids_1", "select_avoid_car_ids_2","lat_offset", "smooth_lateral_offset", "lane_width", "normal_right_avoid_threshold","normal_left_avoid_threshold", "avoid_way", "is_use_ego_position",\
                        "allow_side_max_opposite_offset", "allow_side_max_opposite_offset_id", \
                        "allow_front_max_opposite_offset", "allow_front_max_opposite_offset_id", "ego_l"]
  for key in avoid_debug_key:
    try:
      data_dict2[key] = planning_json[key]
    except:
      data_dict2[key] = 'none'
      pass

  # try:
  #   data_dict2['lateral_offset'] = planning_json["lateral_offset"]
  # except:
  #   data_dict2[key] = 'none'
  #   pass
  # try:
  #   data_dict2['avoid_way'] = planning_json["avoid_way"]
  # except:
  #   pass
  return  data_dict2

def load_time_cost(plan_debug, planning_json):
  time_cost_keys = ["VisionLateralBehaviorPlannerCost", "VisionLateralMotionPlannerCost","VisionLongitudinalBehaviorPlannerCost", \
         "EnvironmentalModelManagerCost", "GeneralPlannerModuleCostTime", "planning_time_cost", 'LateralMotionCostTime', 'RealTimeLateralBehaviorCostTime', 'TrajectoryGeneratorCostTime', \
                         "SccLonBehaviorCostTime", "SccLonMotionCostTime", "dynamic_world_cost"]
  data_dict = {}
  for key in time_cost_keys:
    try:
      data_dict[key] = planning_json[key]
    except:
      data_dict[key] = 'none'
      pass
  print(data_dict)
# 障碍物的id选择
class ObjText:
  def __init__(self,  obj_callback):
    self.id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_id",min=0.0, max=10000)
    ipywidgets.interact(obj_callback, id = self.id)