from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
import lib.load_global_var as global_var
import numpy as np
import re

from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label
import ipywidgets as widgets
from IPython.display import display
from ipywidgets import Button, HBox
from IPython.display import clear_output
import time
import threading
import ipywidgets
from collections import namedtuple
from functools import  partial
from bokeh.models import ColumnDataSource
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS, CheckboxGroup,TapTool
from bokeh.events import Tap
from google.protobuf.json_format import MessageToJson

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
Max_line_size = 200
Road_boundary_max_line_size = 50
Lane_boundary_max_line_size = 300
Max_sdmap_segment_size = 100

def update_local_view_data(fig1, bag_loader, bag_time, local_view_data):
  # get param
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  is_match_planning = global_var.get_value('is_match_planning')
  is_bag_main = global_var.get_value('is_bag_main')
  is_new_loc = global_var.get_value('is_new_loc')
  is_enu_to_car = global_var.get_value('is_enu_to_car')
  is_vis_map = global_var.get_value('is_vis_map')
  is_vis_sdmap = global_var.get_value('is_vis_sdmap')
  is_vis_rdg_line = global_var.get_value('is_vis_rdg_line')
  # get msg
  # bag_time = 1.2
  ### step 1: 时间戳对齐
  soc_state_msg = find_nearest(bag_loader.soc_state_msg, bag_time)
  loc_msg = find_nearest(bag_loader.loc_msg, bag_time)
  origin_loc_msg = find_nearest(bag_loader.origin_loc_msg, bag_time)
  road_msg = find_nearest(bag_loader.road_msg, bag_time)
  lane_topo_msg = find_nearest(bag_loader.lane_topo_msg, bag_time)
  fus_msg = find_nearest(bag_loader.fus_msg, bag_time)
  mobileye_objects_msg = find_nearest(bag_loader.mobileye_objects_msg, bag_time)
  rdg_objects_msg = find_nearest(bag_loader.rdg_objects_msg, bag_time)

  # radar_msg_idx = dict()
  radar_msg = ['radar_fm_msg','radar_fl_msg','radar_fr_msg','radar_rl_msg','radar_rr_msg']
  radar_msg_idx = [0,0,0,0,0]
  bag_loader_radar_msg = [bag_loader.radar_fm_msg, bag_loader.radar_fl_msg, bag_loader.radar_fr_msg,
                          bag_loader.radar_rl_msg, bag_loader.radar_rr_msg]
  # print("bag_loader.radar_fm_msg:",bag_loader.radar_fm_msg['t'][0])
  for i in range(5):
    radar_msg[i] = find_nearest(bag_loader_radar_msg[i],bag_time,False)

  vs_msg = find_nearest(bag_loader.vs_msg, bag_time)
  plan_msg = find_nearest(bag_loader.plan_msg, bag_time)
  plan_debug_msg = find_nearest(bag_loader.plan_debug_msg, bag_time)
  plan_debug_json_msg = find_nearest(bag_loader.plan_debug_msg, bag_time, True)
  prediction_msg = find_nearest(bag_loader.prediction_msg, bag_time)
  ctrl_msg = find_nearest(bag_loader.ctrl_msg, bag_time)
  # ctrl_debug_msg = find_nearest(bag_loader.ctrl_debug_msg, bag_time)
  # ctrl_debug_json_msg = find_nearest(bag_loader.ctrl_debug_msg, bag_time,True)
  ctrl_debug_msg, ctrl_debug_json_msg = [], []
  ehr_static_map_msg = find_nearest(bag_loader.ehr_static_map_msg, bag_time)
  ehr_sd_map_msg = find_nearest(bag_loader.ehr_sd_map_msg, bag_time)
  ground_line_msg = find_nearest(bag_loader.fus_ground_line_msg, bag_time)
  planning_hmi_msg = find_nearest(bag_loader.planning_hmi_msg, bag_time)
  rdg_lane_lines_msg = find_nearest(bag_loader.rdg_lane_lines_msg, bag_time)
  fus_occ_obj_msg = find_nearest(bag_loader.fus_occ_objects_msg, bag_time)
  fus_parking_msg = find_nearest(bag_loader.fus_parking_msg, bag_time)
  fus_speed_bump_msg = find_nearest(bag_loader.fus_speed_bump_msg, bag_time)
  rdg_ground_line_msg = find_nearest(bag_loader.rdg_ground_line_msg, bag_time)
  rdg_parking_slot_msg = find_nearest(bag_loader.rdg_parking_slot_msg, bag_time)
  rdg_general_objects_msg = find_nearest(bag_loader.rdg_general_objects_msg, bag_time)
  rdg_occ_objects_msg = find_nearest(bag_loader.rdg_occ_objects_msg, bag_time)
  rdg_parking_lane_line_msg = find_nearest(bag_loader.rdg_parking_lane_line_msg, bag_time)

  if bag_loader.plan_debug_msg['enable'] == True:
    input_topic_timestamp = plan_debug_msg.input_topic_timestamp
    fusion_road_timestamp = input_topic_timestamp.fusion_road
    fusion_object_timestamp = input_topic_timestamp.fusion_object
    fusion_occ_object_timestamp = input_topic_timestamp.fusion_occupancy_object
    fusion_parking_timestamp = input_topic_timestamp.parking_fusion
    fusion_speed_bump_timestamp = input_topic_timestamp.fusion_speed_bump
    fusion_ground_line_timestamp = input_topic_timestamp.ground_line
    ehr_static_map_timestamp = input_topic_timestamp.map
    soc_state_timestamp = input_topic_timestamp.function_state_machine

    if is_new_loc:
      localization_timestamp = input_topic_timestamp.localization
    else :
      if is_bag_main:
        localization_timestamp = input_topic_timestamp.localization_estimate #main分支录制的包
      else:
        localization_timestamp = input_topic_timestamp.localization # main分支之前录得包
    # prediction_timestamp = input_topic_timestamp.prediction
    # vehicle_service_timestamp = input_topic_timestamp.vehicle_service
    # control_output_timestamp = input_topic_timestamp.control_output

    if is_match_planning:
      plan_msg_tmp = find(bag_loader.plan_msg, plan_debug_msg.timestamp)
      if plan_msg_tmp != None:
        plan_msg = plan_msg_tmp
      else:
        print("match plan_msg fail")

      soc_state_msg_tmp = find(bag_loader.soc_state_msg, soc_state_timestamp)
      if soc_state_msg_tmp != None:
        soc_state_msg = soc_state_msg_tmp
      else:
        print("match soc_state fail")

      fus_msg_tmp = find(bag_loader.fus_msg, fusion_object_timestamp)
      if fus_msg_tmp != None:
        fus_msg = fus_msg_tmp
      else:
        print("match fusion_object fail")

      fus_occ_obj_msg_tmp = find(bag_loader.fus_occ_objects_msg, fusion_occ_object_timestamp)
      if fus_occ_obj_msg_tmp != None:
        fus_occ_obj_msg = fus_occ_obj_msg_tmp
      else:
        print("match fusion_occ_object fail")

      fus_parking_msg_tmp = find(bag_loader.fus_parking_msg, fusion_parking_timestamp)
      if fus_parking_msg_tmp != None:
        fus_parking_msg = fus_parking_msg_tmp
      else:
        print("match fusion_parking fail")

      fus_speed_bump_msg_tmp = find(bag_loader.fus_speed_bump_msg, fusion_speed_bump_timestamp)
      if fus_speed_bump_msg_tmp != None:
        fus_speed_bump_msg = fus_speed_bump_msg_tmp
      else:
        print("match fusion_speed_bump fail")

      fus_ground_line_msg_tmp = find(bag_loader.fus_ground_line_msg, fusion_ground_line_timestamp)
      if fus_ground_line_msg_tmp != None:
        ground_line_msg = fus_ground_line_msg_tmp
      else:
        print("match fusion_ground_line fail")

      ehr_static_map_msg_tmp = find(bag_loader.ehr_static_map_msg, ehr_static_map_timestamp)
      if ehr_static_map_msg_tmp != None:
        ehr_static_map_msg = ehr_static_map_msg_tmp
      else:
        print("match ehr_static_map fail")

      road_msg_tmp = find(bag_loader.road_msg, fusion_road_timestamp)
      if road_msg_tmp != None:
        road_msg = road_msg_tmp
        index = 0
        try:
          for i in range(16):
            if (road_msg_tmp.msg_header.input_list[i].input_type == 22):
              index = i
              break
          rdg_lane_lines_msg_tmp = findbyseq(bag_loader.rdg_lane_lines_msg, road_msg_tmp.msg_header.input_list[index].seq)
        except:
          rdg_lane_lines_msg_tmp = None
        if rdg_lane_lines_msg_tmp != None:
          rdg_lane_lines_msg = rdg_lane_lines_msg_tmp
          print('find rdg_lane_lines_msg success')
        else :
          print('find rdg_lane_lines_msg fail')
      loc_msg_tmp = find(bag_loader.loc_msg, localization_timestamp)
      if loc_msg_tmp != None:
        loc_msg = loc_msg_tmp
      else:
        print('match loc fail')

  local_view_data['data_msg']['plan_msg'] = plan_msg
  local_view_data['data_msg']['plan_debug_msg'] = plan_debug_msg
  local_view_data['data_msg']['planning_hmi_msg'] = planning_hmi_msg
  local_view_data['data_msg']['loc_msg'] = loc_msg
  local_view_data['data_msg']['plan_debug_json_msg'] = plan_debug_json_msg
  local_view_data['data_msg']['ctrl_msg'] = ctrl_msg
  local_view_data['data_msg']['ctrl_debug_msg'] = ctrl_debug_msg
  local_view_data['data_msg']['ctrl_debug_json_msg'] = ctrl_debug_json_msg
  if bag_loader.soc_state_msg['enable'] == True:
    soc_state = soc_state_msg.current_state
    print("FunctionalState: ", soc_state)

  ### step 2-1: 加载pp原始定位信息
  if bag_loader.origin_loc_msg['enable'] == True:
    cur_pos_xn = 0
    cur_pos_yn = 0
    cur_yaw = 0
    cur_pos_xn = origin_loc_msg.position.position_boot.x
    cur_pos_yn = origin_loc_msg.position.position_boot.y
    cur_yaw = origin_loc_msg.orientation.euler_boot.yaw
    coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xb, ego_yb = [], []
    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.origin_loc_msg['data'])):
      if (i % 10 != 0): # 下采样 10
        continue
      # if bag_loader.loc_msg['data'][i].msf_status.msf_status == 2 :
      #   continue
      pos_xn_i = bag_loader.origin_loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.origin_loc_msg['data'][i].position.position_boot.y
      if g_is_display_enu:
        ego_local_x, ego_local_y = pos_xn_i, pos_yn_i
      else:
        ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)
      ego_xn.append(pos_xn_i)
      ego_yn.append(pos_yn_i)

    local_view_data['origin_data_ego'].data.update({
      'ego_xb': ego_xb,
      'ego_yb': ego_yb,
      'ego_xn': ego_xn,
      'ego_yn': ego_yn,
    })

  ### step 2-2: 加载定位信息
  loc_mode = 0
  cur_pos_xn = 0
  cur_pos_yn = 0
  cur_yaw = 0
  if bag_loader.loc_msg['enable'] == True:
    loc_mode = loc_msg.status.status_info.mode
    print("localization mode: ", loc_mode)
    # ego pos in local and global coordinates
    loc_msg = loc_msg
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw
    coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xb, ego_yb = [], []
    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      if (i % 10 != 0): # 下采样 10
        continue
      # if bag_loader.loc_msg['data'][i].msf_status.msf_status == 2 :
      #   continue
      pos_xn_i = bag_loader.loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.loc_msg['data'][i].position.position_boot.y
      if g_is_display_enu:
        ego_local_x, ego_local_y = pos_xn_i, pos_yn_i
      else:
        ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)
      ego_xn.append(pos_xn_i)
      ego_yn.append(pos_yn_i)

    local_view_data['data_ego'].data.update({
      'ego_xb': ego_xb,
      'ego_yb': ego_yb,
      'ego_xn': ego_xn,
      'ego_yn': ego_yn,
    })

    if g_is_display_enu:
      # car pos in global coordinates
      car_xn = []
      car_yn = []
      for i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      local_view_data['data_car'].data.update({
        'car_xb': car_xn,
        'car_yb': car_yn,
      })

      local_view_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [cur_pos_xn],
        'ego_pos_point_y': [cur_pos_yn],
        'ego_pos_point_theta': [cur_yaw],
      })
    else:
       local_view_data['data_car'].data.update({
        'car_xb': car_xb,
        'car_yb': car_yb,
      })
       local_view_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [0],
        'ego_pos_point_y': [0],
        'ego_pos_point_theta': [0],
      })

    try:
      vel_ego = vs_msg.vehicle_speed
    except:
      linear_velocity_from_wheel = math.sqrt(loc_msg.velocity.velocity_boot.vx * loc_msg.velocity.velocity_boot.vx + \
                loc_msg.velocity.velocity_boot.vy * loc_msg.velocity.velocity_boot.vy + \
                loc_msg.velocity.velocity_boot.vz * loc_msg.velocity.velocity_boot.vz)
      vel_ego =  linear_velocity_from_wheel

    steer_deg = vs_msg.steering_wheel_angle * 57.3
    if g_is_display_enu:
      local_view_data['data_text'].data.update({
        'vel_ego_text': ['v={:.2f}\nsteer={:.2f}'.format(round(vel_ego, 2), round(steer_deg, 2))],
        'text_xn': [cur_pos_xn],
        'text_yn': [cur_pos_yn],
      })
    else:
      local_view_data['data_text'].data.update({
        'vel_ego_text': ['v={:.2f}\nsteer={:.2f}'.format(round(vel_ego, 2), round(steer_deg, 2))],
        'text_xn': [-2],
        'text_yn': [0],
      })

  if bag_loader.plan_msg['enable'] == True and  plan_msg != None:
    if plan_msg.trajectory.trajectory_type == 0: # 实时轨迹
      is_enu_to_car = False
      # global_var.set_value('is_enu_to_car', False)

  # step 3: 加载车道线信息
  if bag_loader.road_msg['enable'] == True:
    print("road local_point_valid: ", road_msg.local_point_valid)
    dash_line_x, dash_line_y, dash_line_id = [], [], []
    solid_line_x, solid_line_y, solid_line_id = [], [], []
    virtual_line_x, virtual_line_y, dot_line_id = [], [], []
    try:
      lane_line_list = load_lane_boundary_lines(road_msg, is_enu_to_car, loc_msg, g_is_display_enu)
      dash_line_x, dash_line_y, dash_line_id = lane_line_list['line_x_vec'][0], lane_line_list['line_y_vec'][0], lane_line_list['relative_id_vec'][0]
      solid_line_x, solid_line_y, solid_line_id = lane_line_list['line_x_vec'][1], lane_line_list['line_y_vec'][1], lane_line_list['relative_id_vec'][1]
      virtual_line_x, virtual_line_y, dot_line_id = lane_line_list['line_x_vec'][2], lane_line_list['line_y_vec'][2], lane_line_list['relative_id_vec'][2]

    except:
      print("vis road_msg error")
    local_view_data['data_lane_dashed_line'].data.update({
      'lines_x_vec': dash_line_x,
      'lines_y_vec': dash_line_y,
      'relative_id_vec': dash_line_id,
    })
    local_view_data['data_lane_solid_line'].data.update({
      'lines_x_vec': solid_line_x,
      'lines_y_vec': solid_line_y,
      'relative_id_vec': solid_line_id,
    })
    local_view_data['data_lane_virtual_line'].data.update({
      'lines_x_vec': virtual_line_x,
      'lines_y_vec': virtual_line_y,
      'relative_id_vec': dot_line_id,
    })
    # load lane info
    try:
      line_info_list = load_lane_lines(road_msg, is_enu_to_car, loc_msg, g_is_display_enu)
    except:
      print("vis road_msg error")

    # update lane info
    data_lane_dict = {
      0:local_view_data['data_lane_0'],
      1:local_view_data['data_lane_1'],
      2:local_view_data['data_lane_2'],
      3:local_view_data['data_lane_3'],
      4:local_view_data['data_lane_4'],
      5:local_view_data['data_lane_5'],
      6:local_view_data['data_lane_6'],
      7:local_view_data['data_lane_7'],
      8:local_view_data['data_lane_8'],
      9:local_view_data['data_lane_9'],
      10:local_view_data['data_lane_10'],
      11:local_view_data['data_lane_11'],
      12:local_view_data['data_lane_12'],
      13:local_view_data['data_lane_13'],
      14:local_view_data['data_lane_14'],
      15:local_view_data['data_lane_15'],
      16:local_view_data['data_lane_16'],
      17:local_view_data['data_lane_17'],
      18:local_view_data['data_lane_18'],
      19:local_view_data['data_lane_19'],
    }


    data_center_line_dict = {
      0:local_view_data['data_center_line_0'],
      1:local_view_data['data_center_line_1'],
      2:local_view_data['data_center_line_2'],
      3:local_view_data['data_center_line_3'],
      4:local_view_data['data_center_line_4'],
      5:local_view_data['data_center_line_5'],
      6:local_view_data['data_center_line_6'],
      7:local_view_data['data_center_line_7'],
      8:local_view_data['data_center_line_8'],
      9:local_view_data['data_center_line_9'],
    }

    data_lane_mark_dict = {
      0:local_view_data['lane_mark_data_0'],
      1:local_view_data['lane_mark_data_1'],
      2:local_view_data['lane_mark_data_2'],
      3:local_view_data['lane_mark_data_3'],
      4:local_view_data['lane_mark_data_4'],
      5:local_view_data['lane_mark_data_5'],
      6:local_view_data['lane_mark_data_6'],
      7:local_view_data['lane_mark_data_7'],
      8:local_view_data['lane_mark_data_8'],
      9:local_view_data['lane_mark_data_9'],
    }

    for i in range(20):
      try:
        if line_info_list[i]['type_vec'][0] == ['dashed']:
          fig1.renderers[0 + i].glyph.line_dash = 'dashed'
        elif line_info_list[i]['type_vec'][0] == ['dashdot']:
          fig1.renderers[0 + i].glyph.line_dash = 'dashdot'
        else:
          fig1.renderers[0 + i].glyph.line_dash = 'solid'
        data_lane = data_lane_dict[i]
        data_lane.data.update({
          'line_{}_x'.format(i): line_info_list[i]['line_x_vec'],
          'line_{}_y'.format(i): line_info_list[i]['line_y_vec'],
        })
      except:
        print('error')
        pass

    center_line_list = load_lane_center_lines(road_msg, is_enu_to_car, loc_msg, g_is_display_enu)

    # print(center_line_list)

    for i in range(10):
        data_center_line = data_center_line_dict[i]
        data_center_line.data.update({
          'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
          'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
        })
        lane_mark_data = data_lane_mark_dict[i]
        # lane_mark_loc_x = []
        # lane_mark_loc_y = []
        # lane_mark_loc_x.append((center_line_list[i]['line_x_vec'][0] + center_line_list[i]['lane_mark_point_x'][0]) / 2)
        # lane_mark_loc_y.append((center_line_list[i]['line_y_vec'][0] + center_line_list[i]['lane_mark_point_y'][0]) / 2)
        # for j in range(len(center_line_list[i]['lane_mark_point_x'])) :
          # if j < len(center_line_list[i]['lane_mark_point_x']) - 1:
          #   lane_mark_loc_x.append((center_line_list[i]['lane_mark_point_x'][j] + center_line_list[i]['lane_mark_point_x'][j+1]) / 2)
          #   lane_mark_loc_y.append((center_line_list[i]['lane_mark_point_y'][j] + center_line_list[i]['lane_mark_point_y'][j+1]) / 2)
        lane_mark_data.data.update({
          'lane_mark_{}'.format(i): center_line_list[i]['lane_mark_vec'],
          'text_xn_{}'.format(i): center_line_list[i]['lane_mark_point_x'],
          'text_yn_{}'.format(i): center_line_list[i]['lane_mark_point_y'],
          'lane_mark_loc_x_{}'.format(i): center_line_list[i]['lane_mark_loc_x'],
          'lane_mark_loc_y_{}'.format(i): center_line_list[i]['lane_mark_loc_y'],
        })




    # 加载topo车道线和中心线
    if bag_loader.lane_topo_msg['enable'] == True:
      try:
        line_topo_info_list = load_lane_topo_lines(lane_topo_msg, is_enu_to_car, loc_msg, g_is_display_enu)
        center_line_topo_list = load_lane_topo_center_lines(lane_topo_msg, is_enu_to_car, loc_msg, g_is_display_enu)
        data_lane_topo_dict = {
          0:local_view_data['data_lane_topo_0'],
          1:local_view_data['data_lane_topo_1'],
          2:local_view_data['data_lane_topo_2'],
          3:local_view_data['data_lane_topo_3'],
          4:local_view_data['data_lane_topo_4'],
          5:local_view_data['data_lane_topo_5'],
          6:local_view_data['data_lane_topo_6'],
          7:local_view_data['data_lane_topo_7'],
          8:local_view_data['data_lane_topo_8'],
          9:local_view_data['data_lane_topo_9'],
          10:local_view_data['data_lane_topo_10'],
          11:local_view_data['data_lane_topo_11'],
          12:local_view_data['data_lane_topo_12'],
          13:local_view_data['data_lane_topo_13'],
          14:local_view_data['data_lane_topo_14'],
          15:local_view_data['data_lane_topo_15'],
          16:local_view_data['data_lane_topo_16'],
          17:local_view_data['data_lane_topo_17'],
          18:local_view_data['data_lane_topo_18'],
          19:local_view_data['data_lane_topo_19'],
        }
        data_center_line_topo_dict = {
          0:local_view_data['data_center_line_topo_0'],
          1:local_view_data['data_center_line_topo_1'],
          2:local_view_data['data_center_line_topo_2'],
          3:local_view_data['data_center_line_topo_3'],
          4:local_view_data['data_center_line_topo_4'],
          5:local_view_data['data_center_line_topo_5'],
          6:local_view_data['data_center_line_topo_6'],
          7:local_view_data['data_center_line_topo_7'],
          8:local_view_data['data_center_line_topo_8'],
          9:local_view_data['data_center_line_topo_9'],
        }

        for i in range(20):
          try:
            if line_topo_info_list[i]['type'] == ['dashed']:
              fig1.renderers[21 + i].glyph.line_dash = 'dashed'
            else:
              fig1.renderers[21 + i].glyph.line_dash = 'solid'
            data_lane_topo = data_lane_topo_dict[i]
            data_lane_topo.data.update({
            'line_topo_{}_x'.format(i): line_topo_info_list[i]['line_x_topo'],
            'line_topo_{}_y'.format(i): line_topo_info_list[i]['line_y_topo'],
            })
          except:
            print('error')
            pass

        for i in range(10):
          data_center_line_topo = data_center_line_topo_dict[i]
          data_center_line_topo.data.update({
            'center_line_topo_{}_x'.format(i): center_line_topo_list[i]['center_line_x_topo'],
            'center_line_topo_{}_y'.format(i): center_line_topo_list[i]['center_line_y_topo'],
          })

      except:
        print("vis topo_msg error")



    # 加载rdg车道线
    if is_vis_rdg_line and bag_loader.rdg_lane_lines_msg['enable'] == True:
      rdg_lane_lines = load_rdg_lane_lines(rdg_lane_lines_msg, is_enu_to_car, loc_msg, g_is_display_enu)
      data_lane_dict2 = {
        0:local_view_data['rdg_data_lane_0'],
        1:local_view_data['rdg_data_lane_1'],
        2:local_view_data['rdg_data_lane_2'],
        3:local_view_data['rdg_data_lane_3'],
        4:local_view_data['rdg_data_lane_4'],
        5:local_view_data['rdg_data_lane_5'],
        6:local_view_data['rdg_data_lane_6'],
        7:local_view_data['rdg_data_lane_7'],
        8:local_view_data['rdg_data_lane_8'],
        9:local_view_data['rdg_data_lane_9'],
        10:local_view_data['rdg_data_lane_10'],
        11:local_view_data['rdg_data_lane_11'],
        12:local_view_data['rdg_data_lane_12'],
        13:local_view_data['rdg_data_lane_13'],
        14:local_view_data['rdg_data_lane_14'],
        15:local_view_data['rdg_data_lane_15'],
        16:local_view_data['rdg_data_lane_16'],
        17:local_view_data['rdg_data_lane_17'],
        18:local_view_data['rdg_data_lane_18'],
        19:local_view_data['rdg_data_lane_19'],
      }

      for i in range(20):
        try:
          fig1.renderers[41 + i].glyph.line_dash = 'dashed'
          fig1.renderers[41 + i].glyph.line_color = 'green'
          if rdg_lane_lines[i]['type'] == ['dashed']:
            fig1.renderers[41 + i].glyph.line_dash = 'dashed'
          elif rdg_lane_lines[i]['type'] == ['solid']:
            fig1.renderers[41 + i].glyph.line_dash = 'solid'
          elif rdg_lane_lines[i]['type'] == ['curb']:
            fig1.renderers[41 + i].glyph.line_color = 'red'
          data_lane = data_lane_dict2[i]
          data_lane.data.update({
            'rdg_line_{}_x'.format(i): rdg_lane_lines[i]['line_x_vec'],
            'rdg_line_{}_y'.format(i): rdg_lane_lines[i]['line_y_vec'],
          })
        except:
          print('error1')
          pass

      #加载停止线
      stop_lines = load_stop_lines(rdg_lane_lines_msg, is_enu_to_car, loc_msg, g_is_display_enu)
      data_stop_line_dict = {
        0:local_view_data['stop_line_0'],
        1:local_view_data['stop_line_1'],
        2:local_view_data['stop_line_2'],
        3:local_view_data['stop_line_3'],
        4:local_view_data['stop_line_4']
      }
      for i in range(5):
        try:
          data_stop_line = data_stop_line_dict[i]
          data_stop_line.data.update({
            'stop_line_{}_x'.format(i): stop_lines[i]['stop_line_x'],
            'stop_line_{}_y'.format(i): stop_lines[i]['stop_line_y'],
          })
        except:
          pass

      #加载斑马线人行道
      zebra_crossing_lines = load_zebra_crossing_lines(rdg_lane_lines_msg, is_enu_to_car, loc_msg, g_is_display_enu)
      zebra_crossing_line_dict = {
        0:local_view_data['zebra_crossing_line_0'],
        1:local_view_data['zebra_crossing_line_1'],
        2:local_view_data['zebra_crossing_line_2'],
        3:local_view_data['zebra_crossing_line_3'],
        4:local_view_data['zebra_crossing_line_4'],
        5:local_view_data['zebra_crossing_line_5'],
        6:local_view_data['zebra_crossing_line_6'],
        7:local_view_data['zebra_crossing_line_7'],
        8:local_view_data['zebra_crossing_line_8'],
        9:local_view_data['zebra_crossing_line_9'],
        10:local_view_data['zebra_crossing_line_10'],
        11:local_view_data['zebra_crossing_line_11']
      }
      for i in range(12):
        try:
          data_zebra_crossing_lines = zebra_crossing_line_dict[i]
          data_zebra_crossing_lines.data.update({
            'zebra_crossing_line_{}_x'.format(i): zebra_crossing_lines[i]['zebra_crossing_line_x'],
            'zebra_crossing_line_{}_y'.format(i): zebra_crossing_lines[i]['zebra_crossing_line_y'],
          })
        except:
          pass



    #加载planning 生成中心线的信息
    try:
      plan_gen_refline_list = list(plan_debug_msg.generated_refline_info)
      if(len(plan_gen_refline_list) > 0):
        plan_gen_refline = plan_gen_refline_list[0]
        line_x, line_y = load_intersection_generated_refline(plan_gen_refline, is_enu_to_car, loc_msg)
          #line_x.append(virtual_lane_refline_point.car_point.x)
          #line_y.append(virtual_lane_refline_point.car_point.y)
        local_view_data['data_center_line_gen'].data.update({
              'center_line_gen_x': line_x,
              'center_line_gen_y': line_y,
        })
      else:
        local_view_data['data_center_line_gen'].data.update({
              'center_line_gen_x': [],
              'center_line_gen_y': [],
        })
    except:
      pass
  # fix_lane,origin_lane
  planning_succ = False
  if bag_loader.plan_debug_msg['enable'] == True:
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw
    try:
      scene_type = plan_debug_msg.frame_info.scene_type
      print("scene_type: ", scene_type)
      planning_succ = plan_debug_msg.frame_info.planning_succ
      print("planning_succ: ", planning_succ)
      print("current planning_success: ", plan_debug_json_msg['current planning_success'])
      intersection_state = plan_debug_msg.real_time_lon_behavior_planning_input.intersection_state
      print("intersection_state: ", intersection_state)
    except:
      print("no intersection_state")

    try:
      print("planning debug info:", int(plan_debug_msg.frame_info.frame_num) - bag_loader.plan_debug_msg['data'][0].frame_info.frame_num)
    except:
      pass
    print("distance_to_target_slot: ", plan_debug_json_msg['distance_to_target_slot'])
    lat_behavior_common = plan_debug_msg.lat_behavior_common
    environment_model_info = plan_debug_msg.environment_model_info
    current_lane_virtual_id = environment_model_info.currrent_lane_vitual_id
    fix_lane_ralative_id = lat_behavior_common.fix_lane_virtual_id - current_lane_virtual_id
    target_lane_ralative_id = lat_behavior_common.target_lane_virtual_id - current_lane_virtual_id
    origin_lane_ralative_id = lat_behavior_common.origin_lane_virtual_id - current_lane_virtual_id
    if bag_loader.road_msg['enable'] == True:
      for i in range(len(center_line_list)):
        if center_line_list[i]['relative_id'] == fix_lane_ralative_id:
          local_view_data['data_fix_lane'].data.update({
            'fix_lane_x': center_line_list[i]['line_x_vec'],
            'fix_lane_y':center_line_list[i]['line_y_vec']
          })
        if center_line_list[i]['relative_id'] == target_lane_ralative_id:
          local_view_data['data_target_lane'].data.update({
            'target_lane_x': center_line_list[i]['line_x_vec'],
            'target_lane_y':center_line_list[i]['line_y_vec']
          })
        if center_line_list[i]['relative_id'] == origin_lane_ralative_id:
          local_view_data['data_origin_lane'].data.update({
            'origin_lane_x': center_line_list[i]['line_x_vec'],
            'origin_lane_y':center_line_list[i]['line_y_vec']
          })

    if loc_mode > 0:
      plan_debug_json = plan_debug_json_msg
      plan_traj_x = plan_debug_json["assembled_x"]
      plan_traj_y = plan_debug_json["assembled_y"]
      plan_traj_theta = plan_debug_json["assembled_theta"]
      plan_traj_s = plan_debug_json["traj_s_vec"]
      local_view_data['data_planning_raw'].data.update({
        'plan_traj_y' : [],
        'plan_traj_x' : [],
        'plan_traj_s': []
      })
      local_view_data['data_car_traj_raw'].data.update({
        'car_yb_traj' : [],
        'car_xb_traj' : [],
      })
      if (len(plan_traj_x) > 0):
        if min(plan_traj_x) != max(plan_traj_x) or min(plan_traj_y) != max(plan_traj_y):
          if not g_is_display_enu:
            plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_traj_x, plan_traj_y)
            cur_yaw = loc_msg.orientation.euler_boot.yaw
            plan_traj_theta_local = []
            for i in range(len(plan_traj_theta)):
              plan_traj_theta_local.append(plan_traj_theta[i] - cur_yaw)
            plan_traj_theta = plan_traj_theta_local

          local_view_data['data_planning_raw'].data.update({
            'plan_traj_y' : plan_traj_y,
            'plan_traj_x' : plan_traj_x,
            'plan_traj_s': plan_traj_s
          })

          if len(plan_traj_x) == len(plan_traj_y) and len(plan_traj_x) == len(plan_traj_theta):
            car_xb_traj = []
            car_yb_traj = []
            for i in range(len(plan_traj_x) - 1, -1, -1):
              car_xb_traj_point = []
              car_yb_traj_point = []
              for j in range(len(car_xb)):
                tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_traj_x[i], plan_traj_y[i], plan_traj_theta[i])
                car_xb_traj_point.append(tmp_x)
                car_yb_traj_point.append(tmp_y)
              car_xb_traj.append(car_xb_traj_point)
              car_yb_traj.append(car_yb_traj_point)
            local_view_data['data_car_traj_raw'].data.update({
              'car_yb_traj' : car_yb_traj,
              'car_xb_traj' : car_xb_traj,
            })
          else:
            print("car_traj_raw error!")
            print("raw_plan_traj_x size: ", len(plan_traj_x))
            print("raw_plan_traj_y size: ", len(plan_traj_y))
            print("raw_plan_traj_theta size: ", len(plan_traj_theta))
      else:
        print("no car_traj_raw json debug info!")

      lat_init_state = plan_debug_msg.lateral_motion_planning_input.init_state
      init_state_x = lat_init_state.x
      init_state_y = lat_init_state.y
      init_state_theta = lat_init_state.theta
      init_state_delta = lat_init_state.delta
      ego_pos_compensation_x = plan_debug_json_msg['predicted_ego_x']
      ego_pos_compensation_y = plan_debug_json_msg['predicted_ego_y']
      merge_point_x = plan_debug_json_msg['merge_point_x']
      merge_point_y = plan_debug_json_msg['merge_point_y']
      lon_collision_object_position_x_vec = plan_debug_json_msg['lon_collision_object_position_x_vec']
      lon_collision_object_position_y_vec = plan_debug_json_msg['lon_collision_object_position_y_vec']
      macroeconomic_decider_merge_point_x = plan_debug_json_msg['macroeconomic_decider_merge_point_x']
      macroeconomic_decider_merge_point_y = plan_debug_json_msg['macroeconomic_decider_merge_point_y']
      boundary_line_merge_point_x = plan_debug_json_msg['boundary_line_merge_point_x']
      boundary_line_merge_point_y = plan_debug_json_msg['boundary_line_merge_point_y']
      lon_init_state = plan_debug_msg.longitudinal_motion_planning_input.init_state
      init_state_s = lon_init_state.s
      init_state_v = lon_init_state.v
      init_state_a = lon_init_state.a
      replan_status = plan_debug_json_msg["replan_status"]
      init_pos_point_x = []
      init_pos_point_y = []
      init_pos_line_x = []
      init_pos_line_y = []
      ego_pos_compensation_x_ = []
      ego_pos_compensation_y_ = []
      init_pos_point_theta = []
      lon_collision_object_position_x_vec_ = []
      lon_collision_object_position_y_vec_ = []
      if g_is_display_enu:
        init_pos_point_x.append(init_state_x)
        init_pos_point_y.append(init_state_y)
        init_pos_point_theta.append(init_state_theta)
        ego_pos_compensation_x_.append(ego_pos_compensation_x)
        ego_pos_compensation_y_.append(ego_pos_compensation_y)
        lon_collision_object_position_x_vec_ = lon_collision_object_position_x_vec
        lon_collision_object_position_y_vec_ = lon_collision_object_position_y_vec
      else:
        init_pos_point_x, init_pos_point_y = coord_tf.global_to_local([init_state_x], [init_state_y])
        ego_pos_compensation_x_, ego_pos_compensation_y_ = coord_tf.global_to_local([ego_pos_compensation_x], [ego_pos_compensation_y])
        lon_collision_object_position_x_vec_, lon_collision_object_position_y_vec_ = coord_tf.global_to_local(lon_collision_object_position_x_vec, lon_collision_object_position_y_vec)
        temp_theta = init_state_theta - loc_msg.orientation.euler_boot.yaw
        init_pos_point_theta.append(temp_theta)

      print("lon_collision_object_position_x_vec: ", lon_collision_object_position_x_vec_)
      print("lon_collision_object_position_y_vec: ", lon_collision_object_position_y_vec_)

      for i in range(len(bag_loader.plan_debug_msg['data'])):
        init_pos_xn_i = bag_loader.plan_debug_msg['data'][i].lateral_motion_planning_input.init_state.x
        init_pos_yn_i = bag_loader.plan_debug_msg['data'][i].lateral_motion_planning_input.init_state.y

        if g_is_display_enu:
          local_init_x = init_pos_xn_i
          local_init_y = init_pos_yn_i
        else:
         local_init_x, local_init_y = global2local(init_pos_xn_i, init_pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

        init_pos_line_x.append(local_init_x)
        init_pos_line_y.append(local_init_y)

      local_view_data['data_init_line'].data.update({
        'init_pos_line_x': init_pos_line_x,
        'init_pos_line_y': init_pos_line_y,
       })

      local_view_data['data_init_pos_point'].data.update({
        'init_pos_point_y': init_pos_point_y,
        'init_pos_point_x': init_pos_point_x,
        'init_pos_point_theta': init_pos_point_theta,
        'init_state_x': [init_state_x],
        'init_state_y': [init_state_y],
        'init_state_theta': [init_state_theta],
        'init_state_delta': [init_state_delta],
        'init_state_s': [init_state_s],
        'init_state_v': [init_state_v],
        'init_state_a': [init_state_a],
        'replan_status': [replan_status],
        'ego_pos_compensation_x': ego_pos_compensation_x_,
        'ego_pos_compensation_y': ego_pos_compensation_y_
      })
      local_view_data['data_merge_point'].data.update({
        'merge_point_x': [merge_point_x],
        'merge_point_y': [merge_point_y]})
      local_view_data["data_lon_collision_object_position"].data.update({
        'lon_collision_object_position_x': lon_collision_object_position_x_vec_,
        'lon_collision_object_position_y': lon_collision_object_position_y_vec_,
      })
      macroeconomic_decider_merge_point_x, macroeconomic_decider_merge_point_y = coord_tf.global_to_local([macroeconomic_decider_merge_point_x], [macroeconomic_decider_merge_point_y])
      macroeconomic_decider_merge_point_x = macroeconomic_decider_merge_point_x[0]
      macroeconomic_decider_merge_point_y = macroeconomic_decider_merge_point_y[0]
      local_view_data['macroeconomic_decider_data_merge_point'].data.update({
        'macroeconomic_decider_merge_point_x': [macroeconomic_decider_merge_point_x],
        'macroeconomic_decider_merge_point_y': [macroeconomic_decider_merge_point_y],
      })
      boundary_line_merge_point_x, boundary_line_merge_point_y = coord_tf.global_to_local([boundary_line_merge_point_x], [boundary_line_merge_point_y])
      boundary_line_merge_point_x = boundary_line_merge_point_x[0]
      boundary_line_merge_point_y = boundary_line_merge_point_y[0]
      local_view_data['boundary_line_merge_point'].data.update({
        'boundary_line_merge_point_x': [boundary_line_merge_point_x],
        'boundary_line_merge_point_y': [boundary_line_merge_point_y],
      })

      lat_motion_planning_output = plan_debug_msg.lateral_motion_planning_output
      lat_plan_traj_x = []
      lat_plan_traj_y = []
      lat_plan_traj_theta = []
      for i in range(len(lat_motion_planning_output.x_vec)):
        lat_plan_traj_x.append(lat_motion_planning_output.x_vec[i])
        lat_plan_traj_y.append(lat_motion_planning_output.y_vec[i])
        lat_plan_traj_theta.append(lat_motion_planning_output.theta_vec[i])
      if not g_is_display_enu:
          lat_plan_traj_x, lat_plan_traj_y = coord_tf.global_to_local(lat_plan_traj_x, lat_plan_traj_y)
          cur_yaw = loc_msg.orientation.euler_boot.yaw
          lat_plan_traj_theta_local = []
          for i in range(len(lat_plan_traj_theta)):
            lat_plan_traj_theta_local.append(lat_plan_traj_theta[i] - cur_yaw)
          lat_plan_traj_theta = lat_plan_traj_theta_local

      local_view_data['data_planning_lat'].data.update({
        'plan_traj_y' : lat_plan_traj_y,
        'plan_traj_x' : lat_plan_traj_x,
      })

      if len(lat_plan_traj_x) == len(lat_plan_traj_y) and len(lat_plan_traj_x) == len(lat_plan_traj_theta):
        lat_car_xb_traj = []
        lat_car_yb_traj = []
        for i in range(len(lat_plan_traj_x) - 1, -1, -1):
          car_xb_traj_point = []
          car_yb_traj_point = []
          for j in range(len(car_xb)):
            tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], lat_plan_traj_x[i], lat_plan_traj_y[i], lat_plan_traj_theta[i])
            car_xb_traj_point.append(tmp_x)
            car_yb_traj_point.append(tmp_y)
          lat_car_xb_traj.append(car_xb_traj_point)
          lat_car_yb_traj.append(car_yb_traj_point)
        local_view_data['data_car_traj_lat'].data.update({
          'car_yb_traj' : lat_car_yb_traj,
          'car_xb_traj' : lat_car_xb_traj,
        })
      else:
        print("car_traj_lat error!")
        print("lat_plan_traj_x size: ", len(lat_plan_traj_x))
        print("lat_plan_traj_y size: ", len(lat_plan_traj_y))
        print("lat_plan_traj_theta size: ", len(lat_plan_traj_theta))
    # local_view_data['data_text'].data.update({
    #   'vel_ego_text': ['v={:.2f}({:d})\nsteer={:.2}'.format(round(vel_ego, 2),current_lane_virtual_id, round(steer_deg, 2))],
    #   'text_xn': [text_xn],
    #   'text_yn': [text_yn],
    # })
  ### step 4: 加载障碍物信息
  # load fus_obj
  if bag_loader.fus_msg['enable'] == True:
    if bag_loader.plan_debug_msg['enable'] == False:
      environment_model_info = None
    obstacles_info_all = load_obstacle_params(fus_msg, is_enu_to_car, loc_msg, environment_model_info)
    local_view_data['data_fus_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'polygon_x' : [],
            'polygon_y' : [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_id' : [],
            'obs_label' : [],
          })
    local_view_data['data_snrd_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'polygon_x' : [],
            'polygon_y' : [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_id' : [],
            'obs_label' : [],
          })
    if 1:
      for key in obstacles_info_all:
        obstacles_info = obstacles_info_all[key]
        if key == 1:
          key_name = 'data_fus_obj'
        else:
          key_name = 'data_snrd_obj'
        if g_is_display_enu:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'polygon_x' : obstacles_info['polygon_x'],
            'polygon_y' : obstacles_info['polygon_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_id' : obstacles_info['obstacles_id'],
            'obs_label' : obstacles_info['obs_label'],
          })
        else:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x_rel'],
            'obstacles_y': obstacles_info['obstacles_y_rel'],
            'polygon_x' : obstacles_info['polygon_x_rel'],
            'polygon_y' : obstacles_info['polygon_y_rel'],
            'pos_x' : obstacles_info['pos_x_rel'],
            'pos_y' : obstacles_info['pos_y_rel'],
            'obs_id' : obstacles_info['obstacles_id'],
            'obs_label' : obstacles_info['obs_label'],
          })

  # load fus_occ_obj
  if bag_loader.fus_occ_objects_msg['enable'] == True:
    if bag_loader.plan_debug_msg['enable'] == False:
      environment_model_info = None
    obstacles_info_all = load_occupancy_obstacle(fus_occ_obj_msg, loc_msg, environment_model_info)
    # print("fusion occ objects size:", len(obstacles_info_all))
    for key in obstacles_info_all:
      obstacles_info = obstacles_info_all[0]
      if g_is_display_enu:
        local_view_data['data_fus_occ_obj'].data.update({
          'obstacles_x': obstacles_info['obstacles_x'],
          'obstacles_y': obstacles_info['obstacles_y'],
          'polygon_x': obstacles_info['polygon_x'],
          'polygon_y': obstacles_info['polygon_y'],
          'pos_x' : obstacles_info['pos_x'],
          'pos_y' : obstacles_info['pos_y'],
          'obs_id' : obstacles_info['obstacles_id'],
          'obs_label' : obstacles_info['obs_label'],
        })
      else:
        local_view_data['data_fus_occ_obj'].data.update({
          'obstacles_x': obstacles_info['obstacles_x_rel'],
          'obstacles_y': obstacles_info['obstacles_y_rel'],
          'polygon_x': obstacles_info['polygon_x_rel'],
          'polygon_y': obstacles_info['polygon_y_rel'],
          'pos_x' : obstacles_info['pos_x_rel'],
          'pos_y' : obstacles_info['pos_y_rel'],
          'obs_id' : obstacles_info['obstacles_id'],
          'obs_label' : obstacles_info['obs_label'],
        })
      break

  # load mobileye_obj
  if bag_loader.mobileye_objects_msg['enable'] == True:

    obstacles_info_all1 = load_obstacle_me(mobileye_objects_msg,False)
    local_view_data['data_me_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })

    # 加载自车坐标系下的数据
    if 1:
      for key in obstacles_info_all1:
        obstacles_info = obstacles_info_all1[key]
        key_name = 'data_me_obj'

        if g_is_display_enu:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })
        else:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x_rel'],
            'obstacles_y': obstacles_info['obstacles_y_rel'],
            'pos_x' : obstacles_info['pos_x_rel'],
            'pos_y' : obstacles_info['pos_y_rel'],
            'obs_label' : obstacles_info['obs_label'],
          })

  # load rdg_obj
  if bag_loader.rdg_objects_msg['enable'] == True:

    obstacles_info_all1 = load_obstacle_me(rdg_objects_msg,True)
    # 加载自车坐标系下的数据
    local_view_data['data_rdg_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })
    for key in obstacles_info_all1:
      obstacles_info = obstacles_info_all1[key]
      key_name = 'data_rdg_obj'
      if g_is_display_enu:
        local_view_data[key_name].data.update({
          'obstacles_x': obstacles_info['obstacles_x'],
          'obstacles_y': obstacles_info['obstacles_y'],
          'pos_x' : obstacles_info['pos_x'],
          'pos_y' : obstacles_info['pos_y'],
          'obs_label' : obstacles_info['obs_label'],
        })
      else:
        local_view_data[key_name].data.update({
          'obstacles_x': obstacles_info['obstacles_x_rel'],
          'obstacles_y': obstacles_info['obstacles_y_rel'],
          'pos_x' : obstacles_info['pos_x_rel'],
          'pos_y' : obstacles_info['pos_y_rel'],
          'obs_label' : obstacles_info['obs_label'],
        })

  # load radar_obj
  data_radar_obj = ['data_radar_fm_obj','data_radar_fl_obj','data_radar_fr_obj','data_radar_rl_obj','data_radar_rr_obj']
  for i in range(5):
    # print(data_radar_obj[i])
    if bag_loader_radar_msg[i]['enable'] == True:
      # print(radar_msg_idx[i])
      radar_objects = radar_msg[i].object_list
      num = radar_msg[i].object_list_size
      # print(me_camera_objects.length())
      obstacles_info_all2 = load_obstacle_radar(radar_objects,i,num)

      for key in obstacles_info_all2:
        obstacles_info = obstacles_info_all2[key]
        if key == 11 or key == 12 or key == 13 or key == 14 or key == 15 :
          key_name = data_radar_obj[i]
        else:
          continue
        if g_is_display_enu:
          local_view_data[key_name].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })
        else:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x_rel'],
            'obstacles_y': obstacles_info['obstacles_y_rel'],
            'pos_x' : obstacles_info['pos_x_rel'],
            'pos_y' : obstacles_info['pos_y_rel'],
            'obs_label' : obstacles_info['obs_label'],
          })

  #  加载fix_lane, target_lane信息
  ### step 3: 加载planning轨迹信息
  successful_slot_info_list = []
  # if bag_loader.plan_msg['enable'] == True and loc_mode == 2:
  if bag_loader.plan_msg['enable'] == True and loc_mode > 0:
    trajectory = plan_msg.trajectory
    print("trajectory_type: ", trajectory.trajectory_type)
    hpp_planning_status = plan_msg.planning_status.hpp_planning_status
    print("hpp_planning_status: ", hpp_planning_status)
    successful_slot_info_list = plan_msg.successful_slot_info_list
    plan_traj_s = []
    for i in range(len(trajectory.trajectory_points)):
      plan_traj_s.append(trajectory.trajectory_points[i].distance)
    # if trajectory.trajectory_type == 0 and planning_succ: # 实时轨迹
    #   try:
    #     planning_polynomial = trajectory.target_reference.polynomial
    #     plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
    #     plan_traj_s = [0] * len(plan_traj_x)
    #   except:
    #     plan_traj_x, plan_traj_y, plan_traj_s = [], [], []
    # else:
    plan_traj_x, plan_traj_y, plan_traj_theta, plan_dict = generate_planning_trajectory(trajectory, loc_msg, g_is_display_enu)
    for i in range(5):
      local_view_data['data_planning_' + str(i)].data.update({
        'plan_traj_y' : plan_dict[i]['y'],
        'plan_traj_x' : plan_dict[i]['x'],
    })

    local_view_data['data_planning'].data.update({
        'plan_traj_y' : plan_traj_y,
        'plan_traj_x' : plan_traj_x,
        'plan_traj_s' : plan_traj_s,
    })

    if len(plan_traj_x) == len(plan_traj_y) and len(plan_traj_x) == len(plan_traj_theta):
      car_xb_traj = []
      car_yb_traj = []
      for i in range(len(plan_traj_x) - 1, -1, -1):
        car_xb_traj_point = []
        car_yb_traj_point = []
        for j in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_traj_x[i], plan_traj_y[i], plan_traj_theta[i])
          car_xb_traj_point.append(tmp_x)
          car_yb_traj_point.append(tmp_y)
        car_xb_traj.append(car_xb_traj_point)
        car_yb_traj.append(car_yb_traj_point)
      local_view_data['data_car_traj'].data.update({
        'car_yb_traj' : car_yb_traj,
        'car_xb_traj' : car_xb_traj,
      })
    else:
      print("car_traj error!")
      print("plan_traj_x size: ", len(plan_traj_x))
      print("plan_traj_y size: ", len(plan_traj_y))
      print("plan_traj_theta size: ", len(plan_traj_theta))

  # 加载prediction_msg
  if bag_loader.prediction_msg['enable'] == True:
    prediction_obs_id = local_view_data['data_select_obs_id'].data['prediction_obstacle_id']
    try:
      for i in range(5):
        local_view_data['data_prediction_' + str(i)].data.update({
          'prediction_y' : [],
          'prediction_x' : [],
          'prediction_obs_y' : [],
          'prediction_obs_x' : [],
        })
      # 定位的选择需要修改
      prediction_dict = load_prediction_objects(prediction_msg.prediction_obstacle_list, prediction_obs_id, loc_msg, g_is_display_enu)
      for i in range(5):
        local_view_data['data_prediction_' + str(i)].data.update({
          'prediction_y' : prediction_dict[i]['y'],
          'prediction_x' : prediction_dict[i]['x'],
          'prediction_obs_y' : prediction_dict[i]['obs_y'],
          'prediction_obs_x' : prediction_dict[i]['obs_x'],
        })
    except Exception as error:
      print("prediction error: ", error)
      pass

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    try:
      mpc_dx, mpc_dy, mpc_dtheta = generate_control(ctrl_msg, loc_msg, g_is_display_enu)
    except:
      mpc_dx, mpc_dy, mpc_dtheta = [], [], []
    local_view_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })
    if len(mpc_dx) == len(mpc_dy) and len(mpc_dx) == len(mpc_dtheta):
      car_xb_traj = []
      car_yb_traj = []
      for i in range(len(mpc_dx) - 1, -1, -1):
        car_xb_traj_point = []
        car_yb_traj_point = []
        for j in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], mpc_dx[i], mpc_dy[i], mpc_dtheta[i])
          car_xb_traj_point.append(tmp_x)
          car_yb_traj_point.append(tmp_y)
        car_xb_traj.append(car_xb_traj_point)
        car_yb_traj.append(car_yb_traj_point)
      local_view_data['data_car_traj_mpc'].data.update({
        'car_yb_traj' : car_yb_traj,
        'car_xb_traj' : car_xb_traj,
      })
    else:
      print("car_traj_mpc error!")
      print("mpc_dx size: ", len(mpc_dx))
      print("mpc_dy size: ", len(mpc_dy))
      print("mpc_dtheta size: ", len(mpc_dtheta))

  # # 加载ehr的lane信息
  if is_vis_map and bag_loader.ehr_static_map_msg['enable'] == True :
    #load ehr static map info
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw

    # if bag_loader.planning_hmi_msg['enable'] ==True:
    #   noa_output_info_msg = planning_hmi_msg.noa_output_info
    #   print("dis to ramp:",noa_output_info_msg.dis_to_ramp)
    #   print("dis to split:",noa_output_info_msg.dis_to_split)
    #   print("dis to merge:",noa_output_info_msg.dis_to_merge)

    print("ehr static map timestamp:",ehr_static_map_msg.header)
    print("road_map.lanes len:",len(ehr_static_map_msg.road_map.lanes))
    #load center line

    ehr_line_info_list = ehr_load_center_lane_lines(ehr_static_map_msg.road_map.lanes,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Max_line_size, g_is_display_enu)
    ehr_data_lane_dict = {}
    for i in range(Max_line_size):
      ehr_data_lane_dict[i] = local_view_data['ehr_data_lane_{}'.format(i)]
    for i in range(len(ehr_line_info_list)):
      if ehr_line_info_list[i]['ehr_relative_id'] == 1000: #车道不存在
        ehr_line_info_list[i]['ehr_line_x_vec'] = []
        ehr_line_info_list[i]['ehr_line_y_vec'] = []
      ehr_data_line = ehr_data_lane_dict[i]
      # print("ehr_line_info_list:",len(ehr_line_info_list))
      ehr_data_line.data.update({
            'ehr_line_{}_x'.format(i): ehr_line_info_list[i]['ehr_line_x_vec'],
            'ehr_line_{}_y'.format(i): ehr_line_info_list[i]['ehr_line_y_vec'],
          })

    #load road boundary
    print("road_map.road_boundaries len:",len(ehr_static_map_msg.road_map.road_boundaries))
    ehr_load_road_boundary_info_list = ehr_load_road_boundary_lines(ehr_static_map_msg.road_map.road_boundaries,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Road_boundary_max_line_size, g_is_display_enu)
    ehr_data_road_boundary_dict = {}
    for i in range(Road_boundary_max_line_size):
      ehr_data_road_boundary_dict[i] = local_view_data['ehr_road_boundary_{}'.format(i)]
    for i in range(len(ehr_load_road_boundary_info_list)):
      ehr_data_road_boundary = ehr_data_road_boundary_dict[i]
      # print("ehr_line_info_list:",len(ehr_line_info_list))
      ehr_data_road_boundary.data.update({
            'ehr_road_boundary_{}_x'.format(i): ehr_load_road_boundary_info_list[i]['ehr_road_boundary_x_vec'],
            'ehr_road_boundary_{}_y'.format(i): ehr_load_road_boundary_info_list[i]['ehr_road_boundary_y_vec'],
          })
    #load lane boundary
    print("road_map.lane_boundaries len:",len(ehr_static_map_msg.road_map.lane_boundaries))
    ehr_lane_boundary_info_list = ehr_load_lane_boundary_lines(ehr_static_map_msg.road_map.lane_boundaries,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Lane_boundary_max_line_size, g_is_display_enu)
    ehr_data_lane_boundary_dict = {}
    for i in range(Lane_boundary_max_line_size):
      ehr_data_lane_boundary_dict[i] = local_view_data['ehr_lane_boundary_{}'.format(i)]
    for i in range(len(ehr_lane_boundary_info_list)):
      ehr_data_lane_boundary = ehr_data_lane_boundary_dict[i]
      ehr_data_lane_boundary.data.update({
            'ehr_lane_boundary_{}_x'.format(i): ehr_lane_boundary_info_list[i]['ehr_lane_boundary_x_vec'],
            'ehr_lane_boundary_{}_y'.format(i): ehr_lane_boundary_info_list[i]['ehr_lane_boundary_y_vec'],
          })

  # 加载sdmap info
  if is_vis_sdmap and bag_loader.ehr_sd_map_msg['enable'] == True:
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw
    print("road_map.lanes len:",len(ehr_sd_map_msg.route_map.segms))
    sdmap_road_line_info = load_sd_map_segments(ehr_sd_map_msg.route_map.segms,
                                          cur_pos_xn,cur_pos_yn,cur_yaw,Max_sdmap_segment_size)
    local_view_data['data_sdmap_road_line'].data.update({
      'data_sdmap_road_line_x': sdmap_road_line_info['sdmap_road_line_x_vec'],
      'data_sdmap_road_line_y': sdmap_road_line_info['sdmap_road_line_y_vec']
    })
    local_view_data['data_sdmap_ramp_line'].data.update({
      'data_sdmap_ramp_line_x': sdmap_road_line_info['sdmap_ramp_line_x_vec'],
      'data_sdmap_ramp_line_y': sdmap_road_line_info['sdmap_ramp_line_y_vec']
    })
    local_view_data['data_sdmap_inlink'].data.update({
      'data_sdmap_inlink_x': sdmap_road_line_info['inlinek_x_vec'],
      'data_sdmap_inlink_y': sdmap_road_line_info['inlinek_y_vec']
    })
    local_view_data['data_sdmap_outlink'].data.update({
      'data_sdmap_outlink_x': sdmap_road_line_info['outlinek_x_vec'],
      'data_sdmap_outlink_y': sdmap_road_line_info['outlinek_y_vec']
    })

  # 加载ehr_static_map
  if bag_loader.ehr_static_map_msg['enable'] == True:
    if bag_loader.plan_debug_msg['enable'] == False:
      environment_model_info = None
    parking_space_boxes_x, parking_space_boxes_y, \
    road_mark_boxes_x, road_mark_boxes_y, \
    road_obstacle_x_vec, road_obstacle_y_vec, \
    polygon_obstacle_x_vec, polygon_obstacle_y_vec, polygon_obstacle_label_vec, \
    polygon_x_vec, polygon_y_vec, polygon_id_vec = generate_ehr_static_map(ehr_static_map_msg, loc_msg, environment_model_info, g_is_display_enu)
    local_view_data['data_parking_space'].data.update({
      'parking_space_x' : parking_space_boxes_x,
      'parking_space_y' : parking_space_boxes_y,
    })
    local_view_data['data_road_mark'].data.update({
      'road_mark_x' : road_mark_boxes_x,
      'road_mark_y' : road_mark_boxes_y,
    })
    local_view_data['data_road_obstacle'].data.update({
      'road_obstacle_x' : road_obstacle_x_vec,
      'road_obstacle_y' : road_obstacle_y_vec,
    })
    polygon_obstacle_center_x = []
    for polygon_obstacle_x in polygon_obstacle_x_vec:
      polygon_obstacle_center_x.append((np.array(polygon_obstacle_x[0]) + np.array(polygon_obstacle_x[1]) + np.array(polygon_obstacle_x[2]) + np.array(polygon_obstacle_x[3])) / 4.0)
    polygon_obstacle_center_y = []
    for polygon_obstacle_y in polygon_obstacle_y_vec:
      polygon_obstacle_center_y.append((np.array(polygon_obstacle_y[0]) + np.array(polygon_obstacle_y[1]) + np.array(polygon_obstacle_y[2]) + np.array(polygon_obstacle_y[3])) / 4.0)
    local_view_data['data_polygon_obstacle'].data.update({
      'polygon_obstacle_x' : polygon_obstacle_x_vec,
      'polygon_obstacle_y' : polygon_obstacle_y_vec,
      'polygon_obstacle_label' : polygon_obstacle_label_vec,
      'pos_y' : polygon_obstacle_center_y,
      'pos_x' : polygon_obstacle_center_x,
      'polygon_x' : polygon_x_vec,
      'polygon_y' : polygon_y_vec,
      'polygon_id' : polygon_id_vec
    })

    # parking_space_x_vec, parking_space_y_vec, parking_space_id_vec = hpp_generate_ehr_static_map(ehr_static_map_msg, loc_msg, g_is_display_enu)
    # parking_space_center_x = []
    # for parking_space_box_x in parking_space_x_vec:
    #   parking_space_center_x.append((parking_space_box_x[0] + parking_space_box_x[1] + parking_space_box_x[2] + parking_space_box_x[3]) / 4.0)
    # parking_space_center_y = []
    # for parking_space_box_y in parking_space_y_vec:
    #   parking_space_center_y.append((parking_space_box_y[0] + parking_space_box_y[1] + parking_space_box_y[2] + parking_space_box_y[3]) / 4.0)
    # local_view_data['data_parking_space_text'].data.update({
    #   'parking_space_center_x' : parking_space_center_x,
    #   'parking_space_center_y' : parking_space_center_y,
    #   'parking_space_id_vec' : parking_space_id_vec,
    # })

    parking_assist_info = ehr_static_map_msg.parking_assist_info
    trace_start_x, trace_start_y = [parking_assist_info.trace_start.x], [parking_assist_info.trace_start.y]
    trace_end_x, trace_end_y = [parking_assist_info.trace_end.x], [parking_assist_info.trace_end.y]
    if not g_is_display_enu:
      if loc_msg != None:
        trace_start_x, trace_start_y = coord_tf.global_to_local(trace_start_x, trace_start_y)
        trace_end_x, trace_end_y = coord_tf.global_to_local(trace_end_x, trace_end_y)
    local_view_data['data_map_key_point'].data.update({
      'trace_start_x' : trace_start_x,
      'trace_start_y' : trace_start_y,
      'trace_end_x': trace_end_x,
      'trace_end_y': trace_end_y,
    })

  # 加载fusion ground line
  if bag_loader.fus_ground_line_msg['enable'] == True:
    if bag_loader.plan_debug_msg['enable'] == False:
      environment_model_info = None
    groundline_x_vec, groundline_y_vec, ground_line_id_vec, pos_x_vec, pos_y_vec, ground_line_label_vec, polygon_x_vec, polygon_y_vec = generate_ground_line(ground_line_msg, loc_msg, environment_model_info, g_is_display_enu)
    local_view_data['data_ground_line'].data.update({
      'ground_line_x' : groundline_x_vec,
      'ground_line_y' : groundline_y_vec,
      'polygon_y': polygon_y_vec,
      'polygon_x': polygon_x_vec,
      'ground_line_id' : ground_line_id_vec,
    })

    local_view_data['data_ground_line_label'].data.update({
      'pos_x' : pos_x_vec,
      'pos_y' : pos_y_vec,
      'ground_line_label' : ground_line_label_vec,
    })

    ground_line_point_x_vec, ground_line_point_y_vec, groundline_x_vec, groundline_y_vec, groundline_id_vec = generate_ground_line_clusters(ground_line_msg, loc_msg, g_is_display_enu)
    local_view_data['data_ground_line_point'].data.update({
      'ground_line_x' : ground_line_point_x_vec,
      'ground_line_y' : ground_line_point_y_vec,
    })
    local_view_data['data_ground_line_clusters'].data.update({
      'ground_line_x' : groundline_x_vec,
      'ground_line_y' : groundline_y_vec,
      'ground_line_id' : groundline_id_vec
    })

  # 加载fusion parking slot
  if bag_loader.fus_parking_msg['enable'] == True:
    print("plan release slot id = ", successful_slot_info_list)
    parking_slot_info, release_slot_info, plan_release_slot_info, select_parking_slot_info = generate_parking_slot(fus_parking_msg, loc_msg, successful_slot_info_list)
    # print("fusion parking slot size:", len(parking_slot_info) + len(release_slot_info) + len(plan_release_slot_info) + len(select_parking_slot_info))
    if g_is_display_enu:
      local_view_data['data_parking_slot'].data.update({
        'parking_slot_x' : parking_slot_info['parking_slot_x'],
        'parking_slot_y' : parking_slot_info['parking_slot_y'],
        'pos_x' : parking_slot_info['pos_x'],
        'pos_y' : parking_slot_info['pos_y'],
        'parking_slot_label' : parking_slot_info['parking_slot_label'],
      })
      local_view_data['data_release_slot'].data.update({
        'parking_slot_x' : release_slot_info['parking_slot_x'],
        'parking_slot_y' : release_slot_info['parking_slot_y'],
        'pos_x' : release_slot_info['pos_x'],
        'pos_y' : release_slot_info['pos_y'],
        'parking_slot_label' : release_slot_info['parking_slot_label'],
      })
      local_view_data['data_plan_release_slot'].data.update({
        'parking_slot_x' : plan_release_slot_info['parking_slot_x'],
        'parking_slot_y' : plan_release_slot_info['parking_slot_y'],
        'pos_x' : plan_release_slot_info['pos_x'],
        'pos_y' : plan_release_slot_info['pos_y'],
        'parking_slot_label' : plan_release_slot_info['parking_slot_label'],
      })
      local_view_data['data_select_parking_slot'].data.update({
        'parking_slot_x' : select_parking_slot_info['parking_slot_x'],
        'parking_slot_y' : select_parking_slot_info['parking_slot_y'],
        'pos_x' : select_parking_slot_info['pos_x'],
        'pos_y' : select_parking_slot_info['pos_y'],
        'parking_slot_label' : select_parking_slot_info['parking_slot_label'],
      })
    else:
      local_view_data['data_parking_slot'].data.update({
        'parking_slot_x' : parking_slot_info['parking_slot_x_rel'],
        'parking_slot_y' : parking_slot_info['parking_slot_y_rel'],
        'pos_x' : parking_slot_info['pos_x_rel'],
        'pos_y' : parking_slot_info['pos_y_rel'],
        'parking_slot_label' : parking_slot_info['parking_slot_label'],
      })
      local_view_data['data_release_slot'].data.update({
        'parking_slot_x' : release_slot_info['parking_slot_x_rel'],
        'parking_slot_y' : release_slot_info['parking_slot_y_rel'],
        'pos_x' : release_slot_info['pos_x_rel'],
        'pos_y' : release_slot_info['pos_y_rel'],
        'parking_slot_label' : release_slot_info['parking_slot_label'],
      })
      local_view_data['data_plan_release_slot'].data.update({
        'parking_slot_x' : plan_release_slot_info['parking_slot_x_rel'],
        'parking_slot_y' : plan_release_slot_info['parking_slot_y_rel'],
        'pos_x' : plan_release_slot_info['pos_x_rel'],
        'pos_y' : plan_release_slot_info['pos_y_rel'],
        'parking_slot_label' : plan_release_slot_info['parking_slot_label'],
      })
      local_view_data['data_select_parking_slot'].data.update({
        'parking_slot_x' : select_parking_slot_info['parking_slot_x_rel'],
        'parking_slot_y' : select_parking_slot_info['parking_slot_y_rel'],
        'pos_x' : select_parking_slot_info['pos_x_rel'],
        'pos_y' : select_parking_slot_info['pos_y_rel'],
        'parking_slot_label' : select_parking_slot_info['parking_slot_label'],
      })

    parking_space_x_vec, parking_space_y_vec, parking_space_id_vec = hpp_generate_parking_slot(fus_parking_msg, loc_msg, g_is_display_enu)
    parking_space_center_x = []
    for parking_space_box_x in parking_space_x_vec:
      parking_space_center_x.append((parking_space_box_x[0] + parking_space_box_x[1] + parking_space_box_x[2] + parking_space_box_x[3]) / 4.0)
    parking_space_center_y = []
    for parking_space_box_y in parking_space_y_vec:
      parking_space_center_y.append((parking_space_box_y[0] + parking_space_box_y[1] + parking_space_box_y[2] + parking_space_box_y[3]) / 4.0)
    local_view_data['data_parking_space_text'].data.update({
      'parking_space_center_x' : parking_space_center_x,
      'parking_space_center_y' : parking_space_center_y,
      'parking_space_id_vec' : parking_space_id_vec,
    })

  # 加载fusion speed bump
  if bag_loader.fus_speed_bump_msg['enable'] == True:
    speed_bump_info = generate_speed_bump(fus_speed_bump_msg, loc_msg)
    # print("fusion speed bump size:", len(speed_bump_info))
    if g_is_display_enu:
      local_view_data['data_speed_bump'].data.update({
        'speed_bump_x' : speed_bump_info['speed_bump_x'],
        'speed_bump_y' : speed_bump_info['speed_bump_y'],
        'pos_x' : speed_bump_info['pos_x'],
        'pos_y' : speed_bump_info['pos_y'],
        'speed_bump_label' : speed_bump_info['speed_bump_label'],
      })
    else:
      local_view_data['data_speed_bump'].data.update({
        'speed_bump_x' : speed_bump_info['speed_bump_x_rel'],
        'speed_bump_y' : speed_bump_info['speed_bump_y_rel'],
        'pos_x' : speed_bump_info['pos_x_rel'],
        'pos_y' : speed_bump_info['pos_y_rel'],
        'speed_bump_label' : speed_bump_info['speed_bump_label'],
      })

  # 加载rdg ground line
  if bag_loader.rdg_ground_line_msg['enable'] == True:
    groundline_x_vec, groundline_y_vec = generate_rdg_ground_line(rdg_ground_line_msg, loc_msg, g_is_display_enu)
    local_view_data['data_rdg_ground_line'].data.update({
      'ground_line_x' : groundline_x_vec,
      'ground_line_y' : groundline_y_vec,
    })

  # 加载rdg parking slot
  if bag_loader.rdg_parking_slot_msg['enable'] == True:
    parking_slot_info = generate_rdg_parking_slot(rdg_parking_slot_msg, loc_msg)
    if g_is_display_enu:
      local_view_data['data_rdg_parking_slot'].data.update({
        'parking_slot_x' : parking_slot_info['parking_slot_x'],
        'parking_slot_y' : parking_slot_info['parking_slot_y'],
        'pos_x' : parking_slot_info['pos_x'],
        'pos_y' : parking_slot_info['pos_y'],
        'parking_slot_label' : parking_slot_info['parking_slot_label'],
      })
    else:
      local_view_data['data_rdg_parking_slot'].data.update({
        'parking_slot_x' : parking_slot_info['parking_slot_x_rel'],
        'parking_slot_y' : parking_slot_info['parking_slot_y_rel'],
        'pos_x' : parking_slot_info['pos_x_rel'],
        'pos_y' : parking_slot_info['pos_y_rel'],
        'parking_slot_label' : parking_slot_info['parking_slot_label'],
      })

  # 加载rdg general objects
  if bag_loader.rdg_general_objects_msg['enable'] == True:
    obstacles_info = load_rdg_general_obstacle(rdg_general_objects_msg, loc_msg)
    if g_is_display_enu:
      local_view_data['data_rdg_general_obj'].data.update({
        'obstacles_x': obstacles_info['obstacles_x'],
        'obstacles_y': obstacles_info['obstacles_y'],
        'pos_x' : obstacles_info['pos_x'],
        'pos_y' : obstacles_info['pos_y'],
        'obs_label' : obstacles_info['obs_label'],
      })
    else:
      local_view_data['data_rdg_general_obj'].data.update({
        'obstacles_x': obstacles_info['obstacles_x_rel'],
        'obstacles_y': obstacles_info['obstacles_y_rel'],
        'pos_x' : obstacles_info['pos_x_rel'],
        'pos_y' : obstacles_info['pos_y_rel'],
        'obs_label' : obstacles_info['obs_label'],
      })

  # 加载rdg occ objects
  if bag_loader.rdg_occ_objects_msg['enable'] == True:
    obstacles_info = load_rdg_occupancy_obstacle(rdg_occ_objects_msg, loc_msg)
    if g_is_display_enu:
      local_view_data['data_rdg_occ_obj'].data.update({
        'obstacles_x': obstacles_info['obstacles_x'],
        'obstacles_y': obstacles_info['obstacles_y'],
        'pos_x' : obstacles_info['pos_x'],
        'pos_y' : obstacles_info['pos_y'],
        'obs_label' : obstacles_info['obs_label'],
      })
    else:
      local_view_data['data_rdg_occ_obj'].data.update({
        'obstacles_x': obstacles_info['obstacles_x_rel'],
        'obstacles_y': obstacles_info['obstacles_y_rel'],
        'pos_x' : obstacles_info['pos_x_rel'],
        'pos_y' : obstacles_info['pos_y_rel'],
        'obs_label' : obstacles_info['obs_label'],
      })

  # 加载rdg parking lane line
  if bag_loader.rdg_parking_lane_line_msg['enable'] == True:
    ground_mark_info = load_rdg_parking_lane_line(rdg_parking_lane_line_msg, loc_msg)
    if g_is_display_enu:
      local_view_data['data_rdg_ground_mark'].data.update({
        'ground_mark_x': ground_mark_info['ground_mark_x'],
        'ground_mark_y': ground_mark_info['ground_mark_y'],
        'pos_x' : ground_mark_info['pos_x'],
        'pos_y' : ground_mark_info['pos_y'],
        'ground_mark_label' : ground_mark_info['ground_mark_label'],
      })
    else:
      local_view_data['data_rdg_ground_mark'].data.update({
        'ground_mark_x': ground_mark_info['ground_mark_x_rel'],
        'ground_mark_y': ground_mark_info['ground_mark_y_rel'],
        'pos_x' : ground_mark_info['pos_x_rel'],
        'pos_y' : ground_mark_info['pos_y_rel'],
        'ground_mark_label' : ground_mark_info['ground_mark_label'],
      })

  return local_view_data

def update_select_obstacle_id(prediction_obstacle_id, obstacle_polygon_id, local_view_data):
  select_prediction_obstacle_ids = re.findall(r'\d+', prediction_obstacle_id)
  prediction_obs_id = [int(select_prediction_obstacle_id) for select_prediction_obstacle_id in select_prediction_obstacle_ids]
  print("prediction_obstacle_id: ", prediction_obs_id)

  select_obstacle_polygon_ids = re.findall(r'\d+', obstacle_polygon_id)
  obs_polygon_id = [int(select_obstacle_polygon_id) for select_obstacle_polygon_id in select_obstacle_polygon_ids]
  print("obstacle_polygon_id: ", obs_polygon_id)

  local_view_data['data_select_obs_id'].data.update({
    'prediction_obstacle_id': prediction_obs_id,
    'obstacle_polygon_id': obs_polygon_id,
  })

def load_measure_distance_tool(fig):
  source = ColumnDataSource(data=dict(x=[], y=[]))
  fig.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool', visible = False)
  line_source = ColumnDataSource(data=dict(x=[], y=[]))
  fig.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool', visible = False)
  text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
  fig.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool', visible = False)
  # Define the JavaScript callback code
  callback_code = """
      var x = cb_obj.x;
      var y = cb_obj.y;

      source.data['x'].push(x);
      source.data['y'].push(y);

      if (source.data['x'].length > 2) {
          source.data['x'].shift();
          source.data['y'].shift();
          source.data['x'].shift();
          source.data['y'].shift();
      }
      source.change.emit();

      if (source.data['x'].length >= 2) {
          var x1 = source.data['x'][source.data['x'].length - 2];
          var y1 = source.data['y'][source.data['y'].length - 2];
          var x2 = x;
          var y2 = y;
          var x3 = (x1 + x2) / 2;
          var y3 = (y1 + y2) / 2;

          var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

          console.log("Distance between the last two points: " + distance);

          distance = distance.toFixed(4);
          text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
          text_source.change.emit();

          line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
          line_source.change.emit();
      }

      if (source.data['x'].length == 1) {
          text_source.data['x'].shift();
          text_source.data['y'].shift();
          text_source.data['text'].shift();
      }
      text_source.change.emit();
  """

  # Create a CustomJS callback with the defined code
  callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

  # Attach the callback to the Tap event on the plot
  fig.js_on_event(Tap, callback)

def load_local_view_figure():
  is_vis_map = global_var.get_value('is_vis_map')
  is_vis_sdmap = global_var.get_value('is_vis_sdmap')
  is_vis_hpp = global_var.get_value('is_vis_hpp')
  is_vis_radar = global_var.get_value('is_vis_radar')
  is_vis_rdg_line = global_var.get_value('is_vis_rdg_line')
  is_vis_rdg_obj = global_var.get_value('is_vis_rdg_obj')
  is_vis_me_obj = global_var.get_value('is_vis_me_obj')
  is_vis_lane_mark = global_var.get_value('is_vis_lane_mark')
  is_vis_merge_point = global_var.get_value('is_vis_merge_point')

  data_car = ColumnDataSource(data = {'car_yb':[], 'car_xb':[]})
  data_car_traj = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_car_traj_raw = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_car_traj_mpc = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_car_traj_lat = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_ego = ColumnDataSource(data = {'ego_yb':[], 'ego_xb':[]})
  origin_data_ego = ColumnDataSource(data = {'ego_yb':[], 'ego_xb':[]})
  data_ego_pos_point = ColumnDataSource(data = {'ego_pos_point_y':[],
                                                'ego_pos_point_x':[],
                                                'ego_pos_point_theta':[]})
  data_init_pos_point = ColumnDataSource(data = {'init_pos_point_y':[],
                                                 'init_pos_point_x':[],
                                                 'init_pos_point_theta':[],
                                                 'init_state_x':[],
                                                 'init_state_y':[],
                                                 'init_state_theta':[],
                                                 'init_state_delta':[],
                                                 'init_state_s':[],
                                                 'init_state_v':[],
                                                 'init_state_a':[],
                                                 'replan_status':[],
                                                 'ego_pos_compensation_x': [],
                                                 'ego_pos_compensation_y': []})
  data_init_line = ColumnDataSource(data = {'init_pos_line_x':[], 'init_pos_line_y':[]})
  data_merge_point = ColumnDataSource(data = {'merge_point_x':[],
                                              'merge_point_y':[]})
  data_lon_collision_object_position = ColumnDataSource(data = {'lon_collision_object_position_x':[],
                                                                'lon_collision_object_position_y':[]})
  macroeconomic_decider_data_merge_point = ColumnDataSource(data = {'macroeconomic_decider_merge_point_x':[],
                                              'macroeconomic_decider_merge_point_y':[]})
  boundary_line_merge_point = ColumnDataSource(data = {'boundary_line_merge_point_x':[],
                                              'boundary_line_merge_point_y':[]})
  data_text = ColumnDataSource(data = {'vel_ego_text':[], 'text_xn': [],  'text_yn': []})
  data_lane_dashed_line = ColumnDataSource(data = {'lines_y_vec':[], 'lines_x_vec':[], 'relative_id_vec':[]})
  data_lane_solid_line = ColumnDataSource(data = {'lines_y_vec':[], 'lines_x_vec':[], 'relative_id_vec':[]})
  data_lane_virtual_line = ColumnDataSource(data = {'lines_y_vec':[], 'lines_x_vec':[], 'relative_id_vec':[]})
  data_lane_0 = ColumnDataSource(data = {'line_0_y':[], 'line_0_x':[]})
  data_lane_1 = ColumnDataSource(data = {'line_1_y':[], 'line_1_x':[]})
  data_lane_2 = ColumnDataSource(data = {'line_2_y':[], 'line_2_x':[]})
  data_lane_3 = ColumnDataSource(data = {'line_3_y':[], 'line_3_x':[]})
  data_lane_4 = ColumnDataSource(data = {'line_4_y':[], 'line_4_x':[]})
  data_lane_5 = ColumnDataSource(data = {'line_5_y':[], 'line_5_x':[]})
  data_lane_6 = ColumnDataSource(data = {'line_6_y':[], 'line_6_x':[]})
  data_lane_7 = ColumnDataSource(data = {'line_7_y':[], 'line_7_x':[]})
  data_lane_8 = ColumnDataSource(data = {'line_8_y':[], 'line_8_x':[]})
  data_lane_9 = ColumnDataSource(data = {'line_9_y':[], 'line_9_x':[]})
  data_center_line_gen = ColumnDataSource(data = {'center_line_gen_y':[], 'center_line_gen_x':[]})
  data_lane_10 = ColumnDataSource(data = {'line_10_y':[], 'line_10_x':[]})
  data_lane_11 = ColumnDataSource(data = {'line_11_y':[], 'line_11_x':[]})
  data_lane_12 = ColumnDataSource(data = {'line_12_y':[], 'line_12_x':[]})
  data_lane_13 = ColumnDataSource(data = {'line_13_y':[], 'line_13_x':[]})
  data_lane_14 = ColumnDataSource(data = {'line_14_y':[], 'line_14_x':[]})
  data_lane_15 = ColumnDataSource(data = {'line_15_y':[], 'line_15_x':[]})
  data_lane_16 = ColumnDataSource(data = {'line_16_y':[], 'line_16_x':[]})
  data_lane_17 = ColumnDataSource(data = {'line_17_y':[], 'line_17_x':[]})
  data_lane_18 = ColumnDataSource(data = {'line_18_y':[], 'line_18_x':[]})
  data_lane_19 = ColumnDataSource(data = {'line_19_y':[], 'line_19_x':[]})

  data_lane_topo_0 = ColumnDataSource(data = {'line_topo_0_y':[], 'line_topo_0_x':[]})
  data_lane_topo_1 = ColumnDataSource(data = {'line_topo_1_y':[], 'line_topo_1_x':[]})
  data_lane_topo_2 = ColumnDataSource(data = {'line_topo_2_y':[], 'line_topo_2_x':[]})
  data_lane_topo_3 = ColumnDataSource(data = {'line_topo_3_y':[], 'line_topo_3_x':[]})
  data_lane_topo_4 = ColumnDataSource(data = {'line_topo_4_y':[], 'line_topo_4_x':[]})
  data_lane_topo_5 = ColumnDataSource(data = {'line_topo_5_y':[], 'line_topo_5_x':[]})
  data_lane_topo_6 = ColumnDataSource(data = {'line_topo_6_y':[], 'line_topo_6_x':[]})
  data_lane_topo_7 = ColumnDataSource(data = {'line_topo_7_y':[], 'line_topo_7_x':[]})
  data_lane_topo_8 = ColumnDataSource(data = {'line_topo_8_y':[], 'line_topo_8_x':[]})
  data_lane_topo_9 = ColumnDataSource(data = {'line_topo_9_y':[], 'line_topo_9_x':[]})
  data_lane_topo_10 = ColumnDataSource(data = {'line_topo_10_y':[], 'line_topo_10_x':[]})
  data_lane_topo_11 = ColumnDataSource(data = {'line_topo_11_y':[], 'line_topo_11_x':[]})
  data_lane_topo_12 = ColumnDataSource(data = {'line_topo_12_y':[], 'line_topo_12_x':[]})
  data_lane_topo_13 = ColumnDataSource(data = {'line_topo_13_y':[], 'line_topo_13_x':[]})
  data_lane_topo_14 = ColumnDataSource(data = {'line_topo_14_y':[], 'line_topo_14_x':[]})
  data_lane_topo_15 = ColumnDataSource(data = {'line_topo_15_y':[], 'line_topo_15_x':[]})
  data_lane_topo_16 = ColumnDataSource(data = {'line_topo_16_y':[], 'line_topo_16_x':[]})
  data_lane_topo_17 = ColumnDataSource(data = {'line_topo_17_y':[], 'line_topo_17_x':[]})
  data_lane_topo_18 = ColumnDataSource(data = {'line_topo_18_y':[], 'line_topo_18_x':[]})
  data_lane_topo_19 = ColumnDataSource(data = {'line_topo_19_y':[], 'line_topo_19_x':[]})

  rdg_data_lane_0 = ColumnDataSource(data = {'rdg_line_0_y':[], 'rdg_line_0_x':[]})
  rdg_data_lane_1 = ColumnDataSource(data = {'rdg_line_1_y':[], 'rdg_line_1_x':[]})
  rdg_data_lane_2 = ColumnDataSource(data = {'rdg_line_2_y':[], 'rdg_line_2_x':[]})
  rdg_data_lane_3 = ColumnDataSource(data = {'rdg_line_3_y':[], 'rdg_line_3_x':[]})
  rdg_data_lane_4 = ColumnDataSource(data = {'rdg_line_4_y':[], 'rdg_line_4_x':[]})
  rdg_data_lane_5 = ColumnDataSource(data = {'rdg_line_5_y':[], 'rdg_line_5_x':[]})
  rdg_data_lane_6 = ColumnDataSource(data = {'rdg_line_6_y':[], 'rdg_line_6_x':[]})
  rdg_data_lane_7 = ColumnDataSource(data = {'rdg_line_7_y':[], 'rdg_line_7_x':[]})
  rdg_data_lane_8 = ColumnDataSource(data = {'rdg_line_8_y':[], 'rdg_line_8_x':[]})
  rdg_data_lane_9 = ColumnDataSource(data = {'rdg_line_9_y':[], 'rdg_line_9_x':[]})
  rdg_data_lane_10 = ColumnDataSource(data = {'rdg_line_10_y':[], 'rdg_line_10_x':[]})
  rdg_data_lane_11 = ColumnDataSource(data = {'rdg_line_11_y':[], 'rdg_line_11_x':[]})
  rdg_data_lane_12 = ColumnDataSource(data = {'rdg_line_12_y':[], 'rdg_line_12_x':[]})
  rdg_data_lane_13 = ColumnDataSource(data = {'rdg_line_13_y':[], 'rdg_line_13_x':[]})
  rdg_data_lane_14 = ColumnDataSource(data = {'rdg_line_14_y':[], 'rdg_line_14_x':[]})
  rdg_data_lane_15 = ColumnDataSource(data = {'rdg_line_15_y':[], 'rdg_line_15_x':[]})
  rdg_data_lane_16 = ColumnDataSource(data = {'rdg_line_16_y':[], 'rdg_line_16_x':[]})
  rdg_data_lane_17 = ColumnDataSource(data = {'rdg_line_17_y':[], 'rdg_line_17_x':[]})
  rdg_data_lane_18 = ColumnDataSource(data = {'rdg_line_18_y':[], 'rdg_line_18_x':[]})
  rdg_data_lane_19 = ColumnDataSource(data = {'rdg_line_19_y':[], 'rdg_line_19_x':[]})

  stop_line_0 = ColumnDataSource(data = {'stop_line_0_x':[], 'stop_line_0_y':[]})
  stop_line_1 = ColumnDataSource(data = {'stop_line_1_x':[], 'stop_line_1_y':[]})
  stop_line_2 = ColumnDataSource(data = {'stop_line_2_x':[], 'stop_line_2_y':[]})
  stop_line_3 = ColumnDataSource(data = {'stop_line_3_x':[], 'stop_line_3_y':[]})
  stop_line_4 = ColumnDataSource(data = {'stop_line_4_x':[], 'stop_line_4_y':[]})

  zebra_crossing_line_0 = ColumnDataSource(data = {'zebra_crossing_line_0_x':[], 'zebra_crossing_line_0_y':[]})
  zebra_crossing_line_1 = ColumnDataSource(data = {'zebra_crossing_line_1_x':[], 'zebra_crossing_line_1_y':[]})
  zebra_crossing_line_2 = ColumnDataSource(data = {'zebra_crossing_line_2_x':[], 'zebra_crossing_line_2_y':[]})
  zebra_crossing_line_3 = ColumnDataSource(data = {'zebra_crossing_line_3_x':[], 'zebra_crossing_line_3_y':[]})
  zebra_crossing_line_4 = ColumnDataSource(data = {'zebra_crossing_line_4_x':[], 'zebra_crossing_line_4_y':[]})
  zebra_crossing_line_5 = ColumnDataSource(data = {'zebra_crossing_line_5_x':[], 'zebra_crossing_line_5_y':[]})
  zebra_crossing_line_6 = ColumnDataSource(data = {'zebra_crossing_line_6_x':[], 'zebra_crossing_line_6_y':[]})
  zebra_crossing_line_7 = ColumnDataSource(data = {'zebra_crossing_line_7_x':[], 'zebra_crossing_line_7_y':[]})
  zebra_crossing_line_8 = ColumnDataSource(data = {'zebra_crossing_line_8_x':[], 'zebra_crossing_line_8_y':[]})
  zebra_crossing_line_9 = ColumnDataSource(data = {'zebra_crossing_line_9_x':[], 'zebra_crossing_line_9_y':[]})
  zebra_crossing_line_10 = ColumnDataSource(data = {'zebra_crossing_line_10_x':[], 'zebra_crossing_line_10_y':[]})
  zebra_crossing_line_11 = ColumnDataSource(data = {'zebra_crossing_line_11_x':[], 'zebra_crossing_line_11_y':[]})

  if is_vis_map:
    ehr_data_lanes = []
    for i in range(Max_line_size):
      ehr_data_lanes.append(ColumnDataSource(data={'ehr_line_{}_y'.format(i): [], 'ehr_line_{}_x'.format(i): []}))

    ehr_road_boundary_lanes = []
    for i in range(Road_boundary_max_line_size):
      ehr_road_boundary_lanes.append(ColumnDataSource(data={'ehr_road_boundary_{}_y'.format(i): [], 'ehr_road_boundary_{}_x'.format(i): []}))

    ehr_lane_boundary_lanes = []
    for i in range(Lane_boundary_max_line_size):
      ehr_lane_boundary_lanes.append(ColumnDataSource(data={'ehr_lane_boundary_{}_y'.format(i): [], 'ehr_lane_boundary_{}_x'.format(i): []}))
  ## add plot sdmap info
  if is_vis_sdmap:
    data_sdmap_road_line = ColumnDataSource(data = {'data_sdmap_road_line_y':[], 'data_sdmap_road_line_x':[]})
    data_sdmap_ramp_line = ColumnDataSource(data = {'data_sdmap_ramp_line_y':[], 'data_sdmap_ramp_line_x':[]})
    data_sdmap_inlink = ColumnDataSource(data = {'data_sdmap_inlink_y':[], 'data_sdmap_inlink_x':[]})
    data_sdmap_outlink = ColumnDataSource(data = {'data_sdmap_outlink_y':[], 'data_sdmap_outlink_x':[]})

  data_center_line_0 = ColumnDataSource(data = {'center_line_0_y':[], 'center_line_0_x':[]})
  data_center_line_1 = ColumnDataSource(data = {'center_line_1_y':[], 'center_line_1_x':[]})
  data_center_line_2 = ColumnDataSource(data = {'center_line_2_y':[], 'center_line_2_x':[]})
  data_center_line_3 = ColumnDataSource(data = {'center_line_3_y':[], 'center_line_3_x':[]})
  data_center_line_4 = ColumnDataSource(data = {'center_line_4_y':[], 'center_line_4_x':[]})
  data_center_line_5 = ColumnDataSource(data = {'center_line_5_y':[], 'center_line_5_x':[]})
  data_center_line_6 = ColumnDataSource(data = {'center_line_6_y':[], 'center_line_6_x':[]})
  data_center_line_7 = ColumnDataSource(data = {'center_line_7_y':[], 'center_line_7_x':[]})
  data_center_line_8 = ColumnDataSource(data = {'center_line_8_y':[], 'center_line_8_x':[]})
  data_center_line_9 = ColumnDataSource(data = {'center_line_9_y':[], 'center_line_9_x':[]})

  data_center_line_topo_0 = ColumnDataSource(data = {'center_line_topo_0_y':[], 'center_line_topo_0_x':[]})
  data_center_line_topo_1 = ColumnDataSource(data = {'center_line_topo_1_y':[], 'center_line_topo_1_x':[]})
  data_center_line_topo_2 = ColumnDataSource(data = {'center_line_topo_2_y':[], 'center_line_topo_2_x':[]})
  data_center_line_topo_3 = ColumnDataSource(data = {'center_line_topo_3_y':[], 'center_line_topo_3_x':[]})
  data_center_line_topo_4 = ColumnDataSource(data = {'center_line_topo_4_y':[], 'center_line_topo_4_x':[]})
  data_center_line_topo_5 = ColumnDataSource(data = {'center_line_topo_5_y':[], 'center_line_topo_5_x':[]})
  data_center_line_topo_6 = ColumnDataSource(data = {'center_line_topo_6_y':[], 'center_line_topo_6_x':[]})
  data_center_line_topo_7 = ColumnDataSource(data = {'center_line_topo_7_y':[], 'center_line_topo_7_x':[]})
  data_center_line_topo_8 = ColumnDataSource(data = {'center_line_topo_8_y':[], 'center_line_topo_8_x':[]})
  data_center_line_topo_9 = ColumnDataSource(data = {'center_line_topo_9_y':[], 'center_line_topo_9_x':[]})

  lane_mark_data_0 = ColumnDataSource(data = {'lane_mark_0':[], 'text_xn_0': [],  'text_yn_0': [] , 'lane_mark_loc_x_0': [], 'lane_mark_loc_y_0': []})
  lane_mark_data_1 = ColumnDataSource(data = {'lane_mark_1':[], 'text_xn_1': [],  'text_yn_1': [] , 'lane_mark_loc_x_1': [], 'lane_mark_loc_y_1': []})
  lane_mark_data_2 = ColumnDataSource(data = {'lane_mark_2':[], 'text_xn_2': [],  'text_yn_2': [] , 'lane_mark_loc_x_2': [], 'lane_mark_loc_y_2': []})
  lane_mark_data_3 = ColumnDataSource(data = {'lane_mark_3':[], 'text_xn_3': [],  'text_yn_3': [] , 'lane_mark_loc_x_3': [], 'lane_mark_loc_y_3': []})
  lane_mark_data_4 = ColumnDataSource(data = {'lane_mark_4':[], 'text_xn_4': [],  'text_yn_4': [] , 'lane_mark_loc_x_4': [], 'lane_mark_loc_y_4': []})
  lane_mark_data_5 = ColumnDataSource(data = {'lane_mark_5':[], 'text_xn_5': [],  'text_yn_5': [] , 'lane_mark_loc_x_5': [], 'lane_mark_loc_y_5': []})
  lane_mark_data_6 = ColumnDataSource(data = {'lane_mark_6':[], 'text_xn_6': [],  'text_yn_6': [] , 'lane_mark_loc_x_6': [], 'lane_mark_loc_y_6': []})
  lane_mark_data_7 = ColumnDataSource(data = {'lane_mark_7':[], 'text_xn_7': [],  'text_yn_7': [] , 'lane_mark_loc_x_7': [], 'lane_mark_loc_y_7': []})
  lane_mark_data_8 = ColumnDataSource(data = {'lane_mark_8':[], 'text_xn_8': [],  'text_yn_8': [] , 'lane_mark_loc_x_8': [], 'lane_mark_loc_y_8': []})
  lane_mark_data_9 = ColumnDataSource(data = {'lane_mark_9':[], 'text_xn_9': [],  'text_yn_9': [] , 'lane_mark_loc_x_9': [], 'lane_mark_loc_y_9': []})


  data_fix_lane = ColumnDataSource(data = {'fix_lane_y':[], 'fix_lane_x':[]})
  data_target_lane = ColumnDataSource(data = {'target_lane_y':[], 'target_lane_x':[]})
  data_origin_lane = ColumnDataSource(data = {'origin_lane_y':[], 'origin_lane_x':[]})

  data_select_obs_id = ColumnDataSource(data = {'prediction_obstacle_id':[],
                                                'obstacle_polygon_id':[],
                                                })
  data_fus_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                          'polygon_y':[], 'polygon_x':[],
                                          'pos_y':[], 'pos_x':[],
                                          'obs_id':[], 'obs_label':[]})
  data_me_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_rdg_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_fm_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_fl_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_fr_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_rl_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_rr_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_snrd_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                           'polygon_y':[], 'polygon_x':[],
                                           'pos_y':[], 'pos_x':[],
                                           'obs_id':[], 'obs_label':[]})
  data_planning_lat = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_raw = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],
                                      'plan_traj_s':[],})
  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],
                                      'plan_traj_s':[],})
  data_planning_0 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_1 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_2 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_3 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_4 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})
  data_prediction_0 = ColumnDataSource(data = {'prediction_y':[],
                                               'prediction_x':[],
                                               'prediction_obs_y':[],
                                               'prediction_obs_x':[],})
  data_prediction_1 = ColumnDataSource(data = {'prediction_y':[],
                                               'prediction_x':[],
                                               'prediction_obs_y':[],
                                               'prediction_obs_x':[],})
  data_prediction_2 = ColumnDataSource(data = {'prediction_y':[],
                                               'prediction_x':[],
                                               'prediction_obs_y':[],
                                               'prediction_obs_x':[],})
  data_prediction_3 = ColumnDataSource(data = {'prediction_y':[],
                                               'prediction_x':[],
                                               'prediction_obs_y':[],
                                               'prediction_obs_x':[],})
  data_prediction_4 = ColumnDataSource(data = {'prediction_y':[],
                                               'prediction_x':[],
                                               'prediction_obs_y':[],
                                               'prediction_obs_x':[],})
  data_parking_space = ColumnDataSource(data = {'parking_space_y':[],
                                             'parking_space_x':[],})
  data_parking_space_text = ColumnDataSource(data = {'parking_space_center_y':[],
                                                     'parking_space_center_x':[],
                                                     'parking_space_id_vec':[],})
  data_road_mark = ColumnDataSource(data = {'road_mark_y':[],
                                            'road_mark_x':[],})
  data_road_obstacle = ColumnDataSource(data = {'road_obstacle_y':[],
                                                'road_obstacle_x':[],})
  data_polygon_obstacle = ColumnDataSource(data = {'polygon_obstacle_y':[],
                                                   'polygon_obstacle_x':[],
                                                   'polygon_obstacle_label':[],
                                                   'pos_y':[],
                                                   'pos_x':[],
                                                   'polygon_y':[],
                                                   'polygon_x':[],
                                                   'polygon_id':[]})
  data_map_key_point = ColumnDataSource(data = {'trace_start_y':[],
                                                'trace_start_x':[],
                                                'trace_end_y':[],
                                                'trace_end_x':[]})
  data_ground_line = ColumnDataSource(data = {'ground_line_y':[],
                                              'ground_line_x':[],
                                              'polygon_y':[],
                                              'polygon_x':[],
                                              'ground_line_id':[]})
  data_ground_line_label = ColumnDataSource(data = {'pos_y':[],
                                                    'pos_x':[],
                                                    'ground_line_label':[]})
  data_ground_line_point = ColumnDataSource(data = {'ground_line_y':[],
                                                    'ground_line_x':[],})
  data_ground_line_clusters = ColumnDataSource(data = {'ground_line_y':[],
                                                       'ground_line_x':[],
                                                       'ground_line_id':[],})
  data_rdg_ground_line = ColumnDataSource(data = {'ground_line_y':[],
                                                  'ground_line_x':[],})
  data_fus_occ_obj = ColumnDataSource(data = {'obstacles_y':[],
                                              'obstacles_x':[],
                                              'polygon_y':[],
                                              'polygon_x':[],
                                              'pos_y':[],
                                              'pos_x':[],
                                              'obs_id':[],
                                              'obs_label':[]})

  data_rdg_general_obj = ColumnDataSource(data = {'obstacles_y':[],
                                                  'obstacles_x':[],
                                                  'pos_y':[],
                                                  'pos_x':[],
                                                  'obs_label':[]})

  data_rdg_occ_obj = ColumnDataSource(data = {'obstacles_y':[],
                                              'obstacles_x':[],
                                              'pos_y':[],
                                              'pos_x':[],
                                              'obs_label':[]})

  data_select_parking_slot = ColumnDataSource(data = {'parking_slot_y':[],
                                                      'parking_slot_x':[],
                                                      'pos_y':[],
                                                      'pos_x':[],
                                                      'parking_slot_label':[]})

  data_parking_slot = ColumnDataSource(data = {'parking_slot_y':[],
                                               'parking_slot_x':[],
                                               'pos_y':[],
                                               'pos_x':[],
                                               'parking_slot_label':[]})

  data_release_slot = ColumnDataSource(data = {'parking_slot_y':[],
                                               'parking_slot_x':[],
                                               'pos_y':[],
                                               'pos_x':[],
                                               'parking_slot_label':[]})

  data_plan_release_slot = ColumnDataSource(data = {'parking_slot_y':[],
                                                    'parking_slot_x':[],
                                                    'pos_y':[],
                                                    'pos_x':[],
                                                    'parking_slot_label':[]})

  data_rdg_parking_slot = ColumnDataSource(data = {'parking_slot_y':[],
                                                   'parking_slot_x':[],
                                                   'pos_y':[],
                                                   'pos_x':[],
                                                   'parking_slot_label':[]})

  data_rdg_ground_mark = ColumnDataSource(data = {'ground_mark_y':[],
                                                  'ground_mark_x':[],
                                                  'pos_y':[],
                                                  'pos_x':[],
                                                  'ground_mark_label':[]})

  data_speed_bump = ColumnDataSource(data = {'speed_bump_y':[],
                                             'speed_bump_x':[],
                                             'pos_y':[],
                                             'pos_x':[],
                                             'speed_bump_label':[]})

  data_index = {'loc_msg_idx': 0,
                'road_msg_idx': 0,
                'fus_msg_idx': 0,
                'mobileye_objects_msg_idx':0,
                'rdg_objects_msg_idx':0,
                'radar_fm_msg_idx':0,
                'radar_fl_msg_idx':0,
                'radar_fr_msg_idx':0,
                'radar_rl_msg_idx':0,
                'radar_rr_msg_idx':0,
                'vs_msg_idx': 0,
                'plan_msg_idx': 0,
                'plan_debug_msg_idx': 0,
                'pred_msg_idx': 0,
                'ctrl_msg_idx': 0,
                'ctrl_debug_msg_idx': 0,
                'ehr_static_map_msg_idx': 0,
                'planning_hmi_msg_idx': 0,
                'ehr_sd_map_msg_idx': 0,
               }

  data_msg = {'plan_msg': None,
              'plan_debug_msg': None,
              'loc_msg': None,
              'plan_debug_json_msg': None,
              'ctrl_msg':None,
              'ctrl_debug_msg':None,
              'ctrl_debug_json_msg':None,
  }

  local_view_data = {'data_car':data_car, \
                     'data_car_traj':data_car_traj, \
                     'data_car_traj_raw':data_car_traj_raw, \
                     'data_car_traj_mpc':data_car_traj_mpc, \
                     'data_car_traj_lat':data_car_traj_lat, \
                     'data_ego':data_ego, \
                     'origin_data_ego':origin_data_ego, \
                     'data_ego_pos_point': data_ego_pos_point, \
                     'data_init_pos_point': data_init_pos_point, \
                     'data_init_line': data_init_line, \
                     'data_merge_point': data_merge_point, \
                     'data_lon_collision_object_position': data_lon_collision_object_position, \
                     'macroeconomic_decider_data_merge_point': macroeconomic_decider_data_merge_point, \
                     'boundary_line_merge_point': boundary_line_merge_point, \
                     'data_text':data_text, \
                     'data_lane_dashed_line':data_lane_dashed_line, \
                     'data_lane_solid_line':data_lane_solid_line, \
                     'data_lane_virtual_line':data_lane_virtual_line, \
                     'data_lane_0':data_lane_0, \
                     'data_lane_1':data_lane_1, \
                     'data_lane_2':data_lane_2, \
                     'data_lane_3':data_lane_3, \
                     'data_lane_4':data_lane_4, \
                     'data_lane_5':data_lane_5, \
                     'data_lane_6':data_lane_6, \
                     'data_lane_7':data_lane_7, \
                     'data_lane_8':data_lane_8, \
                     'data_lane_9':data_lane_9, \
                     'data_lane_10':data_lane_10, \
                     'data_lane_11':data_lane_11, \
                     'data_lane_12':data_lane_12, \
                     'data_lane_13':data_lane_13, \
                     'data_lane_14':data_lane_14, \
                     'data_lane_15':data_lane_15, \
                     'data_lane_16':data_lane_16, \
                     'data_lane_17':data_lane_17, \
                     'data_lane_18':data_lane_18, \
                     'data_lane_19':data_lane_19, \
                     'data_center_line_gen': data_center_line_gen, \
                     'data_center_line_0':data_center_line_0, \
                     'data_center_line_1':data_center_line_1, \
                     'data_center_line_2':data_center_line_2, \
                     'data_center_line_3':data_center_line_3, \
                     'data_center_line_4':data_center_line_4, \
                     'data_center_line_5':data_center_line_5, \
                     'data_center_line_6':data_center_line_6, \
                     'data_center_line_7':data_center_line_7, \
                     'data_center_line_8':data_center_line_8, \
                     'data_center_line_9':data_center_line_9, \
                     'rdg_data_lane_0':rdg_data_lane_0, \
                     'rdg_data_lane_1':rdg_data_lane_1, \
                     'rdg_data_lane_2':rdg_data_lane_2, \
                     'rdg_data_lane_3':rdg_data_lane_3, \
                     'rdg_data_lane_4':rdg_data_lane_4, \
                     'rdg_data_lane_5':rdg_data_lane_5, \
                     'rdg_data_lane_6':rdg_data_lane_6, \
                     'rdg_data_lane_7':rdg_data_lane_7, \
                     'rdg_data_lane_8':rdg_data_lane_8, \
                     'rdg_data_lane_9':rdg_data_lane_9, \
                     'rdg_data_lane_10':rdg_data_lane_10, \
                     'rdg_data_lane_11':rdg_data_lane_11, \
                     'rdg_data_lane_12':rdg_data_lane_12, \
                     'rdg_data_lane_13':rdg_data_lane_13, \
                     'rdg_data_lane_14':rdg_data_lane_14, \
                     'rdg_data_lane_15':rdg_data_lane_15, \
                     'rdg_data_lane_16':rdg_data_lane_16, \
                     'rdg_data_lane_17':rdg_data_lane_17, \
                     'rdg_data_lane_18':rdg_data_lane_18, \
                     'rdg_data_lane_19':rdg_data_lane_19, \
                     'data_fix_lane': data_fix_lane ,\
                     'data_target_lane': data_target_lane ,\
                     'data_origin_lane': data_origin_lane ,\
                     'data_select_obs_id': data_select_obs_id,\
                     'data_prediction_0' : data_prediction_0 ,\
                     'data_prediction_1' : data_prediction_1 ,\
                     'data_prediction_2' : data_prediction_2 ,\
                     'data_prediction_3' : data_prediction_3 ,\
                     'data_prediction_4' : data_prediction_4 ,\
                     'data_parking_space' : data_parking_space , \
                     'data_parking_space_text' : data_parking_space_text , \
                     'data_road_mark' : data_road_mark , \
                     'data_road_obstacle': data_road_obstacle, \
                     'data_polygon_obstacle': data_polygon_obstacle, \
                     'data_map_key_point': data_map_key_point, \
                     'data_ground_line' : data_ground_line, \
                     'data_ground_line_label': data_ground_line_label, \
                     'data_ground_line_point' : data_ground_line_point, \
                     'data_ground_line_clusters' : data_ground_line_clusters, \
                     'data_rdg_ground_line': data_rdg_ground_line, \
                     'data_select_parking_slot': data_select_parking_slot, \
                     'data_parking_slot': data_parking_slot, \
                     'data_release_slot': data_release_slot, \
                     'data_plan_release_slot': data_plan_release_slot, \
                     'data_rdg_parking_slot': data_rdg_parking_slot, \
                     'data_rdg_ground_mark': data_rdg_ground_mark, \
                     'data_speed_bump': data_speed_bump, \
                     'data_fus_occ_obj' : data_fus_occ_obj, \
                     'data_fus_obj':data_fus_obj, \
                     'data_me_obj':data_me_obj, \
                     'data_rdg_obj':data_rdg_obj, \
                     'data_rdg_general_obj': data_rdg_general_obj, \
                     'data_rdg_occ_obj': data_rdg_occ_obj, \
                     'data_radar_fm_obj':data_radar_fm_obj, \
                     'data_radar_fl_obj':data_radar_fl_obj, \
                     'data_radar_fr_obj':data_radar_fr_obj, \
                     'data_radar_rl_obj':data_radar_rl_obj, \
                     'data_radar_rr_obj':data_radar_rr_obj, \
                     'data_snrd_obj':data_snrd_obj, \
                     'data_planning_lat':data_planning_lat,\
                     'data_planning_raw':data_planning_raw,\
                     'data_planning':data_planning,\
                     'data_planning_0':data_planning_0,\
                     'data_planning_1':data_planning_1,\
                     'data_planning_2':data_planning_2,\
                     'data_planning_3':data_planning_3,\
                     'data_planning_4':data_planning_4,\
                     'data_control':data_control,\
                     'data_index': data_index, \
                     'data_msg': data_msg,\
                     'data_center_line_topo_0':data_center_line_topo_0, \
                     'data_center_line_topo_1':data_center_line_topo_1, \
                     'data_center_line_topo_2':data_center_line_topo_2, \
                     'data_center_line_topo_3':data_center_line_topo_3, \
                     'data_center_line_topo_4':data_center_line_topo_4, \
                     'data_center_line_topo_5':data_center_line_topo_5, \
                     'data_center_line_topo_6':data_center_line_topo_6, \
                     'data_center_line_topo_7':data_center_line_topo_7, \
                     'data_center_line_topo_8':data_center_line_topo_8, \
                     'data_center_line_topo_9':data_center_line_topo_9,\
                     'lane_mark_data_0':lane_mark_data_0,\
                     'lane_mark_data_1':lane_mark_data_1,\
                     'lane_mark_data_2':lane_mark_data_2,\
                     'lane_mark_data_3':lane_mark_data_3,\
                     'lane_mark_data_4':lane_mark_data_4,\
                     'lane_mark_data_5':lane_mark_data_5,\
                     'lane_mark_data_6':lane_mark_data_6,\
                     'lane_mark_data_7':lane_mark_data_7,\
                     'lane_mark_data_8':lane_mark_data_8,\
                     'lane_mark_data_9':lane_mark_data_9,\
                     'data_lane_topo_1':data_lane_topo_1,\
                     'data_lane_topo_2':data_lane_topo_2,\
                     'data_lane_topo_3':data_lane_topo_3,\
                     'data_lane_topo_4':data_lane_topo_4,\
                     'data_lane_topo_5':data_lane_topo_5,\
                     'data_lane_topo_6':data_lane_topo_6,\
                     'data_lane_topo_7':data_lane_topo_7,\
                     'data_lane_topo_8':data_lane_topo_8,\
                     'data_lane_topo_9':data_lane_topo_9,\
                     'data_lane_topo_10':data_lane_topo_10,\
                     'data_lane_topo_11':data_lane_topo_11,\
                     'data_lane_topo_12':data_lane_topo_12,\
                     'data_lane_topo_13':data_lane_topo_13,\
                     'data_lane_topo_14':data_lane_topo_14,\
                     'data_lane_topo_15':data_lane_topo_15,\
                     'data_lane_topo_16':data_lane_topo_16,\
                     'data_lane_topo_17':data_lane_topo_17,\
                     'data_lane_topo_18':data_lane_topo_18,\
                     'data_lane_topo_19':data_lane_topo_19,\
                     'stop_line_0':stop_line_0,\
                     'stop_line_1':stop_line_1,\
                     'stop_line_2':stop_line_2,\
                     'stop_line_3':stop_line_3,\
                     'stop_line_4':stop_line_4,\
                     'zebra_crossing_line_0':zebra_crossing_line_0,\
                     'zebra_crossing_line_1':zebra_crossing_line_1,\
                     'zebra_crossing_line_2':zebra_crossing_line_2,\
                     'zebra_crossing_line_3':zebra_crossing_line_3,\
                     'zebra_crossing_line_4':zebra_crossing_line_4,\
                     'zebra_crossing_line_5':zebra_crossing_line_5,\
                     'zebra_crossing_line_6':zebra_crossing_line_6,\
                     'zebra_crossing_line_7':zebra_crossing_line_7,\
                     'zebra_crossing_line_8':zebra_crossing_line_8,\
                     'zebra_crossing_line_9':zebra_crossing_line_9,\
                     'zebra_crossing_line_10':zebra_crossing_line_10,\
                     'zebra_crossing_line_11':zebra_crossing_line_11,
                     }
  if is_vis_map:
    for i in range(len(ehr_data_lanes)):
      key = 'ehr_data_lane_' + str(i)
      value = ehr_data_lanes[i]
      local_view_data[key] = value

    for i in range(len(ehr_road_boundary_lanes)):
      key = 'ehr_road_boundary_' + str(i)
      value = ehr_road_boundary_lanes[i]
      local_view_data[key] = value

    for i in range (len(ehr_lane_boundary_lanes)):
      key = 'ehr_lane_boundary_' + str(i)
      value = ehr_lane_boundary_lanes[i]
      local_view_data[key] = value
  if is_vis_sdmap:
    local_view_data ["data_sdmap_road_line"] = data_sdmap_road_line
    local_view_data ["data_sdmap_ramp_line"] = data_sdmap_ramp_line
    local_view_data ["data_sdmap_inlink"] = data_sdmap_inlink
    local_view_data ["data_sdmap_outlink"] = data_sdmap_outlink

  ### figures config
  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=1000, height=1350, match_aspect = True, aspect_scale=1)
  fig1.background_fill_color = 'lightgray'
  fig1.xgrid.grid_line_color = None
  fig1.ygrid.grid_line_color = None

  fig1.x_range.flipped = True
  # figure plot

  # !!!!!!!!!!!! Important: Do not draw above !!!!!!!!!!!
  f0 = fig1.line('line_0_y', 'line_0_x', source = data_lane_0, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_1_y', 'line_1_x', source = data_lane_1, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_2_y', 'line_2_x', source = data_lane_2, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_3_y', 'line_3_x', source = data_lane_3, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_4_y', 'line_4_x', source = data_lane_4, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_5_y', 'line_5_x', source = data_lane_5, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_6_y', 'line_6_x', source = data_lane_6, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_7_y', 'line_7_x', source = data_lane_7, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_8_y', 'line_8_x', source = data_lane_8, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_9_y', 'line_9_x', source = data_lane_9, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_10_y', 'line_10_x', source = data_lane_10, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_11_y', 'line_11_x', source = data_lane_11, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_12_y', 'line_12_x', source = data_lane_12, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_13_y', 'line_13_x', source = data_lane_13, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_14_y', 'line_14_x', source = data_lane_14, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_15_y', 'line_15_x', source = data_lane_15, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_16_y', 'line_16_x', source = data_lane_16, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_17_y', 'line_17_x', source = data_lane_17, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_18_y', 'line_18_x', source = data_lane_18, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('line_19_y', 'line_19_x', source = data_lane_19, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion',visible = False)
  fig1.line('center_line_gen_y', 'center_line_gen_x', source = data_center_line_gen, line_width = 3, line_color = 'cyan', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'center_line_gen')
  # !!!!!!!!!!!! Important: Do not draw above !!!!!!!!!!!
  f21 = fig1.line('line_topo_0_y', 'line_topo_0_x', source = data_lane_topo_0, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_1_y', 'line_topo_1_x', source = data_lane_topo_1, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_2_y', 'line_topo_2_x', source = data_lane_topo_2, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_3_y', 'line_topo_3_x', source = data_lane_topo_3, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_4_y', 'line_topo_4_x', source = data_lane_topo_4, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_5_y', 'line_topo_5_x', source = data_lane_topo_5, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_6_y', 'line_topo_6_x', source = data_lane_topo_6, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_7_y', 'line_topo_7_x', source = data_lane_topo_7, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_8_y', 'line_topo_8_x', source = data_lane_topo_8, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_9_y', 'line_topo_9_x', source = data_lane_topo_9, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_10_y', 'line_topo_10_x', source = data_lane_topo_10, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_11_y', 'line_topo_11_x', source = data_lane_topo_11, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_12_y', 'line_topo_12_x', source = data_lane_topo_12, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_13_y', 'line_topo_13_x', source = data_lane_topo_13, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_14_y', 'line_topo_14_x', source = data_lane_topo_14, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_15_y', 'line_topo_15_x', source = data_lane_topo_15, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_16_y', 'line_topo_16_x', source = data_lane_topo_16, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_17_y', 'line_topo_17_x', source = data_lane_topo_17, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_18_y', 'line_topo_18_x', source = data_lane_topo_18, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)
  fig1.line('line_topo_19_y', 'line_topo_19_x', source = data_lane_topo_19, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'lane_topo',visible = False)

  if is_vis_rdg_line:
    f41 = fig1.line('rdg_line_0_y', 'rdg_line_0_x', source = rdg_data_lane_0, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_1_y', 'rdg_line_1_x', source = rdg_data_lane_1, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_2_y', 'rdg_line_2_x', source = rdg_data_lane_2, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_3_y', 'rdg_line_3_x', source = rdg_data_lane_3, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_4_y', 'rdg_line_4_x', source = rdg_data_lane_4, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_5_y', 'rdg_line_5_x', source = rdg_data_lane_5, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_6_y', 'rdg_line_6_x', source = rdg_data_lane_6, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_7_y', 'rdg_line_7_x', source = rdg_data_lane_7, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_8_y', 'rdg_line_8_x', source = rdg_data_lane_8, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_9_y', 'rdg_line_9_x', source = rdg_data_lane_9, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_10_y', 'rdg_line_10_x', source = rdg_data_lane_10, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_11_y', 'rdg_line_11_x', source = rdg_data_lane_11, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_12_y', 'rdg_line_12_x', source = rdg_data_lane_12, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_13_y', 'rdg_line_13_x', source = rdg_data_lane_13, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_14_y', 'rdg_line_14_x', source = rdg_data_lane_14, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_15_y', 'rdg_line_15_x', source = rdg_data_lane_15, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_16_y', 'rdg_line_16_x', source = rdg_data_lane_16, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_17_y', 'rdg_line_17_x', source = rdg_data_lane_17, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_18_y', 'rdg_line_18_x', source = rdg_data_lane_18, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')
    fig1.line('rdg_line_19_y', 'rdg_line_19_x', source = rdg_data_lane_19, line_width = 1.5, line_color = 'green', line_dash = 'dashed', legend_label = 'rdg_lane')

    f61 = fig1.line('stop_line_0_y', 'stop_line_0_x', source = stop_line_0, line_width = 5, line_color = 'red', line_dash = 'dashed', legend_label = 'stop_line')
    fig1.line('stop_line_1_y', 'stop_line_1_x', source = stop_line_1, line_width = 5, line_color = 'red', line_dash = 'dashed', legend_label = 'stop_line')
    fig1.line('stop_line_2_y', 'stop_line_2_x', source = stop_line_2, line_width = 5, line_color = 'red', line_dash = 'dashed', legend_label = 'stop_line')
    fig1.line('stop_line_3_y', 'stop_line_3_x', source = stop_line_3, line_width = 5, line_color = 'red', line_dash = 'dashed', legend_label = 'stop_line')
    fig1.line('stop_line_4_y', 'stop_line_4_x', source = stop_line_4, line_width = 5, line_color = 'red', line_dash = 'dashed', legend_label = 'stop_line')

    f66 = fig1.patch('zebra_crossing_line_0_y', 'zebra_crossing_line_0_x', source = zebra_crossing_line_0, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_1_y', 'zebra_crossing_line_1_x', source = zebra_crossing_line_1, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_2_y', 'zebra_crossing_line_2_x', source = zebra_crossing_line_2, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_3_y', 'zebra_crossing_line_3_x', source = zebra_crossing_line_3, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_4_y', 'zebra_crossing_line_4_x', source = zebra_crossing_line_4, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_5_y', 'zebra_crossing_line_5_x', source = zebra_crossing_line_5, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_6_y', 'zebra_crossing_line_6_x', source = zebra_crossing_line_6, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_7_y', 'zebra_crossing_line_7_x', source = zebra_crossing_line_7, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_8_y', 'zebra_crossing_line_8_x', source = zebra_crossing_line_8, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_9_y', 'zebra_crossing_line_9_x', source = zebra_crossing_line_9, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_10_y', 'zebra_crossing_line_10_x', source = zebra_crossing_line_10, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')
    fig1.patch('zebra_crossing_line_11_y', 'zebra_crossing_line_11_x', source = zebra_crossing_line_11, line_width = 1, fill_color = "lavender", fill_alpha = 0.5, line_color = 'black', legend_label = 'zebra_crossing_line')

  fig_dashed_line = fig1.multi_line('lines_y_vec', 'lines_x_vec', source = data_lane_dashed_line, line_width = 2.0, line_color = 'white', hover_line_color = "firebrick", line_dash = 'dashed', legend_label = 'lane_line')
  fig_solid_line = fig1.multi_line('lines_y_vec', 'lines_x_vec', source = data_lane_solid_line, line_width = 2.0, line_color = 'white', hover_line_color = "firebrick", line_dash = 'solid', legend_label = 'lane_line')
  fig_virtual_line = fig1.multi_line('lines_y_vec', 'lines_x_vec', source = data_lane_virtual_line, line_width = 2.0, line_color = 'deepskyblue', hover_line_color = "firebrick", selection_line_color = "firebrick", line_dash = 'dotted', legend_label = 'lane_line')

  f81 = fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_lat, fill_color = "violet", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_lat')
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj, fill_color = "palegreen", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj',visible = False)
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_raw, fill_color = "deepskyblue", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_raw',visible = False)
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_mpc, fill_color = "salmon", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_mpc',visible = False)
  fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig_init_point = fig1.circle('init_pos_point_y', 'init_pos_point_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "deepskyblue", fill_alpha = 1, legend_label = 'init_state')
  fig1.circle('ego_pos_compensation_y', 'ego_pos_compensation_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "purple", fill_alpha = 1, legend_label = 'ego_pos_compensation')
  fig_ego_point = fig1.circle('ego_pos_point_y', 'ego_pos_point_x', source = data_ego_pos_point, radius = 0.1, line_width = 2,  line_color = 'purple', line_alpha = 1, fill_alpha = 1, legend_label = 'ego_pos_point')
  if is_vis_merge_point:
    fig1.circle('merge_point_y', 'merge_point_x', source = data_merge_point, radius = 0.2, line_width = 3,  line_color = 'orange', line_alpha = 1, fill_color = "red", fill_alpha = 1, legend_label = 'merge_point')
    fig1.circle('macroeconomic_decider_merge_point_y', 'macroeconomic_decider_merge_point_x', source = macroeconomic_decider_data_merge_point, radius = 0.2, line_width = 3,  line_color = 'black', line_alpha = 1, fill_color = "red", fill_alpha = 1, legend_label = 'cent_line_merge_pt')
    fig1.circle('boundary_line_merge_point_y', 'boundary_line_merge_point_x', source = boundary_line_merge_point, radius = 0.2, line_width = 3,  line_color = 'red', line_alpha = 1, fill_color = "blue", fill_alpha = 1, legend_label = 'bound_line_merge_pt')
  fig1.line('ego_yb', 'ego_xb', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.line('ego_yb', 'ego_xb', source = origin_data_ego, line_width = 1, line_color = 'orange', line_dash = 'dashed', legend_label = 'origin_ego_pos')
  fig1.text('text_yn', 'text_xn', text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'car')

  fig1.line('init_pos_line_y', 'init_pos_line_x', source = data_init_line, line_width = 3, line_color = 'purple', line_dash = 'solid', legend_label = 'init_point_line', visible = False)

  fig1.triangle_pin('lon_collision_object_position_y', 'lon_collision_object_position_x', source = data_lon_collision_object_position, size = 25, line_width = 4.5, line_alpha = 1,line_color = 'black', fill_color = "orange", fill_alpha = 1, legend_label = 'lon_collision_object_pos')

  if is_vis_map:
    for i in range (len(ehr_data_lanes)):
      keyy = 'ehr_line_{}_y'.format(i)
      keyx = 'ehr_line_{}_x'.format(i)
      fig1.line(keyy,keyx,source = ehr_data_lanes[i], line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ehr_center_lane')

    for i in range(len(ehr_road_boundary_lanes)):
      keyy = 'ehr_road_boundary_{}_y'.format(i)
      keyx = 'ehr_road_boundary_{}_x'.format(i)
      fig1.line(keyy,keyx,source = ehr_road_boundary_lanes[i], line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ehr_road_boundary')

    for i in range(len(ehr_lane_boundary_lanes)):
      keyy = 'ehr_lane_boundary_{}_y'.format(i)
      keyx = 'ehr_lane_boundary_{}_x'.format(i)
      fig1.line(keyy,keyx,source = ehr_lane_boundary_lanes[i], line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'ehr_lane_boundary')
  if is_vis_sdmap:
    fig1.line('data_sdmap_road_line_y','data_sdmap_road_line_x',source = data_sdmap_road_line, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'sdmap_road_line')
    fig1.circle('data_sdmap_ramp_line_y', 'data_sdmap_ramp_line_x', source = data_sdmap_ramp_line, radius = 0.3, fill_color="red", line_color='red', legend_label = 'ramp_segment')
    fig1.circle('data_sdmap_inlink_y', 'data_sdmap_inlink_x', source = data_sdmap_inlink, radius = 0.3, fill_color="green", line_color='green', legend_label = 'inlink_segment')
    fig1.circle('data_sdmap_outlink_y', 'data_sdmap_outlink_x', source = data_sdmap_outlink, radius = 0.3, fill_color="yellow", line_color='yellow', legend_label = 'outlink_segment')

  fig1.line('center_line_0_y', 'center_line_0_x', source = data_center_line_0, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig1.line('center_line_1_y', 'center_line_1_x', source = data_center_line_1, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig1.line('center_line_2_y', 'center_line_2_x', source = data_center_line_2, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig1.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')

  fig1.circle('center_line_topo_0_y', 'center_line_topo_0_x', source = data_center_line_topo_0, line_width = 2, line_color = 'red', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line_topo')
  fig1.circle('center_line_topo_1_y', 'center_line_topo_1_x', source = data_center_line_topo_1, line_width = 2, line_color = 'red', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line_topo')
  fig1.circle('center_line_topo_2_y', 'center_line_topo_2_x', source = data_center_line_topo_2, line_width = 2, line_color = 'red', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line_topo')
  fig1.circle('center_line_topo_3_y', 'center_line_topo_3_x', source = data_center_line_topo_3, line_width = 1, line_color = 'red', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line_topo')
  fig1.circle('center_line_topo_4_y', 'center_line_topo_4_x', source = data_center_line_topo_4, line_width = 1, line_color = 'red', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line_topo')

  if is_vis_lane_mark:
    fig1.circle('text_yn_0', 'text_xn_0', source = lane_mark_data_0, radius = 0.8, line_width = 3,  line_color = 'green', line_alpha = 1, fill_color = "blue", fill_alpha = 1, legend_label = 'lane_mark_point')
    fig1.circle('text_yn_1', 'text_xn_1', source = lane_mark_data_1, radius = 0.8, line_width = 3,  line_color = 'green', line_alpha = 1, fill_color = "blue", fill_alpha = 1, legend_label = 'lane_mark_point')
    fig1.circle('text_yn_2', 'text_xn_2', source = lane_mark_data_2, radius = 0.8, line_width = 3,  line_color = 'green', line_alpha = 1, fill_color = "blue", fill_alpha = 1, legend_label = 'lane_mark_point')
    fig1.circle('text_yn_3', 'text_xn_3', source = lane_mark_data_3, radius = 0.8, line_width = 3,  line_color = 'green', line_alpha = 1, fill_color = "blue", fill_alpha = 1, legend_label = 'lane_mark_point')
    fig1.circle('text_yn_4', 'text_xn_4', source = lane_mark_data_4, radius = 0.8, line_width = 3,  line_color = 'green', line_alpha = 1, fill_color = "blue", fill_alpha = 1, legend_label = 'lane_mark_point')
    fig1.text('lane_mark_loc_y_0', 'lane_mark_loc_x_0', text = 'lane_mark_0' ,source = lane_mark_data_0, text_color="firebrick", text_align="center", text_font_size="20pt", legend_label = 'lane_mark')
    fig1.text('lane_mark_loc_y_1', 'lane_mark_loc_x_1', text = 'lane_mark_1' ,source = lane_mark_data_1, text_color="firebrick", text_align="center", text_font_size="20pt", legend_label = 'lane_mark')
    fig1.text('lane_mark_loc_y_2', 'lane_mark_loc_x_2', text = 'lane_mark_2' ,source = lane_mark_data_2, text_color="firebrick", text_align="center", text_font_size="20pt", legend_label = 'lane_mark')
    fig1.text('lane_mark_loc_y_3', 'lane_mark_loc_x_3', text = 'lane_mark_3' ,source = lane_mark_data_3, text_color="firebrick", text_align="center", text_font_size="20pt", legend_label = 'lane_mark')
    fig1.text('lane_mark_loc_y_4', 'lane_mark_loc_x_4', text = 'lane_mark_4' ,source = lane_mark_data_4, text_color="firebrick", text_align="center", text_font_size="20pt", legend_label = 'lane_mark')

  fig1.line('fix_lane_y', 'fix_lane_x', source = data_fix_lane, line_width = 1, line_color = 'red', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'fix_lane')
  fig1.line('target_lane_y', 'target_lane_x', source = data_target_lane, line_width = 1, line_color = 'orange', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'taget_lane')
  fig1.line('origin_lane_y', 'origin_lane_x', source = data_origin_lane, line_width = 1, line_color = 'black', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'origin_lane')
  fig1.patches('obstacles_y', 'obstacles_x', source = data_fus_obj, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.4, legend_label = 'obj')
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_fus_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'fus_obj_info')

  fig1.patches('obstacles_y', 'obstacles_x', source = data_snrd_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.5, legend_label = 'snrd',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_snrd_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'snrd_info',visible = False)

  if is_vis_rdg_obj:
    fig1.patches('obstacles_y', 'obstacles_x', source = data_rdg_obj, fill_color = "orange", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'rdg_obj',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_rdg_obj, text_color="orange", text_align="center", text_font_size="10pt", legend_label = 'rdg_info',visible = False)

  if is_vis_me_obj:
    fig1.patches('obstacles_y', 'obstacles_x', source = data_me_obj, fill_color = "maroon", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'me_obj',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_me_obj, text_color="maroon", text_align="center", text_font_size="10pt", legend_label = 'me_info',visible = False)

  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning_lat, line_width = 5, line_color = 'violet', line_dash = 'solid', line_alpha = 0.6, legend_label = 'lat plan')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning_raw, line_width = 5, line_color = 'deepskyblue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'raw plan')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_0, radius = 0.03, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_1, radius = 0.03, line_width = 1,  line_color = 'blue', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_2, radius = 0.03, line_width = 1,  line_color = 'orange', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_3, radius = 0.03, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_4, radius = 0.03, line_width = 1,  line_color = 'purple', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')

  if is_vis_radar:
    fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fm_obj, fill_color = "green", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fm_obj',visible = False)
    fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fl_obj, fill_color = "red", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fl_obj',visible = False)
    fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fr_obj, fill_color = "blue", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fr_obj',visible = False)
    fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_rl_obj, fill_color = "yellow", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_rl_obj',visible = False)
    fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_rr_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_rr_obj',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fm_obj, text_color="palegreen", text_align="center", text_font_size="10pt", legend_label = 'radar_fm_info',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fl_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'radar_fl_info',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fr_obj, text_color="blue", text_align="center", text_font_size="10pt", legend_label = 'radar_fr_info',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_rl_obj, text_color="yellow", text_align="center", text_font_size="10pt", legend_label = 'radar_rl_info',visible = False)
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_rr_obj, text_color="black", text_align="center", text_font_size="10pt", legend_label = 'radar_rr_info',visible = False)

  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_0, radius = 0.3, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_1, radius = 0.3, line_width = 1,  line_color = 'blue', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_2, radius = 0.3, line_width = 1,  line_color = 'orange', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_3, radius = 0.3, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_4, radius = 0.3, line_width = 1,  line_color = 'purple', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.patches('prediction_obs_y', 'prediction_obs_x', source = data_prediction_0, fill_color = "grey", fill_alpha = 0.15, line_width = 1,  line_color = 'red', line_alpha = 1, legend_label = 'prediction')
  fig1.patches('prediction_obs_y', 'prediction_obs_x', source = data_prediction_1, fill_color = "grey", fill_alpha = 0.15, line_width = 1,  line_color = 'blue', line_alpha = 1, legend_label = 'prediction')
  fig1.patches('prediction_obs_y', 'prediction_obs_x', source = data_prediction_2, fill_color = "grey", fill_alpha = 0.15, line_width = 1,  line_color = 'orange', line_alpha = 1, legend_label = 'prediction')
  fig1.patches('prediction_obs_y', 'prediction_obs_x', source = data_prediction_3, fill_color = "grey", fill_alpha = 0.15, line_width = 1,  line_color = 'black', line_alpha = 1, legend_label = 'prediction')
  fig1.patches('prediction_obs_y', 'prediction_obs_x', source = data_prediction_4, fill_color = "grey", fill_alpha = 0.15, line_width = 1,  line_color = 'purple', line_alpha = 1, legend_label = 'prediction')
  if is_vis_hpp:
    fig1.circle('trace_start_y', 'trace_start_x', source = data_map_key_point, radius = 0.3, line_width = 1,  line_color = 'black', line_alpha = 1, fill_color = "green", fill_alpha = 1, legend_label = 'ehr_start')
    fig1.circle('trace_end_y', 'trace_end_x', source = data_map_key_point, radius = 0.3, line_width = 1,  line_color = 'black', line_alpha = 1, fill_color = "red", fill_alpha = 1, legend_label = 'ehr_end')
    fig1.patches('parking_space_y', 'parking_space_x', source = data_parking_space, fill_color = "grey", fill_alpha = 0.15, line_color = "green", line_width = 3, line_alpha = 0.4, legend_label = 'parking_space')
    # fig1.text('parking_space_center_y', 'parking_space_center_x', text = 'parking_space_id_vec', source = data_parking_space_text, text_color="black", text_align="center", text_font_size="10pt", legend_label = 'parking_space_id', visible = False)
    fig1.patches('parking_slot_y', 'parking_slot_x', source = data_parking_slot, fill_color = "turquoise", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'parking slot')
    fig1.text('pos_y', 'pos_x', text = 'parking_slot_label' ,source = data_parking_slot, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'slot_info')
    fig1.patches('parking_slot_y', 'parking_slot_x', source = data_release_slot, fill_color = "turquoise", fill_alpha = 0.8, line_color = "black", line_width = 1, legend_label = 'parking slot')
    fig1.text('pos_y', 'pos_x', text = 'parking_slot_label' ,source = data_release_slot, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'slot_info')
    fig1.patches('parking_slot_y', 'parking_slot_x', source = data_plan_release_slot, fill_color = "orange", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'parking slot')
    fig1.text('pos_y', 'pos_x', text = 'parking_slot_label' ,source = data_plan_release_slot, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'slot_info')
    fig1.patches('parking_slot_y', 'parking_slot_x', source = data_select_parking_slot, fill_color = "blue", fill_alpha = 0.3, line_color = "black", line_width = 2, legend_label = 'parking slot')
    fig1.text('pos_y', 'pos_x', text = 'parking_slot_label' ,source = data_select_parking_slot, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'slot_info')
    fig_fus_obj_polygon = fig1.patches('polygon_y', 'polygon_x', source = data_fus_obj, fill_color = "grey", fill_alpha = 0.15, line_color = "red", line_width = 3, line_alpha = 0.4, legend_label = 'obs polygon')
    fig_fus_occ_obj_polygon = fig1.patches('polygon_y', 'polygon_x', source = data_fus_occ_obj, fill_color = "grey", fill_alpha = 0.15, line_color = "red", line_width = 3, line_alpha = 0.4, legend_label = 'obs polygon')
    fig1.patches('obstacles_y', 'obstacles_x', source = data_fus_occ_obj, fill_color = "chocolate", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'occ obj')
    fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_fus_occ_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'occ_obj_info', visible = False)
    fig1.patches('polygon_obstacle_y', 'polygon_obstacle_x', source = data_polygon_obstacle, fill_color = "grey", fill_alpha = 0.15, line_color = "green", line_width = 3, line_alpha = 0.4, legend_label = 'ehr_obs')
    fig1.text('pos_y', 'pos_x', text = 'polygon_obstacle_label' ,source = data_polygon_obstacle, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'ehr_obs_info', visible = False)
    fig_ehr_polygon = fig1.patches('polygon_y', 'polygon_x', source = data_polygon_obstacle, fill_color = "grey", fill_alpha = 0.15, line_color = "red", line_width = 3, line_alpha = 0.4, legend_label = 'obs polygon')
    fig1.patches('road_mark_y', 'road_mark_x', source = data_road_mark, fill_color = "green", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'ehr_road_mark')
    fig1.patches('speed_bump_y', 'speed_bump_x', source = data_speed_bump, fill_color = "yellow", fill_alpha = 0.3, hatch_color = "black", hatch_alpha = 0.5, hatch_scale = 50.0, hatch_weight = 1.0, hatch_pattern = 'vertical_line', line_color = "black", line_width = 1, legend_label = 'speed bump')
    # fig1.text('pos_y', 'pos_x', text = 'speed_bump_label' ,source = data_speed_bump, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'speed_bump_info', visible = False)
    fig1.multi_line('road_obstacle_y', 'road_obstacle_x', source = data_road_obstacle, line_width = 2, line_color = 'black', line_dash = 'dotted', legend_label = 'ehr_ground_line', visible = False)
    fig_ground_line_polygon = fig1.patches('polygon_y', 'polygon_x', source = data_ground_line, fill_color = "grey", fill_alpha = 0.15, line_color = "red", line_width = 3, line_alpha = 0.4, legend_label = 'obs polygon')
    fig_ground_line = fig1.multi_line('ground_line_y', 'ground_line_x', source = data_ground_line, line_width = 2, line_color = 'green', line_dash = 'dotted', legend_label = 'ground_line')
    fig1.text('pos_y', 'pos_x', text = 'ground_line_label' ,source = data_ground_line_label, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'ground_line_info', visible = False)
    fig1.circle('ground_line_y', 'ground_line_x', source = data_ground_line_point, radius = 0.01, line_width = 1,  line_color = 'green', line_alpha = 1, fill_color = "green", fill_alpha = 1.0, legend_label = 'ground_line')
    fig_ground_line_cluster = fig1.multi_line('ground_line_y', 'ground_line_x', source = data_ground_line_clusters, line_width = 2, line_color = 'red', line_dash = 'dotted', legend_label = 'ground_line_cluster', visible = False)
    fig1.multi_line('ground_line_y', 'ground_line_x', source = data_rdg_ground_line, line_width = 2, line_color = 'black', line_dash = 'dotted', legend_label = 'rdg_ground_line', visible = False)
    fig1.patches('parking_slot_y', 'parking_slot_x', source = data_rdg_parking_slot, fill_color = "green", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'rdg parking slot', visible = False)
    # fig1.text('pos_y', 'pos_x', text = 'parking_slot_label' ,source = data_rdg_parking_slot, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'rdg slot_info', visible = False)
    fig1.patches('obstacles_y', 'obstacles_x', source = data_rdg_general_obj, fill_color = "yellow", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'rdg gobj', visible = False)
    # fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_rdg_general_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'rdg_gobj_info', visible = False)
    fig1.patches('obstacles_y', 'obstacles_x', source = data_rdg_occ_obj, fill_color = "blue", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'rdg occ', visible = False)
    # fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_rdg_occ_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'rdg_occ_info', visible = False)
    fig1.patches('ground_mark_y', 'ground_mark_x', source = data_rdg_ground_mark, fill_color = "white", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'rdg ground mark', visible = False)
    fig1.text('pos_y', 'pos_x', text = 'ground_mark_label' ,source = data_rdg_ground_mark, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'rdg ground mark', visible = False)

  hover1_1 = HoverTool(renderers=[fig_init_point], tooltips=[('init pos x', '@init_pos_point_x'), ('init pos y', '@init_pos_point_y'), ('init pos theta', '@init_pos_point_theta'),
                                                                ('lat init x', '@init_state_x'), ('lat init y', '@init_state_y'), ('lat init theta', '@init_state_theta'),
                                                                ('lat init delta', '@init_state_delta'), ('lon init s', '@init_state_s'), ('lon init v', '@init_state_v'),
                                                                ('lon init a', '@init_state_a'), ('replan status', '@replan_status')])
  hover1_2 = HoverTool(renderers=[fig_ego_point], tooltips=[('ego pos x', '@ego_pos_point_x'), ('ego pos y', '@ego_pos_point_y'), ('ego pos theta', '@ego_pos_point_theta')])
  # hover1_3 = HoverTool(renderers=[fig1.renderers[51]], tooltips=[('index', '$index')])
  # hover1_4 = HoverTool(renderers=[fig1.renderers[52]], tooltips=[('index', '$index'), ('s', '@plan_traj_s')])
  # hover1_5 = HoverTool(renderers=[fig1.renderers[53]], tooltips=[('index', '$index'), ('s', '@plan_traj_s')])
  # hover1_6 = HoverTool(renderers=[fig1.renderers[59]], tooltips=[('index', '$index')])
  # hover1_7 = HoverTool(renderers=[fig1.renderers[69]], tooltips=[('index', '$index')])
  hover1_9 = HoverTool(renderers=[fig_dashed_line], tooltips=[('relative_id', '@relative_id_vec')])
  hover1_10 = HoverTool(renderers=[fig_solid_line], tooltips=[('relative_id', '@relative_id_vec')])
  hover1_11 = HoverTool(renderers=[fig_virtual_line], tooltips=[('relative_id', '@relative_id_vec')])

  fig1.add_tools(hover1_1)
  fig1.add_tools(hover1_2)
  # fig1.add_tools(hover1_3)
  # fig1.add_tools(hover1_4)
  # fig1.add_tools(hover1_5)
  # fig1.add_tools(hover1_6)
  # fig1.add_tools(hover1_7)
  fig1.add_tools(hover1_9)
  fig1.add_tools(hover1_10)
  fig1.add_tools(hover1_11)
  if is_vis_hpp:
    # hover1_12 = HoverTool(renderers=[fig_ground_line], tooltips=[('id', '@ground_line_id')])
    # hover1_13 = HoverTool(renderers=[fig_ground_line_cluster], tooltips=[('id', '@ground_line_id')])
    hover1_14 = HoverTool(renderers=[fig_fus_obj_polygon], tooltips=[('id', '@obs_id')])
    hover1_15 = HoverTool(renderers=[fig_fus_occ_obj_polygon], tooltips=[('id', '@obs_id')])
    hover1_16 = HoverTool(renderers=[fig_ehr_polygon], tooltips=[('id', '@polygon_id')])
    hover1_17 = HoverTool(renderers=[fig_ground_line_polygon], tooltips=[('id', '@ground_line_id')])
    # fig1.add_tools(hover1_12)
    # fig1.add_tools(hover1_13)
    fig1.add_tools(hover1_14)
    fig1.add_tools(hover1_15)
    fig1.add_tools(hover1_16)
    fig1.add_tools(hover1_17)

  # tap1 = TapTool(renderers=[fig_virtual_line])
  # fig1.add_tools(tap1)
  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data


def init_basic_figure_plot(fig, local_view_data):
  return fig
