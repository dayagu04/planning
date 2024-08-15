from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
from lib.load_ros_bag import LoadRosbag, g_is_display_enu, is_match_planning, is_vis_map, is_bag_main, is_vis_sdmap
import numpy as np

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
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS, CheckboxGroup
from google.protobuf.json_format import MessageToJson

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
Max_line_size = 200
Road_boundary_max_line_size = 50
Lane_boundary_max_line_size = 300
Max_sdmap_segment_size = 100

first_frame_num = 0

def update_local_view_data(fig1, bag_loader, bag_time, local_view_data):
  # bag_time = 1.2
  ### step 1: 时间戳对齐
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
  ctrl_debug_msg = find_nearest(bag_loader.ctrl_debug_msg, bag_time)
  ctrl_debug_json_msg = find_nearest(bag_loader.ctrl_debug_msg, bag_time,True)
  ehr_static_map_msg = find_nearest(bag_loader.ehr_static_map_msg, bag_time)
  ehr_sd_map_msg = find_nearest(bag_loader.ehr_sd_map_msg, bag_time)
  ehr_parking_map_msg = find_nearest(bag_loader.ehr_parking_map_msg, bag_time)
  ground_line_msg = find_nearest(bag_loader.ground_line_msg, bag_time)
  planning_hmi_msg = find_nearest(bag_loader.planning_hmi_msg, bag_time)
  rdg_lane_lines_msg = find_nearest(bag_loader.rdg_lane_lines_msg, bag_time)

  input_topic_timestamp = plan_debug_msg.input_topic_timestamp
  fusion_object_timestamp = input_topic_timestamp.fusion_object
  fusion_road_timestamp = input_topic_timestamp.fusion_road
  if is_bag_main:
    localization_timestamp = input_topic_timestamp.localization_estimate
  else:
    localization_timestamp = input_topic_timestamp.localization
  # prediction_timestamp = input_topic_timestamp.prediction
  # vehicle_service_timestamp = input_topic_timestamp.vehicle_service
  # control_output_timestamp = input_topic_timestamp.control_output
  # ehr_parking_map_timestamp = input_topic_timestamp.ehr_parking_map
  # ground_line_timestamp = input_topic_timestamp.ground_line
  #
  if is_match_planning:
    fus_msg_tmp = find(bag_loader.fus_msg, fusion_object_timestamp)
    if fus_msg_tmp != None:
      fus_msg = fus_msg_tmp
    road_msg_tmp = find(bag_loader.road_msg, fusion_road_timestamp)

    if road_msg_tmp != None:
      road_msg = road_msg_tmp
      rdg_lane_lines_msg_tmp = find(bag_loader.rdg_lane_lines_msg, road_msg_tmp.isp_timestamp)
      if rdg_lane_lines_msg_tmp != None:
        rdg_lane_lines_msg = rdg_lane_lines_msg_tmp
        print('find rdg_lane_lines_msg success')
      else :
        print('find rdg_lane_lines_msg fail')
    loc_msg_tmp = find(bag_loader.loc_msg, localization_timestamp)
    if loc_msg_tmp != None:
      loc_msg = loc_msg_tmp


  local_view_data['data_msg']['plan_msg'] = plan_msg
  local_view_data['data_msg']['plan_debug_msg'] = plan_debug_msg
  local_view_data['data_msg']['planning_hmi_msg'] = planning_hmi_msg
  local_view_data['data_msg']['loc_msg'] = loc_msg
  local_view_data['data_msg']['plan_debug_json_msg'] = plan_debug_json_msg
  local_view_data['data_msg']['ctrl_msg'] = ctrl_msg
  local_view_data['data_msg']['ctrl_debug_msg'] = ctrl_debug_msg
  local_view_data['data_msg']['ctrl_debug_json_msg'] = ctrl_debug_json_msg

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

  is_enu_to_car = False
  if plan_msg != None:
    if plan_msg.trajectory.trajectory_type == 0: # 实时轨迹
      is_enu_to_car = False
    else:
      is_enu_to_car = True

  # step 3: 加载车道线信息
  if bag_loader.road_msg['enable'] == True:
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

    for i in range(20):
      try:
        if line_info_list[i]['type'] == ['dashed']:
          fig1.renderers[0 + i].glyph.line_dash = 'dashed'
        elif line_info_list[i]['type'] == ['dashdot']:
          fig1.renderers[0 + i].glyph.line_dash = 'dashdot'
          print("虚拟线")
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
    if bag_loader.rdg_lane_lines_msg['enable'] == True:
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
  if bag_loader.plan_debug_msg['enable'] == True:
    try:
      global first_frame_num
      if bag_time <= 0.1:
        first_frame_num = int(plan_debug_msg.frame_info.frame_num)
      print("planning debug info:", int(plan_debug_msg.frame_info.frame_num) - first_frame_num)
    except:
      pass
    lat_behavior_common = plan_debug_msg.lat_behavior_common
    environment_model_info = plan_debug_msg.environment_model_info
    current_lane_virtual_id = environment_model_info.currrent_lane_vitual_id
    fix_lane_ralative_id = lat_behavior_common.fix_lane_virtual_id - current_lane_virtual_id
    target_lane_ralative_id = lat_behavior_common.target_lane_virtual_id - current_lane_virtual_id
    origin_lane_ralative_id = lat_behavior_common.origin_lane_virtual_id - current_lane_virtual_id
    for i in range(10):
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

    if loc_mode == 2:
      plan_debug_json = plan_debug_json_msg
      plan_traj_x = plan_debug_json["assembled_x"]
      plan_traj_y = plan_debug_json["assembled_y"]
      plan_traj_theta = plan_debug_json["assembled_theta"]
      plan_traj_s = plan_debug_json["traj_s_vec"]
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
      lon_init_state = plan_debug_msg.longitudinal_motion_planning_input.init_state
      init_state_s = lon_init_state.s
      init_state_v = lon_init_state.v
      init_state_a = lon_init_state.a
      replan_status = plan_debug_json_msg["replan_status"]
      init_pos_point_x = []
      init_pos_point_y = []
      ego_pos_compensation_x_ = []
      ego_pos_compensation_y_ = []
      init_pos_point_theta = []
      if g_is_display_enu:
        init_pos_point_x.append(init_state_x)
        init_pos_point_y.append(init_state_y)
        init_pos_point_theta.append(init_state_theta)
      else:
        init_pos_point_x, init_pos_point_y = coord_tf.global_to_local([init_state_x], [init_state_y])
        ego_pos_compensation_x_, ego_pos_compensation_y_ = coord_tf.global_to_local([ego_pos_compensation_x], [ego_pos_compensation_y])
        temp_theta = init_state_theta - loc_msg.orientation.euler_boot.yaw
        init_pos_point_theta.append(temp_theta)

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
    obstacles_info_all = load_obstacle_params(fus_msg, is_enu_to_car, loc_msg, environment_model_info)
    local_view_data['data_fus_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })
    local_view_data['data_snrd_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
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
  # if bag_loader.plan_msg['enable'] == True and loc_mode == 2:
  if bag_loader.plan_msg['enable'] == True and loc_mode == 2:
    trajectory = plan_msg.trajectory
    plan_traj_s = []
    for i in range(len(trajectory.trajectory_points)):
      plan_traj_s.append(trajectory.trajectory_points[i].distance)
    if trajectory.trajectory_type == 0: # 实时轨迹
      try:
        planning_polynomial = trajectory.target_reference.polynomial
        plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
        plan_traj_s = [0] * len(plan_traj_x)
      except:
        plan_traj_x, plan_traj_y, plan_traj_s = [], [], []
    else:
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
    try:
      for i in range(5):
        local_view_data['data_prediction_' + str(i)].data.update({
          'prediction_y' : [],
          'prediction_x' : [],
        })
      # 定位的选择需要修改
      prediction_dict = load_prediction_objects(prediction_msg.prediction_obstacle_list, loc_msg, g_is_display_enu)
      for i in range(5):
        local_view_data['data_prediction_' + str(i)].data.update({
          'prediction_y' : prediction_dict[i]['y'],
          'prediction_x' : prediction_dict[i]['x'],
        })
    except:
      print("prediction error")
      pass

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    mpc_dx, mpc_dy, mpc_dtheta = generate_control(ctrl_msg, loc_msg, g_is_display_enu)
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

    if bag_loader.planning_hmi_msg['enable'] ==True:
      noa_output_info_msg = planning_hmi_msg.noa_output_info
      print("dis to ramp:",noa_output_info_msg.dis_to_ramp)
      print("dis to split:",noa_output_info_msg.dis_to_split)
      print("dis to merge:",noa_output_info_msg.dis_to_merge)

    print("ehr static map timestamp:",ehr_static_map_msg.header)
    print("road_map.lanes len:",len(ehr_static_map_msg.road_map.lanes))
    #load center line

    ehr_line_info_list = ehr_load_center_lane_lines(ehr_static_map_msg.road_map.lanes,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Max_line_size)
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
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Road_boundary_max_line_size)
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
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Lane_boundary_max_line_size)
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

  # 加载ehr_parking_map
  if bag_loader.ehr_parking_map_msg['enable'] == True:
    parking_space_boxes_x, parking_space_boxes_y, road_mark_boxes_x, road_mark_boxes_y = generate_ehr_parking_map(ehr_parking_map_msg, loc_msg, g_is_display_enu)
    local_view_data['data_parking_space'].data.update({
      'parking_space_x' : parking_space_boxes_x,
      'parking_space_y' : parking_space_boxes_y,
    })
    local_view_data['data_road_mark'].data.update({
      'road_mark_x' : road_mark_boxes_x,
      'road_mark_y' : road_mark_boxes_y,
    })

    loc_msg = loc_msg
    parking_space_x_vec, parking_space_y_vec, parking_space_id_vec = hpp_generate_ehr_parking_map(ehr_parking_map_msg, loc_msg, g_is_display_enu)
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

  if bag_loader.ground_line_msg['enable'] == True:
    groundline_x_vec, groundline_y_vec = generate_ground_line(ground_line_msg, loc_msg, g_is_display_enu)
    local_view_data['data_ground_line'].data.update({
      'ground_line_x' : groundline_x_vec,
      'ground_line_y' : groundline_y_vec,
    })

    ground_line_point_x_vec, ground_line_point_y_vec, groundline_x_vec, groundline_y_vec, groundline_id_vec = generate_ground_line_clusters(ground_line_msg, loc_msg, g_is_display_enu)
    local_view_data['data_ground_line_point'].data.update({
      'ground_line_x' : ground_line_point_x_vec,
      'ground_line_y' : ground_line_point_y_vec,
    })
    local_view_data['data_ground_line_clusters'].data.update({
      'ground_line_x' : groundline_x_vec,
      'ground_line_y' : groundline_y_vec,
      'groundline_id_vec' : groundline_id_vec
    })
  return local_view_data

def load_local_view_figure():
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
  data_text = ColumnDataSource(data = {'vel_ego_text':[], 'text_xn': [],  'text_yn': []})
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



  data_fix_lane = ColumnDataSource(data = {'fix_lane_y':[], 'fix_lane_x':[]})
  data_target_lane = ColumnDataSource(data = {'target_lane_y':[], 'target_lane_x':[]})
  data_origin_lane = ColumnDataSource(data = {'origin_lane_y':[], 'origin_lane_x':[]})
  data_fus_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
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
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
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
                                             'prediction_x':[],})
  data_prediction_1 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_2 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_3 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_4 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_parking_space = ColumnDataSource(data = {'parking_space_y':[],
                                             'parking_space_x':[],})
  data_parking_space_text = ColumnDataSource(data = {'parking_space_center_y':[],
                                                     'parking_space_center_x':[],
                                                     'parking_space_id_vec':[],})
  data_road_mark = ColumnDataSource(data = {'road_mark_y':[],
                                             'road_mark_x':[],})
  data_ground_line = ColumnDataSource(data = {'ground_line_y':[],
                                             'ground_line_x':[],})
  data_ground_line_point = ColumnDataSource(data = {'ground_line_y':[],
                                                    'ground_line_x':[],})
  data_ground_line_clusters = ColumnDataSource(data = {'ground_line_y':[],
                                                       'ground_line_x':[],
                                                       'groundline_id_vec':[],})
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
                     'data_text':data_text, \
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
                     'data_prediction_0' : data_prediction_0 ,\
                     'data_prediction_1' : data_prediction_1 ,\
                     'data_prediction_2' : data_prediction_2 ,\
                     'data_prediction_3' : data_prediction_3 ,\
                     'data_prediction_4' : data_prediction_4 ,\
                     'data_parking_space' : data_parking_space , \
                     'data_parking_space_text' : data_parking_space_text , \
                     'data_road_mark' : data_road_mark , \
                     'data_ground_line' : data_ground_line, \
                     'data_ground_line_point' : data_ground_line_point, \
                     'data_ground_line_clusters' : data_ground_line_clusters, \
                     'data_fus_obj':data_fus_obj, \
                     'data_me_obj':data_me_obj, \
                     'data_rdg_obj':data_rdg_obj, \
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
                     'data_lane_topo_0':data_lane_topo_0,\
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

  fig1.x_range.flipped = True
  # figure plot

  # !!!!!!!!!!!! Important: Do not draw above !!!!!!!!!!!
  f0 = fig1.line('line_0_y', 'line_0_x', source = data_lane_0, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_1_y', 'line_1_x', source = data_lane_1, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_2_y', 'line_2_x', source = data_lane_2, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_3_y', 'line_3_x', source = data_lane_3, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_4_y', 'line_4_x', source = data_lane_4, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_5_y', 'line_5_x', source = data_lane_5, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_6_y', 'line_6_x', source = data_lane_6, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_7_y', 'line_7_x', source = data_lane_7, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_8_y', 'line_8_x', source = data_lane_8, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_9_y', 'line_9_x', source = data_lane_9, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_10_y', 'line_10_x', source = data_lane_10, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_11_y', 'line_11_x', source = data_lane_11, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_12_y', 'line_12_x', source = data_lane_12, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_13_y', 'line_13_x', source = data_lane_13, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_14_y', 'line_14_x', source = data_lane_14, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_15_y', 'line_15_x', source = data_lane_15, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_16_y', 'line_16_x', source = data_lane_16, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_17_y', 'line_17_x', source = data_lane_17, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_18_y', 'line_18_x', source = data_lane_18, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
  fig1.line('line_19_y', 'line_19_x', source = data_lane_19, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane_fusion')
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


  f61 = fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_lat, fill_color = "violet", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_lat')
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj, fill_color = "palegreen", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj',visible = False)
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_raw, fill_color = "deepskyblue", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_raw',visible = False)
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_mpc, fill_color = "salmon", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_mpc',visible = False)
  fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  f66 = fig1.circle('init_pos_point_y', 'init_pos_point_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "deepskyblue", fill_alpha = 1, legend_label = 'init_state')
  fig1.circle('ego_pos_compensation_y', 'ego_pos_compensation_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "purple", fill_alpha = 1, legend_label = 'ego_pos_compensation')
  f68 = fig1.circle('ego_pos_point_y', 'ego_pos_point_x', source = data_ego_pos_point, radius = 0.1, line_width = 2,  line_color = 'purple', line_alpha = 1, fill_alpha = 1, legend_label = 'ego_pos_point')
  fig1.line('ego_yb', 'ego_xb', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.line('ego_yb', 'ego_xb', source = origin_data_ego, line_width = 1, line_color = 'orange', line_dash = 'dashed', legend_label = 'origin_ego_pos')
  fig1.text('text_yn', 'text_xn', text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'car')


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


  fig1.line('fix_lane_y', 'fix_lane_x', source = data_fix_lane, line_width = 1, line_color = 'red', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'fix_lane')
  fig1.line('target_lane_y', 'target_lane_x', source = data_target_lane, line_width = 1, line_color = 'orange', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'taget_lane')
  fig1.line('origin_lane_y', 'origin_lane_x', source = data_origin_lane, line_width = 1, line_color = 'black', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'origin_lane')
  fig1.patches('obstacles_y', 'obstacles_x', source = data_fus_obj, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'obj')
  #if 0:
  fig1.patches('obstacles_y', 'obstacles_x', source = data_rdg_obj, fill_color = "orange", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'rdg_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_me_obj, fill_color = "maroon", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'me_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fm_obj, fill_color = "green", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fm_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fl_obj, fill_color = "red", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fl_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fr_obj, fill_color = "blue", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fr_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_rl_obj, fill_color = "yellow", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_rl_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_rr_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_rr_obj',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_rdg_obj, text_color="orange", text_align="center", text_font_size="10pt", legend_label = 'rdg_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_me_obj, text_color="maroon", text_align="center", text_font_size="10pt", legend_label = 'me_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fm_obj, text_color="palegreen", text_align="center", text_font_size="10pt", legend_label = 'radar_fm_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fl_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'radar_fl_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fr_obj, text_color="blue", text_align="center", text_font_size="10pt", legend_label = 'radar_fr_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_rl_obj, text_color="yellow", text_align="center", text_font_size="10pt", legend_label = 'radar_rl_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_rr_obj, text_color="black", text_align="center", text_font_size="10pt", legend_label = 'radar_rr_info',visible = False)

  fig1.patches('obstacles_y', 'obstacles_x', source = data_snrd_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.5, legend_label = 'snrd')
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_fus_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'fusion_info')
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_snrd_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'snrd_info')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning_lat, line_width = 5, line_color = 'violet', line_dash = 'solid', line_alpha = 0.6, legend_label = 'lat plan')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning_raw, line_width = 5, line_color = 'deepskyblue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'raw plan')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_0, radius = 0.03, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_1, radius = 0.03, line_width = 1,  line_color = 'blue', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_2, radius = 0.03, line_width = 1,  line_color = 'orange', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_3, radius = 0.03, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_4, radius = 0.03, line_width = 1,  line_color = 'purple', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')
  # fig1.circle('prediction_y', 'prediction_x', source = data_prediction_0, radius = 0.3, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  # fig1.circle('prediction_y', 'prediction_x', source = data_prediction_1, radius = 0.3, line_width = 1,  line_color = 'blue', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  # fig1.circle('prediction_y', 'prediction_x', source = data_prediction_2, radius = 0.3, line_width = 1,  line_color = 'orange', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  # fig1.circle('prediction_y', 'prediction_x', source = data_prediction_3, radius = 0.3, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  # fig1.circle('prediction_y', 'prediction_x', source = data_prediction_4, radius = 0.3, line_width = 1,  line_color = 'purple', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  # fig1.patches('parking_space_y', 'parking_space_x', source = data_parking_space, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'parking_space')
  # fig1.text('parking_space_center_y', 'parking_space_center_x', text = 'parking_space_id_vec' ,source = data_parking_space_text, text_color="black", text_align="center", text_font_size="10pt", legend_label = 'parking_space_id')
  # fig1.patches('road_mark_y', 'road_mark_x', source = data_road_mark, fill_color = "green", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'road_mark')
  # fig1.multi_line('ground_line_y', 'ground_line_x', source = data_ground_line, line_width = 2, line_color = 'green', line_dash = 'dotted', legend_label = 'ground_line')
  # fig1.circle('ground_line_y', 'ground_line_x', source = data_ground_line_point, radius = 0.01, line_width = 1,  line_color = 'green', line_alpha = 1, fill_color = "green", fill_alpha = 1.0, legend_label = 'ground_line')
  # fig1.multi_line('ground_line_y', 'ground_line_x', source = data_ground_line_clusters, line_width = 2, line_color = 'red', line_dash = 'dotted', legend_label = 'ground_line_cluster')

  hover1_1 = HoverTool(renderers=[f66], tooltips=[('init pos x', '@init_pos_point_x'), ('init pos y', '@init_pos_point_y'), ('init pos theta', '@init_pos_point_theta'),
                                                                ('lat init x', '@init_state_x'), ('lat init y', '@init_state_y'), ('lat init theta', '@init_state_theta'),
                                                                ('lat init delta', '@init_state_delta'), ('lon init s', '@init_state_s'), ('lon init v', '@init_state_v'),
                                                                ('lon init a', '@init_state_a'), ('replan status', '@replan_status')])
  hover1_2 = HoverTool(renderers=[f68], tooltips=[('ego pos x', '@ego_pos_point_x'), ('ego pos y', '@ego_pos_point_y'), ('ego pos theta', '@ego_pos_point_theta')])
  hover1_3 = HoverTool(renderers=[fig1.renderers[51]], tooltips=[('index', '$index')])
  hover1_4 = HoverTool(renderers=[fig1.renderers[52]], tooltips=[('index', '$index'), ('s', '@plan_traj_s)')])
  hover1_5 = HoverTool(renderers=[fig1.renderers[53]], tooltips=[('index', '$index'), ('s', '@plan_traj_s)')])
  hover1_6 = HoverTool(renderers=[fig1.renderers[59]], tooltips=[('index', '$index')])
  hover1_7 = HoverTool(renderers=[fig1.renderers[69]], tooltips=[('index', '$index')])
  # hover1_8 = HoverTool(renderers=[fig1.renderers[70]], tooltips=[('id', '@groundline_id_vec')])

  fig1.add_tools(hover1_1)
  fig1.add_tools(hover1_2)
  fig1.add_tools(hover1_3)
  fig1.add_tools(hover1_4)
  fig1.add_tools(hover1_5)
  fig1.add_tools(hover1_6)
  fig1.add_tools(hover1_7)
  # fig1.add_tools(hover1_8)
  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data