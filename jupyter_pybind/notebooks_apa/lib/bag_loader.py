from lib.load_struct import *
from lib.load_json import *
import json
import numpy as np
from cyber_record.record import Record

class LoadCyberbag:
  def __init__(self, path) -> None:
    self.bag = Record(path)
    # loclization msg
    self.loc_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # road msg
    self.road_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # fusion object msg
    self.fus_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # vehicle service msg
    self.vs_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}
    # car pos in local coordinates

    # prediction_msg
    self.prediction_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # planning msg
    self.plan_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # planning debug msg
    self.plan_debug_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # control msg
    self.ctrl_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # control debug msg
    self.ctrl_debug_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # slot msg
    self.slot_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # mobileye lane lines msg
    self.mobileye_lane_lines_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # rdg lane lines msg
    self.rdg_lane_lines_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # mobileye objects msg
    self.mobileye_objects_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # radar_fm object msg
    self.rdg_objects_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # radar_fm object msg
    self.radar_fm_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # radar_fl object msg
    self.radar_fl_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # radar_fr object msg
    self.radar_fr_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # radar_rl object msg
    self.radar_rl_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # radar_rr object msg
    self.radar_rr_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # lidar object msg
    self.lidar_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

  def load_all_data(self, normal_print = True):
    max_time = 0.0
    # load localization msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/localization/ego_pose"):
        # load timestamp
        self.loc_msg['t'].append(msg.header.timestamp / 1e6)
        self.loc_msg['timestamp'].append(msg.header.timestamp)
        self.loc_msg['data'].append(msg)
      self.loc_msg['t'] = [tmp - self.loc_msg['t'][0]  for tmp in self.loc_msg['t']]
      max_time = max(max_time, self.loc_msg['t'][-1])
      if normal_print == True:
        print('loc_msg time:',self.loc_msg['t'][-1])
      if len(self.loc_msg['t']) > 0:
        self.loc_msg['enable'] = True
      else:
        self.loc_msg['enable'] = False
    except:
      self.loc_msg['enable'] = False
      print('missing /iflytek/localization/ego_pose !!!')

    # load road_fusion msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/road_fusion"):
        self.road_msg['t'].append(msg.header.timestamp / 1e6)
        self.road_msg['timestamp'].append(msg.header.timestamp)
        self.road_msg['data'].append(msg)
      self.road_msg['t'] = [tmp - self.road_msg['t'][0]  for tmp in self.road_msg['t']]
      if normal_print == True:
        print('road_msg time:',self.road_msg['t'][-1])
      if len(self.road_msg['t']) > 0:
        self.road_msg['enable'] = True
      else:
        self.road_msg['enable'] = False
    except:
      self.road_msg['enable'] = False
      print('missing /iflytek/fusion/road_fusion topic !!!')

    # load fusion objects msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/objects"):
        self.fus_msg['t'].append(msg.header.timestamp / 1e6)
        self.fus_msg['timestamp'].append(msg.header.timestamp)
        self.fus_msg['data'].append(msg)
      self.fus_msg['t'] = [tmp - self.fus_msg['t'][0]  for tmp in self.fus_msg['t']]
      if normal_print == True:
        print('fus_msg time:',self.fus_msg['t'][-1])
      if len(self.fus_msg['t']) > 0:
        self.fus_msg['enable'] = True
      else:
        self.fus_msg['enable'] = False
    except:
      self.fus_msg['enable'] = False
      print('missing /iflytek/fusion/objects !!!')

    # load vehicle service msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/vehicle_service"):
        self.vs_msg['t'].append(msg.header.timestamp / 1e6)
        self.vs_msg['timestamp'].append(msg.header.timestamp)
        self.vs_msg['data'].append(msg)
      self.vs_msg['t'] = [tmp - self.vs_msg['t'][0]  for tmp in self.vs_msg['t']]
      self.vs_msg['enable'] = True
      if normal_print == True:
        print('vs time:',self.vs_msg['t'][-1])
      max_time = max(max_time, self.vs_msg['t'][-1])
      if len(self.vs_msg['t']) > 0:
        self.vs_msg['enable'] = True
      else:
        self.vs_msg['enable'] = False
    except:
      self.vs_msg['enable'] = False
      print("missing /iflytek/vehicle_service !!!")

    # load planning msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/plan"):
        self.plan_msg['t'].append(msg.meta.header.timestamp / 1e6)
        self.plan_msg['timestamp'].append(msg.meta.header.timestamp)
        self.plan_msg['data'].append(msg)
      self.plan_msg['t'] = [tmp - self.plan_msg['t'][0]  for tmp in self.plan_msg['t']]
      max_time = max(max_time, self.plan_msg['t'][-1])
      if normal_print == True:
        print('plan_msg time:',self.plan_msg['t'][-1])
      if len(self.plan_msg['t']) > 0:
        self.plan_msg['enable'] = True
      else:
        self.plan_msg['enable'] = False
    except:
      self.plan_msg['enable'] = False
      print("missing /iflytek/planning/plan !!!")


    # load prediction msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/prediction/prediction_result"):
        self.prediction_msg['t'].append(msg.header.timestamp / 1e6)
        self.prediction_msg['timestamp'].append(msg.header.timestamp)
        self.prediction_msg['data'].append(msg)
      self.prediction_msg['t'] = [tmp - self.prediction_msg['t'][0]  for tmp in self.prediction_msg['t']]
      self.prediction_msg['enable'] = True
      max_time = max(max_time, self.prediction_msg['t'][-1])
      if normal_print == True:
        print('prediction_msg time:',self.prediction_msg['t'][-1])
      if len(self.prediction_msg['t']) > 0:
        self.prediction_msg['enable'] = True
      else:
        self.prediction_msg['enable'] = False
    except:
      self.prediction_msg['enable'] = False
      print("missing /iflytek/prediction/prediction_result !!!")


    # load planning debug msg
    try:
      json_value_list = ["replan_status", "ego_pos_x", "ego_pos_y", "ego_pos_yaw",
                         'acc_target_high', 'acc_target_low',\
                         "road_radius", 'v_limit_road', 'v_limit_in_turns','v_target', 'v_ego',\
                         'lead_one_id', 'lead_one_dis', 'lead_one_vel', "v_target_lead_one",\
                         'lead_two_id', 'lead_two_dis', 'lead_two_vel', "v_target_lead_two",\
                         'temp_lead_one_id', 'temp_lead_one_dis', 'temp_lead_one_vel', "v_target_temp_lead_one",\
                         'temp_lead_two_id', 'temp_lead_two_dis', 'temp_lead_two_vel', "v_target_temp_lead_two",\
                         'potental_cutin_track_id', 'v_target_potental_cutin', "v_target_cutin",  \
                         'stop_start_state', 'v_target_start_stop', 'STANDSTILL', \
                         "dis_to_ramp", "v_target_ramp",\
                         "VisionLateralBehaviorPlannerCost","VisionLateralMotionPlannerCost","VisionLongitudinalBehaviorPlannerCost",
                         'RealTime_v_ref', 'v_ego', 'gap_v_limit_lc', \
                         "fast_lead_id", "slow_lead_id", "fast_car_cut_in_id", "slow_car_cut_in_id", \
                         "EnvironmentalModelManagerCost", "GeneralPlannerModuleCostTime",
                         "RealTime_desired_distance_rss", "RealTime_desired_distance_calibrate", \
                         "RealTimeLonBehaviorCostTime", "RealTimeLonMotionCostTime", 'RealTimeLateralMotionCostTime']

      json_vector_list = ["raw_refline_x_vec", "raw_refline_y_vec"]

      for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info"):
        self.plan_debug_msg['t'].append(msg.timestamp / 1e6)
        self.plan_debug_msg['timestamp'].append(msg.timestamp)
        self.plan_debug_msg['data'].append(msg)
        try:
          json_struct = json.loads(msg.data_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)

          self.plan_debug_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      self.plan_debug_msg['t'] = [tmp - self.plan_debug_msg['t'][0]  for tmp in self.plan_debug_msg['t']]
      max_time = max(max_time, self.plan_debug_msg['t'][-1])
      if normal_print == True:
        print('plan_debug_msg time:',self.plan_debug_msg['t'][-1])
      if len(self.plan_debug_msg['t']) > 0:
        self.plan_debug_msg['enable'] = True
      else:
        self.plan_debug_msg['enable'] = False
    except:
      self.plan_debug_msg['enable'] = False
      print("missing /iflytek/planning/debug_info !!!")


    # load control msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/control/control_command"):
        self.ctrl_msg['t'].append(msg.header.timestamp / 1e6)
        self.ctrl_msg['timestamp'].append(msg.header.timestamp)
        self.ctrl_msg['data'].append(msg)
      self.ctrl_msg['t'] = [tmp - self.ctrl_msg['t'][0]  for tmp in self.ctrl_msg['t']]
      max_time = max(max_time, self.ctrl_msg['t'][-1])
      if normal_print == True:
        print('ctrl_msg time:',self.ctrl_msg['t'][-1])
      if len(self.ctrl_msg['t']) > 0:
        self.ctrl_msg['enable'] = True
      else:
        self.ctrl_msg['enable'] = False
    except:
      self.ctrl_msg['enable'] = False
      print("missing /iflytek/control/control_command !!!")


    # load control debug msg
    try:
      json_value_list = ["steer_angle_cmd", "steer_angle", "acc_ego", "acc_vel", "vel_ego", "vel_wheel", "wheel_angle_cmd",
        "slope_acc", "vel_cmd",  "vel_raw_cmd","vel_error", "vel_fdbk_out", "vel_raw_error", "vel_ffwd_out", "vel_out",
        "vel_raw_out", "lon_err", "lat_err", "phi_err", "controller_status", "driver_hand_torque", "lat_enable", "lon_enable",
        "lat_mpc_status", "planning_type", "planning_time_offset", "planning_update_flag", "vel_KP_term", "vel_KI_term", "yaw_conti",
        "steer_angle_bias_deg", "steer_bias_deg", "axle_torque", "throttle_brake", "euler_angle_pitch", "euler_angle_yaw" ]

      json_vector_list = ["dx_ref_mpc_vec", "dy_ref_mpc_vec", "dphi_ref_mpc_vec", "dx_mpc_vec", "dy_mpc_vec", "delta_mpc_vec", "dphi_mpc_vec"]

      for topic, msg, t in self.bag.read_messages("/iflytek/control/debug_info"):
        self.ctrl_debug_msg['t'].append(msg.timestamp / 1e6)
        self.ctrl_debug_msg['data'].append(msg)
        try:
          json_struct = json.loads(msg.extra_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)

          self.ctrl_debug_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      self.ctrl_debug_msg['t'] = [tmp - self.ctrl_debug_msg['t'][0]  for tmp in self.ctrl_debug_msg['t']]
      max_time = max(max_time, self.ctrl_debug_msg['t'][-1])
      if normal_print == True:
        print('ctrl_debug_msg time:',self.ctrl_debug_msg['t'][-1])
      if len(self.ctrl_debug_msg['t']) > 0:
        self.ctrl_debug_msg['enable'] = True
      else:
        self.ctrl_debug_msg['enable'] = False
    except:
      self.ctrl_debug_msg['enable'] = False
      print("missing /iflytek/control/debug_info !!!")

    # load slot msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/parking_slot"):
        # load timestamp
        self.slot_msg['t'].append(msg.header.timestamp / 1e6)
        self.slot_msg['timestamp'].append(msg.header.timestamp)
        self.slot_msg['data'].append(msg)
      self.slot_msg['t'] = [tmp - self.slot_msg['t'][0]  for tmp in self.slot_msg['t']]
      max_time = max(max_time, self.slot_msg['t'][-1])
      if normal_print == True:
        print('slot_msg time:',self.slot_msg['t'][-1])
      if len(self.slot_msg['t']) > 0:
        self.slot_msg['enable'] = True
      else:
        self.slot_msg['enable'] = False
    except:
      self.slot_msg['enable'] = False
      print('missing /iflytek/fusion/parking_slot !!!')


    # load mobileye lane_lines msg
    try:
      for topic, msg, t in self.bag.read_messages("/mobileye/camera_perception/lane_lines"):
        self.mobileye_lane_lines_msg['t'].append(msg.header.timestamp / 1e6)
        self.mobileye_lane_lines_msg['timestamp'].append(msg.header.timestamp)
        self.mobileye_lane_lines_msg['data'].append(msg)
      self.mobileye_lane_lines_msg['t'] = [tmp - self.mobileye_lane_lines_msg['t'][0]  for tmp in self.mobileye_lane_lines_msg['t']]
      if normal_print == True:
        print('mobileye_lane_lines_msg time:',self.mobileye_lane_lines_msg['t'][-1])
      if len(self.mobileye_lane_lines_msg['t']) > 0:
        self.mobileye_lane_lines_msg['enable'] = True
      else:
        self.mobileye_lane_lines_msg['enable'] = False
    except:
      self.mobileye_lane_lines_msg['enable'] = False
      print('missing /mobileye/camera_perception/lane_lines !!!')

    # load rdg lane_lines msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/lane_lines"):
        self.rdg_lane_lines_msg['t'].append(msg.header.timestamp / 1e6)
        self.rdg_lane_lines_msg['timestamp'].append(msg.header.timestamp)
        self.rdg_lane_lines_msg['data'].append(msg)
      self.rdg_lane_lines_msg['t'] = [tmp - self.rdg_lane_lines_msg['t'][0]  for tmp in self.rdg_lane_lines_msg['t']]
      if normal_print == True:
        print('rdg_lane_lines_msg time:',self.rdg_lane_lines_msg['t'][-1])
      if len(self.rdg_lane_lines_msg['t']) > 0:
        self.rdg_lane_lines_msg['enable'] = True
      else:
        self.rdg_lane_lines_msg['enable'] = False
    except:
      self.rdg_lane_lines_msg['enable'] = False
      print('missing /iflytek/camera_perception/lane_lines !!!')

    # load mobileye objects msg
    try:
      for topic, msg, t in self.bag.read_messages("/mobileye/camera_perception/objects"):
        self.mobileye_objects_msg['t'].append(msg.header.timestamp / 1e6)
        self.mobileye_objects_msg['timestamp'].append(msg.header.timestamp)
        self.mobileye_objects_msg['data'].append(msg)
      self.mobileye_objects_msg['t'] = [tmp - self.mobileye_objects_msg['t'][0]  for tmp in self.mobileye_objects_msg['t']]
      if normal_print == True:
        print('mobileye_objects_msg time:',self.mobileye_objects_msg['t'][-1])
      if len(self.mobileye_objects_msg['t']) > 0:
        self.mobileye_objects_msg['enable'] = True
      else:
        self.mobileye_objects_msg['enable'] = False
    except:
      self.mobileye_objects_msg['enable'] = False
      print('missing /mobileye/camera_perception/objects !!!')

     # load rdg objects msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/objects"):
        self.rdg_objects_msg['t'].append(msg.header.timestamp / 1e6)
        self.rdg_objects_msg['timestamp'].append(msg.header.timestamp)
        self.rdg_objects_msg['data'].append(msg)
      self.rdg_objects_msg['t'] = [tmp - self.rdg_objects_msg['t'][0]  for tmp in self.rdg_objects_msg['t']]
      if normal_print == True:
        print('rdg_objects_msg time:',self.rdg_objects_msg['t'][-1])
      if len(self.rdg_objects_msg['t']) > 0:
        self.rdg_objects_msg['enable'] = True
      else:
        self.rdg_objects_msg['enable'] = False
    except:
      self.rdg_objects_msg['enable'] = False
      print('missing /iflytek/camera_perception/objects !!!')

    # load lidar objects msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/lidar_objects"):
        self.lidar_msg['t'].append(msg.header.timestamp / 1e6)
        self.lidar_msg['timestamp'].append(msg.header.timestamp)
        self.lidar_msg['data'].append(msg)
      self.lidar_msg['t'] = [tmp - self.lidar_msg['t'][0]  for tmp in self.lidar_msg['t']]
      if normal_print == True:
        print('lidar_msg time:',self.lidar_msg['t'][-1])
      if len(self.lidar_msg['t']) > 0:
        self.lidar_msg['enable'] = True
      else:
        self.lidar_msg['enable'] = False
    except:
      self.lidar_msg['enable'] = False
      print('missing /iflytek/lidar_objects !!!')

    # load radar objects msg
    radar_msg = [self.radar_fm_msg,self.radar_fl_msg,self.radar_fr_msg,self.radar_rl_msg,self.radar_rr_msg]
    topic_list = ["/iflytek/radar_fm_perception_info","/iflytek/radar_fl_perception_info","/iflytek/radar_fr_perception_info","/iflytek/radar_rl_perception_info","/iflytek/radar_rr_perception_info"]
    # for i in range(5):
    #   print(topic[i])
    for i in range(5):
      try:
        if i == 0:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fm_perception_info"):
            radar_msg[i]['t'].append(msg.header.timestamp / 1e6)
            radar_msg[i]['timestamp'].append(msg.header.timestamp)
            radar_msg[i]['data'].append(msg)
        elif i == 1:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fl_perception_info"):
            radar_msg[i]['t'].append(msg.header.timestamp / 1e6)
            radar_msg[i]['timestamp'].append(msg.header.timestamp)
            radar_msg[i]['data'].append(msg)
        elif i == 2:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fr_perception_info"):
            radar_msg[i]['t'].append(msg.header.timestamp / 1e6)
            radar_msg[i]['timestamp'].append(msg.header.timestamp)
            radar_msg[i]['data'].append(msg)
        elif i == 3:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_rl_perception_info"):
            radar_msg[i]['t'].append(msg.header.timestamp / 1e6)
            radar_msg[i]['timestamp'].append(msg.header.timestamp)
            radar_msg[i]['data'].append(msg)
        elif i == 4:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_rr_perception_info"):
            radar_msg[i]['t'].append(msg.header.timestamp / 1e6)
            radar_msg[i]['timestamp'].append(msg.header.timestamp)
            radar_msg[i]['data'].append(msg)
        radar_msg[i]['t'] = [tmp - radar_msg[i]['t'][0]  for tmp in radar_msg[i]['t']]
        if normal_print == True:
          print('radar_msg time:',i,":",radar_msg[i]['t'][-1])
          # print("load message:",i)
        # print(i,'_time:',radar_msg[i]['t'][-1])
        if len(radar_msg[i]['t']) > 0:
          radar_msg[i]['enable'] = True
          # print("true:",i)
        else:
          radar_msg[i]['enable'] = False
      except:
        radar_msg[i]['enable'] = False
        print('missing',topic_list[i])

    return max_time
