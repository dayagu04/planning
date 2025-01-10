from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *

sys.path.append('../../python_proto')
from planning_debug_info_pb2 import *
from control_debug_info_pb2 import *
from ehr_sdmap_pb2 import *

import numpy as np
import time
import ipywidgets
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
from collections import OrderedDict, defaultdict, namedtuple
from functools import  partial
from bokeh.models import ColumnDataSource
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS, CheckboxGroup
from google.protobuf.json_format import MessageToJson
import rosbag
import rospy
import time

is_bag_main = True # False: main分支之前的包   True: main分支之后的包
g_is_display_enu = False # True: local_view显示enu坐标系   False: local_view显示自车坐标系
is_new_loc = False #   True:新定位 False:老定位; 目前是自适应的，有新定位就用新定位，没有就用老定位
is_match_planning = True  #True: topic按照planning接收的时间戳匹配；  False:按最近时间匹配
is_vis_map = False
is_vis_sdmap = True

def get_g_is_display_enu():
  return g_is_display_enu
class LoadRosbag:
  def __init__(self, path) -> None:
    self.bag_path = path
    self.bag = rosbag.Bag(path)
    # loclization msg
    self.loc_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}
    self.origin_loc_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    self.old_loc_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # road msg
    self.road_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # fusion object msg
    self.fus_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # me object msg
    self.mobileye_objects_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}
    # rdg object msg
    self.rdg_objects_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}
    self.lidar_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

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

    # vehicle service msg
    self.vs_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}
    # car pos in local coordinates

    # prediction_msg
    self.prediction_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # planning msg
    self.plan_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # planning debug msg
    self.plan_debug_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # planning debug msg
    self.plan_debug_origin_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # control msg
    self.ctrl_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # control debug msg
    self.ctrl_debug_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # parking fusion msg
    self.fus_parking_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # soc state machine
    self.soc_state_msg = {'t':[], 'data':[], 'json':[], 'enable':[], 'timestamp':[]}

    # ehr static map msg
    self.ehr_static_map_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # ehr sd map msg
    self.ehr_sd_map_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # ehr parking map msg
    self.ehr_parking_map_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # ground_line_msg
    self.ground_line_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # planning hmi msg
    self.planning_hmi_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    self.mobileye_lane_lines_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}
    self.rdg_lane_lines_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[], 'seq':[]}

    self.lane_topo_msg = {'t':[], 'data':[], 'enable':[], 'timestamp':[]}

    # time offset
    t0 = 0

    # is_new_loc
    global is_new_loc
    topics = self.bag.get_type_and_topic_info().topics
    for topic in topics:
      if topic == "/iflytek/localization/egomotion":
        is_new_loc = True

  def load_all_data(self, normal_print = True):
    print('load bag')
    start_time = time.time()
    max_time = 0.0
    t0 = 0
    """ try:
      loc_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/localization/egomotion"):
        loc_msg_dict[msg.msg_header.stamp / 1e6] = msg
      loc_msg_dict = {key: val for key, val in sorted(loc_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in loc_msg_dict.items():
        self.loc_msg['t'].append(t)
        self.loc_msg['data'].append(msg)
        self.loc_msg['timestamp'].append(msg.msg_header.stamp)
      t0 = self.loc_msg['t'][0]
      print("T0 in loc msg:",t0)
      self.loc_msg['t'] = [tmp - t0  for tmp in self.loc_msg['t']]
      max_time = max(max_time, self.loc_msg['t'][-1])
      print('loc_msg time:',self.loc_msg['t'][-1])
      if len(self.loc_msg['t']) > 0:
        self.loc_msg['enable'] = True
      else:
        self.loc_msg['enable'] = False
    except Exception as e:
      self.loc_msg['enable'] = False
      print('missing /iflytek/localization/egomotion !!!') """

    if is_new_loc:
      try:
        loc_msg_dict = {}
        for topic, msg, t in self.bag.read_messages("/iflytek/localization/egomotion"):
          loc_msg_dict[msg.msg_header.stamp / 1e6] = msg
        loc_msg_dict = {key: val for key, val in sorted(loc_msg_dict.items(), key = lambda ele: ele[0])}
        for t, msg in loc_msg_dict.items():
          self.loc_msg['t'].append(t)
          self.loc_msg['data'].append(msg)
          self.loc_msg['timestamp'].append(msg.msg_header.stamp)
        t0 = self.loc_msg['t'][0]
        print("T0 in loc msg:",t0)
        self.loc_msg['t'] = [tmp - t0  for tmp in self.loc_msg['t']]
        max_time = max(max_time, self.loc_msg['t'][-1])
        print('loc_msg time:',self.loc_msg['t'][-1])
        if len(self.loc_msg['t']) > 0:
          self.loc_msg['enable'] = True
        else:
          self.loc_msg['enable'] = False
      except Exception as e:
        self.loc_msg['enable'] = False
        print('missing /iflytek/localization/egomotion !!!')
    else: #加载 旧定位
      class PositionBoot:
        def __init__(self,):
          self.x = 0
          self.y = 0
          self.z = 0
          pass

      class Position:
        def __init__(self,):
          self.position_boot = PositionBoot()
          pass

      class StatusInfo :
        def __init__(self,):
          self.mode = 1
          pass
      class Status :
        def __init__(self,):
          self.status_info = StatusInfo()
          pass
      class EulerBoot:
        def __init__(self,):
          self.yaw = 0
          pass

      class Orientation:
        def __init__(self,):
          pass
          self.euler_boot = EulerBoot()

      class VelocityBoot:
        def __init__(self,):
          self.vx = 0
          self.vy = 0
          self.vz = 0
          pass
      class Velocity:
        def __init__(self,):
          pass
          self.velocity_boot = VelocityBoot()

      class OldCvtNewLoc:
        def __init__(self,):
          self.position = Position()
          self.orientation = Orientation()
          self.status = Status()
          self.velocity = Velocity()
      try:
        loc_msg_dict = {}
        for topic, msg, t in self.bag.read_messages("/iflytek/localization/ego_pose"):
          loc_msg_dict[msg.msg_header.timestamp / 1e6] = msg
        sorted_loc_msg_dict = OrderedDict(sorted(loc_msg_dict.items(), key=lambda ele: ele[0]))
        for t, msg in sorted_loc_msg_dict.items():
          self.loc_msg['t'].append(t)
          self.loc_msg['timestamp'].append(msg.msg_header.timestamp)
          cvt_msg = OldCvtNewLoc()
          cvt_msg.position.position_boot.x = msg.pose.local_position.x
          cvt_msg.position.position_boot.y = msg.pose.local_position.y
          cvt_msg.orientation.euler_boot.yaw = msg.pose.euler_angles.yaw
          cvt_msg.velocity.velocity_boot.vx = msg.pose.linear_velocity_from_wheel
          cvt_msg.status.status_info.mode = 2
          self.loc_msg['data'].append(cvt_msg)
        t0 = self.loc_msg['t'][0]
        print("T0 in loc msg:",t0)
        self.loc_msg['t'] = [tmp - t0  for tmp in self.loc_msg['t']]
        max_time = max(max_time, self.loc_msg['t'][-1])
        print('loc_msg time:',self.loc_msg['t'][-1])
        if len(self.loc_msg['t']) > 0:
          self.loc_msg['enable'] = True
        else:
          self.loc_msg['enable'] = False
      except Exception as e:
        self.loc_msg['enable'] = False
        print('missing /iflytek/localization/ego_pose !!!')
        print(e)

      try:
        origin_loc_msg_dict = {}
        for topic, msg, t in self.bag.read_messages("/iflytek/localization/ego_pose_origin"):
          origin_loc_msg_dict[msg.msg_header.timestamp / 1e6] = msg
        sorted_loc_msg_dict = OrderedDict(sorted(origin_loc_msg_dict.items(), key=lambda ele: ele[0]))
        for t, msg in sorted_loc_msg_dict.items():
          self.origin_loc_msg['t'].append(t)
          self.origin_loc_msg['timestamp'].append(msg.msg_header.timestamp)
          cvt_msg = OldCvtNewLoc()
          cvt_msg.position.position_boot.x = msg.pose.local_position.x
          cvt_msg.position.position_boot.y = msg.pose.local_position.y
          cvt_msg.orientation.euler_boot.yaw = msg.pose.euler_angles.yaw
          cvt_msg.velocity.velocity_boot.vx = msg.pose.linear_velocity_from_wheel
          cvt_msg.status.status_info.mode = 2
          self.origin_loc_msg['data'].append(cvt_msg)

        self.origin_loc_msg['t'] = [tmp - t0  for tmp in self.origin_loc_msg['t']]
        max_time = max(max_time, self.origin_loc_msg['t'][-1])
        print('loc_msg time:',self.origin_loc_msg['t'][-1])
        if len(self.origin_loc_msg['t']) > 0:
          self.origin_loc_msg['enable'] = True
        else:
          self.origin_loc_msg['enable'] = False
      except Exception as e:
        self.origin_loc_msg['enable'] = False
        print('missing /iflytek/localization/ego_pose_origin !!!')

    # load road_fusion msg
    try:
      road_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/road_fusion"):
        road_msg_dict[msg.msg_header.stamp / 1e6] = msg
      road_msg_dict = {key: val for key, val in sorted(road_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in road_msg_dict.items():
        self.road_msg['t'].append(t)
        self.road_msg['timestamp'].append(msg.msg_header.stamp)
        self.road_msg['data'].append(msg)
      if t0 == 0:
        t0 = self.road_msg['t'][0]
      self.road_msg['t'] = [tmp - t0  for tmp in self.road_msg['t']]
      print('road_msg time:',self.road_msg['t'][-1])
      if len(self.road_msg['t']) > 0:
        self.road_msg['enable'] = True
      else:
        self.road_msg['enable'] = False
    except Exception as e:
      self.road_msg['enable'] = False
      print('missing /iflytek/fusion/road_fusion topic !!!')


    # load mobileye lane_lines msg
    try:
      mobileye_lane_lines_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/mobileye/camera_perception/lane_lines"):
        # print(msg.msg_header.stamp)
        mobileye_lane_lines_msg_dict[msg.msg_header.stamp / 1e6] = msg
      sorted_mobileye_lane_lines_msg_dict = OrderedDict(sorted(mobileye_lane_lines_msg_dict.items(), key=lambda ele: ele[0]))
      for t, msg in sorted_mobileye_lane_lines_msg_dict.items():
        self.mobileye_lane_lines_msg['t'].append(t)
        self.mobileye_lane_lines_msg['timestamp'].append(msg.msg_header.stamp)
        self.mobileye_lane_lines_msg['data'].append(msg)
      self.mobileye_lane_lines_msg['t'] = [tmp - t0  for tmp in self.mobileye_lane_lines_msg['t']]
      print('mobileye_lane_lines_msg time:',self.mobileye_lane_lines_msg['t'][-1])
      if len(self.mobileye_lane_lines_msg['t']) > 0:
        self.mobileye_lane_lines_msg['enable'] = True
      else:
        self.mobileye_lane_lines_msg['enable'] = False
    except Exception as e:
      self.mobileye_lane_lines_msg['enable'] = False
      print('missing /mobileye/camera_perception/lane_lines topic !!!')
    print('mobileye_lane_lines_msg[enable]:', self.mobileye_lane_lines_msg['enable'])
    # load rdg lane_lines msg
    try:
      rdg_lane_lines_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/lane_lines"):
        rdg_lane_lines_msg_dict[msg.isp_timestamp / 1e6] = msg
      # rdg_lane_lines_msg_dict = {key: val for key, val in sorted(rdg_lane_lines_msg_dict.items(), key = lambda ele: ele[0])}
      sorted_rdg_lane_lines_msg_dict = OrderedDict(sorted(rdg_lane_lines_msg_dict.items(), key=lambda ele: ele[0]))
      for t, msg in sorted_rdg_lane_lines_msg_dict.items():
        self.rdg_lane_lines_msg['t'].append(t)
        self.rdg_lane_lines_msg['timestamp'].append(msg.isp_timestamp)
        self.rdg_lane_lines_msg['seq'].append(msg.msg_header.seq)
        self.rdg_lane_lines_msg['data'].append(msg)
      self.rdg_lane_lines_msg['t'] = [tmp - t0  for tmp in self.rdg_lane_lines_msg['t']]
      print('rdg_lane_lines_msg time:',self.rdg_lane_lines_msg['t'][-1])
      if len(self.rdg_lane_lines_msg['t']) > 0:
        self.rdg_lane_lines_msg['enable'] = True
      else:
        self.rdg_lane_lines_msg['enable'] = False
    except Exception as e:
      self.rdg_lane_lines_msg['enable'] = False
      print('missing /iflytek/camera_perception/lane_lines topic !!!')

    # load lane_topo
    try:
      lane_topo_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/lane_topo"):
        lane_topo_msg_dict[msg.msg_header.stamp / 1e6] = msg
      sorted_lane_topo_msg_dict = OrderedDict(sorted(lane_topo_msg_dict.items(), key=lambda ele: ele[0]))
      for t, msg in sorted_lane_topo_msg_dict.items():
        self.lane_topo_msg['t'].append(t)
        self.lane_topo_msg['timestamp'].append(msg.msg_header.stamp)
        self.lane_topo_msg['data'].append(msg)
      self.lane_topo_msg['t'] = [tmp - t0  for tmp in self.lane_topo_msg['t']]
      print('lane_topo_msg time:',self.lane_topo_msg['t'][-1])
      if len(self.lane_topo_msg['t']) > 0:
        self.lane_topo_msg['enable'] = True
      else:
        self.lane_topo_msg['enable'] = False
    except Exception as e:
      self.lane_topo_msg['enable'] = False
      print('missing /iflytek/camera_perception/lane_topo topic !!!')
    # load fusion objects msg
    try:
      fus_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/objects"):
        fus_msg_dict[msg.msg_header.stamp / 1e6] = msg
      # fus_msg_dict = {key: val for key, val in sorted(fus_msg_dict.items(), key = lambda ele: ele[0])}
      sorted_fus_msg_dict = OrderedDict(sorted(fus_msg_dict.items(), key=lambda ele: ele[0]))
      for t, msg in sorted_fus_msg_dict.items():
        self.fus_msg['t'].append(t)
        self.fus_msg['data'].append(msg)
        self.fus_msg['timestamp'].append(msg.msg_header.stamp)
      self.fus_msg['t'] = [tmp - t0  for tmp in self.fus_msg['t']]
      print('fus_msg time:',self.fus_msg['t'][-1])
      if len(self.fus_msg['t']) > 0:
        self.fus_msg['enable'] = True
      else:
        self.fus_msg['enable'] = False
    except Exception as e:
      self.fus_msg['enable'] = False
      print('missing /iflytek/fusion/objects !!!')

    # load mobile_eye_camera objects msg
    try:
      mobileye_objects_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/mobileye/camera_perception/objects"):
        mobileye_objects_msg_dict[msg.msg_header.stamp / 1e6] = msg
      sorted_mobileye_objects_msg_dict = OrderedDict(sorted(mobileye_objects_msg_dict.items(), key=lambda ele: ele[0]))
      for t, msg in sorted_mobileye_objects_msg_dict.items():
        self.mobileye_objects_msg['t'].append(t)
        self.mobileye_objects_msg['data'].append(msg)
        self.mobileye_objects_msg['timestamp'].append(msg.msg_header.stamp)
      self.mobileye_objects_msg['t'] = [tmp - t0  for tmp in self.mobileye_objects_msg['t']]
      print('mobileye_objects_msg time:',self.mobileye_objects_msg['t'][-1])
      if len(self.mobileye_objects_msg['t']) > 0:
        self.mobileye_objects_msg['enable'] = True
      else:
        self.mobileye_objects_msg['enable'] = False
    except Exception as e:
      self.mobileye_objects_msg['enable'] = False
      print('missing /mobileye/camera_perception/objects !!!')

    # load rdg objects msg
    try:
      rdg_objects_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/objects"):
        rdg_objects_msg_dict[msg.msg_header.stamp / 1e6] = msg
      sorted_rdg_objects_msg_dict = OrderedDict(sorted(rdg_objects_msg_dict.items(), key=lambda ele: ele[0]))
      for t, msg in sorted_rdg_objects_msg_dict.items():
        self.rdg_objects_msg['t'].append(t)
        self.rdg_objects_msg['data'].append(msg)
        self.rdg_objects_msg['timestamp'].append(msg.msg_header.stamp)
      self.rdg_objects_msg['t'] = [tmp - t0  for tmp in self.rdg_objects_msg['t']]
      print('rdg_objects_msg time:',self.rdg_objects_msg['t'][-1])
      if len(self.rdg_objects_msg['t']) > 0:
        self.rdg_objects_msg['enable'] = True
      else:
        self.rdg_objects_msg['enable'] = False
    except Exception as e:
      self.rdg_objects_msg['enable'] = False
      print('missing /iflytek/camera_perception/objects !!!')

    # load lidar objects msg
    try:
      lidar_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/lidar_objects"):
        lidar_msg_dict[msg.msg_header.stamp / 1e6] = msg
      lidar_msg_dict = {key: val for key, val in sorted(lidar_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in lidar_msg_dict.items():
        self.lidar_msg['t'].append(t)
        self.lidar_msg['data'].append(msg)
        self.lidar_msg['timestamp'].append(msg.msg_header.stamp)
      self.lidar_msg['t'] = [tmp - t0  for tmp in self.lidar_msg['t']]
      print('lidar_msg time:',self.lidar_msg['t'][-1])
      if len(self.lidar_msg['t']) > 0:
        self.lidar_msg['enable'] = True
      else:
        self.lidar_msg['enable'] = False
    except Exception as e:
      self.lidar_msg['enable'] = False
      print('missing /iflytek/lidar_objects !!!')

    # load radar objects msg
    radar_msg = [self.radar_fm_msg,self.radar_fl_msg,self.radar_fr_msg,self.radar_rl_msg,self.radar_rr_msg]
    topic_name = ["/iflytek/radar_fm_perception_info","/iflytek/radar_fl_perception_info","/iflytek/radar_fr_perception_info","/iflytek/radar_rl_perception_info","/iflytek/radar_rr_perception_info"]
    # for i in range(5):
    #   print(topic[i])
    for i in range(5):
      try:
        radar_msg_dict = {}
        if i == 0:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fm_perception_info"):
            radar_msg_dict[msg.msg_header.stamp / 1e6] = msg
        elif i == 1:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fl_perception_info"):
            radar_msg_dict[msg.msg_header.stamp / 1e6] = msg
        elif i == 2:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fr_perception_info"):
            radar_msg_dict[msg.msg_header.stamp / 1e6] = msg
        elif i == 3:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_rl_perception_info"):
            radar_msg_dict[msg.msg_header.stamp / 1e6] = msg
        elif i == 4:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_rr_perception_info"):
            radar_msg_dict[msg.msg_header.stamp / 1e6] = msg
        radar_msg_dict = {key: val for key, val in sorted(radar_msg_dict.items(), key = lambda ele: ele[0])}
        for t, msg in radar_msg_dict.items():
          radar_msg[i]['t'].append(t)
          radar_msg[i]['data'].append(msg)
          radar_msg[i]['timestamp'].append(msg.msg_header.stamp)
          #temp_t = radar_msg[i]['t']
        radar_msg[i]['t'] = [tmp - t0  for tmp in radar_msg[i]['t']]
          #print("load message:",i)
        #print(radar_msg[i],'_time:',radar_msg[i]['t'][-1])
        if len(radar_msg[i]['t']) > 0:
          radar_msg[i]['enable'] = True
          print("true:",topic_name[i])
          #print("size:",len(radar_msg[i]['t']))
        else:
          radar_msg[i]['enable'] = False
      except Exception as e:
        radar_msg[i]['enable'] = False
        print('missing',topic[i])

    # # load radar_fl objects msg
    # try:
    #   radar_fl_msg_dict = {}
    #   for topic, msg, t in self.bag.read_messages("/iflytek/radar_fl_perception_info"):
    #     radar_fl_msg_dict[msg.msg_header.stamp / 1e6] = msg
    #   radar_fl_msg_dict = {key: val for key, val in sorted(radar_fl_msg_dict.items(), key = lambda ele: ele[0])}
    #   for t, msg in radar_fl_msg_dict.items():
    #     self.radar_fl_msg['t'].append(t)
    #     self.radar_fl_msg['data'].append(msg)
    #   self.radar_fl_msg['t'] = [tmp - t0  for tmp in self.radar_fl_msg['t']]
    #   print('radar_fl_msg time:',self.radar_fl_msg['t'][-1])
    #   if len(self.radar_fl_msg['t']) > 0:
    #     self.radar_fl_msg['enable'] = True
    #   else:
    #     self.radar_fl_msg['enable'] = False
    # except Exception as e:
    #   self.radar_fl_msg['enable'] = False
    #   print('missing /iflytek/radar_fl/objects !!!')

    # load vehicle service msg
    try:
      vs_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/vehicle_service"):
        vs_msg_dict[msg.msg_header.stamp / 1e6] = msg
      vs_msg_dict = {key: val for key, val in sorted(vs_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in vs_msg_dict.items():
        self.vs_msg['t'].append(t)
        self.vs_msg['data'].append(msg)
        self.vs_msg['timestamp'].append(msg.msg_header.stamp)
      self.vs_msg['t'] = [tmp - t0  for tmp in self.vs_msg['t']]
      self.vs_msg['enable'] = True
      print('vs time:',self.vs_msg['t'][-1])
      max_time = max(max_time, self.vs_msg['t'][-1])
      if len(self.vs_msg['t']) > 0:
        self.vs_msg['enable'] = True
      else:
        self.vs_msg['enable'] = False
    except Exception as e:
      self.vs_msg['enable'] = False
      print("missing /iflytek/vehicle_service !!!")

    # load planning msg
    try:
      plan_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/plan"):
        plan_msg_dict[msg.msg_header.stamp / 1e6] = msg
      plan_msg_dict = {key: val for key, val in sorted(plan_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_msg_dict.items():
        self.plan_msg['t'].append(t)
        self.plan_msg['data'].append(msg)
        self.plan_msg['timestamp'].append(msg.msg_header.stamp)
      self.plan_msg['t'] = [tmp - t0  for tmp in self.plan_msg['t']]
      max_time = max(max_time, self.plan_msg['t'][-1])
      print('plan_msg time:',self.plan_msg['t'][-1])
      if len(self.plan_msg['t']) > 0:
        self.plan_msg['enable'] = True
      else:
        self.plan_msg['enable'] = False
    except Exception as e:
      self.plan_msg['enable'] = False
      print("missing /iflytek/planning/plan !!!")


    # load prediction msg
    try:
      prediction_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/prediction/prediction_result"):
        prediction_msg_dict[msg.msg_header.stamp / 1e6] = msg
      prediction_msg_dict = {key: val for key, val in sorted(prediction_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in prediction_msg_dict.items():
        self.prediction_msg['t'].append(t)
        self.prediction_msg['data'].append(msg)
        self.prediction_msg['timestamp'].append(msg.msg_header.stamp)
      self.prediction_msg['t'] = [tmp - t0  for tmp in self.prediction_msg['t']]
      self.prediction_msg['enable'] = True
      max_time = max(max_time, self.prediction_msg['t'][-1])
      print('prediction_msg time:',self.prediction_msg['t'][-1])
      if len(self.prediction_msg['t']) > 0:
        self.prediction_msg['enable'] = True
      else:
        self.prediction_msg['enable'] = False
    except Exception as e:
      self.prediction_msg['enable'] = False
      print("missing /iflytek/prediction/prediction_result !!!")

    # load planning debug msg
    try:
      json_value_list = ['VisionLonBehavior_a_target_high', 'VisionLonBehavior_a_target_low', \
                         "replan_status", "ego_pos_x", "ego_pos_y", "ego_pos_yaw", 'predicted_ego_x', 'predicted_ego_y', \
                         "solver_condition", "dist_err", "lat_err", "theta_err", "lon_err", "dbw_status", "iLqr_lat_update_time", "concerned_start_q_jerk", \
                         'acc_target_high', 'acc_target_low', 'acc_cipv', 'time_headway_level', 'desired_distance', 'desired_distance_filtered',\
                         "VisionLateralBehaviorPlannerCost", "VisionLateralMotionPlannerCost","VisionLongitudinalBehaviorPlannerCost", \
                         "EnvironmentalModelManagerCost", "GeneralPlannerModuleCostTime", "planning_time_cost", 'construct_st_graph_cost', 'st_graph_searcher_cost', \
                         'v_limit_road', 'v_limit_in_turns','v_target', 'v_cruise', 'v_ego', \
                         'lead_one_id', 'lead_one_dis', 'lead_one_vel', "v_target_lead_one", 'soft_brake_distance_lead',\
                         'lead_two_id', 'lead_two_dis', 'lead_two_vel', "v_target_lead_two", \
                         'temp_lead_one_id', 'temp_lead_one_dis', 'temp_lead_one_vel', "v_target_temp_lead_one", \
                         'temp_lead_two_id', 'temp_lead_two_dis', 'temp_lead_two_vel', "v_target_temp_lead_two", \
                         'potential_cutin_track_id', 'v_target_potential_cutin', "v_target_cutin", "road_radius", \
                         'new_cutin_id', 'new_cutin_id_count', "CIPV_id",\
                         'stop_start_state', 'v_target_start_stop', 'STANDSTILL', 'jlt_status_farslow', 'jlt_status_stable', \
                         "dis_to_ramp", "v_target_ramp", "narrow_agent_id","narrow_agent_v_limit",\
                         'virtual_lane_relative_id_switch_flag', \
                         'is_exist_split_on_ramp', 'is_exist_ramp_on_road', 'is_exist_split_on_expressway', 'is_exist_intersection_split', 'current_segment_passed_distance', \
                         'is_in_ramp_select_split_situation','is_on_road_select_ramp_situation', \
                         'select_ego_lane_without_plan', 'select_ego_lane_with_plan', 'origin_relative_id_zero_nums', \
                         'gap_v_limit_lc', "max_brake_distance", "gap_base_car_id", "gap_front_car_id",\
                         "fast_lead_id", "slow_lead_id", "fast_car_cut_in_id", "slow_car_cut_in_id", \
                         "RealTime_desired_distance_rss", "RealTime_desired_distance_calibrate", \
                         'sdmap_valid_','lane_change_cmd_','cur_state','lc_map_decision', \
                         "is_in_merge_area","current_lane_order_id","current_lane_virtual_id","current_lane_relative_id","left_boundary_type","right_boundary_type", \
                         "enable_l_", "enable_r_", "is_left_lane_change_safe_", "is_right_lane_change_safe_", "overtake_count_", "is_left_overtake", "is_right_overtake", "trigger_left_overtake", "trigger_right_overtake", "overtake_vehicle_id", "dash_line_len", \
                         "left_route_traffic_speed", "right_route_traffic_speed", "speed_threshold", \
                         "is_cone_lane_change_situation_", "cone_alc_trigger_counter_", "cone_lane_change_direction_", "cone_nums_of_front_objects", "is_emergency_avoidance_situation_", "leading_vehicle_id_", \
                         "both_lane_line_exist_virtual_or_not_","is_merge_lane_change_situation_", "merge_alc_trigger_counter_", "left_boundary_exist_virtual_type", "right_boundary_exist_virtual_type", \
                         'LateralMotionCostTime', 'RealTimeLateralBehaviorCostTime', 'TrajectoryGeneratorCostTime', \
                         "SccLonBehaviorCostTime", "SccLonMotionCostTime", "dynamic_world_cost", \
                         "front_node_id", "rear_node_id","prohibit_acc_", \
                         "ego_left_node", "ego_left_front_node", "ego_left_rear_node", \
                         "ego_right_node", "ego_right_front_node", "ego_right_rear_node", \
                         "current_intersection_state", "last_intersection_state", "distance_to_stopline", "traffic_status_straight", "v_target_intersection", "v_target_virtual_obs", "distance_to_crosswalk", \
                         "lane_width", "smooth_lateral_offset", "normal_left_avoid_threshold","normal_right_avoid_threshold", "lat_offset","smooth_lateral_offset", "avoid_way", "allow_side_max_opposite_offset", "allow_side_max_opposite_offset_id", \
                         "allow_front_max_opposite_offset", "allow_front_max_opposite_offset_id", "ego_l", "avoid_car_id", "avoid_car_ids_1", "avoid_car_ids_2", \
                         "select_avoid_car_ids_1", "select_avoid_car_ids_2", "turn_switch_state","is_ego_on_expressway","current_segment_id","distance_to_route_end","sum_dis_to_last_merge_point","sum_dis_to_last_split_point", \
                         "is_leaving_ramp","is_nearing_ramp", 'road_to_ramp_turn_signal','lat_diff', "far_kappa_radius",'ramp_direction','is_merge_region', 'is_split_region', 'merge_lane_virtual_id', \
                          'ego_lane_boundary_exist_virtual_line','target_lane_boundary_exist_virtual_line', \
                         'sdmap_min_curv_radius',"is_static_avoid_scene", \
                         "is_overlap", "merge_target_one_id", "merge_target_two_id", "v_target_merge", "rear_agent_merge_time", "merge_orintation","merge_direction_plan",'ego_has_rightof_tar_lane',
                         'merge_exist','is_merge_region_plan', 'merge_point_distance', "merge_point_x", "merge_point_y", "current_lane_is_continue", 'cipv_id_st',
                         'distance_to_ramp','distance_to_first_road_merge','distance_to_first_road_split','is_nearing_other_lane_merge_to_road_point',
                         'macroeconomic_decider_merge_point_x','macroeconomic_decider_merge_point_y',
                         'boundary_line_merge_point_x','boundary_line_merge_point_y','cur_lane_is_continue','forward_lane_num',
                         'is_ego_on_split_region', 'last_split_seg_dir', 'need_continue_lc_num_on_off_ramp_region',
                         'is_left_merge_direction', 'is_right_merge_direction', 'search_succeed', 'expanded_nodes_size', 'history_cur_nodes_size', 'open_set_empty','v3_start_stop_status','cipv_relative_s',
                         "agents_headway_id", "agents_headway_value", "has_target_follow_curve", "has_stable_follow_target", "has_farslow_follow_target",'cipv_relative_s_ego_stop',"distance_to_go_condition",
                         "cipv_vel_frenet","traffic_light_can_pass","gap_lon_decision_update","gap_front_agent_id","gap_rear_agent_id","lane_change_status"]

      json_vector_list = ["raw_refline_x_vec", "raw_refline_y_vec", "raw_refline_s_vec", "raw_refline_k_vec", "assembled_x", "assembled_y", "assembled_theta", "assembled_delta", "assembled_omega", "traj_s_vec", "traj_x_vec", "traj_y_vec", "limit_v_type",
                         "ego_front_agent_traj_x_vec","ego_front_agent_traj_y_vec","ego_front_agent_traj_theta_vec",
                         "ego_rear_agent_traj_x_vec","ego_rear_agent_traj_y_vec","ego_rear_agent_traj_theta_vec",
                         "ego_left_agent_traj_x_vec","ego_left_agent_traj_y_vec","ego_left_agent_traj_theta_vec",
                         "ego_right_agent_traj_x_vec","ego_right_agent_traj_y_vec","ego_right_agent_traj_theta_vec",
                         "ego_left_front_agent_traj_x_vec","ego_left_front_agent_traj_y_vec","ego_left_front_agent_traj_theta_vec",
                         "ego_right_front_agent_traj_x_vec","ego_right_front_agent_traj_y_vec","ego_right_front_agent_traj_theta_vec",
                         "ego_left_rear_agent_traj_x_vec","ego_left_rear_agent_traj_y_vec","ego_left_rear_agent_traj_theta_vec",
                         "ego_right_rear_agent_traj_x_vec","ego_right_rear_agent_traj_y_vec","ego_right_rear_agent_traj_theta_vec",
                         "expanded_nodes_t_vec", "expanded_nodes_s_vec", "history_cur_nodes_t_vec", "history_cur_nodes_s_vec",
                         "st_path_final_nodes_total_cost_vec","st_path_final_nodes_g_cost_vec","st_path_final_nodes_h_cost_vec",
                         "st_path_final_nodes_cost_yield_vec","st_path_final_nodes_cost_overtake_vec","st_path_final_nodes_cost_vel_vec",
                         "st_path_final_nodes_cost_accel_vec","st_path_final_nodes_cost_accel_sign_changed_vec",
                         "st_path_final_nodes_cost_jerk_vec","st_path_final_nodes_cost_length_vec", "st_path_final_nodes_time_vec",]

      plan_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info"):
        planning_debug_output = PlanningDebugInfo()
        planning_debug_output.ParseFromString(msg.debug_info)
        plan_debug_msg_dict[planning_debug_output.timestamp / 1e6] = planning_debug_output
      plan_debug_msg_dict = {key: val for key, val in sorted(plan_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_debug_msg_dict.items():
        self.plan_debug_msg['t'].append(t)
        self.plan_debug_msg['data'].append(msg)
        self.plan_debug_msg['timestamp'].append(msg.timestamp)
        try:
          json_struct = json.loads(msg.data_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)
          try:
            # print(json_struct['assembled_omega'])
            print(json_struct['limit_v_type'])
          except Exception as e:
            pass
          self.plan_debug_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      self.plan_debug_msg['t'] = [tmp - t0  for tmp in self.plan_debug_msg['t']]
      max_time = max(max_time, self.plan_debug_msg['t'][-1])
      print('plan_debug_msg time:',self.plan_debug_msg['t'][-1])
      if len(self.plan_debug_msg['t']) > 0:
        self.plan_debug_msg['enable'] = True
      else:
        self.plan_debug_msg['enable'] = False
    except Exception as e:
      self.plan_debug_msg['enable'] = False
      print("missing /iflytek/planning/debug_info !!!")


    # load planning debug origin msg
    try:
      plan_debug_origin_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info_origin"):
        planning_debug_output = PlanningDebugInfo()
        planning_debug_output.ParseFromString(msg.debug_info)
        plan_debug_origin_msg_dict[planning_debug_output.timestamp / 1e6] = planning_debug_output
      plan_debug_origin_msg_dict = {key: val for key, val in sorted(plan_debug_origin_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_debug_origin_msg_dict.items():
        self.plan_debug_origin_msg['t'].append(t)
        self.plan_debug_origin_msg['data'].append(msg)
        self.plan_debug_origin_msg['timestamp'].append(msg.timestamp)
        try:
          json_struct = json.loads(msg.data_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)
          try:
            # print(json_struct['assembled_omega'])
            print(json_struct['limit_v_type'])
          except Exception as e:
            pass
          self.plan_debug_origin_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      self.plan_debug_origin_msg['t'] = [tmp - t0  for tmp in self.plan_debug_origin_msg['t']]
      max_time = max(max_time, self.plan_debug_origin_msg['t'][-1])
      print('plan_debug_origin_msg time:',self.plan_debug_origin_msg['t'][-1])
      if len(self.plan_debug_origin_msg['t']) > 0:
        self.plan_debug_origin_msg['enable'] = True
      else:
        self.plan_debug_origin_msg['enable'] = False
    except Exception as e:
      self.plan_debug_origin_msg['enable'] = False
      print("missing /iflytek/planning/debug_info_origin !!!")


    # load control msg
    try:
      ctrl_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/control/control_command"):
        ctrl_msg_dict[msg.msg_header.stamp / 1e6] = msg
      ctrl_msg_dict = {key: val for key, val in sorted(ctrl_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ctrl_msg_dict.items():
        self.ctrl_msg['t'].append(t)
        self.ctrl_msg['data'].append(msg)
        self.ctrl_msg['timestamp'].append(msg.msg_header.stamp)
      self.ctrl_msg['t'] = [tmp - t0  for tmp in self.ctrl_msg['t']]
      max_time = max(max_time, self.ctrl_msg['t'][-1])
      print('ctrl_msg time:',self.ctrl_msg['t'][-1])
      if len(self.ctrl_msg['t']) > 0:
        self.ctrl_msg['enable'] = True
      else:
        self.ctrl_msg['enable'] = False
    except Exception as e:
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

      ctrl_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/control/debug_info"):
        ctrl_debug_output = ControlDebugInfo()
        ctrl_debug_output.ParseFromString(msg.debug_info)
        ctrl_debug_msg_dict[ctrl_debug_output.timestamp / 1e6] = ctrl_debug_output
      ctrl_debug_msg_dict = {key: val for key, val in sorted(ctrl_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ctrl_debug_msg_dict.items():
        self.ctrl_debug_msg['t'].append(t)
        self.ctrl_debug_msg['data'].append(msg)
        self.ctrl_debug_msg['timestamp'].append(msg.timestamp)
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
      print('ctrl_debug_msg time:',self.ctrl_debug_msg['t'][-1])
      if len(self.ctrl_debug_msg['t']) > 0:
        self.ctrl_debug_msg['enable'] = True
      else:
        self.ctrl_debug_msg['enable'] = False
    except Exception as e:
      self.ctrl_debug_msg['enable'] = False
      print("missing /iflytek/control/debug_info !!!")

    # load parking fusion msg
    try:
      fus_parking_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/parking_slot"):
        fus_parking_msg_dict[msg.msg_header.stamp / 1e6] = msg
      fus_parking_msg_dict = {key: val for key, val in sorted(fus_parking_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in fus_parking_msg_dict.items():
        self.fus_parking_msg['t'].append(t)
        self.fus_parking_msg['data'].append(msg)
        self.fus_parking_msg['timestamp'].append(msg.msg_header.stamp)
      self.fus_parking_msg['t'] = [tmp - self.fus_parking_msg['t'][0]  for tmp in self.fus_parking_msg['t']]
      max_time = max(max_time, self.fus_parking_msg['t'][-1])
      print('fus_parking_msg time:',self.fus_parking_msg['t'][-1])
      if len(self.fus_parking_msg['t']) > 0:
        self.fus_parking_msg['enable'] = True
      else:
        self.fus_parking_msg['enable'] = False
    except Exception as e:
      self.fus_parking_msg['enable'] = False
      print('missing /iflytek/fusion/parking_slot !!!')

    # load state machine msg
    try:
      soc_state_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fsm/soc_state"):
        soc_state_msg_dict[msg.msg_header.stamp / 1e6] = msg
      soc_state_msg_dict = {key: val for key, val in sorted(soc_state_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in soc_state_msg_dict.items():
        self.soc_state_msg['t'].append(t)
        self.soc_state_msg['data'].append(msg)
        self.soc_state_msg['timestamp'].append(msg.msg_header.stamp)
      self.soc_state_msg['t'] = [tmp - self.soc_state_msg['t'][0]  for tmp in self.soc_state_msg['t']]
      max_time = max(max_time, self.soc_state_msg['t'][-1])
      print('soc_state_msg time:',self.soc_state_msg['t'][-1])
      if len(self.soc_state_msg['t']) > 0:
        self.soc_state_msg['enable'] = True
      else:
        self.soc_state_msg['enable'] = False
    except Exception as e:
      self.soc_state_msg['enable'] = False
      print('missing /iflytek/fsm/soc_state !!!')

    # load ehr static map msg
    try:
      ehr_static_map_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ehr/static_map"):
        ehr_static_map_msg_dict[msg.msg_header.stamp / 1e3] = msg
      ehr_static_map_msg_dict = {key: val for key, val in sorted(ehr_static_map_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ehr_static_map_msg_dict.items():
        self.ehr_static_map_msg['t'].append(t)
        self.ehr_static_map_msg['data'].append(msg)
        self.ehr_static_map_msg['timestamp'].append(msg.msg_header.stamp)
      self.ehr_static_map_msg['t'] = [tmp - t0  for tmp in self.ehr_static_map_msg['t']]
      print('ehr_static_map_msg time:',self.ehr_static_map_msg['t'][-1])
      if len(self.ehr_static_map_msg['t']) > 0:
        self.ehr_static_map_msg['enable'] = True
      else:
        self.ehr_static_map_msg['enable'] = False
    except Exception as e:
      self.ehr_static_map_msg['enable'] = False
      print('missing /iflytek/ehr/static_map topic !!!')

    # load ehr sd map msg
    try:
      ehr_sd_map_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ehr/sdmap_info"):
        sdmap = SdMap()
        sdmap.ParseFromString(msg.debug_info)
        ehr_sd_map_msg_dict[sdmap.header.timestamp / 1e6] = sdmap
      ehr_sd_map_msg_dict = {key: val for key, val in sorted(ehr_sd_map_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ehr_sd_map_msg_dict.items():
        self.ehr_sd_map_msg['t'].append(t)
        self.ehr_sd_map_msg['data'].append(msg)
        self.ehr_sd_map_msg['timestamp'].append(msg.header.timestamp)
      self.ehr_sd_map_msg['t'] = [tmp - t0  for tmp in self.ehr_sd_map_msg['t']]
      print('ehr_sd_map_msg time:',self.ehr_sd_map_msg['t'][-1])
      if len(self.ehr_sd_map_msg['t']) > 0:
        self.ehr_sd_map_msg['enable'] = True
      else:
        self.ehr_sd_map_msg['enable'] = False
    except Exception as e:
      self.ehr_sd_map_msg['enable'] = False
      print('missing /iflytek/ehr/sdmap topic !!!')

    # load ehr parking map msg
    try:
      ehr_parking_map_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ehr/parking_map"):
        ehr_parking_map_msg_dict[msg.msg_header.stamp / 1e6] = msg
      ehr_parking_map_msg_dict = {key: val for key, val in sorted(ehr_parking_map_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ehr_parking_map_msg_dict.items():
        self.ehr_parking_map_msg['t'].append(t)
        self.ehr_parking_map_msg['data'].append(msg)
        self.ehr_parking_map_msg['timestamp'].append(msg.msg_header.stamp)
      self.ehr_parking_map_msg['t'] = [tmp - t0  for tmp in self.ehr_parking_map_msg['t']]
      print('ehr_parking_map_msg time:',self.ehr_parking_map_msg['t'][-1])
      if len(self.ehr_parking_map_msg['t']) > 0:
        self.ehr_parking_map_msg['enable'] = True
      else:
        self.ehr_parking_map_msg['enable'] = False
    except Exception as e:
      self.ehr_parking_map_msg['enable'] = False
      print('missing /iflytek/ehr/parking_map topic !!!')

    # load ground_line_msg
    try:
      ground_line_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/ground_line"):
        ground_line_msg_dict[msg.msg_header.stamp / 1e6] = msg
      ground_line_msg_dict = {key: val for key, val in sorted(ground_line_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ground_line_msg_dict.items():
        self.ground_line_msg['t'].append(t)
        self.ground_line_msg['data'].append(msg)
        self.ground_line_msg['timestamp'].append(msg.msg_header.stamp)
      self.ground_line_msg['t'] = [tmp - t0  for tmp in self.ground_line_msg['t']]
      print('ground_line_msg time:',self.ground_line_msg['t'][-1])
      if len(self.ground_line_msg['t']) > 0:
        self.ground_line_msg['enable'] = True
      else:
        self.ground_line_msg['enable'] = False
    except Exception as e:
      self.ground_line_msg['enable'] = False
      print('missing /iflytek/fusion/ground_line topic !!!')

    # load planning hmi msg
    try:
      planning_hmi_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/hmi"):
        planning_hmi_msg_dict[msg.msg_header.stamp / 1e6] = msg
      planning_hmi_msg_dict = {key: val for key, val in sorted(planning_hmi_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in planning_hmi_msg_dict.items():
        self.planning_hmi_msg['t'].append(t)
        self.planning_hmi_msg['data'].append(msg)
        self.planning_hmi_msg['timestamp'].append(msg.msg_header.stamp)
      self.planning_hmi_msg['t'] = [tmp - t0  for tmp in self.planning_hmi_msg['t']]
      print('planning_hmi_msg time:',self.planning_hmi_msg['t'][-1])
      if len(self.planning_hmi_msg['t']) > 0:
        self.planning_hmi_msg['enable'] = True
      else:
        self.planning_hmi_msg['enable'] = False
    except Exception as e:
      self.planning_hmi_msg['enable'] = False
      print('missing /iflytek/planning/hmi topic !!!')

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("load bag 耗时：", elapsed_time, "秒")
    return max_time
#/mobileye/camera_perception/objects
  def msg_timeline_figure(self):
    topic_list = [
      '/iflytek/localization/egomotion',
      '/iflytek/localization/ego_pose',
      '/iflytek/fusion/road_fusion',
      '/iflytek/fusion/objects',
      '/mobileye/camera_perception/objects',
      '/iflytek/radar_fm_perception_info',
      '/iflytek/radar_fl_perception_info',
      '/iflytek/radar_fr_perception_info',
      '/iflytek/radar_rl_perception_info',
      '/iflytek/radar_rr_perception_info',
      '/iflytek/vehicle_service',
      '/iflytek/prediction/prediction_result',
      '/iflytek/planning/plan',
      '/iflytek/planning/debug_info',
      '/iflytek/control/control_command',
      '/iflytek/control/debug_info',
      '/iflytek/fusion/parking_slot',
      '/iflytek/fsm/soc_state',
    ]
    detail_list = [
      '/iflytek/planning/plan',
      '/iflytek/planning/debug_info',
      '/iflytek/fsm/soc_state',
    ]
    print("========【使用说明】========")
    print("========鼠标滚轮缩放时间轴，鼠标悬停或点击以下topic的点，可以在浏览器控制台（F12打开）查看该msg具体内容: ")
    print("========", detail_list)
    data_list = [
      self.loc_msg,
      self.old_loc_msg,
      self.road_msg,
      self.fus_msg,
      self.mobileye_objects_msg,
      self.radar_fm_msg,
      self.radar_fl_msg,
      self.radar_fr_msg,
      self.radar_rl_msg,
      self.radar_rr_msg,
      self.vs_msg,
      self.prediction_msg,
      self.plan_msg,
      self.plan_debug_msg,
      self.ctrl_msg,
      self.ctrl_debug_msg,
      self.fus_parking_msg,
      self.soc_state_msg,
    ]

    topic_list_with_hz = topic_list[:]
    for i in range(len(topic_list)):
      if len(data_list[i]['t']) != 0:
        time_span = max(data_list[i]['t']) - min(data_list[i]['t'])
        hz = int(len(data_list[i]['t']) / time_span) if time_span != 0.0 else 0
        topic_list_with_hz[i] += ' (' + str(hz) + 'hz)'

    data = {'topic_with_hz':[], 't':[], 'msg':[]}
    min_time = sys.maxsize
    for i in range(len(topic_list)):
      for j in range(len(data_list[i]['t'])):
        data['topic_with_hz'].append(topic_list_with_hz[i])
        if topic_list[i] in detail_list:
          data['msg'].append(MessageToJson(data_list[i]['data'][j]))
        else:
          data['msg'].append('')
        t = data_list[i]['t'][j]
        data['t'].append(t)
        if t != 0 and t < min_time:
          min_time = t

    # if msg_header time is 0, don't minus
    for i in range(len(data['t'])):
      if data['t'][i] == 0:
        data['t'][i] = 0
      else:
        data['t'][i] = data['t'][i] - min_time

    source = ColumnDataSource(data=data)
    hover = HoverTool(tooltips=[('topic_with_hz', '@topic_with_hz'), ('t', '@t'), ('msg', '@msg')])
    #args=dict(msg_data=msg_data),
    callback = CustomJS(code="""
        //console.log(cb_obj);
        //console.log(cb_data);

        var data = cb_data.source.data;
        var indices = cb_data.source.selected.indices;
        var selected_msgs='';

        for (let i = 0; i < indices.length; i++) {
            const index = indices[i]
            selected_msgs += data['msg'][index];
        }

        console.log(selected_msgs);
    """)
    taptool = TapTool(callback=callback)

    fig1 = bkp.figure(plot_width=1600, plot_height=400,
              y_range=topic_list_with_hz, x_axis_type='datetime', title=self.bag_path,
              tools=[hover, taptool, "xwheel_zoom,reset"], active_scroll='xwheel_zoom')
    fig1.circle(x='t', y='topic_with_hz', source=source)
    return fig1