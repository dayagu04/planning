#!/usr/bin/python
# encoding=utf-8

from IPython.core.display import display, HTML
from cyber_record.record import Record
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import bokeh.plotting as bkp
import rosbag

kRefLat = 39.907500
kRefLon = 116.3880555
kRefAlt = 44.4
ee = 0.00669342162296594323
a = 6378245.0

latitude0 = kRefLat
longitude0 = kRefLon
altitude0 = kRefAlt

class LocalizationApaPlotter(object):
  def __init__(self):
  # bag path and frame dt
    self.file_path = '/data_cold/abu_zone/autoparse/chery_e0y_10034/trigger/20240720/20240720-16-21-25/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-07-20-16-21-25_no_camera.bag'
    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()

    self.planning_call_interval_vec = []
    self.planning_time_vec = []
    self.static_map_call_interval_vec = []
    self.static_map_time_vec = []
    self.pbox_imu_call_interval_vec = []
    self.pbox_imu_time_vec = []
    self.pbox_gnss_call_interval_vec = []
    self.pbox_gnss_time_vec = []
    self.fusion_objects_call_interval_vec = []
    self.fusion_objects_time_vec = []
    self.control_call_interval_vec = []
    self.control_time_vec = []
    self.ego_pose_call_interval_vec = []
    self.ego_pose_time_vec = []
    self.vehicle_service_call_interval_vec = []
    self.vehicle_service_time_vec = []
    self.ehr_position_call_interval_vec = []
    self.ehr_position_time_vec = []
    self.road_fusion_call_interval_vec = []
    self.road_fusion_time_vec = []


  def load_data(self):
    bag = rosbag.Bag(self.file_path)
    start_time = None
    last_planning_time = None
    for topic, msg, t in bag.read_messages('/iflytek/planning/plan'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_planning_time is None:
        self.planning_call_interval_vec.append(0.0)
      else:
        self.planning_call_interval_vec.append((msg.msg_header.stamp - last_planning_time) * 1e-6)
      self.planning_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_planning_time = msg.msg_header.stamp

    last_static_map_time = None
    for topic, msg, t in bag.read_messages('/iflytek/ehr/static_map'):
      if last_static_map_time is None:
        self.static_map_call_interval_vec.append(0.0)
      else:
        self.static_map_call_interval_vec.append((msg.msg_header.stamp * 1000 - last_static_map_time) * 1e-6)
      self.static_map_time_vec.append((msg.msg_header.stamp * 1000 - start_time) * 1e-6)
      last_static_map_time = msg.msg_header.stamp * 1000

    last_pbox_imu_time = None
    for topic, msg, t in bag.read_messages('/iflytek/sensor/pbox/imu'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_pbox_imu_time is None:
        self.pbox_imu_call_interval_vec.append(0.0)
      else:
        self.pbox_imu_call_interval_vec.append((msg.msg_header.stamp - last_pbox_imu_time) * 1e-6)
      self.pbox_imu_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_pbox_imu_time = msg.msg_header.stamp

    last_pbox_gnss_time = None
    for topic, msg, t in bag.read_messages('/iflytek/sensor/pbox/gnss'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_pbox_gnss_time is None:
        self.pbox_gnss_call_interval_vec.append(0.0)
      else:
        self.pbox_gnss_call_interval_vec.append((msg.msg_header.stamp - last_pbox_gnss_time) * 1e-6)
      self.pbox_gnss_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_pbox_gnss_time = msg.msg_header.stamp

    last_fusion_objects_time = None
    for topic, msg, t in bag.read_messages('/iflytek/fusion/objects'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_fusion_objects_time is None:
        self.fusion_objects_call_interval_vec.append(0.0)
      else:
        self.fusion_objects_call_interval_vec.append((msg.msg_header.stamp - last_fusion_objects_time) * 1e-6)
      self.fusion_objects_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_fusion_objects_time = msg.msg_header.stamp

    last_control_time = None
    for topic, msg, t in bag.read_messages('/iflytek/control/control_command'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_control_time is None:
        self.control_call_interval_vec.append(0.0)
      else:
        self.control_call_interval_vec.append((msg.msg_header.stamp - last_control_time) * 1e-6)
      self.control_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_control_time = msg.msg_header.stamp

    last_ego_pose_time = None
    for topic, msg, t in bag.read_messages('/iflytek/localization/ego_pose'):
      if last_ego_pose_time is None:
        self.ego_pose_call_interval_vec.append(0.0)
      else:
        self.ego_pose_call_interval_vec.append((msg.msg_header.stamp * 1000 - last_ego_pose_time) * 1e-6)
      self.ego_pose_time_vec.append((msg.msg_header.stamp * 1000 - start_time) * 1e-6)
      last_ego_pose_time = msg.msg_header.stamp * 1000

    last_vehicle_service_time = None
    for topic, msg, t in bag.read_messages('/iflytek/vehicle_service'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_vehicle_service_time is None:
        self.vehicle_service_call_interval_vec.append(0.0)
      else:
        self.vehicle_service_call_interval_vec.append((msg.msg_header.stamp - last_vehicle_service_time) * 1e-6)
      self.vehicle_service_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_vehicle_service_time = msg.msg_header.stamp

    last_ehr_position_time = None
    for topic, msg, t in bag.read_messages('/iflytek/ehr/position'):
      if start_time is None:
        start_time = msg.msg_header.stamp * 1000
      if last_ehr_position_time is None:
        self.ehr_position_call_interval_vec.append(0.0)
      else:
        self.ehr_position_call_interval_vec.append((msg.msg_header.stamp * 1000 - last_ehr_position_time) * 1e-6)
      self.ehr_position_time_vec.append((msg.msg_header.stamp * 1000 - start_time) * 1e-6)
      last_ehr_position_time = msg.msg_header.stamp * 1000

    last_road_fusion_time = None
    for topic, msg, t in bag.read_messages('/iflytek/fusion/road_fusion'):
      if start_time is None:
        start_time = msg.msg_header.stamp
      if last_road_fusion_time is None:
        self.road_fusion_call_interval_vec.append(0.0)
      else:
        self.road_fusion_call_interval_vec.append((msg.msg_header.stamp - last_road_fusion_time) * 1e-6)
      self.road_fusion_time_vec.append((msg.msg_header.stamp - start_time) * 1e-6)
      last_road_fusion_time = msg.msg_header.stamp


  def plot_figure(self):
    self.load_data()
    self.plot_data()


  def plot_data(self):
    fig1 = bkp.figure(x_axis_label='time', y_axis_label='interval', width=1500, height=500, y_range = [0, 2])
    fig1.line(self.planning_time_vec, self.planning_call_interval_vec, line_width=1, line_color='red', legend_label = 'planning')
    fig1.line(self.static_map_time_vec, self.static_map_call_interval_vec, line_width=1, line_color='blue', legend_label = 'static_map')
    fig1.line(self.pbox_imu_time_vec, self.pbox_imu_call_interval_vec, line_width=1, line_color='green', legend_label = 'pbox_imu')
    fig1.line(self.pbox_gnss_time_vec, self.pbox_gnss_call_interval_vec, line_width=1, line_color='black', legend_label = 'pbox_gnss')
    fig1.line(self.fusion_objects_time_vec, self.fusion_objects_call_interval_vec, line_width=1, line_color='gold', legend_label = 'fusion_objects')
    fig1.line(self.control_time_vec, self.control_call_interval_vec, line_width=1, line_color='indigo', legend_label = 'control')
    # fig1.line(self.ego_pose_time_vec, self.ego_pose_call_interval_vec, line_width=1, line_color='brown', legend_label = 'ego_pose')
    fig1.line(self.vehicle_service_time_vec, self.vehicle_service_call_interval_vec, line_width=1, line_color='pink', legend_label = 'vehicle_service')
    fig1.line(self.ehr_position_time_vec, self.ehr_position_call_interval_vec, line_width=1, line_color='grey', legend_label = 'ehr_position')
    fig1.line(self.road_fusion_time_vec, self.road_fusion_call_interval_vec, line_width=1, line_color='navy', legend_label = 'road_fusion')
    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'
    bkp.show(fig1, notebook_handle=True)


if __name__ == '__main__':
  localization_plotter = LocalizationApaPlotter()
  localization_plotter.plot_figure()
