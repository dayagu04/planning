#!/usr/bin/python
# encoding=utf-8

from IPython.core.display import display, HTML
from cyber_record.record import Record
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import bokeh.plotting as bkp
import json
import math
import os
import pymap3d as pm
import numpy as np
import bisect
import datetime


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
    self.file_path = '/data_cold/abu_zone/autoparse/jac_s811_72kx6/trigger/20240311/20240311-17-40-13/data_collection_JAC_S811_72KX6_EVENT_MANUAL_2024-03-11-17-40-13.record'
    self.abnormal_timestamp_path = '/mnt/noa/abnormal_timestamp.txt'
    self.online_insd_data_file_path = '/data_cold/abu_zone/S811-6/1201-xiawu/1201-1/sensor_navi_navifusion.json'
    self.offline_insd_data_file_path = '/data_cold/abu_zone/S811-6/1201-xiawu/1201-1/test.txt'
    self.loc_truth_val_file_path = '/data_cold/abu_zone/S811-6/1201-xiawu/truth_val/P1.txt'
    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()

    # positions
    self.positions_accuracy_vec = []
    self.positions_lateral_accuracy_vec = []
    self.positions_longitudinal_accuracy_vec = []
    self.positions_original_lon_vec = []
    self.positions_original_lat_vec = []
    self.positions_deviation_vec = []
    self.positions_probability_vec = []
    self.positions_x_vec = []
    self.positions_y_vec = []
    self.positions_time_vec = []
    self.positions_lat_diff_vec = []
    self.abnormal_positons_timestamp = []
    self.positons_header_timestamp_vec = []
    self.positons_timestamp_vec = []

    # AbsolutePostion
    self.absolute_pos_original_loc_timestamp_vec = []
    self.absolute_pos_original_loc_utc_time_vec = []
    self.absolute_lon_vec = []
    self.absolute_lat_vec = []
    self.absolute_x_vec = []
    self.absolute_y_vec = []
    self.absolute_pos_time_vec = []
    # PostionFailSafe
    self.failsafe_loc_status_vec = []
    self.failsafe_gnss_status_vec = []
    self.failsafe_camera_status_vec = []
    self.failsafe_hdmap_status_vec = []
    self.failsafe_vehicle_status_vec = []
    self.failsafe_imu_status_vec = []
    self.failsafe_time_vec = []
    # Abnormal points
    self.abnormal_absolute_x_vec = []
    self.abnormal_absolute_y_vec = []
    self.abnormal_absolute_lat_vec = []
    self.abnormal_absolute_lon_vec = []
    self.abnormal_positions_x_vec = []
    self.abnormal_positions_y_vec = []
    self.abnormal_positions_lat_vec = []
    self.abnormal_positions_lon_vec = []
    # PositionGeofence
    self.position_geofence_time_vec = []
    self.geofence_judge_status_vec = []

    # iflytek localization node
    self.iflytek_local_position_x_vec = []
    self.iflytek_local_position_y_vec = []
    self.iflytek_localization_time_vec = []
    self.iflytek_localization_timestamp_interval_vec = []

    # pbox gnss
    self.pbox_gnss_lon_vec = []
    self.pbox_gnss_lat_vec = []
    self.pbox_gnss_time_vec = []
    self.pbox_gnss_quality_vec = []
    self.pbox_gnss_timestamp_interval_vec = []
    self.pbox_gnss_timestamp_vec = []

    # pbox imu
    self.pbox_imu_timestamp_vec = []
    self.pbox_imu_timestamp_interval_vec = []
    self.pbox_imu_time_vec = []
    self.pbox_imu_acc_x_vec = []
    self.pbox_imu_acc_y_vec = []
    self.pbox_imu_acc_z_vec = []
    self.pbox_imu_yaw_rate_x_vec = []
    self.pbox_imu_yaw_rate_y_vec = []
    self.pbox_imu_yaw_rate_z_vec = []

    # gnss
    self.gnss_lon_vec = []
    self.gnss_lat_vec = []

    # offline insd
    self.offline_insd_utc_time_vec = []
    self.offline_insd_lon_vec = []
    self.offline_insd_lat_vec = []
    self.offline_insd_x_vec = []
    self.offline_insd_y_vec = []
    self.offline_corrected_insd_x_vec = []
    self.offline_corrected_insd_y_vec = []

    # online insd
    self.online_insd_lat_vec = []
    self.online_insd_lon_vec = []
    self.online_insd_x_vec = []
    self.online_insd_y_vec = []
    self.online_corrected_insd_x_vec = []
    self.online_corrected_insd_y_vec = []

    # vehicle_service
    self.vehicle_service_timestamp_vec = []
    self.vehicle_service_time_vec = []
    self.vehicle_service_timestamp_interval_vec = []
    self.vehicle_service_turn_switch_state_vec = []

    # planning
    self.planning_timestamp_vec = []
    self.planning_time_vec = []
    self.planning_timestamp_interval_vec = []

    # control
    self.control_timestamp_vec = []
    self.control_time_vec = []
    self.control_timestamp_interval_vec = []

    # road_fusion
    self.road_fusion_timestamp_vec = []
    self.road_fusion_time_vec = []
    self.road_fusion_timestamp_interval_vec = []

    # object_fusion
    self.object_fusion_timestamp_vec = []
    self.object_fusion_time_vec = []
    self.object_fusion_timestamp_interval_vec = []
    self.cur_lane_source_vec = []

    # perception_lane
    self.perception_lane_timestamp_vec = []
    self.perception_lane_time_vec = []
    self.perception_lane_timestamp_interval_vec = []

    # perception_object
    self.perception_object_timestamp_vec = []
    self.perception_object_time_vec = []
    self.perception_object_timestamp_interval_vec = []

    # localization truth val
    self.loc_truth_val_utc_time_vec = []
    self.loc_truth_val_lat_vec = []
    self.loc_truth_val_lon_vec = []
    self.loc_truth_val_x_vec = []
    self.loc_truth_val_y_vec = []

    # localization error
    self.offline_corrected_insd_x_error_vec = []
    self.offline_corrected_insd_y_error_vec = []
    self.x_error_abnormal_offline_corrected_insd_x_vec = []
    self.y_error_abnormal_offline_corrected_insd_y_vec = []
    self.x_error_abnormal_loc_truth_val_x_vec = []
    self.y_error_abnormal_loc_truth_val_y_vec = []
    self.absolute_x_error_vec = []
    self.absolute_y_error_vec = []

    # FuncStateMachine
    self.function_state_machine_timestamp_vec = []
    self.function_state_machine_time_vec = []
    self.function_state_machine_timestamp_interval_vec = []
    self.current_state_vec = []

  def is_out_of_china(self, lla):
    if(not(lla[0] > 0.8293 and lla[0] < 55.8271 and lla[1] > 72.004 and \
              lla[1] < 137.8347)):
      return [0,0,0]


  def transform_lat(self, lat, lon):
    ret = -100.0 + 2.0 * lon + 3.0 * lat + 0.2 * lat * lat + \
                  0.1 * lon * lat + 0.2 * math.sqrt(math.fabs(lon))
    ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 * math.sin(2.0 * lon * math.pi)) * 2.0 / \
            3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * \
            2.0 / 3.0
    return ret


  def transform_lon(self, lat, lon):
    ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + 0.1 * lon * lat + \
                  0.1 * math.sqrt(math.fabs(lon))
    ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 * math.sin(2.0 * lon *math.pi)) * 2.0 / \
            3.0
    ret += (20.0 * math.sin(lon * math.pi) + 40.0 * math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lon / 12.0 * math.pi) + 300.0 * math.sin(lon / 30.0 * math.pi)) * \
            2.0 / 3.0
    return ret


  def correct_localization_by_external_parameters(self, xyz_in_enu, yaw):
    gnss_to_enu_r = self.roll_pitch_raw_to_r_matrix(0, 0, yaw)
    gnss_to_enu_t_vector = np.array([xyz_in_enu[0], xyz_in_enu[1], xyz_in_enu[2]])
    gnss_to_enu = np.eye(4)
    gnss_to_enu[:3, 3] = gnss_to_enu_t_vector
    gnss_to_enu[:3, :3] = gnss_to_enu_r

    lidar_to_gnss_r = self.roll_pitch_raw_to_r_matrix(0.0059852922245445545, 0.008283748108464804, -0.011274372654086581)
    lidar_to_gnss_t_vector = np.array([-0.178372528506434, 0.8809019984963019, 1.2326251653197116])
    lidar_to_gnss = np.eye(4)
    lidar_to_gnss[:3, 3] = lidar_to_gnss_t_vector
    lidar_to_gnss[:3, :3] = lidar_to_gnss_r

    lidar_to_ego_r = self.roll_pitch_raw_to_r_matrix(-0.009021534911056395, -0.011910021126027534, -1.579902386060273)
    lidar_to_ego_t_vector = np.array([0.8228984002955497, -0.013759881080784415, 1.7031276414135466])
    lidar_to_ego = np.eye(4)
    lidar_to_ego[:3, 3] = lidar_to_ego_t_vector
    lidar_to_ego[:3, :3] = lidar_to_ego_r

    ego_to_lidar = np.eye(4)
    ego_to_lidar_r = np.transpose(lidar_to_ego_r)
    ego_to_lidar_t_vector = np.negative(ego_to_lidar_r) @ lidar_to_ego_t_vector
    ego_to_lidar[:3, 3] = ego_to_lidar_t_vector
    ego_to_lidar[:3, :3] = ego_to_lidar_r

    ego_to_gnss = lidar_to_gnss @ ego_to_lidar

    return (gnss_to_enu @ ego_to_gnss)[:3, 3]


  def roll_pitch_raw_to_r_matrix(self, roll, pitch, yaw):
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    return rz @ ry @ rx


  def gcj02_to_wgs84(self, lla):
    if(self.is_out_of_china(lla)):
      return [lla[0], lla[1], lla[2]]
    else:
      dlat = self.transform_lat(lla[0] - 35.0, lla[1] - 105.0)
      dlon = self.transform_lon(lla[0] - 35.0, lla[1] - 105.0)
      radlat = lla[0] / 180.0 * math.pi
      magic = math.sin(radlat)
      magic = 1 - ee * magic * magic
      sqrtmagic = math.sqrt(magic)
      dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
      dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
      mglat = lla[0] + dlat
      mglon = lla[1] + dlon
      return [lla[0] * 2 - mglat, lla[1] * 2 - mglon, lla[2]]

  def load_iflytek_position_data(self):
    bag = Record(self.file_path)
    start_iflytek_localization_timestamp = None
    last_iflytek_localization_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/localization/ego_pose'):
      if start_iflytek_localization_timestamp is None:
        start_iflytek_localization_timestamp = msg.header.timestamp
      if abs(msg.pose.local_position.x) < 0.0001:
        continue
      self.iflytek_local_position_x_vec.append(msg.pose.local_position.x)
      self.iflytek_local_position_y_vec.append(msg.pose.local_position.y)
      time = (msg.header.timestamp - start_iflytek_localization_timestamp) * 1e-6
      interval = 0.0
      if last_iflytek_localization_timestamp is not None:
        interval = (msg.header.timestamp - last_iflytek_localization_timestamp) * 1e-6
      self.iflytek_localization_timestamp_interval_vec.append(interval)
      self.iflytek_localization_time_vec.append(time)
      last_iflytek_localization_timestamp = msg.header.timestamp
    # print('average time interval:', sum(self.iflytek_localization_timestamp_interval_vec) / len(self.iflytek_localization_timestamp_interval_vec))


  def load_pbox_gnss_data(self):
    bag = Record(self.file_path)
    start_pbox_gnss_timestamp = None
    last_pbox_gnss_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/sensor/pbox/gnss'):
      self.pbox_gnss_timestamp_vec.append(msg.header.timestamp)
      if start_pbox_gnss_timestamp is None:
        start_pbox_gnss_timestamp = msg.header.timestamp
      if(self.is_out_of_china([msg.gnss_msg.gnss_lat, msg.gnss_msg.gnss_lon, 0.0])):
        continue
      self.pbox_gnss_lon_vec.append(msg.gnss_msg.gnss_lon)
      self.pbox_gnss_lat_vec.append(msg.gnss_msg.gnss_lat)
      self.pbox_gnss_quality_vec.append(int(msg.gnss_msg.gnss_quality))
      time = (msg.header.timestamp - start_pbox_gnss_timestamp) * 1e-6
      interval = 0.0
      if last_pbox_gnss_timestamp is not None:
        interval = (msg.header.timestamp - last_pbox_gnss_timestamp) * 1e-6
      self.pbox_gnss_timestamp_interval_vec.append(interval)
      self.pbox_gnss_time_vec.append(time)
      last_pbox_gnss_timestamp = msg.header.timestamp

  def load_pbox_imu_data(self):
    bag = Record(self.file_path)
    start_pbox_imu_timestamp = None
    last_pbox_imu_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/sensor/pbox/imu'):
      if start_pbox_imu_timestamp is None:
        start_pbox_imu_timestamp = msg.header.timestamp
      self.pbox_imu_timestamp_vec.append(msg.header.timestamp)
      self.pbox_imu_acc_x_vec.append(msg.acc_val.x)
      self.pbox_imu_acc_y_vec.append(msg.acc_val.y)
      self.pbox_imu_acc_z_vec.append(msg.acc_val.z)
      self.pbox_imu_yaw_rate_x_vec.append(msg.angular_rate_val.x)
      self.pbox_imu_yaw_rate_y_vec.append(msg.angular_rate_val.y)
      self.pbox_imu_yaw_rate_z_vec.append(msg.angular_rate_val.z)
      time = (msg.header.timestamp - start_pbox_imu_timestamp) * 1e-6
      interval = 0.0
      if last_pbox_imu_timestamp is not None:
        interval = (msg.header.timestamp - last_pbox_imu_timestamp) * 1e-6
      self.pbox_imu_time_vec.append(time)
      self.pbox_imu_timestamp_interval_vec.append(interval)
      last_pbox_imu_timestamp = msg.header.timestamp


  def load_gnss_data(self):
    bag = Record(self.file_path)
    for topic, msg, t in bag.read_messages('/iflytek/sensor/gnss'):
      self.gnss_lon_vec.append(msg.gnss_msg.new_longitude)
      self.gnss_lat_vec.append(msg.gnss_msg.new_latitude)


  def load_timestamp_data(self):
    self.load_vehicle_service_data()
    self.load_planning_data()
    self.load_control_data()
    self.load_road_fusion_data()
    self.load_object_fusion_data()
    self.load_perception_lane_data()
    self.load_perception_object_data()


  def load_vehicle_service_data(self):
    bag = Record(self.file_path)
    start_vehicle_service_timestamp = None
    last_vehicle_service_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/vehicle_service'):
      self.vehicle_service_timestamp_vec.append(msg.header.timestamp)
      if start_vehicle_service_timestamp is None:
        start_vehicle_service_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_vehicle_service_timestamp) * 1e-6
      self.vehicle_service_time_vec.append(time)
      interval = 0.0
      if last_vehicle_service_timestamp is not None:
        interval = (msg.header.timestamp - last_vehicle_service_timestamp) * 1e-6
      self.vehicle_service_timestamp_interval_vec.append(interval)
      self.vehicle_service_turn_switch_state_vec.append(msg.turn_switch_state)
      last_vehicle_service_timestamp = msg.header.timestamp


  def load_planning_data(self):
    bag = Record(self.file_path)
    start_planning_timestamp = None
    last_planning_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/planning/plan'):
      self.planning_timestamp_vec.append(msg.meta.header.timestamp)
      if start_planning_timestamp is None:
        start_planning_timestamp = msg.meta.header.timestamp
      time = (msg.meta.header.timestamp - start_planning_timestamp) * 1e-6
      self.planning_time_vec.append(time)
      interval = 0.0
      if last_planning_timestamp is not None:
        interval = (msg.meta.header.timestamp - last_planning_timestamp) * 1e-6
      self.planning_timestamp_interval_vec.append(interval)
      last_planning_timestamp = msg.meta.header.timestamp


  def load_control_data(self):
    bag = Record(self.file_path)
    start_control_timestamp = None
    last_control_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/control/control_command'):
      self.control_timestamp_vec.append(msg.header.timestamp)
      if start_control_timestamp is None:
        start_control_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_control_timestamp) * 1e-6
      self.control_time_vec.append(time)
      interval = 0.0
      if last_control_timestamp is not None:
        interval = (msg.header.timestamp - last_control_timestamp) * 1e-6
      self.control_timestamp_interval_vec.append(interval)
      last_control_timestamp = msg.header.timestamp


  def load_road_fusion_data(self):
    bag = Record(self.file_path)
    start_road_fusion_timestamp = None
    last_road_fusion_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/fusion/road_fusion'):
      self.road_fusion_timestamp_vec.append(msg.header.timestamp)
      if start_road_fusion_timestamp is None:
        start_road_fusion_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_road_fusion_timestamp) * 1e-6
      self.road_fusion_time_vec.append(time)
      interval = 0.0
      if last_road_fusion_timestamp is not None:
        interval = (msg.header.timestamp - last_road_fusion_timestamp) * 1e-6
      self.road_fusion_timestamp_interval_vec.append(interval)
      for ref_line_msg in msg.reference_line_msg:
        if ref_line_msg.relative_id == 0:
          self.cur_lane_source_vec.append(ref_line_msg.lane_sources[0].source)
          break
      last_road_fusion_timestamp = msg.header.timestamp


  def load_object_fusion_data(self):
    bag = Record(self.file_path)
    start_object_fusion_timestamp = None
    last_object_fusion_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/fusion/objects'):
      self.object_fusion_timestamp_vec.append(msg.header.timestamp)
      if start_object_fusion_timestamp is None:
        start_object_fusion_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_object_fusion_timestamp) * 1e-6
      self.object_fusion_time_vec.append(time)
      interval = 0.0
      if last_object_fusion_timestamp is not None:
        interval = (msg.header.timestamp - last_object_fusion_timestamp) * 1e-6
      self.object_fusion_timestamp_interval_vec.append(interval)
      last_object_fusion_timestamp = msg.header.timestamp


  def load_perception_lane_data(self):
    bag = Record(self.file_path)
    start_perception_lane_timestamp = None
    last_perception_lane_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/camera_perception/lane_lines'):
      self.perception_lane_timestamp_vec.append(msg.header.timestamp)
      if start_perception_lane_timestamp is None:
        start_perception_lane_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_perception_lane_timestamp) * 1e-6
      self.perception_lane_time_vec.append(time)
      interval = 0.0
      if last_perception_lane_timestamp is not None:
        interval = (msg.header.timestamp - last_perception_lane_timestamp) * 1e-6
      self.perception_lane_timestamp_interval_vec.append(interval)
      last_perception_lane_timestamp = msg.header.timestamp


  def load_perception_object_data(self):
    bag = Record(self.file_path)
    start_perception_object_timestamp = None
    last_perception_object_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/camera_perception/objects'):
      self.perception_object_timestamp_vec.append(msg.header.timestamp)
      if start_perception_object_timestamp is None:
        start_perception_object_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_perception_object_timestamp) * 1e-6
      self.perception_object_time_vec.append(time)
      interval = 0.0
      if last_perception_object_timestamp is not None:
        interval = (msg.header.timestamp - last_perception_object_timestamp) * 1e-6
      self.perception_object_timestamp_interval_vec.append(interval)
      last_perception_object_timestamp = msg.header.timestamp


  def load_offline_insd_data(self):
      file_object = open(self.offline_insd_data_file_path, 'r')
      lines = file_object.readlines()
      ell_wgs84 = pm.Ellipsoid('wgs84')
      for line in lines:
        if line.startswith("2290"):
            strs = line.split(",")
            lat = float(strs[2])
            lon = float(strs[3])
            utc_time = strs[-1].strip().split(':')
            time = float(utc_time[0]) * 3600.0 + float(utc_time[1]) * 60.0 + float(utc_time[2]) 
            self.offline_insd_utc_time_vec.append(time)
            heading = float(strs[8])
            self.offline_insd_lat_vec.append(lat)
            self.offline_insd_lon_vec.append(lon)
            e, n, u = pm.geodetic2enu(lat, lon, 0.0, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
            x, y, z = self.correct_localization_by_external_parameters([e, n, u], heading * math.pi / 180.0)
            self.offline_insd_x_vec.append(e)
            self.offline_insd_y_vec.append(n)
            self.offline_corrected_insd_x_vec.append(x)
            self.offline_corrected_insd_y_vec.append(y)


  def load_ehr_position_data(self):
    ell_wgs84 = pm.Ellipsoid('wgs84')
    bag = Record(self.file_path)
    if os.path.exists(self.abnormal_timestamp_path):
      os.remove(self.abnormal_timestamp_path)
    with open(self.abnormal_timestamp_path, 'a') as file:
      msg_index = 0
      abnormal_position_index = 0
      last_lat = None
      last_lon = None
      ehr_start_time = None
      for topic, msg, t in bag.read_messages('/iflytek/ehr/position'):
        msg_index = msg_index + 1
        if ehr_start_time is None:
          ehr_start_time = msg.timestamp
        # positions
        is_positions_valid = False
        is_absolute_valid = False
        if len(msg.positions) != 0:
          if abs(msg.positions[0].original_lat) > 90.0:
            continue
          llu_lat, llu_lon, llu_alt = self.gcj02_to_wgs84([msg.positions[0].original_lat, msg.positions[0].original_lon, 0.0])
          if(self.is_out_of_china([llu_lat, llu_lon, llu_alt])):
            continue
          is_positions_valid = True
          self.positions_accuracy_vec.append(msg.positions[0].accuracy)
          self.positions_lateral_accuracy_vec.append(msg.positions[0].lateral_accuracy)
          self.positions_longitudinal_accuracy_vec.append(msg.positions[0].longitudinal_accuracy)
          self.positions_deviation_vec.append(msg.positions[0].deviation)
          self.positions_probability_vec.append(msg.positions[0].probability)
          self.positions_time_vec.append((msg.timestamp - ehr_start_time) * 0.001)
          self.positions_original_lon_vec.append(llu_lon)
          self.positions_original_lat_vec.append(llu_lat)
          e, n, u = pm.geodetic2enu(llu_lat,llu_lon, llu_alt, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
          self.positions_x_vec.append(e)
          self.positions_y_vec.append(n)
        # AbsolutePostion
        if len(msg.absolute_pos) != 0:
          if abs(msg.absolute_pos[0].lat) > 90.0:
            continue
          llu_lat, llu_lon, llu_alt = self.gcj02_to_wgs84([msg.absolute_pos[0].lat, msg.absolute_pos[0].lon, 0.0])
          if(self.is_out_of_china([llu_lat, llu_lon, llu_alt])):
            continue
          is_absolute_valid = True
          self.absolute_pos_original_loc_timestamp_vec.append(msg.absolute_pos[0].original_loc_timestamp * 1000)
          self.positons_header_timestamp_vec.append(msg.header.timestamp * 1000)
          self.positons_timestamp_vec.append(msg.timestamp * 1000)
          self.absolute_pos_time_vec.append((msg.timestamp - ehr_start_time) * 0.001)
          self.absolute_lon_vec.append(llu_lon)
          self.absolute_lat_vec.append(llu_lat)
          e, n, u = pm.geodetic2enu(llu_lat,llu_lon, llu_alt, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
          self.absolute_x_vec.append(e)
          self.absolute_y_vec.append(n)
          dt_object = datetime.datetime.fromtimestamp(msg.absolute_pos[0].original_loc_timestamp * 0.001)
          utc_time = str(dt_object).split(' ')[1].split(':')
          time = (float(utc_time[0]) - 8.0) * 3600.0 + float(utc_time[1]) * 60.0 + float(utc_time[2])
          self.absolute_pos_original_loc_utc_time_vec.append(time)
          if len(self.absolute_x_vec) >= 3:
            x1 = self.absolute_x_vec[-2] - self.absolute_x_vec[-3]
            y1 = self.absolute_y_vec[-2] - self.absolute_y_vec[-3]
            x2 = self.absolute_x_vec[-1] - self.absolute_x_vec[-2]
            y2 = self.absolute_y_vec[-1] - self.absolute_y_vec[-2]
            len_v1 = math.sqrt(x1 * x1 + y1 * y1)
            diff = 0.0
            if len_v1 > 1e-6:
              diff = (x1 * y2 - y1 * x2) / len_v1
            self.positions_lat_diff_vec.append(diff)
            if abs(diff) > 0.1 and msg_index - abnormal_position_index > 1:
              # print("timestmap:", msg.timestamp, "lateral jumpiness: ", diff,
              #     ", lat:", msg.absolute_pos[0].lat, ", lon:", msg.absolute_pos[0].lon)
              abnormal_position_index = msg_index
        last_lat = msg.absolute_pos[0].lat
        last_lon = msg.absolute_pos[0].lon
        # PostionFailSafe
        if len(msg.fail_safe) != 0:
          self.failsafe_loc_status_vec.append(msg.fail_safe[0].failsafe_loc_status)
          self.failsafe_gnss_status_vec.append(msg.fail_safe[0].failsafe_gnss_status)
          self.failsafe_camera_status_vec.append(msg.fail_safe[0].failsafe_camera_status)
          self.failsafe_hdmap_status_vec.append(msg.fail_safe[0].failsafe_hdmap_status)
          self.failsafe_vehicle_status_vec.append(msg.fail_safe[0].failsafe_vehicle_status)
          self.failsafe_imu_status_vec.append(msg.fail_safe[0].failsafe_imu_status)
          self.failsafe_time_vec.append((msg.timestamp - ehr_start_time) * 0.001)
          if self.failsafe_loc_status_vec[-1] != 0:
            file.writelines(str(msg.timestamp)+'\n')
            if is_positions_valid:
              self.abnormal_positions_x_vec.append(self.positions_x_vec[-1])
              self.abnormal_positions_y_vec.append(self.positions_y_vec[-1])
              self.abnormal_positions_lat_vec.append(self.positions_original_lat_vec[-1])
              self.abnormal_positions_lon_vec.append(self.positions_original_lon_vec[-1])
            if is_absolute_valid:
              self.abnormal_absolute_x_vec.append(self.absolute_x_vec[-1])
              self.abnormal_absolute_y_vec.append(self.absolute_y_vec[-1])
              self.abnormal_absolute_lat_vec.append(self.absolute_lat_vec[-1])
              self.abnormal_absolute_lon_vec.append(self.absolute_lon_vec[-1])
        if len(msg.position_geofence) != 0:
          self.geofence_judge_status_vec.append(msg.position_geofence[0].geofence_judge_status)


  def load_online_insd_data(self):
    ell_wgs84 = pm.Ellipsoid('wgs84')
    with open(self.online_insd_data_file_path, 'r') as f:
      data = json.load(f)
      messages = data['messages']
      for msg in messages:
        latitude = float(msg['latitude'])
        longitude = float(msg['longitude'])
        altitude = float(msg['altitude'])
        heading = float(msg['yaw'])
        if self.is_out_of_china([latitude, longitude, altitude]):
          continue
        self.online_insd_lat_vec.append(latitude)
        self.online_insd_lon_vec.append(longitude)
        e, n, u = pm.geodetic2enu(latitude, longitude, 0.0, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
        x, y, z = self.correct_localization_by_external_parameters([e, n, u], heading * math.pi / 180.0)
        self.online_insd_x_vec.append(e)
        self.online_insd_y_vec.append(n)
        self.online_corrected_insd_x_vec.append(x)
        self.online_corrected_insd_y_vec.append(y)


  def load_loc_truth_val(self):
    ell_wgs84 = pm.Ellipsoid('wgs84')
    with open(self.loc_truth_val_file_path, 'r') as file:
      lines = file.readlines()
      for line in lines:
        if line.startswith('12/01/2023'):
          strs = line.split(",")
          latitude = float(strs[3])
          longitude = float(strs[4])
          if self.is_out_of_china([latitude, longitude, 0.0]):
            continue
          self.loc_truth_val_lat_vec.append(latitude)
          self.loc_truth_val_lon_vec.append(longitude)
          utc_time = strs[1].split(':')
          time = (float(utc_time[0]) - 8.0) * 3600.0 + float(utc_time[1]) * 60.0 + float(utc_time[2])
          self.loc_truth_val_utc_time_vec.append(time)
          e, n, u = pm.geodetic2enu(latitude, longitude, 0.0, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
          self.loc_truth_val_x_vec.append(e)
          self.loc_truth_val_y_vec.append(n)


  def cal_offline_insd_loc_error(self):
    offline_insd_utc_time_vec_len = len(self.offline_insd_utc_time_vec)
    loc_truth_val_utc_time_vec_len = len(self.loc_truth_val_utc_time_vec)
    for i in range(offline_insd_utc_time_vec_len):
      offline_insd_time = self.offline_insd_utc_time_vec[i]
      index = bisect.bisect_left(self.loc_truth_val_utc_time_vec, offline_insd_time)
      if index == loc_truth_val_utc_time_vec_len:
        break
      if index == 0:
        continue
      pre_index = index - 1
      time_diff = self.loc_truth_val_utc_time_vec[index] - self.loc_truth_val_utc_time_vec[pre_index]
      if time_diff > 0.03:
        continue
      v1_x = self.loc_truth_val_x_vec[index] - self.loc_truth_val_x_vec[pre_index]
      v1_y = self.loc_truth_val_y_vec[index] - self.loc_truth_val_y_vec[pre_index]
      v2_x = self.offline_corrected_insd_x_vec[i] - self.loc_truth_val_x_vec[pre_index]
      v2_y = self.offline_corrected_insd_y_vec[i] - self.loc_truth_val_y_vec[pre_index]
      len_v1 = math.sqrt(v1_x * v1_x + v1_y * v1_y)
      if len_v1 < 0.01:
        continue
      x_error = (v1_x * v2_x + v1_y * v2_y) / len_v1
      y_error = (v1_x * v2_y - v1_y * v2_x) / len_v1
      self.offline_corrected_insd_x_error_vec.append(x_error)
      self.offline_corrected_insd_y_error_vec.append(y_error)
      if x_error > 1.0 or y_error > 1.0:
        self.x_error_abnormal_offline_corrected_insd_x_vec.append(self.offline_corrected_insd_x_vec[i])
        self.y_error_abnormal_offline_corrected_insd_y_vec.append(self.offline_corrected_insd_y_vec[i])
        self.x_error_abnormal_loc_truth_val_x_vec.append(self.loc_truth_val_x_vec[index])
        self.y_error_abnormal_loc_truth_val_y_vec.append(self.loc_truth_val_y_vec[index])


  def cal_ehr_loc_error(self):
    absolute_pos_original_loc_utc_time_vec_len = len(self.absolute_pos_original_loc_utc_time_vec)
    loc_truth_val_utc_time_vec_len = len(self.loc_truth_val_utc_time_vec)
    for i in range(absolute_pos_original_loc_utc_time_vec_len):
      absolute_pos_original_loc_utc_time = self.absolute_pos_original_loc_utc_time_vec[i]
      index = bisect.bisect_left(self.loc_truth_val_utc_time_vec, absolute_pos_original_loc_utc_time)
      if index == loc_truth_val_utc_time_vec_len:
        break
      if index == 0:
        continue
      pre_index = index - 1
      time_diff = self.loc_truth_val_utc_time_vec[index] - self.loc_truth_val_utc_time_vec[pre_index]
      if time_diff > 0.03:
        continue
      v1_x = self.loc_truth_val_x_vec[index] - self.loc_truth_val_x_vec[pre_index]
      v1_y = self.loc_truth_val_y_vec[index] - self.loc_truth_val_y_vec[pre_index]
      v2_x = self.absolute_x_vec[i] - self.loc_truth_val_x_vec[pre_index]
      v2_y = self.absolute_y_vec[i] - self.loc_truth_val_y_vec[pre_index]
      len_v1 = math.sqrt(v1_x * v1_x + v1_y * v1_y)
      if len_v1 < 0.01:
        continue
      x_error = (v1_x * v2_x + v1_y * v2_y) / len_v1
      y_error = (v1_x * v2_y - v1_y * v2_x) / len_v1
      self.absolute_x_error_vec.append(x_error)
      self.absolute_y_error_vec.append(y_error)


  def load_function_state_machine_data(self):
    bag = Record(self.file_path)
    start_function_state_machine_timestamp = None
    last_function_state_machine_timestamp = None
    for topic, msg, t in bag.read_messages('/iflytek/system_state/soc_state'):
      self.function_state_machine_timestamp_vec.append(msg.header.timestamp)
      if start_function_state_machine_timestamp is None:
        start_function_state_machine_timestamp = msg.header.timestamp
      time = (msg.header.timestamp - start_function_state_machine_timestamp) * 1e-6
      self.function_state_machine_time_vec.append(time)
      interval = 0.0
      if last_function_state_machine_timestamp is not None:
        interval = (msg.header.timestamp - last_function_state_machine_timestamp) * 1e-6
      self.function_state_machine_timestamp_interval_vec.append(interval)
      last_function_state_machine_timestamp = msg.header.timestamp
      self.current_state_vec.append(msg.current_state)


  def plot_figure(self):
    self.load_ehr_position_data()
    # self.load_pbox_imu_data()
    # self.load_iflytek_position_data()
    # self.load_pbox_gnss_data()
    # self.load_gnss_data()
    # self.load_online_insd_data()
    # self.load_rtk_data()
    self.load_timestamp_data()
    # self.load_offline_insd_data()
    # self.load_loc_truth_val()
    # self.cal_offline_insd_loc_error()
    # self.cal_ehr_loc_error()
    self.load_function_state_machine_data()
    self.plot_data()


  def plot_data(self):
    normal_size = 2
    abnormal_size = 2
    fig1 = bkp.figure(x_axis_label='lat', y_axis_label='lon', width=1000, height=500, match_aspect=True, aspect_scale=1.0)
    fig1.circle(self.absolute_lat_vec, self.absolute_lon_vec, size=normal_size, fill_color='red', line_color='red', alpha=0.5, legend_label='absolute_pos')
    fig1.circle(self.absolute_lat_vec[0], self.absolute_lon_vec[0], size=10, fill_color='red', line_color='red', alpha=0.5, legend_label='absolute_pos')
    fig1.line(self.absolute_lat_vec, self.absolute_lon_vec, line_width=1, line_color='red', legend_label = 'absolute_pos')
    fig1.circle(self.abnormal_absolute_lat_vec, self.abnormal_absolute_lon_vec, size=abnormal_size, fill_color='brown', line_color='brown', alpha=0.5, legend_label='abnormal absolute_pos')
    fig1.circle(self.positions_original_lat_vec, self.positions_original_lon_vec, size = normal_size, fill_color='blue', line_color='blue', alpha = 0.5, legend_label = 'positions')
    fig1.line(self.positions_original_lat_vec, self.positions_original_lon_vec, line_width=1, line_color='blue', legend_label = 'positions')
    fig1.circle(self.abnormal_positions_lat_vec, self.abnormal_positions_lon_vec, size = abnormal_size, fill_color='green', line_color='green', alpha = 0.5, legend_label = 'abnormal positions')
    # fig1.circle(self.online_insd_lat_vec, self.online_insd_lon_vec, size = normal_size, fill_color='indigo', line_color='indigo', alpha = 0.5, legend_label = 'insd raw positions')
    # fig1.line(self.online_insd_lat_vec, self.online_insd_lon_vec, line_width=1, line_color='indigo', legend_label = 'insd raw positions')
    # fig1.circle(self.pbox_gnss_lat_vec, self.pbox_gnss_lon_vec, size = normal_size, fill_color='gold', line_color='gold', alpha = 0.5, legend_label = 'pbox gnss positions')
    # fig1.line(self.pbox_gnss_lat_vec, self.pbox_gnss_lon_vec, line_width=1, line_color='gold', legend_label = 'pbox gnss positions')
    # fig1.circle(self.offline_insd_lat_vec, self.offline_insd_lon_vec, size = normal_size, fill_color='cyan', line_color='cyan', alpha = 0.5, legend_label = 'offline insd positions')
    # fig1.line(self.offline_insd_lat_vec, self.offline_insd_lon_vec, line_width=1, line_color='cyan', legend_label = 'offline insd positions')
    # fig1.circle(self.online_insd_lat_vec, self.online_insd_lon_vec, size = normal_size, fill_color='green', line_color='green', alpha = 0.5, legend_label = 'online insd positions')
    # fig1.line(self.online_insd_lat_vec, self.online_insd_lon_vec, line_width=1, line_color='green', legend_label = 'online insd positions')
    # fig1.circle(self.gnss_lat_vec, self.gnss_lon_vec, size = normal_size, fill_color='pink', line_color='pink', alpha = 0.5, legend_label = 'gnss positions')
    # fig1.line(self.gnss_lat_vec, self.gnss_lon_vec, line_width=1, line_color='pink', legend_label = 'gnss positions')
    # fig1.circle(self.loc_truth_val_lat_vec, self.loc_truth_val_lon_vec, size = normal_size, fill_color='pink', line_color='pink', alpha = 0.5, legend_label = 'truth val positions')
    # fig1.line(self.loc_truth_val_lat_vec, self.loc_truth_val_lon_vec, line_width=1, line_color='pink', legend_label = 'truth val positions')

    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'
    bkp.show(fig1, notebook_handle=True)

    fig2 = bkp.figure(x_axis_label='x(m)', y_axis_label='y(m)', width=1000, height=500, match_aspect=True, aspect_scale=1.0)
    fig2.circle(self.absolute_x_vec, self.absolute_y_vec, size=normal_size, fill_color='red', line_color='red', alpha = 0.5, legend_label='absolute_pos')
    fig2.circle(self.absolute_x_vec[0], self.absolute_y_vec[0], size=10, fill_color='red', line_color='red', alpha = 0.5, legend_label='absolute_pos')
    fig2.line(self.absolute_x_vec, self.absolute_y_vec, line_width=1, line_color='red', legend_label = 'absolute_pos')
    fig2.circle(self.abnormal_absolute_x_vec, self.abnormal_absolute_y_vec, size=abnormal_size, fill_color='brown', line_color='brown', alpha = 0.5, legend_label='abnormal absolute_pos')
    fig2.circle(self.positions_x_vec, self.positions_y_vec, size=normal_size, fill_color='blue', line_color='blue', alpha=0.5, legend_label='positions')
    fig2.line(self.positions_x_vec, self.positions_y_vec, line_width=1, line_color='blue', legend_label = 'positions')
    fig2.circle(self.abnormal_positions_x_vec, self.abnormal_positions_y_vec, size=abnormal_size, fill_color='green', line_color='green', alpha=0.5, legend_label='abnormal positions')
    # fig2.circle(self.online_insd_x_vec, self.online_insd_y_vec, size=normal_size, fill_color='indigo', line_color='indigo', alpha=0.5, legend_label='insd raw positions')
    # fig2.line(self.online_insd_x_vec, self.online_insd_y_vec, line_width=1, line_color='indigo', legend_label = 'insd raw positions')
    # fig2.circle(self.iflytek_local_position_x_vec, self.iflytek_local_position_y_vec, size=normal_size, fill_color='cyan', line_color='cyan', alpha=0.5, legend_label='ilfytek positions')
    # fig2.line(self.iflytek_local_position_x_vec, self.iflytek_local_position_y_vec, line_width=1, line_color='cyan', legend_label = 'ilfytek positions')
    # fig2.circle(self.offline_insd_x_vec, self.offline_insd_y_vec, size=normal_size, fill_color='cyan', line_color='cyan', alpha=0.5, legend_label='offline insd positions')
    # fig2.line(self.offline_insd_x_vec, self.offline_insd_y_vec, line_width=1, line_color='cyan', legend_label = 'offline insd positions')
    # fig2.circle(self.offline_corrected_insd_x_vec, self.offline_corrected_insd_y_vec, size=normal_size, fill_color='lime', line_color='lime', alpha=0.5, legend_label='offline corrected insd positions')
    # fig2.line(self.offline_corrected_insd_x_vec, self.offline_corrected_insd_y_vec, line_width=1, line_color='lime', legend_label = 'offline corrected insd positions')
    # fig2.circle(self.loc_truth_val_x_vec, self.loc_truth_val_y_vec, size=normal_size, fill_color='pink', line_color='pink', alpha=0.5, legend_label='truth val positions')
    # fig2.line(self.loc_truth_val_x_vec, self.loc_truth_val_y_vec, line_width=1, line_color='pink', legend_label = 'truth val positions')
    # fig2.circle(self.online_insd_x_vec, self.online_insd_y_vec, size=normal_size, fill_color='green', line_color='green', alpha=0.5, legend_label='online insd positions')
    # fig2.line(self.online_insd_x_vec, self.online_insd_y_vec, line_width=1, line_color='green', legend_label = 'online insd positions')
    # fig2.circle(self.online_corrected_insd_x_vec, self.online_corrected_insd_y_vec, size=normal_size, fill_color='indigo', line_color='indigo', alpha=0.5, legend_label='online corrected insd positions')
    # fig2.line(self.online_corrected_insd_x_vec, self.online_corrected_insd_y_vec, line_width=1, line_color='indigo', legend_label = 'online corrected insd positions')

    fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
    fig2.legend.click_policy = 'hide'
    bkp.show(fig2, notebook_handle=True)

    delta_time = 200
    if len(self.iflytek_localization_time_vec) != 0:
      delta_time = self.iflytek_localization_time_vec[-1]
    time_range = [0, delta_time]

    fig3 = bkp.figure(x_axis_label='time', y_axis_label='status', y_range = [-1, 5], width=1000, height=500)
    fig3.line(self.failsafe_time_vec, self.failsafe_loc_status_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='failsafe_loc_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_gnss_status_vec, line_width=1, line_color='red', line_dash='solid', legend_label='failsafe_gnss_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_camera_status_vec, line_width=1, line_color='yellow', line_dash='solid', legend_label='failsafe_camera_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_hdmap_status_vec, line_width=1, line_color='green', line_dash='solid', legend_label='failsafe_hdmap_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_vehicle_status_vec, line_width=1, line_color='brown', line_dash='solid', legend_label='failsafe_vehicle_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_imu_status_vec, line_width=1, line_color='black', line_dash='solid', legend_label='failsafe_imu_status')

    fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
    fig3.legend.click_policy = 'hide'
    bkp.show(fig3, notebook_handle=True)

    fig4 = bkp.figure(x_axis_label='time', y_axis_label='accuracy(cm)', width=1000, height=500)
    fig4.line(self.positions_time_vec, self.positions_accuracy_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='accuracy')
    fig4.line(self.positions_time_vec, self.positions_lateral_accuracy_vec, line_width=1, line_color='red', line_dash='solid', legend_label='lateral_accuracy')
    fig4.line(self.positions_time_vec, self.positions_longitudinal_accuracy_vec, line_width=1, line_color='green', line_dash='solid', legend_label='longitudinal_accuracy')

    fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
    fig4.legend.click_policy = 'hide'
    bkp.show(fig4, notebook_handle=True)

    fig5 = bkp.figure(x_axis_label='time', y_axis_label='deviation(cm)', width=1000, height=500)
    fig5.line(self.positions_time_vec, self.positions_deviation_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='deviation')

    fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
    fig5.legend.click_policy = 'hide'
    bkp.show(fig5, notebook_handle=True)

    # fig6 = bkp.figure(x_axis_label='time', y_axis_label='probability', width=1000, height=500)
    # fig6.line(self.positions_time_vec, self.positions_probability_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='probability')

    fig6 = bkp.figure(x_axis_label='time', y_axis_label='geofence', width=1000, height=500)
    fig6.line(self.positions_time_vec, self.geofence_judge_status_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='geofence_judge_status')

    fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
    fig6.legend.click_policy = 'hide'
    bkp.show(fig6, notebook_handle=True)

    fig7 = bkp.figure(x_axis_label='time', y_axis_label='time interval', width=1000, height=500)
    fig7.line(self.iflytek_localization_time_vec, self.iflytek_localization_timestamp_interval_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='iflytek loc time interval')
    fig7.line(self.pbox_gnss_time_vec, self.pbox_gnss_timestamp_interval_vec, line_width=1, line_color='red', line_dash='solid', legend_label='pbox gnss time interval')
    fig7.line(self.pbox_imu_time_vec, self.pbox_imu_timestamp_interval_vec, line_width=1, line_color='green', line_dash='solid', legend_label='pbox imu time interval')
    fig7.line(self.planning_time_vec, self.planning_timestamp_interval_vec, line_width=1, line_color='brown', line_dash='solid', legend_label='planning time interval')
    fig7.line(self.control_time_vec, self.control_timestamp_interval_vec, line_width=1, line_color='green', line_dash='solid', legend_label='control time interval')
    fig7.line(self.road_fusion_time_vec, self.road_fusion_timestamp_interval_vec, line_width=1, line_color='indigo', line_dash='solid', legend_label='road_fusion time interval')
    fig7.line(self.object_fusion_time_vec, self.object_fusion_timestamp_interval_vec, line_width=1, line_color='gold', line_dash='solid', legend_label='object_fusion time interval')
    fig7.line(self.perception_lane_time_vec, self.perception_lane_timestamp_interval_vec, line_width=1, line_color='cyan', line_dash='solid', legend_label='perception_lane time interval')
    fig7.line(self.perception_object_time_vec, self.perception_object_timestamp_interval_vec, line_width=1, line_color='lime', line_dash='solid', legend_label='perception_object time interval')

    fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)
    fig7.legend.click_policy = 'hide'
    bkp.show(fig7, notebook_handle=True)

    fig8 = bkp.figure(x_axis_label='time', y_axis_label='timestamp', width=1000, height=500)
    fig8.line(self.absolute_pos_time_vec, self.absolute_pos_original_loc_timestamp_vec, line_width=1, line_color='red', line_dash='solid', legend_label='absolute_pos_original_loc')
    fig8.line(self.absolute_pos_time_vec, self.positons_header_timestamp_vec, line_width=1, line_color='green', line_dash='solid', legend_label='positons_header_timestamp')
    fig8.line(self.absolute_pos_time_vec, self.positons_timestamp_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='positons_timestamp')
    fig8.line(self.vehicle_service_time_vec, self.vehicle_service_timestamp_vec, line_width=1, line_color='brown', line_dash='solid', legend_label='vehicle_service_timestampc')
    fig8.line(self.pbox_imu_time_vec, self.pbox_imu_timestamp_vec, line_width=1, line_color='indigo', line_dash='solid', legend_label='pbox_imu_timestamp')
    fig8.line(self.pbox_gnss_time_vec, self.pbox_gnss_timestamp_vec, line_width=1, line_color='gold', line_dash='solid', legend_label='pbox_gnss_timestamp')
    fig8.toolbar.active_scroll = fig8.select_one(WheelZoomTool)
    fig8.legend.click_policy = 'hide'
    bkp.show(fig8, notebook_handle=True)

    fig9 = bkp.figure(x_axis_label='time', y_axis_label='position diff', width=1000, height=500)
    fig9.line(self.absolute_pos_time_vec[2:], self.positions_lat_diff_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='position diff')

    fig9.toolbar.active_scroll = fig9.select_one(WheelZoomTool)
    fig9.legend.click_policy = 'hide'
    bkp.show(fig9, notebook_handle=True)
    '''
    fig10 = bkp.figure(x_axis_label='time', y_axis_label='pbox gnss_quality', y_range = [-1, 6], width=1000, height=500)
    fig10.line(self.pbox_gnss_time_vec, self.pbox_gnss_quality_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='gnss_quality')

    fig10.toolbar.active_scroll = fig10.select_one(WheelZoomTool)
    fig10.legend.click_policy = 'hide'
    bkp.show(fig10, notebook_handle=True)

    fig11 = bkp.figure(x_axis_label='time', y_axis_label='pbox imu acc', width=1000, height=500)
    fig11.line(self.pbox_imu_time_vec, self.pbox_imu_acc_x_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='acc x')
    fig11.line(self.pbox_imu_time_vec, self.pbox_imu_acc_y_vec, line_width=1, line_color='red', line_dash='solid', legend_label='acc y')
    fig11.line(self.pbox_imu_time_vec, self.pbox_imu_acc_z_vec, line_width=1, line_color='green', line_dash='solid', legend_label='acc z')

    fig11.toolbar.active_scroll = fig11.select_one(WheelZoomTool)
    fig11.legend.click_policy = 'hide'
    bkp.show(fig11, notebook_handle=True)

    fig12 = bkp.figure(x_axis_label='time', y_axis_label='pbox imu yaw rate', width=1000, height=500)
    fig12.line(self.pbox_imu_time_vec, self.pbox_imu_yaw_rate_x_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='yaw rate x')
    fig12.line(self.pbox_imu_time_vec, self.pbox_imu_yaw_rate_y_vec, line_width=1, line_color='red', line_dash='solid', legend_label='yaw rate y')
    fig12.line(self.pbox_imu_time_vec, self.pbox_imu_yaw_rate_z_vec, line_width=1, line_color='green', line_dash='solid', legend_label='yaw rate z')

    fig12.toolbar.active_scroll = fig12.select_one(WheelZoomTool)
    fig12.legend.click_policy = 'hide'
    bkp.show(fig12, notebook_handle=True)

    fig13 = bkp.figure(x_axis_label='time', y_axis_label='offline insd error', width=1000, height=500)
    fig13.line(np.arange(len(self.offline_corrected_insd_x_error_vec)), self.offline_corrected_insd_x_error_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='x error')
    fig13.line(np.arange(len(self.offline_corrected_insd_y_error_vec)), self.offline_corrected_insd_y_error_vec, line_width=1, line_color='red', line_dash='solid', legend_label='y error')

    fig13.toolbar.active_scroll = fig13.select_one(WheelZoomTool)
    fig13.legend.click_policy = 'hide'
    bkp.show(fig13, notebook_handle=True)

    fig14 = bkp.figure(x_axis_label='lat', y_axis_label='lon', width=1000, height=500, match_aspect=True, aspect_scale=1.0)
    fig14.circle(self.x_error_abnormal_offline_corrected_insd_x_vec, self.y_error_abnormal_offline_corrected_insd_y_vec, size = normal_size, fill_color='blue', line_color='blue', alpha = 0.5, legend_label = 'offline_corrected_insd')
    fig14.line(self.x_error_abnormal_offline_corrected_insd_x_vec, self.y_error_abnormal_offline_corrected_insd_y_vec, line_width=1, line_color='blue', legend_label = 'offline_corrected_insd')
    fig14.circle(self.x_error_abnormal_loc_truth_val_x_vec, self.y_error_abnormal_loc_truth_val_y_vec, size = normal_size, fill_color='red', line_color='red', alpha = 0.5, legend_label = 'loc_truth_val')
    fig14.line(self.x_error_abnormal_loc_truth_val_x_vec, self.y_error_abnormal_loc_truth_val_y_vec, line_width=1, line_color='red', legend_label = 'loc_truth_val')

    fig14.toolbar.active_scroll = fig14.select_one(WheelZoomTool)
    fig14.legend.click_policy = 'hide'
    bkp.show(fig14, notebook_handle=True)

    fig15 = bkp.figure(x_axis_label='time', y_axis_label='ehr error', width=1000, height=500)
    fig15.line(np.arange(len(self.absolute_x_error_vec)), self.absolute_x_error_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='x error')
    fig15.line(np.arange(len(self.absolute_y_error_vec)), self.absolute_y_error_vec, line_width=1, line_color='red', line_dash='solid', legend_label='y error')

    fig15.toolbar.active_scroll = fig15.select_one(WheelZoomTool)
    fig15.legend.click_policy = 'hide'
    bkp.show(fig15, notebook_handle=True)
    '''
    fig13 = bkp.figure(x_axis_label='time', y_axis_label='current_state', width=1000, height=500)
    fig13.line(self.function_state_machine_time_vec, self.current_state_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='current_state')

    fig13.toolbar.active_scroll = fig13.select_one(WheelZoomTool)
    fig13.legend.click_policy = 'hide'
    bkp.show(fig13, notebook_handle=True)

    fig14 = bkp.figure(x_axis_label='time', y_axis_label='cur_lane_source', width=1000, height=500, match_aspect=True, aspect_scale=1.0)
    fig14.line(self.road_fusion_time_vec, self.cur_lane_source_vec, line_width=1, line_color='blue', legend_label = 'cur_lane_source')

    fig14.toolbar.active_scroll = fig14.select_one(WheelZoomTool)
    fig14.legend.click_policy = 'hide'
    bkp.show(fig14, notebook_handle=True)

    fig15 = bkp.figure(x_axis_label='time', y_axis_label='turn_switch_state(1:left, 2:right)', width=1000, height=500, match_aspect=True, aspect_scale=1.0)
    fig15.line(self.vehicle_service_time_vec, self.vehicle_service_turn_switch_state_vec, line_width=1, line_color='blue', legend_label = 'turn_switch_state')

    fig15.toolbar.active_scroll = fig15.select_one(WheelZoomTool)
    fig15.legend.click_policy = 'hide'
    bkp.show(fig15, notebook_handle=True)


if __name__ == '__main__':
  localization_plotter = LocalizationApaPlotter()
  localization_plotter.plot_figure()
