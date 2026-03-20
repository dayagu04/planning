#!/usr/bin/python
# encoding=utf-8

import csv
from cyber_record.record import Record
import json
import os

class ConvertPboxMsgToCsv(object):
  def __init__(self):
  # bag path and frame dt
    self.file_path = '/mnt/noa/20230925/output.record'
    self.rtk_file_path_json = '/mnt/noa/20230925/output_json/sensor_navi_navifusion.json'

    self.gnss_data = {
      'gnss_utc_time_year': [],
      'gnss_utc_time_month': [],
      'gnss_utc_time_day': [],
      'gnss_utc_time_hour': [],
      'gnss_utc_time_minute': [],
      'gnss_utc_time_second': [],
      'gnss_utc_time_milli_second': [],
      'gnss_utc_time_time_accuracy': [],
      'gnss_altitude': [],
      'gnss_ellipsoid': [],
      'gnss_quality': [],
      'gnss_pos_status': [],
      'gnss_num_satellites': [],
      'gnss_tdop': [],
      'gnss_hdop': [],
      'gnss_vdop': [],
      'gnss_heading': [],
      'gnss_course': [],
      'gnss_heading_err': [],
      'gnss_course_err': [],
      'gnss_lat': [],
      'gnss_lon': [],
      'gnss_horipos_err': [],
      'gnss_vertpos_err': [],
      'gnss_horivel_err': [],
      'gnss_vertvel_err': [],
      'gnss_vel_north': [],
      'gnss_vel_east': [],
      'gnss_vel_down': [],
    }

    self.imu_data = {
      'temperature_val': [],
      'gyro_selftest_result': [],
      'acc_selftest_result': [],
      'imu_utc_time_year': [],
      'imu_utc_time_month': [],
      'imu_utc_time_day': [],
      'imu_utc_time_hour': [],
      'imu_utc_time_minute': [],
      'imu_utc_time_second': [],
      'imu_utc_time_milli_second': [],
      'imu_utc_time_time_accuracy': [],
      'acc_val_x': [],
      'acc_val_y': [],
      'acc_val_z': [],
      'angular_rate_val_x': [],
      'angular_rate_val_y': [],
      'angular_rate_val_z': [],
    }

    self.cti_data = {
      'gps_time': [],
      'age': [],
      've': [],
      'yaw': [],
      'altitude': [],
      'longitude': [],
      'sat_cnt1': [],
      'warn': [],
      'header_stamp_secs': [],
      'header_stamp_nsecs': [],
      'header_frame_id': [],
      'header_seq': [],
      'imu_data_temperature': [],
      'imu_data_accy': [],
      'imu_data_accx': [],
      'imu_data_accz': [],
      'imu_data_gyroz': [],
      'imu_data_gyrox': [],
      'imu_data_gyroy': [],
      'gps_week': [],
      'meta_timestamp_us': [],
      'vn': [],
      'pitch': [],
      'latitude': [],
      'gps_status': [],
      'speed': [],
      'roll': [],
      'sat_cnt2': [],
      'vu': [],
    }


  def is_out_of_china(self, lla):
    if(not(lla[0] > 0.8293 and lla[0] < 55.8271 and lla[1] > 72.004 and \
              lla[1] < 137.8347)):
      return [0,0,0]


  def load_data(self):
    bag = Record(self.file_path)
    for topic, msg, t in bag.read_messages('/iflytek/sensor/pbox/gnss'):
      self.gnss_data['gnss_utc_time_year'].append(msg.gnss_msg.utc_time.year)
      self.gnss_data['gnss_utc_time_month'].append(msg.gnss_msg.utc_time.month)
      self.gnss_data['gnss_utc_time_day'].append(msg.gnss_msg.utc_time.day)
      self.gnss_data['gnss_utc_time_hour'].append(msg.gnss_msg.utc_time.hour)
      self.gnss_data['gnss_utc_time_minute'].append(msg.gnss_msg.utc_time.minute)
      self.gnss_data['gnss_utc_time_second'].append(msg.gnss_msg.utc_time.second)
      self.gnss_data['gnss_utc_time_milli_second'].append(msg.gnss_msg.utc_time.milli_second)
      self.gnss_data['gnss_utc_time_time_accuracy'].append(msg.gnss_msg.utc_time.time_accuracy)
      self.gnss_data['gnss_altitude'].append(msg.gnss_msg.gnss_altitude)
      self.gnss_data['gnss_ellipsoid'].append(msg.gnss_msg.gnss_ellipsoid)
      self.gnss_data['gnss_quality'].append(msg.gnss_msg.gnss_quality)
      self.gnss_data['gnss_pos_status'].append(msg.gnss_msg.gnss_pos_status)
      self.gnss_data['gnss_num_satellites'].append(msg.gnss_msg.gnss_num_satellites)
      self.gnss_data['gnss_tdop'].append(msg.gnss_msg.gnss_tdop)
      self.gnss_data['gnss_hdop'].append(msg.gnss_msg.gnss_hdop)
      self.gnss_data['gnss_vdop'].append(msg.gnss_msg.gnss_vdop)
      self.gnss_data['gnss_heading'].append(msg.gnss_msg.gnss_heading)
      self.gnss_data['gnss_course'].append(msg.gnss_msg.gnss_course)
      self.gnss_data['gnss_heading_err'].append(msg.gnss_msg.gnss_heading_err)
      self.gnss_data['gnss_course_err'].append(msg.gnss_msg.gnss_course_err)
      self.gnss_data['gnss_lat'].append(msg.gnss_msg.gnss_lat)
      self.gnss_data['gnss_lon'].append(msg.gnss_msg.gnss_lon)
      self.gnss_data['gnss_horipos_err'].append(msg.gnss_msg.gnss_horipos_err)
      self.gnss_data['gnss_vertpos_err'].append(msg.gnss_msg.gnss_vertpos_err)
      self.gnss_data['gnss_horivel_err'].append(msg.gnss_msg.gnss_horivel_err)
      self.gnss_data['gnss_vertvel_err'].append(msg.gnss_msg.gnss_vertvel_err)
      self.gnss_data['gnss_vel_north'].append(msg.gnss_msg.gnss_vel_north)
      self.gnss_data['gnss_vel_east'].append(msg.gnss_msg.gnss_vel_east)
      self.gnss_data['gnss_vel_down'].append(msg.gnss_msg.gnss_vel_down)

    for topic, msg, t in bag.read_messages('/iflytek/sensor/pbox/imu'):
      self.imu_data['temperature_val'].append(msg.temperature_val)
      self.imu_data['gyro_selftest_result'].append(msg.gyro_selftest_result)
      self.imu_data['acc_selftest_result'].append(msg.acc_selftest_result)
      self.imu_data['imu_utc_time_year'].append(msg.imu_utc_time.year)
      self.imu_data['imu_utc_time_month'].append(msg.imu_utc_time.month)
      self.imu_data['imu_utc_time_day'].append(msg.imu_utc_time.day)
      self.imu_data['imu_utc_time_hour'].append(msg.imu_utc_time.hour)
      self.imu_data['imu_utc_time_minute'].append(msg.imu_utc_time.minute)
      self.imu_data['imu_utc_time_second'].append(msg.imu_utc_time.second)
      self.imu_data['imu_utc_time_milli_second'].append(msg.imu_utc_time.milli_second)
      self.imu_data['imu_utc_time_time_accuracy'].append(msg.imu_utc_time.time_accuracy)
      self.imu_data['acc_val_x'].append(msg.acc_val.x)
      self.imu_data['acc_val_y'].append(msg.acc_val.y)
      self.imu_data['acc_val_z'].append(msg.acc_val.z)
      self.imu_data['angular_rate_val_x'].append(msg.angular_rate_val.x)
      self.imu_data['angular_rate_val_y'].append(msg.angular_rate_val.y)
      self.imu_data['angular_rate_val_z'].append(msg.angular_rate_val.z)

    with open(self.rtk_file_path_json, 'r') as f:
      data = json.load(f)
      messages = data['messages']
      for msg in messages:
        latitude = float(msg['latitude'])
        longitude = float(msg['longitude'])
        altitude = float(msg['altitude'])
        if self.is_out_of_china([latitude, longitude, altitude]):
          continue
        self.cti_data['gps_time'].append(float(msg['gps_time']))
        self.cti_data['age'].append(float(msg['age']))
        self.cti_data['ve'].append(float(msg['ve']))
        self.cti_data['yaw'].append(float(msg['yaw']))
        self.cti_data['altitude'].append(float(msg['altitude']))
        self.cti_data['longitude'].append(float(msg['longitude']))
        self.cti_data['sat_cnt1'].append(float(msg['sat_cnt1']))
        self.cti_data['warn'].append(float(msg['warn']))
        self.cti_data['header_stamp_secs'].append(float(msg['header']['stamp']['secs']))
        self.cti_data['header_stamp_nsecs'].append(float(msg['header']['stamp']['nsecs']))
        self.cti_data['header_frame_id'].append(msg['header']['frame_id'])
        self.cti_data['header_seq'].append(float(msg['header']['seq']))
        self.cti_data['imu_data_temperature'].append(float(msg['imu_data']['temperature']))
        self.cti_data['imu_data_accy'].append(float(msg['imu_data']['accy']))
        self.cti_data['imu_data_accx'].append(float(msg['imu_data']['accx']))
        self.cti_data['imu_data_accz'].append(float(msg['imu_data']['accz']))
        self.cti_data['imu_data_gyroz'].append(float(msg['imu_data']['gyroz']))
        self.cti_data['imu_data_gyrox'].append(float(msg['imu_data']['gyrox']))
        self.cti_data['imu_data_gyroy'].append(float(msg['imu_data']['gyroy']))
        self.cti_data['gps_week'].append(float(msg['gps_week']))
        self.cti_data['meta_timestamp_us'].append(float(msg['meta']['timestamp_us']))
        self.cti_data['vn'].append(float(msg['vn']))
        self.cti_data['pitch'].append(float(msg['pitch']))
        self.cti_data['latitude'].append(float(msg['latitude']))
        self.cti_data['gps_status'].append(float(msg['gps_status']))
        self.cti_data['speed'].append(float(msg['speed']))
        self.cti_data['roll'].append(float(msg['roll']))
        self.cti_data['sat_cnt2'].append(float(msg['sat_cnt2']))
        self.cti_data['vu'].append(float(msg['vu']))


  def write_all_data(self):
    self.write_data('gnss_data.csv', self.gnss_data)
    self.write_data('imu_data.csv', self.imu_data)
    self.write_data('cti_data.csv', self.cti_data)

  def write_data(self, file_name, data):
    if os.path.exists(file_name):
      os.remove(file_name)
    file = open(file_name, 'w', encoding='utf-8', newline='')
    csv_writer = csv.DictWriter(file, fieldnames=list(data.keys()))
    csv_writer.writeheader()
    for i in range(len(data[list(data.keys())[0]])):
        dic1 = {key: data[key][i] for key in data.keys()}
        csv_writer.writerow(dic1)
    file.close()


if __name__ == '__main__':
  converter = ConvertPboxMsgToCsv()
  converter.load_data()
  converter.write_all_data()
