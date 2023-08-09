#!/usr/bin/python
# encoding=utf-8

from IPython.core.display import display, HTML
from cyber_record.record import Record
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import bokeh.plotting as bkp
import json
import math
import pymap3d as pm


class LocalizationPlotter(object):
  def __init__(self):
    # bag path and frame dt
    self.data_index = 1
    self.gps_file_path = ''
    self.odom_file_path = ''
    # rotate
    rotate_angle = 0.0
    # data1
    if self.data_index == 1:
      self.gps_file_path = '/asw/test_data/0729/2023-07-29-11-21-07_json/sensor_navi_navifusion.json'
      self.odom_file_path = '/asw/test_data/0729/apa_2.00000'
      rotate_angle = 0.042
    # data2
    elif self.data_index == 2:
      self.gps_file_path = '/asw/test_data/0729/2023-07-29-11-52-48_json/sensor_navi_navifusion.json'
      self.odom_file_path = '/asw/test_data/0729/loc_0.00000'
      rotate_angle = 0.012
    # data3
    elif self.data_index == 3:
      self.gps_file_path = '/asw/test_data/0729/2023-07-29-11-56-01_json/sensor_navi_navifusion.json'
      self.odom_file_path = '/asw/test_data/0729/loc_1.00000'
      rotate_angle = 0.052 + math.pi
    self.cos_rotate_angle = math.cos(rotate_angle)
    self.sin_rotate_angle = math.sin(rotate_angle)
    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()
    # gps localization
    self.gps_latitude_vec = []
    self.gps_longitude_vec = []
    self.gps_altitude_vec = []
    self.gps_x_vec = []
    self.gps_y_vec = []
    # odometer localization
    self.odem_x_vec = []
    self.odem_y_vec = []
    # vehicle service
    self.vehicle_service_t_vec = []
    self.vehicle_speed_vec = []
    self.steering_wheel_angle_vec = []
    
  def rotate(self, x, y):
    rotate_x = x * self.cos_rotate_angle - y * self.sin_rotate_angle
    rotate_y = x * self.sin_rotate_angle + y * self.cos_rotate_angle
    return rotate_x, rotate_y


  def load_gps_data(self):
    # load localization msg
    latitude0 = None
    longitude0 = None
    altitude0 = None
    ell_wgs84 = pm.Ellipsoid('wgs84')
    with open(self.gps_file_path, 'r') as f:
      data = json.load(f)
      messages = data['messages']
      for msg in messages:
        latitude = float(msg['latitude'])
        longitude = float(msg['longitude'])
        altitude = float(msg['altitude'])
        # skip abnormal data, threshold should be updated according to test location 
        if (latitude < 31.0 or latitude > 32.0):
          print('abnormal latitude:', latitude)
          continue
        if (longitude < 117.0 or longitude > 118.0):
          print('abnormal longitude:', longitude)
          continue
        if (altitude < 42.0 or altitude > 43.0):
          print('abnormal altitude:', altitude)
          continue
        if latitude0 is None or longitude0 is None or altitude0 is None:
          latitude0 = latitude
          longitude0 = longitude
          altitude0 = altitude
        e, n, u = pm.geodetic2enu(latitude, longitude, altitude, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
        self.gps_latitude_vec.append(latitude)
        self.gps_longitude_vec.append(longitude)
        self.gps_altitude_vec.append(altitude)
        rotate_x, rotate_y = self.rotate(e, n)
        self.gps_x_vec.append(rotate_x)
        self.gps_y_vec.append(rotate_y)


  def load_odem_data(self):
    # load odem msg
    bag = Record(self.odom_file_path)
    for topic, msg, t in bag.read_messages('/iflytek/localization/ego_pose'):
      self.odem_x_vec.append(msg.pose.local_position.x)
      self.odem_y_vec.append(msg.pose.local_position.y)
    start_t = None
    for topic, msg, t in bag.read_messages('/iflytek/vehicle_service'):
      if start_t is None:
        start_t = msg.header.timestamp
      t = (msg.header.timestamp - start_t) / 1e6
      self.vehicle_service_t_vec.append(t)
      self.vehicle_speed_vec.append(msg.vehicle_speed)
      self.steering_wheel_angle_vec.append(msg.steering_wheel_angle)


  def plot_figure(self):
    self.load_gps_data()
    self.load_odem_data()
    self.plot_data()


  def plot_data(self):
    fig1 = bkp.figure(x_axis_label='x(m)', y_axis_label='y(m)', width=800, height=400, match_aspect=True, aspect_scale=1.0)
    fig1.line(self.gps_x_vec, self.gps_y_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='rtk pos')
    fig1.line(self.odem_x_vec, self.odem_y_vec, line_width=1, line_color='red', line_dash='solid', legend_label='odem pos')

    fig2 = bkp.figure(x_axis_label='t(s)', width=800, height=400)
    fig2.line(self.vehicle_service_t_vec, self.vehicle_speed_vec, line_width=1, line_color='red', line_dash='solid', legend_label='vehicle_speed')
    fig2.line(self.vehicle_service_t_vec, self.steering_wheel_angle_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='steering_wheel_angle')

    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'
    bkp.show(fig1, notebook_handle=True)

    fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
    fig2.legend.click_policy = 'hide'
    bkp.show(fig2, notebook_handle=True)

if __name__ == '__main__':
  localization_plotter = LocalizationPlotter()
  localization_plotter.plot_figure()
