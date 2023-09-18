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

kRefLat = 39.907500
kRefLon = 116.3880555
ee = 0.00669342162296594323
a = 6378245.0

latitude0 = kRefLat
longitude0 = kRefLon
altitude0 = 0.0

class LocalizationApaPlotter(object):
  def __init__(self):
  # bag path and frame dt
    self.file_path = '/mnt/noa/20230916/202309163.bag.record'
    self.abnormal_timestamp_path = '/mnt/noa/20230916/abnormal_timestamp.txt'
    self.rtk_file_path = '/mnt/noa/20230914/sensor_navi_navifusion.json'
    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()

    self.start_time = None
    # rtk positions
    self.rtk_latitude_vec = []
    self.rtk_longitude_vec = []
    self.rtk_x_vec = []
    self.rtk_y_vec = []

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
    # AbsolutePostion
    self.absolute_pos_original_loc_timestamp_vec = []
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


  def load_position_data(self):
    ell_wgs84 = pm.Ellipsoid('wgs84')
    # load odem msg
    bag = Record(self.file_path)
    if os.path.exists(self.abnormal_timestamp_path):
      os.remove(self.abnormal_timestamp_path)
    with open(self.abnormal_timestamp_path, 'a') as file:
      for topic, msg, t in bag.read_messages('/iflytek/ehr/position'):
        if self.start_time is None:
          self.start_time = msg.timestamp
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
          self.positions_time_vec.append((msg.timestamp - self.start_time) * 0.001)
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
          self.absolute_pos_original_loc_timestamp_vec.append(msg.absolute_pos[0].original_loc_timestamp)
          self.absolute_pos_time_vec.append((msg.timestamp - self.start_time) * 0.001)
          self.absolute_lon_vec.append(llu_lon)
          self.absolute_lat_vec.append(llu_lat)
          e, n, u = pm.geodetic2enu(llu_lat,llu_lon, llu_alt, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
          self.absolute_x_vec.append(e)
          self.absolute_y_vec.append(n)
        # PostionFailSafe
        if len(msg.fail_safe) != 0:
          self.failsafe_loc_status_vec.append(msg.fail_safe[0].failsafe_loc_status)
          self.failsafe_gnss_status_vec.append(msg.fail_safe[0].failsafe_gnss_status)
          self.failsafe_camera_status_vec.append(msg.fail_safe[0].failsafe_camera_status)
          self.failsafe_hdmap_status_vec.append(msg.fail_safe[0].failsafe_hdmap_status)
          self.failsafe_vehicle_status_vec.append(msg.fail_safe[0].failsafe_vehicle_status)
          self.failsafe_imu_status_vec.append(msg.fail_safe[0].failsafe_imu_status)
          self.failsafe_time_vec.append((msg.timestamp - self.start_time) * 0.001)
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


  def load_rtk_data(self):
    # load rtk localization msg
    ell_wgs84 = pm.Ellipsoid('wgs84')
    with open(self.rtk_file_path, 'r') as f:
      data = json.load(f)
      messages = data['messages']
      for msg in messages:
        latitude = float(msg['latitude'])
        longitude = float(msg['longitude'])
        altitude = float(msg['altitude'])
        if self.is_out_of_china([latitude, longitude, altitude]):
          continue
        self.rtk_latitude_vec.append(latitude)
        self.rtk_longitude_vec.append(longitude)
        e, n, u = pm.geodetic2enu(latitude, longitude, altitude, latitude0, longitude0, altitude0, ell=ell_wgs84, deg=True)
        self.rtk_x_vec.append(e)
        self.rtk_y_vec.append(n)


  def plot_figure(self):
    self.load_position_data()
    # self.load_rtk_data()
    self.plot_data()


  def plot_data(self):
    normal_size = 2
    abnormal_size = 2
    fig1 = bkp.figure(x_axis_label='lat', y_axis_label='lon', width=800, height=300, match_aspect=True, aspect_scale=1.0)
    fig1.circle(self.absolute_lat_vec, self.absolute_lon_vec, size=normal_size, fill_color='red', line_color='red', alpha=0.5, legend_label='absolute_pos')
    fig1.circle(self.abnormal_absolute_lat_vec, self.abnormal_absolute_lon_vec, size=abnormal_size, fill_color='brown', line_color='brown', alpha=0.5, legend_label='abnormal absolute_pos')
    fig1.circle(self.positions_original_lat_vec, self.positions_original_lon_vec, size = normal_size, fill_color='blue', line_color='blue', alpha = 0.5, legend_label = 'positions')
    fig1.circle(self.abnormal_positions_lat_vec, self.abnormal_positions_lon_vec, size = abnormal_size, fill_color='green', line_color='green', alpha = 0.5, legend_label = 'abnormal positions')
    fig1.circle(self.rtk_latitude_vec, self.rtk_longitude_vec, size = normal_size, fill_color='yellow', line_color='yellow', alpha = 0.5, legend_label = 'rtk positions')

    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'
    bkp.show(fig1, notebook_handle=True)

    fig2 = bkp.figure(x_axis_label='x(m)', y_axis_label='y(m)', width=800, height=300, match_aspect=True, aspect_scale=1.0)
    fig2.circle(self.absolute_x_vec, self.absolute_y_vec, size=normal_size, fill_color='red', line_color='red', alpha = 0.5, legend_label='absolute_pos')
    fig2.circle(self.abnormal_absolute_x_vec, self.abnormal_absolute_y_vec, size=abnormal_size, fill_color='brown', line_color='brown', alpha = 0.5, legend_label='abnormal absolute_pos')
    fig2.circle(self.positions_x_vec, self.positions_y_vec, size=normal_size, fill_color='blue', line_color='blue', alpha=0.5, legend_label='positions')
    fig2.circle(self.abnormal_positions_x_vec, self.abnormal_positions_y_vec, size=abnormal_size, fill_color='green', line_color='green', alpha=0.5, legend_label='abnormal positions')
    fig2.circle(self.rtk_x_vec, self.rtk_y_vec, size=normal_size, fill_color='yellow', line_color='yellow', alpha=0.5, legend_label='rtk positions')

    fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
    fig2.legend.click_policy = 'hide'
    bkp.show(fig2, notebook_handle=True)

    time_range = [0, 1000]

    fig3 = bkp.figure(x_axis_label='time', y_axis_label='status', x_range = time_range, y_range = [-1, 5], width=800, height=300)
    fig3.line(self.failsafe_time_vec, self.failsafe_loc_status_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='failsafe_loc_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_gnss_status_vec, line_width=1, line_color='red', line_dash='solid', legend_label='failsafe_gnss_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_camera_status_vec, line_width=1, line_color='yellow', line_dash='solid', legend_label='failsafe_camera_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_hdmap_status_vec, line_width=1, line_color='green', line_dash='solid', legend_label='failsafe_hdmap_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_vehicle_status_vec, line_width=1, line_color='brown', line_dash='solid', legend_label='failsafe_vehicle_status')
    fig3.line(self.failsafe_time_vec, self.failsafe_imu_status_vec, line_width=1, line_color='black', line_dash='solid', legend_label='failsafe_imu_status')

    fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
    fig3.legend.click_policy = 'hide'
    bkp.show(fig3, notebook_handle=True)

    fig4 = bkp.figure(x_axis_label='time', y_axis_label='accuracy(cm)', x_range = time_range, width=800, height=300)
    fig4.line(self.positions_time_vec, self.positions_accuracy_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='accuracy')
    fig4.line(self.positions_time_vec, self.positions_lateral_accuracy_vec, line_width=1, line_color='red', line_dash='solid', legend_label='lateral_accuracy')
    fig4.line(self.positions_time_vec, self.positions_longitudinal_accuracy_vec, line_width=1, line_color='green', line_dash='solid', legend_label='longitudinal_accuracy')

    fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
    fig4.legend.click_policy = 'hide'
    bkp.show(fig4, notebook_handle=True)

    fig5 = bkp.figure(x_axis_label='time', y_axis_label='deviation(cm)', x_range = time_range, width=800, height=300)
    fig5.line(self.positions_time_vec, self.positions_deviation_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='deviation')

    fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
    fig5.legend.click_policy = 'hide'
    bkp.show(fig5, notebook_handle=True)

    fig6 = bkp.figure(x_axis_label='time', y_axis_label='probability', x_range = time_range, width=800, height=300)
    fig6.line(self.positions_time_vec, self.positions_probability_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='probability')

    fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
    fig6.legend.click_policy = 'hide'
    bkp.show(fig6, notebook_handle=True)


if __name__ == '__main__':
  localization_plotter = LocalizationApaPlotter()
  localization_plotter.plot_figure()
