#!/usr/bin/python
# encoding=utf-8

from IPython.core.display import display, HTML
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import bokeh.plotting as bkp
import rosbag


class LocalizationPlotter(object):
  def __init__(self):
    # bag path and frame dt
    self.file_path = '/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240727/20240727-12-53-52/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-07-27-12-53-52_no_camera.bag'

    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()
    # /iflytek/localization/egomotion
    self.egomotion_time_vec = []
    self.egomotion_available_vec = []
    self.egomotion_x_vec = []
    self.egomotion_y_vec = []

    self.egomotion_yaw_vec = []
    self.egomotion_pitch_vec = []
    self.egomotion_roll_vec = []

  def load_egomotion_data(self):
    bag = rosbag.Bag(self.file_path)
    start_egomotion_time = None
    for topic, msg, t in bag.read_messages('/iflytek/localization/egomotion'):
      if start_egomotion_time is None:
        start_egomotion_time = msg.msg_header.timestamp
      time = (msg.msg_header.timestamp - start_egomotion_time) * 1e-6
      self.egomotion_time_vec.append(time)

      self.egomotion_available_vec.append(msg.position.position_boot.available)
      self.egomotion_x_vec.append(msg.position.position_boot.x)
      self.egomotion_y_vec.append(msg.position.position_boot.y)

      self.egomotion_yaw_vec.append(msg.orientation.euler_boot.yaw)
      self.egomotion_pitch_vec.append(msg.orientation.euler_boot.pitch)
      self.egomotion_roll_vec.append(msg.orientation.euler_boot.roll)


  def plot_figure(self):
    self.load_egomotion_data()
    self.plot_data()


  def plot_data(self):
    fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=1500, height=500, match_aspect=True, aspect_scale=1.0)
    fig1.line(self.egomotion_x_vec, self.egomotion_y_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='position_boot')

    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'
    bkp.show(fig1, notebook_handle=True)

    fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta', width=1500, height=500)
    fig2.line(self.egomotion_time_vec, self.egomotion_yaw_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='yaw')
    fig2.line(self.egomotion_time_vec, self.egomotion_pitch_vec, line_width=1, line_color='red', line_dash='solid', legend_label='pitch')
    fig2.line(self.egomotion_time_vec, self.egomotion_roll_vec, line_width=1, line_color='green', line_dash='solid', legend_label='roll')

    fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
    fig2.legend.click_policy = 'hide'
    bkp.show(fig2, notebook_handle=True)


if __name__ == '__main__':
  localization_plotter = LocalizationPlotter()
  localization_plotter.plot_figure()
