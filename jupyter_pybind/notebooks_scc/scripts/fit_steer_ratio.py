#!/usr/bin/python
# encoding=utf-8

from IPython.core.display import display, HTML
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import bokeh.plotting as bkp
import numpy as np


class SteerRatioPlotter(object):
  def __init__(self):
    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()
    self.raw_left_dir_steer_angle_vec = [
        38.595561,
        135.984360,
        166.080899,
        197.177322,
        233.573136,
        256.670480,
        290.366604,
        315.163752,
        345.660245,
        375.756783,
        395.254541,
        423.451298,
        446.748561,
        467.646215,
        488.743731,
        510.241316,
        525.039556]
    self.left_dir_left_wheel_angle_vec = [
        2.15,
        8.15,
        10.05,
        12.05,
        14.5,
        16.05,
        18.3,
        20.1,
        22.1,
        24.5,
        25.95,
        28.15,
        30.1,
        31.85,
        33.8,
        35.8,
        37.1]
    self.left_dir_right_wheel_angle_vec = [
        2.05,
        7.9,
        9.7,
        11.55,
        13.80,
        15.15,
        17.1,
        18.6,
        20.35,
        22.25,
        23.35,
        25.1,
        26.5,
        27.75,
        29.05,
        30.35,
        31.25]
    self.raw_right_dir_steer_angle_vec = [
        30.496493,
        60.293066,
        87.389949,
        130.984935,
        160.781508,
        192.077909,
        223.774263,
        268.769031,
        312.164097,
        339.061004,
        365.357979,
        403.853552,
        448.248388,
        476.145237,
        504.841937]
    self.right_dir_left_wheel_angle_vec = [
        1.95,
        3.7,
        5.35,
        7.9,
        9.7,
        11.5,
        13.6,
        16,
        18.75,
        20.25,
        21.85,
        24.15,
        26.8,
        28.45,
        30.2]
    self.right_dir_right_wheel_angle_vec = [
        2.05,
        3.85,
        5.5,
        8.25,
        10.2,
        12.2,
        14.3,
        17.3,
        20.5,
        22.35,
        24.35,
        27.35,
        31,
        33.45,
        36.25]
    self.poly_coef = []
    self.left_dir_steer_angle_with_offset_vec = []
    self.right_dir_steer_angle_with_offset_vec = []
    self.left_dir_avg_wheel_angle_vec = []
    self.right_dir_avg_wheel_angle_vec = []
    self.left_dir_steer_ratio_vec = []
    self.right_dir_steer_ratio_vec = []
    self.all_steer_angle_with_offset_vec = []
    self.all_steer_ratio_vec = []
    self.output_steer_angle_vec = [50.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0, 500.0]
    self.output_steer_ratio_vec = []
    self.left_dir_raw_steer_ratio = []
    self.right_dir_raw_steer_ratio = []

  def process_data(self):
    steer_offset = 3.2
    for steer in self.raw_left_dir_steer_angle_vec:
      self.left_dir_steer_angle_with_offset_vec.append(steer - steer_offset)
    for steer in self.raw_right_dir_steer_angle_vec:
      self.right_dir_steer_angle_with_offset_vec.append(steer + steer_offset)
    for i in range(len(self.left_dir_left_wheel_angle_vec)):
      avg_wheel_ang = (self.left_dir_left_wheel_angle_vec[i] + self.left_dir_right_wheel_angle_vec[i]) * 0.5
      self.left_dir_avg_wheel_angle_vec.append(avg_wheel_ang)
      steer_ratio = self.left_dir_steer_angle_with_offset_vec[i] / avg_wheel_ang
      self.left_dir_steer_ratio_vec.append(steer_ratio)
      raw_raw_steer_ratio = self.raw_left_dir_steer_angle_vec[i] / avg_wheel_ang
      self.left_dir_raw_steer_ratio.append(raw_raw_steer_ratio)
    for i in range(len(self.right_dir_left_wheel_angle_vec)):
      avg_wheel_ang = (self.right_dir_left_wheel_angle_vec[i] + self.right_dir_right_wheel_angle_vec[i]) * 0.5
      self.right_dir_avg_wheel_angle_vec.append(avg_wheel_ang)
      steer_ratio = self.right_dir_steer_angle_with_offset_vec[i] / avg_wheel_ang
      self.right_dir_steer_ratio_vec.append(steer_ratio)
      raw_raw_steer_ratio = self.raw_right_dir_steer_angle_vec[i] / avg_wheel_ang
      self.right_dir_raw_steer_ratio.append(raw_raw_steer_ratio)
    self.all_steer_angle_with_offset_vec = self.left_dir_steer_angle_with_offset_vec + self.right_dir_steer_angle_with_offset_vec
    self.all_steer_ratio_vec = self.left_dir_steer_ratio_vec + self.right_dir_steer_ratio_vec
    self.poly_coef = np.polyfit(self.all_steer_angle_with_offset_vec, self.all_steer_ratio_vec, 3)
    for steer in self.output_steer_angle_vec:
      steer_ratio = ((self.poly_coef[0] * steer + self.poly_coef[1]) * steer + self.poly_coef[2]) * steer + self.poly_coef[3]
      self.output_steer_ratio_vec.append(steer_ratio)
    print('steer ratio:', self.output_steer_ratio_vec)


  def plot_figure(self):
    self.process_data()
    self.plot_data()


  def plot_data(self):
    fig1 = bkp.figure(x_axis_label='steer ang(deg)', y_axis_label='steer ratio', width=800, height=400)
    fig1.line(self.left_dir_steer_angle_with_offset_vec, self.left_dir_steer_ratio_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='left dir steer ratio')
    fig1.line(self.right_dir_steer_angle_with_offset_vec, self.right_dir_steer_ratio_vec, line_width=1, line_color='red', line_dash='solid', legend_label='right dir steer ratio')
    fig1.line(self.output_steer_angle_vec, self.output_steer_ratio_vec, line_width=1, line_color='green', line_dash='solid', legend_label='output steer ratio')

    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    fig1.legend.click_policy = 'hide'
    bkp.show(fig1, notebook_handle=True)

    fig2 = bkp.figure(x_axis_label='steer ang(deg)', y_axis_label='steer ratio', width=800, height=400)
    fig2.line(self.raw_left_dir_steer_angle_vec, self.left_dir_raw_steer_ratio, line_width=1, line_color='blue', line_dash='solid', legend_label='raw left dir steer ratio')
    fig2.line(self.raw_right_dir_steer_angle_vec, self.right_dir_raw_steer_ratio, line_width=1, line_color='red', line_dash='solid', legend_label='raw right dir steer ratio')

    fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
    fig2.legend.click_policy = 'hide'
    bkp.show(fig2, notebook_handle=True)

if __name__ == '__main__':
  steer_ratio_plotter = SteerRatioPlotter()
  steer_ratio_plotter.plot_figure()
