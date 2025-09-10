import math
import numpy as np
import sys, os
sys.path.append("..")
from lib.load_local_view_parking import *
from lib.load_lon_plan import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
# from python_proto import common_pb2
from jupyter_pybind import apa_speed_optimizer_py
import argparse
from collections import defaultdict
import re
import shutil
import time
import datetime
from lib.load_common import *


log_path = "/asw/planning/glog/open_space_replay.log.INFO.20250911-163430.971888"

elem_map = {
            'apa_total_time': [], \
            'jlt optimizer time': [], \
            'speed qp time': [], \
            'dp optimizer time': [], \
            'slot manager time': [], \
            'apa_stop_decider': [], \
            'speed_limit_decider': [], \
            'astar_time': [], \
            }

# load log file
def load_log():
  print (log_path)
  input = open(log_path, 'r')
  lines = input.readlines()
  return lines

def get_frame_data_from_line(line):
    x = []

    # .*？ 表示匹配任意个字符到下一个符合条件的字符
    # (.*?) 表示匹配任意个字符到下一个符合条件的字符，且只保留（）中的内容

    # []，匹配集合中的所有字符

    # https://zhuanlan.zhihu.com/p/139596371

    pat = re.compile(r'[(](.*?)[)]', re.S)

    str_list = re.findall(pat, line)
    for string in str_list:
        num = string.split(",")
        x.append(float(num[0]))
    return x

def parse_time(lines):
    for line in lines:
      for key in elem_map.keys():
        if key in line:
          time_ms = get_frame_data_from_line(line)
          if time_ms:
            # print(key,time_ms )
            elem_map[key].append(time_ms[0])

    return

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=1250, height=600, \
                match_aspect = True, aspect_scale=1)

total_time = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = total_time, line_width = 2, \
        line_color = 'green', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'total_time')

jlt_time = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = jlt_time, line_width = 2, \
        line_color = 'black', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'jlt time')

speed_qp_time = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = speed_qp_time, line_width = 2, \
        line_color = 'red', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'speed qp time')

dp_optimizer_time = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = dp_optimizer_time, line_width = 2, \
        line_color = 'purple', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'dp optimizer time')

slot_manager_time = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = slot_manager_time, line_width = 2, \
        line_color = 'orange', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'slot manager time')

apa_stop_decider = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = apa_stop_decider, line_width = 2, \
        line_color = 'yellow', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'apa_stop_decider')

speed_limit_decider = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = speed_limit_decider, line_width = 2, \
        line_color = 'cyan', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'speed_limit_decider')

astar_time = ColumnDataSource(data={'x': [],
                                    'y': [] })
fig1.line('x', 'y', source = astar_time, line_width = 2, \
        line_color = 'black', line_dash = 'solid', line_alpha = 0.5, \
        legend_label = 'astar_time')

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red',\
             legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', \
           line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', \
          text_align='center', text_font_size='15pt', legend_label='measure tool')

# Create a CustomJS callback with the defined code
callback = CustomJS(args=dict(source=source, line_source=line_source,\
                               text_source=text_source), code=callback_code)
# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)
# toolbar
fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
# legend
fig1.legend.click_policy = 'hide'


def update_time():

  # plot path
  x = [x for x in range(len(elem_map['apa_total_time']))]
  # print("size:")
  # print(len(x))
  # size = len(x)

  total_time.data.update({
    'x': x,
    'y': elem_map['apa_total_time'],
  })

  x = [x for x in range(len(elem_map['jlt optimizer time']))]
  jlt_time.data.update({
    'x': x,
    'y': elem_map['jlt optimizer time'],
  })
  print("jlt time size:",len(x))

  x = [x for x in range(len(elem_map['speed qp time']))]
  speed_qp_time.data.update({
    'x': x,
    'y': elem_map['speed qp time'],
  })
  x = [x for x in range(len(elem_map['dp optimizer time']))]
  dp_optimizer_time.data.update({
    'x': x,
    'y': elem_map['dp optimizer time'],
  })

  x = [x for x in range(len(elem_map['slot manager time']))]
  slot_manager_time.data.update({
    'x': x,
    'y': elem_map['slot manager time'],
  })

  x = [x for x in range(len(elem_map['apa_stop_decider']))]
  apa_stop_decider.data.update({
    'x': x,
    'y': elem_map['apa_stop_decider'],
  })

  x = [x for x in range(len(elem_map['speed_limit_decider']))]
  speed_limit_decider.data.update({
    'x': x,
    'y': elem_map['speed_limit_decider'],
  })

  x = [x for x in range(len(elem_map['astar_time']))]
  astar_time.data.update({
    'x': x,
    'y': elem_map['astar_time'],
  })

class LocalViewSlider:
  def __init__(self,  slider_callback):
    ipywidgets.interact(slider_callback)

## sliders callback
def slider_callback():
  kwargs = locals()

  update_time()

  push_notebook()


if __name__ == '__main__':
  lines = load_log()
  parse_time(lines)

  bkp.show(fig1, notebook_handle=True)
  slider_class = LocalViewSlider(slider_callback)
