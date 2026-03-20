from lib.load_rotate import *
import numpy as np
import time
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label, LabelSet, DataTable, DateFormatter, TableColumn, Panel, Tabs
import ipywidgets as widgets
from IPython.display import display
from ipywidgets import Button, HBox
from IPython.display import clear_output
import time
import threading
import ipywidgets
from collections import namedtuple
from functools import  partial
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool
from bokeh.plotting import ColumnDataSource
from cyber_record.record import Record
from lib.load_json import *
from lib.load_struct import *
from jupyter_pybind.python_proto import planning_debug_info_pb2

coord_tf = coord_transformer()

def update_sk_online_data(curve, plan_data):
  # plot optimization data
  node_s =[]
  node_k =[]
  curve_s =[]
  curve_k =[]

  for i in range(len(curve)):
    # nly plot curve
    if (curve[i][2] < 0.5):
      curve_s.append(curve[i][0])
      curve_k.append(curve[i][1])

    # node include: node circle + spiral + rs + dubins
    node_s.append(curve[i][0])
    node_k.append(curve[i][1])
    # print('s ', s[i], 'k ',  k[i])

  plan_data['data_node_s_kappa'].data.update({
    's': node_s,
    'kappa': node_k,
  })

  plan_data['data_curve_s_kappa'].data.update({
    's': curve_s,
    'kappa': curve_k,
  })


# online data
def create_online_lat_plan_figure(fig1):
  data_node_s_kappa = ColumnDataSource(data = {'s':[], 'kappa':[]})
  data_curve_s_kappa = ColumnDataSource(data = {'s':[], 'kappa':[]})

  plan_data = {'data_node_s_kappa':data_node_s_kappa,
               'data_curve_s_kappa': data_curve_s_kappa,
  }

  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type'),
  ])

  # fig2 S-kappa
  fig_s_kappa = bkp.figure(x_axis_label='s', y_axis_label='kappa',x_range = [-0.1, 30.0], y_range = [-0.2, 0.2], width=500, height=500, match_aspect = True, aspect_scale=1)
  f2 = fig_s_kappa.line('s', 'kappa', source = data_node_s_kappa, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'node kappa')
  f2 = fig_s_kappa.line('s', 'kappa', source = data_curve_s_kappa, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'curve kappa')


  hover5 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('v_lb', '@vel_min_vec'), ('v_ref', '@ref_vel_vec'), ('v_plan', '@vel_vec'), ('v_ub', '@vel_max_vec')], mode='vline')

  fig_s_kappa.add_tools(hover5)

  fig_s_kappa.toolbar.active_scroll = fig_s_kappa.select_one(WheelZoomTool)
  fig_s_kappa.legend.click_policy = 'hide'

  pan1 = Panel(child=row(column(fig_s_kappa)), title="online simulation")


  pans = Tabs(tabs=[ pan1])

  return pans, plan_data



