import math
import numpy as np
import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2
from jupyter_pybind import hybrid_astar_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

search_path_display_num = 15


coord_tf = coord_transformer()

data_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
# data_moving_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_target_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_target_line = ColumnDataSource(data = {'y':[], 'x':[]})
data_car_safe_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_astar_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_end_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_safe_circle_tang_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_safe_circle_tang_line = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_rectangle_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_virtual_wall = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_path_envelop = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_pt_inside_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_real_time_node_list = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_search_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
# include node: open set, close set, safe node
data_all_search_node = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_all_search_collision_node = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
rs_heuristic_path = ColumnDataSource(data = {'x':[], 'y':[]})
data_obstacle_points = ColumnDataSource(data = {'x':[], 'y':[]})


data_full_astar_path = ColumnDataSource(data={'plan_path_x': [],
                                              'plan_path_y': [],
                                              'plan_path_heading': [], })

data_planning_tune = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_rs_path = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
data_polynomial_path = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_rs_lib_test = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_veh_circle = ColumnDataSource(data = {'car_circle_xn':[], 'car_circle_yn':[], 'car_circle_rn':[]})

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, line_alpha = 0.5,legend_label = 'car',visible = True,fill_alpha = 0.2)
# fig1.patch('car_xn', 'car_yn', source = data_moving_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'moving_car',visible = False)
# fig1.patch('car_xn', 'car_yn', source = data_car_target_pos, fill_color = "blue", line_color = "red", line_width = 1, line_alpha = 0.5, legend_label = 'car_target_pos')
# fig1.patch('car_xn', 'car_yn', source = data_car_safe_pos, fill_color = "orange", line_color = "red", line_width = 1, line_alpha = 0.5, legend_label = 'car_safe_pos', visible = False)
fig1.line('x', 'y', source = data_car_target_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car_target_line',visible = False)
fig1.patches('x_vec', 'y_vec', source = data_path_envelop, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'veh_body_envelope', visible = False)

fig1.circle('x','y', source = data_pt_inside_pos, size=8, color='green', legend_label = 'pt_inside_pos')

fig1.multi_line('x_vec', 'y_vec', source=data_real_time_node_list, line_width=1.0, line_color='red', line_dash='solid', legend_label='real_time_node_list')



fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start_pos')
fig1.circle('x','y', source = data_car_end_pos, size=8, color='blue', legend_label = 'car_end_pos')
fig1.circle('x','y', source = data_astar_target_pos, size=8, color='orange', legend_label = 'astar_target')
fig1.circle('x','y', source = data_safe_circle_tang_pos, size=8, color='black', legend_label = 'safe_circle_tang_pos', visible = False)
fig1.multi_line('x_vec', 'y_vec',source = data_safe_circle_tang_line, line_width = 3, line_color = 'black', line_dash = 'solid',legend_label = 'safe_circle_tang_line', visible = False)
fig1.multi_line('x_vec', 'y_vec',source = data_slot, line_width = 1.5, line_color = 'black', line_dash = 'solid',legend_label = 'slot')
fig1.multi_line('x_vec', 'y_vec',source = data_rectangle_slot, line_width = 1.5, line_color = 'blue', line_dash = 'solid',legend_label = 'rectangle slot')
fig1.multi_line('x_vec', 'y_vec',source = data_virtual_wall, line_width = 1.5, line_color = 'red', line_dash = 'solid',legend_label = 'virtual_wall')
fig1.multi_line('x', 'y',source = rs_heuristic_path, line_width = 1.5, line_color = 'purple', line_dash = 'solid',legend_label = 'rs_h_path')

fig1.circle('plan_path_x', 'plan_path_y', source = data_full_astar_path, size=4, color='yellow', legend_label = 'circle_astar_path',visible = False)
fig1.circle('x', 'y', source = data_obstacle_points, size=4, color='red', legend_label = 'obstacle_points')
fig1.line('plan_path_x', 'plan_path_y', source = data_full_astar_path, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'sim_tuned_plan')
fig1.line('plan_path_x', 'plan_path_y', source = data_rs_path, line_width = 6, line_color = 'orange', line_dash = 'solid', line_alpha = 0.5, legend_label = 'rs_path')
fig1.line('plan_path_x', 'plan_path_y', source = data_polynomial_path, line_width = 6, line_color = 'purple', line_dash = 'solid', line_alpha = 0.5, legend_label = 'polynomial')
fig1.line('plan_path_x', 'plan_path_y', source = data_rs_lib_test, line_width = 6, line_color = 'black', line_dash = 'solid', line_alpha = 0.5, legend_label = 'rs_lib_test')
fig1.line('x_vec', 'y_vec', source = data_search_path, line_width = 2, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'search_path')
fig1.circle('x_vec', 'y_vec', source = data_all_search_node, size=4, color='black',  legend_label = 'all_search_node')
fig1.circle('x_vec', 'y_vec', source = data_all_search_collision_node, size=4, color='gray',  legend_label = 'all_collision_node')

fig1.circle(x='car_circle_xn', y='car_circle_yn', radius='car_circle_rn', source = data_veh_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'veh_circle', visible = True)


source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

# Define the JavaScript callback code
callback_code = """
    var x = cb_obj.x;
    var y = cb_obj.y;

    source.data['x'].push(x);
    source.data['y'].push(y);

    if (source.data['x'].length > 2) {
        source.data['x'].shift();
        source.data['y'].shift();
        source.data['x'].shift();
        source.data['y'].shift();
    }
    source.change.emit();

    if (source.data['x'].length >= 2) {
        var x1 = source.data['x'][source.data['x'].length - 2];
        var y1 = source.data['y'][source.data['y'].length - 2];
        var x2 = x;
        var y2 = y;
        var x3 = (x1 + x2) / 2;
        var y3 = (y1 + y2) / 2;

        var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        console.log("Distance between the last two points: " + distance);

        distance = distance.toFixed(4);
        text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
        text_source.change.emit();

        line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
        line_source.change.emit();
    }

    if (source.data['x'].length == 1) {
        text_source.data['x'].shift();
        text_source.data['y'].shift();
        text_source.data['text'].shift();
    }
    text_source.change.emit();
"""

# Create a CustomJS callback with the defined code
callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

hybrid_astar_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-10, max=10, value=0.1, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=1.6, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=0, max=360, value=45.0, step=1)

    self.is_left = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_left",min=0, max=1, value=0, step=1)
    self.trigger_plan = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="trigger_plan", min=0, max=1, value=0, step=1)
    self.slot_phi_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_phi",min=45, max=90, value=90, step=15.0)

    self.right_obj_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "right_obj_dx",min=-2.0, max=2.0, value=0.6, step=0.05)
    self.left_virtual_wall_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "left_virtual_wall_x",min=-30.0, max=20.0, value=-12.6, step=0.05)
    self.right_virtual_wall_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "right_virtual_wall_x",min=0.0, max=20.0, value=15, step=0.01)
    self.right_obj_dy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "right_obj_dy",min=0, max=2.0, value=0.6, step=0.05)
    self.left_obj_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "left_obj_dx",min=-2.0, max=2.0, value=0.6, step=0.5)
    self.left_obj_dy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "left_obj_dy",min=0, max=2.0, value=0.6, step=0.05)
    self.channel_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_width",min=3.0, max=20, value=8.8, step=0.1)

    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_width",min=0, max=3, value=2.4, step=0.01)
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_length",min=0, max=6, value=5.0, step=0.01)
    self.inside_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "inside_dx",min=-3, max=3, value=0.0, step=0.1)

    self.slot_pt0_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_pt0_x",min=-10, max=10, value=2.0, step=0.01)
    self.slot_pt0_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_pt0_y",min=-10, max=10, value=-2.0, step=0.01)

    ipywidgets.interact(slider_callback, ego_x=self.ego_x_slider,
                        ego_y=self.ego_y_slider,
                        ego_heading=self.ego_heading_slider,
                        slot_pt0_x=self.slot_pt0_x_slider,
                        slot_pt0_y=self.slot_pt0_y_slider,
                        is_left=self.is_left,
                        trigger_plan=self.trigger_plan,
                        slot_phi=self.slot_phi_slider,
                        slot_width=self.slot_width_slider,
                        slot_length=self.slot_length_slider,
                        inside_dx=self.inside_dx_slider,
                        right_obj_dx=self.right_obj_dx_slider,
                        right_obj_dy=self.right_obj_dy_slider,
                        left_obj_dx=self.left_obj_dx_slider,
                        left_obj_dy=self.left_obj_dy_slider,
                        channel_width=self.channel_width_slider,
                        right_virtual_wall_x=self.right_virtual_wall_x_slider,
                        left_virtual_wall_x=self.left_virtual_wall_x_slider,
                        )

## sliders callback
def slider_callback(ego_x, ego_y, ego_heading, slot_pt0_x, slot_pt0_y, is_left, trigger_plan, slot_phi, slot_width, slot_length, inside_dx, right_obj_dx,
                    right_obj_dy, left_obj_dx, left_obj_dy, channel_width, right_virtual_wall_x, left_virtual_wall_x):
  kwargs = locals()

  # vehicle_type = 'CHERY_T26'
  vehicle_type = 'CHERY_E0X'
  lat_buffer = 0.0
  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, lat_buffer)

  car_xn = []
  car_yn = []

  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(
          car_xb[i], car_yb[i], ego_x, ego_y, ego_heading * math.pi/180.0)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
      print('y', car_yb[i])
  data_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # update start point pos
  data_car_start_pos.data.update({
    'x': [ego_x],
    'y': [ego_y],
  })

  delta_b = 0

  if is_left == 0:
    phi_rad = slot_phi/57.3
    heading_rad = phi_rad
    parallelogram_a = slot_width/math.sin(phi_rad)
    delta_b = slot_width/math.tan(phi_rad)
    parallelogram_b = delta_b + slot_length

    # (x0,y0)-->(x1,y1)
    delta_y1 = 0
    delta_x1 = parallelogram_a
    # (x0,y0)-->(x2,y2)
    delta_y2 = parallelogram_b * math.sin(heading_rad)
    delta_x2 = parallelogram_b * math.cos(heading_rad)
    # (x2,y2)-->(x3,y3)
    delta_y3 = 0
    delta_x3 = parallelogram_a

    slot_pt1_x = slot_pt0_x - delta_x1
    slot_pt1_y = slot_pt0_y + delta_y1

    slot_pt2_x = slot_pt0_x - delta_x2
    slot_pt2_y = slot_pt0_y - delta_y2

    slot_pt3_x = slot_pt2_x - delta_x3
    slot_pt3_y = slot_pt2_y + delta_y3
  else:
    phi_rad = (180 - slot_phi)/57.3
    heading_rad = phi_rad
    parallelogram_a = slot_width/math.sin(180/57.3 - phi_rad)
    delta_b = slot_width/math.tan(180/57.3 - phi_rad)
    parallelogram_b = delta_b + slot_length

    # (x0,y0)-->(x1,y1)
    delta_y1 = 0
    delta_x1 = parallelogram_a
    # (x0,y0)-->(x2,y2)
    delta_y2 = parallelogram_b * math.sin(heading_rad)
    delta_x2 = parallelogram_b * math.cos(heading_rad)
    # (x2,y2)-->(x3,y3)
    delta_y3 = 0
    delta_x3 = parallelogram_a

    slot_pt1_x = slot_pt0_x - delta_x1
    slot_pt1_y = slot_pt0_y + delta_y1

    slot_pt2_x = slot_pt0_x - delta_x2
    slot_pt2_y = slot_pt0_y - delta_y2

    slot_pt3_x = slot_pt2_x - delta_x3
    slot_pt3_y = slot_pt2_y + delta_y3

  slot_bound_x_vec, slot_bound_y_vec = [], []
  slot_bound_x_vec.append([slot_pt0_x, slot_pt2_x])
  slot_bound_y_vec.append([slot_pt0_y, slot_pt2_y])

  slot_bound_x_vec.append([slot_pt2_x, slot_pt3_x])
  slot_bound_y_vec.append([slot_pt2_y, slot_pt3_y])

  slot_bound_x_vec.append([slot_pt3_x, slot_pt1_x])
  slot_bound_y_vec.append([slot_pt3_y, slot_pt1_y])

  slot_bound_x_vec.append([slot_pt1_x, slot_pt0_x])
  slot_bound_y_vec.append([slot_pt1_y, slot_pt0_y])

  print("slot_bound_x_vec", slot_bound_x_vec)
  print("slot_bound_y_vec", slot_bound_y_vec)


  data_slot.data.update({
    'x_vec': slot_bound_x_vec,
    'y_vec': slot_bound_y_vec,})

  ego_pose = [ego_x, ego_y, ego_heading / 57.3]
  slot_pt = [[slot_pt0_x, slot_pt0_y], [slot_pt1_x, slot_pt1_y],
              [slot_pt2_x, slot_pt2_y], [slot_pt3_x, slot_pt3_y],]

  obs_params = [right_obj_dx, right_obj_dy, left_obj_dx, left_obj_dy,
                channel_width, right_virtual_wall_x, left_virtual_wall_x]

  current_path_point_global_vec_ = hybrid_astar_py.Update(
      ego_pose, slot_pt, inside_dx, obs_params, trigger_plan)

  # rs
  data_rs_path.data.update({
      'plan_path_x': [],
      'plan_path_y': [],
      'plan_path_heading': [],
  })

  rs_path = hybrid_astar_py.GetReedsShapePath()

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(rs_path)):
     plan_path_x.append(rs_path[i][0])
     plan_path_y.append(rs_path[i][1])
     plan_path_heading.append(rs_path[i][2])

  data_rs_path.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  # plot polynomial
  polynomial_path = hybrid_astar_py.GetPolynomialPath()

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(polynomial_path)):
     plan_path_x.append(polynomial_path[i][0])
     plan_path_y.append(polynomial_path[i][1])
     plan_path_heading.append(polynomial_path[i][2])

  data_polynomial_path.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  # test rs lib
  data_rs_lib_test.data.update({
      'plan_path_x': [],
      'plan_path_y': [],
      'plan_path_heading': [],
  })

  rs_path = hybrid_astar_py.GetRSLibPath()

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(rs_path)):
     plan_path_x.append(rs_path[i][0])
     plan_path_y.append(rs_path[i][1])
     plan_path_heading.append(rs_path[i][2])

  data_rs_lib_test.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  # open list search path
  data_search_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  path = hybrid_astar_py.GetSearchPathPoint()
  plan_path_x = []
  plan_path_y = []

  for i in range(len(path)):
     if (i > search_path_display_num):
       break
     plan_path_x.append(path[i][0])
     plan_path_y.append(path[i][1])

  data_search_path.data.update({
    'x_vec': plan_path_x,
    'y_vec': plan_path_y
  })

  # all search node
  data_all_search_node.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  nodes = hybrid_astar_py.GetAllSearchNode()
  safe_node_x = []
  safe_node_y = []
  collision_node_x = []
  collision_node_y = []

  for i in range(len(nodes)):
    if (nodes[i][2] > 0.8):
      safe_node_x.append(nodes[i][0])
      safe_node_y.append(nodes[i][1])
    else:
      collision_node_x.append(nodes[i][0])
      collision_node_y.append(nodes[i][1])

  data_all_search_node.data.update({
    'x_vec': safe_node_x,
    'y_vec': safe_node_y
  })
  data_all_search_collision_node.data.update({
    'x_vec': collision_node_x,
    'y_vec': collision_node_y
  })

  # all h rs path
  paths = hybrid_astar_py.GetRSHeuristicPath()
  plan_path_x = []
  plan_path_y = []

  rs_heuristic_path.data.update({
       'x': plan_path_x,
       'y': plan_path_y,
  })

  start=[]
  end=[]

  for k in range(len(paths)):
    for i in range(len(paths[k])-1):
        start = paths[k][i]

        end = paths[k][i+1]

        plan_path_x.append([start[0], end[0]])
        plan_path_y.append([start[1], end[1]])

  rs_heuristic_path.data.update({
      'x': plan_path_x,
      'y': plan_path_y,
  })

  rectangle_solt_pos_vec_ = hybrid_astar_py.GetRectangleSoltPos()

  slot_rectangle_x_vec, slot_rectangle_y_vec = [], []

  if (len(rectangle_solt_pos_vec_) > 1):
    slot_rectangle_x_vec.append([rectangle_solt_pos_vec_[0][0], rectangle_solt_pos_vec_[1][0]])
    slot_rectangle_x_vec.append([rectangle_solt_pos_vec_[1][0], rectangle_solt_pos_vec_[3][0]])
    slot_rectangle_x_vec.append([rectangle_solt_pos_vec_[3][0], rectangle_solt_pos_vec_[2][0]])
    slot_rectangle_x_vec.append([rectangle_solt_pos_vec_[2][0], rectangle_solt_pos_vec_[0][0]])

    slot_rectangle_y_vec.append([rectangle_solt_pos_vec_[0][1], rectangle_solt_pos_vec_[1][1]])
    slot_rectangle_y_vec.append([rectangle_solt_pos_vec_[1][1], rectangle_solt_pos_vec_[3][1]])
    slot_rectangle_y_vec.append([rectangle_solt_pos_vec_[3][1], rectangle_solt_pos_vec_[2][1]])
    slot_rectangle_y_vec.append([rectangle_solt_pos_vec_[2][1], rectangle_solt_pos_vec_[0][1]])

  data_rectangle_slot.data.update({
    'x_vec': slot_rectangle_x_vec,
    'y_vec': slot_rectangle_y_vec,})

  car_end_pose = hybrid_astar_py.GetTargetPose()
  data_car_end_pos.data.update({
    'x': [car_end_pose[0]],
    'y': [car_end_pose[1]],
  })

  pose = hybrid_astar_py.GetAstarEndPose()
  data_astar_target_pos.data.update({
    'x': [pose[0]],
    'y': [pose[1]],
  })

  print(len(current_path_point_global_vec_))


  data_full_astar_path.data.update({
    'plan_path_x': [],
    'plan_path_y': [],
    'plan_path_heading': [],
  })

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(current_path_point_global_vec_)):
     plan_path_x.append(current_path_point_global_vec_[i][0])
     plan_path_y.append(current_path_point_global_vec_[i][1])
     plan_path_heading.append(current_path_point_global_vec_[i][2])

  data_full_astar_path.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  cur_pos_xn0 = 0
  cur_pos_yn0 = 0
  car_xn = []
  car_yn = []
  line_xn = []
  line_yn = []
  car_box_x_vec = []
  car_box_y_vec = []
  if (len(plan_path_x) > 1):
    half_car_width = car_yb[0]
    last_x = plan_path_x[-1]
    last_y = plan_path_y[-1]
    last_heading = plan_path_heading[-1]
    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], last_x, last_y, last_heading)
      car_xn.append(tmp_x - cur_pos_xn0)
      car_yn.append(tmp_y - cur_pos_yn0)

    heading_vec = [math.cos(last_heading), math.sin(last_heading)]
    norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
    norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
    x1 = last_x + norm_vec_1[0]
    y1 = last_y + norm_vec_1[1]
    x2 = last_x + norm_vec_2[0]
    y2 = last_y + norm_vec_2[1]
    line_xn = [x1, x2]
    line_yn = [y1, y2]

  # data_car_target_pos.data.update({
  #   'car_xn': car_xn,
  #   'car_yn': car_yn,
  # })

  # data_car_target_line.data.update({
  #   'x' : line_xn,
  #   'y' : line_yn,
  # })

  # envelop
  for k in range(len(plan_path_x)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], plan_path_x[k], plan_path_y[k], plan_path_heading[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_path_envelop.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  pt_inside_pos = hybrid_astar_py.GetPtInsidePose()

  data_pt_inside_pos.data.update({
    'x' : [pt_inside_pos[0]],
    'y' : [pt_inside_pos[1]],
  })


  safe_circle_tang_pose = hybrid_astar_py.GetCircleTangentPose()
  print('circle_tang_pos handing: ', safe_circle_tang_pose[2] * 57.3)

  extend_pos_x = safe_circle_tang_pose[0] + math.cos(safe_circle_tang_pose[2])
  extend_pos_y = safe_circle_tang_pose[1] + math.sin(safe_circle_tang_pose[2])

  data_safe_circle_tang_pos.data.update({
    'x': [safe_circle_tang_pose[0]],
    'y': [safe_circle_tang_pose[1]],
  })

  tang_line_x_vec = []
  tang_line_y_vec = []
  tang_line_x_vec.append([safe_circle_tang_pose[0], extend_pos_x])
  tang_line_y_vec.append([safe_circle_tang_pose[1], extend_pos_y])

  data_safe_circle_tang_line.data.update({
    'x_vec': tang_line_x_vec,
    'y_vec': tang_line_y_vec,
  })

  #update tangent pos car
  car_xn = []
  car_yn = []
  line_xn = []
  line_yn = []
  car_box_x_vec = []
  car_box_y_vec = []
  if (len(plan_path_x) > 1):
    half_car_width = car_yb[0]
    last_x = safe_circle_tang_pose[0]
    last_y = safe_circle_tang_pose[1]
    last_heading = safe_circle_tang_pose[2]

    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], last_x, last_y, last_heading)
      car_xn.append(tmp_x - cur_pos_xn0)
      car_yn.append(tmp_y - cur_pos_yn0)

    heading_vec = [math.cos(last_heading), math.sin(last_heading)]
    norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
    norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
    x1 = last_x + norm_vec_1[0]
    y1 = last_y + norm_vec_1[1]
    x2 = last_x + norm_vec_2[0]
    y2 = last_y + norm_vec_2[1]
    line_xn = [x1, x2]
    line_yn = [y1, y2]

  # data_car_safe_pos.data.update({
  #   'car_xn': car_xn,
  #   'car_yn': car_yn,
  # })

  # circle obs
  obs_pts = hybrid_astar_py.GetVirtualWallObstacles()
  obs_pt_x, obs_pt_y = [], []
  for i in range(len(obs_pts)):
    obs_pt_x.append(obs_pts[i][0])
    obs_pt_y.append(obs_pts[i][1])

  data_obstacle_points.data.update({
    'x': obs_pt_x,
    'y': obs_pt_y,
  })

  # child node

  # all search node
  line_list_x_vec, line_list_y_vec = [], []
  node_list = hybrid_astar_py.GetAstarAllNodes()
  for i in range(len(node_list)):
    plan_path_x =[]
    plan_path_y =[]
    for j in range(len(node_list[i])):
      path_point = node_list[i][j]
      plan_path_x.append(path_point[0])
      plan_path_y.append(path_point[1])

    line_list_x_vec.append(plan_path_x)
    line_list_y_vec.append(plan_path_y)

  data_real_time_node_list.data.update({
        'x_vec': [],
        'y_vec': [],
    })
  data_real_time_node_list.data.update({
    'x_vec': line_list_x_vec,
    'y_vec': line_list_y_vec,})

  # obs line
  line_list = hybrid_astar_py.GetObsLineList()
  line_list_x_vec, line_list_y_vec = [], []

  for i in range(len(line_list)):
    obs_point_num = len(line_list[i])
    if obs_point_num < 4:
      continue

    line_list_x_vec.append([line_list[i][0], line_list[i][2]])
    line_list_y_vec.append([line_list[i][1], line_list[i][3]])

  print("=========")

  data_virtual_wall.data.update({
    'x_vec': line_list_x_vec,
    'y_vec': line_list_y_vec,})


  # car_xn = []
  # car_yn = []
  # pose = hybrid_astar_py.GetTrajPoseByDist()

  # for i in range(len(car_xb)):
  #     tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], pose[0], pose[1], pose[2])
  #     car_xn.append(tmp_x)
  #     car_yn.append(tmp_y)
  # data_moving_car.data.update({
  #   'car_xn': car_xn,
  #   'car_yn': car_yn,
  # })

  # vehicle_type = 'CHERY_T26'
  footprint_model_global = hybrid_astar_py.GetFootPrintModelGlobal()

  car_circle_xn = []
  car_circle_yn = []
  car_circle_rn = []

  # draw origin car, no expansion
  if 0:
    car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord_by_veh(vehicle_type)
    for i in range(len(car_circle_x)):
      x = ego_pose[0]
      y = ego_pose[1]
      heading = ego_pose[2]

      if i == 1 or i==5:
        tmp_x, tmp_y = local2global(
          car_circle_x[i], car_circle_y[i]+lat_buffer, x, y, heading)
      elif i==2 or i==4:
        tmp_x, tmp_y = local2global(
            car_circle_x[i], car_circle_y[i]-lat_buffer, x, y, heading)
      else:
        tmp_x, tmp_y = local2global(
          car_circle_x[i], car_circle_y[i], x, y, heading)
      car_circle_xn.append(tmp_x)
      car_circle_yn.append(tmp_y)

      if i == 0:
        car_circle_rn.append(car_circle_r[i]+lat_buffer)
      elif i == 3 or i == 6:
        car_circle_rn.append(car_circle_r[i]+lat_buffer)
      elif i ==1 or i==2 or i==4 or i==5:
        car_circle_rn.append(car_circle_r[i])
      else:
        car_circle_rn.append(car_circle_r[i]+lat_buffer)

    data_veh_circle.data.update({
        'car_circle_xn': car_circle_xn,
        'car_circle_yn': car_circle_yn,
        'car_circle_rn': car_circle_rn
    })

  for i in range(len(footprint_model_global)):
    x = ego_pose[0]
    y = ego_pose[1]
    heading = ego_pose[2]

    car_circle_xn.append(footprint_model_global[i][0])
    car_circle_yn.append(footprint_model_global[i][1])
    car_circle_rn.append(footprint_model_global[i][2])

  # astar current gear path target
  pose = hybrid_astar_py.GetCurrentGearPathEnd()
  footprint_model_local = hybrid_astar_py.GetFootPrintModelLocal()
  for i in range(len(footprint_model_local)):
    tmp_x, tmp_y = local2global(
        footprint_model_local[i][0], footprint_model_local[i][1], pose[0], pose[1], pose[2])

    car_circle_xn.append(tmp_x)
    car_circle_yn.append(tmp_y)
    car_circle_rn.append(footprint_model_local[i][2])

  data_veh_circle.data.update({
    'car_circle_xn': car_circle_xn,
    'car_circle_yn': car_circle_yn,
    'car_circle_rn': car_circle_rn,
  })

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
