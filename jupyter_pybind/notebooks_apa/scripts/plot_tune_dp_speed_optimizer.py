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
from python_proto import common_pb2
from jupyter_pybind import dp_speed_optimizer_py


display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


coord_tf = coord_transformer()

data_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_virtual_wall = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_path_envelop = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_path = ColumnDataSource(data={'plan_path_x': [],
                                              'plan_path_y': [],
                                              'plan_path_heading': [], })

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=400, height=600, match_aspect = True, aspect_scale=1)
# plot speed
pans, lon_plan_data = create_online_lon_plan_figure(fig1)

fig1.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, line_alpha = 0.5,legend_label = 'car',visible = True,fill_alpha = 0.2)
fig1.patches('x_vec', 'y_vec', source = data_path_envelop, fill_color = "black", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'path_envelope', visible = False)
fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start_pos')
fig1.line('plan_path_x', 'plan_path_y', source = data_path, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'path')

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

dp_speed_optimizer_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-10, max=10, value=0.1, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=1.6, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=0, max=360, value=45.0, step=1)
    self.path_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path_length",min=0, max=20, value=4.1, step=0.1)
    self.path_radius_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path_radius",min=-1000.0, max=1000.0, value=-10.0, step=1.0)
    self.ego_v_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_v",min=0.0, max=20.0, value=0.7, step=0.1)
    self.ego_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_acc",min=-10.0, max=10.0, value=0.0, step=0.1)
    self.obs_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "dist_s",min=0.0, max=20.0, value=0.5, step=0.1)
    self.dist_to_obs_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "dist_to_obs",min=-2.0, max=20.0, value=0.3, step=0.1)
    self.max_cruise_speed_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "max_speed",min=-2.0, max=20.0, value=1.0, step=0.1)


    ipywidgets.interact(slider_callback, ego_x=self.ego_x_slider,
                        ego_y=self.ego_y_slider,
                        ego_heading=self.ego_heading_slider,
                        path_length=self.path_length_slider,
                        path_radius=self.path_radius_slider,
                        ego_v=self.ego_v_slider,
                        ego_acc=self.ego_acc_slider,
                        obs_s=self.obs_s_slider,
                        dist_to_obs=self.dist_to_obs_slider,
                        max_cruise_speed=self.max_cruise_speed_slider,
                    )

## sliders callback
def slider_callback(ego_x, ego_y, ego_heading, path_length, path_radius, ego_v, ego_acc, obs_s,dist_to_obs,max_cruise_speed):

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
  data_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # update start point pos
  data_car_start_pos.data.update({
    'x': [ego_x],
    'y': [ego_y],
  })

  ego_pose = [ego_x, ego_y, ego_heading / 57.3]

  current_path_point_global_vec = dp_speed_optimizer_py.Update(
      ego_pose, path_length, path_radius, ego_v, ego_acc,obs_s,dist_to_obs,max_cruise_speed)

  # plot path
  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(current_path_point_global_vec)):
     plan_path_x.append(current_path_point_global_vec[i][0])
     plan_path_y.append(current_path_point_global_vec[i][1])
     plan_path_heading.append(current_path_point_global_vec[i][2])

  data_path.data.update({
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

  dp_speed_constraints = dp_speed_optimizer_py.GetDpSpeedConstraints()
  qp_speed_constraints = dp_speed_optimizer_py.GetQPSpeedConstraints()
  ref_cruise_speed = dp_speed_optimizer_py.GetRefCruiseSpeed()
  dp_speed_data = dp_speed_optimizer_py.GetDPSpeedOptimizationData()
  qp_speed_data = dp_speed_optimizer_py.GetQPSpeedOptimizationData()
  update_lon_plan_online_data(
      dp_speed_constraints,qp_speed_constraints, ref_cruise_speed, dp_speed_data,qp_speed_data, lon_plan_data)

  push_notebook()

bkp.show(row(fig1, pans), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
