import sys, os
sys.path.append("..")

sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

from lib.load_local_view_parking import *
from bokeh.events import Tap
from bokeh.models import Range1d
from bokeh.io import export_svgs
from jupyter_pybind import parallel_planning_py
from bokeh.models import SingleIntervalTicker
import os
import sys
# sys.path.append('/root/miniconda3/lib/python3.7/site-packages')

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

car_xb, car_yb = load_car_params_patch_parking(vehicle_type = CHERY_E0X, car_lat_inflation = 0.0)
coord_tf = coord_transformer()

data_fix_car_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_fix_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_start_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_target_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_PA = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BT = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_tag_point = ColumnDataSource(data = {'x':[], 'y':[]})

data_channel = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_obs_pt = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_virtual_obs_pt = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_debug_arc = ColumnDataSource(data = {'cx_vec':[],
                                        'cy_vec':[],
                                        'radius_vec':[],
                                        'pBx_vec':[],
                                        'pBy_vec':[],
                                        'pCx_vec':[],
                                        'pCy_vec':[]})



fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=800, height=600, match_aspect = True, aspect_scale=1)

fig1.x_range.flipped = False
fig1.xgrid.grid_line_color = None
fig1.ygrid.grid_line_color = None
fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度
fig1.legend.location = 'bottom_right'

fig1.x_range = Range1d(start = -1.0, end = 13.0)
fig1.y_range = Range1d(start = -3.0, end = 8.0)

fig1.xaxis.axis_label_text_font_size = '12pt'
fig1.xaxis.axis_label_text_font = 'Times New Roman'
fig1.yaxis.axis_label_text_font_size = '12pt'
fig1.yaxis.axis_label_text_font = 'Times New Roman'

fig1.xaxis.ticker = SingleIntervalTicker(interval = 3, num_minor_ticks=0)
fig1.yaxis.ticker = SingleIntervalTicker(interval = 3, num_minor_ticks=0)

# 尝试确保图表内容比例一致
aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (fig1.y_range.end - fig1.y_range.start)
fig1.plot_height = int(fig1.plot_width / aspect_ratio)

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

fig1.line('x_vec','y_vec',source =data_path,  line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'car_path')
fig1.patches('x_vec', 'y_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 0.5, legend_label = 'sampled carbox')
## t-lane
fig1.line('x_vec', 'y_vec',source = data_slot, line_width = 1, line_color = 'black', line_dash = 'solid',legend_label = 'slot')
fig1.line('x_vec','y_vec',source =data_channel,  line_width = 1, line_color = 'black', line_dash = 'solid',legend_label = 'channel')

# car box at start pose
fig1.circle('x', 'y', source = data_start_pos, size=8, color='red', legend_label = 'start_pos')
fig1.patch( 'car_xn', 'car_yn', source = data_start_car, fill_color = "palegreen", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'start_pos')
# fix car
fig1.patch( 'car_xn', 'car_yn', source = data_fix_car, fill_color = "yellow", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'fix car', visible =False)
fig1.circle('x', 'y', source = data_fix_car_pos, size= 8, color='green', legend_label = 'fix car', visible = False)

fig1.circle('x', 'y', source = data_tag_point, size=8, color='grey', legend_label = 'tag point')

# target pose
fig1.circle('x', 'y', source = data_target_pos, size=8, color='green', legend_label = 'target_pos')
fig1.patch( 'car_xn', 'car_yn', source = data_target_car, fill_color = "red", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'target_pos')
# obstacles
fig1.scatter("x_vec", "y_vec", source=data_obs_pt, size=8, color='blue',legend_label = 'obstacles')
fig1.scatter("x_vec", "y_vec", source=data_virtual_obs_pt, size=8, color='red', marker='star', legend_label = 'virtual obstacles')

fig1.circle(x = 'cx_vec', y = 'cy_vec', radius = 'radius_vec', source = data_debug_arc, line_alpha = 1, line_width = 2, line_color = "red",
            fill_alpha=0, legend_label = 'data_debug_arc', visible = False)

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

parallel_planning_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    # ego pose
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-15, max=15, value= 9.64, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value= 4.36, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value= 0.0, step=0.1)
    self.s_init_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_init",min=-10.0, max=10.0, value=0.0, step=0.01)
    # obs pt pos
    self.obs_tlane_p_inside_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_in_x",min=-15, max=15, value= 6.0, step=0.01)
    self.obs_tlane_p_inside_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_in_y",min=-15, max=15, value= -0.868469, step=0.01)
    self.obs_tlane_p_outside_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_out_x",min=-15, max=15, value= -0.2, step=0.01)
    self.obs_tlane_p_outside_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_out_y",min=-15, max=15, value=-1.00849, step=0.01)
    # slot pt
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot length",min=5.0, max=8.0, value=6.0, step=0.01)
    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "width",min=2.0, max=3.0, value=2.4, step=0.01)
    self.set_left_slot_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set left slot",min=0, max=1, value=0, step=1)

    # terminal pos
    self.tlane_pt_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_x",min=-2.0, max=2.0, value= 1.52, step=0.01)
    self.tlane_pt_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_y",min=-2.0, max=2.0, value= 0.0, step=0.01)
    # channel
    self.channel_max_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_x",min=3.0, max=20.0, value=20, step=0.1)
    self.channel_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_y",min=-30.0, max=30.0, value = -6.2, step=0.1)
    self.curb_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "curb_offset",min=-5.0, max=5.0, value=1.47454, step=0.1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path ds",min=0.025, max=1.0, value=0.1, step=0.025)
    self.obs_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs ds",min=0.025, max=1.0, value=0.38, step=0.025)

    self.fix_car_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "fix_car",min=-20.0, max=20.0, value=6.27911, step=0.01)
    self.fix_car_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "fix_car",min=-20.0, max=20.0, value= 5.29415, step=0.01)
    self.fix_car_heading_deg_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "fix_car",min=-180.0, max=180.0, value=18.2098, step=0.01)
    self.fix_result_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "fix_result",min=0, max=1, value=0, step=1)


    self.set_start_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_start",min=0, max=1, value=0, step=1)
    self.reset_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "reset_target",min=0, max=1, value=0, step=1)


    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=1, step=1)
    self.is_plan_first_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_plan_first",min=0, max=1, value=1, step=1)

    self.ref_gear_drive_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "dirve",min=0, max=1, value=1, step=1)
    self.ref_steer_left_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "left",min=0, max=1, value=0, step=1)


    ipywidgets.interact(slider_callback, ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         s_init = self.s_init_slider,

                                         obs_tlane_p_outside_x = self.obs_tlane_p_outside_x_slider,
                                         obs_tlane_p_outside_y = self.obs_tlane_p_outside_y_slider,
                                         obs_tlane_p_inside_x = self.obs_tlane_p_inside_x_slider,
                                         obs_tlane_p_inside_y = self.obs_tlane_p_inside_y_slider,

                                         slot_length = self.slot_length_slider,
                                         slot_width = self.slot_width_slider,
                                         set_left_slot = self.set_left_slot_slider,

                                         tlane_pt_x = self.tlane_pt_x_slider,
                                         tlane_pt_y = self.tlane_pt_y_slider,

                                         channel_x = self.channel_max_x_slider,
                                         channel_y = self.channel_y_slider,
                                         curb_y = self.curb_y_slider,

                                         ds = self.ds_slider,
                                         obs_ds = self.obs_ds_slider,

                                         fix_car_x = self.fix_car_x_slider,
                                         fix_car_y = self.fix_car_y_slider,
                                         fix_car_heading_deg = self.fix_car_heading_deg_slider,
                                         fix_result = self.fix_result_slider,

                                         is_complete_path = self.is_complete_path_slider,
                                         is_plan_first = self.is_plan_first_slider,
                                         ref_gear_drive = self.ref_gear_drive_slider,
                                         ref_steer_left = self.ref_steer_left_slider)

### sliders callback
def slider_callback(ego_x, ego_y, ego_heading,s_init,
                    obs_tlane_p_outside_x, obs_tlane_p_outside_y,
                    obs_tlane_p_inside_x, obs_tlane_p_inside_y,
                    slot_length, slot_width, set_left_slot,
                    tlane_pt_x, tlane_pt_y,
                    channel_x, channel_y,curb_y,
                    ds,obs_ds,fix_car_x, fix_car_y,fix_car_heading_deg,  fix_result,
                    is_complete_path,is_plan_first, ref_gear_drive,ref_steer_left):
  kwargs = locals()

  slot_side_sgn = 1.0
  if set_left_slot:
    slot_side_sgn = -1.0

  half_slot_width_sgn = 0.5 * slot_width * slot_side_sgn

  slot_x_vec = [0.0, 0.0, slot_length, slot_length, 0.0]
  slot_y_vec = [half_slot_width_sgn, -half_slot_width_sgn, -half_slot_width_sgn, half_slot_width_sgn, half_slot_width_sgn]

  data_slot.data.update({
     'x_vec': slot_x_vec,
     'y_vec': slot_y_vec,})

  # channel
  channel_y = slot_side_sgn * abs(channel_y)
  data_channel.data.update({'y_vec':[channel_y,channel_y],'x_vec':[obs_tlane_p_outside_x, channel_x]})


  # ego start position
  data_start_pos.data.update({'x':[ego_x],'y':[ego_y]})
  data_target_pos.data.update({'x':[tlane_pt_x],'y':[tlane_pt_y]})

  x_start = ego_x + s_init * math.cos(ego_heading / 57.2958)
  y_start = ego_y + s_init * math.sin(ego_heading / 57.2958)

  # ego car
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], x_start, y_start,
                                  ego_heading / 57.2958)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)

  data_start_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # target car
  target_car_xn = []
  target_car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], tlane_pt_x, tlane_pt_y, 0.0)
      target_car_xn.append(tmp_x)
      target_car_yn.append(tmp_y)
  data_target_car.data.update({
    'car_xn': target_car_xn,
    'car_yn': target_car_yn,
  })


  # fix car
  fix_car_xn = []
  fix_car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], fix_car_x, fix_car_y,
                                  fix_car_heading_deg /  57.2958)
      fix_car_xn.append(tmp_x)
      fix_car_yn.append(tmp_y)

  data_fix_car.data.update({
    'car_xn': fix_car_xn,
    'car_yn': fix_car_yn,
  })

  data_fix_car_pos.data.update({
    'x': [fix_car_x],
    'y': [fix_car_y],
  })

  if fix_result == 0:
    parallel_planning_py.Update(ego_x, ego_y, ego_heading / 57.3,
                                obs_tlane_p_inside_x, obs_tlane_p_inside_y,
                                obs_tlane_p_outside_x, obs_tlane_p_outside_y,
                                slot_length, slot_width,
                                tlane_pt_x, tlane_pt_y,
                                channel_x, channel_y, curb_y,
                                ds, obs_ds, is_complete_path,
                                is_plan_first, set_left_slot,
                                ref_gear_drive,ref_steer_left)
  # clear
  data_path.data.update({
    'x_vec': [],
    'y_vec': [],
    'theta_vec': [],
  })

  path_x_vec = parallel_planning_py.GetPathEle(0)
  path_y_vec = parallel_planning_py.GetPathEle(1)
  path_theta_vec = parallel_planning_py.GetPathEle(2)

  data_path.data.update({
    'x_vec': path_x_vec,
    'y_vec': path_y_vec,
    'theta_vec': path_theta_vec,
  })

  # obstacles
  data_obs_pt.data.update({
    'x_vec': parallel_planning_py.GetObsX(),
    'y_vec': parallel_planning_py.GetObsY(),
  })
  data_virtual_obs_pt.data.update({
    'x_vec': parallel_planning_py.GetVirtualObsX(),
    'y_vec': parallel_planning_py.GetVirtualObsY(),
  })

  # path ego car
  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(path_x_vec)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], path_x_vec[k], path_y_vec[k], path_theta_vec[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_car_box.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  res = parallel_planning_py.GetInverseArcVec()
  print("arc info: cx, cy, radius, pBx, pBy")
  print("arc info len: ", len(res))
  cx_vec = []
  cy_vec = []
  radius_vec = []
  pBx_vec = []
  pBy_vec = []

  pCx_vec = []
  pCy_vec = []

  info_len = 7
  arc_size = int(len(res) / info_len)
  for i in range(arc_size):
    print(i, ": [", res[info_len * i], res[info_len * i + 1], res[info_len * i + 2], res[info_len * i + 3], res[info_len * i + 4], "]")
    cx_vec.append(res[info_len * i])
    cy_vec.append(res[info_len * i + 1])
    radius_vec.append(res[info_len * i + 2])

    pBx_vec.append(res[info_len * i + 3])
    pBy_vec.append(res[info_len * i + 4])

    pCx_vec.append(res[info_len * i + 5])
    pCy_vec.append(res[info_len * i + 6])

  data_debug_arc.data.update({
    'cx_vec':cx_vec,
    'cy_vec':cy_vec,
    'radius_vec':radius_vec,
    'pBx_vec':pBx_vec,
    'pBy_vec':pBy_vec,
    'pCx_vec':pCx_vec,
    'pCy_vec':pCy_vec
  })

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)

# os.environ['WEB_BROWSER'] = 'chrome'
# fig1.output_backend = "svg"
# export_svgs(fig1, filename="plot.svg")

