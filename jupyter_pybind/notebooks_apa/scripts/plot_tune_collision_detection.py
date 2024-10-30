import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
# from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS
# from bokeh.models import DataTable, DateFormatter, TableColumn
# from bokeh.models import TextInput
from jupyter_pybind import collision_detection_py

from bokeh.models import Arrow, NormalHead
from bokeh.events import Tap

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_start = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_end_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_end = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_turn_circle = ColumnDataSource(data = {'x':[], 'y':[], 'r':[]})

data_car_collision_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_car_collision = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})

data_obstacle_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_obstacle_line = ColumnDataSource(data = {'x':[], 'y':[]})

data_obstacle_turn_circle = ColumnDataSource(data = {'x':[], 'y':[], 'r':[]})

data_col_pt_ego = ColumnDataSource(data = {'x':[], 'y':[]})

data_remain_dist_line = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_collision_line = ColumnDataSource(data = {'x':[], 'y':[]})

data_traj_bound = ColumnDataSource(data = {'x':[], 'y':[]})

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=960, height=640, match_aspect = True, aspect_scale=1)

fig1.circle('x','y', source = data_car_turn_circle, size=8, color='grey', legend_label = 'car_turn_circle', visible = True)
fig1.circle(x ='x', y ='y', radius = 'r', source=data_car_turn_circle, line_alpha = 0.5, line_width = 1, line_color = "grey", fill_alpha=0, legend_label = 'car_turn_circle', visible = True)
fig1.patch('car_xn', 'car_yn', source = data_car_start, fill_color = "red", line_color = "red", line_width = 1, legend_label = 'car_start', fill_alpha=0.5)
fig1.line('x', 'y', source = data_car_collision_line, line_width = 5.0, line_color = 'black', line_dash = 'solid', legend_label = 'car_collision_line', visible = True)
fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start')
fig1.patch('car_xn', 'car_yn', source = data_car_end, fill_color = "blue", line_color = "blue", line_width = 1, legend_label = 'car_end', fill_alpha=0.5)
fig1.circle('x','y', source = data_car_end_pos, size=8, color='blue', legend_label = 'car_end')

fig1.patch('car_xn', 'car_yn', source = data_car_collision, fill_color = "grey", line_color = "grey", line_width = 1, legend_label = 'car_collision', fill_alpha=0.5)
fig1.circle('x','y', source = data_car_collision_pos, size=8, color='grey', legend_label = 'car_collision')

fig1.circle('x','y', source = data_obstacle_pos, size=8, color='red', legend_label = 'obstacle_pos')
fig1.line('x', 'y', source = data_obstacle_line, line_width = 1.5, line_color = 'orange', line_dash = 'solid', legend_label = 'obstacle_line', visible = False)
fig1.circle('x','y', source = data_obstacle_turn_circle, size=8, color='yellow', legend_label = 'obstacle_turn_circle', visible = False)
fig1.circle(x ='x', y ='y', radius = 'r', source=data_obstacle_turn_circle, line_alpha = 0.5, line_width = 1, line_color = "black", fill_alpha=0, legend_label = 'obstacle_turn_circle', visible = False)

fig1.circle('x','y', source = data_col_pt_ego, size=8, color='orange', legend_label = 'col_pt_ego')

fig1.patch('x', 'y', source = data_traj_bound, fill_color = "red", line_color = "black", line_width = 1, legend_label = 'traj_bound', fill_alpha=0.0)

# fig1.line('x', 'y', source = data_remain_dist_line, line_width = 5.0, line_color = 'green', line_dash = 'solid', legend_label = 'remain_dist', visible = False)

data_simu_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.patches('x_vec', 'y_vec', source = data_simu_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sim_sampled_carbox', visible = False)

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

columns = [
  TableColumn(field="names", title="names",),
  TableColumn(field="datas", title="datas"),
]
debug_data = ColumnDataSource(data = {'names':[], 'datas':[]})
debug_table = DataTable(source=debug_data, columns=columns, width=600, height=500)

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

collision_detection_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.vehicle_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=2, value=1, step=1)
    self.traj_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "traj_bound",min=0.2, max=2, value=0.5, step=0.1)
    # obstacle info
    self.obstacle_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obstacle_x",min=-15, max=15, value=-0.7, step=0.01)
    self.obstacle_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obstacle_y",min=-15, max=15, value=4.4, step=0.01)
    self.obstacle_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obstacle_heading",min=-180, max=180, value=0.0, step=1)
    self.obstacle_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obstacle_length",min=1, max=20, value=3.0, step=1)
    self.is_line_obs_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_line_obs",min=0, max=1, value=0, step=1)

    # cal car motion traj
    self.ego_x_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x_start",min=-10, max=10, value=0.0, step=0.01)
    self.ego_y_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y_start",min=-10, max=10, value=0.0, step=0.01)
    self.ego_heading_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading_start",min=-180, max=180, value=90.0, step=1)
    self.turn_radius_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "turn_radius",min=0.0, max=10, value=5.5, step=0.01)
    self.traj_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "traj_length",min=0, max=10, value=9.0, step=0.05)
    self.straight_left_right_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "straight_left_right",min=0, max=2, value=2, step=1)
    self.forward_back_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "forward_back",min=0, max=1, value=0, step=1)

    # car inflation
    self.lat_inflation_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lat_inflation",min=0.0, max=0.3, value=0.07, step=0.01)

    # use directly car traj
    self.start_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "start_x",min=-10, max=10, value=6.99931, step=0.01)
    self.start_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "start_y",min=-10, max=10, value=-3.50461, step=0.01)
    self.start_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "start_heading",min=-4.1, max=4.1, value=-1.5563, step=0.01)
    self.end_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "end_x",min=-10, max=10, value=4.60131, step=0.01)
    self.end_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "end_y",min=-10, max=10, value=0.95784, step=0.01)
    self.end_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "end_heading",min=-4.1, max=4.1, value=-0.599093, step=0.01)
    self.turn_center_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "turn_center_x",min=-10.0, max=10.0, value=1.49989, step=0.01)
    self.turn_center_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "turn_center_y",min=-10.0, max=10.0, value=-3.58432, step=0.01)



    ipywidgets.interact(slider_callback, vehicle_type = self.vehicle_type_slider,
                                         traj_bound = self.traj_bound_slider,
                                         obstacle_x = self.obstacle_x_slider,
                                         obstacle_y = self.obstacle_y_slider,
                                         obstacle_heading = self.obstacle_heading_slider,
                                         obstacle_length = self.obstacle_length_slider,
                                         is_line_obs = self.is_line_obs_slider,

                                         ego_x_start = self.ego_x_start_slider,
                                         ego_y_start = self.ego_y_start_slider,
                                         ego_heading_start = self.ego_heading_start_slider,
                                         turn_radius = self.turn_radius_slider,
                                         traj_length = self.traj_length_slider,
                                         straight_left_right = self.straight_left_right_slider,
                                         forward_back = self.forward_back_slider,

                                         lat_inflation = self.lat_inflation_slider,

                                         start_x = self.start_x_slider,
                                         start_y = self.start_y_slider,
                                         start_heading = self.start_heading_slider,
                                         end_x = self.end_x_slider,
                                         end_y = self.end_y_slider,
                                         end_heading = self.end_heading_slider,
                                         turn_center_x = self.turn_center_x_slider,
                                         turn_center_y = self.turn_center_y_slider)

def normalize_angle(angle):
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle

def round_list(lst, decimals):
    rounded_list = []
    for num in lst:
        rounded_num = round(num, decimals)
        rounded_list.append(rounded_num)
    return rounded_list

### sliders callback
def slider_callback(vehicle_type, traj_bound, obstacle_x, obstacle_y, obstacle_heading, obstacle_length, is_line_obs,
                    ego_x_start, ego_y_start, ego_heading_start, turn_radius, traj_length, straight_left_right, forward_back,
                    lat_inflation,
                    start_x, start_y, start_heading, end_x, end_y,  end_heading, turn_center_x, turn_center_y):

  if vehicle_type == 0:
    vehicle_type = 'JAC_S811'
  elif vehicle_type == 1:
    vehicle_type = 'CHERY_T26'
  elif vehicle_type == 2:
    vehicle_type = 'CHERY_E0X'

  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type)

  kwargs = locals()

  # x, y, heading
  ego_pos_start = [ego_x_start, ego_y_start, normalize_angle(ego_heading_start / 57.2958)]

  ego_pos_end = [0.0, 0.0, 0.0]

  # x, y, radius, rotation angle, rotation direction(True notes anticlockwise, False notes clockwise)
  ego_turn_circle = [0.0, 0.0, 0.0, 0.0, True]

  obstacle_turn_circle = [0.0, 0.0, 0.0, 0.0, True]

  # cal ego end pos and heading
  if straight_left_right == 0:
    print("go straight")
    # go straight
    if forward_back == 0:
      # car gear is drive
      ego_pos_end[0] = ego_pos_start[0] + traj_length * math.cos(ego_pos_start[2])
      ego_pos_end[1] = ego_pos_start[1] + traj_length * math.sin(ego_pos_start[2])
    else:
      # car gear is reverse
      ego_pos_end[0] = ego_pos_start[0] - traj_length * math.cos(ego_pos_start[2])
      ego_pos_end[1] = ego_pos_start[1] - traj_length * math.sin(ego_pos_start[2])
    ego_pos_end[2] = ego_pos_start[2]
  else:
    # turn
    ego_turn_circle[2] = turn_radius
    # determine whether the car rotates anticlockwise or clockwise
    if (forward_back == 0 and straight_left_right == 1) or (forward_back == 1 and straight_left_right == 2):
      # forward left or back right notes that car rotates anticlockwise
      ego_turn_circle[4] = True
    elif (forward_back == 0 and straight_left_right == 2) or (forward_back == 1 and straight_left_right == 1):
      # forward right or back left notes that car rotates clockwise
      ego_turn_circle[4] = False

    # sure tangent_unit_vector rotates correctly
    ego_start_heading = ego_pos_start[2]
    if (forward_back == 1):
       ego_pos_start[2] = normalize_angle(ego_start_heading + math.pi)

    ego_turn_center_coord = collision_detection_py.GetTrunCenterCoord(ego_pos_start, ego_turn_circle)

    ego_pos_start[2] = ego_start_heading
    ego_turn_circle[0] = ego_turn_center_coord[0]
    ego_turn_circle[1] = ego_turn_center_coord[1]

    ego_turn_circle[3] = traj_length / turn_radius

    ego_pos_coord_end = collision_detection_py.GetEgoPosCoord(ego_pos_start, ego_turn_circle)
    ego_pos_end[0] = ego_pos_coord_end[0]
    ego_pos_end[1] = ego_pos_coord_end[1]

    if ego_turn_circle[4] == True:
      ego_pos_end[2] = ego_pos_start[2] + ego_turn_circle[3]
    else:
      ego_pos_end[2] = ego_pos_start[2] - ego_turn_circle[3]

    obstacle_turn_circle[0] = ego_turn_circle[0]
    obstacle_turn_circle[1] = ego_turn_circle[1]

    obstacle_turn_circle[2] = math.sqrt((obstacle_x - obstacle_turn_circle[0])**2 + (obstacle_y - obstacle_turn_circle[1])**2)
    obstacle_turn_circle[4] = not ego_turn_circle[4]

  # update ego start pos
  data_car_start_pos.data.update({
    'x': [ego_pos_start[0]],
    'y': [ego_pos_start[1]],
  })

  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_pos_start[0], ego_pos_start[1], ego_pos_start[2])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  data_car_start.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # update ego end pos
  data_car_end_pos.data.update({
    'x': [ego_pos_end[0]],
    'y': [ego_pos_end[1]],
  })

  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_pos_end[0], ego_pos_end[1], ego_pos_end[2])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  data_car_end.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # update obstacle pos
  data_obstacle_pos.data.update({
    'x': [obstacle_x],
    'y': [obstacle_y],
  })

  # update ego turn center pos and circle
  # update obstacle turn center pos and circle
  if straight_left_right == 0:
    data_car_turn_circle.data.update({
      'x': [],
      'y': [],
      'r': [],
    })
    data_obstacle_turn_circle.data.update({
      'x': [],
      'y': [],
      'r': [],
    })
  else:
    data_car_turn_circle.data.update({
      'x': [ego_turn_circle[0]],
      'y': [ego_turn_circle[1]],
      'r': [ego_turn_circle[2]],
    })
    data_obstacle_turn_circle.data.update({
      'x': [obstacle_turn_circle[0]],
      'y': [obstacle_turn_circle[1]],
      'r': [obstacle_turn_circle[2]],
    })

  obs_pos_start = [obstacle_x, obstacle_y, normalize_angle(obstacle_heading / 57.2958)]
  obs_pos_end = [0.0, 0.0, 0.0]
  obs_pos_end[0] = obs_pos_start[0] + obstacle_length * math.cos(obs_pos_start[2])
  obs_pos_end[1] = obs_pos_start[1] + obstacle_length * math.sin(obs_pos_start[2])

  data_obstacle_line.data.update({
    'x': [obs_pos_start[0], obs_pos_end[0]],
    'y': [obs_pos_start[1], obs_pos_end[1]],
  })

  collision_detection_py.SetObstacleLine(obs_pos_start[0], obs_pos_start[1], obs_pos_end[0], obs_pos_end[1])

  # set obstacle start pos
  collision_detection_py.SetObstacle(obstacle_x, obstacle_y)

  # set param
  collision_detection_py.SetParam(lat_inflation, traj_bound)

  # collision detect
  if straight_left_right == 0:
    collision_detection_py.UpdateRefTrajLine(ego_pos_start, ego_pos_end, is_line_obs)
  else:
    collision_detection_py.UpdateRefTrajArc(ego_pos_start, ego_pos_end, ego_turn_circle, ego_turn_circle[4], is_line_obs)

  # get collision detect result
  collision_flag = collision_detection_py.GetCollisionFlag()
  remain_dist = collision_detection_py.GetRemainDist()
  car_remain_dist = collision_detection_py.GetRemainCarDist()
  obstacle_remain_dist = collision_detection_py.GetRemainObstacleDist()
  car_line_order = collision_detection_py.GetCarLineOrder()
  pt_vec = collision_detection_py.GetSamplePt()
  col_pt_ego = collision_detection_py.GetCollisionPointEgoGlobal()
  is_obs_in_car = collision_detection_py.GetObsIsInCar()
  traj_bound_vec = collision_detection_py.GetTrajBound()
  is_line_intersect_arc = collision_detection_py.GetLineIntersectArc()

  if car_remain_dist < obstacle_remain_dist:
    collision_flag = False
    car_line_order = -1
  else:
    collision_flag = True

  # update car sample car_box
  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(pt_vec)):
    plan_path_x.append(pt_vec[i][0])
    plan_path_y.append(pt_vec[i][1])
    plan_path_heading.append(pt_vec[i][2])

  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(plan_path_x)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], plan_path_x[k], plan_path_y[k], plan_path_heading[k])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_simu_car_box.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  traj_x, traj_y = [], []

  for i in range(len(traj_bound_vec)):
    traj_x.append(traj_bound_vec[i][0])
    traj_y.append(traj_bound_vec[i][1])

  data_traj_bound.data.update({
    'x': traj_x,
    'y': traj_y,
  })

  names = []
  datas = []
  names.append("ego_pos_start")
  round_ego_pos_start = round_list([ego_pos_start[0], ego_pos_start[1], ego_pos_start[2] * 57.3], 3)
  datas.append(round_ego_pos_start)
  names.append("ego_pos_end")
  round_ego_pos_end = round_list([ego_pos_end[0], ego_pos_end[1], ego_pos_end[2] * 57.3], 3)
  datas.append(round_ego_pos_end)
  # update collision_flag
  names.append("collision_flag")
  datas.append(collision_flag)

  names.append("is_obs_in_car")
  datas.append(is_obs_in_car)

  names.append("is_line_intersect_arc")
  datas.append(is_line_intersect_arc)

  # update car_remain_dist
  names.append("car_remain_dist")
  datas.append(round(car_remain_dist, 3))

  names.append("obstacle_remain_dist")
  datas.append(round(obstacle_remain_dist, 3))

  names.append("remain_dist")
  datas.append(round(remain_dist, 3))

  # update car line
  names.append("car_line_order")
  datas.append(car_line_order)
  data_car_collision_line.data.update({
    'x': [],
    'y': [],
  })
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
    tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_pos_start[0], ego_pos_start[1], ego_pos_start[2])
    car_xn.append(tmp_x)
    car_yn.append(tmp_y)
  if car_line_order != -1:
    if car_line_order < len(car_xb) - 1:
      data_car_collision_line.data.update({
        'x': [car_xn[car_line_order], car_xn[car_line_order + 1]],
        'y': [car_yn[car_line_order], car_yn[car_line_order + 1]],
      })
    else:
      data_car_collision_line.data.update({
        'x': [car_xn[car_line_order], car_xn[0]],
        'y': [car_yn[car_line_order], car_yn[0]],
      })

  # update car col ego point
  names.append("col_pt_ego")
  datas.append(round_list(col_pt_ego, 3))
  data_col_pt_ego.data.update({
    'x': [],
    'y': [],
  })
  if car_line_order != -1:
    data_col_pt_ego.data.update({
      'x': [col_pt_ego[0]],
      'y': [col_pt_ego[1]],
    })

  # update car col pos
  ego_pos_collision = [0.0, 0.0, 0.0]
  if collision_flag:
    if straight_left_right == 0:
      # go straight
      if forward_back == 0:
        # car gear is drive
        ego_pos_collision[0] = ego_pos_start[0] + obstacle_remain_dist * math.cos(ego_pos_start[2])
        ego_pos_collision[1] = ego_pos_start[1] + obstacle_remain_dist * math.sin(ego_pos_start[2])
      else:
        # car gear is reverse
        ego_pos_collision[0] = ego_pos_start[0] - obstacle_remain_dist * math.cos(ego_pos_start[2])
        ego_pos_collision[1] = ego_pos_start[1] - obstacle_remain_dist * math.sin(ego_pos_start[2])
      ego_pos_collision[2] = ego_pos_start[2]
    else:
      ego_rot_angle_collision = obstacle_remain_dist / turn_radius
      ego_rot_angle = ego_turn_circle[3]
      ego_turn_circle[3] = ego_rot_angle_collision
      ego_pos_coord_collision = collision_detection_py.GetEgoPosCoord(ego_pos_start, ego_turn_circle)
      ego_turn_circle[3] = ego_rot_angle

      ego_pos_collision[0] = ego_pos_coord_collision[0]
      ego_pos_collision[1] = ego_pos_coord_collision[1]
      if ego_turn_circle[4] == True:
        ego_pos_collision[2] = ego_pos_start[2] + ego_rot_angle_collision
      else:
        ego_pos_collision[2] = ego_pos_start[2] - ego_rot_angle_collision
  names.append("ego_pos_collision")
  round_collision_point = round_list(ego_pos_collision, 3)
  datas.append(round_collision_point)

  data_car_collision.data.update({
    'car_xn': [],
    'car_yn': [],
  })

  data_car_collision_pos.data.update({
    'x': [],
    'y': [],
  })

  if collision_flag:
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_pos_collision[0], ego_pos_collision[1], ego_pos_collision[2])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)

    data_car_collision.data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })

    data_car_collision_pos.data.update({
      'x': [ego_pos_collision[0]],
      'y': [ego_pos_collision[1]],
    })

  debug_data.data.update({
    'names': names,
    'datas': datas,
  })

  push_notebook()

bkp.show(row(fig1, debug_table), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
