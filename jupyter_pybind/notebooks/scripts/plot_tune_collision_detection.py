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
sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import collision_detection_py

from bokeh.models import Arrow, NormalHead

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

car_xb, car_yb = load_car_params_patch()

data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_start = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_end_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_end = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_turn_circle = ColumnDataSource(data = {'x':[], 'y':[], 'r':[]})

data_car_collision_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_car_collision = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})

data_obstacle_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_obstacle_turn_circle = ColumnDataSource(data = {'x':[], 'y':[], 'r':[]})

data_collision_point_pos = ColumnDataSource(data = {'x':[], 'y':[]})


fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=960, height=640, match_aspect = True, aspect_scale=1)

fig1.circle('x','y', source = data_car_turn_circle, size=8, color='grey', legend_label = 'car_turn_circle', visible = True)
fig1.circle(x ='x', y ='y', radius = 'r', source=data_car_turn_circle, line_alpha = 0.5, line_width = 1, line_color = "grey", fill_alpha=0, legend_label = 'car_turn_circle', visible = True)
fig1.patch('car_xn', 'car_yn', source = data_car_start, fill_color = "red", line_color = "red", line_width = 1, legend_label = 'car_start', fill_alpha=0.5)
fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start')
fig1.patch('car_xn', 'car_yn', source = data_car_end, fill_color = "blue", line_color = "blue", line_width = 1, legend_label = 'car_end', fill_alpha=0.5)
fig1.circle('x','y', source = data_car_end_pos, size=8, color='blue', legend_label = 'car_end')

fig1.patch('car_xn', 'car_yn', source = data_car_collision, fill_color = "grey", line_color = "grey", line_width = 1, legend_label = 'car_collision', fill_alpha=0.5)
fig1.circle('x','y', source = data_car_collision_pos, size=8, color='grey', legend_label = 'car_collision')

fig1.circle('x','y', source = data_obstacle_pos, size=8, color='red', legend_label = 'obstacle_pos')
fig1.circle('x','y', source = data_obstacle_turn_circle, size=8, color='yellow', legend_label = 'obstacle_turn_circle', visible = False)
fig1.circle(x ='x', y ='y', radius = 'r', source=data_obstacle_turn_circle, line_alpha = 0.5, line_width = 1, line_color = "black", fill_alpha=0, legend_label = 'obstacle_turn_circle', visible = False)

fig1.circle('x','y', source = data_collision_point_pos, size=8, color='orange', legend_label = 'collision_point_pos')

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

columns = [
  TableColumn(field="names", title="names",),
  TableColumn(field="datas", title="datas"),
]
debug_data = ColumnDataSource(data = {'names':[], 'datas':[]})
debug_table = DataTable(source=debug_data, columns=columns, width=600, height=500)

collision_detection_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):

    # obstacle info
    self.obstacle_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obstacle_x",min=-15, max=15, value=-0.7, step=0.01)
    self.obstacle_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obstacle_y",min=-15, max=15, value=4.4, step=0.01)

    # cal car motion traj
    self.ego_x_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x_start",min=-10, max=10, value=0.0, step=0.01)
    self.ego_y_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y_start",min=-10, max=10, value=0.0, step=0.01)
    self.ego_heading_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading_start",min=-180, max=180, value=90.0, step=1)
    self.turn_radius_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "turn_radius",min=0.0, max=10, value=5.5, step=0.01)
    self.traj_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "traj_length",min=0, max=10, value=9.0, step=0.05)
    self.straight_left_right_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "straight_left_right",min=0, max=2, value=0, step=1)
    self.forward_back_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "forward_back",min=0, max=1, value=0, step=1)

    # car inflation
    self.lat_inflation_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lat_inflation",min=0.0, max=0.3, value=0.05, step=0.01)

    # use directly car traj
    self.start_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "start_x",min=-10, max=10, value=6.99931, step=0.01)
    self.start_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "start_y",min=-10, max=10, value=-3.50461, step=0.01)
    self.start_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "start_heading",min=-4.1, max=4.1, value=-1.5563, step=0.01)
    self.end_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "end_x",min=-10, max=10, value=4.60131, step=0.01)
    self.end_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "end_y",min=-10, max=10, value=0.95784, step=0.01)
    self.end_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "end_heading",min=-4.1, max=4.1, value=-0.599093, step=0.01)
    self.turn_center_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "turn_center_x",min=-10.0, max=10.0, value=1.49989, step=0.01)
    self.turn_center_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "turn_center_y",min=-10.0, max=10.0, value=-3.58432, step=0.01)



    ipywidgets.interact(slider_callback, obstacle_x = self.obstacle_x_slider,
                                         obstacle_y = self.obstacle_y_slider,

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
def slider_callback(obstacle_x, obstacle_y,
                    ego_x_start, ego_y_start, ego_heading_start, turn_radius, traj_length, straight_left_right, forward_back,
                    lat_inflation,
                    start_x, start_y, start_heading, end_x, end_y,  end_heading, turn_center_x, turn_center_y):

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

  # set obstacle start pos
  collision_detection_py.SetObstacle(obstacle_x, obstacle_y)

  # set param
  collision_detection_py.SetParam(lat_inflation)

  # collision detect
  if straight_left_right == 0:
    collision_detection_py.UpdateRefTrajLine(ego_pos_start, ego_pos_end)
  else:
    collision_detection_py.UpdateRefTrajArc(ego_pos_start, ego_pos_end, ego_turn_circle)

  # get collision detect result
  collision_flag = collision_detection_py.GetCollisionFlag()
  remain_dist = collision_detection_py.GetRemainDist()
  car_remain_dist = collision_detection_py.GetRemainCarDist()
  obstacle_remain_dist = collision_detection_py.GetRemainObstacleDist()

  names = []
  datas = []
  names.append("collision_flag")
  datas.append(collision_flag)
  names.append("car_remain_dist")
  datas.append(round(car_remain_dist, 3))

  print("collision_flag = ", collision_flag)

  ego_pos_collision = [0.0, 0.0, 0.0]
  if collision_flag == True:
    collision_point = collision_detection_py.GetCollisionPoint()
    data_collision_point_pos.data.update({
      'x': [collision_point[0]],
      'y': [collision_point[1]],
    })
    names.append("obstacle_remain_dist")
    datas.append(round(obstacle_remain_dist, 3))
    names.append("remain_dist")
    datas.append(round(remain_dist, 3))
    names.append("collision_point")
    round_collision_point = round_list(collision_point, 3)
    datas.append(round_collision_point)
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

      names.append("ego_rotate_angle")
      datas.append(round(ego_rot_angle*57.2958,3))
      names.append("ego_rot_angle_collision")
      datas.append(round(ego_rot_angle_collision*57.2958,3))

    names.append("ego_pos_start")
    round_ego_pos_start = round_list(ego_pos_start, 3)
    datas.append(round_ego_pos_start)

    names.append("ego_pos_end")
    round_ego_pos_end = round_list(ego_pos_end, 3)
    datas.append(round_ego_pos_end)

    names.append("ego_pos_collision")
    round_ego_pos_collision = round_list(ego_pos_collision, 3)
    datas.append(round_ego_pos_collision)

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

  else:
    data_collision_point_pos.data.update({
      'x': [],
      'y': [],
    })
    data_car_collision_pos.data.update({
      'x': [],
      'y': [],
    })
    data_car_collision.data.update({
      'car_xn': [],
      'car_yn': [],
    })

  debug_data.data.update({
    'names': names,
    'datas': datas,
  })

  #   data_collision_point_pos.data.update({
  #     'x': [collision_point[0]],
  #     'y': [collision_point[1]],
  #   })
  #   if straight_left_right != 0:
  #     obstacle_turn_x = ego_turn_center_x
  #     obstacle_turn_y = ego_turn_center_y
  #     obstacle_turn_radius = math.sqrt((obstacle_x - obstacle_turn_x)*(obstacle_x - obstacle_turn_x)+(obstacle_y - obstacle_turn_y)*(obstacle_y - obstacle_turn_y))
  #     data_obstacle_turn_circle.data.update({
  #       'x': [ego_turn_center_x],
  #       'y': [ego_turn_center_y],
  #       'r': [obstacle_turn_radius],
  #     })
  #     # 畫出剛好碰撞時的車輛  根據剩餘距離畫出車輛
  #     ego_rot_angle_collision = obstacle_remain_dist / turn_radius
  #     ego_heading_collision = 0.0
  #     if is_anticlockwise_rotation == True:
  #       ego_heading_collision = ego_heading_start + ego_rot_angle_collision
  #     else:
  #       ego_heading_collision = ego_heading_start - ego_rot_angle_collision

  #     ego_pos_collision = collision_detection_py.GetEgoPosCoord(ego_pos_start, ego_turn_center_pos, ego_rot_angle_collision, is_anticlockwise_rotation)

  #     car_xn = []
  #     car_yn = []
  #     for i in range(len(car_xb)):
  #         tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_pos_collision[0], ego_pos_collision[1], ego_heading_collision)
  #         car_xn.append(tmp_x)
  #         car_yn.append(tmp_y)
  #     data_car_collision.data.update({
  #       'car_xn': car_xn,
  #       'car_yn': car_yn,
  #     })

  #     data_car_collision_pos.data.update({
  #       'x': [ego_pos_collision[0]],
  #       'y': [ego_pos_collision[1]],
  #     })








  #   else:
  #     data_obstacle_turn_circle.data.update({
  #       'x': [],
  #       'y': [],
  #       'r': [],
  #     })

  # else:
  #   data_collision_point_pos.data.update({
  #     'x': [],
  #     'y': [],
  #   })

  #   data_obstacle_turn_circle.data.update({
  #       'x': [],
  #       'y': [],
  #       'r': [],
  #   })




  # # get and update obstacle end pos
  # obstacles_end = collision_detection_py.GetObstaclesEnd()
  # obstacles_end_x = []
  # obstacles_end_y = []
  # for i in range(len(obstacles_end)):
  #   obstacles_end_x.append(obstacles_end[i][0])
  #   obstacles_end_y.append(obstacles_end[i][1])

  # data_obstacle_end_pos.data.update({
  #   'x': obstacles_end_x,
  #   'y': obstacles_end_y,
  # })

  # # get and update obstacles center pos
  # obstacles_turn_center = collision_detection_py.GetObstaclesCenter()
  # obstacles_turn_center_x = []
  # obstacles_turn_center_y = []
  # obstacles_turn_radius = []
  # for i in range(len(obstacles_turn_center)):
  #   obstacles_turn_center_x.append(obstacles_turn_center[i][0])
  #   obstacles_turn_center_y.append(obstacles_turn_center[i][1])
  #   obstacles_turn_radius.append(turn_radius)

  # data_obstacle_turn_circle.data.update({
  #   'x': obstacles_turn_center_x,
  #   'y': obstacles_turn_center_y,
  #   'r': obstacles_turn_radius,
  # })



  # x_start = ego_x_start
  # y_start = ego_y_start

  # heading_start = 0.0
  # if forward_back == 0:
  #   # forward
  #   heading_start = ego_heading_start / 57.2958
  # elif forward_back == 1:
  #   # back
  #   vel_heading = ego_heading_start + 180
  #   if vel_heading > 360:
  #     vel_heading = vel_heading - 360
  #   heading_start = (vel_heading) / 57.2958

  # x_end = 0.0
  # y_end = 0.0
  # center_x = 0.0
  # center_y = 0.0
  # if straight_left_right == 0:
  #   # go straight
  #   x_end = x_start + traj_length * math.cos(heading_start)
  #   y_end = y_start + traj_length * math.sin(heading_start)
  #   turn_radius = 0.0
  # else:
  #   point_start = [x_start, y_start]
  #   point_center = collision_detection_py.GetTrunCenterCoord(point_start, heading_start, turn_radius, straight_left_right, forward_back)
  #   center_x = point_center[0]
  #   center_y = point_center[1]
  #   arc_AB_theta = traj_length / turn_radius
  #   point_end = collision_detection_py.GetTrunPointEndCoord(point_start, point_center, arc_AB_theta, straight_left_right, forward_back)
  #   x_end = point_end[0]
  #   y_end = point_end[1]

  # obstacles_x = 5.4
  # obstacles_y = 1.22511
  # # update obstacle pos
  # obstacles_x = [obstacle_x]
  # obstacles_y = [obstacle_y]
  # data_obstacle_pos.data.update({
  #   'x': [obstacle_x],
  #   'y': [obstacle_y],
  # })

  # # set obstacle start pos
  # collision_detection_py.SetObstacles(obstacles_x, obstacles_y)


  # x_start = 5.53048
  # x_end = 6.71119
  # y_start = 0.16552
  # y_end = -0.797519
  # center_x = 9.56367
  # center_y = 3.90496
  # # collision detection
  # car_start = [x_start, y_start]
  # car_end = [x_end, y_end]
  # car_turn_center = [center_x, center_y]

  # # if straight_left_right == 0:
  # #   collision_detection_py.UpdateRefTrajLine(car_start, car_end, heading_start)
  # # else:
  # #   collision_detection_py.UpdateRefTrajArc(car_start, car_end, heading_start, car_turn_center, turn_radius)
  # car_start = [5.53048, 0.16552]
  # car_end = [6.71119, -0.797519]
  # car_turn_center = [9.56367, 3.90496]
  # heading_start = -0.823174
  # turn_radius = 5.5
  # collision_detection_py.UpdateRefTrajArc(car_start, car_end, heading_start, car_turn_center, turn_radius)

  # collision_flag = collision_detection_py.GetCollisionFlag()
  # remain_dist = collision_detection_py.GetRamainDist()

  #   # update start point pos
  # data_car_start_pos.data.update({
  #   'x': [x_start],
  #   'y': [y_start],
  # })

  # # update center point pos
  # data_car_turn_circle.data.update({
  #   'x': [center_x],
  #   'y': [center_y],
  #   'r': [turn_radius],
  # })

  # # update end point pos
  # data_car_end_pos.data.update({
  #   'x': [x_end],
  #   'y': [y_end],
  # })

  # # update ego car pos
  # car_xn = []
  # car_yn = []
  # for i in range(len(car_xb)):
  #     tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], x_start, y_start, heading_start)
  #     car_xn.append(tmp_x)
  #     car_yn.append(tmp_y)
  # data_car_start.data.update({
  #   'car_xn': car_xn,
  #   'car_yn': car_yn,
  # })

  # names = []
  # datas = []
  # names.append("collision_flag")
  # datas.append(collision_flag)
  # names.append("remain_dist")
  # datas.append(remain_dist)
  # debug_data.data.update({
  #   'names': names,
  #   'datas': datas,
  # })

  # # get obstacle end pos
  # obstacles_end = collision_detection_py.GetObstaclesEnd()
  # obstacles_end_x = []
  # obstacles_end_y = []
  # for i in range(len(obstacles_end)):
  #   obstacles_end_x.append(obstacles_end[i][0])
  #   obstacles_end_y.append(obstacles_end[i][1])

  # data_obstacle_end_pos.data.update({
  #   'x': obstacles_end_x,
  #   'y': obstacles_end_y,
  # })

  # # get obstacles center pos
  # obstacles_turn_center = collision_detection_py.GetObstaclesCenter()
  # obstacles_turn_center_x = []
  # obstacles_turn_center_y = []
  # obstacles_turn_radius = []
  # for i in range(len(obstacles_turn_center)):
  #   obstacles_turn_center_x.append(obstacles_turn_center[i][0])
  #   obstacles_turn_center_y.append(obstacles_turn_center[i][1])
  #   obstacles_turn_radius.append(turn_radius)

  # data_obstacle_turn_circle.data.update({
  #   'x': obstacles_turn_center_x,
  #   'y': obstacles_turn_center_y,
  #   'r': obstacles_turn_radius,
  # })


  push_notebook()

bkp.show(row(fig1, debug_table), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
