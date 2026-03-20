import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import planning_plan_pb2
from jupyter_pybind import apa_simulation_py

# bag path and frame dt
bag_path = '/home/xlwang71/Downloads/simulation/test_2.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

# try before sliders
apa_simulation_py.Init()

data_planning_tune = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

fig1.line('plan_path_y', 'plan_path_x', source = data_planning_tune, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'tuned plan')

### sliders callback
bag_time = 0.0
update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)
index_map = bag_loader.get_msg_index(bag_time)

fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
loc_msg = bag_loader.loc_msg['data'][index_map['loc_msg_idx']]

res = apa_simulation_py.InterfaceUpdate(soc_state_msg.SerializeToString(),
                                  fus_parking_msg.SerializeToString(),
                                  loc_msg.SerializeToString(),
                                  vs_msg.SerializeToString(),
                                  wave_msg.SerializeToString())

if res == True:
  tuned_planning_output = planning_plan_pb2.PlanningOutput()
  tuned_planning_output.ParseFromString(apa_simulation_py.GetPlanningOutput())

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []
  for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
    plan_path_x.append(tuned_planning_output.trajectory.trajectory_points[i].x)
    plan_path_y.append(tuned_planning_output.trajectory.trajectory_points[i].y)
    plan_path_heading.append(tuned_planning_output.trajectory.trajectory_points[i].heading_yaw)

  data_planning_tune.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })


