import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import diag_slot_planning_py

# bag path and frame dt
bag_path = '/home/xlwang71/Downloads/APA/1012/test_2.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

origin_selected_id = bag_loader.fus_parking_msg['data'][int(len(bag_loader.fus_parking_msg['t'])/2)].select_slot_id
diag_slot_planning_py.Init()

data_planning_tune = ColumnDataSource(data = {'plan_traj_x':[],
                                              'plan_traj_y':[],})
data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

# find bag time when planning at real test
# plan_msg_idx = 0
# if bag_loader.plan_msg['enable'] == True:
#   while plan_msg_idx < (len(bag_loader.plan_msg['t'])-1):
#     trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
#     print(trajectory)
#     if len(trajectory.trajectory_points) > 1:
#       break
#     plan_msg_idx = plan_msg_idx + 1
# bag_time0 = plan_msg_idx * 0.1
bag_time0 = 0.0
fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning_tune, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'tuned plan')

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=bag_time0, step=frame_dt)
    self.selected_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='25%'), description= "selected_id",min=0, max=100, value=origin_selected_id, step=1)
    self.force_planning_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "force_planning",min=0, max=1, value=0, step=1)
    self.turn_on_plan_stm_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "turn_on_plan_stm",min=0, max=1, value=0, step=1)
    self.plan_stm_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "plan_stm",min=0, max=4, value=0, step=1)
    self.is_complete_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "is_complete",min=0, max=1, value=0, step=1)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                        selected_id = self.selected_id_slider,
                                        force_planning = self.force_planning_slider,
                                        turn_on_plan_stm = self.turn_on_plan_stm_slider,
                                        plan_stm = self.plan_stm_slider,
                                        is_complete = self.is_complete_slider)

### sliders callback
def slider_callback(bag_time, selected_id, force_planning, turn_on_plan_stm, plan_stm, is_complete):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)

  soc_state_msg_idx = local_view_data['data_index']['soc_state_msg_idx']
  fus_parking_msg_idx = local_view_data['data_index']['fus_parking_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  vs_msg_idx = local_view_data['data_index']['vs_msg_idx']
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  plan_msg_idx = local_view_data['data_index']['plan_msg_idx']

  soc_state_input = bag_loader.soc_state_msg['data'][soc_state_msg_idx]
  fus_parking_input = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx]
  loc_msg_input = bag_loader.loc_msg['data'][loc_msg_idx]
  vs_msg_input = bag_loader.vs_msg['data'][vs_msg_idx]
  planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]
  planning_data = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
  origin_plan_output = bag_loader.plan_msg['data'][plan_msg_idx]


  # print("soc_state_input = ", soc_state_input)
  # print("fus_parking_input = ", fus_parking_input)
  # print("loc_msg_input = ", loc_msg_input)
  # print("vs_msg_input = ", vs_msg_input)
  planning_output = planning_plan_pb2.PlanningOutput()

  plan_statemachine = plan_stm
  # if turn_on_plan_stm == 1:
  #   last_seg_name = plan_stm
  # else:
  #   last_seg_name = planning_json['last_segment_name']
  # print("plan_statemachine = ", plan_statemachine)

  # print(origin_plan_output)

  is_replan = planning_json['is_replan']

  print("is_replan = ", is_replan)
  print("ego_yaw_deg = ", loc_msg_input.pose.euler_angles.yaw * 57.3)
  is_replan = False
  try:
    diag_slot_planning_py.UpdateBytesByParam(soc_state_input.SerializeToString(),
                                      fus_parking_input.SerializeToString(),
                                      loc_msg_input.SerializeToString(),
                                      vs_msg_input.SerializeToString(),
                                      planning_data.slot_management_info.SerializeToString(),
                                      selected_id, (force_planning or is_replan), plan_statemachine, is_complete)
    planning_output.ParseFromString(diag_slot_planning_py.GetOutputBytes())
  except:
    print("error")
    pass

  plan_traj_x = []
  plan_traj_y = []
  for i in range(len(planning_output.trajectory.trajectory_points)):
    plan_traj_x.append(planning_output.trajectory.trajectory_points[i].x)
    plan_traj_y.append(planning_output.trajectory.trajectory_points[i].y)

  data_planning_tune.data.update({
    'plan_traj_x': plan_traj_x,
    'plan_traj_y': plan_traj_y,
  })

  # print("planning_output:\n", planning_output)

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
