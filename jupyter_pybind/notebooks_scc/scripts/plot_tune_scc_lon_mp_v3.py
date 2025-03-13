import sys, os
sys.path.append("..")
from lib.load_local_view import *
from lib.load_tune_lon_mp_v3 import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from python_proto import longitudinal_motion_planner_pb2
from jupyter_pybind import scc_lon_motion_planning_v3_py

# bag path and frame dt
#bag_path = "/mnt/s811_1_0907/motion_14.00000"
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_14520/trigger/20241206/20241206-15-48-40/data_collection_CHERY_E0Y_14520_EVENT_MANUAL_2024-12-06-15-48-40_no_camera.bag.1733887758.close-loop.scc.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
global_var.set_value('g_is_display_enu', False)
fig1, local_view_data = load_local_view_figure()

velocity_fig, acc_fig = load_lon_global_figure(bag_loader)

# load longitudinal planning (behavior and motion)
pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig)

# init pybind
scc_lon_motion_planning_v3_py.Init()

lon_motion_plan_input0 = bag_loader.plan_debug_msg['data'][-1].longitudinal_motion_planning_input

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.q_default_s_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "s_ref_default",min=0.0, max=100.0, value=3.0, step=0.1)
    self.q_follow_s_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "s_ref_follow",min=0.0, max=100.0, value=3.0, step=0.1)
    self.q_overtake_s_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "s_ref_overtake",min=0.0, max=100.0, value=3.0, step=0.1)
    self.q_neighbor_s_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "s_ref_neighbor",min=0.0, max=100.0, value=3.0, step=0.1)
    self.q_default_v_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "v_ref_default",min=0.0, max=100.0, value=0.0, step=0.5)
    self.q_cruise_v_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "v_ref_cruise",min=0.0, max=100.0, value=40.0, step=0.5)
    self.q_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc",min=0.0, max=300.0, value=lon_motion_plan_input0.q_acc, step=frame_dt)
    self.q_jerk_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk",min=0.0, max=300.0, value=lon_motion_plan_input0.q_jerk, step=frame_dt)
    self.q_hard_pos_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_hard_pos_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_hard_pos_bound, step=1.0)
    self.q_vel_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_vel_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_vel_bound, step=1.0)
    self.q_acc_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_acc_bound, step=1.0)
    self.q_jerk_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_jerk_bound, step=1.0)
    self.q_sv_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_sv_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_sv_bound, step=1.0)
    self.q_stop_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_stop_s",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_stop_s, step=1.0)
    self.q_const_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_const_s",min=0.0, max=100.0, value=10.0, step=0.1)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         q_default_s_weight = self.q_default_s_weight_slider,
                                         q_follow_s_weight = self.q_follow_s_weight_slider,
                                         q_overtake_s_weight = self.q_overtake_s_weight_slider,
                                         q_neighbor_s_weight = self.q_neighbor_s_weight_slider,
                                         q_default_v_weight = self.q_default_v_weight_slider,
                                         q_cruise_v_weight = self.q_cruise_v_weight_slider,
                                         q_acc = self.q_acc_slider,
                                         q_jerk = self.q_jerk_slider,
                                         q_hard_pos_bound = self.q_hard_pos_bound_slider,
                                         q_vel_bound = self.q_vel_bound_slider,
                                         q_acc_bound = self.q_acc_bound_slider,
                                         q_jerk_bound = self.q_jerk_bound_slider,
                                         q_stop_s = self.q_stop_s_slider,
                                         q_const_s = self.q_const_s_slider)

### sliders callback
def slider_callback(bag_time, q_default_s_weight, q_follow_s_weight, q_overtake_s_weight, q_neighbor_s_weight, q_default_v_weight,
                    q_cruise_v_weight, q_acc, q_jerk, q_hard_pos_bound, q_vel_bound, q_acc_bound, q_jerk_bound, q_stop_s, q_const_s):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data)

  planning_debug_info = local_view_data['data_msg']['plan_debug_msg']
  lon_motion_plan_input = planning_debug_info.longitudinal_motion_planning_input

  ## get target_type/is_urgent/lon_state_v/urgent_scale
  weight_maker_replay_info = planning_debug_info.weight_maker.weight_maker_replay_info
  ## print(weight_maker_replay_info)
  target_type_vec = []
  for item in (weight_maker_replay_info.target_point):
    target_type_vec.append(item.target_type)
  is_urgent = weight_maker_replay_info.is_urgent
  lon_state_v = lon_motion_plan_input.init_state.v
  urgent_scale = weight_maker_replay_info.urgent_scale

  input_string = lon_motion_plan_input.SerializeToString()
  scc_lon_motion_planning_v3_py.UpdateByParams(input_string, q_acc,q_jerk, q_hard_pos_bound, q_vel_bound, q_acc_bound, q_jerk_bound, q_stop_s,
                                               q_default_s_weight, q_follow_s_weight, q_overtake_s_weight, q_neighbor_s_weight, target_type_vec,
                                               is_urgent, lon_state_v, urgent_scale, q_default_v_weight, q_cruise_v_weight, q_const_s)

  planning_output = longitudinal_motion_planner_pb2.LongitudinalPlanningOutput()
  output_string_tmp = scc_lon_motion_planning_v3_py.GetOutputBytes()
  planning_output.ParseFromString(output_string_tmp)
  # print(planning_output)

  lon_plan_data['data_lon_motion_plan'].data.update({
    'time_vec' : planning_output.time_vec,
    'pos_vec_t' : planning_output.pos_vec,
    'vel_vec_t' : planning_output.vel_vec,
    'acc_vec_t' : planning_output.acc_vec,
    'jerk_vec_t' : planning_output.jerk_vec,
  })

  # print("lon_motion_plan_output:=", lon_motion_plan_output)
  motion_solver_info = planning_output.solver_info
  iter_count = motion_solver_info.iter_count
  cost_size = motion_solver_info.cost_size
  cost_vec = motion_solver_info.cost_vec
  lists = [cost_vec[i * cost_size : (i + 1) * cost_size] for i in range(iter_count)]
  cost_list = ["ReferenceCost", "LonAccCost", "LonJerkCost", "LonHardPosBoundCost", \
               "LonVelBoundCost", "LonAccBoundCost", "LonJerkBoundCost", "LonStopPointCost", "NonNegativeVelCost"]
  print(cost_list)
  for i, sub_list in enumerate(lists):
    if i == 0:
      print(f"Cost init: {sub_list}")
    else:
      print(f"Cost {i}: {sub_list}")

  push_notebook()

bkp.show(row(fig1, pans), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)




