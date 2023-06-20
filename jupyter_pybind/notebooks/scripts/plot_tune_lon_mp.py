import sys, os
sys.path.append("..")
from lib.load_local_view import *
from lib.load_tune_lon_mp import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, longitudinal_motion_planner_pb2
from jupyter_pybind import longitudinal_motion_planning_py

# bag path and frame dt
bag_path = "/home/xlwang71/Downloads/0614/long_time_hualong_1.00000.1687105422.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

velocity_fig, acc_fig = load_lon_global_figure(bag_loader)

# load longitudinal planning (behavior and motion)
pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig)

# init pybind
longitudinal_motion_planning_py.Init()

lon_motion_plan_input0 = bag_loader.plan_debug_msg['data'][-1].longitudinal_motion_planning_input


### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.q_ref_pos_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_pos",min=0.02, max=100.0, value=lon_motion_plan_input0.q_ref_pos, step=0.005)
    self.q_ref_vel_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_vel",min=0.02, max=100.0, value=lon_motion_plan_input0.q_ref_vel, step=0.01)
    self.q_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc",min=0.0, max=100.0, value=lon_motion_plan_input0.q_acc, step=frame_dt)
    self.q_jerk_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk",min=0.0, max=100.0, value=lon_motion_plan_input0.q_jerk, step=frame_dt)
    self.q_pos_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_pos_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_pos_bound, step=1.0)
    self.q_vel_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_vel_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_vel_bound, step=1.0)
    self.q_acc_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_acc_bound, step=1.0)
    self.q_jerk_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk_bound",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_jerk_bound, step=1.0)
    self.q_stop_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_stop_s",min=0.0, max=3000.0, value=lon_motion_plan_input0.q_stop_s, step=1.0)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                        q_ref_pos = self.q_ref_pos_slider,
                                        q_ref_vel = self.q_ref_vel_slider,
                                        q_acc = self.q_acc_slider,
                                        q_jerk = self.q_jerk_slider,
                                        q_pos_bound = self.q_pos_bound_slider,
                                        q_vel_bound = self.q_vel_bound_slider,
                                        q_acc_bound = self.q_acc_bound_slider,
                                        q_jerk_bound = self.q_jerk_bound_slider,
                                        q_stop_s = self.q_stop_s_slider,
                                        )


### sliders callback
def slider_callback(bag_time, q_ref_pos, q_ref_vel, q_acc, q_jerk, q_pos_bound, q_vel_bound, q_acc_bound, q_jerk_bound, q_stop_s):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data)

  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']

  lon_motion_plan_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].longitudinal_motion_planning_input

  input_string = lon_motion_plan_input.SerializeToString()
  longitudinal_motion_planning_py.UpdateByParams(input_string, q_ref_pos, q_ref_vel, q_acc, q_jerk, q_pos_bound, q_vel_bound, q_acc_bound, q_jerk_bound, q_stop_s)

  planning_output = longitudinal_motion_planner_pb2.LongitudinalPlanningOutput()
  output_string_tmp = longitudinal_motion_planning_py.GetOutputBytes()
  planning_output.ParseFromString(output_string_tmp)

  lon_plan_data['data_lon_motion_plan'].data.update({
    'time_vec' : planning_output.time_vec,
    'pos_vec_t' : planning_output.pos_vec,
    'vel_vec_t' : planning_output.vel_vec,
    'acc_vec_t' : planning_output.acc_vec,
    'jerk_vec_t' : planning_output.jerk_vec,
  })


  push_notebook()

bkp.show(row(fig1, pans), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
