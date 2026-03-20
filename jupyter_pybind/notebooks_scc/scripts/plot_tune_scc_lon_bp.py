import sys, os
sys.path.append("..")
from lib.load_local_view import *
from lib.load_lon_plan import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, task_basic_types_pb2, real_time_lon_behavior_planner_pb2
from jupyter_pybind import real_time_lon_behavior_planning_py

# bag path and frame dt
bag_path = "/docker_share/urban_bag_0213/yunsu40_2.00000.1693964013.plan"
cfg_path = "/asw/planning/res/conf/module_configs"
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

velocity_fig, acc_fig, obs_st_ids = load_lon_global_figure(bag_loader)

# load lateral planning (behavior and motion)
pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig, obs_st_ids)

# init pybind
real_time_lon_behavior_planning_py.Init(cfg_path)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.1, step=frame_dt)
    self.q_safe_dis_base_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_safe_dis_base",min=0.0, max=10.0, value=3.0, step=1.0)
    self.q_safe_dis_ttc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_safe_dis_ttc",min=0.0, max=1.0, value=0.3, step=0.1)
    self.q_t_actor_delay_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_t_actor_delay",min=0.0, max=1.0, value=0.4, step=0.1)
    self.q_lk_cutinp_thred_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_lk_cutinp_thred",min=0.0, max=1.0, value=0.2, step=0.1)
    self.q_lc_cutinp_thred_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_lc_cutinp_thred",min=0.0, max=1.0, value=0.6, step=0.1)
    self.q_corridor_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_corridor_width",min=0.0, max=3.0, value=1.5, step=0.3)
    self.q_preview_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_preview_x",min=0.0, max=200, value=80.0, step=10)
    self.q_dis_zero_speed_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_dis_zero_speed",min=0.0, max=10.0, value=3.5, step=0.5)
    self.q_dis_zero_speed_accident_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_dis_zero_speed_accident",min=0.0, max=10.0, value=6, step=0.5)
    self.q_ttc_brake_hyst_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ttc_brake_hyst",min=0.0, max=1.0, value=0.3, step=0.1)
    self.q_t_curv_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_t_curv",min=0.0, max=10.0, value=3.0, step=0.5)
    self.q_dis_curv_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_dis_curv",min=0.0, max=50.0, value=0.0, step=5.0)
    self.q_vel_upper_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_vel_upper_bound",min=0.0, max=66.66, value=33.33, step=0.33)
    self.q_v_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_v_start",min=0.0, max=0.5, value=0.3, step=0.05)
    self.q_dis_stop_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_dis_stop",min=0.0, max=5.0, value=1.0, step=0.5)
    self.q_dis_start_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_dis_start",min=0.0, max=3.0, value=0.3, step=0.1)
    
    """ tuned_params_dict = {}
    tuned_params_dict['q_safe_dis_base'] = self.q_safe_dis_base_slider
    tuned_params_dict['q_safe_dis_ttc'] = self.q_safe_dis_ttc_slider
    tuned_params_dict['q_t_actor_delay'] = self.q_t_actor_delay_slider
    tuned_params_dict['q_lk_cutinp_thred'] = self.q_lk_cutinp_thred_slider
    tuned_params_dict['q_lc_cutinp_thred'] = self.q_lc_cutinp_thred_slider
    tuned_params_dict['q_corridor_width'] = self.q_corridor_width_slider
    tuned_params_dict['q_preview_x'] = self.q_preview_x_slider
    tuned_params_dict['q_dis_zero_speed'] = self.q_dis_zero_speed_slider
    tuned_params_dict['q_dis_zero_speed_accident'] = self.q_dis_zero_speed_accident_slider
    tuned_params_dict['q_ttc_brake_hyst'] = self.q_ttc_brake_hyst_slider
    tuned_params_dict['q_t_curv'] = self.q_t_curv_slider
    tuned_params_dict['q_dis_curv'] = self.q_dis_curv_slider
    tuned_params_dict['q_vel_upper_bound'] = self.q_vel_upper_bound_slider
    tuned_params_dict['q_v_start'] = self.q_v_start_slider
    tuned_params_dict['q_dis_stop'] = self.q_dis_stop_slider
    tuned_params_dict['q_dis_start'] = self.q_dis_start_slider

    ipywidgets.interact(slider_callback, bag_time = self.time_slider, params = tuned_params_dict) """
    
    ipywidgets.interact(slider_callback, bag_time = self.time_slider, 
                        q_safe_dis_base = self.q_safe_dis_base_slider,
                        q_safe_dis_ttc = self.q_safe_dis_ttc_slider,
                        q_t_actor_delay = self.q_t_actor_delay_slider,
                        q_lk_cutinp_thred = self.q_lk_cutinp_thred_slider,
                        q_lc_cutinp_thred = self.q_lc_cutinp_thred_slider,
                        q_corridor_width = self.q_corridor_width_slider,
                        q_preview_x = self.q_preview_x_slider,
                        q_dis_zero_speed = self.q_dis_zero_speed_slider,
                        q_dis_zero_speed_accident = self.q_dis_zero_speed_accident_slider,
                        q_ttc_brake_hyst = self.q_ttc_brake_hyst_slider,
                        q_t_curv = self.q_t_curv_slider,
                        q_dis_curv = self.q_dis_curv_slider,
                        q_vel_upper_bound = self.q_vel_upper_bound_slider,
                        q_v_start = self.q_v_start_slider,
                        q_dis_stop = self.q_dis_stop_slider,
                        q_dis_start = self.q_dis_start_slider,)

### sliders callback
def slider_callback(bag_time, q_safe_dis_base, q_safe_dis_ttc,
                    q_t_actor_delay, q_lk_cutinp_thred, q_lc_cutinp_thred, q_corridor_width,
                    q_preview_x, q_dis_zero_speed, q_dis_zero_speed_accident, q_ttc_brake_hyst,
                    q_t_curv, q_dis_curv, q_vel_upper_bound, q_v_start, q_dis_stop, q_dis_start):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  tuned_params = real_time_lon_behavior_planner_pb2.RealTimeLonBehaviorTunedParams()
  
  """ 
  tuned_params.safe_distance_base = params['q_safe_dis_base']
  tuned_params.safe_distance_ttc = params['q_safe_dis_ttc']
  tuned_params.t_actuator_delay = params['q_t_actor_delay']
  tuned_params.lane_keep_cutinp_threshold = params['q_lk_cutinp_thred']
  tuned_params.lane_change_cutinp_threshold = params['q_lc_cutinp_thred']
  tuned_params.corridor_width = params['q_corridor_width']
  tuned_params.preview_x = params['q_preview_x']
  tuned_params.dis_zero_speed = params['q_dis_zero_speed']
  tuned_params.dis_zero_speed_accident = params['q_dis_zero_speed_accident']
  tuned_params.ttc_brake_hysteresis = params['q_ttc_brake_hyst']
  tuned_params.t_curv = params['q_t_curv']
  tuned_params.dis_curv = params['q_dis_curv']
  tuned_params.velocity_upper_bound = params['q_vel_upper_bound']
  tuned_params.v_start = params['q_v_start']
  tuned_params.distance_stop = params['q_dis_stop']
  tuned_params.distance_start = params['q_dis_start'] """

  tuned_params.safe_distance_base = q_safe_dis_base
  tuned_params.safe_distance_ttc = q_safe_dis_ttc
  tuned_params.t_actuator_delay = q_t_actor_delay
  tuned_params.lane_keep_cutinp_threshold = q_lk_cutinp_thred
  tuned_params.lane_change_cutinp_threshold = q_lc_cutinp_thred
  tuned_params.corridor_width = q_corridor_width
  tuned_params.preview_x = q_preview_x
  tuned_params.dis_zero_speed = q_dis_zero_speed
  tuned_params.dis_zero_speed_accident = q_dis_zero_speed_accident
  tuned_params.ttc_brake_hysteresis = q_ttc_brake_hyst
  tuned_params.t_curv = q_t_curv
  tuned_params.dis_curv = q_dis_curv
  tuned_params.velocity_upper_bound = q_vel_upper_bound
  tuned_params.v_start = q_v_start
  tuned_params.distance_stop = q_dis_stop
  tuned_params.distance_start = q_dis_start
  
  params_string = tuned_params.SerializeToString()
  real_time_lon_behavior_planning_py.SetConfigFromPy(params_string)

  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  lon_behavior_plan_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].real_time_lon_behavior_planning_input
  input_string = lon_behavior_plan_input.SerializeToString()

  real_time_lon_behavior_planning_py.Update(input_string)

  planning_output = real_time_lon_behavior_planner_pb2.LonRefPath()
  output_string_tmp = real_time_lon_behavior_planning_py.GetOutputBytes()
  planning_output.ParseFromString(output_string_tmp)
  
  update_lon_ref_path(planning_output, lon_plan_data)

  push_notebook()

bkp.show(row(fig1, pans), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
