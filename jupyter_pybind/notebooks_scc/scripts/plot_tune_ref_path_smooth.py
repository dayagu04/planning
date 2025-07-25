import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_tune_ref_path_smooth import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from python_proto import fem_pos_deviation_smoother_config_pb2
from python_proto import reference_path_smooth_pb2
from jupyter_pybind import reference_path_smooth_py
from bokeh.resources import INLINE

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20251013/20251013-17-08-22/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-10-13-17-08-22_no_camera.bag.1760674846.open-loop.noa.plan"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20251013/20251013-17-08-22/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-10-13-17-08-22_no_camera.bag.1760679825.open-loop.noa.plan"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20251013/20251013-17-08-22/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-10-13-17-08-22_no_camera.bag.1760680363.open-loop.noa.plan"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20251011/20251011-14-51-38/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-10-11-14-51-38_no_camera.bag.1761707787.close-loop.noa.plan"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20251011/20251011-14-51-38/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-10-11-14-51-38_no_camera.bag.1761718850.close-loop.noa.plan"

frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
# JAC_S811 CHERY_T26 CHERY_E0X CHERY_M32T
global_var.set_value('car_type', 'CHERY_E0X')
global_var.set_value('g_is_display_enu', False)
fig1, local_view_data = load_local_view_figure()
fig1.height = 1500
# init pybind
reference_path_smooth_py.Init()

reference_path_smooth_info0 = bag_loader.plan_debug_msg['data'][-1].reference_path_smooth_info
smoother_config0 = reference_path_smooth_info0.smoother_config

fig1, fig2, fig3, fig4, fig5, fig6, ref_path_data = load_ref_path_figure(fig1)
load_measure_distance_tool(fig1)
fig_time_cost = load_smooth_time_cost(bag_loader)

coord_tf = coord_transformer()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)

    self.is_enable_local = ipywidgets.Checkbox(value=True, description='is_enable_local')
    self.apply_curvature_constraint = ipywidgets.Checkbox(value=smoother_config0.apply_curvature_constraint, description='apply_curvature_constraint')
    self.use_sqp = ipywidgets.Checkbox(value=smoother_config0.use_sqp, description='use_sqp')
    self.verbose = ipywidgets.Checkbox(value=smoother_config0.verbose, description='verbose')
    self.scaled_termination = ipywidgets.Checkbox(value=smoother_config0.scaled_termination, description='scaled_termination')
    self.warm_start = ipywidgets.Checkbox(value=smoother_config0.warm_start, description='warm_start')

    self.sqp_pen_max_iter = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "sqp_pen_max_iter",min=0, max=100, value=smoother_config0.sqp_pen_max_iter, step=1)
    self.sqp_sub_max_iter = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "sqp_sub_max_iter",min=0, max=1000, value=smoother_config0.sqp_sub_max_iter, step=1)
    self.max_iter = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "max_iter",min=0, max=1000, value=smoother_config0.max_iter, step=1)

    self.weight_fem_pos_deviation = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "weight_fem_pos_deviation",min=0.0, max=1.0e11, value=smoother_config0.weight_fem_pos_deviation, step=0.01)
    self.weight_ref_deviation = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "weight_ref_deviation",min=0.0, max=100.0, value=smoother_config0.weight_ref_deviation, step=0.01)
    self.weight_path_length = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "weight_path_length",min=0.0, max=100.0, value=smoother_config0.weight_path_length, step=0.01)
    self.weight_curvature_constraint_slack_var = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "weight_curvature_constraint_slack_var",min=0.0, max=1.0e3, value=smoother_config0.weight_curvature_constraint_slack_var, step=0.01)
    self.curvature_constraint = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "curvature_constraint",min=0.0, max=10.0, value=smoother_config0.curvature_constraint, step=0.01)
    self.sqp_ftol = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "sqp_ftol",min=0.0, max=10.0, value=smoother_config0.sqp_ftol, step=0.001)
    self.sqp_ctol = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "sqp_ctol",min=0.0, max=10.0, value=smoother_config0.sqp_ctol, step=0.001)
    self.time_limit = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "time_limit",min=0.0, max=10.0, value=smoother_config0.time_limit, step=0.001)
    self.bound_val = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "bound_val",min=0.0, max=10.0, value=reference_path_smooth_info0.bound_val, step=0.001)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         is_enable_local = self.is_enable_local,
                                         apply_curvature_constraint = self.apply_curvature_constraint,
                                         use_sqp = self.use_sqp,
                                         verbose = self.verbose,
                                         scaled_termination = self.scaled_termination,
                                         warm_start = self.warm_start,
                                         sqp_pen_max_iter = self.sqp_pen_max_iter,
                                         sqp_sub_max_iter = self.sqp_sub_max_iter,
                                         max_iter = self.max_iter,
                                         weight_fem_pos_deviation = self.weight_fem_pos_deviation,
                                         weight_ref_deviation = self.weight_ref_deviation,
                                         weight_path_length = self.weight_path_length,
                                         weight_curvature_constraint_slack_var = self.weight_curvature_constraint_slack_var,
                                         curvature_constraint = self.curvature_constraint,
                                         sqp_ftol = self.sqp_ftol,
                                         sqp_ctol = self.sqp_ctol,
                                         time_limit = self.time_limit,
                                         bound_val = self.bound_val)

### sliders callback
def slider_callback(bag_time, is_enable_local, apply_curvature_constraint,
                    use_sqp, verbose, scaled_termination, warm_start,
                    sqp_pen_max_iter, sqp_sub_max_iter, max_iter,
                    weight_fem_pos_deviation, weight_ref_deviation,
                    weight_path_length, weight_curvature_constraint_slack_var,
                    curvature_constraint, sqp_ftol, sqp_ctol, time_limit, bound_val):
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  coord_tf = update_tune_ref_path_data(bag_loader, bag_time,  local_view_data, ref_path_data, g_is_display_enu)
  raw_points_x = ref_path_data['data_raw_ref_path'].data['raw_ref_path_xn']
  raw_points_y = ref_path_data['data_raw_ref_path'].data['raw_ref_path_yn']
  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  start_time = time.time()
  reference_path_smooth_py.Update(raw_points_x, raw_points_y,
                                  is_enable_local, apply_curvature_constraint,
                                  use_sqp, verbose, scaled_termination, warm_start,
                                  sqp_pen_max_iter, sqp_sub_max_iter, max_iter,
                                  weight_fem_pos_deviation, weight_ref_deviation,
                                  weight_path_length, weight_curvature_constraint_slack_var,
                                  curvature_constraint, sqp_ftol, sqp_ctol, time_limit, bound_val)
  end_time = time.time()
  print("qp smooth time cost:", end_time - start_time)
  smooth_points_x = reference_path_smooth_py.GetSmoothPointsX()
  smooth_points_y = reference_path_smooth_py.GetSmoothPointsY()
  raw_init_point_x, raw_init_point_y = 0, 0
  if is_enable_local:
    raw_init_point_x = raw_points_x[0]
    raw_init_point_y = raw_points_y[0]
  smooth_ref_path_xn = []
  smooth_ref_path_yn = []
  for i in range(len(smooth_points_x)):
    smooth_ref_path_xn.append(smooth_points_x[i] + raw_init_point_x)
    smooth_ref_path_yn.append(smooth_points_y[i] + raw_init_point_y)

  smooth_ref_path_x, smooth_ref_path_y = coord_tf.global_to_local(smooth_ref_path_xn, smooth_ref_path_yn)
  ref_path_data['data_tune_smooth_ref_path'].data.update({
    'smooth_ref_path_x': smooth_ref_path_x,
    'smooth_ref_path_y': smooth_ref_path_y,
    'smooth_ref_path_xn': smooth_ref_path_xn,
    'smooth_ref_path_yn': smooth_ref_path_yn,
  })

  push_notebook()

bkp.show(row(fig1, column(fig2, fig3, fig4, fig5, fig6, fig_time_cost)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


