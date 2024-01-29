import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = '/data_cold/abu_zone/autoparse/jac_s811_58977/parking/20240123/20240123-10-48-02/park_in__JAC_S811_58977_MANUAL_ALL_2024-01-23-10-48-02_no_camera.record'
frame_dt = 0.1 # sec
plot_ctrl_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, True)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

if plot_ctrl_flag:
  fig2, fig3, fig4, fig5, fig6, fig7, data_ctrl_debug_table = load_local_view_figure_parking_ctrl(bag_loader, local_view_data)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data, plot_ctrl_flag)
  index_map = bag_loader.get_msg_index(bag_time)

  plan_msg = bag_loader.plan_msg['data'][index_map['plan_msg_idx']]
  # print("plan_msg = ", plan_msg)

  planning_json = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
  print("remain_dist = ", planning_json['remain_dist'])

  # print("planning_json = ", planning_json)

  # planning_data = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]

  # print("is_replan_once = ", planning_json['is_replan_once'])
  # print("replan_count = ", planning_json['replan_count'])


  push_notebook()

if plot_ctrl_flag:
  bkp.show(row(fig1, column(fig2, fig3, fig4, fig5), column(fig6, fig7, data_ctrl_debug_table)), notebook_handle=True)
else:
  bkp.show(fig1, notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
