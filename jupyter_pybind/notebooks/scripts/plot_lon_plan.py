import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
from lib.load_lon_plan import *
sys.path.append('../..')

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

# bag path and frame dt
bag_path = "/docker_share/bags/record_tmp/planning_result_bag_0525.00000"
frame_dt = 0.1

bag_loder = LoadCyberbag(bag_path)
bag_loder.load_all_data()

bag_loder.plan_msg['data'][0]
# bag_loder.plan_debug_msg['data'][0]

lon_plan_plot_data, fig1, fig2 = load_lon_plan_figure()

# sliders
class PlanningSlider:
  def __init__(self, silder_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=100, value=-0.1, step=frame_dt)
    ipywidgets.interact(silder_callback, bag_time = self.time_slider)


def silder_callback(bag_time):
  kwargs = locals()

  update_lon_plan_figure(bag_time, bag_loder, lon_plan_plot_data)
  push_notebook()


bkp.show(row(fig1, fig2), notebook_handle=True)
slider_class = PlanningSlider(silder_callback)
