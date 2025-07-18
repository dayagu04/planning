import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_spatio_temporal_union_plan import *
from bokeh.models import DataTable, TableColumn
from bokeh.resources import INLINE
sys.path.append('../..')
sys.path.append('../../../')

# +
# bag path and frame dt
# bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250322/20250322-15-30-02/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-22-15-30-02_no_camera.bag.16-32.split.1745285223.close-loop.scc.plan"
# bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250416/20250416-17-09-02/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2025-04-16-17-09-02_no_camera.bag.1745301727.close-loop.noa.plan"
# bag_path = "/data_cold/abu_zone/datasets/aeb_data/chery_e0y_04228/trigger/20250421/20250421-18-01-07/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-04-21-18-01-07_no_camera.bag.1745391461.close-loop.scc.plan"
# bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250528/20250528-15-20-42/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2025-05-28-15-20-42_no_camera.bag.1749003160.close-loop.scc.plan"
# bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250605/20250605-15-24-23/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2025-06-05-15-24-23_no_camera.bag"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_50818/trigger/20250717/20250717-12-17-05/data_collection_CHERY_M32T_50818_EVENT_FILTER_2025-07-17-12-17-05_no_camera.bag.22-35.split.1752826695.close-loop.scc.plan"

# bag_path = "bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_10034/trigger/20240723/20240723-19-33-25/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-07-23-19-33-25_no_camera.bag
# -

frame_dt = 0.1 # sec

# +
# plot global figure?
# global_fig_plot = True
# -

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

# load lateral planning (behavior and motion)
fig1, spatio_temporal_union_plan_data = load_spatio_temporal_union_plan_figure(fig1)
fig1.height = 1500

fig_cost_time, average_cost_time = load_spatio_temporal_union_cost_time(bag_loader)

fig_dp_result = load_spatio_temporal_union_dp_result(bag_loader)

spatio_temporal_data_1 = ColumnDataSource({
  'name':[],
  'data':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_spatio_temporal_table_1 = DataTable(source=spatio_temporal_data_1, columns=columns, width=400, height=300)

def update_spatio_temporal_data(spatio_temporal_plan_info, average_cost_time):
  vars = ['st_dp_is_sucess','cost_time','enable_using_st_plan']
  vars_average = ['average_cost_time']
  names  = []
  datas = []
  for name in vars:
    try:
      # print(getattr(spatio_temporal_plan_info,name))
      datas.append(getattr(spatio_temporal_plan_info,name))
      names.append(name)
    except:
      pass
  for name in vars_average:
    try:
      print("average_cost_time: ", average_cost_time)
      datas.append(average_cost_time)
      names.append(name)
    except:
      pass

  spatio_temporal_data_1.data.update({
    'name': names,
    'data': datas,
  })

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_spatio_temporal_union_plan_data(bag_loader, bag_time, local_view_data, spatio_temporal_union_plan_data)
  spatio_temporal_plan_info = local_view_data['data_msg']['plan_debug_msg'].spatio_temporal_union_plan
  update_spatio_temporal_data(spatio_temporal_plan_info, average_cost_time)
  push_notebook()

bkp.show(row(fig1, column(data_spatio_temporal_table_1, fig_cost_time, fig_dp_result)), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
