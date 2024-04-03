import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_lat_plan import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240525/20240525-15-54-05/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-05-25-15-54-05.bag"
frame_dt = 0.1 # sec

# plot global figure?
global_fig_plot = False

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

# load lateral planning (behavior and motion)
fig1, fig2, fig3, fig4, fig5, fig6, fig7, lat_plan_data = load_lat_plan_figure(fig1)

fig8 = load_lateral_offset(bag_loader)
# obstacle
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
obstacle_data = ColumnDataSource({
  'name':[],
  'data':[]
})
data_obstacle_table = DataTable(source=obstacle_data, columns=columns, width=400, height=500)
# 障碍物id的文本框的回调函数
plan_debug_msg_idx = 0
obj_id = 0
def obj_id_handler(id):
  global obj_id
  obj_id = id
  if bag_loader.plan_debug_msg['enable'] == True:
    environment_model_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].environment_model_info
    obj_vars = ['id','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego', 'vs_lat_relative','vs_lon_relative','vs_lon',
              'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
              'current_lead_obstacle_to_ego','cutin_p']

    names  = []
    datas = []
    is_find = False
    for obstacle in environment_model_info.obstacle:
      if obstacle.id == id:
        is_find = True
        for name in obj_vars:
          try:
            # print(getattr(obstacle,name))
            datas.append(getattr(obstacle,name))
            names.append(name)
          except:
            pass
    if not is_find:
      for obstacle in environment_model_info.obstacle:
        id = obstacle.id
        for name in obj_vars:
          try:
            datas.append(getattr(obstacle,name))
            names.append(name)
          except:
            pass
        break
    # try:
    names.append('ego_s')
    names.append('ego_l')
    datas.append(environment_model_info.ego_s)
    datas.append(environment_model_info.ego_l)
    # except:
      # pass
    obstacle_data.data.update({
      'name': names,
      'data': datas,
    })
  push_notebook()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, g_is_display_enu)

  global plan_debug_msg_idx
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  obj_id_handler(obj_id)
  push_notebook()

if global_fig_plot:
  bkp.show(row(fig1, fig7, column(fig2, fig3, fig4, fig5, fig6), column(data_obstacle_table)), notebook_handle=True)
else:
  bkp.show(row(fig1, column(fig2, fig3, fig4, fig5, fig6, fig8), column(data_obstacle_table)), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
slider_class = ObjText(obj_id_handler)
