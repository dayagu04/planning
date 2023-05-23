import sys, os
sys.path.append("..")
import inspect
# from lib.load_cyberbag import *
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
# bag path and frame dt
bag_path = "/docker_share/data/clren/code/new_planning_3/planning/20230207055150-plan.record.00000"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
plan_debug_msg_idx = 0
obj_id = 0
### sliders config
class LatBehaviorSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

# 障碍物的id选择
class ObjText:
  def __init__(self,  obj_callback):
    self.id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_id",min=0.0, max=10000)
    ipywidgets.interact(obj_callback, id = self.id)

obstacle_data = ColumnDataSource({
  'name':[],
  'data':[]
})

columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_table = DataTable(source=obstacle_data, columns=columns, width=400, height=800)

# 障碍物id的文本框的回调函数
def obj_id_handler(id):
  global obj_id
  obj_id = id
  if bag_loader.plan_debug_msg['enable'] == True:
    environment_model_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].environment_model_info
    print(plan_debug_msg_idx)
    vars = ['id','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego', 'vs_lat_relative','vs_lon_relative','vs_lon',
              'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
              'current_lead_obstacle_to_ego']
    names  = []
    datas = []
    for obstacle in environment_model_info.obstacle:
      if obstacle.id == id:
        for name in vars:
          try:
            # print(getattr(obstacle,name))
            names.append(name)
            datas.append(getattr(obstacle,name))
          except:
            pass
    obstacle_data.data.update({
      'name': names,
      'data': datas,
    })
  push_notebook()  

def slider_callback(bag_time):
  global plan_debug_msg_idx
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  obj_id_handler(obj_id)
  push_notebook()

bkp.show(row(fig1, data_table), notebook_handle=True)
slider_class = LatBehaviorSlider(slider_callback)
slider_class = ObjText(obj_id_handler)
