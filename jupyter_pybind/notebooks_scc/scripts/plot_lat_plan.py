import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_lat_plan import *
from bokeh.models import DataTable, TableColumn
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_10034/trigger/20241207/20241207-10-49-15/park_in_data_collection_CHERY_E0Y_10034_ALL_FILTER_2024-12-07-10-49-15_no_camera.bag"
# bag_path = "bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_10034/trigger/20240723/20240723-19-33-25/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-07-23-19-33-25_no_camera.bag

frame_dt = 0.1 # sec

# plot global figure?
global_fig_plot = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

# load lateral planning (behavior and motion)
fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9, lat_plan_data = load_lat_plan_figure(fig1)
fig1.height = 1500

fig_lat_offset = load_lateral_offset(bag_loader)

behavior_data_1 = ColumnDataSource({
  'name':[],
  'data':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_behavior_table_1 = DataTable(source=behavior_data_1, columns=columns, width=400, height=300)

def update_lat_behavior_data(lat_behavior_common):
  vars = ['fix_lane_virtual_id','target_lane_virtual_id','origin_lane_virtual_id',\
          'lc_request','lc_request_source','turn_light','map_turn_light','lc_turn_light','act_request_source','lc_back_invalid_reason','lc_status',\
            'is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',\
      'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',\
        'v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',\
          'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',\
            'is_side_borrow_bicycle_lane','is_side_borrow_lane','has_origin_lane',\
              'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason', ]
  # 'near_car_ids_origin','near_car_ids_target', 'left_alc_car_ids','right_alc_car_ids', ,'avoid_car_ids','avoid_car_allow_max_opposite_offset'
  names  = []
  datas = []
  for name in vars:
    try:
      # print(getattr(vo_lat_behavior_plan,name))
      datas.append(getattr(lat_behavior_common,name))
      names.append(name)
    except:
      pass

  behavior_data_1.data.update({
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
  update_lat_plan_data(fig7, bag_loader, bag_time, local_view_data, lat_plan_data, g_is_display_enu)
  lat_behavior_common = local_view_data['data_msg']['plan_debug_msg'].lat_behavior_common
  update_lat_behavior_data(lat_behavior_common)
  push_notebook()

if global_fig_plot:
  bkp.show(row(fig1, column(fig7, fig_lat_offset, data_behavior_table_1), column(fig2, fig9, fig3, fig4, fig5, fig6)), notebook_handle=True)
else:
  bkp.show(row(fig1, column(fig2, fig9, fig3, fig4, fig5, fig6, fig_lat_offset)), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
