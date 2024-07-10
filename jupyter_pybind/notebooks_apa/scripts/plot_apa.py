import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = '/data_cold/abu_zone/autoparse/jac_s811_19cp2/trigger/20240710/20240710-10-47-48/park_in_data_collection_JAC_S811_19CP2_ALL_FILTER_2024-07-10-10-47-49_no_camera.record'
frame_dt = 0.1 # sec
plot_ctrl_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, True)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

data_obs_slm_filtered = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=2, source=data_obs_slm_filtered, color='blue', legend_label='obs after filtered by slm')
# Define the JavaScript callback code
callback_code = """
    var x = cb_obj.x;
    var y = cb_obj.y;

    source.data['x'].push(x);
    source.data['y'].push(y);

    if (source.data['x'].length > 2) {
        source.data['x'].shift();
        source.data['y'].shift();
        source.data['x'].shift();
        source.data['y'].shift();
    }
    source.change.emit();

    if (source.data['x'].length >= 2) {
        var x1 = source.data['x'][source.data['x'].length - 2];
        var y1 = source.data['y'][source.data['y'].length - 2];
        var x2 = x;
        var y2 = y;
        var x3 = (x1 + x2) / 2;
        var y3 = (y1 + y2) / 2;

        var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        console.log("Distance between the last two points: " + distance);

        distance = distance.toFixed(4);
        text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
        text_source.change.emit();

        line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
        line_source.change.emit();
    }

    if (source.data['x'].length == 1) {
        text_source.data['x'].shift();
        text_source.data['y'].shift();
        text_source.data['text'].shift();
    }
    text_source.change.emit();
"""

# Create a CustomJS callback with the defined code
callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)

if plot_ctrl_flag:
  fig2, fig3, fig4, fig5, fig6, fig7, data_ctrl_debug_table = load_local_view_figure_parking_ctrl(bag_loader, local_view_data)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.vehicle_type = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=2, value=0, step=1)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         vehicle_type = self.vehicle_type)


### sliders callback
def slider_callback(bag_time, vehicle_type):
  kwargs = locals()

  if vehicle_type == 0:
    vehicle_type = 'JAC_S811'
  elif vehicle_type == 1:
    vehicle_type = 'CHERY_T26'
  elif vehicle_type == 2:
    vehicle_type = 'CHERY_E0X'

  update_local_view_data_parking(fig1, bag_loader, bag_time, vehicle_type, local_view_data, plot_ctrl_flag)
  index_map = bag_loader.get_msg_index(bag_time)

  if bag_loader.plan_msg['enable'] == True:
    plan_msg = bag_loader.plan_msg['data'][index_map['plan_msg_idx']]
    # print("plan_msg = ", plan_msg.trajectory.trajectory_points)
    # print("plan_release_slots_id = ", plan_msg.successful_slot_info_list)

  if bag_loader.plan_debug_msg['enable'] == True:
    planning_json = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
    print("remain_dist = ", planning_json['remain_dist'])
    print("remain_dist_uss =", planning_json['remain_dist_uss'])
    # print("path plan time =", planning_json['path_plan_time_ms'])

  if bag_loader.fus_parking_msg['enable'] == True:
    fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
    # for i in range(len(fus_parking_msg.parking_fusion_slot_lists)):
    #   slot_info = fus_parking_msg.parking_fusion_slot_lists[i]
    #   print("slot_id = ", slot_info.id, "  slot_type = ", slot_info.type,
    #         "slot_allow_parking = ", slot_info.allow_parking, "slot_fusion_source = ", slot_info.fusion_source,
    #         "slot_slot_side = ", slot_info.slot_side, "slot_uss_id = ", slot_info.uss_id,
    #         )

  if bag_loader.uss_percept_msg['enable'] == True:
    uss_percept_msg = bag_loader.uss_percept_msg['data'][index_map['uss_percept_msg_idx']]
  #print("fus_parking_msg",fus_parking_msg.parking_fusion_slot_lists)
  # replan_time_list, correct_path_for_limiter_list = [],[]
  # if planning_json['replan_flag'] == True:
  #   replan_time_list.append(bag_time)

  # if planning_json['correct_path_for_limiter'] == True:
  #   replan_time_list.append(bag_time)

  # print("replan_time_list = ", replan_time_list)
  # print("correct_path_for_limiter_list = ", correct_path_for_limiter_list)

  # print("path plan time =", planning_json['path_plan_time_ms'])
  # print("tlane_p1 =", planning_json['tlane_p1_x'], ", ", planning_json['tlane_p1_y'])
  # print("tlane_p0 =", planning_json['tlane_p0_x'], ", ", planning_json['tlane_p0_y'])

  # print("planning_json = ", planning_json)

  # planning_data = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]

  # print("is_replan_once = ", planning_json['is_replan_once'])
  # print("replan_count = ", planning_json['replan_count'])

  ## debug for detecting distance in uss perception
  # if (len(uss_percept_msg.out_line_dataori) > 4):
  #   print("---------------")
  #   print(uss_percept_msg.out_line_dataori[4].dis_from_car_to_obj)

  # print("obs filtered for selected slot by slm")
  # data_obs_slm_filtered.data.update({
  #     'y': [],
  #     'x': []
  # })
  # slm_selected_obs_x = planning_json['slm_selected_obs_x']
  # slm_selected_obs_y = planning_json['slm_selected_obs_y']
  # print("slm_selected_obs_x size = ", len(slm_selected_obs_x))

  # if len(slm_selected_obs_x) > 1:
  #   data_obs_slm_filtered.data.update({
  #       'y': slm_selected_obs_x,
  #       'x': slm_selected_obs_y
  #   })
  #   # for i in range(len(slm_selected_obs_x)):
  #   #   print(slm_selected_obs_x[i], ", ", slm_selected_obs_y[i])

  #   print("para_tlane_side_sgn", planning_json['para_tlane_side_sgn'])
  #   print("para_tlane_is_front_vacant",planning_json['para_tlane_is_front_vacant'])
  #   print("para_tlane_is_rear_vacant",planning_json['para_tlane_is_rear_vacant'])
  #   tlane = planning_json['para_tlane_obs_pt_before_uss']
  #   print("para_tlane_obs_pt_before_uss = (", tlane[0], ", ", tlane[1], ")  (", tlane[2], ", ",tlane[3],")")
  #   # tlane_after_uss = planning_json['para_tlane_obs_pt_after_uss']
  #   # print("para_tlane_obs_pt_after_uss = (", tlane_after_uss[0], ", ", tlane_after_uss[1], ")  (", tlane_after_uss[2], ", ",tlane_after_uss[3],")")
  #   print("slot length = ", planning_json['slot_length'])
  #   print("slot width = ", planning_json['slot_width'])

  #   print("------------front---------------")
  #   print("para_tlane_front_min_x_before_clamp", planning_json['para_tlane_front_min_x_before_clamp'])
  #   print("para_tlane_front_min_x_after_clamp", planning_json['para_tlane_front_min_x_after_clamp'])

  #   print("tlane_front_que_x:-----------------------")
  #   for i in range(len(planning_json['tlane_front_que_x'])):
  #     print("( ", planning_json['tlane_front_que_x'][i], ", ", planning_json['tlane_front_que_y'][i], ")  ")

  #   print("--------------rear-------------")
  #   print("para_tlane_rear_max_x_before_clamp", planning_json['para_tlane_rear_max_x_before_clamp'])
  #   print("para_tlane_rear_max_x_after_clamp", planning_json['para_tlane_rear_max_x_after_clamp'])

  #   print("tlane_rear_que_x:-----------------------")
  #   for i in range(len(planning_json['tlane_rear_que_x'])):
  #     print("( ", planning_json['tlane_rear_que_x'][i], ", ", planning_json['tlane_rear_que_y'][i], ")  ")
  # print("----------------------------------------------")

  push_notebook()

if plot_ctrl_flag:
  bkp.show(row(fig1, column(fig2, fig3, fig4, fig5), column(fig6, fig7, data_ctrl_debug_table)), notebook_handle=True)
else:
  bkp.show(fig1, notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
