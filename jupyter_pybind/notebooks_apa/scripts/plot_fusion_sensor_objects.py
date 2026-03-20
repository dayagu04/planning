import sys, os, copy
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import planning_plan_pb2


# plot info
# localization
# car history path
# slot
# uss
# fusion info
# ground line info


# bag path and frame dt
# bag_path = '/docker_share/common_obs.record'
# bag_path = '/docker_share/road_bound.record'
# bag_path = '/docker_share/astar_0622_1/test_1.00000'
bag_path = '/docker_share/astar_0624_1/test_0.00000'
frame_dt = 0.1  # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source,
            color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color='pink',
          line_dash='solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red',
          text_align='center', text_font_size='15pt', legend_label='measure tool')

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
callback = CustomJS(args=dict(source=source, line_source=line_source,
                    text_source=text_source), code=callback_code)

# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)


# sliders config
class LocalViewSlider:
    def __init__(self,  slider_callback):
        self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='75%'), description="bag_time", min=0.0, max=max_time, value=-0.1, step=frame_dt)
        self.select_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(
            width='18%'), description="select_id", min=0, max=20, value=0, step=1)

        ipywidgets.interact(slider_callback,
                            bag_time=self.time_slider,
                            select_id=self.select_id_slider
                            )

# ## sliders callback


def slider_callback(bag_time, select_id):
    kwargs = locals()
    vehicle_type = 'JAC_S811'
    update_local_view_data_parking(fig1, bag_loader, bag_time,vehicle_type, local_view_data)
    index_map = bag_loader.get_msg_index(bag_time)

    if index_map['plan_debug_msg_idx'] < len(bag_loader.plan_debug_msg['json']):
        plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]

    if (index_map['fus_parking_msg_idx'] < len(bag_loader.fus_parking_msg['data'])):
        fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]

    if (index_map['wave_msg_idx'] < len(bag_loader.wave_msg['data'])):
        wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]

    if (index_map['vs_msg_idx'] < len(bag_loader.vs_msg['data'])):
        vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]

    if (index_map['soc_state_msg_idx'] < len(bag_loader.soc_state_msg['data'])):
        soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]

    try:
        uss_perception_msg = bag_loader.uss_percept_msg['data'][index_map['uss_percept_msg_idx']]
    except Exception:
        uss_perception_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]

    if (index_map['loc_msg_idx'] < len(bag_loader.loc_msg['data'])):
        loc_msg = copy.deepcopy(
            bag_loader.loc_msg['data'][index_map['loc_msg_idx']])

    if (index_map['plan_debug_msg_idx'] < len(bag_loader.plan_debug_msg['data'])):
        slot_management_info = bag_loader.plan_debug_msg['data'][
            index_map['plan_debug_msg_idx']].slot_management_info

    if (index_map['fus_parking_msg_idx'] < len(bag_loader.fus_parking_msg['data'])):
        select_slot_id = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']].select_slot_id

    if (index_map['fus_msg_idx'] < len(bag_loader.fus_msg['data'])):
        fusion_obj_msg = bag_loader.fus_msg['data'][index_map['fus_msg_idx']]

        print('fusion_obj_msg ' , fusion_obj_msg.fusion_object_num)
        for i in range(fusion_obj_msg.fusion_object_num):
            obj = fusion_obj_msg.fusion_object[i].additional_info.polygon
            print('num ' , obj.num)


    if index_map['ground_line_msg_idx'] < len(bag_loader.gl_msg['data']):
        ground_line_msg = bag_loader.gl_msg['data'][index_map['ground_line_msg_idx']]

        print('size',len(ground_line_msg.ground_lines))

        for i in range(len(ground_line_msg.ground_lines)):
            obj = ground_line_msg.ground_lines[i]
            print('2d num ' , len(obj.points_2d))
            print('3d num ' , len(obj.points_3d))

    target_managed_slot_x_vec = []
    target_managed_slot_y_vec = []
    for i in range(len(slot_management_info.slot_info_vec)):
        maganed_slot_vec = slot_management_info.slot_info_vec[i]
        corner_point = maganed_slot_vec.corner_points.corner_point
        if maganed_slot_vec.id == select_slot_id:
            target_managed_slot_x_vec = [
                corner_point[0].x, corner_point[1].x, corner_point[2].x, corner_point[3].x]
            target_managed_slot_y_vec = [
                corner_point[0].y, corner_point[1].y, corner_point[2].y, corner_point[3].y]

    target_managed_slot_x_vec = []
    target_managed_slot_y_vec = []
    target_managed_limiter_x_vec = []
    target_managed_limiter_y_vec = []
    if soc_state_msg.current_state >= 26:
        target_managed_slot_x_vec = plan_debug_msg['slot_corner_X']
        target_managed_slot_y_vec = plan_debug_msg['slot_corner_Y']
        target_managed_limiter_x_vec = plan_debug_msg['limiter_corner_X']
        target_managed_limiter_y_vec = plan_debug_msg['limiter_corner_Y']

    current_ego_x = loc_msg.pose.local_position.x
    current_ego_y = loc_msg.pose.local_position.y

    print("ego_position ", current_ego_x, current_ego_y)

    push_notebook()


bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
