import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = '/data_cold/abu_zone/APA/planning-d688abcd-CHERY_T26/test_4.00000'
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
