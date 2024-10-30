import sys, os
import copy
sys.path.append("..")
from io import BytesIO
from lib.load_local_view_parking import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from jupyter_pybind import prepareplan_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

data_mono_circle = ColumnDataSource(data = {'mono_circle_y':[], 'mono_circle_x':[], 'mono_circle_r':[]})
data_prepareline = ColumnDataSource(data = {'prepareline_y': [], 'prepareline_x': [],})
data_pt_inside = ColumnDataSource(data = {'pt_inside_y':[], 'pt_inside_x':[]})
data_tangent_point = ColumnDataSource(data = {'tangent_point_y':[], 'tangent_point_x':[]})

fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=800, match_aspect = True, aspect_scale=1)
fig1.line('prepareline_y', 'prepareline_x', source = data_prepareline, line_width = 3, line_color = 'blue', line_dash = 'solid',legend_label = 'prepareline')
fig1.circle(x = 'mono_circle_y', y = 'mono_circle_x', radius = 'mono_circle_r', source = data_mono_circle, size = 8, line_alpha = 1, line_width = 2, line_color = "red", fill_alpha=0, legend_label = 'mono_circle')
fig1.circle(x = 'mono_circle_y', y = 'mono_circle_x', source = data_mono_circle, size = 8, line_alpha = 1, line_width = 2, line_color = "red", fill_alpha=0, legend_label = 'mono_circle')
fig1.circle('pt_inside_y', 'pt_inside_x', source = data_pt_inside, size=8, color='green', legend_label='pt_inside')
fig1.circle('tangent_point_y', 'tangent_point_x', source = data_tangent_point, size=10, color='orange', legend_label='tangent_point')

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


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'


prepareplan_py.InitTargetInfo()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.x_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "x_offset",min=2.0, max=10.0, value=2.1, step=0.1)
    self.heading_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "heading_offset",min=8.8, max=26.8, value=9.01, step=1.0)
    self.prepareline_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "length",min=2.0, max=10.0, value=8.0, step=1.0)
    self.slot_side_sign_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_side_sign",min=-1.0, max=1.0, value=-1.0, step=2.0)
    ipywidgets.interact(slider_callback, x_offset = self.x_offset_slider,
                                        heading_offset = self.heading_offset_slider,
                                        length = self.prepareline_slider,
                                        slot_side_sign = self.slot_side_sign_slider)

def slider_callback(x_offset, heading_offset,length, slot_side_sign):
  kwargs = locals()
  prepareplan_py.Update(x_offset, heading_offset,length, slot_side_sign)
  pt_inside_x_vec = []
  pt_inside_y_vec = []
  mono_circle_x_vec = []
  mono_circle_y_vec = []
  mono_circle_r_vec = []
  prepareline_x_vec = []
  prepareline_y_vec = []
  tangent_point_x_vec = []
  tangent_point_y_vec = []

  pt_inside = prepareplan_py.GetPt_0()
  pt_inside_x_vec.append(pt_inside[0])
  pt_inside_y_vec.append(pt_inside[1])

  mono_circle_point = prepareplan_py.GetMonoCircleCenter()
  mono_circle_radius = prepareplan_py.GetMonoCircleRadius()
  mono_circle_x_vec.append(mono_circle_point[0])
  mono_circle_y_vec.append(mono_circle_point[1])
  mono_circle_r_vec.append(mono_circle_radius)

  prepareline_pa = prepareplan_py.GetPreLinePointA()
  prepareline_pb = prepareplan_py.GetPreLinePointB()
  prepareline_x_vec.append(prepareline_pa[0])
  prepareline_x_vec.append(prepareline_pb[0])
  prepareline_y_vec.append(prepareline_pa[1])
  prepareline_y_vec.append(prepareline_pb[1])

  tangent_point = prepareplan_py.GetTangentPoint()
  tangent_point_x_vec.append(tangent_point[0])
  tangent_point_y_vec.append(tangent_point[1])

  data_mono_circle.data.update({'mono_circle_y':mono_circle_y_vec, 'mono_circle_x':mono_circle_x_vec, 'mono_circle_r':mono_circle_r_vec})
  data_prepareline.data.update({'prepareline_y': prepareline_y_vec, 'prepareline_x': prepareline_x_vec,})
  data_pt_inside.data.update({'pt_inside_y':pt_inside_y_vec, 'pt_inside_x':pt_inside_x_vec})
  data_tangent_point.data.update({'tangent_point_y':tangent_point_y_vec, 'tangent_point_x':tangent_point_x_vec})
  push_notebook()
bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
