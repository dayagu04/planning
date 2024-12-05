import json
from math import pi
import bokeh
import sys, os, copy
sys.path.append("..")
from io import BytesIO
from lib.load_local_view_parking import *
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
sys.path.append('../../../build/devel/lib/python3/dis-packagers')
from bokeh.plotting import figure, show, output_file
from lib.load_rotate import *


def construct_scenario(
      slot_width = 2.2,
      slot_length = 6.0,
      curb_offset = 0.4,
      dx = 0.4,
      channel_width = 5.5,
      front_car_y_offset = 0.4,
      front_car_heading = 5.0 / 57.3,

      rear_car_y_offset = 0.0,
      rear_car_heading = -2.0 / 57.3,
      is_front_occupied = True,
      is_rear_occupied = True):

  # channel car
  x_offset_vec = [1.2, 3.6, 6.0, 8.9,12.3]
    # y_offset_vec = [-0.9, 0.3, 0.6, 0.2, 0.4]
  y_offset_vec = [-0.3, 0.3, 0.6, 0.2, 0.4]
  heading_offset_vec = [0.0, 0.0, 0.0, 6.0 /57.3, 70 / 57.3]

  curb_file_name = '/asw/data/curb.json'
  obs_car_file_name = '/asw/data/parallel_obs_car_0p1.json'
  # display(HTML("<style>.container { width:95% !important;  }</style>"))
  # output_notebook()

  # load json
  with open(obs_car_file_name, 'r') as file:
      obs_car = json.load(file)

  obs_x_vec = []
  obs_y_vec = []
  obs_to_slot_line = dx * 0.5
  front_obs_car_matrix = []

  if is_front_occupied:
      new_obs_car_x_vec = []
      new_obs_car_y_vec = []
      for i in range(len(obs_car['obs_x'])):
        tmp_x, tmp_y = rotate(obs_car['obs_x'][i],obs_car['obs_y'][i], front_car_heading)
        tmp_y += front_car_y_offset
        new_obs_car_x_vec.append(tmp_x)
        new_obs_car_y_vec.append(tmp_y)

      min_x = min(new_obs_car_x_vec)
      for i in range(len(new_obs_car_x_vec)):
         new_obs_car_x_vec[i] += - min_x + slot_length + obs_to_slot_line

      obs_x_vec += new_obs_car_x_vec
      obs_y_vec += new_obs_car_y_vec

      front_obs_car_matrix.append(new_obs_car_x_vec)
      front_obs_car_matrix.append(new_obs_car_y_vec)

  rear_obs_car_matrix = []
  if is_rear_occupied:
      new_obs_car_x_vec = []
      new_obs_car_y_vec = []
      for i in range(len(obs_car['obs_x'])):
        tmp_x, tmp_y = rotate(obs_car['obs_x'][i],obs_car['obs_y'][i],rear_car_heading)
        tmp_y += rear_car_y_offset
        new_obs_car_x_vec.append(tmp_x)
        new_obs_car_y_vec.append(tmp_y)
      max_x = max(new_obs_car_x_vec)
      new_obs_car_x_vec = [x - max_x - obs_to_slot_line for x in new_obs_car_x_vec]
      obs_x_vec+= new_obs_car_x_vec
      obs_y_vec+= new_obs_car_y_vec
      rear_obs_car_matrix.append(new_obs_car_x_vec)
      rear_obs_car_matrix.append(new_obs_car_y_vec)

  half_slot_width = 0.5 * slot_width
  target_corner_x_vec = [slot_length, 0.0, 0.0, slot_length]
  target_corner_y_vec = [half_slot_width, half_slot_width, -half_slot_width, -half_slot_width]

  front_corner_x_vec = [x + slot_length for x in target_corner_x_vec]
  rear_corner_x_vec = [x - slot_length for x in target_corner_x_vec]

  ## channel
  channel_x_vec = []
  channel_y_vec = []
  channel_matrix = []
  # # car in channel
  for i in range(len(x_offset_vec)):
    channel_car_x_vec = []
    channel_car_y_vec = []
    for j in range(len(obs_car['obs_x'])):
      tmp_x, tmp_y = rotate(obs_car['obs_x'][j],obs_car['obs_y'][j],-pi/2 + heading_offset_vec[i])
      channel_car_x_vec.append(tmp_x)
      channel_car_y_vec.append(tmp_y)

    min_x = min(channel_car_x_vec)
    min_y = min(channel_car_y_vec)
    channel_car_x_vec = [x - min_x + x_offset_vec[i] for x in channel_car_x_vec]
    channel_car_y_vec = [y - min_y + y_offset_vec[i] + half_slot_width + channel_width for y in channel_car_y_vec]

    channel_x_vec += channel_car_x_vec
    channel_y_vec += channel_car_y_vec
    channel_matrix.append(channel_car_x_vec)
    channel_matrix.append(channel_car_y_vec)

  # curb
  with open(curb_file_name, 'r') as file:
      obs_curb = json.load(file)

  curb_x_vec = [x - 4.0 for x in obs_curb["obs_x"]]
  curb_y_vec = [y - half_slot_width - curb_offset for y in obs_curb["obs_y"]]
  # to json

  obs_x_vec = obs_x_vec + curb_x_vec + channel_x_vec
  obs_y_vec = obs_y_vec + curb_y_vec + channel_y_vec
  data = {
    "obs_x": obs_x_vec,
    "obs_y": obs_y_vec,
    "target_corner_x": target_corner_x_vec,
    "target_corner_y": target_corner_y_vec,
    "channel_matrix" : channel_matrix,
    "front_obs_car_matrix": front_obs_car_matrix,
    "rear_obs_car_matrix": rear_obs_car_matrix,
  }

  output_file_name = "{}{}_{}{}".format("/asw/data/",
      "front_occupied" if is_front_occupied else "front_vacant",
      "rear_occupied" if is_rear_occupied else "rear_vacant",
      ".json"
  )
  print(output_file_name)

  with open(output_file_name, 'w') as file:
      json.dump(data, file, indent=4)


  # # ## for debug json data
  # fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=600, height=600, match_aspect = True, aspect_scale=1)

  # source = ColumnDataSource(data=dict(x=[], y=[]))
  # fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
  # line_source = ColumnDataSource(data=dict(x=[], y=[]))
  # fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
  # text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
  # fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

  # # Define the JavaScript callback code
  # callback_code = """
  #     var x = cb_obj.x;
  #     var y = cb_obj.y;

  #     source.data['x'].push(x);
  #     source.data['y'].push(y);

  #     if (source.data['x'].length > 2) {
  #         source.data['x'].shift();
  #         source.data['y'].shift();
  #         source.data['x'].shift();
  #         source.data['y'].shift();
  #     }
  #     source.change.emit();

  #     if (source.data['x'].length >= 2) {
  #         var x1 = source.data['x'][source.data['x'].length - 2];
  #         var y1 = source.data['y'][source.data['y'].length - 2];
  #         var x2 = x;
  #         var y2 = y;
  #         var x3 = (x1 + x2) / 2;
  #         var y3 = (y1 + y2) / 2;

  #         var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

  #         console.log("Distance between the last two points: " + distance);

  #         distance = distance.toFixed(4);
  #         text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
  #         text_source.change.emit();

  #         line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
  #         line_source.change.emit();
  #     }

  #     if (source.data['x'].length == 1) {
  #         text_source.data['x'].shift();
  #         text_source.data['y'].shift();
  #         text_source.data['text'].shift();
  #     }
  #     text_source.change.emit();
  # """

  # # Create a CustomJS callback with the defined code
  # callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

  # # Attach the callback to the Tap event on the plot
  # fig1.js_on_event(Tap, callback)

  # fig1.circle(obs_x_vec, obs_y_vec, size=3, color="green", alpha=0.5)
  # fig1.circle(curb_x_vec, curb_y_vec, size=3, color="green", alpha=0.5)
  # fig1.circle(channel_x_vec, channel_y_vec, size=3, color="green", alpha=0.5)

  # fig1.patches(xs=[data["target_corner_x"]], ys=[data["target_corner_y"]], fill_color=['blue'], line_color='black', alpha=0.3)
  # fig1.patches(xs=[data["front_corner_x"]], ys=[data["front_corner_y"]], fill_color=['grey'], line_color='black', alpha=0.2)
  # fig1.patches(xs=[data["rear_corner_x"]], ys=[data["rear_corner_y"]], fill_color=['grey'], line_color='black', alpha=0.2)

  # # toolbar
  # fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
  # # legend
  # # fig1.legend.click_policy = 'hide'

  # handle = show(fig1, notebook_handle=True)
  # push_notebook(handle=handle)
