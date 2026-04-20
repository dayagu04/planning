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
bag_path = "/share/data/clren/code/planning8/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2025-06-21-10-18-12.bag_1750472282000_1750472297000_no_camera_9d5895e02c392aa16833026421de46234675a22c.bag.1775737338.open-loop.scc.plan"

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

# Initialize data sources for DP visualization
debug_path_source = ColumnDataSource(data={'x': [], 'y': [], 's': [], 'l': [], 't': [], 'total_cost': [], 'obstacle_cost': [], 'path_cost': [], 'path_l_cost': [], 'path_dl_cost': [], 'path_ddl_cost': [], 'stitching_cost': [], 'long_cost': [], 'pre_t_index': [], 'pre_s_index': [], 'pre_l_index': [], 't_index': [], 's_index': [], 'l_index': []})

last_frame_path_source = ColumnDataSource(data={'x': [], 'y': []})

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

def update_dp_visualization(bag_loader, bag_time, local_view_data, debug_path_source, last_frame_path_source):
  """Update DP nodes and last frame path visualization from protobuf data"""
  import lib.load_global_var as global_var

  g_is_display_enu = global_var.get_value('g_is_display_enu')
  coord_tf = coord_transformer()
  loc_msg = local_view_data['data_msg']['loc_msg']
  coord_tf.set_info(loc_msg.position.position_boot.x,
                    loc_msg.position.position_boot.y,
                    loc_msg.orientation.euler_boot.yaw)

  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']

  # Update debug path (optimal path) from debug_points
  try:
    debug_points = plan_debug_msg.spatio_temporal_union_plan.debug_points

    if len(debug_points) > 0:
      # Extract data from debug_points
      s_vals = [p.s for p in debug_points]
      l_vals = [p.l for p in debug_points]
      t_vals = [p.t for p in debug_points]
      total_costs = [p.total_cost for p in debug_points]
      obstacle_costs = [p.obstacle_cost for p in debug_points]
      path_costs = [p.path_cost for p in debug_points]
      path_l_costs = [p.path_l_cost for p in debug_points]
      path_dl_costs = [p.path_dl_cost for p in debug_points]
      path_ddl_costs = [p.path_ddl_cost for p in debug_points]
      stitching_costs = [p.stitching_cost for p in debug_points]
      long_costs = [p.long_cost for p in debug_points]
      pre_t_indices = [p.pre_t_index for p in debug_points]
      pre_s_indices = [p.pre_s_index for p in debug_points]
      pre_l_indices = [p.pre_l_index for p in debug_points]
      t_indices = [p.t_index for p in debug_points]
      s_indices = [p.s_index for p in debug_points]
      l_indices = [p.l_index for p in debug_points]

      # 模拟 KDPath::SLToXY: 用 ResetCurveCalculate 的 theta 计算方式（中间点用前后差分）
      debug_x = []
      debug_y = []
      import bisect, math
      ref_pts = plan_debug_msg.spatio_temporal_union_plan_input.ref_points_vec
      if len(ref_pts) > 1:
        ref_x = [pt.x for pt in ref_pts]
        ref_y = [pt.y for pt in ref_pts]
        n = len(ref_x)
        # 计算 cum_s（与 ResetSCalculte 一致）
        cum_s = [0.0]
        for i in range(1, n):
          cum_s.append(cum_s[-1] + math.hypot(ref_x[i] - ref_x[i-1], ref_y[i] - ref_y[i-1]))
        # 计算每个点的 theta（与 ResetCurveCalculate 一致）
        thetas = []
        for i in range(n):
          if i == 0:
            dx, dy = ref_x[1] - ref_x[0], ref_y[1] - ref_y[0]
          elif i == n - 1:
            dx, dy = ref_x[i] - ref_x[i-1], ref_y[i] - ref_y[i-1]
          else:
            dx, dy = ref_x[i+1] - ref_x[i-1], ref_y[i+1] - ref_y[i-1]
          thetas.append(math.atan2(dy, dx))
        # SLToXY: 插值得到 (x, y, theta)，然后沿法向量偏移 l
        for s, l in zip(s_vals, l_vals):
          idx = bisect.bisect_right(cum_s, s) - 1
          idx = max(0, min(idx, n - 2))
          s0, s1 = cum_s[idx], cum_s[idx + 1]
          seg_len = s1 - s0
          r = (s - s0) / seg_len if seg_len > 1e-6 else 0.0
          base_x = ref_x[idx] + r * (ref_x[idx+1] - ref_x[idx])
          base_y = ref_y[idx] + r * (ref_y[idx+1] - ref_y[idx])
          # slerp theta
          t0, t1 = thetas[idx], thetas[idx+1]
          dt = t1 - t0
          while dt > math.pi: dt -= 2*math.pi
          while dt < -math.pi: dt += 2*math.pi
          theta = t0 + r * dt
          normal_heading = theta + math.pi / 2
          debug_x.append(base_x + l * math.cos(normal_heading))
          debug_y.append(base_y + l * math.sin(normal_heading))

      # Transform to local coordinates if needed
      if not g_is_display_enu and len(debug_x) > 0:
        debug_x, debug_y = coord_tf.global_to_local(debug_x, debug_y)

      debug_path_source.data.update({
        'x': debug_x,
        'y': debug_y,
        's': s_vals,
        'l': l_vals,
        't': t_vals,
        'total_cost': total_costs,
        'obstacle_cost': obstacle_costs,
        'path_cost': path_costs,
        'path_l_cost': path_l_costs,
        'path_dl_cost': path_dl_costs,
        'path_ddl_cost': path_ddl_costs,
        'stitching_cost': stitching_costs,
        'long_cost': long_costs,
        'pre_t_index': pre_t_indices,
        'pre_s_index': pre_s_indices,
        'pre_l_index': pre_l_indices,
        't_index': t_indices,
        's_index': s_indices,
        'l_index': l_indices,
      })
  except Exception as e:
    print(f"Error updating debug path: {e}")
    import traceback
    traceback.print_exc()

  # Update last frame path
  try:
    # Find previous frame's plan_debug_msg
    current_time = plan_debug_msg.timestamp
    prev_plan_debug_msg = None

    for msg in reversed(bag_loader.plan_debug_msg['data']):
      if msg.timestamp < current_time:
        prev_plan_debug_msg = msg
        break

    if prev_plan_debug_msg is not None:
      prev_traj = prev_plan_debug_msg.spatio_temporal_union_plan.trajectory_points
      if len(prev_traj) > 0:
        last_x_global = [pt.x for pt in prev_traj]
        last_y_global = [pt.y for pt in prev_traj]

        if g_is_display_enu:
          last_frame_path_source.data.update({'x': last_x_global, 'y': last_y_global})
        else:
          last_x_local, last_y_local = coord_tf.global_to_local(last_x_global, last_y_global)
          last_frame_path_source.data.update({'x': last_x_local, 'y': last_y_local})
  except Exception as e:
    print(f"Error updating last frame path: {e}")

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

  # Update DP visualization from protobuf debug_points
  update_dp_visualization(bag_loader, bag_time, local_view_data, debug_path_source, last_frame_path_source)

  push_notebook()

# Add DP visualization to main figure
from bokeh.models import HoverTool

# Add optimal path (debug_path) visualization
debug_path_renderer = fig1.circle('y', 'x', source=debug_path_source,
                                   size=6, color='red', alpha=0.8,
                                   legend_label='DP optimal path')
hover_debug_path = HoverTool(renderers=[debug_path_renderer], tooltips=[
    ('t_index', '@t_index'),
    ('s_index', '@s_index'),
    ('l_index', '@l_index'),
    ('s', '@s{0.00}'),
    ('l', '@l{0.00}'),
    ('t', '@t{0.00}'),
    ('total_cost', '@total_cost{0.00}'),
    ('obstacle_cost', '@obstacle_cost{0.00}'),
    ('path_cost', '@path_cost{0.00}'),
    ('  path_l_cost', '@path_l_cost{0.00}'),
    ('  path_dl_cost', '@path_dl_cost{0.00}'),
    ('  path_ddl_cost', '@path_ddl_cost{0.00}'),
    ('  stitching_cost', '@stitching_cost{0.00}'),
    ('long_cost', '@long_cost{0.00}'),
    ('pre_t_index', '@pre_t_index'),
    ('pre_s_index', '@pre_s_index'),
    ('pre_l_index', '@pre_l_index'),
])
fig1.add_tools(hover_debug_path)

# Add last frame path
fig1.line('y', 'x', source=last_frame_path_source, line_width=3, color='blue',
          alpha=0.7, line_dash='dashed', legend_label='last frame path')

bkp.show(row(fig1, column(data_spatio_temporal_table_1, fig_cost_time, fig_dp_result)), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
