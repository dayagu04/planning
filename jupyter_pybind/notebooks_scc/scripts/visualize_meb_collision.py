import sys
import os
import math
import numpy as np
from bokeh.plotting import figure, show, output_file, save
from bokeh.models import ColumnDataSource, Slider, CustomJS, Div, HoverTool
from bokeh.layouts import column, row

# Add the build directory to sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.abspath(os.path.join(script_dir, '../../../build/jupyter_pybind'))
if module_path not in sys.path:
    sys.path.append(module_path)

try:
    import meb_box_collision_py
except ImportError as e:
    print(f"Error importing meb_box_collision_py: {e}")
    sys.exit(1)

def get_box_corners(center_x, center_y, length, width, angle):
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    dx = length / 2.0
    dy = width / 2.0
    
    # Corners relative to center
    # FL, FR, RR, RL
    corners_x = [dx, dx, -dx, -dx]
    corners_y = [dy, -dy, -dy, dy]
    
    rotated_x = []
    rotated_y = []
    
    for x, y in zip(corners_x, corners_y):
        rx = center_x + x * cos_a - y * sin_a
        ry = center_y + x * sin_a + y * cos_a
        rotated_x.append(rx)
        rotated_y.append(ry)
        
    return rotated_x, rotated_y

def main():
    # 1. Setup Input
    input_info = meb_box_collision_py.CalculateETTCInputStr()
    input_info.ego_width = 1.9
    input_info.ego_length = 4.8
    input_info.ego_backshaft_2_fbumper = 3.8
    input_info.ego_v_x = 10.0
    input_info.ego_a_x = 0.0
    input_info.ego_radius = 100000.0 # Straight
    
    input_info.obj_x = 15.0
    input_info.obj_y = 0.0
    input_info.obj_v_x = 2.0
    input_info.obj_v_y = 0.0
    input_info.obj_a_x = 0.0
    input_info.obj_a_y = 0.0
    input_info.obj_width = 1.0
    input_info.obj_length = 1.0
    input_info.obj_heading_angle = 0.0
    
    t_start = 0.0
    t_end = 3.0
    dt = 0.1
    delay = 0.5
    dec = -3.0
    
    # 2. Run C++ Check and get Trace
    box_lib = meb_box_collision_py.BoxCollisonLib()
    is_collision = box_lib.GetCollisionResultBySimEgoDec(
        input_info, t_start, t_end, dt, delay, dec
    )
    print(f"C++ Collision Result: {is_collision}")
    
    # Retrieve debug trace from C++
    debug_trace = box_lib.debug_trace_
    print(f"Retrieved {len(debug_trace)} frames from C++ trace.")

    # 3. Process Trace Data for Visualization
    times = []
    ego_box_xs = []
    ego_box_ys = []
    obj_box_xs = []
    obj_box_ys = []
    
    # New lists for velocity/acceleration
    ego_vs = []
    ego_as = []
    obj_vxs = []
    obj_vys = []
    
    for frame in debug_trace:
        times.append(frame.time)
        ego_vs.append(frame.ego_v)
        ego_as.append(frame.ego_a)
        obj_vxs.append(frame.obj_vx)
        obj_vys.append(frame.obj_vy)
        
        # Calculate corners for visualization
        ex, ey = get_box_corners(frame.ego_x, frame.ego_y, input_info.ego_length, input_info.ego_width, frame.ego_heading)
        ego_box_xs.append(ex)
        ego_box_ys.append(ey)
        
        ox, oy = get_box_corners(frame.obj_x, frame.obj_y, input_info.obj_length, input_info.obj_width, frame.obj_heading)
        obj_box_xs.append(ox)
        obj_box_ys.append(oy)

    # 4. Bokeh Visualization
    output_file("meb_collision_vis.html", title="MEB Collision Visualization")
    
    # Sources
    source_data = {
        'time': times,
        'ego_box_x': ego_box_xs,
        'ego_box_y': ego_box_ys,
        'obj_box_x': obj_box_xs,
        'obj_box_y': obj_box_ys,
        'ego_v': ego_vs,
        'ego_a': ego_as,
        'obj_vx': obj_vxs,
        'obj_vy': obj_vys,
        'time_str': [f"Time: {t:.2f}s" for t in times]
    }
    all_source = ColumnDataSource(data=source_data)
    
    # Initial frame (index 0)
    if len(times) > 0:
        curr_source = ColumnDataSource(data={
            'ego_box_x': [ego_box_xs[0]],
            'ego_box_y': [ego_box_ys[0]],
            'obj_box_x': [obj_box_xs[0]],
            'obj_box_y': [obj_box_ys[0]],
            'time_str': [f"Time: {times[0]:.2f}s"]
        })
        init_text = f"<h3>Time: {times[0]:.2f}s | Ego V: {ego_vs[0]:.2f} m/s, A: {ego_as[0]:.2f} m/s² | Obj Vx: {obj_vxs[0]:.2f} m/s, Vy: {obj_vys[0]:.2f} m/s</h3>"
    else:
        curr_source = ColumnDataSource(data={'ego_box_x':[], 'ego_box_y':[], 'obj_box_x':[], 'obj_box_y':[], 'time_str':[]})
        init_text = "<h3>No Data</h3>"
    
    # Plot
    p = figure(title=f"MEB Collision Check (Result: {is_collision})", 
               x_axis_label='X (m)', y_axis_label='Y (m)',
               match_aspect=True, width=800, height=400)
    
    # Draw Ego
    p.patches('ego_box_x', 'ego_box_y', source=curr_source, color="blue", alpha=0.5, legend_label="Ego")
    # Draw Object
    p.patches('obj_box_x', 'obj_box_y', source=curr_source, color="red", alpha=0.5, legend_label="Object")
    
    # Text for time
    time_div = Div(text=init_text)
    
    # Slider
    slider = Slider(start=0, end=max(0, len(times)-1), value=0, step=1, title="Frame Index")
    
    # JS Callback
    callback = CustomJS(args=dict(source=all_source, curr=curr_source, slider=slider, time_div=time_div), code="""
        const data = source.data;
        const curr_data = curr.data;
        const idx = slider.value;
        
        if (idx < data['time'].length) {
            curr_data['ego_box_x'] = [data['ego_box_x'][idx]];
            curr_data['ego_box_y'] = [data['ego_box_y'][idx]];
            curr_data['obj_box_x'] = [data['obj_box_x'][idx]];
            curr_data['obj_box_y'] = [data['obj_box_y'][idx]];
            
            const t = data['time'][idx];
            const ev = data['ego_v'][idx];
            const ea = data['ego_a'][idx];
            const ovx = data['obj_vx'][idx];
            const ovy = data['obj_vy'][idx];
            
            time_div.text = `<h3>Time: ${t.toFixed(2)}s | Ego V: ${ev.toFixed(2)} m/s, A: ${ea.toFixed(2)} m/s² | Obj Vx: ${ovx.toFixed(2)} m/s, Vy: ${ovy.toFixed(2)} m/s</h3>`;
            
            curr.change.emit();
        }
    """)
    
    slider.js_on_change('value', callback)
    
    # Layout
    layout = column(
        Div(text=f"<h3>Collision Check Result: {is_collision}</h3><p>Drag slider to view simulation frames (Data from C++ Trace).</p>"),
        time_div,
        p, 
        slider
    )
    
    save(layout)
    print(f"Visualization saved to {os.path.abspath('meb_collision_vis.html')}")

if __name__ == "__main__":
    main()
