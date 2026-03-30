import sys
import os

# Add the build directory to sys.path so we can import the pybind module
# Use absolute path relative to this script
script_dir = os.path.dirname(os.path.abspath(__file__))
# Path to: planning/build/jupyter_pybind
# script_dir is .../planning/jupyter_pybind/notebooks_scc/scripts
# ../../../build/jupyter_pybind is .../planning/build/jupyter_pybind
module_path = os.path.abspath(os.path.join(script_dir, '../../../build/jupyter_pybind'))

if module_path not in sys.path:
    sys.path.append(module_path)

print(f"Added {module_path} to sys.path", flush=True)

try:
    import meb_box_collision_py
    print("Successfully imported meb_box_collision_py", flush=True)
except ImportError as e:
    print(f"Failed to import meb_box_collision_py: {e}", flush=True)
    sys.exit(1)

def test_collision():
    # 1. Initialize the library
    print("Initializing BoxCollisonLib...", flush=True)
    try:
        box_collision = meb_box_collision_py.BoxCollisonLib()
        print("BoxCollisonLib initialized.", flush=True)
    except Exception as e:
        print(f"Failed to initialize BoxCollisonLib: {e}", flush=True)
        return

    # 2. Setup input parameters
    print("Setting up input parameters...", flush=True)
    input_info = meb_box_collision_py.CalculateETTCInputStr()
    
    # Ego vehicle parameters (example values)
    input_info.ego_width = 1.9
    input_info.ego_length = 4.8
    input_info.ego_backshaft_2_fbumper = 3.8
    input_info.ego_v_x = 10.0  # 10 m/s
    input_info.ego_a_x = 0.0
    # Note: ego_radius cannot be 0.0 because the C++ code divides by it. 
    # Use a large value for straight line approximation.
    input_info.ego_radius = 100000.0
    
    # Object parameters (example values)
    input_info.obj_x = 50.0  # 50m ahead
    input_info.obj_y = 0.0   # In the same lane
    input_info.obj_v_x = 5.0 # Slower vehicle ahead
    input_info.obj_v_y = 0.0
    input_info.obj_a_x = 0.0
    input_info.obj_a_y = 0.0
    input_info.obj_width = 1.8
    input_info.obj_length = 4.5
    input_info.obj_heading_angle = 0.0
    
    # 3. Call the collision detection function
    # Parameters: info, time_t0, time_end, sim_step_time, time_before_aeb_dec, aeb_dec_acc
    t_start = 0.0
    t_end = 3.0
    time_step = 0.1
    time_delay = 0.5
    dec_request = -3.0 # Deceleration request
    
    print("Running collision simulation (Scenario 1: No Collision expected)...", flush=True)
    try:
        is_collision = box_collision.GetCollisionResultBySimEgoDec(
            input_info, 
            t_start, 
            t_end, 
            time_step, 
            time_delay, 
            dec_request
        )
        print(f"Collision Result: {is_collision}", flush=True)
    except Exception as e:
        print(f"Error during simulation: {e}", flush=True)

    # Test with a closer object to force a collision
    input_info.obj_x = 15.0
    # Use 0 deceleration to ensure collision
    dec_request_collision = 0.0 
    print("\nRunning collision simulation (Scenario 2: Collision expected with 0 dec)...", flush=True)
    try:
        is_collision_close = box_collision.GetCollisionResultBySimEgoDec(
            input_info, 
            t_start, 
            t_end, 
            time_step, 
            time_delay, 
            dec_request_collision
        )
        print(f"Collision Result (Closer): {is_collision_close}", flush=True)
    except Exception as e:
        print(f"Error during simulation: {e}", flush=True)

if __name__ == "__main__":
    test_collision()
