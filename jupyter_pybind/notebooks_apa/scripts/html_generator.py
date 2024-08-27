import sys
import os
import subprocess
import multiprocessing
from concurrent.futures import ThreadPoolExecutor

def run_command_by_path(file_path, vehicle_type):
    if command == "plot_ctrl_html":
        run_command = ["python", "plot_ctrl_html.py", file_path]
    elif command == "plot_lat_plan_html":
        run_command = ["python", "plot_lat_plan_html.py", file_path]
    elif command == "plot_lon_plan_html":
        run_command = ["python", "plot_lon_plan_html.py", file_path]
    elif command == "plot_apa_html":
        if vehicle_type:
            run_command = ["python", "plot_apa_html.py", file_path, vehicle_type]
        else:
            run_command = ["python", "plot_apa_html.py", file_path]
    else:
        print(f"未知的命令: {command}")
        return

    print(f"运行命令{run_command}处理文件{file_path}")
    subprocess.run(run_command)

def find_and_run_files(path, command, vehicle_type):
    if os.path.isfile(path):
        run_command_by_path(path, vehicle_type)
    else:
        with multiprocessing.Pool(processes=6) as pool:
            with ThreadPoolExecutor() as executor:
                for root, dirs, files in os.walk(path):
                    for file in files:
                        if ".0000" in file or ".record" in file or "_no_camera.bag" in file:
                            file_path = os.path.join(root, file)
                            executor.submit(run_command_by_path, file_path, vehicle_type)

def process_file(file_path, vehicle_type):
    run_command_by_path(file_path, vehicle_type)

def process_file(file_path):
    run_command_by_path(file_path)

# 获取命令行参数
if len(sys.argv) != 4 and len(sys.argv) != 3:
    print("请依次提供命令、路径、车辆类型(JAC_S811, CHERY_T26, CHERY_E0X)")
    sys.exit(1)

# 获取命令行传递的命令和路径参数
command = sys.argv[1]
path = sys.argv[2]
vehicle_type = sys.argv[3] if len(sys.argv) == 4 else None

# 找到文件并运行命令
find_and_run_files(path, command, vehicle_type)