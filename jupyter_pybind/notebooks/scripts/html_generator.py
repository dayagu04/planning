import sys
import os
import subprocess
import multiprocessing
from concurrent.futures import ThreadPoolExecutor

def run_command_by_path(file_path):
    if command == "plot_ctrl_html":
        run_command = ["python", "plot_ctrl_html.py", file_path]
    elif command == "plot_lat_plan_html":
        run_command = ["python", "plot_lat_plan_html.py", file_path]
    elif command == "plot_lon_plan_html":
        run_command = ["python", "plot_lon_plan_html.py", file_path]
    elif command == "plot_apa_html":
        run_command = ["python", "plot_apa_html.py", file_path]
    else:
        print(f"未知的命令: {command}")
        return

    print(f"运行命令{run_command}处理文件{file_path}")
    subprocess.run(run_command)

def find_and_run_files(path, command):
    if os.path.isfile(path):
        run_command_by_path(path)
    else:
        with multiprocessing.Pool(processes=6) as pool:
            with ThreadPoolExecutor() as executor:
                for root, dirs, files in os.walk(path):
                    for file in files:
                        if ".0000" in file or ".record" in file:
                            file_path = os.path.join(root, file)
                            executor.submit(run_command_by_path, file_path)

def process_file(file_path):
    run_command_by_path(file_path)

# 获取命令行参数
if len(sys.argv) != 3:
    print("请提供一个命令和一个路径参数")
    sys.exit(1)

# 获取命令行传递的命令和路径参数
command = sys.argv[1]
path = sys.argv[2]

# 找到文件并运行命令
find_and_run_files(path, command)