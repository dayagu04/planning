import os
import sys
import subprocess
import json

config_file_path = os.getenv('CONFIG_FILE_PATH')

with open(config_file_path, "r") as file:
    data = json.load(file)
bag_file = data["bag_file"]
task_id = data["task_id"]
scene_lib_id = data["scene_lib_id"]
case_id = data["case_id"]
checker_list = data["checker_list"]
config_version = data["config_version"]
common_tools_branch  = data["common_tools_branch"]

export_command = "bash -c 'source /opt/ros/melodic/setup.sh && "
# 获取bag文件，分为下载链接和路径两种
if "http" in bag_file:
    file_path = f"/root/planning/{task_id}_{scene_lib_id}_{case_id}.bag"
    command = 'curl ' + '"' + bag_file + '"' + ' -o ' + file_path
    try:
        subprocess.run(command, shell=True, text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Downloading bag error: {e.output}")
        sys.exit(1)
else:
    if os.path.exists(bag_file):
        file_path = bag_file
    else:
        print(f"The file '{bag_file}' does not exist.")
        sys.exit(1)
print("Get bag successfully !")

# 生成结果文件目录
out_root_path = "/out"
out_dir = f"{out_root_path}/{task_id}/{scene_lib_id}/{case_id}"
try:
    os.makedirs(out_dir, exist_ok=True)
except Exception as e:
    print(f"Creating out_dir error: {e}")
print("Creat out_dir successfully !")

# 运行PP
PP_bag = f"{out_dir}/{task_id}_{scene_lib_id}_{case_id}.bag.PP"
mileage_path = f"{out_dir}/case_result.json"
command = export_command + f"/root/planning/build/tools/planning_player/pp --play {file_path} --out-bag {PP_bag} --mileage-path {mileage_path} --close-loop'"
try:
    subprocess.run(command, shell=True, text=True, check=True)
except subprocess.CalledProcessError as e:
    print(f"Runing PP error: {e.output}")
    # sys.exit(1)
print("Run PP successfully !")

# 运行checker
command = f"cd /root/common_tools/ && git fetch && git checkout -b {task_id}_{scene_lib_id}_{case_id} origin/{common_tools_branch}"
try:
    subprocess.run(command, shell=True, text=True, check=True)
except subprocess.CalledProcessError as e:
    print(f"Pulling common_tools error: {e.output}")
    # sys.exit(1)
print("Pulling common_tools successfully !")

checker_path = "/root/common_tools/checker_scc/task"
json_path = f"{checker_path}/scc_checker_task.json"
# 修改JSON文件
with open(json_path, 'r', encoding='utf-8') as file:
    data = json.load(file)
data['checker_list'] = checker_list
data['plotter_list'] = []
data['bag_path_list'] = [out_dir]
data['output_path'] = out_dir

with open(json_path, 'w', encoding='utf-8') as file:
    json.dump(data, file, ensure_ascii=False, indent=2)
print("Update json successfully !")

command = export_command + f"cd {checker_path} && /root/miniconda3/bin/python scc_checker_task.py scc_checker_task.json simulation_mode'"
try:
    subprocess.run(command, shell=True, text=True, check=True)
except subprocess.CalledProcessError as e:
    print(f"Runing checker error: {e.output}")
    # sys.exit(1)
print("Run checker successfully !")

# 生成html
script_path = "/root/common_tools/jupyter/notebooks_scc/scripts/"
command_lat = export_command + f"cd {script_path} && /root/miniconda3/bin/python html_generator.py plot_lat_plan_html {PP_bag}'"
command_lon = export_command + f"cd {script_path} && /root/miniconda3/bin/python html_generator.py plot_lon_plan_html {PP_bag}'"
command_behavior = export_command + f"cd {script_path} && /root/miniconda3/bin/python html_generator.py plot_vo_lat_behavior_html {PP_bag}'"
command_local_view = export_command + f"cd {script_path} && /root/miniconda3/bin/python html_generator.py plot_local_view_html {PP_bag}'"
try:
    subprocess.run(command_lat, shell=True, text=True, check=True)
    subprocess.run(command_lon, shell=True, text=True, check=True)
    subprocess.run(command_behavior, shell=True, text=True, check=True)
    subprocess.run(command_local_view, shell=True, text=True, check=True)
except subprocess.CalledProcessError as e:
    print(f"Creating html error: {e.output}")
    sys.exit(1)
print("Creat html successfully !")