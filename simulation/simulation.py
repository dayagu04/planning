import os
import sys
import subprocess
import json
import time
from multiprocessing import Process
import boto3

begin_time = time.time()

config_file_path = os.getenv('CONFIG_FILE_PATH')
enable_static_fusion = os.getenv('ENABLE_STATIC_FUSION')

with open(config_file_path, "r") as file:
    data = json.load(file)
bag_file = data["bag_file"]
task_id = data["task_id"]
scene_lib_id = data["scene_lib_id"]
case_id = data["case_id"]
task_time = data["task_time"]
checker_list = data["checker_list"]
config_version = data["config_version"]
common_tools_branch  = data["common_tools_branch"]
file_name  = data["file_name"]
s3_bucket = data["s3_bucket"]
s3_url = data["s3_url"]
s3_access_key = data["s3_access_key"]
s3_secret_key = data["s3_secret_key"]

# 默认 CHERY_E0X
car_type = "CHERY_E0X"
if "CHERY_E0Y" in file_name:
    car_type = "CHERY_E0X"
elif "S811" in file_name:
    car_type = "JAC_S811"
elif "CHERY_TIGGO9" in file_name:
    car_type = "CHERY_T26"

# 定义S3 client
object_prefix = f"{task_time}/{task_id}/{scene_lib_id}/{case_id}"
client = boto3.client(
    's3',
    endpoint_url=s3_url,  # Ceph S3端点
    aws_access_key_id=s3_access_key,     # 访问密钥
    aws_secret_access_key=s3_secret_key,  # 密钥
    use_ssl=False
)

# 获取bag文件，分为下载链接和路径两种
start_time = time.time()
if "http" in bag_file:
    file_path = f"/root/planning/{task_id}_{scene_lib_id}_{case_id}.bag"
    command = 'curl ' + '"' + bag_file + '"' + ' -o ' + file_path
    try:
        result = subprocess.run(command, shell=True, text=True, check=True)
    except Exception as e:
        print(f"Downloading bag error: {e}")
        sys.exit(1)
    if (result.returncode != 0):
        print(f"Downloading bag error")
        sys.exit(1)
else:
    if os.path.exists(bag_file):
        file_path = bag_file
    else:
        print(f"The file '{bag_file}' does not exist.")
        sys.exit(1)
print("Get bag successfully !")
end_time = time.time()
print(f"Get bag 耗时：{end_time - start_time}秒")

# 生成结果文件目录
shm_path = "/dev/shm"
out_root_path = "/out"
out_dir = f"{out_root_path}/{task_id}/{scene_lib_id}/{case_id}"
try:
    os.makedirs(out_dir, exist_ok=True)
except Exception as e:
    print(f"Creating out_dir error: {e}")
print("Creat out_dir successfully !")

# 运行前置模块
if (enable_static_fusion == "1"):
    start_time = time.time()
    command = f"cd /root/planning/system_integration/components/static_fusion/core/install/staticFusion/bin/ && chmod 777 write_fusionroad_rosbag && ./write_fusionroad_rosbag {file_path}"
    try:
        result = subprocess.run(command, shell=True, text=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        print("Run static_fusion Error:", e.stderr)
    print("Run static_fusion successfully !")
    # 更新file_path
    parts = file_path.rsplit('.', 1)
    if len(parts) == 2:
        file_path = parts[0] + "_road_fusion.bag"
    else:
        print("Run static_fusion Error: file_path中没有找到'.'")
    end_time = time.time()
    print(f"Run static_fusion 耗时：{end_time - start_time}秒")

# 运行PP
start_time = time.time()
PP_bag = f"{shm_path}/{task_id}_{scene_lib_id}_{case_id}.bag.PP"
result_path = f"{out_dir}/case_result.json"
command = f"/root/planning/install/bin/pp --play {file_path} --out-bag {PP_bag} --mileage-path {result_path} --close-loop --interface-check --no-version-check"
try:
    result = subprocess.run(command, shell=True, text=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
except subprocess.CalledProcessError as e:
    print(f"Runing PP error: {e}")
    # print("PP Output:", e.stdout)
    print("PP Error:", e.stderr)
if (result.returncode != 0):
    print(f"Runing PP error !")
if not os.path.exists(PP_bag):
    sys.exit(1)
print("Run PP successfully !")
end_time = time.time()
print(f"Run PP 耗时：{end_time - start_time}秒")

# 运行checker
start_time = time.time()
command = f"cd /root/common_tools/ && git fetch && git checkout -b {task_id}_{scene_lib_id}_{case_id} origin/{common_tools_branch}"
try:
    result = subprocess.run(command, shell=True, text=True, check=True)
except Exception as e:
    print(f"Pulling common_tools error: {e}")
    sys.exit(1)
if (result.returncode != 0):
    print(f"Pulling common_tools error")
    sys.exit(1)
print("Pulling common_tools successfully !")
end_time = time.time()
print(f"Pulling common_tools 耗时：{end_time - start_time}秒")

with open(result_path, 'r', encoding='utf-8') as file:
    result_data = json.load(file)
scene_type = result_data["scene_type"]
if (scene_type == "apa"):
    json_path = f"/root/common_tools/checker_apa/task/checker_task.json"
else:
    json_path = f"/root/common_tools/checker_scc/task/scc_checker_task.json"
# 修改JSON文件
start_time = time.time()
with open(json_path, 'r', encoding='utf-8') as file:
    data = json.load(file)
data['checker_list'] = checker_list
data['plotter_list'] = []
data['bag_path_list'] = [shm_path]
data['output_path'] = out_dir

with open(json_path, 'w', encoding='utf-8') as file:
    json.dump(data, file, ensure_ascii=False, indent=2)
print("Update json successfully !")
end_time = time.time()
print(f"Update json 耗时：{end_time - start_time}秒")

start_time = time.time()
if (scene_type == "apa"):
    os.chdir('/root/common_tools/checker_apa/task')
    sys.path.append('/root/common_tools/checker_apa/task')
    import checker_task_for_simu
    try:
        t1 = Process(target=checker_task_for_simu.main, args=("checker_task.json",))
        t1.start()
    except Exception as e:
        print(f"Runing checker error: {e}")
        sys.exit(1)
else:
    os.chdir('/root/common_tools/checker_scc/task')
    sys.path.append('/root/common_tools/checker_scc/task')
    import scc_checker_task_for_simu
    try:
        t1 = Process(target=scc_checker_task_for_simu.main, args=("scc_checker_task.json",))
        t1.start()
    except Exception as e:
        print(f"Runing checker error: {e}")
        sys.exit(1)
end_time = time.time()
print(f"Run checker 耗时：{end_time - start_time}秒")

# 生成html
def upload_and_remove_file(suffix, result_data, key):
    object_name = f"{object_prefix}/{file_name}{suffix}"
    file_path = PP_bag + suffix
    try:
        client.upload_file(file_path, s3_bucket, object_name)
        result_data[key] = object_name
    except Exception as e:
        print(f"Error uploading {key} file: {e}")
    # finally:
    #     try:
    #         os.remove(file_path)
    #     except Exception as e:
    #         print(f"Error removing {key} file: {e}")

start_time = time.time()

if (scene_type == "apa"):
    script_path = "/root/common_tools/jupyter/notebooks_apa/scripts/"
    command_proto = f"cd {script_path} && /root/miniconda3/bin/python proto_gen.py"
    command = f"cd {script_path} && /root/miniconda3/bin/python plot_apa_html.py {PP_bag} {car_type}"
    try:
        result0 = subprocess.run(command_proto, shell=True, text=True, check=True)
        result1 = subprocess.run(command, shell=True, text=True, check=True)
        html_start_time = time.time()
        upload_and_remove_file('.apa.html', result_data, "apa_html_url")
        html_end_time = time.time()
        print(f"Upload html file 耗时：{html_end_time - html_start_time}秒")
    except Exception as e:
        print(f"Creating html error: {e}")
        sys.exit(1)
    if (result0.returncode != 0 or result1.returncode != 0):
        print(f"Creating html error")
        sys.exit(1)
else:
    script_path = "/root/common_tools/jupyter/notebooks_scc/scripts/"
    command_proto = f"cd {script_path} && /root/miniconda3/bin/python proto_gen.py"
    command = f"cd {script_path} && /root/miniconda3/bin/python plot_mutil_html.py {PP_bag}"
    try:
        result0 = subprocess.run(command_proto, shell=True, text=True, check=True)
        result1 = subprocess.run(command, shell=True, text=True, check=True)
        html_start_time = time.time()
        upload_and_remove_file('.lat_plan.html', result_data, "lat_html_url")
        upload_and_remove_file('.lon_plan.html', result_data, "lon_html_url")
        upload_and_remove_file('.enu_local_view.html', result_data, "local_view_html_url")
        upload_and_remove_file('.vo_lat_behavior.html', result_data, "behavior_html_url")
        html_end_time = time.time()
        print(f"Upload html file 耗时：{html_end_time - html_start_time}秒")
    except Exception as e:
        print(f"Creating html error: {e}")
        sys.exit(1)
    if (result0.returncode != 0 or result1.returncode != 0):
        print(f"Creating html error")
        sys.exit(1)
print("Creat html successfully !")
end_time = time.time()
print(f"Creat and upload html 耗时：{end_time - start_time}秒")

# 上传PP bag
start_time = time.time()
try:
    object_name = f"{object_prefix}/{file_name}.PP"
    client.upload_file(PP_bag, s3_bucket, object_name)
    result_data["PP_url"] = object_name
except Exception as e:
    print(f"Error uploading PP bag: {e}")
end_time = time.time()
print(f"Upload PP bag 耗时：{end_time - start_time}秒")

print("结果信息：")
for key, value in result_data.items():
    print(f'{key}: {value}')

with open(result_path, 'w', encoding='utf-8') as file:
    json.dump(result_data, file, ensure_ascii=False, indent=2)

t1.join()
end_time = time.time()
print(f"任务总耗时：{end_time - begin_time}秒")