'''
根据数据平台提供的json, 批量下载bag
json_file_path: json路径
download_dir: 下载的bag存放路径
'''

import os
import json
import multiprocessing
import subprocess

def subprocess_run(param):
  subprocess.run(param, shell=True, universal_newlines=True)

if __name__ == "__main__":
    json_file_path = "/docker_share/planning/tools/dataset4.json"
    download_dir = "./downloads"
    processes_num = 10

    with open(json_file_path, 'r') as file:
        data = json.load(file)

    if not os.path.exists(download_dir):
        os.makedirs(download_dir)

    commands = []
    command = ""
    pool = multiprocessing.Pool(processes_num)

    for item in data:
        file_path = os.path.join(download_dir, item["key"])
        
        if os.path.exists(file_path):
            continue
            # file_path = os.path.join(output_dir, f"{file_name}_{os.path.getmtime(file_path)}")
    
        print('download ' + item["key"])
        if "tag" in item["key"]:
            command = 'curl ' + '"' + item["value"] + '"' + ' -o ' + file_path[:-4] + '.json'
        else:
            command = 'curl ' + '"' + item["value"] + '"' + ' -o ' + file_path
        print(command)
        commands.append(command)
    results = pool.map(subprocess_run, commands)
    pool.close()
    pool.join()
