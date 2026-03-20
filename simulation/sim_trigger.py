# -*- coding: utf-8 -*-

#运行前置
#pip3 install requests polling

import sys
import subprocess
import argparse
import os
import requests
from polling import poll
import time
import json
from datetime import datetime

def check_status():
    get_url = f"http://172.30.170.35:9003/manager/simulation-task-api/{current_time}/"
    response = requests.get(url=get_url)
    res = response.json()
    print('res:' + res['status'])
    if res['status'] != 'pending':
        return res

if __name__ == '__main__':
    #整合发送内容
    in_dir = sys.argv[1]
    tools_branch = sys.argv[2]
    current_time =  datetime.fromtimestamp(int(os.getenv('START_TIME')) / 1000).strftime("%Y%m%d_%H%M%S")
    print("start_time:" + current_time + "=========================" + os.getenv('START_TIME'))
    docker_image = "artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/planning_simulation:" + os.getenv('COMMIT_ID')
    url = "http://172.30.170.35:9003/manager/simulation-task-api/"
    headers = {"Content-Type": "application/json"}
    data = {
        "task_id": current_time,
        "version_1": os.getenv('GIT_BRANCH'),
        "commit_id_1": os.getenv('COMMIT_ID'),
        "current_time": current_time,
        "in_dir": in_dir,
        "docker_image": docker_image,
        "common_tools_branch": tools_branch
    }
    print(data)
    response = requests.post(url, headers=headers, json=data)
    if response.json()['status'] == 'failed':
        print('ci Simulate failed, please check sim server status!')
        exit()
    #轮询发送数据
    print("The simulation program has started running.")
    result = poll(
        check_status,
        timeout=3 * 60 * 60,
        step=60,
        ignore_exceptions=(requests.exceptions.ConnectionError,)
    )
    # #处理收到的数据
    if result['in_dir'] != '':
        dir_path = "target/sim_report"
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        file_path = os.path.join(dir_path, "sim_report.txt")
        with open(file_path, "w", encoding="utf-8") as file:
            file.write('sim_start_time: ' + current_time + '\n')
            file.write('sim_result: ' + result['status']+ '\n')
            file.write('sim_server: 172.30.170.34\n')
            file.write('sim_report_path: /data_cold/abu_zone/simulation_test_result/CI/'+ current_time)
        exit()
    print("task timeout!")
    