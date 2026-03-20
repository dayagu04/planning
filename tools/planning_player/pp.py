'''
批量运行PP的脚本
--dir            必填参数; 存放bag的路径, 可包含子文件夹
--out_suffix     必填参数; PP生成的新包的后缀
--close_loop     可选参数; 是否为闭环, 不填表示开环
--input_suffix   可选参数; 输入包的后缀, 填入只运行包含特定后缀的文件。不填则会运行所有以'.00000'和'.record'结尾的文件
--ignore_suffix  可选参数; 运行除了以'.json'结尾之外的所有文件, 默认关闭
--out_dir        可选参数; 生成的新包的存放路径, 不填则表示与--dir的路径相同
示例：
python tools/planning_player/pp.py --dir "/xxxxx/" --out_suffix ".PP"
python tools/planning_player/pp.py --dir "/xxxxx/" --out_suffix ".PP" --out_dir "/xxxxx/" --close_loop 1 --ignore_suffix 1
'''

import subprocess
import argparse
import os
import multiprocessing
import sys

def pp_run(param):
  subprocess.run(param, shell=True, universal_newlines=True)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, help = 'bag path')
    parser.add_argument('--out_suffix', type = str)
    parser.add_argument('--close_loop', type=bool, default = False)
    parser.add_argument('--input_suffix', type=str, default = "")
    parser.add_argument('--ignore_suffix', type=bool, default = False)
    parser.add_argument('--out_dir', type=str, help = 'output path')

    args = parser.parse_args()
    bag_dir = args.dir 
    out_suffix = args.out_suffix
    close_loop = args.close_loop
    input_suffix = args.input_suffix
    ignore_suffix = args.ignore_suffix
    out_dir = args.out_dir
    if (bag_dir is None) or (out_suffix is None):
      print("--dir and --out_suffix is required")
      sys.exit(2)
    print("close_loop:" ,close_loop)
    print("ignore_suffix:" ,ignore_suffix)
    isfile = os.path.isfile(bag_dir)
    if isfile:
      print('not support file!!!')
    else:
      commands = []
      bag_num = 0
      for path, dir_lst, file_lst in os.walk(bag_dir):
        for file_name in file_lst:
          if file_name.endswith('.json'):
            continue
          if input_suffix != "" and not file_name.endswith(input_suffix):
            continue
          if input_suffix == "" and (not file_name.endswith('.00000')) and (not file_name.endswith('.record')) and (not ignore_suffix):
            continue
          
          bag_path = os.path.join(path, file_name)
          if out_dir != "":
            os.makedirs(out_dir, exist_ok=True)
            out_path = os.path.join(out_dir, file_name)
          else:
            out_path = bag_path
          print('run ' + bag_path)
          command = ""
          if close_loop:
            command = "build/tools/planning_player/pp --play " + bag_path + ' --out-bag=' + out_path + out_suffix  + ' --close-loop'
          else:
            command = "build/tools/planning_player/pp --play " + bag_path + ' --out-bag=' + out_path + out_suffix
          commands.append(command)
          bag_num+=1
          json_file = bag_path + ".json"
          if os.path.exists(json_file):
            command = "cp " + json_file + " " + out_path + out_suffix + ".json"
            commands.append(command)
      num_cores = multiprocessing.cpu_count()
      usage_rate = 0.8
      num_process = max(8, int(num_cores * usage_rate))
      num_process = min(bag_num, num_process)
      pool = multiprocessing.Pool(processes = num_process)
      results = pool.map(pp_run, commands)
      pool.close()
      pool.join()