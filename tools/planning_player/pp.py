import subprocess
import argparse
import os
import multiprocessing

def pp_run(param):
  subprocess.run(param, shell=True, universal_newlines=True)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, help = 'bag path')
    parser.add_argument('--out_suffix', type = str)
    parser.add_argument('--close_loop', type=bool, default = False)
    parser.add_argument('--input_suffix', type=str, default = "")
    args = parser.parse_args()
    bag_dir = args.dir 
    out_suffix = args.out_suffix
    close_loop = args.close_loop
    input_suffix = args.input_suffix
    print("close_loop:" ,close_loop)
    isfile = os.path.isfile(bag_dir)
    if isfile:
      print('not support file!!!')
    else:
      
      commands = []
      pool = multiprocessing.Pool()
      for file_name in os.listdir(bag_dir):
        if input_suffix != "" and not file_name.endswith(input_suffix):
          continue
        if input_suffix == "" and (not file_name.endswith('.00000')) and (not file_name.endswith('.record')):
          continue
        
        bag_path = os.path.join(bag_dir, file_name)
        print('run ' + bag_path)
        command = ""
        if close_loop:
          command = "build/tools/planning_player/pp --play " + bag_path + ' --out-bag=' + bag_path + out_suffix  + ' --close-loop'
        else:
          command = "build/tools/planning_player/pp --play " + bag_path + ' --out-bag=' + bag_path + out_suffix
        commands.append(command)
      results = pool.map(pp_run, commands)
      pool.close()
      pool.join()