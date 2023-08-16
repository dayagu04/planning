import subprocess
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, help='stop constainer')
    parser.add_argument('--suffix', type=str, help='stop constainer')
    parser.add_argument('--close_loop', type=bool, default=False, help='stop constainer')
    args = parser.parse_args()
    bag_dir = args.dir 
    suffix = args.suffix
    close_loop = args.close_loop
    print("close_loop:" ,close_loop)
    isfile = os.path.isfile(bag_dir)
    if isfile:
      print('not support file!!!')
      # if close_loop:
      #   subprocess.run("build/tools/planning_player/pp --play " + bag_dir + ' --close-loop', shell=True, universal_newlines=True)
      # else:
      #   subprocess.run("build/tools/planning_player/pp --play " + bag_dir, shell=True, universal_newlines=True)
    else:
      for file_name in os.listdir(bag_dir):
        if not file_name.endswith('.00000'):
          continue
        bag_path = os.path.join(bag_dir, file_name)
        print('run ' + bag_path)
        if close_loop:
          subprocess.run("build/tools/planning_player/pp --play " + bag_path + ' --out-bag=' + bag_path + suffix  + ' --close-loop', shell=True, universal_newlines=True)
        else:
          subprocess.run("build/tools/planning_player/pp --play " + bag_path + ' --out-bag=' + bag_path + suffix, shell=True, universal_newlines=True)
    