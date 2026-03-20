import os
import sys
import glob
import re
from multiprocessing import Pool, cpu_count

from calculate_slot_error import ParkingSlotEvaluator

def find_first_level_no_camera_bags(root_dir, start_hour=None, end_hour=None):
    bag_files = []
    for entry in os.scandir(root_dir):
        if entry.is_dir():
            sub_dir = entry.path
            # 只有给定了小时区间才尝试正则匹配
            if start_hour is not None and end_hour is not None:
                match = re.search(r'\d{8}-(\d{2})-\d{2}-\d{2}', entry.name)
                if not match:
                    continue  # 不是时间格式，直接跳过
                hour = int(match.group(1))
                if not (start_hour <= hour <= end_hour):
                    continue  # 不在范围内，跳过
            for file in os.listdir(sub_dir):
                if file.endswith('no_camera.bag'):
                    bag_files.append(os.path.join(sub_dir, file))
    return sorted(bag_files)

def eval_bag(bag_file):
    try:
        print(f"Processing {bag_file}")
        evaluator = ParkingSlotEvaluator()
        evaluator.record_bags(dt=0.2, bag_path=bag_file)
        print(f"Done {bag_file}")
    except Exception as e:
        print(f"Error processing {bag_file}: {e}")

def main():
    if len(sys.argv) not in [2, 4]:
        print("Usage: python multi_proc_eval_bags.py <folder_path> [<start_hour> <end_hour>]")
        sys.exit(1)

    root_dir = sys.argv[1]
    if not os.path.isdir(root_dir):
        print(f"{root_dir} is not a valid directory.")
        sys.exit(1)

    if len(sys.argv) == 4:
        start_hour = int(sys.argv[2])
        end_hour = int(sys.argv[3])
        bag_files = find_first_level_no_camera_bags(root_dir, start_hour, end_hour)
    else:
        bag_files = find_first_level_no_camera_bags(root_dir)

    print(f"Found {len(bag_files)} bag files.")
    for f in bag_files:
        print(f)

    # 多进程处理
    pool_size = min(cpu_count(), 3)  # 防止过多进程，实际可以调整
    with Pool(pool_size) as pool:
        pool.map(eval_bag, bag_files)

if __name__ == "__main__":
    main()