#!/usr/bin/env python3
"""
Collect frame-level test data from bugs.
Exports to test_data/<bug_name>/ directory.

Usage:
  python3 collect_frame_data.py bug7          # collect one bug
  python3 collect_frame_data.py --all         # collect all bugs
"""
import os
import sys
import glob
import json
import shutil
import subprocess

DATASET_DIR = "/docker_share/data/vl_cost_cases"
PP_BIN = "/root/code/planning/build/tools/planning_player/pp"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "test_data")


def collect_bug(bug_name):
    bug_dir = os.path.join(DATASET_DIR, bug_name)
    bags = [f for f in os.listdir(bug_dir) if f.endswith('.bag') and 'close-loop' not in f]
    if not bags:
        print(f"  No bag found")
        return False

    bag_path = os.path.join(bug_dir, bags[0])

    # Clean previous exports
    for f in glob.glob("/tmp/frame_data_*.json"):
        os.remove(f)

    # Run with export enabled
    env = os.environ.copy()
    env["EXPORT_FRAME_DATA"] = "1"
    subprocess.run(
        [PP_BIN, "--play", bag_path, "--close-loop"],
        cwd=bug_dir, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

    # Collect exported files
    frames = []
    i = 0
    while os.path.exists(f"/tmp/frame_data_{i}.json"):
        with open(f"/tmp/frame_data_{i}.json") as f:
            frames.append(json.load(f))
        i += 1

    if not frames:
        print(f"  No frames exported")
        return False

    # Save to test_data
    bug_output_dir = os.path.join(OUTPUT_DIR, bug_name)
    os.makedirs(bug_output_dir, exist_ok=True)

    with open(os.path.join(bug_output_dir, "frames.json"), 'w') as f:
        json.dump(frames, f)

    # Copy checker
    checker_src = os.path.join(bug_dir, "checker.json")
    if os.path.exists(checker_src):
        shutil.copy2(checker_src, os.path.join(bug_output_dir, "checker.json"))

    print(f"  {len(frames)} frames saved")
    return True


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 collect_frame_data.py <bug_name|--all>")
        sys.exit(1)

    if sys.argv[1] == '--all':
        bugs = sorted([d for d in os.listdir(DATASET_DIR) if d.startswith('bug')])
        for bug in bugs:
            checker_path = os.path.join(DATASET_DIR, bug, "checker.json")
            if not os.path.exists(checker_path):
                print(f"{bug}: no checker, skipping")
                continue
            print(f"{bug}:")
            collect_bug(bug)
    else:
        bug_name = sys.argv[1]
        print(f"{bug_name}:")
        collect_bug(bug_name)


if __name__ == '__main__':
    main()
