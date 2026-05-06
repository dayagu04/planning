#!/usr/bin/env python3
"""
Test all bugs and compare with baseline.
"""
import json
import sys
import os
import subprocess


def get_bag_start_time(bug_name):
    """Get bag start time in microseconds"""
    bug_dir = f'/docker_share/data/vl_cost_cases/{bug_name}'
    bags = [f for f in os.listdir(bug_dir) if f.endswith('.bag') and 'close-loop' not in f]
    if not bags:
        return None

    bag_path = os.path.join(bug_dir, bags[0])
    result = subprocess.run(
        ['rosbag', 'info', bag_path, '-y', '-k', 'start'],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        start_sec = float(result.stdout.strip())
        return int(start_sec * 1e6)
    return None


def load_test_data(bug_name):
    """Load frames and checker from test_data directory"""
    test_dir = os.path.join(os.path.dirname(__file__), 'test_data', bug_name)

    frames_file = os.path.join(test_dir, 'frames.json')
    if not os.path.exists(frames_file):
        return None, None

    with open(frames_file) as f:
        frames = json.load(f)

    checker_file = os.path.join(test_dir, 'checker.json')
    if not os.path.exists(checker_file):
        return frames, None

    with open(checker_file) as f:
        checker_data = json.load(f)

    for check in checker_data.get('checks', []):
        if check.get('type') == 'frame_range':
            frame_start, frame_end = check['frames']
            conditions = check.get('conditions', [])

            expected = {}
            for cond in conditions:
                order = cond.get('order')
                is_on_route = cond.get('expect', {}).get('is_on_route')
                if isinstance(order, str):
                    # Handle special cases: '!2', '>=2', etc.
                    expected[order] = is_on_route
                else:
                    expected[int(order)] = is_on_route

            checker = {
                'frame_start': frame_start,
                'frame_end': frame_end,
                'expected': expected,
                'description': checker_data.get('description', '')
            }
            return frames, checker

    return frames, None


def test_bug(bug_name):
    """Test one bug and return success rate"""
    frames, checker = load_test_data(bug_name)

    if frames is None or checker is None:
        return None

    bag_start_time = get_bag_start_time(bug_name)
    if not bag_start_time:
        return None

    checker_start_time = bag_start_time + int(checker['frame_start'] * 0.1 * 1e6)
    checker_end_time = bag_start_time + int(checker['frame_end'] * 0.1 * 1e6)

    frames_in_range = [f for f in frames
                       if checker_start_time <= f['timestamp'] <= checker_end_time]

    if not frames_in_range:
        return None

    total_checks = 0
    total_correct = 0

    for frame in frames_in_range:
        for vl in frame['vls']:
            order = vl['vl_order']
            actual = vl['is_on_route']

            # Get expected value
            expected = None
            if order in checker['expected']:
                expected = checker['expected'][order]
            elif '!2' in checker['expected'] and order != 2:
                expected = checker['expected']['!2']
            elif '>=2' in checker['expected'] and order >= 2:
                expected = checker['expected']['>=2']

            if expected is None:
                continue

            total_checks += 1
            if actual == expected:
                total_correct += 1

    if total_checks > 0:
        return total_correct / total_checks * 100
    return None


def main():
    test_dir = os.path.join(os.path.dirname(__file__), 'test_data')
    bugs = sorted([d for d in os.listdir(test_dir) if d.startswith('bug')])

    print("=== Testing All Bugs ===\n")

    results = []
    for bug in bugs:
        rate = test_bug(bug)
        if rate is not None:
            results.append((bug, rate))
            print(f"{bug}: {rate:.1f}%")
        else:
            print(f"{bug}: no data or checker")

    if results:
        avg_rate = sum(r[1] for r in results) / len(results)
        print(f"\n{'='*50}")
        print(f"Average success rate: {avg_rate:.1f}%")
        print(f"{'='*50}")


if __name__ == '__main__':
    main()
