#!/usr/bin/env python3
"""
Batch test different parameter combinations for VL Cost iteration.
"""
import subprocess
import json

# Test configurations
configs = [
    # Iteration 10-15: Adjust fork_match bonus
    {"name": "iter10_fork_bonus_0.15", "line": 166, "old": "fork_match + 0.2 * (len(splits) - 1)", "new": "fork_match + 0.15 * (len(splits) - 1)"},
    {"name": "iter11_fork_bonus_0.25", "line": 166, "old": "fork_match + 0.2 * (len(splits) - 1)", "new": "fork_match + 0.25 * (len(splits) - 1)"},
    {"name": "iter12_fork_bonus_0.1", "line": 166, "old": "fork_match + 0.2 * (len(splits) - 1)", "new": "fork_match + 0.1 * (len(splits) - 1)"},
    {"name": "iter13_fork_bonus_0.3", "line": 166, "old": "fork_match + 0.2 * (len(splits) - 1)", "new": "fork_match + 0.3 * (len(splits) - 1)"},

    # Iteration 14-18: Adjust lane_num confidence discount
    {"name": "iter14_ln_conf_0.6", "line": 252, "old": "elif lnd == 1: weight *= 0.7", "new": "elif lnd == 1: weight *= 0.6"},
    {"name": "iter15_ln_conf_0.8", "line": 252, "old": "elif lnd == 1: weight *= 0.7", "new": "elif lnd == 1: weight *= 0.8"},
    {"name": "iter16_ln_conf_0.5", "line": 252, "old": "elif lnd == 1: weight *= 0.7", "new": "elif lnd == 1: weight *= 0.5"},
    {"name": "iter17_ln_conf2_0.2", "line": 251, "old": "if lnd >= 2: weight *= 0.3", "new": "if lnd >= 2: weight *= 0.2"},
    {"name": "iter18_ln_conf2_0.4", "line": 251, "old": "if lnd >= 2: weight *= 0.3", "new": "if lnd >= 2: weight *= 0.4"},

    # Iteration 19-20: Adjust sample_step
    {"name": "iter19_step_2.5", "line": 48, "old": "dist += sample_step", "new": "dist += 2.5"},
    {"name": "iter20_step_3.5", "line": 48, "old": "dist += sample_step", "new": "dist += 3.5"},
]

print("VL Cost Batch Testing - Iterations 10-20")
print("=" * 60)

baseline_result = None

for i, config in enumerate(configs, start=10):
    print(f"\n[Iteration {i}] Testing: {config['name']}")

    # Restore baseline
    subprocess.run(["cp", "fast_test_backup.py", "fast_test.py"], check=True)

    # Apply modification
    with open("fast_test.py", "r") as f:
        lines = f.readlines()

    # Simple string replacement (not line-based for simplicity)
    content = "".join(lines)
    content = content.replace(config["old"], config["new"])

    with open("fast_test.py", "w") as f:
        f.write(content)

    # Run test
    result = subprocess.run(["python3", "fast_test.py", "--all"],
                          capture_output=True, text=True)

    # Parse results
    lines = result.stdout.strip().split("\n")
    bug_results = {}
    for line in lines:
        if line.startswith("bug") and ":" in line:
            parts = line.split(":")
            bug_name = parts[0].strip()
            rate_str = parts[1].strip().split("%")[0]
            try:
                rate = float(rate_str)
                bug_results[bug_name] = rate
            except:
                pass
        elif line.startswith("Average:"):
            avg_str = line.split(":")[1].strip().split("%")[0]
            try:
                bug_results["average"] = float(avg_str)
            except:
                pass

    # Store baseline
    if i == 10 and baseline_result is None:
        # Get baseline first
        subprocess.run(["cp", "fast_test_backup.py", "fast_test.py"], check=True)
        baseline = subprocess.run(["python3", "fast_test.py", "--all"],
                                capture_output=True, text=True)
        baseline_lines = baseline.stdout.strip().split("\n")
        baseline_result = {}
        for line in baseline_lines:
            if line.startswith("bug") and ":" in line:
                parts = line.split(":")
                bug_name = parts[0].strip()
                rate_str = parts[1].strip().split("%")[0]
                try:
                    baseline_result[bug_name] = float(rate_str)
                except:
                    pass
            elif line.startswith("Average:"):
                avg_str = line.split(":")[1].strip().split("%")[0]
                try:
                    baseline_result["average"] = float(avg_str)
                except:
                    pass

    # Compare with baseline
    if baseline_result:
        print(f"  bug1: {bug_results.get('bug1', 0):.1f}% (baseline: {baseline_result.get('bug1', 0):.1f}%)")
        print(f"  bug2: {bug_results.get('bug2', 0):.1f}% (baseline: {baseline_result.get('bug2', 0):.1f}%)")
        print(f"  bug8: {bug_results.get('bug8', 0):.1f}% (baseline: {baseline_result.get('bug8', 0):.1f}%)")
        print(f"  bug9: {bug_results.get('bug9', 0):.1f}% (baseline: {baseline_result.get('bug9', 0):.1f}%)")
        print(f"  bug10: {bug_results.get('bug10', 0):.1f}% (baseline: {baseline_result.get('bug10', 0):.1f}%)")
        print(f"  Average: {bug_results.get('average', 0):.1f}% (baseline: {baseline_result.get('average', 0):.1f}%)")

        # Check if improved
        if bug_results.get('average', 0) > baseline_result.get('average', 0):
            print(f"  ✓ IMPROVED by {bug_results.get('average', 0) - baseline_result.get('average', 0):.1f}%")
        elif bug_results.get('average', 0) < baseline_result.get('average', 0):
            print(f"  ✗ REGRESSED by {baseline_result.get('average', 0) - bug_results.get('average', 0):.1f}%")
        else:
            print(f"  ○ NO CHANGE")

# Restore baseline
subprocess.run(["cp", "fast_test_backup.py", "fast_test.py"], check=True)

print("\n" + "=" * 60)
print("Batch testing complete. Baseline restored.")
