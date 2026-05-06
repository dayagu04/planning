#!/usr/bin/env python3
"""
Fast iteration test for CalculateMatchScore.
Contains complete Python implementation matching C++ logic.
Modify calculate_match_score() and run to see is_on_route success rate instantly.

Usage:
  python3 fast_test.py bug7           # test one bug with Python impl
  python3 fast_test.py --all          # test all bugs
  python3 fast_test.py bug7 --cpp     # verify against C++ exported scores
"""
import json, sys, os, subprocess, math


# ============================================================
# Python implementation of CalculateMatchScore
# ============================================================

def calculate_match_score(lane_nums, topo_nodes, exit_orders,
                                     sample_step=3.0, max_distance=120.0):
    """压缩优化版本"""
    if not topo_nodes:
        return 0.0

    # 1. 提取 splits
    splits = []
    for i in range(len(topo_nodes) - 1):
        if topo_nodes[i]["lane_num"] - topo_nodes[i+1]["lane_num"] >= 1 and i < len(exit_orders):
            splits.append({
                "dist": topo_nodes[i+1]["begin_dist"],
                "exit_ords": exit_orders[i],
                "pre_lane_num": topo_nodes[i]["lane_num"]
            })

    # 2. 构建 samples
    samples = []
    dist = 0.0
    while dist < max_distance:
        per_left, per_right = -1, -1
        for ln in lane_nums:
            if ln["end"] <= ln["begin"] or ln["left"] > 20 or ln["right"] > 20:
                continue
            if dist >= ln["begin"] and dist < ln["end"]:
                per_left, per_right = ln["left"], ln["right"]
                break
        if per_left >= 0 and per_right >= 0:
            samples.append({"dist": dist, "left": per_left, "right": per_right,
                           "total": per_left + per_right + 1,
                           "order_from_right": per_right + 1})
        dist += sample_step

    # 3. Fork matching - 单次遍历完成 drop 检测和 sequence matching
    accumulated_fork_score = 0.0
    accumulated_fork_weight = 0.0
    has_any_fork_signal = False

    for split_idx, split in enumerate(splits):
        if not split["exit_ords"]:
            continue

        # 动态搜索范围
        search_range = 30.0
        if split_idx + 1 < len(splits):
            d2n = splits[split_idx+1]["dist"] - split["dist"]
            if d2n < 60.0:
                search_range = max(15.0, d2n / 2.0)

        # 状态变量
        right_dropped, left_dropped = False, False
        pre_sample, post_sample = None, None

        # Sequence matching 统计
        split_samples = []
        strong, weak = 0, 0
        trend_match, trend_total = 0, 0

        # 单次遍历：同时检测 drop 和收集 sequence 统计
        for i in range(len(samples)):
            s = samples[i]

            # A. 收集 split 区域的 samples（用于 sequence matching）
            if split["dist"] - 15.0 <= s["dist"] <= split["dist"] + 30.0:
                split_samples.append(s)
                r_in = s["order_from_right"] in split["exit_ords"]
                l_in = (split["pre_lane_num"] - s["left"]) in split["exit_ords"]
                if r_in and l_in: strong += 1
                elif r_in or l_in: weak += 1
                if s["dist"] >= split["dist"]:
                    trend_total += 1
                    if abs(s["total"] - len(split["exit_ords"])) <= 1:
                        trend_match += 1

            # B. Drop 检测（只在相邻 sample 对之间，且在搜索范围内）
            if i + 1 < len(samples) and not (right_dropped or left_dropped):
                nxt = samples[i+1]
                if (split["dist"] - search_range <= s["dist"] <= split["dist"] + search_range and
                    split["dist"] - search_range <= nxt["dist"] <= split["dist"] + search_range):

                    # 合并 left/right drop 检测（消除对称重复）
                    def check_drop(field):
                        if s[field] > 0 and nxt[field] == 0:
                            # 检查持久性
                            return all(samples[j][field] == 0 for j in range(i+1, min(i+4, len(samples))))
                        return False

                    if check_drop("right"):
                        right_dropped, pre_sample, post_sample = True, s, nxt
                    elif check_drop("left"):
                        left_dropped, pre_sample, post_sample = True, s, nxt

        # Fallback: 如果没检测到 drop，找最近的 pre/post
        if not right_dropped and not left_dropped:
            tp, tpo = split["dist"] - 5.0, split["dist"] + 10.0
            mpd, mpod = float('inf'), float('inf')
            for s in samples:
                if abs(s["dist"] - tp) <= mpd:
                    mpd, pre_sample = abs(s["dist"] - tp), s
                if abs(s["dist"] - tpo) <= mpod:
                    mpod, post_sample = abs(s["dist"] - tpo), s

        if pre_sample is None or post_sample is None:
            continue

        # 判断 path 分支方向
        mid = (split["pre_lane_num"] + 1.0) / 2.0
        min_e, max_e = min(split["exit_ords"]), max(split["exit_ords"])
        path_is_right = (max_e <= mid)
        path_is_left = (min_e >= mid)
        path_spans = not path_is_right and not path_is_left

        # 计算 split match score
        current_split_match = 0.0
        current_has_signal = False
        is_hard = False

        if right_dropped or left_dropped:
            # 几何信号
            current_has_signal = True
            is_hard = True
            vl_left, vl_right = right_dropped, left_dropped

            if vl_left and path_is_left: current_split_match = 1.0
            elif vl_right and path_is_right: current_split_match = 1.0
            elif vl_left and path_is_right: current_split_match = 0.0
            elif vl_right and path_is_left: current_split_match = 0.0
            elif path_spans:
                r_in = pre_sample["order_from_right"] in split["exit_ords"]
                l_in = (split["pre_lane_num"] - pre_sample["left"]) in split["exit_ords"]
                current_split_match = 0.5 if (r_in or l_in) else 0.0
        else:
            # Sequence matching（使用已收集的统计）
            if split_samples:
                seq_score = (strong * 1.2 + weak * 0.4) / len(split_samples)
                if trend_total > 0:
                    seq_score = min(1.0, seq_score + (trend_match / trend_total) * 0.2)

                if seq_score >= 0.8: current_split_match = 0.8
                elif seq_score >= 0.6: current_split_match = 0.5
                elif seq_score >= 0.4: current_split_match = 0.3
                elif seq_score >= 0.2: current_split_match = 0.1
                else: current_split_match = 0.0
                current_has_signal = True

        # 累积
        if current_has_signal:
            has_any_fork_signal = True
            dw = max(0.3, 1.0 - (split["dist"] - 50.0) / (max_distance - 50.0)) if split["dist"] > 50.0 else 1.0
            tw = 2.0 if is_hard else 1.0
            fw = dw * tw
            accumulated_fork_score += current_split_match * fw
            accumulated_fork_weight += fw

    # Fork match 最终得分
    fork_match = (accumulated_fork_score / accumulated_fork_weight) if accumulated_fork_weight > 1e-6 else 0.0
    if fork_match > 0.0 and len(splits) >= 2:
        fork_match = min(1.0, fork_match + 0.3 * (len(splits) - 1))
    elif len(splits) >= 2:
        fork_match += 0.15 * (len(splits) - 1)

    # 4. 逐点打分（保持不变）
    total_score, total_weight = 0.0, 0.0

    for sample in samples:
        path_node, node_idx = None, -1
        for ni, n in enumerate(topo_nodes):
            if sample["dist"] >= n["begin_dist"] and sample["dist"] < n["end_dist"]:
                path_node, node_idx = n, ni
                break
        if path_node is None or path_node["lane_num"] <= 0:
            continue

        diff = abs(sample["total"] - path_node["lane_num"])
        lane_num_score = 1.0 if diff == 0 else (0.5 if diff == 1 else 0.1)

        order_score = 0.0
        has_order_signal = False
        if 0 <= node_idx < len(exit_orders) and exit_orders[node_idx]:
            eo = exit_orders[node_idx]
            has_order_signal = True
            r_in = sample["order_from_right"] in eo
            l_ord = path_node["lane_num"] - sample["left"]
            l_in = l_ord in eo

            lookahead_adj = 0.0
            if node_idx + 1 < len(exit_orders) and exit_orders[node_idx + 1]:
                dist_to_end = path_node["end_dist"] - sample["dist"]
                if 0 < dist_to_end < 35.0:
                    proximity = 1.0 - dist_to_end / 20.0
                    next_eo = exit_orders[node_idx + 1]
                    if sample["order_from_right"] in next_eo or l_ord in next_eo:
                        lookahead_adj = 0.2 * proximity

            if r_in and l_in:
                order_score = 1.0 if lane_num_score >= 0.5 else 0.5
            elif not r_in and not l_in:
                order_score = -0.1
            else:
                rs, ls = (1.0 if r_in else 0.0), (1.0 if l_in else 0.0)
                pt = sample["left"] + sample["right"] + 1
                if pt <= 3:
                    order_score = (rs + ls) / 2.0
                else:
                    wr, wl = 1.0 + sample["right"] * 0.3, 1.0 + sample["left"] * 0.3
                    order_score = (wr * rs + wl * ls) / (wr + wl)

            if diff >= 2: order_score *= 0.2
            elif diff >= 1: order_score *= 0.4

            order_score = min(1.0, order_score + lookahead_adj)

        if has_any_fork_signal:
            if has_order_signal:
                point_score = 0.5 * fork_match + 0.3 * lane_num_score + 0.2 * order_score
            else:
                point_score = 0.625 * fork_match + 0.375 * lane_num_score
        else:
            if has_order_signal:
                point_score = 0.4 * lane_num_score + 0.6 * order_score
            else:
                point_score = lane_num_score

        weight = max(0.3, 1.0 - (sample["dist"] - 20.0) / 80.0) if sample["dist"] > 20.0 else 1.0
        lnd = abs(sample["total"] - path_node["lane_num"])
        if lnd >= 2: weight *= 0.3
        elif lnd == 1: weight *= 0.7

        total_score += weight * point_score
        total_weight += weight

    return (total_score / total_weight) if total_weight > 1e-6 else 0.0




# ============================================================
# Test framework
# ============================================================
def get_bag_start_time(bug_name):
    bug_dir = f'/docker_share/data/vl_cost_cases/{bug_name}'
    bags = [f for f in os.listdir(bug_dir) if f.endswith('.bag') and 'close-loop' not in f]
    if not bags: return None
    result = subprocess.run(['rosbag', 'info', os.path.join(bug_dir, bags[0]), '-y', '-k', 'start'],
                           capture_output=True, text=True)
    return int(float(result.stdout.strip()) * 1e6) if result.returncode == 0 else None


def load_test_data(bug_name):
    test_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data', bug_name)
    frames_file = os.path.join(test_dir, 'frames.json')
    checker_file = os.path.join(test_dir, 'checker.json')
    if not os.path.exists(frames_file) or not os.path.exists(checker_file):
        return None, None

    with open(frames_file) as f: frames = json.load(f)
    with open(checker_file) as f: checker_data = json.load(f)

    # Parse all checks
    result = {'description': checker_data.get('description', ''), 'checks': []}
    for check in checker_data.get('checks', []):
        if check.get('type') != 'frame_range':
            continue
        fs, fe = check['frames']
        parsed = {'frame_start': fs, 'frame_end': fe}

        if 'conditions' in check:
            # is_on_route check
            expected = {}
            for cond in check.get('conditions', []):
                o = cond.get('order')
                v = cond.get('expect', {}).get('is_on_route')
                expected[o if isinstance(o, str) else int(o)] = v
            parsed['type'] = 'is_on_route'
            parsed['expected'] = expected
        elif 'condition' in check:
            # min_cost check
            parsed['type'] = 'min_cost'
            parsed['condition'] = check['condition']
            parsed['expect'] = check['expect']

        result['checks'].append(parsed)

    if not result['checks']:
        return frames, None

    # Backward compat: set top-level fields from first is_on_route check
    for c in result['checks']:
        if c['type'] == 'is_on_route':
            result['frame_start'] = c['frame_start']
            result['frame_end'] = c['frame_end']
            result['expected'] = c['expected']
            break

    return frames, result


def get_expected(order, expected, all_orders=None):
    if order in expected: return expected[order]
    for k, v in expected.items():
        if isinstance(k, str):
            if k == 'max' and all_orders and order == max(all_orders): return v
            if k == 'max-1' and all_orders and len(all_orders) >= 2 and order == sorted(all_orders)[-2]: return v
            if k == '!max' and all_orders and order != max(all_orders): return v
            if k.startswith('>=') and order >= int(k[2:]): return v
            if k.startswith('!') and k[1:].isdigit() and order != int(k[1:]): return v
    return None


def calculate_lane_match_confidence(lane_nums, best_path_nodes, best_match_score):
    """Calculate lane_match_confidence by sampling along best path"""
    total_match_error = 0.0
    valid_samples = 0

    for dist in range(0, 121, 5):  # Sample every 5m up to 120m
        # Find node at this distance
        path_node = None
        for node in best_path_nodes:
            if dist >= node["begin_dist"] and dist < node["end_dist"]:
                path_node = node
                break

        if not path_node or not path_node.get("recommended_orders") or path_node["lane_num"] == 0:
            continue

        # Get observed values from lane_nums
        per_left, per_right = -1, -1
        for ln in lane_nums:
            if ln["end"] <= ln["begin"] or ln["left"] > 20 or ln["right"] > 20:
                continue
            if dist >= ln["begin"] and dist < ln["end"]:
                per_left, per_right = ln["left"], ln["right"]
                break

        if per_left < 0 or per_right < 0:
            continue

        # Compute observed order (weighted fusion of left/right)
        per_order_from_right = per_right + 1
        if per_order_from_right > path_node["lane_num"]:
            per_order_from_right = path_node["lane_num"]

        per_order_from_left = path_node["lane_num"] - per_left
        if per_order_from_left < 1:
            per_order_from_left = 1

        epsilon = 1e-6
        weight_left = 1.0 / (per_left + 1.0 + epsilon)
        weight_right = 1.0 / (per_right + 1.0 + epsilon)
        total_weight = weight_left + weight_right

        if total_weight > epsilon:
            per_order = (weight_left * per_order_from_left + weight_right * per_order_from_right) / total_weight
        else:
            per_order = 0.5 * (per_order_from_left + per_order_from_right)

        # Find min distance to recommended orders
        min_dist = min(abs(per_order - rec_ord) for rec_ord in path_node["recommended_orders"])

        # Compute confidence weight based on lane_num difference
        per_total = per_left + per_right + 1
        lane_num_diff = abs(per_total - path_node["lane_num"])

        if lane_num_diff > 2:
            continue  # Skip samples with large lane_num mismatch

        lane_num_diff_weight = math.exp(-lane_num_diff / 2.0)

        # Accumulate error (normalized to 0-1)
        error = lane_num_diff_weight * min_dist / max(1, path_node["lane_num"])
        total_match_error += error
        valid_samples += 1

    if valid_samples > 0:
        avg_error = total_match_error / valid_samples
        confidence = 1.0 - min(1.0, avg_error)
        # Weight by topological match score
        confidence *= max(0.3, best_match_score)
        return confidence
    else:
        return 0.0


def calculate_virtual_lane_route_cost(vl, topo_paths, num_groups=1, ego_on_route=True):
    """
    Python implementation of CalculateVirtualLaneRouteCost.
    """
    lane_nums = vl['lane_nums']

    # C++ check: lane_nums.size() < 100
    if len(lane_nums) >= 100:
        return {'is_on_route': False, 'total_cost': 1000.0, 'distance_penalty': 0.0,
                'lane_match_confidence': 0.0, 'on_link_route_score': 0.0, 'path_scores': []}

    # Check valid lane_nums
    has_valid = any(ln["end"] > ln["begin"] and ln["left"] <= 20 and ln["right"] <= 20 for ln in lane_nums)
    if not has_valid:
        return {'is_on_route': False, 'total_cost': 1000.0, 'distance_penalty': 0.0,
                'lane_match_confidence': 0.0, 'on_link_route_score': 0.0, 'path_scores': []}

    # C++ check: ego_on_route == false → penalize everything
    if not ego_on_route:
        total_cost = 0.5 * 1.0 + 0.1 * (1.0 - min(1.0, num_groups / 8.0)) + 0.4 * 1.0
        return {'is_on_route': False, 'total_cost': total_cost, 'distance_penalty': 1.0,
                'lane_match_confidence': 0.0, 'on_link_route_score': 0.0, 'path_scores': []}

    # Calculate match score for each path
    path_scores = []
    for path in topo_paths:
        score = calculate_match_score(lane_nums, path["nodes"], path["exit_orders"], max_distance=120.0)
        path_scores.append(score)

    best_match_score = max(path_scores) if path_scores else 0.0
    best_idx = -1
    if path_scores and best_match_score > 0.0:
        for pi, score in enumerate(path_scores):
            if score <= 0.0:
                continue
            if score > (path_scores[best_idx] if best_idx >= 0 else -1.0):
                best_idx = pi
        best_match_score = path_scores[best_idx] if best_idx >= 0 else 0.0

    # Cost components
    distance_penalty = 0.5
    lane_match_confidence = 0.0
    on_link_route_score = 0.0
    topo_trace_score = min(1.0, num_groups / 8.0)
    is_on_route = False

    if best_idx >= 0:
        best_path = topo_paths[best_idx]
        on_link_route_score = best_match_score

        path_has_on_route = all(len(n.get("recommended_orders", [])) > 0 for n in best_path["nodes"])
        is_on_route = path_has_on_route

        if not path_has_on_route:
            distance_penalty = 1.0
        else:
            distance_penalty = 0.0
            lane_match_confidence = calculate_lane_match_confidence(
                lane_nums, best_path["nodes"], best_match_score
            )

    # CalculateTotalCost: same weights as C++
    total_cost = (0.5 * (1.0 - lane_match_confidence)
                + 0.1 * (1.0 - topo_trace_score)
                + 0.4 * distance_penalty)

    return {
        'is_on_route': is_on_route,
        'total_cost': total_cost,
        'distance_penalty': distance_penalty,
        'lane_match_confidence': lane_match_confidence,
        'on_link_route_score': on_link_route_score,
        'path_scores': path_scores
    }


def test_frame_python(frame):
    """Recalculate scores with Python implementation"""
    topo_paths = frame['vls'][0]['topo_paths']
    results = {}

    # C++ check: ego lane (rel=0) must have valid lane_nums
    ego_has_valid_lane_nums = False
    for vl in frame['vls']:
        if vl.get('vl_relative_id', 0) == 0:
            for ln in vl['lane_nums']:
                if ln["end"] > ln["begin"] and ln["left"] <= 20 and ln["right"] <= 20:
                    ego_has_valid_lane_nums = True
                    break
            break
    if not ego_has_valid_lane_nums:
        return None

    # C++ check: max_node_dist < 30.0 → skip frame
    max_node_dist = 0.0
    if topo_paths:
        for node in topo_paths[0]["nodes"]:
            if node.get("end_dist", 0) > max_node_dist:
                max_node_dist = node["end_dist"]
    if max_node_dist < 30.0:
        return None

    # C++ check: ego_on_route from topo_paths metadata
    ego_on_route = frame.get('ego_on_route', True)

    for vl in frame['vls']:
        cost_result = calculate_virtual_lane_route_cost(vl, topo_paths, ego_on_route=ego_on_route)

        results[vl['vl_order']] = {
            'is_on_route': cost_result['is_on_route'],
            'total_cost': cost_result['total_cost'],
            'distance_penalty': cost_result['distance_penalty'],
            'lane_match_confidence': cost_result['lane_match_confidence'],
            'cpp_is_on_route': vl['is_on_route'],
            'path_scores': cost_result['path_scores'],
            'cpp_scores': vl['path_scores']
        }

    # Plan A: cross-VL rec_order cost adjustment
    # For on_route VLs sharing the same best_path, use per_order vs recommended_orders
    # to create cost differentiation. This only affects total_cost, NOT is_on_route.
    on_route_path_indices = [pi for pi, p in enumerate(topo_paths)
                            if all(len(n.get("recommended_orders", [])) > 0 for n in p["nodes"])]

    if on_route_path_indices:
        on_route_path = topo_paths[on_route_path_indices[0]]
        on_route_vls = [(vl['vl_order'], vl['lane_nums']) for vl in frame['vls']
                        if vl['vl_order'] in results and results[vl['vl_order']]['is_on_route']]

        if len(on_route_vls) >= 2:
            # Compute rec_order distance for each on_route VL
            vl_rec_dists = []
            for vl_order, lane_nums_v in on_route_vls:
                valid_lns = [ln for ln in lane_nums_v if ln["end"] > ln["begin"] and ln["left"] <= 20 and ln["right"] <= 20]
                total_d, count = 0.0, 0
                for dist_s in range(0, 121, 5):
                    path_node = None
                    for node in on_route_path["nodes"]:
                        if dist_s >= node["begin_dist"] and dist_s < node["end_dist"]:
                            path_node = node
                            break
                    if not path_node or path_node["lane_num"] == 0:
                        continue
                    rec = path_node.get("recommended_orders", [])
                    if not rec:
                        continue
                    for ln in valid_lns:
                        if ln["end"] <= ln["begin"]:
                            continue
                        if dist_s >= ln["begin"] and dist_s < ln["end"]:
                            per_ofr = min(ln["right"] + 1, path_node["lane_num"])
                            per_ofl = max(1, path_node["lane_num"] - ln["left"])
                            eps = 1e-6
                            wl = 1.0 / (ln["left"] + 1.0 + eps)
                            wr = 1.0 / (ln["right"] + 1.0 + eps)
                            per_order = (wl * per_ofl + wr * per_ofr) / (wl + wr)
                            min_d = min(abs(per_order - r) for r in rec)
                            total_d += min_d
                            count += 1
                            break
                avg_d = total_d / count if count > 0 else 999.0
                vl_rec_dists.append((vl_order, avg_d))

            # Apply small cost adjustment based on rec_order distance
            # VL closer to recommended_orders gets lower cost
            if vl_rec_dists:
                min_d = min(d for _, d in vl_rec_dists)
                max_d = max(d for _, d in vl_rec_dists)
                if max_d - min_d > 0.4:
                    for vl_order, avg_d in vl_rec_dists:
                        normalized = (avg_d - min_d) / (max_d - min_d)
                        results[vl_order]['total_cost'] += 0.05 * normalized

    return results


def check_min_cost(results, condition, expect_str, frame_orders, frame_vls=None):
    """Check min_cost condition for one frame. Returns (pass, fail)"""
    if condition == 'min_cost_in_on_route_lanes':
        on_route = {o: r for o, r in results.items() if r['is_on_route']}
        if not on_route:
            return 0, 0  # skip
        min_order = min(on_route.keys(), key=lambda o: on_route[o]['total_cost'])
    elif condition == 'min_cost_in_relative_id_0':
        # Only consider VLs with relative_id == 0
        rel0_orders = set()
        if frame_vls:
            for vl in frame_vls:
                if vl.get('vl_relative_id', -999) == 0:
                    rel0_orders.add(vl['vl_order'])
        rel0 = {o: r for o, r in results.items() if o in rel0_orders}
        if not rel0:
            return 0, 0
        min_order = min(rel0.keys(), key=lambda o: rel0[o]['total_cost'])
    elif condition == 'min_cost_in_all_lanes':
        if not results:
            return 0, 0
        min_order = min(results.keys(), key=lambda o: results[o]['total_cost'])
    else:
        return 0, 0

    # Support fallback array
    if isinstance(expect_str, list):
        for es in expect_str:
            p, f = _check_single_min_cost(min_order, es, results)
            if p > 0:
                return p, f
        return 0, 1
    return _check_single_min_cost(min_order, expect_str, results)


def _check_single_min_cost(min_order, expect_str, results):
    if expect_str == 'order == max':
        expected_order = max(results.keys())
        return (1, 0) if min_order == expected_order else (0, 1)
    elif expect_str == 'order == max-1':
        sorted_orders = sorted(results.keys())
        if len(sorted_orders) >= 2:
            expected_order = sorted_orders[-2]
            return (1, 0) if min_order == expected_order else (0, 1)
        return 0, 1
    elif '>=' in expect_str:
        threshold = int(expect_str.split('>=')[1].strip())
        return (1, 0) if min_order >= threshold else (0, 1)
    elif '==' in expect_str:
        target_str = expect_str.split('==')[1].strip()
        expected_order = int(target_str)
        if expected_order not in results:
            return 0, 1
        return (1, 0) if min_order == expected_order else (0, 1)
    return 0, 0


def test_bug(bug_name, use_cpp=False):
    frames, checker = load_test_data(bug_name)
    if not frames or not checker: return None

    bag_start = get_bag_start_time(bug_name)
    if not bag_start: return None

    cs = bag_start + int(checker['frame_start'] * 0.1 * 1e6)
    ce = bag_start + int(checker['frame_end'] * 0.1 * 1e6)
    in_range = [f for f in frames if cs <= f['timestamp'] <= ce]
    if not in_range: return None

    total_checks, total_correct = 0, 0
    mc_total, mc_correct = 0, 0
    order_stats = {}

    # Find min_cost check
    mc_check = None
    for c in checker.get('checks', []):
        if c.get('type') == 'min_cost':
            mc_check = c
            break

    for frame in in_range:
        frame_orders = [vl['vl_order'] for vl in frame['vls']]
        if use_cpp:
            for vl in frame['vls']:
                order = vl['vl_order']
                actual = vl['is_on_route']
                exp = get_expected(order, checker['expected'], frame_orders)
                if exp is None: continue
                if order not in order_stats: order_stats[order] = {'c': 0, 'w': 0}
                total_checks += 1
                if actual == exp:
                    total_correct += 1; order_stats[order]['c'] += 1
                else:
                    order_stats[order]['w'] += 1
        else:
            results = test_frame_python(frame)
            if results is None:
                continue
            for order, res in results.items():
                exp = get_expected(order, checker['expected'], frame_orders)
                if exp is None: continue
                if order not in order_stats: order_stats[order] = {'c': 0, 'w': 0}
                total_checks += 1
                if res['is_on_route'] == exp:
                    total_correct += 1; order_stats[order]['c'] += 1
                else:
                    order_stats[order]['w'] += 1

            # min_cost check
            if mc_check and not use_cpp:
                p, f = check_min_cost(results, mc_check['condition'], mc_check['expect'], frame_orders, frame['vls'])
                mc_total += p + f
                mc_correct += p

    rate = total_correct / total_checks * 100 if total_checks > 0 else 0
    mc_rate = mc_correct / mc_total * 100 if mc_total > 0 else -1
    return {'rate': rate, 'correct': total_correct, 'total': total_checks,
            'orders': order_stats, 'mc_rate': mc_rate, 'mc_correct': mc_correct, 'mc_total': mc_total}


def main():
    use_cpp = '--cpp' in sys.argv
    test_all = '--all' in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    mode = "C++ scores" if use_cpp else "Python impl"

    if test_all:
        test_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
        bugs = sorted([d for d in os.listdir(test_dir) if d.startswith('bug')])
    else:
        bugs = [args[0]] if args else ['bug7']

    print(f"=== Testing ({mode}) ===\n")
    print(f"{'Bug':<8} {'is_on_route':>15} {'min_cost':>15}")
    print(f"{'-'*8} {'-'*15} {'-'*15}")
    all_results = {}
    for bug in bugs:
        r = test_bug(bug, use_cpp)
        if r:
            all_results[bug] = r
            on_route_str = f"{r['rate']:.1f}% ({r['correct']}/{r['total']})"
            mc_str = f"{r['mc_rate']:.1f}% ({r['mc_correct']}/{r['mc_total']})" if r['mc_rate'] >= 0 else "N/A"
            print(f"{bug:<8} {on_route_str:>15} {mc_str:>15}")
        else:
            print(f"{bug:<8} {'no data':>15} {'no data':>15}")

    if len(all_results) > 1:
        avg_on = sum(r['rate'] for r in all_results.values()) / len(all_results)
        mc_vals = [r['mc_rate'] for r in all_results.values() if r['mc_rate'] >= 0]
        avg_mc = sum(mc_vals) / len(mc_vals) if mc_vals else -1
        mc_avg_str = f"{avg_mc:.1f}%" if avg_mc >= 0 else "N/A"
        print(f"\n{'Avg':<8} {avg_on:>14.1f}% {mc_avg_str:>15}")


if __name__ == '__main__':
    main()
