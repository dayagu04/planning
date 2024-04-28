#!/usr/bin/env python3

"""
在下方函数中填入两个checker结果json文件的路径(json1_path, json1_path), 程序将对比这两个json文件并输出差异, 差异结果将存储在指定的output_path中

结果json层级结构:
checker_1:
    pass_rates:          # 两个json的通过率对比
        json1:
        json2:
        diff:
    faild_pass:          # json1中未通过, json2中通过的case
        case_1:
        case_2:
        ···
    pass_failed:         # json1中通过, json2中未通过的case
        case_1:
        case_2:
        ···
    diff_failed:         # json1和json2中均未通过且结果不一致的case
        case_1:
        case_2:
        ···
    same_failed:         # json1和json2中均未通过且结果一致的case
        case_1:
        case_2:
        ···
    same_pass:           # json1和json2中均通过的case
        case_1:
        case_2:
        ···

checker_2:
    pass_rates:
        json1:
        json2:
        diff:
    faild_pass:
        case_1:
        case_2:
        ···
    pass_failed:
        case_1:
        case_2:
        ···
    diff_failed:
        case_1:
        case_2:
        ···
    same_failed:
        case_1:
        case_2:
        ···
    same_pass:
        case_1:
        case_2:
        ···

···
"""

import json

# 使用提供的文件路径/dock
json1_path = "/data_cold/abu_zone/simulation_test_result/CI/20240428_120437/develop_eb2bb3c0a53c5cef8928fc9831d6cd5ec19891a2/simulation_1/checker_result.2024.0428.1206.44/scc_checker_task.2024.0428.1206.44.json"
json2_path = "/data_cold/abu_zone/simulation_test_result/CI/20240428_112836/develop_eb2bb3c0a53c5cef8928fc9831d6cd5ec19891a2/simulation_1/checker_result.2024.0428.1147.59/scc_checker_task.2024.0428.1147.59.json"
output_path = "/docker_share/temp/compare_output1.json"

def compare_jsons(json1_path, json2_path, output_path):
    # 读取两个JSON文件
    with open(json1_path, 'r') as f1, open(json2_path, 'r') as f2:
        json1 = json.load(f1)
        json2 = json.load(f2)

    # 从两个JSON文件中提取所有checker的名称
    all_checkers = set()
    for bag in json1 + json2:
        all_checkers.update(bag["checker_result"].keys())

    # 初始化输出结果字典
    output = {checker: {
        "pass_rates": {
            "json1": "",
            "json2": "",
            "diff": ""
        },
        "failed_pass": {},
        "pass_failed": {},
        "diff_failed": {},
        "same_failed": {},
        "same_pass": {}
    } for checker in all_checkers}

    # 初始化通过率
    checker_success_counts = {checker: {"json1": 0, "json2": 0} for checker in all_checkers}
    checker_bag_counts = {checker: {"json1": len(json1), "json2": len(json2)} for checker in all_checkers}

    # 遍历两个JSON文件中的bags进行比较和计数
    for bag1 in json1:
        for checker, checker1 in bag1["checker_result"].items():
            if checker1["success"]:
                checker_success_counts[checker]["json1"] += 1

    for bag2 in json2:
        for checker, checker2 in bag2["checker_result"].items():
            if checker2["success"]:
                checker_success_counts[checker]["json2"] += 1

    # 计算通过率
    for checker, counts in checker_success_counts.items():
        total_bags_json1 = checker_bag_counts[checker]["json1"]
        total_bags_json2 = checker_bag_counts[checker]["json2"]
        pass_rate_json1 = (counts["json1"] / total_bags_json1) * 100 if total_bags_json1 > 0 else 0
        pass_rate_json2 = (counts["json2"] / total_bags_json2) * 100 if total_bags_json2 > 0 else 0
        output[checker]["pass_rates"]["json1"] = str(pass_rate_json1) + " %"
        output[checker]["pass_rates"]["json2"] = str(pass_rate_json2) + " %"
        output[checker]["pass_rates"]["diff"] = str(pass_rate_json1 - pass_rate_json2) + " %"

    # 遍历两个JSON文件中的bags进行比较
    for bag1 in json1:
        for bag2 in json2:
            if bag1["bag_name"].split('/')[-1] == bag2["bag_name"].split('/')[-1]:
                common_checkers = set(bag1["checker_result"].keys()) & set(bag2["checker_result"].keys())
                for checker in common_checkers:
                    checker1 = bag1["checker_result"][checker]
                    checker2 = bag2["checker_result"][checker]

                    # 情况一：json1中的success为true，json2中的success为false
                    if checker1["success"] and not checker2["success"]:
                        output[checker]["pass_failed"][bag1["bag_name"].split('/')[-1]] = {
                            "json_1_bag_path": bag1["bag_name"],
                            "json_2_bag_path": bag2["bag_name"],
                            "json_1_result": checker1,
                            "json_2_result": checker2
                        }
                    # 情况二：json1中的success为false，json2中的success为true
                    elif not checker1["success"] and checker2["success"]:
                        output[checker]["failed_pass"][bag1["bag_name"].split('/')[-1]] = {
                            "json_1_bag_path": bag1["bag_name"],
                            "json_2_bag_path": bag2["bag_name"],
                            "json_1_result": checker1,
                            "json_2_result": checker2
                        }
                    # 情况三 & 情况四：两个json中的success都为false
                    elif not checker1["success"] and not checker2["success"]:
                        diff_fields = {key: (checker1[key], checker2[key]) for key in checker1 if key != "success" and checker1[key] != checker2[key]}
                        if diff_fields:
                            output[checker]["diff_failed"][bag1["bag_name"].split('/')[-1]] = {
                                "json_1_bag_path": bag1["bag_name"],
                                "json_2_bag_path": bag2["bag_name"],
                                "json_1_result": checker1,
                                "json_2_result": checker2,
                                "diff_fields": diff_fields
                            }
                        else:
                            output[checker]["same_failed"][bag1["bag_name"].split('/')[-1]] = {
                                "json_1_bag_path": bag1["bag_name"],
                                "json_2_bag_path": bag2["bag_name"],
                                "json_1_and_2_result": checker1,
                            }
                    # 情况五：两个json中的success都为true
                    elif checker1["success"] and checker2["success"]:
                        output[checker]["same_pass"][bag1["bag_name"].split('/')[-1]] = {
                            "json_1_bag_path": bag1["bag_name"],
                            "json_2_bag_path": bag2["bag_name"],
                        }

    # 将输出结果写入JSON文件
    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)

# 执行比较并输出结果
compare_jsons(json1_path, json2_path, output_path)