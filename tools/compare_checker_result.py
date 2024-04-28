#!/usr/bin/env python3

"""
在下方函数中填入两个checker结果json文件的路径(json1_path, json1_path), 程序将对比这两个json文件并输出差异, 差异结果将存储在指定的output_path中

结果json层级结构:
checker_1:
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

checker_2:
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
···
"""

import json

# 使用提供的文件路径/dock
json1_path = "/docker_share/temp/json1.json"
json2_path = "/docker_share/temp/json2.json"
output_path = "/docker_share/temp/compare_output.json"

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
    output = {checker: {"failed_pass": {}, "pass_failed": {}, "diff_failed": {}}
              for checker in all_checkers}

    # 遍历两个JSON文件中的bags进行比较
    for bag1 in json1:
        for bag2 in json2:
            if bag1["bag_name"] == bag2["bag_name"]:
                common_checkers = set(bag1["checker_result"].keys()) & set(bag2["checker_result"].keys())
                for checker in common_checkers:
                    # 情况一：json1中的success为true，json2中的success为false
                    if bag1["checker_result"][checker]["success"] and not bag2["checker_result"][checker]["success"]:
                        output[checker]["failed_pass"][bag1["bag_name"]] = {
                            "json_1_result": bag1["checker_result"][checker],
                            "json_2_result": bag2["checker_result"][checker]
                        }
                    # 情况二：json1中的success为false，json2中的success为true
                    elif not bag1["checker_result"][checker]["success"] and bag2["checker_result"][checker]["success"]:
                        output[checker]["pass_failed"][bag1["bag_name"]] = {
                            "json_1_result": bag1["checker_result"][checker],
                            "json_2_result": bag2["checker_result"][checker]
                        }
                    # 情况三：两个json中的success都为false，但是其他字段不一样
                    elif not bag1["checker_result"][checker]["success"] and not bag2["checker_result"][checker]["success"]:
                        diff_fields = {}
                        for key in bag1["checker_result"][checker]:
                            if key != "success" and bag1["checker_result"][checker][key] != bag2["checker_result"][checker][key]:
                                diff_fields[key] = (bag1["checker_result"][checker][key], bag2["checker_result"][checker][key])
                        if diff_fields:
                            output[checker]["diff_failed"][bag1["bag_name"]] = {
                                "json_1_result": bag1["checker_result"][checker],
                                "json_2_result": bag2["checker_result"][checker],
                                "diff_fields": diff_fields
                            }

    # 将输出结果写入JSON文件
    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)

# 执行比较并输出结果
compare_jsons(json1_path, json2_path, output_path)