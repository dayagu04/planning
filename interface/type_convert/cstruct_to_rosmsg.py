# -*- coding: utf-8 -*-
import re
import os
import sys

# Usage: python cstruct_to_rosmsg.py [struct_path] [output_path]
# 注意：会生成重名的接口 需要手动修改
# Header 改 MsgHeader
# McuOutputMsg_T.msg SnsDtBufType.msg SnsOpInfoType.msg DtObjUPASnsDtObjDisInfoType.msg 多维数组
# LDPOutputInfoStr.msg ELKOutputInfoStr.msg 定义跨行了 改boolen为bool
# AusspsUss.msg corner_points的size有问题


def match_struct_enum(c_header):
    stack = []
    pop = []
    all = []
    with open(c_header, "r", encoding='utf-8') as f:
        content = f.read()
        # 删除跨行注释
        content = re.sub(r"/\*.*?\*/", "", content, flags=re.DOTALL)
        # 删除行内注释
        content = re.sub(r"\/\/.*", "", content)
        content = re.sub(r"[\n]+", "\n", content, flags=re.DOTALL)
        #
        matches = re.findall(union_pattern, content)
        matches_content = re.findall(union_pattern_content, content)
        if len(matches) != 0 and len(matches) == len(matches_content):
            for i in range(len(matches)):
                content = content.replace(matches[i], matches_content[i])
        for line in content.splitlines():
            # print(line)
            # 不匹配namespace的{}
            if "namespace" in line:
                continue
            # 去除无效换行
            if line == "" or line == "\n":
                continue
            # 只匹配typedef
            if "{" in line:
                if "typedef" in line:
                    stack.append([])
                    stack[-1].append(line)
                else:
                    continue
            elif "}" in line:
                if len(stack) == 0:
                    continue
                stack[-1].append(line)
                pop.append(stack.pop())
            elif len(stack) != 0:
                stack[-1].append(line)
    for i in pop:
        all.append("\n".join(i))
    return all


def process_define(c_header):
    global struct_define
    c_path = str(sys.argv[1]) + "/" + c_header
    # 读取C语言头文件
    with open(c_path, "r", encoding='utf-8') as f:
        content = f.read()

    matches = re.findall(define_pattern, content)
    # print(matches)
    if len(matches) == 0:
        return
    struct_define += "# {}\n".format(c_header)
    for match in matches:
        if match[0] == "_STRUCT_ALIGNED_" or match[0] == "_ENUM_PACKED_":
            continue
        if "/" in match[0]:
            continue
        left = match[0]
        right = re.sub(r"\((\d+)[uU]\)", r"\1", match[1]).replace(" ", "")
        define_map[left] = right
        # avmsize 图像数据改成变长
        if left == "PDATA_SIZE":
            define_map[left] = ""  # 512*512*3
            continue
        if right in define_map.keys():
            right = define_map[right]
            define_map[left] = right

        struct_define += "int32 {} = {}\n".format(left, right)
    struct_define += "\n"


def process_enum(content):
    # 匹配枚举定义
    matches = re.findall(enum_pattern, content)
    for match in matches:
        enum_name = match.replace("_ENUM_PACKED_", "")
        enum_name = enum_name.replace(" ", "")
        # ENUM_PACK后都是1字节
        ros_type_map[enum_name] = "int16"

    # 匹配枚举定义
    common_matches = re.findall(common_enum_pattern, content)
    for match in common_matches:
        common_enum_name = match.replace("_ENUM_PACKED_", "")
        common_enum_name = common_enum_name.replace(" ", "")
        ros_type_map[common_enum_name] = "int16"


def process_struct(content):
    # # 处理结构体内容
    # content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    # content = content.replace("/", "#")

    # 删除跨行注释
    content = re.sub(r"/\*.*?\*/", "", content, flags=re.DOTALL)
    # 删除行内注释
    content = re.sub(r"\/\/.*", "", content)
    content = re.sub(r"[\n]+", "\n", content, flags=re.DOTALL)

    out = []

    test = []
    # print("-"*20)
    for line in content.split(";"):
        line = line.replace("\n", "")
        line = re.sub(r"\s+", " ", line)
        test.append(line)
    content = "\n".join(test)
    content = content.replace(" [", "[")
    # print(content)

    for line in content.splitlines():
        # 去除等号右边默认值
        if "=" in line:
            line = line.split("=")[0]
        # 去除无效换行
        if line == "" or line == "\n":
            continue
        # 去除默认值
        line = re.sub(r"(=.*;)", "", line)
        line = line.replace(";", "")
        # 去除空格
        lines = [i for i in line.split(" ") if i != ""]
        if len(lines) < 2:
            out.append(" ".join(lines))
            continue
        # 替换为ros类型
        if lines[0] in ros_type_map.keys():
            # print("---")
            # print(lines[0])
            lines[0] = ros_type_map[lines[0]]
            # print(lines[0])
            # print("---")
        # 数组转换
        res = re.search(r"(\[.*\])", lines[1])
        if res:
            size = res.group(1)
            lines[0] += size
            lines[1] = re.sub(r"(\[.*\])", "", lines[1])
            size = size.replace("[", "").replace("]", "")
            if size in define_map.keys():
                # print(define_map[size])
                lines[0] = lines[0].replace(size, define_map[size])
        # 变长数组转换
        flex_array_name = [lines[1] + i for i in suffix]
        # print(flex_array_name)
        for flex_array in flex_array_name:
          if flex_array in content:
            lines[0] = re.sub(r"(\[.*\])", "[]", lines[0])
            
        line = " ".join(lines)
        out.append(line)
    # print(out)
    return "\n".join(out)

def process_fm_content(content, rosmsg_folder):
    matches = re.findall(struct_pattern, content)
    rosmsg_file = (
            rosmsg_folder
            + "/FmInfoArray.msg"
        )
    for match in matches:
      new_content = match[1].replace(" ", "").replace("_STRUCT_ALIGNED_", "") + "[] fmInfos\n"
      new_content = "uint64 alarm_counts \n" +new_content
      new_content = "std_msgs/Header header # ros header\n" +new_content
      with open(rosmsg_file, "w", encoding='utf-8') as f:
            f.write(new_content)

def process_content(content, rosmsg_folder):
    # 匹配结构体定义
    matches = re.findall(struct_pattern, content)

    # 输出结果
    for match in matches:
        # rosmsg_file = rosmsg_folder + "/" + prefix + match[1].replace(" ", "").replace("_STRUCT_ALIGNED_", "") + ".msg"
        rosmsg_file = (
            rosmsg_folder
            + "/"
            + match[1].replace(" ", "").replace("_STRUCT_ALIGNED_", "")
            + ".msg"
        )
        #rosmsg_file = rosmsg_file.replace("Header", "MsgHeader")
        file_name = rosmsg_file.split("/")[-1].split(".")[0]
        # 处理结构体内容
        new_content = process_struct(match[0])
        if "MsgHeader msg_header" in new_content:
          new_content = "std_msgs/Header header # ros header\n"+new_content
        # interface里没header的消息类型手动加header
        if file_name in no_header:
          new_content = "std_msgs/Header header # ros header\n"+new_content
        #
        with open(rosmsg_file, "w", encoding='utf-8') as f:
            f.write(new_content)

def process_cmake_msg_add_message_files(directory):
    try:
        # 获取目录中的所有条目
        entries = os.listdir(directory)
        # 过滤出其中的子目录
        subdirectories = [entry for entry in entries if os.path.isdir(os.path.join(directory, entry))]
        subdirectories.sort()
        # print("子目录:", subdirectories)
        cmake_msg_add_message_files_array = []
        for subdir in subdirectories:
            if "common_platform_type_soc" in subdir: continue
            cmake_msg_add_message_files_array.append(f'add_message_files(DIRECTORY msg/{subdir})\n')
        return ''.join(cmake_msg_add_message_files_array)
    except FileNotFoundError:
        print(f"错误: 目录 '{directory}' 不存在。")
        return []
    except PermissionError:
        print(f"错误: 没有权限访问目录 '{directory}'。")
        return []


# 获取当前目录下的文件和文件夹列表
h_list = os.listdir(sys.argv[1])
out_path = str(sys.argv[2]) + "/msg"

ros_type_map = {
    "float": "float32",
    "double": "float64",
    "sint8": "int8",
    "sint16": "int16",
    "sint32": "int32",
    "sint64": "int64",
    "boolean": "bool",
    "uint64_t": "uint64",
    "uint8_t": "uint8",
    "uint32_t": "uint32",
    "size_t": "uint32",
    "int64_t": "int64",
    "int32_t": "int32",
    "int16_t": "int16",
    "int8_t": "int8",
    "HmiTagMask": "uint64",
    "int": "int32",
}
define_map = {"PROTO_STR_LEN": "64"}

suffix = ["_size", "_num"]

struct_define = ""
enum_pattern = r"typedef\s+enum\s+\{[\s\S]*?\}(.*);"
common_enum_pattern = r"enum\s+(.*)\{[\s\S]*?\};"
struct_pattern = r"typedef\s+struct.*\{([\s\S]*?)\}(.*);"
define_pattern = r"#define\s+(\w+)[ \f\r\t\v]+([ \w\(\)]+)[\S+/\*.*]*"
union_pattern_content = r"union\s+\{([\s\S]*?)\}.*;"
union_pattern = r"union\s+\{[\s\S]*?\}.*;"

not_convert = ["c_to_json.h", "c_to_ros.h", "c_to_ros_macro.h", "sensor_image_c.h", "ifly_phm_c.h", "camera_hot_swap_c.h", 
               "disk_info_c.h", "fdl_net_stat_check_c.h", "fdl_trigger_c.h", "icc_c.h", "ota_log_upload_c.h", 
               "sensor_camera_status_c.h"]
no_header = ["Front_Camera_Image_Info", "Panorama_Camera_Image_Info", "Surround_Camera_Image_Info"]
fm_convert = ["fm_info_c.h"]

cmakelist_txt_ori = """cmake_minimum_required(VERSION 2.8.3)
project(struct_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs)
add_service_files(DIRECTORY srv)
add_service_files(DIRECTORY srv/HMI)
add_message_files(DIRECTORY msg)
{0}
generate_messages(DEPENDENCIES std_msgs)
include_directories(${{catkin_INCLUDE_DIRS}})
catkin_package()
"""

if __name__ == "__main__":
    try:
        os.mkdir(out_path)
    except:
        # print("The path already exists!")
        pass
    for c_header in h_list:
        if (
            ".hpp" in c_header
            or ".proto" in c_header
            or ".txt" in c_header
            or c_header in not_convert
        ):
            continue
        c_path = str(sys.argv[1]) + "/" + c_header
        # 处理define
        process_define(c_header)
        all_define = match_struct_enum(c_path)
        for define in all_define:
            process_enum(define)
    # print(ros_type_map)
    for c_header in h_list:
        if (
            ".hpp" in c_header
            or ".proto" in c_header
            or ".txt" in c_header
            or c_header in not_convert
        ):
            continue
        c_path = str(sys.argv[1]) + "/" + c_header
        all_define = match_struct_enum(c_path)
        rosmsg_folder = out_path + "/" + c_header.split(".")[0]
        try:
            os.mkdir(rosmsg_folder)
        except:
            pass
        for define in all_define:
            process_content(define, rosmsg_folder)

        if (c_header in fm_convert):
            process_fm_content(define, rosmsg_folder)

    str_cmake_msg_add_message_files = process_cmake_msg_add_message_files(out_path)
    cmakelist_txt_content = cmakelist_txt_ori.format(str_cmake_msg_add_message_files)
    # print("文件内容:", cmakelist_txt_content)
    with open(sys.argv[2] + "/CMakeLists.txt", 'w') as cmakelist_file:
        cmakelist_file.write(cmakelist_txt_content)
