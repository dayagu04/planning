# -*- coding: utf-8 -*-
import re
import os
import sys
import string

class interface_version:
    var = None
    @classmethod
    def init_var(cls, value):
        cls.var = value
def prefix_generation(src_and_dst):
    result = "#pragma once\n\n"
    result += "#include \"base_convert.h\"\n"
    result += "#include \"rosbag_converter/converter_register.h\"\n\n"
    for typename in src_and_dst:
        typename = typename.split(".")[0]
        result += "#include \"" + dst_namespace + "/$typename.h\"\n"
        result += "#include \"" + src_namespace + "/$typename.h\"\n"
        result = result.replace("$typename", typename)
    result += "\n"
    return result

def base_convert_generation(first, second):
    result = "convert(old_ros_v.$second, ros_v.$second, type);\n"
    result = result.replace("$second", second)
    return result

def array_convert_generation(first, second):
    pattern = re.compile(r'\[(\d+)\]')
    size = re.search(pattern, first).group(1)
    if(second == "interface_version"):
        result = "static char cur_interface_version[$size] = \"interface$cur_interface_version\";\n"
        result = result.replace("$cur_interface_version", interface_version.var)
        result += "for (int i = 0; i < $size; i++) {\n"
        result += "    convert(cur_interface_version[i], ros_v.$second[i], type);\n"
    else:
        result = "for (int i = 0; i < $size; i++) {\n"
        result += "    convert(old_ros_v.$second[i], ros_v.$second[i], type);\n"
    result += "}\n"
    result = result.replace("$size", size)
    result = result.replace("$second", second)
    return result

def vector_convert_generation(first, second):
    result = "ros_v.$second.resize(old_ros_v.$second.size());\n"
    result += "for (int i = 0; i < ros_v.$second.size(); i++) {\n"
    result += "    convert(old_ros_v.$second[i], ros_v.$second[i], type);\n"
    result += "}\n"
    result = result.replace("$second", second)
    return result

def parser_element(element):
    pattern = re.compile(r'\[(\d+)\]')
    first = element.split(" ")[0]
    second = element.split(" ")[1]
    if first.endswith("[]"):
        return vector_convert_generation(first, second)
    if re.search(pattern, first):
        return array_convert_generation(first, second)
    return base_convert_generation(first, second)

def parser_difftype_element(src_diff_dst, dst_diff_src):
    result = ""
    for src_elem in src_diff_dst[:]:
        for dst_elem in dst_diff_src[:]:
            if src_elem.split(' ')[1] == dst_elem.split(' ')[1]:
                result += "// warning : modified struct member : $src_elem -> $dst_elem\n"
                result = result.replace("$src_elem", src_elem)
                result = result.replace("$dst_elem", dst_elem)
                result += parser_element(dst_elem)
                src_diff_dst.remove(src_elem)
                dst_diff_src.remove(dst_elem)
    return result

def parser_postprocess(src_diff_dst, dst_diff_src):
    result = ""
    result += parser_difftype_element(src_diff_dst, dst_diff_src)
    if src_diff_dst != []:
        for item in src_diff_dst:
            result += "// warning : removed struct member : " + item + "\n"
    if dst_diff_src != []:
        for item in dst_diff_src:
            result += "// warning : added struct member : " + item + "\n"
    return result


def parser_content(dst_content, src_content):
    result = ""
    src_and_dst = [item for item in src_content.splitlines() if item in dst_content.splitlines()]
    src_diff_dst = [item for item in src_content.splitlines() if item not in dst_content.splitlines()]
    dst_diff_src = [item for item in dst_content.splitlines() if item not in src_content.splitlines()]
    for element in src_and_dst:
        result += parser_element(element)
    result += parser_postprocess(src_diff_dst, dst_diff_src)
    result = result.rstrip('\n')
    return result

def infix_generation(dst_path, src_path, src_and_dst):
    result = ""
    for rosmsg in src_and_dst:
        with open(dst_path + rosmsg, 'r', encoding='utf-8') as f:
            dst_content = f.read()
        with open(src_path + rosmsg, 'r', encoding='utf-8') as f:
            src_content = f.read()
        typename = rosmsg.split(".")[0]
        result += "template <typename T1>\n"
        result += "void convert(T1 &&old_ros_v, " + dst_namespace + "::$typename &ros_v, ConvertTypeInfo type) {\n"
        result += re.sub(r'^', '\t', parser_content(dst_content, src_content), flags=re.MULTILINE) + "\n"
        result += "}\n\n"
        result = result.replace("$typename", typename)
    return result

def parser_topic_convert(filename):
    result = []
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    for line in content.splitlines():
        result.append(line)
    return result

def single_convert_generation(topic, typename):
    name = topic.replace("/", "_") + "_converter"
    result = "REG_CONVERT_SINGLE($name, \"$topic\", $typename);\n"
    result = result.replace("$name", name)
    result = result.replace("$topic", topic)
    result = result.replace("$typename", typename)
    return result

def postfix_generation(src_and_dst):
    result = ""
    topic_list = parser_topic_convert(single_convert_file)
    for rostopic in src_and_dst:
        for topic in topic_list:
            if rostopic.split('.')[0] == topic.split(' ')[1]:
                result += single_convert_generation(topic.split(' ')[0], topic.split(' ')[1])
    return result

def direct_convert_generation(topic):
    name = topic.replace("/", "_") + "_converter"
    result = "REG_CONVERT_DIRECT($name, \"$topic\");\n"
    result = result.replace("$name", name)
    result = result.replace("$topic", topic)
    return result

def directfix_generation():
    result = ""
    result = "#pragma once\n\n"
    result += "#include \"base_convert.h\"\n"
    result += "#include \"rosbag_converter/converter_register.h\"\n\n"
    topic_list = parser_topic_convert(direct_convert_file)
    for topic in topic_list:
        result += direct_convert_generation(topic)
    return result


def include_generation():
    result = ""
    include_path = sorted(os.listdir(ros_convert_path))
    for file in include_path:
        if file.endswith('.h'):
            result += "#include \"" + ros_convert_path + "/" +  file + "\"\n"
    with open("rosbag_converter/ros_convert_inc.h", 'w', encoding='utf-8') as f:
        f.write(result)

def parser_struct(dst_path, src_path, src_and_dst):
    try:
        os.mkdir(ros_convert_path)
    except:
        pass
    for struct_dir in src_and_dst:
        dst_path_ = dst_path + struct_dir + "/"
        src_path_ = src_path + struct_dir + "/"
        dst_struct_list = sorted(os.listdir(dst_path_))
        src_struct_list = sorted(os.listdir(src_path_))
        src_and_dst = [item for item in src_struct_list if item in dst_struct_list]
        src_diff_dst = [item for item in src_struct_list if item not in dst_struct_list]
        dst_diff_src = [item for item in dst_struct_list if item not in src_struct_list]
        result = prefix_generation(src_and_dst)
        result += infix_generation(dst_path_, src_path_, src_and_dst)
        if src_diff_dst != []:
            for item in src_diff_dst:
                result += "// warning : removed ros message : " + item + "\n"
        if dst_diff_src != []:
            for item in dst_diff_src:
                result += "// warning : added ros message : " + item + "\n"
        result += postfix_generation(src_and_dst)
        with open(ros_convert_path + "/" + struct_dir + ".h", 'w', encoding='utf-8') as f:
            f.write(result)
    result = directfix_generation()
    with open(ros_convert_path + "/direct_convert_c.h", 'w', encoding='utf-8') as f:
        f.write(result)
    include_generation()

def parser_path(dst_path, src_path):
    if(len(sys.argv) < 2):
        print("Usage: python ros_convert_generator.py <interface_version>")
        exit(1)
    interface_version.init_var(sys.argv[1])
    print("interface_version: ", interface_version.var)
    dst_path = dst_path + "/msg/"
    src_path = src_path + "/msg/"
    dst_path_list = sorted(os.listdir(dst_path))
    src_path_list = sorted(os.listdir(src_path))
    src_and_dst = [item for item in src_path_list if item in dst_path_list]
    src_diff_dst = [item for item in src_path_list if item not in dst_path_list]
    dst_diff_src = [item for item in dst_path_list if item not in src_path_list]
    parser_struct(dst_path, src_path, src_and_dst)


ros_path = "../rosmsg/"
dst_path = ros_path + "struct_msgs"
src_path = ros_path + "struct_msgs_v2_10"
dst_namespace = "struct_msgs"
src_namespace = "struct_msgs_v2_10"
ros_convert_path = "ros_convert"
single_convert_file = "single_topic_convert.md"
direct_convert_file = "direct_topic_convert.md"

if __name__ == '__main__':
    parser_path(dst_path, src_path)
