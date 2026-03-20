# -*- coding: utf-8 -*-
import re
import os
import sys
import string
        
def match_struct_enum(c_header):
    stack = []
    pop = []
    all = []
    includes = []
    include_pattern = re.compile(r'#include\s*["<](.*?)[">]')
    with open(c_header, 'r', encoding='utf-8') as f:
        content = f.read()
        # 删除跨行注释
        content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
        # 删除行内注释
        content = re.sub(r'\/\/.*', '', content)
        content = re.sub(r'[\n]+', '\n', content, flags=re.DOTALL)
        #
        matches = re.findall(union_pattern, content)
        matches_content = re.findall(union_pattern_content, content)
        if len(matches) != 0 and len(matches) == len(matches_content):
          for i in range(len(matches)):
            content = content.replace(matches[i], matches_content[i])
        for line in content.splitlines():
            include_match = include_pattern.search(line)
            if include_match:
                includes.append(include_match.group(1))
            # 不匹配namespace的{}
            if 'namespace' in line: continue
            # 只匹配typedef
            if '{' in line:
                if 'typedef' in line:
                    stack.append([])
                    stack[-1].append(line)
                else:
                    continue
            elif '}' in line:
                if len(stack) == 0: continue
                stack[-1].append(line)
                pop.append(stack.pop())
            elif len(stack) != 0:
                stack[-1].append(line)
    for i in pop:
        all.append('\n'.join(i))
    return all,includes

def process_struct(content):
    # 删除跨行注释
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    # 删除行内注释
    content = re.sub(r'\/\/.*', '', content)
    content = re.sub(r'[\n]+', '\n', content, flags=re.DOTALL)
    
  
    out = []
    now_line = ""
    new = ""
    # lines = [i for i in content.splitlines() if i != '']
    
    test = []
    for line in content.split(";"):
      line = line.replace('\n', '')
      line = re.sub(r'\s+', ' ', line)
      test.append(line)
    content = ';\n'.join(test)
    content = content.replace(" [", "[")   
    # print("-"*20)
    # print(content) 
    
    for line in content.splitlines():
        # 去除无效换行
        if line == "" or line == "\n": continue
        # 获取完整行
        now_line += line 
        if ';' not in now_line:
          continue
        new += now_line + "\n"
        # 去除默认值
        now_line = re.sub(r'(=.*);', '', now_line)
        now_line = now_line.replace(";", "")
        # 去除空格
        line_elem = [i for i in now_line.split(" ") if i != ""]
        if len(line_elem) < 2:
          continue
          now_line = ""
        # 去除数组
        res = re.search(r'\[(.*)\]', line_elem[1])
        if res:
          out.append([re.sub(r'\[.*\]', '', line_elem[1]), res])
        else:
          out.append(re.sub(r'\[.*\]', '', line_elem[1]))
        now_line = ""
    # print(new)
    return out

for_loop_schema = """  for (size_t i$count = 0; i$count < ros_v.$fild.size(); i$count++) {
	  convert(struct_v.$fild[i$count], ros_v.$fild[i$count], type);
  }"""

suffix = ["_size", "_num"]
flex_for_loop_schema = """  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.$flex_fild >= 0 && struct_v.$flex_fild <= $max_num) {
      ros_v.$fild.resize(struct_v.$flex_fild);
    } else {
      std::cout << "convert/$file.h:" << __LINE__ 
                << " [convert][TO_ROS] $flex_fild=" << struct_v.$flex_fild 
                << " not in range $max_num=" << $max_num 
                << std::endl;
      ros_v.$flex_fild = $max_num;
      ros_v.$fild.resize($max_num);
    }
    for (size_t i$count = 0; i$count < ros_v.$fild.size(); i$count++) {
      convert(struct_v.$fild[i$count], ros_v.$fild[i$count], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.$flex_fild > $max_num || ros_v.$flex_fild < 0 || ros_v.$fild.size() > $max_num) {
      std::cout << "convert/$file.h:" << __LINE__ 
                << "[convert][TO_STRUCT] $flex_fild=" << ros_v.$flex_fild 
                << " ros_v.$fild.size()=" << ros_v.$fild.size()
                << " not in range $max_num=" << $max_num 
                << std::endl;
    }
    if (ros_v.$fild.size() > $max_num) {
      for (size_t i$count = 0; i$count < $max_num; i$count++) {
        convert(struct_v.$fild[i$count], ros_v.$fild[i$count], type);
      }
    } else {
      for (size_t i$count = 0; i$count < ros_v.$fild.size(); i$count++) {
        convert(struct_v.$fild[i$count], ros_v.$fild[i$count], type);
      }
    }
  }
  //"""

union_pattern_content = r'union\s+\{([\s\S]*?)\}.*;'
union_pattern = r'union\s+\{[\s\S]*?\}.*;'

def process_content(content, rosmsg_folder):
    # 匹配结构体定义
    matches = re.findall(struct_pattern, content)
    out = ""
    for match in matches:
        filds = process_struct(match[0])
        new_filds = []
        for_count = 0
        # print(filds)
        for fild in filds:
          if type(fild) is list:
            max_num = fild[1][0].replace('[', '').replace(']', '')
            flex_array_name = [fild[0] + i for i in suffix]
            is_flex = False
            flex_name = ""
            for flex_array in flex_array_name:
              if flex_array in content:
                is_flex = True
                flex_name = flex_array
                break
            if is_flex:
              new_filds.append(string.Template(flex_for_loop_schema).substitute(
                {"count": for_count, "fild": fild[0], "flex_fild": flex_name, "max_num": max_num, "file":rosmsg_folder.split("/")[-1]}))
            else:
              if fild[0] == "pData":  # avm image改变长数组
                new_filds.append("  ros_v.pData.resize(struct_v.SizeBytes);  // avm image flex size")
              new_filds.append(string.Template(for_loop_schema).substitute({"count": for_count, "fild": fild[0]}))
            for_count+=1
            
          else:
            if fild == "msg_header":
              new_filds.append("  convert(struct_v.msg_header, ros_v.msg_header, type);")
              new_filds.append("  convert_ros_header(struct_v.msg_header, ros_v.header, type);")
            else:
              new_filds.append("  convert(struct_v.{}, ros_v.{}, type);".format(fild, fild))
        ins = {
              "type": match[1].replace(" ", "").replace("_STRUCT_ALIGNED_", ""),
              "fild": "\n".join(new_filds)
            }
        
        out = template.substitute(ins)
    return out
        

# 添加头文件
def process_include_str(all_include):
    include_array = []
    for include in all_include:
        if "common_platform_type_soc.h" in include or "Platform_Types.h" in include or "stdint.h" in include: continue
        include_array.append(f'#include "struct_convert/{include}"\n')
    return ''.join(include_array)



# 获取当前目录下的文件和文件夹列表
h_list = os.listdir(sys.argv[1])
out_path = str(sys.argv[2]) 

struct_pattern = r'typedef\s+struct.*\{([\s\S]*?)\}(.*);'
#
init_schema = """template <typename T2>
void convert(iflyauto::$type &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
$fild\n}\n
"""

template = string.Template(init_schema)
h_schema = """#pragma once

#include \"base_convert.h\"
#include \"c/{0}.h\"
{1}
using namespace iflyauto;

{2}"""

not_convert = ["c_to_json.h", "c_to_ros.h", "c_to_ros_macro.h", "ifly_phm_c.h",  "camera_hot_swap_c.h", 
               "disk_info_c.h", "fdl_net_stat_check_c.h", "fdl_trigger_c.h", "icc_c.h",
               "ota_log_upload_c.h", "sensor_camera_status_c.h"]

if __name__ == '__main__':
    try:
        os.mkdir(out_path)
    except:
        # print("The path already exists!")
        pass
    for c_header in h_list:
        if ".hpp" in c_header or ".proto" in c_header or ".txt" in c_header or c_header in not_convert: continue
        c_path = str(sys.argv[1]) + "/" + c_header
        rosmsg_folder = out_path + "/" + c_header.split(".")[0]
        all_define,all_include = match_struct_enum(c_path)
        
        add_include_str=process_include_str(all_include)
        
        all_def = ""
        for i in all_define:
          all_def += process_content(i, rosmsg_folder)
        res = h_schema.format(rosmsg_folder.split("/")[-1], add_include_str, all_def)
        
        with open(rosmsg_folder + ".h", 'w', encoding='utf-8') as f:
          f.write(res)
          