#!/bin/bash
src_file="CHANGELOG.md"
target_file="Makefile"

first_line=$(head -n 1 $src_file)
first_line=$(echo "${first_line}" | cut -c 2-) 

# 读取文件第二行并提取":="符号后面的内容  
second_line=$(sed -n '2p' "$target_file")  
content_after_equal=$(echo "$second_line" | cut -d'=' -f2-)  
  
# 替换字符串变量的值  
modified_second_line=$(echo "$second_line" | sed "s|$content_after_equal|$first_line|")  
  
# 将修改后的第二行写回到文件  
sed -i "2s|.*|$modified_second_line|" "$target_file"