import re

def base_topic_match(content):
    result = ""
    pattern = re.compile(r'\*\*topic：\*\* (.+?)\n.+?\*\*结构体：\*\* (.+?)\n', re.DOTALL)
    matches = pattern.findall(content)
    if matches:
        for match in matches:
            result += match[0] + " " + match[1] + "\n"
    return result

def multi_topic_fix(first, second):
    result = ""
    pattern = re.compile(r'((/[0-9a-zA-Z_/]+))')
    matches = pattern.findall(first)
    if matches:
        for match in matches:
            result += match[0] + " " + second + "\n"
    return result

def multi_topic_match(content):
    result = ""
    pattern = re.compile(r'\*\*topic：\*\*\n(.+?)\*\*结构体：\*\* (.+?)\n', re.DOTALL)
    matches = pattern.findall(content)
    if matches:
        for match in matches:
            result += multi_topic_fix(match[0], match[1])
    return result

def remove_chinese_line(content):
    chinese_pattern = re.compile(r'[\u4e00-\u9fff]')
    lines = content.split('\n')
    filtered_lines = [line for line in lines if not chinese_pattern.search(line)]
    result = '\n'.join(filtered_lines)
    return result

def parser_interface_description(filepath):
    result = ""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    result += base_topic_match(content)
    result += multi_topic_match(content)
    result = remove_chinese_line(result)
    return result

if __name__ == '__main__':
    result = parser_interface_description("../src/interface_description.md")
    with open("single_topic_convert.md", 'w', encoding='utf-8') as f:
        f.write(result)
