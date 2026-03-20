import os
import subprocess
import glob
import sys
import datetime

# 可执行文件路径
executable_path = "/home/qcchen/code/ehr_e541/components/ad_common/build/sdpromap_test"

# 数据路径列表
data_dirs = [
    "/data_cold/abu_zone/autoparse/bestune_e541_35401/trigger/20260119/20260119-15-59-57",
    "/data_cold/abu_zone/autoparse/chery_m32t_74563/trigger/20260120/20260120-15-38-45",
    "/data_cold/abu_zone/autoparse/chery_m32t_74563/trigger/20260120/20260120-16-43-13",
    "/data_cold/abu_zone/autoparse/chery_m32t_74563/trigger/20260120/20260120-16-45-51",
    "/data_cold/abu_zone/autoparse/chery_m32t_74563/trigger/20260120/20260120-17-34-55",
]


def check_log_errors(log_files_mapping):
    """检测日志文件中是否包含错误信息

    Args:
        log_files_mapping: 字典，键为日志文件路径，值为对应的数据路径
    """
    error_patterns = [
        "cannot get next link",
        "cannot get nearest link",
        "cannot merge route links",
        "check cur link id error",
        "check next link id error",
    ]

    print("\n" + "=" * 70)
    print("开始检测日志文件中的错误信息...")
    print("=" * 70)

    total_errors = 0
    files_with_errors = []

    for log_file, data_path in log_files_mapping.items():
        if not os.path.exists(log_file):
            continue

        errors_in_file = {}
        try:
            with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()
                for line_num, line in enumerate(lines, 1):
                    for pattern in error_patterns:
                        if pattern in line.lower():
                            if pattern not in errors_in_file:
                                errors_in_file[pattern] = []
                            errors_in_file[pattern].append(line_num)
                            total_errors += 1
        except Exception as e:
            print(f"[WARNING] 无法读取日志文件 {log_file}: {e}")
            continue

        if errors_in_file:
            files_with_errors.append((log_file, data_path, errors_in_file))

    # 输出检测结果
    if files_with_errors:
        print(
            f"\n发现 {total_errors} 个错误，涉及 {len(files_with_errors)} 个日志文件:\n"
        )
        for log_file, data_path, errors in files_with_errors:
            print(f"文件: {log_file}")
            print(f"数据路径: {data_path}")
            for pattern, line_nums in errors.items():
                print(
                    f"  - {pattern}: 出现 {len(line_nums)} 次 (行号: {line_nums[:10]}{'...' if len(line_nums) > 10 else ''})"
                )
            print()

        # 统一打印所有出错的数据路径
        print("=" * 70)
        print("所有包含错误的数据路径汇总:")
        print("-" * 70)
        for log_file, data_path, errors in files_with_errors:
            print(f"{data_path}")
    else:
        print("\n✓ 未发现任何错误日志")

    print("=" * 70)
    return total_errors, files_with_errors


def run_test():
    """执行测试程序并生成日志文件

    Returns:
        dict: 字典，键为日志文件路径，值为对应的数据路径
    """
    if not os.path.exists(executable_path):
        print(f"Error: Executable not found at {executable_path}")
        return {}

    print(f"Starting batch processing with executable: {executable_path}")
    print(f"Total directories to process: {len(data_dirs)}")
    print("-" * 50)

    # 创建log目录（如果不存在）
    log_dir = "log"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
        print(f"Created log directory: {os.path.abspath(log_dir)}")

    # 记录所有生成的日志文件路径及其对应的数据路径
    log_files_mapping = {}

    for data_dir in data_dirs:
        # Search for bag files ending with _no_camera.bag
        search_pattern = os.path.join(data_dir, "*_no_camera.bag")
        bag_files = glob.glob(search_pattern)

        if not bag_files:
            print(f"[SKIP] No matching bag file found in: {data_dir}")
            continue

        for bag_file in bag_files:
            # 提取包名用于生成日志文件名
            # 例如: /path/to/data_no_camera.bag -> data_no_camera.log
            bag_name = os.path.basename(bag_file)
            log_name = os.path.splitext(bag_name)[0] + ".log"

            # 日志文件将保存在log目录下
            log_file_path = os.path.abspath(os.path.join(log_dir, log_name))

            # 记录日志文件路径与数据路径的映射关系
            log_files_mapping[log_file_path] = data_dir

            print(f"\n[{datetime.datetime.now()}] [RUNNING] Processing bag: {bag_file}")
            print(f"       Log file: {log_file_path}")

            try:
                # 为每个包单独打开一个日志文件
                # 使用行缓冲模式 (buffering=1) 确保输出及时写入
                # 注意：buffering=0 在某些系统上可能不可用，所以使用 buffering=1
                with open(
                    log_file_path, "w", encoding="utf-8", buffering=1
                ) as log_file:
                    start_msg = f"Processing bag: {bag_file}\nDate: {datetime.datetime.now()}\n{'-'*50}\n"
                    log_file.write(start_msg)
                    log_file.flush()
                    os.fsync(log_file.fileno())  # 强制同步到磁盘

                    # 将 stdout 和 stderr 都重定向到该包的独立日志文件
                    # 使用 subprocess.Popen 并实时读取，确保捕获所有输出（包括崩溃前的输出）
                    env = os.environ.copy()
                    env["PYTHONUNBUFFERED"] = "1"
                    # 设置 C++ 程序的无缓冲输出环境变量
                    env["_GLIBCXX_FORCE_NEW"] = "1"  # 某些情况下可以帮助

                    # 直接使用文件对象，让 subprocess 直接写入文件
                    # 这样可以避免 Python 层面的缓冲问题，C++ 程序的输出会直接写入文件
                    # 注意：需要确保文件对象是行缓冲或无缓冲的
                    result = subprocess.run(
                        [executable_path, bag_file],
                        stdout=log_file,
                        stderr=log_file,  # stderr 也写入同一个文件
                        check=False,  # 不抛出异常，我们自己处理
                        env=env,
                    )

                    # 确保所有内容都写入文件
                    log_file.flush()
                    os.fsync(log_file.fileno())
                    return_code = result.returncode

                    if return_code != 0:
                        raise subprocess.CalledProcessError(
                            return_code, executable_path
                        )

                    end_msg = f"\n{'-'*50}\n[SUCCESS] Finished processing.\n"
                    log_file.write(end_msg)
                    log_file.flush()

                print(f"[SUCCESS] Finished processing {bag_name}")

            except subprocess.CalledProcessError as e:
                print(
                    f"[ERROR] Failed to process {bag_name}. Exit code: {e.returncode}"
                )
                # 错误信息也追加到日志中（如果有部分输出的话）
                with open(log_file_path, "a") as log_file:
                    log_file.write(
                        f"\n[ERROR] Process failed with exit code {e.returncode}\n"
                    )
            except Exception as e:
                print(f"[ERROR] Unexpected error processing {bag_name}: {e}")
                with open(log_file_path, "a") as log_file:
                    log_file.write(f"\n[ERROR] Unexpected exception: {e}\n")

            print("-" * 50)

    print("\nBatch processing completed.")

    return log_files_mapping


if __name__ == "__main__":
    # 执行测试程序，生成日志文件
    log_files_mapping = run_test()

    # 在所有测试完成后，检测日志文件中的错误
    if log_files_mapping:
        total_errors, files_with_errors = check_log_errors(log_files_mapping)

        # 只保留有错误的日志文件，删除没有错误的日志
        error_log_files = (
            set(log_file for log_file, _, _ in files_with_errors)
            if files_with_errors
            else set()
        )

        for log_file in log_files_mapping.keys():
            if log_file not in error_log_files and os.path.exists(log_file):
                os.remove(log_file)
