#!/usr/bin/env python3
"""
Usage: run_bag_convert.py [-h] [-i INPUT] [-f FILE] -t TARGET_COMMIT [-o OUTPUT_DIR]

Options:
  -i  一个或多个 bag 或目录, 逗号分隔 (至少提供 -i 或 -f 之一)
  -f  文件路径, 文件内容为每行一个 bag 路径或目录 (至少提供 -i 或 -f 之一)
  -t  目标 commit
  -o  输出目录, 默认为 output_<目标commit前7位>

Example:
python3 run_bag_convert.py -i /path/to/bag1,/path/to/bag_dir1 -t dst_commit
python3 run_bag_convert.py -f /path/to/bag_list.txt -t dst_commit -o /path/to/output_dir

"""

import argparse
import base64
import io
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_URL = "ssh://git@code.iflytek.com:30004/AUTO_FPilot_ToolChains/ClosedLoopAutomation/interface_trans_builder.git"
CLONE_DIR = SCRIPT_DIR / ".interface_trans_builder"
CONVERTERS_DIR = SCRIPT_DIR / ".converters"

# 留空则使用系统默认 KEY
KEY = "LS0tLS1CRUdJTiBPUEVOU1NIIFBSSVZBVEUgS0VZLS0tLS0KYjNCbGJuTnphQzFyWlhrdGRqRUFBQUFBQkc1dmJtVUFBQUFFYm05dVpRQUFBQUFBQUFBQkFBQUJsd0FBQUFkemMyZ3RjbgpOaEFBQUFBd0VBQVFBQUFZRUEzTnlsdUY5amNkS1FuVTUwQmptcGYvbFNCc1NUWVEzUkhRRmx6ZGk4dmxNcXlWczFlYmZQCnlxVUtUOEttMDdCTDU5aVQvdDY3MkRlS0N0RTJ4N2o4c21kQTdNMW15YnYydlVsVUJCN0JSM3o4dk5yRkozNDVTaVZleGUKR280VXkxcFNXdFJYaEpKL255NXBmZ2owdTJOenNZR215TzF2b29zdUtXY3RGMWI4M0VPRWxIRmlZeUpreW5mMzZ1dm9zTwpPUnhMbXBKeElGR3NRMVVNWHlYVmtzQlBJeXh2Slc1NUpOQi94QjdBUG1YRHpQMDlzeWsxVzFSaWhjcEtQeURDcHowSlp2CmFSYnlyNnRiKzVja1NsUUZQeUpaYkVQQ3QwazRGVzhSMVlqV29uSmVmT21uQ2tONWMzWkdVTHhtM3B2T3RQRlYwbit2K0cKWmNRVkNneTNzVkNzQ1h0L1Zxb0w4c2E5YTliTVVmb252ZjhQUXRBZnJqMGFUUEtnd0czNTVDUFVac1pmbjJXcTN1QXpVTgpuQ3o1dGZXMllDT1VtSlBKUGtaT0wzNG90MW4waWszek8rbnVQQmlxaFFWL0lmOWxtb2J3TXNpMHFjS1pJWWR3M2k5SXdqClZaRGFiUTJTN08yYjVEMnFoeHd0Wk1ydXJrOEdac2pPVDVzWFZhaFZBQUFGaUh0MXNsbDdkYkpaQUFBQUIzTnphQzF5YzIKRUFBQUdCQU56Y3BiaGZZM0hTa0oxT2RBWTVxWC81VWdiRWsyRU4wUjBCWmMzWXZMNVRLc2xiTlhtM3o4cWxDay9DcHRPdwpTK2ZZay83ZXU5ZzNpZ3JSTnNlNC9MSm5RT3pOWnNtNzlyMUpWQVFld1VkOC9MemF4U2QrT1VvbFhzWGhxT0ZNdGFVbHJVClY0U1NmNTh1YVg0STlMdGpjN0dCcHNqdGI2S0xMaWxuTFJkVy9OeERoSlJ4WW1NaVpNcDM5K3JyNkxEamtjUzVxU2NTQlIKckVOVkRGOGwxWkxBVHlNc2J5VnVlU1RRZjhRZXdENWx3OHo5UGJNcE5WdFVZb1hLU2o4Z3dxYzlDV2Iya1c4cStyVy91WApKRXBVQlQ4aVdXeER3cmRKT0JWdkVkV0kxcUp5WG56cHB3cERlWE4yUmxDOFp0NmJ6clR4VmRKL3IvaG1YRUZRb010N0ZRCnJBbDdmMWFxQy9MR3ZXdld6Rkg2SjczL0QwTFFINjQ5R2t6eW9NQnQrZVFqMUdiR1g1OWxxdDdnTTFEWndzK2JYMXRtQWoKbEppVHlUNUdUaTkrS0xkWjlJcE44enZwN2p3WXFvVUZmeUgvWlpxRzhETEl0S25DbVNHSGNONHZTTUkxV1EybTBOa3V6dAptK1E5cW9jY0xXVEs3cTVQQm1iSXprK2JGMVdvVlFBQUFBTUJBQUVBQUFHQUFSVHZsdHlZdnhqNTdsdFJCQ3JLSXI5VTRJCldiOXpDN0lubzhSbXJXV2RSTWRGb0hCQWFiZFBHa0FFb0N5U3ltNG9iVUNyNktpditTcElUcjBNU2VNRDJET3VXeXFEQk0KZ0JiWTI0QlBFQXVUclZVZy9CV0pNRXpJancyUTV6R1ovaW9XRkM0SHBiTjZ0aGh1V0NWVnlpVjI4OEl1WE8weXY5V09TUgpldmpKR2tjV1lpZkFCSjVKZ2hoZVZYODhUNGw5MEVuY2d6MTZXdXcxR3FVdGo3bDlGclVjL0k1NVVIOE44NGtQeFprclNxCjVka2pzVG4xY2ZJUzNYYmxNRFAzM2NlQXJYRmwxT1lVbm9Id1krYUJuUnVnMnNEenN4bUtoZVgxTUUzK3VyTzNlUDJaeUgKOENRT2ZjelFmeW5XUzZSZWxpbGhQclVDY2FWbDFPazV0VzFGekFGY05UaUJGcXE1d1BuMXd6S21ma21sSkVvUVVHNFpjVgpNbmlab0w1STVsRjh2M2U1TCtzcE1zdHo3TDlXOUVvRnQxdkxrM1R0WnpsQ0dvMHJCMGw4ekQ3MUtEYlJTS0M1cWpiQW5hCjhNaFc4U0hmQVgreTBmeGNHODBsaUY1TDE5V3p0RlBlTDBLaDkyejVqRTF5NlhqbXFnd2I3SGVUMjNSWXMzRVBQaEFBQUEKd1FEZkt2Q1ZkcUVwbUkzSUR0MExQbTBHTmJwMFFIZWgvWUVzTlNIYkFNOXdWd1ZRYzlyY0xmdThQMDJHaVI5ZkxvbCtFawpaVW03RmNFd2xPS1NldFN6U1FjeFFLTE9aeE5JYngxdjZ1cjZhNWRmbmJWZEJGL05ZMURZZHZ6ZzUwMVZHVVJPb2hCWXE5CjkyNUMydnRSRUpBMVJpSjJKaXFPVUU2MmJmTEpSeWl3elpocWIycDB5M2N4eHlFNGZIUzlJLzhoeFBPcXFTaDNwVWhucFcKWlAxdlA4ei91SDdPS1JmbFUrZ21sSE9NWWJTdTJRaUF6d3dWUmpBdWR1dFEwVE1rc0FBQURCQU92MDRvd1BRSi9OR1dWbgoxd2g0RWFhWnAzejFzeEQ3NmV0MEVQalB4M3R1RHVsMitmTnN1ZkJaUUdhYWRUT3VCSWxHdFYraE9nQ3Uzb3M4TWNvY2VzCjFiK3dmMkFrNkFmU1dUNnhMYjZET3ViTzJUR05zOEdiUGo4d0x3SkhldWhrK2Z1M2hWYnR3N2tuUzRwY0VYQnpTOW1wU1QKQTVLTFlCRW1uUXZDbWl6OUNxaGZBTXkyaHRJV2sxVXZQM1VCNldEZFVmdzZ2amFuSnVmMmFXWVNBZk1iS2ZoZHI3WnlYbApTNnFWWkExR05xMlVXL1hHL3ZkYm1GNXhQV2VEdWdUUUFBQU1FQTc1K0RhTXpIWnZhV3JUVGdKaGZ1QnNzUnV5Qk9pUm1ECklPTzd5ckxTdFI0L2pDSU95eEdzSkE4bFZPdVJoQjhRamdpd2QreEVEUitkNlN0c2ZXbm9sN3JxQkdJcHhzN1p2dzYvcTAKUFhyRFNZV2JJdXpGYXh2MXlEVzdkdzJGNXd0T2U3cUs2ZnM3dnlLMjMxRSsraGxVMXFXc0MwU0R4SFdyLzFXMDFjakhqUgpRaExLTnpIYTd4WlJXUWNvZHI2YXIxVFNEaUVFMkNTeDlqeFJzUUlSY1pXVkdiV1VxQmhmbVVyczdHMjczRjZZc3p3VXZMCi9qbWZoVGZNYUhDdXdwQUFBQUQzbHlhMjl1WjBCM2IzSnJaWEl5TkFFQ0F3PT0KLS0tLS1FTkQgT1BFTlNTSCBQUklWQVRFIEtFWS0tLS0tCg=="


def get_interface_commitid(bag_file: str) -> str:
    """从 bag 的 /iflytek/system/version topic 解析 interface commit, 未找到返回空串. """
    try:
        import rosbag
    except ImportError:
        raise SystemExit("缺失 rosbag 依赖, 请在 ROS 环境执行或安装 rosbag 依赖")

    try:
        with rosbag.Bag(bag_file, "r") as bag:
            for _topic, msg, _t in bag.read_messages(topics=["/iflytek/system/version"]):
                try:
                    data = msg._buff
                except Exception:
                    buff = io.BytesIO()
                    msg.serialize(buff)
                    data = buff.getvalue()

                pos = 0
                while True:
                    pos = data.find(b"interface", pos)
                    if pos == -1:
                        break
                    module_start = pos
                    while module_start > 0 and data[module_start - 1] != 0:
                        module_start -= 1
                    module_name = data[module_start : pos + 9].decode("utf-8", errors="ignore")
                    if module_name != "interface":
                        pos += 1
                        continue
                    commitid_start = module_start + 64
                    if commitid_start + 40 > len(data):
                        pos += 1
                        continue
                    commitid_bytes = data[commitid_start : commitid_start + 40]
                    commitid = ""
                    for b in commitid_bytes:
                        if b == 0:
                            break
                        commitid += chr(b)
                    if len(commitid) >= 20:
                        return commitid
                    pos += 1
        return ""
    except Exception:
        return ""


def _git_env_with_key():
    """返回 (env, key_path); 否则 (os.environ, None). 结束后 unlink key_path. """
    if not (KEY and KEY.strip()):
        return os.environ.copy(), None
    try:
        raw = base64.b64decode(KEY.strip())
        key_content = raw.decode("utf-8", errors="replace")
    except Exception:
        return os.environ.copy(), None
    fd, path = tempfile.mkstemp(prefix="clone_key_", suffix="_rsa")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(key_content)
        os.chmod(path, 0o600)
        env = os.environ.copy()
        env["GIT_SSH_COMMAND"] = f"ssh -i {path} -o StrictHostKeyChecking=no"
        return env, path
    except Exception:
        try:
            os.unlink(path)
        except Exception:
            pass
        return os.environ.copy(), None


def ensure_repo() -> Path:
    """拉取或更新 interface_trans_builder 仓库, 返回克隆目录. """
    env, key_path = _git_env_with_key()
    try:
        if CLONE_DIR.is_dir() and (CLONE_DIR / ".git").is_dir():
            try:
                subprocess.run(
                    ["git", "restore", "."],
                    cwd=str(CLONE_DIR),
                    capture_output=True,
                    timeout=30,
                    env=env,
                )
            except Exception:
                try:
                    subprocess.run(
                        ["git", "reset", "--hard", "HEAD"],
                        cwd=str(CLONE_DIR),
                        capture_output=True,
                        timeout=30,
                        env=env,
                    )
                except Exception:
                    pass
            try:
                subprocess.run(
                    ["git", "pull", "--rebase"],
                    cwd=str(CLONE_DIR),
                    capture_output=True,
                    timeout=120,
                    env=env,
                )
            except Exception:
                pass
            return CLONE_DIR
        CLONE_DIR.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            ["git", "clone", REPO_URL, str(CLONE_DIR)],
            check=True,
            timeout=300,
            capture_output=True,
            env=env,
        )
        return CLONE_DIR
    finally:
        if key_path:
            try:
                os.unlink(key_path)
            except Exception:
                pass


# modify build.sh
BUILD_SH_ABSOLUTE_INTERFACE = """    local bare_repo="/interface/interface_bare.git"
    local lock_file="/interface/interface_bare.lock"
    mkdir -p "/interface\""""

BUILD_SH_RELATIVE_INTERFACE = """    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local bare_repo="${script_dir}/interface/interface_bare.git"
    local lock_file="${script_dir}/interface/interface_bare.lock"
    mkdir -p "${script_dir}/interface\""""

BUILD_SH_MAKE_LINE = "    make\n"
BUILD_SH_MAKE_PARALLEL = "    make -j$(nproc)\n"

BUILD_SH_RM_STRUCT = '        rm -rf "$struct_msgs_vxx"\n'
BUILD_SH_RM_STRUCT_AND_MDC = '        rm -rf "$struct_msgs_vxx" "$dst_dir/mdc"\n'

def patch_build_sh(build_sh: Path) -> None:
    """修改 build.sh """
    text = build_sh.read_text(encoding="utf-8", errors="replace")
    changed = False
    if BUILD_SH_ABSOLUTE_INTERFACE in text and BUILD_SH_RELATIVE_INTERFACE not in text:
        text = text.replace(BUILD_SH_ABSOLUTE_INTERFACE, BUILD_SH_RELATIVE_INTERFACE)
        changed = True
    if BUILD_SH_MAKE_LINE in text and BUILD_SH_MAKE_PARALLEL not in text:
        text = text.replace(BUILD_SH_MAKE_LINE, BUILD_SH_MAKE_PARALLEL)
        changed = True
    if BUILD_SH_RM_STRUCT in text and BUILD_SH_RM_STRUCT_AND_MDC not in text:
        text = text.replace(BUILD_SH_RM_STRUCT, BUILD_SH_RM_STRUCT_AND_MDC)
        changed = True
    if changed:
        build_sh.write_text(text, encoding="utf-8")


def collect_bag_paths(args) -> list:
    """从 -i 和 -f 收集所有 bag 的绝对路径并去重"""
    bags = []
    seen_real = set()
    seen_basename = set()

    def add_bag(p: str):
        p = os.path.abspath(os.path.expanduser(p.strip()))
        if not p:
            return
        if p.endswith(".bag") and os.path.isfile(p):
            try:
                real = os.path.realpath(p)
            except Exception:
                real = p
            if real in seen_real:
                return
            base = os.path.basename(p)
            if base in seen_basename:
                return
            seen_real.add(real)
            seen_basename.add(base)
            bags.append(p)
            return
        if os.path.isdir(p):
            for root, _dirs, files in os.walk(p):
                for f in files:
                    if f.endswith(".bag"):
                        fp = os.path.abspath(os.path.join(root, f))
                        if not os.path.isfile(fp):
                            continue
                        try:
                            real = os.path.realpath(fp)
                        except Exception:
                            real = fp
                        if real in seen_real:
                            continue
                        if f in seen_basename:
                            continue
                        seen_real.add(real)
                        seen_basename.add(f)
                        bags.append(fp)

    if args.input:
        for part in args.input.split(","):
            part = part.strip()
            if part:
                add_bag(part)
    if args.file:
        path = os.path.abspath(os.path.expanduser(args.file.strip()))
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    add_bag(line)

    return bags


def build_converter(src_commit: str, dst_commit: str) -> Path:
    """构建转换工具. """
    bin_name = f"rosbag_converter_{src_commit}_{dst_commit}"
    bin_cached = CONVERTERS_DIR / bin_name
    if bin_cached.is_file():
        print(f"使用已有转换工具: {bin_name}", flush=True)
        return bin_cached

    ensure_repo()
    build_dir = CLONE_DIR / "src"
    build_sh = build_dir / "build.sh"
    if not build_sh.is_file():
        raise FileNotFoundError(f"build.sh 不存在: {build_sh}")
    patch_build_sh(build_sh)

    env, key_path = _git_env_with_key()
    try:
        print(f"正在编译转包工具, 耗时较长请等待 ...", flush=True)
        proc = subprocess.run(
            [str(build_sh), src_commit, dst_commit],
            cwd=str(build_dir),
            timeout=1800,
            capture_output=True,
            text=True,
            env=env,
        )
        if proc.returncode != 0:
            if proc.stderr:
                sys.stderr.write(proc.stderr)
            if proc.stdout:
                sys.stderr.write(proc.stdout)
            proc.check_returncode()
    finally:
        if key_path:
            try:
                os.unlink(key_path)
            except Exception:
                pass

    bin_in_repo = build_dir / bin_name
    if not bin_in_repo.is_file():
        raise FileNotFoundError(f"构建产物未找到: {bin_in_repo}")
    CONVERTERS_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(bin_in_repo), str(bin_cached))
    os.chmod(bin_cached, 0o755)
    return bin_cached


def run_convert(bin_path: Path, input_bag: str, output_bag: str) -> None:
    """执行转换：<工具> --input=... --output=..."""
    subprocess.run(
        [str(bin_path), f"--input={input_bag}", f"--output={output_bag}"],
        check=True,
        timeout=30,
    )


class _HelpFormatter(argparse.HelpFormatter):
    """拉长选项列宽度, 对齐说明. """
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("max_help_position", 40)
        super().__init__(*args, **kwargs)


def main():
    parser = argparse.ArgumentParser(
        description="根据 bag 的 commit 构建转换工具并转换到目标 commit",
        formatter_class=_HelpFormatter,
    )
    parser._optionals.title = "options"
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        default=None,
        help="一个或多个 bag 或目录, 逗号分隔",
    )
    parser.add_argument(
        "-f",
        "--file",
        type=str,
        default=None,
        help="文件路径, 每行一个 bag 路径或目录(目录会递归收集其下所有 .bag)",
    )
    parser.add_argument(
        "-t",
        "--target",
        type=str,
        required=True,
        help="目标 commit",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="输出目录, 默认为 output_<目标commit前7位>",
    )
    args = parser.parse_args()

    if not args.input and not args.file:
        parser.error("必须提供 -i 或 -f 之一")

    bags = collect_bag_paths(args)
    if not bags:
        print("未找到任何 bag 文件", file=sys.stderr)
        sys.exit(1)

    print(f"收集到的 bag 路径 (共 {len(bags)} 个):")
    for p in bags:
        print(f"  {p}")

    dst_commit = args.target.strip()
    groups = {}
    no_commit = []

    for b in bags:
        src = get_interface_commitid(b)
        if not src or len(src) < 7:
            no_commit.append(b)
            continue
        groups.setdefault(src, []).append(b)

    if no_commit:
        print("以下 bag 无法读取 commit, 将跳过:", file=sys.stderr)
        for p in no_commit:
            print(f"  {p}", file=sys.stderr)

    if not groups:
        print("没有可转换的 bag (均无有效 commit)", file=sys.stderr)
        sys.exit(1)

    out_dir_name = f"output_{dst_commit[:7]}"
    if args.output:
        out_root = Path(args.output).resolve()
    else:
        out_root = SCRIPT_DIR / out_dir_name
    out_root.mkdir(parents=True, exist_ok=True)
    print(f"输出目录: {out_root}")

    built = {}

    for src_commit, group_bags in groups.items():
        print(f"当前处理的 commit: {src_commit} (共 {len(group_bags)} 个 bag):")
        for p in group_bags:
            print(f"  {p}")
        key = (src_commit, dst_commit)
        if key not in built:
            built[key] = build_converter(src_commit, dst_commit)
        bin_path = built[key]
        for bag_path in group_bags:
            base = os.path.basename(bag_path)
            out_bag = out_root / base
            print(f"当前处理: {bag_path} -> {out_bag}")
            run_convert(bin_path, bag_path, str(out_bag))

    print("全部完成.")


if __name__ == "__main__":
    main()
