#!/usr/bin/env python2

import os
import subprocess
import argparse

clang_format_path = ".ci/clang-format/wrapper"

def error(msg=""):
  print "error: %s" % msg
  exit(1)


def get_root_path(anchor=".clang-format"):
  path = os.path.abspath(__file__)
  while True:
    path = os.path.dirname(path)
    if (os.path.exists(path + "/" + anchor)):
      return path
    if (path == "/"):
      error("%s not found" % anchor)


def _run_cpp(sub_folders,
             file_types=["*.cpp", "*.c", "*.hpp", "*.h", "*.cc"],
             format_file=".clang-format"):
  assert isinstance(file_types, list)
  assert format_file.strip()
  root = get_root_path(format_file)
  print "run in [%s] with [%s]" % (", ".join(sub_folders),
                                     ", ".join(file_types))
  for folder in sub_folders:
    for file_type in file_types:
      cmd = "find %s/%s -path %s/src/thirdparty -prune -o -name %s | xargs %s -i 2>/dev/null" % (
        root, folder, root, file_type, clang_format_path)
      os.system(cmd)


def run():
  sub_folders = [
    "src",
    "tools",
    "test",
  ]
  _run_cpp(sub_folders)


def _check_cpp(sub_folders,
               file_types=["*.cpp", "*.c", "*.hpp", "*.h"],
               format_file=".clang-format"):
  assert isinstance(file_types, list)
  assert format_file.strip()
  root = get_root_path(format_file)
  print "check in [%s] with [%s]" % (", ".join(sub_folders),
                                     ", ".join(file_types))
  for folder in sub_folders:
    for file_type in file_types:
      try:
        cmd = "find %s/%s -path %s/src/thirdparty -prune -o -name %s | xargs %s -output-replacements-xml | grep -c '<replacement '" % (
          root, folder, root, file_type, clang_format_path)
        result = subprocess.check_output(cmd, shell=True)
        error("not all %s in %s/%s is formatted" % (file_type, root, folder))
      except Exception as e:
        continue


def check():
  sub_folders = [
    "src",
  ]
  _check_cpp(sub_folders)


if __name__ == "__main__":
  print "clang-format path %s" % (clang_format_path)
  os.system(clang_format_path + " --version")

  parser = argparse.ArgumentParser()
  parser.add_argument("action",
                      choices=["check", "run"],
                      nargs="?",
                      default="run",
                      help="The actions")
  args = parser.parse_args()
  if (args.action == "run"):
    run()
  elif (args.action == "check"):
    check()
  exit(0)
