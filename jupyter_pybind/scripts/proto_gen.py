import sys
import os
import shutil

def proto_gen_py():
    if os.path.exists("../python_proto"):
        print(f"python_protos already exist")
        shutil.rmtree("../python_proto")

    os.mkdir("../python_proto")
    current_dir = os.getcwd()
    new_dir = os.path.join(current_dir, "../python_proto")
    os.chdir("../../proto")
    proto_lists = os.listdir(".")

    for i in range(len(proto_lists)):
        if ".proto" in proto_lists[i]:
          command = f"protoc --python_out={new_dir} {proto_lists[i]}"
          os.system(command)
    # add ehr_sdmap.proto
    os.chdir(current_dir)
    if os.path.exists("../../interface"):
      if "ehr_sdmap.proto" in os.listdir("../../interface/src/proto/"):
        os.chdir("../../interface/src/proto/")
        proto_lists.append("ehr_sdmap.proto")
        command = f"protoc --python_out={new_dir} {proto_lists[-1]}"
        os.system(command)
    elif os.path.exists("../../../interface"):
      if "ehr_sdmap.proto" in os.listdir("../../../interface/src/proto/"):
        os.chdir("../../../interface/src/proto/")
        proto_lists.append("ehr_sdmap.proto")
        command = f"protoc --python_out={new_dir} {proto_lists[-1]}"
        os.system(command)
        proto_lists.append("ehr_sdmap.proto")
    # add common.proto
    os.chdir(current_dir)
    if os.path.exists("../../interface"):
      if "common.proto" in os.listdir("../../interface/src/proto/"):
        os.chdir("../../interface/src/proto/")
        proto_lists.append("common.proto")
        command = f"protoc --python_out={new_dir} {proto_lists[-1]}"
        os.system(command)
    elif os.path.exists("../../../interface"):
      if "common.proto" in os.listdir("../../../interface/src/proto/"):
        os.chdir("../../../interface/src/proto/")
        proto_lists.append("common.proto")
        command = f"protoc --python_out={new_dir} {proto_lists[-1]}"
        os.system(command)
        proto_lists.append("common.proto")
    return 0

if __name__ == "__main__":

    proto_gen_py()
