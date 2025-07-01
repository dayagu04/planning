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
    # add ehr_sdmap.proto\common.proto
    os.chdir(current_dir)
    if os.path.exists("../../interface"):
      if "ehr_sdmap.proto" and  "common.proto" in os.listdir("../../interface/src/proto/"):
        os.chdir("../../interface/src/proto/")
        command = f"protoc --python_out={new_dir} {'ehr_sdmap.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'common.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'ehr.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'ifly_parking_map.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'map_data.proto'}"
        os.system(command)
    elif os.path.exists("../../../interface"):
      if "ehr_sdmap.proto" in os.listdir("../../../interface/src/proto/"):
        os.chdir("../../../interface/src/proto/")
        command = f"protoc --python_out={new_dir} {'ehr_sdmap.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'common.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'ehr.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'ifly_parking_map.proto'}"
        os.system(command)
        command = f"protoc --python_out={new_dir} {'map_data.proto'}"
        os.system(command)
    return 0

if __name__ == "__main__":

    proto_gen_py()
