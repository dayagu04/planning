#!/usr/bin/python3
# -*- encoding: utf-8 -*-

# start/stop/attach dev containers

import subprocess
import getpass
import argparse
import pathlib

PLATFORM = ""

def get_user_name():
    return getpass.getuser()

def get_container_name(image):
    return image.split('/')[-1].replace(':', '_') + '_' + get_user_name()

def run(command):
    print(command)
    subprocess.run(command, shell=True, universal_newlines=True)#, stdout=subprocess.PIPE)#, stderr=subprocess.PIPE)

def check_output(command, shell=True):
    return subprocess.check_output(command, shell=shell).decode().strip()

def start_container(image, container_name, volumes, envs, root):
    command = f'docker run -tid --rm --privileged --network host --name {container_name}'

    if not root:
        uid = check_output(['id', '-u'], shell=False)
        gid = check_output(['id', '-g'], shell=False)
        command += f' --user {uid}:{gid}'
        user_name = get_user_name()
        command += f' -w /home/{user_name}'
    else:
        command += f' -w /root'

    if envs:
        for env in envs:
            if env[1]:
                command += ' -e {0}={1}'.format(env[0], env[1])
            else:
                command += ' -e {0}'.format(env[0])

    for volume in volumes:
        if not pathlib.Path(volume[0]).is_file():
            pathlib.Path(volume[0]).mkdir(parents=True, exist_ok=True)
        command += ' -v {0}:{1}'.format(volume[0], volume[1])

    command += f' {image} /bin/bash'
    run(command)

def stop_container(container_name):
    command = f'docker stop {container_name}'
    run(command)

def attach_container(container_name, cmd):
    user_name = get_user_name()
    command = f'docker exec -it -w /home/{user_name} {container_name} /bin/bash -c "{cmd};zsh"'
    run(command)

def container_running(container_name):
    try:
        command = f"docker inspect --format {{{{.State.Running}}}} {container_name}"
        if check_output(command) == 'true':
            return True
    except:
        print(f'container not running: {container_name}')
        return False

def get_image_name(platform):
    global PLATFORM

    if platform == 'BZT_2331':
        PLATFORM = 'BZT'
        return 'artifacts.iflytek.com/auto-docker-private/autodriver_bsw/bsw_dev/a1000b-sdk-fad:2.3.3.1'
    if platform == 'BZT':
        PLATFORM = 'BZT'
        return 'a1000b-sdk-fad:2.3.0.4'
    if platform == 'X86_2331':
        PLATFORM = 'X86'
        return 'artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/ros_melodic:v0.2.4_cyber'
    if platform == 'X86':
        PLATFORM = 'X86'
        return 'pnc_x86_pybind_v0.1.0:latest'
    if platform == 'ROS':
        PLATFORM = 'X86'
        return 'artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/ros_melodic:v0.2.3_dev'
    exit(f'platform no supported: {platform}')
    return ''

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--stop', action='store_true', default=False, help='stop constainer')
    parser.add_argument('--root', action='store_true', default=False, help='use root in container')
    parser.add_argument('--platform', type=str, default='X86', help='supported platform: X86 BZT')
    args = parser.parse_args()

    image_name = get_image_name(args.platform)
    container_name = get_container_name(image_name)

    if args.stop:
        stop_container(container_name)
    else:
        if (not container_running(container_name)):
            user_name = get_user_name()
            start_container(
                image=image_name,
                container_name=container_name,
                volumes=[
                    (f'/home/{user_name}', f'/home/{user_name}'),
                    (f'/home/{user_name}/.vscode-server-containers', f'/home/{user_name}/.vscode-server'),
                    ('/etc/passwd', '/etc/passwd:ro'),
                    ('/etc/group', '/etc/group:ro'),
                    ('/mnt', '/mnt'),
                    ('/data_cold', '/data_cold'),
                ],
                envs=[
                    ('PLATFORM', PLATFORM),
                    ('MAVEN_REPOSITORY_PATH', f'/home/{user_name}/.m2/repository'),
                ],
                root=args.root
            )
            subprocess.check_call(['docker', 'exec', '-u', 'root', '-i', container_name, 'bash', '-c', f'echo \'{user_name} ALL=(ALL) NOPASSWD: ALL\' >> /etc/sudoers'])
            subprocess.check_call(['docker', 'exec', '-u', 'root', '-i', container_name, 'bash', '-c', f'echo "123456" | su -c "apt update; apt install -y maven p7zip-full zsh curl; mkdir /asw; chmod 777 /asw"'])
        
        command = f'git config --global --add safe.directory \'*\''
        print(PLATFORM)
        if PLATFORM == 'BZT':
            command += f';unset LD_LIBRARY_PATH;source /opt/bstos/*/environment-setup-aarch64-bst-linux 2>/dev/null'
        if PLATFORM == 'X86':
            command += f';source /opt/ros/melodic/setup.zsh; source /usr/bin/cyber/setup.bash 2>/dev/null'
        command += ';which cmake'
        attach_container(container_name, command)
