pipeline {
    agent any
    
    parameters {
        string(name: 'VERSION_JSON', defaultValue: '', description: 'Version JSON content')
    }
    
    environment {
        PLANNING_REPO = 'ssh://git@code.iflytek.com:30004/ZNQC_AutonomousDriving/planning.git'
        INTERFACE_REPO = 'ssh://git@code.iflytek.com:30004/ZNQC_AutonomousDriving/interface.git'
        COMMON_TOOLS_REPO = 'ssh://git@code.iflytek.com:30004/ZNQC_AutonomousDriving/common_tools.git'
        IMAGE_NAME = 'artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/pp-process'
        WORKSPACE_DIR = "planning_build_${BUILD_NUMBER}_${System.currentTimeMillis()}"
    }
    
    stages {
        stage('Create Workspace') {
            steps {
                script {
                    sh """
                        mkdir -p ${env.WORKSPACE_DIR}
                        echo "Created workspace: ${env.WORKSPACE_DIR}"
                        ls -la ${env.WORKSPACE_DIR}
                    """
                }
            }
        }
        
        stage('Parse Version JSON') {
            steps {
                script {
                    def versionData = readJSON text: params.VERSION_JSON
                    env.PLANNING_COMMIT = versionData.modules.planning.commitid
                    env.INTERFACE_COMMIT = versionData.modules.interface.commitid
                    env.SYSTEM_VERSION = versionData.system_version
                    
                    // Convert system_version dots to underscores
                    env.SYSTEM_VERSION_TAG = env.SYSTEM_VERSION.replace('.', '_')
                    
                    // Set image tag
                    env.IMAGE_TAG = env.SYSTEM_VERSION_TAG
                    
                    echo "Planning commit: ${env.PLANNING_COMMIT}"
                    echo "Interface commit: ${env.INTERFACE_COMMIT}"
                    echo "System version: ${env.SYSTEM_VERSION}"
                    echo "Image tag: ${env.IMAGE_TAG}"
                    echo "Workspace: ${env.WORKSPACE_DIR}"
                }
            }
        }
        
        stage('Clone Planning Repository') {
            steps {
                script {
                    sh """
                        cd ${env.WORKSPACE_DIR}
                        echo "Planning repository: ${env.PLANNING_REPO}"
                        echo "Target commit: ${env.PLANNING_COMMIT}"
                    """
                    
                    checkout([
                        $class: 'GitSCM',
                        branches: [[name: env.PLANNING_COMMIT]],
                        userRemoteConfigs: [[
                            url: env.PLANNING_REPO
                        ]],
                        extensions: [
                            [
                                $class: 'RelativeTargetDirectory',
                                relativeTargetDir: "${env.WORKSPACE_DIR}/planning"
                            ]
                        ]
                    ])
                    
                    sh """
                        cd ${env.WORKSPACE_DIR}/planning
                        echo "Planning repository cloned successfully"
                        git log --oneline -1
                        echo "Current directory: \$(pwd)"
                        ls -la
                    """
                }
            }
        }
        
        stage('Download Submodules') {
            steps {
                script {
                    sh """
                        cd ${env.WORKSPACE_DIR}/planning
                        git submodule update --init --recursive
                    """
                }
            }
        }
        
        stage('Clone Interface Repository') {
            steps {
                script {
                    sh """
                        echo "Cloning interface repository..."
                        echo "Interface repository: ${env.INTERFACE_REPO}"
                        echo "Target commit: ${env.INTERFACE_COMMIT}"
                        echo "Workspace directory: ${env.WORKSPACE_DIR}"
                        echo "Current directory: \$(pwd)"
                    """
                    
                    checkout([
                        $class: 'GitSCM',
                        branches: [[name: env.INTERFACE_COMMIT]],
                        userRemoteConfigs: [[
                            url: env.INTERFACE_REPO
                        ]],
                        extensions: [
                            [
                                $class: 'RelativeTargetDirectory',
                                relativeTargetDir: "${env.WORKSPACE_DIR}/interface"
                            ]
                        ]
                    ])
                    
                    sh """
                        echo "Interface repository cloned successfully"
                        echo "Current directory after checkout: \$(pwd)"
                        echo "Listing workspace contents:"
                        ls -la ${env.WORKSPACE_DIR}/
                        
                        echo "Listing interface directory:"
                        ls -la ${env.WORKSPACE_DIR}/interface/
                        
                        echo "Removing existing interface submodule from planning..."
                        rm -rf ${env.WORKSPACE_DIR}/planning/interface
                        echo "Existing interface submodule removed"
                        
                        echo "Copying interface to planning directory..."
                        cp -r ${env.WORKSPACE_DIR}/interface ${env.WORKSPACE_DIR}/planning/interface
                        echo "Interface copied to planning directory"
                        
                        echo "Verifying copy:"
                        ls -la ${env.WORKSPACE_DIR}/planning/
                    """
                }
            }
        }
        
        stage('Clone Common Tools Repository') {
            steps {
                script {
                    sh """
                        echo "Cloning common_tools repository..."
                        echo "Common tools repository: ${env.COMMON_TOOLS_REPO}"
                        echo "Target branch: develop"
                        echo "Workspace directory: ${env.WORKSPACE_DIR}"
                        echo "Current directory: \$(pwd)"
                    """
                    
                    checkout([
                        $class: 'GitSCM',
                        branches: [[name: 'develop']],
                        userRemoteConfigs: [[
                            url: env.COMMON_TOOLS_REPO
                        ]],
                        extensions: [
                            [
                                $class: 'RelativeTargetDirectory',
                                relativeTargetDir: "${env.WORKSPACE_DIR}/common_tools"
                            ]
                        ]
                    ])
                    
                    sh """
                        echo "Common tools repository cloned successfully"
                        echo "Current directory after checkout: \$(pwd)"
                        echo "Listing common_tools directory:"
                        ls -la ${env.WORKSPACE_DIR}/common_tools/
                        
                        echo "Copying common_tools to planning directory..."
                        cp -r ${env.WORKSPACE_DIR}/common_tools ${env.WORKSPACE_DIR}/planning/common_tools
                        echo "Common tools copied to planning directory"
                        
                        echo "Verifying copy:"
                        ls -la ${env.WORKSPACE_DIR}/planning/
                    """
                }
            }
        }
        
        stage('Build Docker Image') {
            steps {
                script {
                    sh """
                        cd ${env.WORKSPACE_DIR}/planning
                        
                        echo "Creating multi-stage Dockerfile..."
                        cat > Dockerfile << 'EOF'
# Stage 1: Build inner image with source code
FROM artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/simulation_for_ci:V11 AS inner

ARG ENABLE_STATIC_FUSION=0

WORKDIR /root/planning

ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib:\$LD_LIBRARY_PATH \\
    CMAKE_PREFIX_PATH=/opt/ros/melodic:\$CMAKE_PREFIX_PATH

RUN /root/miniconda3/bin/pip install empy==3.3.4 shapely boto3 && \\
    echo 'source /opt/ros/melodic/setup.sh' >> ~/.bashrc && \\
    mkdir -p /tmp

COPY ./CMakeLists.txt ./Makefile /root/planning/
COPY ./.ci /root/planning/.ci
COPY ./thirdparty /root/planning/thirdparty/
RUN bash -c "make clean && make thirdparty_build BUILD_TYPE=Release"
COPY ./interface /root/planning/interface/
RUN bash -c "source /opt/ros/melodic/setup.sh && make interface_build BUILD_TYPE=Release"
COPY ./ad_common /root/planning/ad_common/
RUN bash -c "source /opt/ros/melodic/setup.sh && make ad_build BUILD_TYPE=Release"

COPY . /root/planning/
COPY ./common_tools /root/common_tools
RUN bash -c "source /opt/ros/melodic/setup.sh && make pp_build NUM_JOB=32 BUILD_TYPE=Release && rm -rf /root/planning/build/"

RUN if [ "\$ENABLE_STATIC_FUSION" -eq 1 ]; then \\
        bash -c "cd ./system_integration/components/static_fusion/core/ && chmod 777 ./target/build.sh && WORKSPACE='/root/planning/system_integration' ./target/build.sh simulate && chmod 777 ./install/staticFusion/bin/write_fusionroad_rosbag"; \\
    fi

# Create CI_entrypoint_pp_open.sh only if it doesn't exist in git repo
RUN if [ ! -f "/root/planning/CI_entrypoint_pp_open.sh" ]; then \\
        echo "Creating CI_entrypoint_pp_open.sh (not found in git repo)" && \\
        echo '#!/bin/bash' > /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '# conda initialize -------------------------------' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '# !! Contents within this block are managed by "conda init" !!' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '__conda_setup="\$(\"/root/miniconda3/bin/conda\" \"shell.bash\" \"hook\" 2> /dev/null)"' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo 'if [ \$? -eq 0 ]; then' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '    eval "\$__conda_setup"' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo 'else' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '    if [ -f "/root/miniconda3/etc/profile.d/conda.sh" ]; then' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '        . "/root/miniconda3/etc/profile.d/conda.sh"' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '    else' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '        export PATH="/root/miniconda3/bin:\$PATH"' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '    fi' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo 'fi' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo 'unset __conda_setup' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '# ros env' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo 'source /opt/ros/melodic/setup.sh' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        echo '/root/miniconda3/bin/python /root/planning/simulation/simulation_open.py "\$@"' >> /root/planning/CI_entrypoint_pp_open.sh && \\
        chmod +x /root/planning/CI_entrypoint_pp_open.sh; \\
    else \\
        echo "Using existing CI_entrypoint_pp_open.sh from git repo" && \\
        chmod +x /root/planning/CI_entrypoint_pp_open.sh; \\
    fi

# Create simulation_open.py only if it doesn't exist in git repo
RUN mkdir -p /root/planning/simulation && \\
    if [ ! -f "/root/planning/simulation/simulation_open.py" ]; then \\
        echo "Creating simulation_open.py (not found in git repo)" && \\
        echo 'import os' > /root/planning/simulation/simulation_open.py && \\
        echo 'import sys' >> /root/planning/simulation/simulation_open.py && \\
        echo 'import subprocess' >> /root/planning/simulation/simulation_open.py && \\
        echo 'import json' >> /root/planning/simulation/simulation_open.py && \\
        echo 'import time' >> /root/planning/simulation/simulation_open.py && \\
        echo 'from multiprocessing import Process' >> /root/planning/simulation/simulation_open.py && \\
        echo 'from pathlib import Path' >> /root/planning/simulation/simulation_open.py && \\
        echo '' >> /root/planning/simulation/simulation_open.py && \\
        echo 'def simulation_open(file_path):' >> /root/planning/simulation/simulation_open.py && \\
        echo '    dir_path = os.path.dirname(file_path)' >> /root/planning/simulation/simulation_open.py && \\
        echo '' >> /root/planning/simulation/simulation_open.py && \\
        echo '    time_fold = str(int(time.time()))' >> /root/planning/simulation/simulation_open.py && \\
        echo '    out_bag_path = file_path + "." + time_fold + ".default_open_loop.plan"' >> /root/planning/simulation/simulation_open.py && \\
        echo '    command = f"/root/planning/install/bin/pp --play {file_path} --out-bag {out_bag_path} --interface-check --no-version-check"' >> /root/planning/simulation/simulation_open.py && \\
        echo '    try:' >> /root/planning/simulation/simulation_open.py && \\
        echo '        subprocess.run(command, shell=True, text=True, check=True)' >> /root/planning/simulation/simulation_open.py && \\
        echo '    except subprocess.CalledProcessError as e:' >> /root/planning/simulation/simulation_open.py && \\
        echo '        print(f"Runing PP error: {e}")' >> /root/planning/simulation/simulation_open.py && \\
        echo '        return None' >> /root/planning/simulation/simulation_open.py && \\
        echo '    ' >> /root/planning/simulation/simulation_open.py && \\
        echo '    for item in os.listdir(dir_path):' >> /root/planning/simulation/simulation_open.py && \\
        echo '        item_path = os.path.join(dir_path, item)' >> /root/planning/simulation/simulation_open.py && \\
        echo '        if os.path.isfile(item_path):' >> /root/planning/simulation/simulation_open.py && \\
        echo '            if time_fold in item:' >> /root/planning/simulation/simulation_open.py && \\
        echo '                 return item_path' >> /root/planning/simulation/simulation_open.py && \\
        echo '    print(f"out pp path no found")    # return None' >> /root/planning/simulation/simulation_open.py && \\
        echo '    return None' >> /root/planning/simulation/simulation_open.py && \\
        echo 'if __name__ == "__main__":' >> /root/planning/simulation/simulation_open.py && \\
        echo '    if len(sys.argv) > 1:' >> /root/planning/simulation/simulation_open.py && \\
        echo '        file_path = sys.argv[1]' >> /root/planning/simulation/simulation_open.py && \\
        echo '    else:' >> /root/planning/simulation/simulation_open.py && \\
        echo '        file_path = "/share/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20250905/20250905-15-47-25/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-09-05-15-47-25_no_camera.bag"' >> /root/planning/simulation/simulation_open.py && \\
        echo '    simulation_open(file_path)' >> /root/planning/simulation/simulation_open.py; \\
    else \\
        echo "Using existing simulation_open.py from git repo"; \\
    fi

# Verify files were created
RUN echo "Created files:" && \\
    ls -la /root/planning/CI_entrypoint_pp_open.sh && \\
    ls -la /root/planning/simulation/simulation_open.py

# Stage 2: Build outer image based on inner, remove source code
FROM inner AS outer

# First, copy all necessary files from inner stage
COPY --from=inner /root/planning/install /root/planning/install
COPY --from=inner /root/planning/simulation /root/planning/simulation
COPY --from=inner /root/planning/CI_entrypoint_pp_open.sh /root/planning/CI_entrypoint_pp_open.sh
COPY --from=inner /root/planning/CI_entrypoint_simulation.sh /root/planning/CI_entrypoint_simulation.sh
COPY --from=inner /root/planning/CI_entrypoint.sh /root/planning/CI_entrypoint.sh

# Verify files were copied from inner stage
RUN echo "Verifying files copied from inner stage:" && \\
    ls -la /root/planning/CI_entrypoint_pp_open.sh && \\
    ls -la /root/planning/simulation/simulation_open.py && \\
    echo "Files copied from inner stage successfully"

# Copy necessary runtime files before removing source code
RUN echo "Checking files before copy:" && \\
    ls -la /root/planning/CI_entrypoint_pp_open.sh && \\
    ls -la /root/planning/simulation/simulation_open.py && \\
    echo "Creating runtime directory:" && \\
    mkdir -p /root/runtime && \\
    echo "Copying install directory:" && \\
    cp -r /root/planning/install /root/runtime/ && \\
    echo "Copying simulation directory:" && \\
    cp -r /root/planning/simulation /root/runtime/ && \\
    echo "Copying CI_entrypoint_pp_open.sh:" && \\
    cp /root/planning/CI_entrypoint_pp_open.sh /root/runtime/ && \\
    echo "Files copied successfully:" && \\
    ls -la /root/runtime/ && \\
    echo "Verifying CI_entrypoint_pp_open.sh exists:" && \\
    ls -la /root/runtime/CI_entrypoint_pp_open.sh && \\
    echo "Verifying simulation_open.py exists:" && \\
    ls -la /root/runtime/simulation/simulation_open.py && \\
    echo "Setting permissions on copied files:" && \\
    chmod +x /root/runtime/CI_entrypoint_pp_open.sh && \\
    echo "Final check - all files should exist:" && \\
    test -f /root/runtime/CI_entrypoint_pp_open.sh && echo "CI_entrypoint_pp_open.sh exists" || echo "ERROR: CI_entrypoint_pp_open.sh missing" && \\
    test -f /root/runtime/simulation/simulation_open.py && echo "simulation_open.py exists" || echo "ERROR: simulation_open.py missing"

# Remove source code directories
RUN rm -rf /root/planning /root/common_tools

# Copy /root/runtime contents to /root/planning to maintain original path structure
RUN echo "Before copy - checking /root/runtime contents:" && \\
    ls -la /root/runtime/ && \\
    echo "Creating /root/planning directory:" && \\
    mkdir -p /root/planning && \\
    echo "Copying contents from /root/runtime to /root/planning:" && \\
    cp -r /root/runtime/* /root/planning/ && \\
    echo "After copy - checking /root/planning contents:" && \\
    ls -la /root/planning/ && \\
    echo "Verifying files exist:" && \\
    ls -la /root/planning/CI_entrypoint_pp_open.sh && \\
    ls -la /root/planning/simulation/simulation_open.py && \\
    echo "Setting executable permissions:" && \\
    chmod +x /root/planning/CI_entrypoint_pp_open.sh && \\
    echo "Final verification - listing /root/planning contents:" && \\
    ls -la /root/planning/ && \\
    echo "Checking file permissions:" && \\
    ls -la /root/planning/CI_entrypoint_pp_open.sh && \\
    echo "Cleaning up /root/runtime:" && \\
    rm -rf /root/runtime

# Final verification and permission setting
RUN echo "Final verification - files should exist now:" && \\
    ls -la /root/planning/CI_entrypoint_pp_open.sh && \\
    ls -la /root/planning/simulation/simulation_open.py && \\
    chmod +x /root/planning/CI_entrypoint_pp_open.sh && \\
    echo "All files verified and permissions set"

# Set entrypoint to use CI_entrypoint_pp_open.sh
ENTRYPOINT ["/bin/bash", "/root/planning/CI_entrypoint_pp_open.sh"]
EOF
                        
                        echo "Multi-stage Dockerfile created successfully"
                        echo "Building Docker image..."
                        echo "Current directory: \$(pwd)"
                        echo "Listing planning directory contents:"
                        ls -la
                        
                        # Build the multi-stage image
                        docker build --target outer -t ${env.IMAGE_NAME}:${env.IMAGE_TAG} .
                    """
                }
            }
        }
        
        stage('Push Image') {
            steps {
                script {
                    sh """
                        docker push ${env.IMAGE_NAME}:${env.IMAGE_TAG}
                    """
                }
            }
        }
    }
    
    post {
        always {
            script {
                sh """
                    # Clean up workspace
                    echo "Cleaning up workspace: ${env.WORKSPACE_DIR}"
                    rm -rf ${env.WORKSPACE_DIR}
                    echo "Workspace cleanup completed"
                """
            }
        }
        success {
            echo "Pipeline completed successfully!"
            echo "Image: ${env.IMAGE_NAME}:${env.IMAGE_TAG}"
            echo "Usage: docker run --privileged ${env.IMAGE_NAME}:${env.IMAGE_TAG} a.bag"
            echo "Workspace ${env.WORKSPACE_DIR} has been cleaned up"
        }
        failure {
            echo "Pipeline failed!"
            echo "Cleaning up workspace ${env.WORKSPACE_DIR} due to failure"
        }
    }
}
