FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
ARG yarp_branch="master"
ARG gazebo_version=harmonic
ARG NJOBS=4



# Default value in case it's not provided
ARG USERNAME=user1
ARG ros_distro=iron

# Set the locale
RUN apt update && \
    apt install -y -qq locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt update && \
    apt install --no-install-recommends -y -qq apt-utils build-essential ca-certificates cmake cmake-curses-gui curl \ 
    git glmark2 gnupg2 htop iputils-ping jq lsb-release mesa-utils nano psmisc python3-virtualenv sudo unzip vim wget zip libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*

# Install additional dependencies for the robotology superbuild
RUN apt update && \
    git clone https://github.com/robotology/robotology-superbuild && \ 
    cd robotology-superbuild && \
    bash scripts/install_apt_dependencies.sh && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf ../robotology-superbuild

# ROS2 Installation
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y && apt-get install -y ros-${ros_distro}-desktop ros-${ros_distro}-navigation2 ros-${ros_distro}-nav2-bringup \
    ros-${ros_distro}-slam-toolbox ros-${ros_distro}-test-msgs ${gazebo_pkg} ros-${ros_distro}-rmw-cyclonedds-cpp\
    python3-colcon-common-extensions python3-flake8 python3-flake8-blind-except python3-flake8-builtins \
    python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings \
    python3-flake8-import-order python3-flake8-quotes python3-pytest python3-pytest-cov python3-pytest-repeat \
    python3-pytest-rerunfailures python3-rosdep python3-vcstool python3-venv && rosdep init && rm -rf /var/lib/apt/lists/*

# Create user: user1
USER root
RUN useradd -l -u 33334 -G sudo -md /home/${USERNAME} -s /bin/bash -p ${USERNAME} ${USERNAME} && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ROS2 environment setup
RUN rosdep fix-permissions && rosdep update


# Use /bin/bash instead of /bin/sh
SHELL ["/bin/bash", "-c"]

# Build robotology-superbuild
RUN source /opt/ros/${ros_distro}/setup.bash && \
    git clone https://github.com/robotology/robotology-superbuild && \
    cd robotology-superbuild && \
    mkdir build && cd build && \
    cmake -DROBOTOLOGY_ENABLE_CORE=ON -DROBOTOLOGY_ENABLE_DYNAMICS=ON -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON -DROBOTOLOGY_USES_ROS2=ON -DROBOTOLOGY_USES_GAZEBO=ON -DROBOTOLOGY_USES_PYTHON=ON -DROBOTOLOGY_USES_GZ=OFF -DYCM_EP_ADDITIONAL_CMAKE_ARGS:STRING="-DICUB_COMPILE_BINDINGS:BOOL=ON -DENABLE_yarpmod_rgbdSensor_nws_yarp:BOOL=ON -DENABLE_yarpmod_RGBDSensorClient:BOOL=ON -DENABLE_yarpcar_mjpeg:BOOL=ON -DENABLE_yarppm_depthimage_to_rgb:BOOL=ON -DENABLE_yarppm_depthimage_compression_zlib:BOOL=ON" ../ && \
    make -j$NJOBS

RUN echo "source /opt/ros/${ros_distro}/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/${USERNAME}/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/${USERNAME}/.bashrc

# Build ergocub-cartesian-control
RUN git clone https://github.com/hsp-iit/ergocub-cartesian-control.git && \
    cd ergocub-cartesian-control && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/home/${USERNAME}/robotology-superbuild/build/install .. && \
    make -j$NJOBS && make install

# Install r1-models
RUN git clone https://github.com/icub-tech-iit/r1-models && \
    cd r1-models && cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/home/${USERNAME}/robotology-superbuild/build/install && \
    cmake --build build && cmake --build build --target install

ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/${USERNAME}/robotology-superbuild/build/install/share/R1Mk3/robots
ENV ROS_DOMAIN_ID=78
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV YARP_ROBOT_NAME=R1Mk3Gazebo
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:/home/${USERNAME}/robotology/superbuild/build/install/share/R1Mk3
ENV YARP_CLOCK=/clock