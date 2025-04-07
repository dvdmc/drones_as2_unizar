FROM arm64v8/ros:jazzy-ros-base

ARG ROS_VERSION=jazzy

ENV ROS_DISTRO=${ROS_VERSION} \
    ROS_ROOT=/opt/ros/${ROS_VERSION} \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash

SHELL ["/bin/bash", "-c"]

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y  git \
    tmux \
    tmuxinator \
    vim \
    xterm \
    curl \
    ssh \
    tree \
    python3-tk \
    nano \
    libtool \
    libtool-bin \
    htop \
    gdb \
    net-tools \
    build-essential \
    cmake \
    lsb-release \
    iputils-ping 

RUN echo "set -g mouse on" > /root/.tmux.conf

# ROS2 utils

## ROS2 Jazzy utils
RUN apt-get install -y \
    python3-rosdep \
    python3-setuptools \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    ros-dev-tools

RUN apt-get install -y \
    python3-flake8 \
    python3-flake8-builtins \
    python3-flake8-comprehensions \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes

RUN apt-get install -y pylint flake8 pycodestyle
RUN apt-get install cppcheck lcov -y

# For allowing connection between machines, fastdds has some problem.
RUN apt-get install ros-jazzy-rmw-cyclonedds-cpp -y

# Mavlink
RUN apt install ros-jazzy-mavros
RUN source /opt/ros/jazzy/setup.bash && ros2 run mavros install_geographiclib_datasets.sh

# Gazebo (Harmonic)
RUN sudo apt-get install ros-jazzy-ros-gz

# OptiTrack
RUN mkdir -p /root/optitrack_ws/src
WORKDIR /root/optitrack_ws/src
RUN git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4r2
RUN git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs
RUN git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack
WORKDIR /root/optitrack_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --executor sequential

# AS2 TODO: update to binaries. ETA after June 2025
RUN mkdir -p /root/aerostack2_ws/src
RUN git clone -b jazzy-patch-trajgen https://github.com/dvdmc/aerostack2.git /root/aerostack2_ws/src/aerostack2 && \
    git clone -b jazzy-patch-param https://github.com/dvdmc/as2_platform_tello.git /root/aerostack2_ws/src/as2_platform_tello
RUN git clone https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs.git /root/aerostack2_ws/src/mocap4r2_msgs
WORKDIR /root/aerostack2_ws
RUN rosdep update && rosdep install -y -r -q --from-paths src --ignore-src

# AS2 dependencies on Nav2
RUN apt update && apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb* -y

RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

RUN pip3 install colcon-lcov-result cpplint cmakelint PySimpleGUI-4-foss --break-system-packages

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/optitrack_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/aerostack2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/