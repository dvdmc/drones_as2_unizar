FROM osrf/ros:jazzy-desktop-full

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
    lsb-release

RUN echo "set -g mouse on" > /root/.tmux.conf

## ROS2 Jazzy utils
RUN apt-get install -y \
    python3-rosdep \
    python3-pip \
    python3-setuptools \
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

RUN apt-get install cppcheck lcov -y

# For allowing connection between machines, fastdds has some problem.
RUN apt-get install ros-jazzy-rmw-cyclonedds-cpp -y

# Gazebo (Harmonic)
RUN sudo apt-get install ros-jazzy-ros-gz

# AS2 TODO: update to binaries. ETA after June 2025
RUN mkdir -p /root/aerostack2_ws/src
RUN git clone -b jazzy https://github.com/dvdmc/aerostack2.git /root/aerostack2_ws/src/aerostack2
RUN git clone https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs.git /root/aerostack2_ws/src/mocap4r2_msgs
WORKDIR /root/aerostack2_ws
# RUN rosdep update && rosdep install -y -r -q --from-paths src --ignore-src
# RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# # Fingers crossed for the one below
# RUN apt-get install ros-humble-as2-platform-tello -y
# RUN colcon mixin update default
# RUN rm -rf log # remove log folder

# RUN pip3 install colcon-lcov-result cpplint cmakelint
# RUN pip3 install PySimpleGUI-4-foss

# # OptiTrack
# RUN mkdir -p /root/optitrack_ws/src
# WORKDIR /root/optitrack_ws/src
# RUN git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4r2
# RUN git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs
# RUN git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack

# WORKDIR /root/optitrack_ws
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --executor sequential

# RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
# RUN echo "source /root/optitrack_ws/install/setup.bash" >> /root/.bashrc
# RUN echo "source /root/aerostack2_ws/install/setup.bash" >> /root/.bashrc

# RUN rm -rf log # remove log folder
# WORKDIR /root/