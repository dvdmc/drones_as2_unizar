# Drones AS2 Unizar

This repository is intended to be an entry point to anyone working with [Aerostack2](https://aerostack2.github.io/) in Unizar. The structure and code is largely based on the AS2's `project_*` file and those should be the primary reference. This repository currently contains unified code to run:

- Gazebo simulation of drones

- Tello drones

It can be used as a template to create more complex projects.

## Installation

This project can be cloned into a workspace where Aerostack2 and Gazebo are assumed to be installed.

```bash
git clone ...
```

### Docker installation

We provide a Dockerfile to run the system within Docker. This allows for easy install of all the dependencies. Also, this Dockerfile works as a reference of anything that should be done in a host machine in order to have the system ready. You can run the Docker with:

```bash
xhost + # To allow the Docker use the host graphical interface
sudo docker compose up -d # To run the docker-compose.yml and keep it running in the background. It will compile the container the first time you run it.
sudo docker exec -it drones_as2_unizar /bin/bash # To attach a terminal to the Docker.
```

By default, this will mount the current directory as a package in a worskpace. If you want to mount your own workspace 

## Usage 

There are currently two execution modes in the repository. One uses Gazebo and the other the Tello platform. For the sake of simplicity, we will use the terms applied to Gazebo but scripts and config files will usually be specific to either the **Gazebo** or the **Tello** version by either adding a suffix `_[gazebo/tello]` or being in a subfolder `gazebo/` or `tello/`.

### 1. Launch aerostack2 nodes for each drone

To launch aerostack2 nodes for each drone, execute once the following command:
```bash
./launch_as2_gazebo.bash
```

The flags for the components launcher (different flags apply to the tello version) are:

- **-m**: multi agent. Default not set
- **-n**: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file
- **-s**: if set, the simulation will not be launched. Default launch simulation
- **-g**: launch using gnome-terminal instead of tmux. Default not set

### 2. Launch aerostack2 nodes for the ground station

To launch aerostack2 nodes for the ground station, execute once the following command:
```bash
./launch_ground_station.bash
```

The flags for the components launcher are:

- -**m**: multi agent. Default not set
- -**t**: launch keyboard teleoperation. Default not launch
- -**v**: open rviz. Default not launch
- -**r**: record rosbag. Default not launch
- -**n**: drone namespaces, comma separated. Default get from world description config file
- -**g**: launch using gnome-terminal instead of tmux. Default not set

### 3. Launch a mission
There are several missions that can be executed:

- **AS2 keyboard teleoperation control**: You can use the keyboard teleoperation launched with the ground station, using the flag `-t`:
  ```bash
  ./launch_ground_station.bash -t
  ```
  You can launch a **swarm of drones** with the flag `-m` and control them with the keyboard teleoperation, as:
  ```bash
  ./launch_as2.bash -m
  ```
  ```bash
  ./launch_ground_station.bash -m -t
  ```
- **AS2 Python API single drone mission**: You can execute a mission that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission.py
  ```
- **AS2 Python API single drone mission using GPS**: You can execute a mission that used AS2 Python API with GPS, launching the mission with:
  ```bash
  python3 mission_gps.py
  ```
- **AS2 Python API swarm of drones mission**: You can execute a mission with a swarm of drones that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission_swarm.py
  ```
  You must launch a **swarm of drones** with the flag `-m`, as:
  ```bash
  ./launch_as2.bash -m
  ```
  ```bash
  ./launch_ground_station.bash -m
  ```
- **AS2 Mission Interpreter single drone mission**: You can execute a mission that used AS2 Mission Interpreter, launching the mission with:
  ```bash
  python3 mission_interpreter.py
  ```
- **AS2 Behavior Trees single drone mission**: You can execute a mission that used AS2 Behavior Trees, launching the mission with:
  ```bash
  python3 mission_behavior_tree.py

### 4. End the execution

If you are using tmux, you can end the execution with the following command:

```bash
./stop.bash
```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.

## Developers guide

### Development environment

In order to modify and persist code outside of the Docker, you have to mount an outside directory as a workspace. You can run:

```bash
./create_workspace.sh
```

And modify `docker-compose.yml` to mount the correct volume on the docker machine.

### AS2 project structure & Unizar variation

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
  - **ground_station.yaml**: Tmuxinator launch file for the ground station. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml*.
- **launch_ground_station.bash**: Script to launch nodes defined in *tmuxinator/ground_station.yaml*.
- **mission_\*.py**: Differents python mission files that can be executed.
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script. As this project combines several platforms, `config` folders will have platform-specific subfolders and `launch` scripts will have platform-specific suffix.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.

## Config Jetson Orin Nano

This system was tested on a ROS2 Jazzy Docker container provided by Nvidia on a Jetson Oring Nano (8Gb) with JetPack 6.2 [L4T >36.0] and ground station computers running a ROS2 Jazzy Docker container. The Pixhawk firmware version is v1.14. Install Docker with containers. The base Docker container comes from [dustynv](https://hub.docker.com/r/dustynv/ros/tags)