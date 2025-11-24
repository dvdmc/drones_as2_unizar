#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: drone namespaces, comma separated. Default is 'drone0'"
    echo "      -t: launch keyboard teleoperation. Default not launch"
    echo "      -v: open rviz. Default launch"
    echo "      -m: launch mocap4ros2. Default launch"
}

# Initialize variables with default values
drones_namespace_comma="drone0"
keyboard_teleop="false"
rviz="true"
mocap4ros2="true"

# Parse command line arguments
while getopts "mtvn:" opt; do
  case ${opt} in
    t )
      keyboard_teleop="true"
      ;;
    v )
      rviz="true"
      ;;
    n )
      drones_namespace="${OPTARG}"
      ;;
    m )
      mocap4ros2="false"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If no drone namespaces are provided, finish the execution
if [ -z "$drones_namespace" ]; then
  echo "No drone namespace provided. Set it using the -n option"
  exit 1
fi

# Launch aerostack2 ground station
eval "tmuxinator start -n ground_station -p tmuxinator/mavros/ground_station.yaml \
  drone_namespace=${drones_namespace} \
  keyboard_teleop=${keyboard_teleop} \
  rviz=${rviz} \
  mocap4ros2=${mocap4ros2} \
  wait"

# Attach to tmux session
tmux attach-session -t ground_station
