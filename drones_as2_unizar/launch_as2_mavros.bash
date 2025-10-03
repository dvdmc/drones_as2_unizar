#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: select drone namespace to launch. Default is 'drone0'"
    echo "      -c: motion controller plugin (pid_speed_controller, differential_flatness_controller), choices: [pid, df]. Default: pid"
    echo "      -r: record rosbag. Default not launch"
}

# Initialize variables with default values
drones_namespace="drone0"
motion_controller_plugin="pid"
rosbag="false"

# Arg parser
while getopts "n" opt; do
  case ${opt} in
    n )
      drones_namespace="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
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

# Check if motion controller plugins are valid
case ${motion_controller_plugin} in
  pid )
    motion_controller_plugin="pid_speed_controller"
    ;;
  df )
    motion_controller_plugin="differential_flatness_controller"
    ;;
  * )
    echo "Invalid motion controller plugin: ${motion_controller_plugin}" >&2
    usage
    exit 1
    ;;
esac

# Launch aerostack2 for each drone namespace
eval "tmuxinator start -n ${drones_namespace} -p tmuxinator/mavros/aerostack2.yaml \
    drone_namespace=${drones_namespace} \
    motion_controller_plugin=${motion_controller_plugin} \
    rosbag=${rosbag}"
    wait

# Attach to tmux session
tmux attach-session -t ${drone_namespace}
