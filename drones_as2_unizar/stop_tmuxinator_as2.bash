#!/bin/bash

usage() {
    echo "  Add drones namespaces as arguments, separated by commas"
}

# Get the directory of the current script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get drone namespaces from command-line argument if not provided, use drone0 and drone1 as default
if [ -z "$1" ]; then
    usage
    echo "No drone namespaces provided. Using default: drone0,drone1"
    drones_namespace_comma="drone0,drone1"
else
    drones_namespace_comma="$1"
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Make a tmux list of sessions to be killed
tmux_session_list=()

# Add drones from user input
for namespace in ${drone_namespaces[@]}; do
  tmux_session_list+=("$namespace")
done

# If inside tmux session, get the current session name
if [[ -n "$TMUX" ]]; then
  current_session=$(tmux display-message -p '#S')
fi

# Send Ctrl+C signal to each window of each session
for session in ${tmux_session_list[@]}; do
  # Check if session exists
  if tmux has-session -t "$session" 2>/dev/null; then
    # Get the list of windows in the session
    windows=($(tmux list-windows -t "$session" -F "#{window_index}"))
    # Iterate through each window and send Ctrl+C
    for window in "${windows[@]}"; do
      # Send Ctrl+C to the window
      tmux send-keys -t "$session:$window" C-c
      sleep 0.1 # Add a small delay to allow the signal to be processed
    done
  fi
done

# Kill all tmux sessions from the list except for the current one
for session in ${tmux_session_list[@]}; do
  if [[ "$session" != "$current_session" ]]; then
    tmux kill-session -t "$session" 2>/dev/null
  fi
done

# Kill the current tmux session, if in a tmux session
if [[ -n "$TMUX" ]]; then
  tmux kill-session -t "$current_session" 2>/dev/null
fi
