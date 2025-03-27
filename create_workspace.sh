#!/bin/bash

# Define workspace and source directory
WORKSPACE="drones_unizar_dev_ws"
SRC_DIR="$WORKSPACE/src"

# Create the src directory if it doesn't exist
mkdir -p "$SRC_DIR"

# Clone the repositories (Replace URLs with actual repo links)
# git clone https://github.com/user/repo1.git "$SRC_DIR/repo1"
# git clone https://github.com/user/repo2.git "$SRC_DIR/repo2"

# NOTE: Symlink doesn't work in Docker. Thus, folders within this repository are added using the docker-compose file

echo "Setup complete."