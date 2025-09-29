#!/bin/bash

# --- Configuration ---
# Set your ROS 2 distro (e.g., humble, iron, etc.)
ROS_DISTRO="jazzy"

# These should match the 'session_name' and 'socket_name' in your YAML file
TMUX_SESSION_NAME="sweeping_generator"
TMUX_SOCKET_NAME="mrs"

# The name of your tmuxp configuration file
CONFIG_FILE="session.yml"
# ---------------------

# Get the script's absolute directory and change into it to find the config file
SCRIPTPATH=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPTPATH"

# Source the ROS 2 environment BEFORE launching the session
echo "Sourcing ROS 2 ${ROS_DISTRO}..."
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# If this script is in the root of a colcon workspace, source it too
if [ -f "install/setup.bash" ]; then
    echo "Sourcing local workspace..."
    source "install/setup.bash"
fi

# Load the session using tmuxp, which starts it in the background
echo "Loading tmuxp session from '${CONFIG_FILE}'..."
tmuxp load -d "$CONFIG_FILE"

# --- Attach or Switch Logic ---

# If we are NOT currently in a tmux session
if [ -z "$TMUX" ]; then
    echo "Attaching to new session..."
    # Attach to the session on the correct socket
    tmux -L "$TMUX_SOCKET_NAME" attach-session -t "$TMUX_SESSION_NAME"

# If we ARE currently in a tmux session
else
    echo "Switching to new session..."
    # Detach the current client and immediately run the command to attach to the new session
    tmux detach-client -E "tmux -L '$TMUX_SOCKET_NAME' attach-session -t '$TMUX_SESSION_NAME'"
fi