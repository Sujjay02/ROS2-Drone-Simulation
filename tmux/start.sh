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

# ...existing code...

# Wait a short moment to let tmuxp create the session
sleep 0.3

# --- Attach or Switch Logic ---
# Try the requested socket first; fall back to the default server if needed.

# helper to test for a session on a given socket
has_session_on_socket() {
  local socket="$1"; local session="$2"
  if [ -n "$socket" ]; then
    tmux -L "$socket" has-session -t "$session" >/dev/null 2>&1
    return $?
  else
    tmux has-session -t "$session" >/dev/null 2>&1
    return $?
  fi
}

# pick where the session exists
TARGET_SOCKET=""
if has_session_on_socket "$TMUX_SOCKET_NAME" "$TMUX_SESSION_NAME"; then
  TARGET_SOCKET="$TMUX_SOCKET_NAME"
elif has_session_on_socket "" "$TMUX_SESSION_NAME"; then
  TARGET_SOCKET=""
else
  echo "Session '$TMUX_SESSION_NAME' not found on socket '$TMUX_SOCKET_NAME' or default server."
  echo "Run 'tmux ls' and 'tmux -L $TMUX_SOCKET_NAME ls' to inspect servers."
  exit 1
fi

# If not currently inside tmux, attach normally to the correct server
if [ -z "$TMUX" ]; then
  echo "Attaching to session '$TMUX_SESSION_NAME' on socket '${TARGET_SOCKET:-default}'..."
  if [ -n "$TARGET_SOCKET" ]; then
    tmux -L "$TARGET_SOCKET" attach-session -t "$TMUX_SESSION_NAME"
  else
    tmux attach-session -t "$TMUX_SESSION_NAME"
  fi

# If inside tmux, detach client and reattach to the target server
else
  echo "Switching to session '$TMUX_SESSION_NAME' on socket '${TARGET_SOCKET:-default}'..."
  if [ -n "$TARGET_SOCKET" ]; then
    tmux detach-client -E "tmux -L '$TARGET_SOCKET' attach-session -t '$TMUX_SESSION_NAME'"
  else
    tmux detach-client -E "tmux attach-session -t '$TMUX_SESSION_NAME'"
  fi
fi
# ...existing code...