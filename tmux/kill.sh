#!/bin/bash

# --- Configuration ---
# Set your ROS 2 distro (e.g., humble, iron, etc.)
ROS_DISTRO="jazzy"

# Set the name of the tmux session you want to control
export TMUX_SESSION_NAME="sweeping_generator"

# Set a unique socket name to avoid conflicts with other tmux servers
export TMUX_SOCKET_NAME="mrs"
# ---------------------

# Absolute path this script is in.
SCRIPTPATH=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPTPATH"

# Source the ROS 2 environment to make ros2 commands available
# This ensures that even if you run this script from a non-sourced terminal, it will work.
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# If this script is in the root of a colcon workspace, source it too.
if [ -f "install/setup.bash" ]; then
    echo "Sourcing local workspace..."
    source "install/setup.bash"
fi

echo "Targeting tmux session: '$TMUX_SESSION_NAME' on socket '$TMUX_SOCKET_NAME'"

# Create a new, temporary pane in the target session to run the cleanup commands
tmux -L "$TMUX_SOCKET_NAME" split-window -t "$TMUX_SESSION_NAME"

# Define the sequence of commands to send to the new pane
CLEANUP_COMMAND="
echo '--- Starting ROS 2 Cleanup ---';
sleep 1;
PIDS_TO_KILL=\$(tmux list-panes -s -F '#{pane_pid} #{pane_current_command}' | grep -v tmux | cut -d' ' -f1);
if [ -n \"\$PIDS_TO_KILL\" ]; then
    echo \"Sending SIGINT (Ctrl+C) to processes...\";
    kill -2 \$PIDS_TO_KILL;
    sleep 2;
    echo \"Sending SIGKILL to any remaining processes...\";
    kill -9 \$PIDS_TO_KILL 2>/dev/null || true;
fi;
echo 'Cleanup complete. Closing pane.';
exit;
"

# Send the commands to the temporary pane and press ENTER
tmux -L "$TMUX_SOCKET_NAME" send-keys -t "$TMUX_SESSION_NAME" "$CLEANUP_COMMAND" ENTER

echo "Cleanup command sent to session '$TMUX_SESSION_NAME'."