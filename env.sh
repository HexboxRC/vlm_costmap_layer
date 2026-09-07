#!/usr/bin/env bash
# Project environment for semantic-nav-ros.
#
# Source this manually in every terminal you work in:
#   source env.sh
#
# Do NOT add this to ~/.bashrc or any other global shell config — the
# user has a separate ROS workspace on this machine, and sourcing two
# workspaces in the same shell causes package shadowing.

# Uncommon domain ID, chosen to avoid colliding with the user's other
# ROS 2 workspace/robots on this machine.
export ROS_DOMAIN_ID=47

# Base ROS 2 install (Jazzy).
source /opt/ros/jazzy/setup.bash

# This project's colcon workspace overlay, if it has been built yet.
SEMANTIC_NAV_WS_SETUP="$(dirname "${BASH_SOURCE[0]}")/ros2_ws/install/setup.bash"
if [ -f "$SEMANTIC_NAV_WS_SETUP" ]; then
    source "$SEMANTIC_NAV_WS_SETUP"
else
    echo "env.sh: ros2_ws not built yet (no install/setup.bash) — sourced /opt/ros/jazzy only." >&2
fi

echo "env.sh: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
