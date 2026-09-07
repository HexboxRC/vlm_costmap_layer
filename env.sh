#!/usr/bin/env bash
# Project environment for semantic-nav-ros.
#
# Source this manually in every terminal you work in:
#   source env.sh
#
# Do NOT add this to ~/.bashrc or any other global shell config — the
# user has a separate ROS workspace on this machine, and sourcing two
# workspaces in the same shell causes package shadowing.

# Reset, not layer: if this shell already has another ROS workspace
# sourced (e.g. from ~/.bashrc), prepending on top of it is exactly the
# package-shadowing scenario we're avoiding. Clear the ROS/ament/colcon
# environment first so this project starts from a clean base.
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
# The user's other workspace's CYCLONEDDS_URI (~/.config/cyclonedds/
# cyclonedds.xml) pins a network interface by IP. WSL2 reassigns IPs
# across reboots, so a stale IP there makes rmw_cyclonedds_cpp abort on
# EVERY node in EVERY workspace, not just this project (observed
# 2026-09-06: pinned to 192.168.4.13, which no longer exists — see
# `ip -4 -o addr show`). Unset it here so this project's shell falls
# back to CycloneDDS's zero-config default (listen on all interfaces)
# regardless of what the other workspace's config currently says. This
# does not fix that file — it isn't ours to touch — it just stops this
# project's shell from inheriting its breakage.
unset CYCLONEDDS_URI

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
