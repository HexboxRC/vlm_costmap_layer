#!/usr/bin/env bash
# Run from a normal host terminal: bash scripts/phase0.sh <runner arguments>.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source ./env.sh
runtime="$PWD/results/phase0/runtime"
mkdir -p "$runtime"/{ros,gz,cache,config,tmp,matplotlib}
export ROS_HOME="$runtime/ros" ROS_LOG_DIR="$runtime/ros/log"
export GZ_HOMEDIR="$runtime/gz" XDG_CACHE_HOME="$runtime/cache"
export XDG_CONFIG_HOME="$runtime/config" TMPDIR="$runtime/tmp"
export MPLCONFIGDIR="$runtime/matplotlib"
if [[ "${1:-}" == smoke || "${1:-}" == pilot ]]; then
    exec python3 scripts/run_matched_pilot.py "$1"
fi
if [[ "${1:-}" == projection-capture ]]; then
    shift
    exec python3 scripts/capture_projection_evidence.py "$@"
fi
if [[ "${1:-}" == campaign ]]; then
    shift
    exec python3 scripts/run_campaign.py "$@"
fi
if [[ "${1:-}" == contacts ]]; then
    exec python3 scripts/probe_contacts.py
fi
if [[ "${1:-}" == design ]]; then
    exec python3 scripts/check_scenario_design.py
fi
if [[ "${1:-}" == test ]]; then
    exec python3 -m unittest discover -s ros2_ws/src/semantic_nav_sim/tests -v
fi
exec python3 ros2_ws/src/semantic_nav_sim/scripts/run_trials.py "$@"
