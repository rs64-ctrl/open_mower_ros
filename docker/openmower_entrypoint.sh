#!/bin/bash
set -e

# Source ROS2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source prebuilt slic3r underlay (if present)
if [ -f /opt/prebuilt/slic3r_coverage_planner/setup.bash ]; then
    source /opt/prebuilt/slic3r_coverage_planner/setup.bash
fi

# Source workspace overlay
source /opt/open_mower_ros/install/setup.bash

# Source version info
if [ -f /opt/open_mower_ros/version_info.env ]; then
    source /opt/open_mower_ros/version_info.env
fi

# ROS2 logging configuration controlled via DEBUG env var
shopt -s nocasematch
case "${DEBUG:-0}" in
    1|true|yes|on|y)
        # Debug mode: verbose logging
        export RCUTILS_LOGGING_BUFFERED_STREAM=0
        export ROS_LOG_DIR=/root/.ros/log
    ;;
    *)
        # Production mode: warn and above only
        export RCUTILS_LOGGING_BUFFERED_STREAM=1
        export RCUTILS_COLORIZED_OUTPUT=0
    ;;
esac
shopt -u nocasematch || true

# Ensure stdout/stderr are unbuffered for real-time logging
export PYTHONUNBUFFERED=1

exec -- "$@"
