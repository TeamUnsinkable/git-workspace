#!/bin/bash
set -e

export ROS_DOMAIN_ID=69
# Source ROS
source /opt/ros/humble/setup.bash
# Source Compiled Workspace
source /app/install/setup.bash
# Source CYPHAL Environment
source /app/scripts/cyphal_env_source.sh

exec "$@"
