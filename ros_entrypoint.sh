#!/bin/bash
set -e

export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source /app/install/setup.bash
source /app/scripts/cyphal_env_source.sh

exec "$@"
