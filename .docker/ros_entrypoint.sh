#!/bin/bash
set -e

source /opt/ros/humble/setup.sh
source ~/ros2_ws/install/setup.bash

exec "$@"