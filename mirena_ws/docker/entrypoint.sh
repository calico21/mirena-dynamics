#!/usr/bin/env bash
#
# Deployment entrypoint: source ROS and the workspace, then run the stack.

set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /home/mirena/mirena_ws/install/setup.bash

# exec, so ros2 launch becomes PID 1 and receives the stop signal directly.
# Without it the signal stops at this shell and the launch is left to be
# SIGKILLed on timeout -- which drops whatever the CAN logger still holds in its
# write buffer and leaves the mission's nodes as orphans.
exec ros2 launch mirena_supervisor hardware.launch.py
