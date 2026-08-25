#!/usr/bin/env bash
# Sim-side wrapper for the 5C.3 command subscriber.
# Identical environment to the publisher wrapper (Option 1 from the 5C.2
# progress note): the bridge's BUNDLED rclpy (Python 3.11); native
# /opt/ros/jazzy deliberately NOT sourced here. Same launch environment,
# opposite data direction.
set -e

BRIDGE_LIB="$HOME/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate env_isaaclab

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_LIB"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python "$SCRIPT_DIR/stage5c3_command_subscriber.py"
