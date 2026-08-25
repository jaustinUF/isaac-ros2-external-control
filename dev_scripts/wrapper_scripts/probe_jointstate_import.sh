#!/usr/bin/env bash
# Sim-side wrapper (Option 1 from the 5C.2 progress note, verbatim) pointed at
# the JointState import probe. Uses the bridge's BUNDLED rclpy (Python 3.11);
# native /opt/ros/jazzy is deliberately NOT sourced here.
set -e

BRIDGE_LIB="$HOME/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate env_isaaclab

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_LIB"

# Run the probe next to this wrapper.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python "$SCRIPT_DIR/probe_jointstate_import.py"
