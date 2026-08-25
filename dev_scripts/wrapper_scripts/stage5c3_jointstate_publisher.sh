#!/usr/bin/env bash
# Sim-side wrapper for the 5C.3 JointState publisher.
# Option 1 from the 5C.2 progress note, verbatim: uses the bridge's BUNDLED
# rclpy (Python 3.11-matched); native /opt/ros/jazzy is deliberately NOT sourced
# here (that's the controller side's job -- system Python 3.12 matches native).
#
# The LD_LIBRARY_PATH line is load-bearing: it lets the bundle's libraries
# resolve EACH OTHER (librmw_implementation.so finding libament_index_cpp.so,
# etc). Without it the bridge finds its bundle but the libs can't locate their
# siblings -> "ROS2 Bridge startup failed". With native Jazzy sourced INSTEAD,
# you'd get the ABI crash (3.12 .so under Isaac's 3.11 interpreter). This env --
# bundle lib/ on the path, native absent -- is the one clean state.
set -e

BRIDGE_LIB="$HOME/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate env_isaaclab

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_LIB"

# Run the publisher next to this wrapper.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python "$SCRIPT_DIR/stage5c3_jointstate_publisher.py"
