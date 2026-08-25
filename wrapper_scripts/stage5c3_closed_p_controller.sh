#!/usr/bin/env bash
# Controller-side wrapper for the closed-loop (P-structure) external controller.
#
# THIS IS THE OPPOSITE OF THE SIM-SIDE WRAPPERS. The sim side uses Isaac's
# BUNDLED rclpy (Python 3.11) and does NOT source native Jazzy. The controller
# side SOURCES native /opt/ros/jazzy (Python 3.12) and uses the system rclpy.
# Two environments that never share a Python, talking only over DDS topics.
#
# CRITICAL -- CONDA SHADOWING: if conda `base` auto-activates in your shell
# (prompt shows "(base)"), then `python3` resolves to conda's Python, NOT system
# 3.12. Native Jazzy is built for 3.12, so conda's interpreter both (a) can't
# load native rclpy's C extensions and (b) lacks apt-installed deps like
# python3-yaml. We therefore fully DEACTIVATE conda here before sourcing Jazzy,
# so `python3` is the system 3.12 that /opt/ros/jazzy expects.
#
# WHY THIS ISN'T JUST `conda deactivate`: `conda` is a shell FUNCTION defined by
# conda's init block in ~/.bashrc. A script run via `bash foo.sh` is a
# NON-interactive shell and does NOT source that init, so the `conda` function
# doesn't exist here -- `command -v conda` returns false and any deactivate is
# skipped. But the PARENT shell's CONDA_PREFIX and PATH (with conda's bin
# prepended) ARE inherited. So we must load conda's function ourselves, then
# deactivate every stacked env. Fallback: if conda's init can't be found, strip
# conda paths from PATH directly -- removing them is all we actually need.
set -e

if [ -n "${CONDA_PREFIX:-}" ]; then
    # Try to source conda's shell function from the standard location(s).
    _conda_sh=""
    for _cand in \
        "$HOME/miniconda3/etc/profile.d/conda.sh" \
        "$HOME/anaconda3/etc/profile.d/conda.sh" \
        "${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")/etc/profile.d/conda.sh}"; do
        if [ -n "$_cand" ] && [ -f "$_cand" ]; then _conda_sh="$_cand"; break; fi
    done

    if [ -n "$_conda_sh" ]; then
        # shellcheck disable=SC1090
        source "$_conda_sh"
        while [ -n "${CONDA_PREFIX:-}" ]; do
            conda deactivate || break
        done
    else
        # Fallback: no conda.sh found -- strip conda dirs from PATH by hand.
        echo "[wrapper] conda.sh not found; stripping conda from PATH directly."
        PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -E '(miniconda|anaconda)' | paste -sd ':' -)"
        export PATH
        unset CONDA_PREFIX
    fi
fi

# Source native ROS 2 Jazzy (system Python 3.12).
source /opt/ros/jazzy/setup.bash
echo "[wrapper] ROS 2 sourced: ROS_DISTRO=$ROS_DISTRO"

# Prove which interpreter we're about to use -- should be /usr/bin/python3, 3.12.
echo "[wrapper] python3 -> $(which python3)"
python3 -c "import sys; print('[wrapper] version:', sys.version.split()[0])"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/stage5c3_closed_p_controller.py"
