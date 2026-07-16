#!/usr/bin/env bash
# The ONLY entrypoint for every ROS / Gazebo command in this repo.
#
# Why this exists (all verified on this machine, 2026-07-15):
#   1. `python3` on PATH is miniforge 3.13.13, which SHADOWS /usr/bin/python3 (3.10.12).
#      ROS 2 Humble C-extensions are built for 3.10 → `import rclpy` fails under conda.
#   2. ~/.bashrc sources four unrelated workspaces (rmf_ws, movebot_ws, colcon_ws,
#      micro_ros_ws) into PYTHONPATH/AMENT_PREFIX_PATH. None of them collide with
#      weedwatch's dependency set by name, but they are noise an agent should not inherit.
#   3. EGL on this box enumerates the Intel iGPU at index 0 and llvmpipe at index 1 —
#      the RTX 4060 is NOT in the list. gz-sim#1272 (open) always picks index 0, and
#      gz-sim#1116 means llvmpipe renders BLACK. Pinning the NVIDIA ICD is mandatory.
#
# Usage:  ./scripts/env.sh <command> [args...]
#         ./scripts/env.sh python3 -c 'import rclpy; print("ok")'

set -eo pipefail

WW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WW

# --- 1. Strip the inherited environment ------------------------------------
unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH \
      CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH \
      PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE \
      IGN_GAZEBO_RESOURCE_PATH IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export PYTHONNOUSERSITE=1

# --- 2. Force EGL onto the NVIDIA ICD --------------------------------------
# libglvnd: "it is a colon-separated list of JSON filenames. The ICDs described in
# those files are loaded, in the order given. No other ICDs are loaded."
# → only NVIDIA devices enumerate → EGL device index 0 == RTX 4060.
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Fortress renders camera sensors in the SERVER. EGL is ogre2-only — never fall
# back to ogre or headless dies silently.
export IGN_GAZEBO_RENDER_ENGINE=ogre2

# Keep parallel test runs from cross-talking over DDS.
export ROS_DOMAIN_ID="${WW_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1

# --- 3. Source ROS (setup.bash trips `set -u`, so guard it) ----------------
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WW/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WW/install/setup.bash"
fi
set -u

# --- 4. Project resource paths ---------------------------------------------
export IGN_GAZEBO_RESOURCE_PATH="$WW/worlds:$WW/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <command> [args...]" >&2
  exit 2
fi

exec "$@"
