#!/usr/bin/env bash
# Sourced environment for subprocesses launched by server.py.
# A Python subprocess never runs ~/.bashrc's interactive body (it
# returns early for non-interactive shells), so the ROS/Gazebo
# environment that's normally set up by hand in an interactive
# terminal has to be replicated explicitly here. Mirrors ~/.bashrc
# lines 118-126.
set -e

# Project root (SIH-Drone-Simulation), two levels up from this script.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ardu_ws/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$ROOT/ardu_ws/src/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH"
export GZ_VERSION=harmonic
export PATH="$PATH:$ROOT/ardu_ws/src/Micro-XRCE-DDS-Gen/scripts"

# server.py `import cv2`, which sets QT_QPA_PLATFORM_PLUGIN_PATH to its
# own bundled (broken, headless-oriented) Qt plugins directory and that
# leaks into every subprocess launched from it. That breaks Qt/X11 for
# real GUI apps in this chain (gz sim, rviz2) - gz sim's actual server
# process (the one owning physics + the camera sensor, not just the
# GUI window) was crashing on startup because of this, which meant no
# /camera/image was ever published and the live feed panel never had
# anything to show. Force the real system Qt5 plugin dir instead.
unset QT_PLUGIN_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"

# Render on the NVIDIA GPU. On a hybrid (Intel + NVIDIA, PRIME on-demand)
# laptop, Gazebo's headless/EGL rendering otherwise falls back to Mesa and
# then to software rendering ("libEGL warning: egl: failed to create dri2
# screen"), so every camera frame and lidar scan is drawn by the CPU:
# measured 0.62x real time and a ~4 fps feed. No-op without the NVIDIA driver.
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi

exec "$@"
