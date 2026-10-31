#!/bin/bash
# setup.sh — one-shot setup of AeroTHON26 on a Raspberry Pi 5 or a laptop.
#
#   ./setup.sh            install everything and build
#   ./setup.sh --build    only rebuild (no apt / pip)
#   ./setup.sh --build --camera   ... also rebuild the Pi camera workspace
#                         (libcamera + camera_ros: slow; only built
#                         automatically the first time)
#
# Needs: Ubuntu 24.04 with ROS 2 Jazzy installed (/opt/ros/jazzy) and sudo.
# Safe to run again (after a git pull, say): it skips what is already there.
#
# What it does:
#   1. apt: colcon, rosdep, MAVROS (+ its GeographicLib datasets), pip
#   2. rosdep: every system package the ROS packages declare (OAK-D driver,
#      cv_bridge, shapely, OpenCV, ...)
#   3. pip (system python3, --user): requirements.txt
#   4. build ros2_ws; on a Raspberry Pi also camera_ws (the Pi camera driver,
#      camera_ros, built against the Pi's own libcamera)
# Afterwards, in every terminal:  source <repo>/env.sh

set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ONLY=false
CAMERA=false
for a in "$@"; do
    [ "$a" = "--build" ] && BUILD_ONLY=true
    [ "$a" = "--camera" ] && CAMERA=true
done

say() { echo -e "\n\033[1;32m==> $*\033[0m"; }
die() { echo -e "\n\033[1;31mERROR: $*\033[0m" >&2; exit 1; }

[ -f /opt/ros/jazzy/setup.bash ] || die "ROS 2 Jazzy not found in /opt/ros/jazzy.
Install it first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

IS_PI=false
if grep -qs "Raspberry Pi" /proc/device-tree/model; then
    IS_PI=true
fi
say "AeroTHON26 in $ROOT ($( $IS_PI && echo 'Raspberry Pi' || echo 'laptop / PC'))"

if ! $BUILD_ONLY; then
    say "apt packages"
    sudo apt-get update
    sudo apt-get install -y python3-pip python3-colcon-common-extensions python3-rosdep \
        ros-jazzy-mavros ros-jazzy-mavros-extras

    say "MAVROS GeographicLib datasets"
    if [ ! -d /usr/share/GeographicLib/geoids ]; then
        sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
    else
        echo "already installed"
    fi

    say "rosdep (system dependencies of the ROS packages)"
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
    rosdep update
    rosdep install --from-paths "$ROOT/ros2_ws/src" --ignore-src -y --rosdistro jazzy
    if $IS_PI; then
        # libcamera itself comes from the Pi's own (Raspberry Pi) build, not apt
        rosdep install --from-paths "$ROOT/camera_ws/src" --ignore-src -y --rosdistro jazzy \
            --skip-keys libcamera
    fi

    say "pip packages (system python3, --user)"
    /usr/bin/python3 -m pip install --user --break-system-packages -r "$ROOT/requirements.txt"

    if $IS_PI && ! id -nG "$USER" | grep -qw dialout; then
        say "adding $USER to the dialout group (Pixhawk serial port) — log out and in again"
        sudo usermod -aG dialout "$USER"
    fi
fi

# camera_ws (libcamera + camera_ros) only changes when the camera stack does:
# built the first time, afterwards only with --camera (one package at a time).
if $IS_PI; then
    if $CAMERA || [ ! -f "$ROOT/camera_ws/install/camera_ros/share/camera_ros/package.xml" ]; then
        say "building camera_ws (Pi camera driver) — takes a while"
        (cd "$ROOT/camera_ws" && colcon build --symlink-install --parallel-workers 1)
    else
        say "camera_ws already built — skipped (./setup.sh --build --camera to rebuild it)"
    fi
fi

say "building ros2_ws"
(cd "$ROOT/ros2_ws" && colcon build --symlink-install)

say "checking the Python packages the nodes import"
/usr/bin/python3 - <<'EOF'
import cv2, numpy, shapely, statemachine
assert cv2.__version__.startswith("4."), f"OpenCV {cv2.__version__}: cv_bridge needs 4.x"
assert numpy.__version__.startswith("1."), f"NumPy {numpy.__version__}: cv_bridge needs 1.x"
print(f"OpenCV {cv2.__version__}, NumPy {numpy.__version__}, shapely {shapely.__version__}, "
      f"python-statemachine {statemachine.__version__}: OK")
EOF

say "done. In every new terminal:   source $ROOT/env.sh"
