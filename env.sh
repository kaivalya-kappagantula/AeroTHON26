# env.sh — set up a terminal for AeroTHON26:
#
#   source ~/AeroTHON26/env.sh
#
# Sources ROS 2 Jazzy, the camera workspace (Pi only, if it was built) and the
# mission workspace, and sets AEROTHON_ROOT (used by the run scripts).

AEROTHON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AEROTHON_ROOT

source /opt/ros/jazzy/setup.bash
if [ -f "$AEROTHON_ROOT/camera_ws/install/setup.bash" ]; then
    source "$AEROTHON_ROOT/camera_ws/install/setup.bash"
fi
if [ -f "$AEROTHON_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$AEROTHON_ROOT/ros2_ws/install/setup.bash"
else
    echo "[env.sh] ros2_ws is not built yet — run $AEROTHON_ROOT/setup.sh first"
fi
