#!/bin/bash
# ───────────────────────────────────────────────────────────────────
# run_sim.sh — Start ArduPilot SITL + ROS 2 mission stack
#
# Usage (after colcon build; or run scripts/run_sim.sh from source):
#   ros2 run mission_bringup run_sim.sh                            # default location
#   ros2 run mission_bringup run_sim.sh --lat 12.9716 --lon 77.5946  # custom location
#   ros2 run mission_bringup run_sim.sh --mavros-only              # only start MAVROS
# ───────────────────────────────────────────────────────────────────

set -e

# ── Defaults ─────────────────────────────────────────────────────
SITL_LAT="12.9716"
SITL_LON="77.5946"
SITL_ALT="0"
SITL_HDG="0"
MAVROS_ONLY=false
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ROS2_WS="${ROS2_WS:-${AEROTHON_ROOT:-$HOME/AeroTHON26}/ros2_ws}"

# ── Parse command-line arguments ─────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --lat)      SITL_LAT="$2"; shift 2 ;;
        --lon)      SITL_LON="$2"; shift 2 ;;
        --alt)      SITL_ALT="$2"; shift 2 ;;
        --heading)  SITL_HDG="$2"; shift 2 ;;
        --mavros-only) MAVROS_ONLY=true; shift ;;
        -h|--help)
            echo "Usage: $0 [--lat LAT] [--lon LON] [--alt ALT] [--heading HDG] [--mavros-only]"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

SITL_LOCATION="${SITL_LAT},${SITL_LON},${SITL_ALT},${SITL_HDG}"
echo "═══════════════════════════════════════════════════════"
echo "  Aerothon SITL Mission Launcher"
echo "  Location: ${SITL_LOCATION}"
echo "═══════════════════════════════════════════════════════"

# ── Helper: wait for a port to be listening ──────────────────────
wait_for_port() {
    local port=$1 timeout=$2 elapsed=0
    echo "  Waiting for port ${port}..."
    while ! ss -tlnp 2>/dev/null | grep -q ":${port} " && [ $elapsed -lt $timeout ]; do
        sleep 1; elapsed=$((elapsed + 1))
    done
    if [ $elapsed -ge $timeout ]; then
        echo "  WARNING: Port ${port} not ready after ${timeout}s"
    else
        echo "  Port ${port} ready (${elapsed}s)"
    fi
}

# ── TERMINAL 1: ArduPilot SITL ───────────────────────────────────
echo ""
echo "► Starting ArduPilot SITL..."
gnome-terminal --title="SITL" -- bash -c "
cd ${ARDUPILOT_DIR}/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f quad --no-mavproxy \
    -l ${SITL_LOCATION}
exec bash" 2>/dev/null

# Wait for SITL to start (MAVLink heartbeat on port 5760)
sleep 8
echo "  SITL started"

# ── TERMINAL 2: MAVROS ──────────────────────────────────────────
echo ""
echo "► Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ${ROS2_WS}/install/setup.bash
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760
exec bash" 2>/dev/null

# Wait for MAVROS to connect
sleep 5
echo "  MAVROS started"

if [ "$MAVROS_ONLY" = true ]; then
    echo ""
    echo "  --mavros-only mode: SITL + MAVROS running. Launch mission manually:"
    echo "  ros2 launch mission_bringup sitl_mission.launch.py"
    exit 0
fi

# ── TERMINAL 3: ROS2 Mission Stack ──────────────────────────────
echo ""
echo "► Starting ROS2 mission stack..."
gnome-terminal --title="Mission Stack" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ${ROS2_WS}/install/setup.bash
ros2 launch mission_bringup sitl_mission.launch.py
exec bash" 2>/dev/null

# ── TERMINAL 4: Telemetry Monitor ───────────────────────────────
echo ""
echo "► Starting telemetry monitor..."
gnome-terminal --title="Telemetry" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ${ROS2_WS}/install/setup.bash
echo 'Telemetry feed — /telemetry/combined'
echo '──────────────────────────────────────'
ros2 topic echo /telemetry/combined
exec bash" 2>/dev/null

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  All systems launched!"
echo ""
echo "  Windows:"
echo "    • SITL        — ArduPilot simulator (headless, --no-mavproxy)"
echo "    • MAVROS      — ROS2 ↔ MAVLink bridge"
echo "    • Mission     — Full autonomous mission stack"
echo "    • Telemetry   — Live JSON telemetry feed"
echo ""
echo "  Useful commands:"
echo "    ros2 topic echo /mission/status"
echo "    ros2 topic echo /mission/telemetry"
echo "    ros2 topic echo /safety/alert"
echo "    ros2 topic echo /qr/detection"
echo "═══════════════════════════════════════════════════════"
