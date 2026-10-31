#!/bin/bash
# fly.sh — the whole mission, one command.
#
#   ./fly.sh            on the drone (Pi 5): MAVROS on /dev/ttyAMA2,
#                       downward Cam 1.3, every phase, land
#   ./fly.sh sim        on a laptop: ArduPilot SITL at wp_home (the real site,
#                       from mission_params.yaml) + simulated cameras, same code
#
# Extra arguments go to mission.launch.py, e.g. ./fly.sh fcu_url:=/dev/ttyACM0:115200
#
# Live view:  http://<pi>:8080/ (down camera)
# Logs:       ~/aerothon_logs/mission_<time>/   one file per phase + summary.log
# Stop:       Ctrl+C (in the air the FC keeps holding position in GUIDED:
#             take over with the RC in LOITER / LAND)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/env.sh" >/dev/null
if [ ! -f "$ROOT/ros2_ws/install/setup.bash" ]; then
    echo "[fly.sh] ros2_ws is not built — run $ROOT/setup.sh first"
    exit 1
fi

if [ "$1" != "sim" ]; then
    exec ros2 launch mission_bringup mission.launch.py "$@"
fi
shift

# ── SITL ────────────────────────────────────────────────────────────
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
PARAMS="$ROOT/ros2_ws/src/mission_bringup/config/mission_params.yaml"
HOME_LL=$(python3 -c "import yaml; p = yaml.safe_load(open('$PARAMS'))['/**']['ros__parameters']['wp_home']; print(f'{p[0]},{p[1]}')")
HEADING=296          # the drone is placed facing the arena (site survey)
SITL_LOG="$HOME/aerothon_logs/sitl_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$HOME/aerothon_logs"

echo "[fly.sh] SITL at wp_home $HOME_LL, heading $HEADING (log $SITL_LOG)"
(cd "$ARDUPILOT_DIR/Tools/autotest" && exec python3 sim_vehicle.py -v ArduCopter -f quad \
    --no-mavproxy --no-rebuild -w -l "$HOME_LL,0,$HEADING" \
    --add-param-file="$ROOT/ros2_ws/src/mission_bringup/config/sitl_relays.parm") >"$SITL_LOG" 2>&1 &
SITL_PID=$!
cleanup() {
    pkill -INT -P $SITL_PID 2>/dev/null; kill $SITL_PID 2>/dev/null
    pkill -f "arducopter.*--model" 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 60); do
    ss -tln 2>/dev/null | grep -q ":5760 " && break
    sleep 1
done
if ! ss -tln 2>/dev/null | grep -q ":5760 "; then
    echo "[fly.sh] SITL did not open tcp:5760 — see $SITL_LOG"
    exit 1
fi
ros2 launch mission_bringup mission.launch.py sim:=true "$@"
