#!/usr/bin/env bash
# run_stage1_sweep.sh — Gates 1A / 1B / 1C, fully automated & observation-only.
#
# Brings up a headless SITL + MAVROS + the synthetic node, sweeps the synthetic
# obstacle bearing (0, -30, +30) and yaws 90 deg, then exports the PRX proximity
# sectors from the dataflash log and prints WHICH sector was nearest per window.
# It does NOT assert the expected mapping — the whole point of 1B is catching a
# sign error, and a check whose expectation came from geometry.py would pass on
# mirrored data. YOU read the table and judge.
#
# WARNING: this kills any running sim_vehicle / MAVProxy / MAVROS first, so it
# will take over the machine's SITL. Run it when you own the box.
#
# Usage:  scripts/run_stage1_sweep.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
WS="${WS:-${AEROTHON_ROOT:-$HOME/AeroTHON26}/ros2_ws}"
CTRL_URL="tcp:127.0.0.1:5760"       # our pymavlink control link (SERIAL0)
FCU_URL="tcp://127.0.0.1:5762"      # MAVROS link (SERIAL1)
PARAM="$PKG/params/stage1_proximity.param"
WINDOWS="/tmp/stage1_windows.csv"
PRX_CSV="/tmp/stage1_prx_raw.csv"
PIDS=()

log(){ echo -e "\n=== $* ==="; }
cleanup(){ log "cleanup"; for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
           pkill -9 -f "sim_vehicle|arducopter|mavros_node|synthetic_sector_node|obstacle_publisher_node" 2>/dev/null; }
trap cleanup EXIT
fail(){ echo "FATAL: $*" >&2; exit 1; }

# ROS setup scripts reference unset vars; disable nounset while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

log "0. kill any existing SITL/MAVROS/nodes"
pkill -9 -f "sim_vehicle|arducopter|mavros_node|mavproxy|synthetic_sector_node|obstacle_publisher_node" 2>/dev/null
sleep 3

log "1. launch headless SITL (--no-mavproxy)"
( cd "$ARDUPILOT_DIR" && python3 Tools/autotest/sim_vehicle.py -v ArduCopter \
    --no-mavproxy --no-rebuild -I0 >/tmp/sitl.log 2>&1 ) &
PIDS+=($!)
echo "waiting for SITL MAVLink on 5760 ..."
for i in $(seq 1 90); do (echo >/dev/tcp/127.0.0.1/5760) 2>/dev/null && break; sleep 1; done
(echo >/dev/tcp/127.0.0.1/5760) 2>/dev/null || fail "SITL never opened 5760 (see /tmp/sitl.log)"
sleep 5

# PRX1_TYPE is already active in the SITL eeprom (the readback below confirms
# got=2), and LOG_DISARMED takes effect live, so NO reboot is needed. (Headless
# SITL launched with --no-mavproxy does not reliably come back from a MAVLink
# reboot anyway.) If you ever start from a wiped eeprom, pass the param file to
# SITL at launch via --add-param-file instead of rebooting here.
# SYSID_MYGCS=250 matches our pymavlink control link's source system, so the FC
# accepts our RC_CHANNELS_OVERRIDE (climb/yaw). ArduPilot ignores RC override
# from any sysid != SYSID_MYGCS (default 255) — that is why the drone sat on the
# ground the first time. (Arming via COMMAND_LONG is not sysid-gated, so it armed.)
log "2. load stage1 params + LOG_DISARMED=1 + SYSID_MYGCS=250 (no reboot)"
python3 "$HERE/gate_lib.py" --url "$CTRL_URL" load-params \
    --file "$PARAM" --set LOG_DISARMED=1 --set SYSID_MYGCS=250 --strict \
    || fail "param load failed"

log "3. launch MAVROS on $FCU_URL, wait for connected"
ros2 launch mavros apm.launch fcu_url:="$FCU_URL" >/tmp/mavros.log 2>&1 &
PIDS+=($!)
python3 - <<'PY' || fail "MAVROS not connected"
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
q=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
rclpy.init(); n=Node('wc'); ok={'v':False}
n.create_subscription(State,'/mavros/state', lambda m: ok.__setitem__('v', m.connected), q)
t=time.time()
while time.time()-t<60 and not ok['v']: rclpy.spin_once(n, timeout_sec=0.3)
print("connected:", ok['v']); n.destroy_node(); rclpy.shutdown()
import sys; sys.exit(0 if ok['v'] else 1)
PY

log "4. launch synthetic node + our obstacle_publisher (NOT the MAVROS plugin)"
# /avoidance/scan stays canonical (no remap). Our publisher builds a correct
# OBSTACLE_DISTANCE (CCW->CW reversed, frame=BODY_FRD) and injects it via
# /uas1/mavlink_sink. The MAVROS obstacle_distance plugin is NOT used: it
# mirrors L/R (copies ranges[] with no reversal) and defaults frame=GLOBAL.
ros2 run obstacle_avoidance synthetic_sector_node --ros-args \
    -p obstacle_range_m:=3.0 -p obstacle_width_deg:=10.0 \
    -p obstacle_bearing_deg:=0.0 >/tmp/synth.log 2>&1 &
PIDS+=($!)
ros2 run obstacle_avoidance obstacle_publisher_node --ros-args \
    -p transport:=mavros >/tmp/obsp.log 2>&1 &
PIDS+=($!)
for i in $(seq 1 30); do ros2 node list 2>/dev/null | grep -q obstacle_publisher_node && break; sleep 1; done
ros2 node list 2>/dev/null | grep -q synthetic_sector_node || fail "synthetic node did not start"
ros2 node list 2>/dev/null | grep -q obstacle_publisher_node || fail "obstacle_publisher did not start"
sleep 2

log "4b. ASSERT the chain: synthetic -> /avoidance/scan (RELIABLE) -> our publisher"
INFO="$(ros2 topic info -v /avoidance/scan 2>/dev/null)"
echo "$INFO" | grep -E "Publisher count|Subscription count"
PUBC=$(echo "$INFO" | sed -n 's/.*Publisher count: *\([0-9]*\).*/\1/p')
SUBC=$(echo "$INFO" | sed -n 's/.*Subscription count: *\([0-9]*\).*/\1/p')
RELPUB=$(echo "$INFO" | awk '/Publisher/{p=1} /Subscription/{p=0} p&&/Reliability: RELIABLE/{c++} END{print c+0}')
echo "publisher=$PUBC subscription=$SUBC reliable_publishers=$RELPUB"
[ "$PUBC" = "1" ] || fail "expected 1 publisher on /avoidance/scan, got $PUBC"
[ "${SUBC:-0}" -ge 1 ] || fail "obstacle_publisher not subscribed to /avoidance/scan"
[ "${RELPUB:-0}" -ge 1 ] || fail "/avoidance/scan publisher must be RELIABLE"
grep -q "obstacle_publisher_node up" /tmp/obsp.log || fail "obstacle_publisher did not init (see /tmp/obsp.log)"
echo "chain assertion PASSED (scan RELIABLE + our publisher injecting via /uas1/mavlink_sink)"

log "5-7. arm, loiter, climb 5 m, sweep 0/-30/+30, yaw 90 (Gate 1C)"
python3 "$HERE/gate_lib.py" --url "$CTRL_URL" stage1-flight \
    --node /synthetic_sector_node --alt 5.0 --bearings 0.0,-30.0,30.0 \
    --hold 10 --range 3.0 --width 10.0 --windows-out "$WINDOWS" \
    || fail "flight sequence failed"

log "8. locate newest .BIN"
# sim_vehicle --no-mavproxy writes logs to $ARDUPILOT_DIR/logs, NOT
# ArduCopter/logs — search both and take the newest overall.
BIN="$(ls -t "$ARDUPILOT_DIR"/logs/*.BIN "$ARDUPILOT_DIR"/ArduCopter/logs/*.BIN \
        2>/dev/null | head -1)"
[ -n "$BIN" ] || fail "no .BIN log found"
echo "log: $BIN"

log "9-10. export PRX + print OBSERVED sector table (you judge 1B; D0 for 1A)"
python3 "$HERE/gate_lib.py" analyze-prx --bin "$BIN" --windows "$WINDOWS" \
    --range 3.0 --csv-out "$PRX_CSV"

echo -e "\nDONE. Windows: $WINDOWS   Raw PRX: $PRX_CSV"
echo "Gate 1A: D0 median in the bearing 0 window should be ~3.0 m."
echo "Gate 1B: judge which sector holds ~3 m for -30 vs +30 (do NOT trust me)."
echo "Gate 1C: in the yaw window the obstacle should stay in the SAME sector."
