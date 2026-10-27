#!/usr/bin/env bash
# run_stage2_avoidance.sh — Gate 2 (the FC acts on the proximity data).
#
# Loads simple-avoidance params (and VERIFIES AVOID_ENABLE / AVOID_MARGIN /
# AVOID_BEHAVE actually took — AVOID_BEHAVE may be silently rejected as unknown
# the way PRX1_YAW_CORR was), puts a synthetic wall at 3 m dead ahead, pitches
# forward in Loiter, and prints the closest approach for YOU to judge against
# the 1.2 m margin.
#
# CAVEAT worth knowing before you read the number: the synthetic obstacle is
# BODY-FIXED at a constant 3 m, so it moves with the drone as it advances rather
# than sitting at a world point. Closest approach is therefore measured against
# a frozen snapshot of where the wall was when the push began. A truly static
# stop test wants a world-fixed obstacle (Gazebo, Stage 6) — this is the
# limitation of testing avoidance with the synthetic publisher. Read the number
# with that in mind.
#
# WARNING: kills any running SITL/MAVROS first. Usage: scripts/run_stage2_avoidance.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
WS="${WS:-${AEROTHON_ROOT:-$HOME/AeroTHON26}/ros2_ws}"
CTRL_URL="tcp:127.0.0.1:5760"
FCU_URL="tcp://127.0.0.1:5762"
PARAM="$PKG/params/stage2_simple_avoidance.param"
RESULT="/tmp/stage2_result.csv"
PIDS=()

log(){ echo -e "\n=== $* ==="; }
cleanup(){ log "cleanup"; for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
           pkill -9 -f "sim_vehicle|arducopter|mavros_node|synthetic_sector_node" 2>/dev/null; }
trap cleanup EXIT
fail(){ echo "FATAL: $*" >&2; exit 1; }

# ROS setup scripts reference unset vars; disable nounset while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

log "0. kill existing SITL/MAVROS/nodes"
pkill -9 -f "sim_vehicle|arducopter|mavros_node|mavproxy|synthetic_sector_node" 2>/dev/null
sleep 3

log "1. launch headless SITL"
( cd "$ARDUPILOT_DIR" && python3 Tools/autotest/sim_vehicle.py -v ArduCopter \
    --no-mavproxy --no-rebuild -I0 >/tmp/sitl.log 2>&1 ) &
PIDS+=($!)
for i in $(seq 1 90); do (echo >/dev/tcp/127.0.0.1/5760) 2>/dev/null && break; sleep 1; done
(echo >/dev/tcp/127.0.0.1/5760) 2>/dev/null || fail "SITL never opened 5760 (see /tmp/sitl.log)"
sleep 5

# No reboot: AVOID_* are live params and PRX1_TYPE=2 is already active in the
# eeprom. load-params reads each value back, so a param that did not take shows
# as MISMATCH / UNKNOWN-NO-ACK right here (this is where AVOID_BEHAVE surfaces
# if it is rejected as unknown, exactly like PRX1_YAW_CORR was). NOT --strict:
# we want to SEE a rejection, not abort on it.
# SYSID_MYGCS=250 so the FC accepts our RC override (pitch-forward); ArduPilot
# ignores RC override from any sysid != SYSID_MYGCS.
log "2. load stage2 params + LOG_DISARMED=1 + SYSID_MYGCS=250, VERIFY (no reboot)"
python3 "$HERE/gate_lib.py" --url "$CTRL_URL" load-params \
    --file "$PARAM" --set LOG_DISARMED=1 --set SYSID_MYGCS=250
echo ">>> Read the AVOID_ENABLE / AVOID_MARGIN / AVOID_BEHAVE rows above:"
echo ">>> 'MISMATCH' or 'UNKNOWN/NO-ACK' means that param did not take."

log "3. launch MAVROS, wait connected"
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

log "4. launch synthetic node: wall at 3 m, bearing 0"
ros2 run obstacle_avoidance synthetic_sector_node --ros-args \
    -r /avoidance/scan:=/mavros/obstacle/send \
    -p obstacle_range_m:=3.0 -p obstacle_width_deg:=10.0 \
    -p obstacle_bearing_deg:=0.0 >/tmp/synth.log 2>&1 &
PIDS+=($!)
for i in $(seq 1 30); do ros2 node list 2>/dev/null | grep -q synthetic_sector_node && break; sleep 1; done
ros2 node list 2>/dev/null | grep -q synthetic_sector_node || fail "synthetic node did not start"

log "4b. ASSERT QoS on /mavros/obstacle/send"
INFO="$(ros2 topic info -v /mavros/obstacle/send 2>/dev/null)"
PUBC=$(echo "$INFO" | sed -n 's/.*Publisher count: *\([0-9]*\).*/\1/p')
SUBC=$(echo "$INFO" | sed -n 's/.*Subscription count: *\([0-9]*\).*/\1/p')
RELN=$(echo "$INFO" | grep -c "Reliability: RELIABLE")
echo "publisher=$PUBC subscription=$SUBC reliable_endpoints=$RELN"
[ "$PUBC" = "1" ] && [ "$SUBC" = "1" ] && [ "$RELN" -ge 2 ] || fail "QoS assertion failed"
echo "QoS assertion PASSED"

log "5. arm, loiter, climb 5 m, pitch forward, measure closest approach"
python3 "$HERE/gate_lib.py" --url "$CTRL_URL" stage2-flight \
    --node /synthetic_sector_node --alt 5.0 --range 3.0 --width 10.0 \
    --push-s 15 --out "$RESULT" || fail "flight sequence failed"

echo -e "\nDONE. Result: $RESULT"
echo "Judge closest_approach_m against AVOID_MARGIN 1.2 m — but see the"
echo "body-fixed-obstacle caveat in this script's header before concluding."
