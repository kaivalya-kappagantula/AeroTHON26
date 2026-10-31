#!/bin/bash
# pi_check.sh — test the OAK-D + Pi Camera 1.3 feeds and the Pi 5's capacity
# (tools/pi_check.py). On the drone, PROPS OFF:  ./tools/pi_check.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/env.sh" >/dev/null
exec python3 "$ROOT/tools/pi_check.py" "$@"
