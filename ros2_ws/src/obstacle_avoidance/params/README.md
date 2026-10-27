# ArduPilot parameter sets (spec §5)

One `.param` file per stage. Load the set that matches the stage under test.
**Reboot the FC after loading a set that touches `PRX1_TYPE` or `OA_TYPE`**
(they take effect on reboot); `stage0_bench` needs no reboot. Commit gate
results with the exact `.param` file that produced them; a gate result without
its parameters is meaningless.

| File | Stage(s) | Adds |
|---|---|---|
| `stage0_bench.param` | 0 + GPS survey | `LOG_DISARMED=1` only (powered, never armed) |
| `stage1_proximity.param` | 1 | MAVLink proximity input only (display, not action) |
| `stage2_simple_avoidance.param` | 2 | + simple avoidance (Loiter/AltHold test instrument) |
| `stage5_6_corridor.param` | 5, 6 | OA off, `GUID_OPTIONS 0`, simple avoidance for pilot takeover, GCS failsafe = Land |

## Loading

MAVProxy:
```
param load params/stage5_6_corridor.param
reboot
```

MAVROS:
```
ros2 run mavros mavparam load params/stage5_6_corridor.param
```

## Why ArduPilot's path planner is off in the corridors

The corridors are 5 ft (1.524 m) wide with obstacles of 30–40 % of the width.
BendyRuler passes between a wall and an obstacle only when the gap is at least
2 × `OA_MARGIN_MAX`. The free gap is at most ~0.9–1.1 m (obstacle against a
wall) or ~0.45 m each side (centred), so the margin would have to be under
0.45 m — less than the vehicle's half span plus position error — and BendyRuler
cannot fly over an obstacle either. So `OA_TYPE 0`, and `path_planner` flies the
corridor from the OAK-D corridor fit (`corridor_perception_node`): centring,
picking the free side, or climbing over (see `path_planner/corridor_follow.py`).

`GUID_OPTIONS` must be **0**. With bit 6 (64) set, GUIDED position targets go
through the waypoint navigator at `WP_SPD` and ignore `DO_CHANGE_SPEED`.

## AVOID_MARGIN is 0.4

Simple avoidance only acts when the pilot flies (Loiter/AltHold, e.g. after an
RC takeover in a corridor); it is not applied to GUIDED position targets. The
walls are 0.76 m from the centreline, so 0.6 would leave the pilot ~0.3 m of
lateral room. `stage2_simple_avoidance.param` keeps 0.6 for the bench box test.

## Corridor speed is staged

`corridor_follow_speed_ms` (mission_params.yaml) is the speed inside the
corridors, sent with `DO_CHANGE_SPEED`. Stage it on hardware: **0.3** for the
first corridor flight, then 0.5.

## Two subsystems, one trap

`AVOID_*` (simple avoidance) only acts horizontally in AltHold/Loiter. `OA_*`
(BendyRuler) is the AUTO/GUIDED path planner, which this mission does not use.
They read the same proximity data but are independent — confirm which one is
active before tuning.
