# exit_corridor.py — where to look for the exit corridor (mission phase 5).
#
# Pure Python (no ROS), unit-tested in test/test_exit_corridor.py.
#
# The two corridors stand side by side with the takeoff point between them:
# the drone flew out through corridor 1 (the entry corridor) into the arena
# and comes back through the exit corridor. Assumed symmetric: the exit
# corridor is corridor 1's mirror image across the line through the takeoff
# point parallel to corridor 1. So corridor 1's far end (where the drone left
# its walls, recorded at the end of phase 2) mirrored across that line is
# the exit corridor's arena-side entrance — where its blue banner is.
#
# The drone searches from standoff_m further out into the arena from there,
# facing back down the corridors (corridor 1's direction + 180 deg); the
# banner search sweeps +-90 deg around that. Not found -> the same point
# shifted shift_m sideways along the line of corridor entrances, first AWAY
# from corridor 1 (the takeoff point may be nearer one corridor than the
# other), then toward it.
#
# Frame: metres (east, north) from the takeoff point; yaw ENU (rad, CCW from
# East).

import math


def mirror_across(point, axis_yaw):
    """point reflected across the line through the origin along axis_yaw."""
    ux, uy = math.cos(axis_yaw), math.sin(axis_yaw)
    along = point[0] * ux + point[1] * uy
    return (2.0 * along * ux - point[0], 2.0 * along * uy - point[1])


def search_points(c1_exit, axis_yaw, standoff_m, shift_m):
    """Where to search for the blue banner, in order: [(point, facing yaw)].

    c1_exit: corridor 1's far end (east, north) from the takeoff point;
    axis_yaw: corridor 1's direction (into the arena). shift_m <= 0: only the
    predicted point."""
    ux, uy = math.cos(axis_yaw), math.sin(axis_yaw)
    nx, ny = -uy, ux                                    # left of the axis
    mx, my = mirror_across(c1_exit, axis_yaw)
    p0 = (mx + standoff_m * ux, my + standoff_m * uy)
    side = mx * nx + my * ny                            # predicted corridor's side
    away = 1.0 if side >= 0.0 else -1.0
    facing = math.atan2(-uy, -ux)
    pts = [p0]
    if shift_m > 0.0:
        for k in (away, -away):
            pts.append((p0[0] + k * shift_m * nx, p0[1] + k * shift_m * ny))
    return [(p, facing) for p in pts]
