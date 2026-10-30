# Mission 2 state machine
#
# This class only TRACKS mission phase and enforces legal transitions. It does
# no I/O and never blocks: the flight actions for each phase live in
# mission_node._run_mission (a worker thread). Keeping blocking work out of
# on_enter_* callbacks matters because transitions are also triggered from
# ROS callbacks, and a blocking callback would stall the executor.
#
# Phase flow:
#   IDLE -> TAKEOFF -> SCAN_REFERENCE_QR -> ENTRY_CORRIDOR -> ARENA_SEARCH
#        -> TARGET_FOUND -> DROPPING -> CORRIDOR_2_TRANSIT -> HOMING -> LANDING
#        -> LANDED
#   ARENA_SEARCH -> CORRIDOR_2_TRANSIT  (search_exhausted: no match and no
#                                        QR seen to drop on at random)
#   TARGET_FOUND / DROPPING -> CORRIDOR_2_TRANSIT  (drop_failed: the drop
#                                        phase failed — moving on)
# PHASE FAILSAFE: a failure in a phase (not the pilot, FC, battery or a dead
# sensor) moves the mission on to the next phase — mission_node._phase.
# Test flights land straight after the phase they test:
#   TAKEOFF -> LANDING            (takeoff_test_done: takeoff_test)
#   SCAN_REFERENCE_QR -> LANDING  (qr_test_done: qr_scan_mission_node)
#   ENTRY_CORRIDOR -> LANDING     (entry_test_done: entry_corridor_mission)
#   ARENA_SEARCH / TARGET_FOUND -> LANDING  (arena_test_done: arena_search_mission)
#   CORRIDOR_2_TRANSIT -> LANDING (exit_test_done: exit_corridor_mission)
#   DROPPING -> LANDING           (drop_test_done: payload_drop_mission,
#                                  arena_search_mission arena_test_drop)
#
# Phase 1 = TAKEOFF (gradual, to cruise altitude) + SCAN_REFERENCE_QR (read
# in the air over the launch point and stored). qr_scanned leaves the QR
# phase whether or not the QR was read: without it the mission carries on
# with no target. ENTRY_CORRIDOR = descend, green banner search, line-up on
# the walls under it, corridor flown through, climb to cruise.
# CORRIDOR_2_TRANSIT = phase 5, the exit corridor: to where it should be
# (corridor 1 mirrored), blue banner search, through with obstacles passed.
#
# Exits from any active phase:
#   abort    -> RC_MODE  (pilot flipped the auto switch low; pilot flies)
#   failsafe -> ABORTED  (companion-detected failure: LAND / stay on ground)

import time

from statemachine import StateMachine, State


class MissionSM(StateMachine):

    IDLE               = State(initial=True)

    SCAN_REFERENCE_QR  = State()
    TAKEOFF            = State()
    ENTRY_CORRIDOR     = State()
    ARENA_SEARCH       = State()
    TARGET_FOUND       = State()
    DROPPING           = State()
    CORRIDOR_2_TRANSIT = State()
    HOMING             = State()
    LANDING            = State()

    LANDED             = State(final=True)
    RC_MODE            = State(final=True)   # pilot has control, mission over
    ABORTED            = State(final=True)   # companion failsafe, mission over

    # forward sequence
    auto_activate         = IDLE.to(TAKEOFF)
    airborne              = TAKEOFF.to(SCAN_REFERENCE_QR)
    qr_scanned            = SCAN_REFERENCE_QR.to(ENTRY_CORRIDOR)
    entry_complete        = ENTRY_CORRIDOR.to(ARENA_SEARCH)
    qr_matched            = ARENA_SEARCH.to(TARGET_FOUND)
    search_exhausted      = ARENA_SEARCH.to(CORRIDOR_2_TRANSIT)
    drop_position_reached = TARGET_FOUND.to(DROPPING)
    drop_complete         = DROPPING.to(CORRIDOR_2_TRANSIT)
    drop_failed           = TARGET_FOUND.to(CORRIDOR_2_TRANSIT) | DROPPING.to(CORRIDOR_2_TRANSIT)
    c2_complete           = CORRIDOR_2_TRANSIT.to(HOMING)
    home_reached          = HOMING.to(LANDING)
    touchdown             = LANDING.to(LANDED)

    # test flights only (one phase, then land)
    takeoff_test_done     = TAKEOFF.to(LANDING)
    qr_test_done          = SCAN_REFERENCE_QR.to(LANDING)
    entry_test_done       = ENTRY_CORRIDOR.to(LANDING)
    arena_test_done       = ARENA_SEARCH.to(LANDING) | TARGET_FOUND.to(LANDING)
    exit_test_done        = CORRIDOR_2_TRANSIT.to(LANDING)
    drop_test_done        = DROPPING.to(LANDING)

    abort = (
        IDLE.to(RC_MODE) |
        SCAN_REFERENCE_QR.to(RC_MODE) |
        TAKEOFF.to(RC_MODE) |
        ENTRY_CORRIDOR.to(RC_MODE) |
        ARENA_SEARCH.to(RC_MODE) |
        TARGET_FOUND.to(RC_MODE) |
        DROPPING.to(RC_MODE) |
        CORRIDOR_2_TRANSIT.to(RC_MODE) |
        HOMING.to(RC_MODE) |
        LANDING.to(RC_MODE)
    )
    failsafe = (
        IDLE.to(ABORTED) |
        SCAN_REFERENCE_QR.to(ABORTED) |
        TAKEOFF.to(ABORTED) |
        ENTRY_CORRIDOR.to(ABORTED) |
        ARENA_SEARCH.to(ABORTED) |
        TARGET_FOUND.to(ABORTED) |
        DROPPING.to(ABORTED) |
        CORRIDOR_2_TRANSIT.to(ABORTED) |
        HOMING.to(ABORTED) |
        LANDING.to(ABORTED)
    )

    def __init__(self, logger=None):
        self._logger = logger
        self.mission_target_qr = None
        self.drop_confirmed = False
        self._start_time = time.time()
        super().__init__()

    @property
    def state_id(self) -> str:
        return str(self.current_state_value)

    def _log(self, text: str):
        line = f"[{time.time() - self._start_time:.1f}s] {text}"
        if self._logger is not None:
            self._logger.info(line)
        else:
            print(line)

    def after_transition(self, event, source, target):
        self._log(f"[SM] {source.id} --{event}--> {target.id}")

    def on_enter_DROPPING(self):
        self.drop_confirmed = False

