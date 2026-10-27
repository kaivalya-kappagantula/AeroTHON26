import pytest
from statemachine.exceptions import TransitionNotAllowed

from mission_manager.state_machine import MissionSM

FORWARD = ["auto_activate", "airborne", "qr_scanned", "entry_complete", "qr_matched",
           "drop_position_reached", "drop_complete", "c2_complete",
           "home_reached", "touchdown"]


def _run(sm, events):
    for ev in events:
        sm.send(ev)


def test_full_mission_ends_landed():
    sm = MissionSM()
    _run(sm, FORWARD)
    assert sm.state_id == "LANDED"


def test_no_qr_match_skips_drop_and_continues():
    sm = MissionSM()
    _run(sm, ["auto_activate", "airborne", "qr_scanned", "entry_complete",
              "search_exhausted"])
    assert sm.state_id == "CORRIDOR_2_TRANSIT"
    _run(sm, ["c2_complete", "home_reached", "touchdown"])
    assert sm.state_id == "LANDED"
    assert sm.drop_confirmed is False


def test_phase_1_is_takeoff_then_the_reference_qr_in_the_air():
    sm = MissionSM()
    sm.auto_activate()
    assert sm.state_id == "TAKEOFF"
    sm.airborne()
    assert sm.state_id == "SCAN_REFERENCE_QR"
    sm.qr_scanned()
    assert sm.state_id == "ENTRY_CORRIDOR"
    sm.entry_complete()
    assert sm.state_id == "ARENA_SEARCH"


def test_takeoff_test_flight_lands():
    sm = MissionSM()
    _run(sm, ["auto_activate", "takeoff_test_done", "touchdown"])
    assert sm.state_id == "LANDED"


def test_qr_test_flight_lands():
    sm = MissionSM()
    _run(sm, ["auto_activate", "airborne", "qr_test_done", "touchdown"])
    assert sm.state_id == "LANDED"


@pytest.mark.parametrize("event", ["takeoff_test_done", "qr_test_done"])
def test_test_landings_not_from_the_entry_corridor(event):
    sm = MissionSM()
    _run(sm, ["auto_activate", "airborne", "qr_scanned"])
    with pytest.raises(TransitionNotAllowed):
        sm.send(event)


def test_entry_corridor_test_flight_lands():
    # entry_corridor_mission: take off -> entry corridor -> land.
    sm = MissionSM()
    _run(sm, ["auto_activate", "airborne", "qr_scanned", "entry_test_done", "touchdown"])
    assert sm.state_id == "LANDED"


def test_entry_test_done_only_from_the_entry_corridor():
    sm = MissionSM()
    _run(sm, ["auto_activate", "airborne", "qr_scanned", "entry_complete"])
    with pytest.raises(TransitionNotAllowed):
        sm.entry_test_done()


def test_landing_is_a_separate_phase_from_homing():
    # HOMING must not jump straight to LANDED (that disarmed in the air).
    sm = MissionSM()
    _run(sm, FORWARD[:-2])
    assert sm.state_id == "HOMING"
    with pytest.raises(TransitionNotAllowed):
        sm.touchdown()


@pytest.mark.parametrize("n_events", range(1, len(FORWARD)))
def test_pilot_abort_from_every_active_phase(n_events):
    sm = MissionSM()
    _run(sm, FORWARD[:n_events])
    sm.abort()
    assert sm.state_id == "RC_MODE"


@pytest.mark.parametrize("n_events", range(0, len(FORWARD)))
def test_failsafe_from_every_active_phase(n_events):
    sm = MissionSM()
    _run(sm, FORWARD[:n_events])
    sm.failsafe()
    assert sm.state_id == "ABORTED"


def test_terminal_states_accept_nothing():
    sm = MissionSM()
    _run(sm, FORWARD)
    with pytest.raises(TransitionNotAllowed):
        sm.abort()


@pytest.mark.parametrize("found", [True, False])
def test_arena_test_flight_lands_with_or_without_a_match(found):
    sm = MissionSM()
    _run(sm, ["auto_activate", "airborne", "qr_scanned", "entry_complete"]
         + (["qr_matched"] if found else []) + ["arena_test_done", "touchdown"])
    assert sm.state_id == "LANDED"
