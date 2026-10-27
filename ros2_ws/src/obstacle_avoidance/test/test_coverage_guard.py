# test_coverage_guard.py — guards the timeout-ordering invariant.
#
# The guard's stale cutoff (depth_stale_timeout_s) must sit clearly ahead of the
# builder's scan watchdog (depth_watchdog_timeout_s) so failures log in a fixed
# order. These tests fail at CI/import time if the SHIPPED config ever drifts
# into a bad ordering, and pin the check function itself.

import os

import pytest

from obstacle_avoidance.coverage_guard_node import (
    check_timeout_ordering, GUARD_TIMEOUT_MARGIN_S)

yaml = pytest.importorskip('yaml')

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'avoidance.yaml')


def _config_params():
    with open(CONFIG) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def test_shipped_config_satisfies_ordering():
    # This is the "fails at startup if the config is wrong" gate.
    p = _config_params()
    stale = p['depth_stale_timeout_s']
    watchdog = p['depth_watchdog_timeout_s']
    check_timeout_ordering(stale, watchdog)          # raises if violated
    assert watchdog - stale >= GUARD_TIMEOUT_MARGIN_S


def test_ordering_rejects_insufficient_margin():
    # Current-values case from the spec: 0.40 + 0.15 > 0.50 must fail.
    with pytest.raises(ValueError):
        check_timeout_ordering(0.40, 0.50)
    # 100 ms of separation (one 10 Hz frame) is explicitly not enough.
    with pytest.raises(ValueError):
        check_timeout_ordering(0.40, 0.50, margin_s=GUARD_TIMEOUT_MARGIN_S)
    with pytest.raises(ValueError):
        check_timeout_ordering(0.40, 0.549)


def test_ordering_accepts_sufficient_margin():
    check_timeout_ordering(0.40, 0.55)               # exactly the margin: ok
    check_timeout_ordering(0.40, 0.60)               # the shipped value: ok


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
