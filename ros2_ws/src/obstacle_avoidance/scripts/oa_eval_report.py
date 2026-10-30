#!/usr/bin/env python3
"""Summarise oa_eval results (results.jsonl from oa_eval.sh / oa_eval_node).

One row per (planner, scenario, dropout, label, FC parameters): clean runs out
of total, the failure reasons, worst and median clearances, median time. The
pass bar from docs/OA_EVALUATION.md is >= 9 of 10 clean.

    python3 scripts/oa_eval_report.py ~/oa_eval/20261029/results.jsonl [...]
"""

import json
import statistics
import sys
from collections import Counter, defaultdict

PASS_RATE = 0.9


def _key(r):
    t = r.get('tags', {})
    return (r.get('planner', '?'), t.get('scenario', '?'), t.get('dropout', 0.0),
            t.get('label', ''), t.get('fc', ''))


def _num(vals):
    return [v for v in vals if isinstance(v, (int, float))]


def main(paths):
    groups = defaultdict(list)
    for p in paths:
        with open(p) as f:
            for line in f:
                if line.strip():
                    r = json.loads(line)
                    groups[_key(r)].append(r)
    if not groups:
        print('no results')
        return 1
    for (planner, scenario, dropout, label, fc), rs in sorted(groups.items()):
        clean = sum(r['clean'] for r in rs)
        reasons = Counter(x for r in rs for x in r.get('reasons', []))
        walls = _num(r.get('min_wall_clearance_m') for r in rs)
        obst = _num(r.get('min_obstacle_clearance_m') for r in rs)
        times = _num(r.get('time_s') for r in rs if r['clean'])
        spacings = sorted({r.get('tags', {}).get('spacing_m') for r in rs} - {None})
        rate = clean / len(rs)
        verdict = ('PASS' if rate >= PASS_RATE and len(rs) >= 5 else
                   'pass (too few runs)' if rate >= PASS_RATE else 'FAIL')
        print(f"\n{planner:7s} {scenario:9s} dropout {dropout:.2f}"
              + (f"  [{label}]" if label else ''))
        if fc:
            print(f"  FC: {fc}")
        if scenario == 'staggered':
            print(f"  spacing: {spacings}")
        print(f"  clean {clean}/{len(rs)} ({100 * rate:.0f} %) -> {verdict}")
        if reasons:
            print('  failures: ' + ', '.join(f'{k} x{v}' for k, v in reasons.most_common()))
        if walls:
            print(f"  wall clearance   min {min(walls):+.2f} m, median {statistics.median(walls):+.2f} m")
        if obst:
            print(f"  obstacle clear.  min {min(obst):+.2f} m, median {statistics.median(obst):+.2f} m")
        if times:
            print(f"  time (clean runs) median {statistics.median(times):.1f} s")
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    sys.exit(main(sys.argv[1:]))
