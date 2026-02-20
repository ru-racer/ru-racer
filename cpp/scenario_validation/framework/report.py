"""
Report generation for scenario validation runs.
"""

from __future__ import annotations

import csv
import os
from typing import List

from .runner import RunResult


def write_summary_csv(results: List[RunResult], path: str) -> None:
    """Write scenario_summary.csv with all metrics."""
    if not results:
        return
    headers = [
        "scenario_id",
        "scenario_name",
        "road_condition",
        "passed",
        "planner_success",
        "planner_nodes",
        "planner_best_cost",
        "nmpc_final_pos_err",
        "nmpc_rms_pos_err",
        "failures",
    ]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        for r in results:
            m = r.metrics
            failures_str = "; ".join(r.failures) if r.failures else ""
            w.writerow([
                r.scenario_id,
                r.scenario_name,
                r.road_condition,
                1 if r.passed else 0,
                1 if (m and m.planner_success) else 0,
                m.planner_nodes if m else 0,
                m.planner_best_cost if m else float("inf"),
                m.nmpc_final_pos_err if m else float("inf"),
                m.nmpc_rms_pos_err if m else float("inf"),
                failures_str,
            ])
    print(f"Wrote: {path}")


def print_summary(results: List[RunResult]) -> None:
    """Print human-readable summary to stdout."""
    passed = sum(1 for r in results if r.passed)
    total = len(results)
    print(f"\n--- Scenario Validation Summary ---")
    print(f"Passed: {passed}/{total}")
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"  [{status}] {r.scenario_id} ({r.road_condition})")
        if r.failures:
            for f in r.failures:
                print(f"       - {f}")
