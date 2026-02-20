"""
Metrics extraction from planner/controller outputs and pass/fail evaluation.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class Metrics:
    """Collected metrics from a scenario run."""

    planner_success: bool = False
    planner_nodes: int = 0
    planner_best_cost: float = float("inf")
    nmpc_final_pos_err: float = float("inf")
    nmpc_rms_pos_err: float = float("inf")
    nmpc_max_pos_err: float = float("inf")
    extra: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, any]:
        d = {
            "planner_success": self.planner_success,
            "planner_nodes": self.planner_nodes,
            "planner_best_cost": self.planner_best_cost,
            "nmpc_final_pos_err": self.nmpc_final_pos_err,
            "nmpc_rms_pos_err": self.nmpc_rms_pos_err,
            "nmpc_max_pos_err": self.nmpc_max_pos_err,
        }
        d.update(self.extra)
        return d


def _read_csv_rows(path: str) -> List[Dict[str, str]]:
    if not os.path.exists(path):
        return []
    rows: List[Dict[str, str]] = []
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(dict(row))
    return rows


def extract_metrics(
    run_dir: str,
    goal_x: float,
    goal_y: float,
    delta_goal: float = 1.0,
) -> Metrics:
    """
    Extract metrics from a scenario run directory.

    Expects: rrt_path.csv, rrt_nodes.csv, nmpc_exec.csv
    """
    m = Metrics()

    # Planner success: path found
    rrt_path = os.path.join(run_dir, "rrt_path.csv")
    m.planner_success = os.path.exists(rrt_path)

    # Planner nodes
    nodes_path = os.path.join(run_dir, "rrt_nodes.csv")
    m.planner_nodes = len(_read_csv_rows(nodes_path))

    # Best cost (min cum_cost among nodes within delta_goal of goal)
    best = float("inf")
    for row in _read_csv_rows(nodes_path):
        try:
            x = float(row.get("x", 0))
            y = float(row.get("y", 0))
            c = float(row.get("cum_cost", float("inf")))
        except (ValueError, TypeError):
            continue
        if math.hypot(x - goal_x, y - goal_y) <= delta_goal:
            best = min(best, c)
    m.planner_best_cost = best if math.isfinite(best) else float("inf")

    # NMPC tracking errors
    nmpc_path = os.path.join(run_dir, "nmpc_exec.csv")
    errs: List[float] = []
    last_err = float("inf")
    for row in _read_csv_rows(nmpc_path):
        try:
            x = float(row.get("x", 0))
            y = float(row.get("y", 0))
            xr = float(row.get("x_ref", 0))
            yr = float(row.get("y_ref", 0))
        except (ValueError, TypeError):
            continue
        e = math.hypot(x - xr, y - yr)
        errs.append(e)
        last_err = e
    if errs:
        m.nmpc_final_pos_err = last_err
        m.nmpc_rms_pos_err = math.sqrt(sum(e * e for e in errs) / len(errs))
        m.nmpc_max_pos_err = max(errs)
    else:
        m.nmpc_final_pos_err = float("inf")
        m.nmpc_rms_pos_err = float("inf")
        m.nmpc_max_pos_err = float("inf")

    return m


def evaluate_pass_fail(
    metrics: Metrics,
    expected: "ExpectedOutcome",
) -> Tuple[bool, List[str]]:
    """
    Evaluate pass/fail against expected outcome.

    Returns (passed, list of failure reasons).
    """
    failures: List[str] = []

    if expected.planner_success and not metrics.planner_success:
        failures.append("planner_success: expected True, got False")

    if expected.controller_reaches_goal:
        if metrics.nmpc_final_pos_err == float("inf"):
            failures.append("controller_reaches_goal: no NMPC execution data")
        elif metrics.nmpc_final_pos_err > expected.metrics.get("nmpc_final_pos_err_max", float("inf")):
            failures.append(
                f"nmpc_final_pos_err {metrics.nmpc_final_pos_err:.4f} > "
                f"max {expected.metrics.get('nmpc_final_pos_err_max', float('inf'))}"
            )

    for key, max_val in expected.metrics.items():
        if not key.endswith("_max"):
            continue
        base_key = key[:-4]  # remove _max
        val = getattr(metrics, base_key, metrics.extra.get(base_key))
        if val is None:
            continue
        try:
            v = float(val)
        except (TypeError, ValueError):
            continue
        if v > max_val:
            failures.append(f"{base_key}: {v:.4f} > max {max_val}")

    return len(failures) == 0, failures
