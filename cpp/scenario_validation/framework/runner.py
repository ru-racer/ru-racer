"""
Scenario runner: orchestrates map build, planner + controller execution, metrics.
"""

from __future__ import annotations

import glob
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .map_region import MapOutput, build_map_region
from .metrics import Metrics, extract_metrics, evaluate_pass_fail
from .scenario_def import ScenarioDef, ExpectedOutcome


def now_stamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def read_kv_cfg(path: str) -> Dict[str, str]:
    kv: Dict[str, str] = {}
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#") or "=" not in s:
                continue
            k, v = s.split("=", 1)
            kv[k.strip()] = v.strip()
    return kv


def write_kv_cfg(path: str, kv: Dict[str, str]) -> None:
    with open(path, "w") as f:
        for k in sorted(kv.keys()):
            f.write(f"{k}={kv[k]}\n")


ROAD_MU = {
    "dry": 1.0,
    "wet": 0.7,
    "snow": 0.4,
    "ice": 0.2,
    "gravel": 0.5,
}


@dataclass
class RunResult:
    """Result of a single scenario run."""

    scenario_id: str
    scenario_name: str
    road_condition: str
    passed: bool
    failures: List[str] = field(default_factory=list)
    metrics: Optional[Metrics] = None
    run_dir: str = ""
    map_output: Optional[MapOutput] = None


class ScenarioRunner:
    """
    Runs a scenario: builds map, invokes ru_racer binary, extracts metrics, evaluates pass/fail.
    """

    def __init__(
        self,
        bin_path: str = "cpp/build/ru_racer",
        vehicle_cfg_path: Optional[str] = None,
        output_root: str = "cpp/results/scenario_validation",
    ):
        self.bin_path = os.path.abspath(bin_path)
        self.vehicle_cfg_path = vehicle_cfg_path
        self.output_root = os.path.abspath(output_root)

    def run_scenario(
        self,
        scenario: ScenarioDef,
        vehicle_cfg_path: Optional[str] = None,
        road_condition: Optional[str] = None,
    ) -> List[RunResult]:
        """
        Run scenario for each road condition in map_region.

        Returns list of RunResult (one per condition).
        """
        cfg_path = vehicle_cfg_path or self.vehicle_cfg_path
        if not cfg_path or not os.path.exists(cfg_path):
            raise FileNotFoundError(f"Vehicle config not found: {cfg_path}")
        if not os.path.exists(self.bin_path):
            raise FileNotFoundError(f"Binary not found: {self.bin_path}")

        base_cfg = read_kv_cfg(cfg_path)
        base_cfg.pop("mode", None)

        # Build map for first condition (shared geometry)
        conditions = scenario.map_region.road_conditions
        if road_condition:
            conditions = [road_condition]

        run_dir = os.path.join(
            self.output_root,
            f"run_{now_stamp()}_{scenario.id}",
        )
        os.makedirs(run_dir, exist_ok=True)

        map_output = build_map_region(
            scenario.map_region,
            run_dir,
            scenario.id,
        )

        # Merge scenario config
        for k, v in scenario.config_overrides.items():
            base_cfg[k] = str(v)

        base_cfg["map_file"] = map_output.map_pgm_path
        base_cfg["map_use_affine"] = "false"
        base_cfg["map_pixels_per_meter"] = str(map_output.ppm)
        base_cfg["map_origin_x_px"] = f"{map_output.origin_x_px:.6f}"
        base_cfg["map_origin_y_px"] = f"{map_output.origin_y_px:.6f}"
        base_cfg["map_free_threshold"] = "0"
        base_cfg["map_invert"] = "false"
        base_cfg["map_vx_min"] = "0.0"
        base_cfg["map_vx_max"] = str(max(6.0, scenario.goal_state.vx * 2.0))

        base_cfg["xmin_x"] = f"{map_output.xmin:.6f}"
        base_cfg["xmax_x"] = f"{map_output.xmax:.6f}"
        base_cfg["xmin_y"] = f"{map_output.ymin:.6f}"
        base_cfg["xmax_y"] = f"{map_output.ymax:.6f}"

        s = scenario.initial_state
        base_cfg["start_x"] = str(s.x)
        base_cfg["start_y"] = str(s.y)
        base_cfg["start_psi"] = str(s.psi)
        base_cfg["start_vx"] = str(s.vx)
        base_cfg["start_vy"] = str(s.vy)
        base_cfg["start_r"] = str(s.r)

        g = scenario.goal_state
        base_cfg["goal_x"] = str(g.x)
        base_cfg["goal_y"] = str(g.y)
        base_cfg["goal_psi"] = str(g.psi)
        base_cfg["goal_vx"] = str(g.vx)
        base_cfg["goal_vy"] = str(g.vy)
        base_cfg["goal_r"] = str(g.r)

        base_cfg["auto_visualize"] = "true"
        base_cfg["viz_bg"] = ""
        base_cfg["viz_show_start_goal"] = "true"

        delta_goal = float(base_cfg.get("delta_goal", "1.0"))

        results: List[RunResult] = []

        for idx, cond in enumerate(conditions):
            mu = ROAD_MU.get(cond.lower(), 1.0)
            out_cfg = dict(base_cfg)
            out_cfg["seed"] = str(int(float(base_cfg.get("seed", "1234"))) + str(idx))
            out_cfg["friction_mu_scale_f0"] = str(mu)
            out_cfg["friction_mu_scale_r0"] = str(mu)

            case_dir = os.path.join(run_dir, f"out_{cond}")
            os.makedirs(case_dir, exist_ok=True)
            out_cfg["output_root"] = case_dir

            cfg_path_out = os.path.join(run_dir, f"case_{cond}.cfg")
            write_kv_cfg(cfg_path_out, out_cfg)

            print(f"\n=== {scenario.id} | condition={cond} mu={mu} ===")
            proc = subprocess.run(
                [self.bin_path, "--config", cfg_path_out],
                cwd=os.path.dirname(self.bin_path),
                check=False,
            )
            if proc.returncode not in (0, 1, 2):
                print(f"WARN: return code {proc.returncode}")

            # Find latest run_* under case_dir
            runs = sorted(
                glob.glob(os.path.join(case_dir, "run_*")),
                key=os.path.getmtime,
                reverse=True,
            )
            if not runs:
                metrics = Metrics()
                passed, failures = evaluate_pass_fail(metrics, scenario.expected_outcome)
                results.append(
                    RunResult(
                        scenario_id=scenario.id,
                        scenario_name=scenario.name,
                        road_condition=cond,
                        passed=passed,
                        failures=failures + ["No output run folder found"],
                        metrics=metrics,
                        run_dir=case_dir,
                        map_output=map_output,
                    )
                )
                continue

            latest = runs[0]
            metrics = extract_metrics(latest, g.x, g.y, delta_goal)
            passed, failures = evaluate_pass_fail(metrics, scenario.expected_outcome)

            results.append(
                RunResult(
                    scenario_id=scenario.id,
                    scenario_name=scenario.name,
                    road_condition=cond,
                    passed=passed,
                    failures=failures,
                    metrics=metrics,
                    run_dir=latest,
                    map_output=map_output,
                )
            )

        return results
