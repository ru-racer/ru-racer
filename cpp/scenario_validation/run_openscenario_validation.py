#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Tuple

# Local import (no package install required)
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)
from roadgen import RoadSpec, centerline, rasterize_corridor, write_bg_png, write_pgm  # type: ignore


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


def parse_xosc_params(xosc_path: str) -> Dict[str, str]:
    tree = ET.parse(xosc_path)
    root = tree.getroot()
    # Find ParameterDeclarations anywhere
    params: Dict[str, str] = {}
    for pd in root.findall(".//ParameterDeclaration"):
        name = pd.attrib.get("name")
        value = pd.attrib.get("value")
        if name and value is not None:
            params[name] = value
    return params


def split_list(s: str) -> List[str]:
    out: List[str] = []
    for p in (s or "").split(","):
        t = p.strip()
        if t:
            out.append(t)
    return out


def f(params: Dict[str, str], k: str, default: float) -> float:
    try:
        return float(params.get(k, default))
    except Exception:
        return float(default)


def i(params: Dict[str, str], k: str, default: int) -> int:
    try:
        return int(float(params.get(k, default)))
    except Exception:
        return int(default)


ROAD_MU = {
    "dry": 1.0,
    "wet": 0.7,
    "snow": 0.4,
    "ice": 0.2,
    "gravel": 0.5,
}


@dataclass
class RunMetric:
    road_condition: str
    mu_scale: float
    success: int
    nodes: int
    best_cost: float
    nmpc_final_pos_err: float
    nmpc_rms_pos_err: float


def read_rrt_goal_success(rrt_path_csv: str) -> bool:
    return os.path.exists(rrt_path_csv)


def read_nodes_count(nodes_csv: str) -> int:
    n = 0
    if not os.path.exists(nodes_csv):
        return 0
    with open(nodes_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for _ in r:
            n += 1
    return n


def read_best_cost_from_nodes(nodes_csv: str, goal_x: float, goal_y: float, delta_goal: float) -> float:
    # approx: take min cum_cost among nodes within delta_goal
    best = float("inf")
    if not os.path.exists(nodes_csv):
        return best
    with open(nodes_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                x = float(row["x"])
                y = float(row["y"])
                c = float(row["cum_cost"])
            except Exception:
                continue
            if math.hypot(x - goal_x, y - goal_y) <= delta_goal:
                best = min(best, c)
    return best


def read_nmpc_errors(nmpc_csv: str) -> Tuple[float, float]:
    if not os.path.exists(nmpc_csv):
        return float("inf"), float("inf")
    errs: List[float] = []
    last = float("inf")
    with open(nmpc_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                x = float(row["x"])
                y = float(row["y"])
                xr = float(row["x_ref"])
                yr = float(row["y_ref"])
            except Exception:
                continue
            e = math.hypot(x - xr, y - yr)
            errs.append(e)
            last = e
    if not errs:
        return float("inf"), float("inf")
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    return last, rms


def main() -> int:
    ap = argparse.ArgumentParser(description="Scenario-based validation runner (OpenSCENARIO parameter-only).")
    ap.add_argument("--scenario", required=True, help="Path to .xosc scenario")
    ap.add_argument("--vehicle-cfg", required=True, help="Base vehicle+planner config (key=value)")
    ap.add_argument("--bin", default="cpp/build/ru_racer", help="Path to ru_racer binary")
    ap.add_argument("--out-root", default="cpp/results/scenario_validation", help="Output root")
    args = ap.parse_args()

    scenario_path = os.path.abspath(args.scenario)
    vehicle_cfg = os.path.abspath(args.vehicle_cfg)
    bin_path = os.path.abspath(args.bin)
    out_root = os.path.abspath(args.out_root)

    if not os.path.exists(bin_path):
        print(f"ERROR: missing binary: {bin_path}")
        return 2

    params = parse_xosc_params(scenario_path)
    spec = RoadSpec(
        shape=params.get("road_shape", "tangent"),
        width_m=f(params, "road_width_m", 8.0),
        length_m=f(params, "road_length_m", 120.0),
        radius_m=f(params, "road_radius_m", 30.0),
        angle_deg=f(params, "road_angle_deg", 90.0),
        roundabout_radius_m=f(params, "roundabout_radius_m", 25.0),
        s_curve_radius_m=f(params, "s_curve_radius_m", 35.0),
        s_curve_angle_deg=f(params, "s_curve_angle_deg", 45.0),
    )
    ppm = max(5, i(params, "ppm", 10))
    scenario_name = params.get("scenario_name", os.path.splitext(os.path.basename(scenario_path))[0])

    # Geometry
    cl = centerline(spec)
    data, W, H, xmin, xmax, ymin, ymax = rasterize_corridor(cl, spec.width_m, ppm)

    run_dir = os.path.join(out_root, f"run_{now_stamp()}_{scenario_name}")
    os.makedirs(run_dir, exist_ok=True)

    map_pgm = os.path.join(run_dir, "road_map.pgm")
    map_png = os.path.join(run_dir, "road_map.png")
    write_pgm(map_pgm, W, H, data)
    write_bg_png(map_png, W, H, data)

    # OccupancyGrid legacy transform parameters for this generated map
    origin_x_px = 1.0 - xmin * ppm
    origin_y_px = 1.0 + ymax * ppm

    # Start/goal from scenario
    sx = f(params, "start_x", 0.0)
    sy = f(params, "start_y", 0.0)
    spsi = f(params, "start_psi", 0.0)
    svx = f(params, "start_vx", 12.0)
    gx = f(params, "goal_x", cl[-1][0])
    gy = f(params, "goal_y", cl[-1][1])
    gpsi = f(params, "goal_psi", 0.0)
    gvx = f(params, "goal_vx", svx)

    road_conditions = split_list(params.get("road_conditions", "dry,wet"))

    base = read_kv_cfg(vehicle_cfg)
    # Ensure planner runs (mode=plan is default in C++)
    base.pop("mode", None)
    # Set map
    base["map_file"] = map_pgm
    base["map_use_affine"] = "false"
    base["map_pixels_per_meter"] = str(ppm)
    base["map_origin_x_px"] = f"{origin_x_px:.6f}"
    base["map_origin_y_px"] = f"{origin_y_px:.6f}"
    base["map_free_threshold"] = "0"
    base["map_invert"] = "false"
    # Map-based vx feasibility limits (avoid small-car default 0..5 on full-size scenarios)
    base["map_vx_min"] = "0.0"
    base["map_vx_max"] = str(max(6.0, gvx * 2.0))

    # Bounds aligned to the map
    base["xmin_x"] = f"{xmin:.6f}"
    base["xmax_x"] = f"{xmax:.6f}"
    base["xmin_y"] = f"{ymin:.6f}"
    base["xmax_y"] = f"{ymax:.6f}"

    # Start/goal
    base["start_x"] = str(sx)
    base["start_y"] = str(sy)
    base["start_psi"] = str(spsi)
    base["start_vx"] = str(svx)
    base["start_vy"] = "0.0"
    base["start_r"] = "0.0"
    base["goal_x"] = str(gx)
    base["goal_y"] = str(gy)
    base["goal_psi"] = str(gpsi)
    base["goal_vx"] = str(gvx)
    base["goal_vy"] = "0.0"
    base["goal_r"] = "0.0"

    # Visualization: open-space (no bg in plot_rrt_tree), but we will keep our road_map.png for reference
    base["auto_visualize"] = "true"
    base["viz_bg"] = ""  # keep world plots
    base["viz_bg_transform"] = "none"
    base["viz_show_start_goal"] = "true"

    # Metrics & summary
    metrics: List[RunMetric] = []

    for idx, cond in enumerate(road_conditions):
        mu = ROAD_MU.get(cond.lower(), 1.0)
        out_cfg = dict(base)
        out_cfg["seed"] = str(int(float(base.get("seed", "1234"))) + idx)
        out_cfg["friction_mu_scale_f0"] = str(mu)
        out_cfg["friction_mu_scale_r0"] = str(mu)
        # Write each condition into its own output_root so we can keep results separated.
        case_root = os.path.join(run_dir, f"tmp_{cond}")
        os.makedirs(case_root, exist_ok=True)
        out_cfg["output_root"] = case_root

        cfg_path = os.path.join(run_dir, f"case_{cond}.cfg")
        write_kv_cfg(cfg_path, out_cfg)

        print(f"\n=== Running condition={cond} mu_scale={mu} ===")
        proc = subprocess.run([bin_path, "--config", cfg_path], cwd=os.path.dirname(bin_path), check=False)
        # Binary writes to cpp/results by its own output_root; we override it by setting output_root in vehicle cfg.
        # But in our run we want outputs in a subfolder. We'll set output_root to run_dir for this scenario.
        # If user cfg set something else, ensure override:
        # (We do this by forcing output_root just before write.)
        if proc.returncode not in (0, 1, 2):
            print(f"WARN: non-zero return code: {proc.returncode}")

        # Find latest run_* directory created under case_root
        runs = sorted(glob.glob(os.path.join(case_root, "run_*")), key=os.path.getmtime, reverse=True)
        if not runs:
            print("WARN: could not find output run folder")
            continue
        latest = runs[0]

        # Move run folder into stable case folder name (keeps all artifacts intact).
        case_dir = os.path.join(run_dir, f"out_{cond}")
        if os.path.exists(case_dir):
            # shouldn't happen, but keep safe
            case_dir = os.path.join(run_dir, f"out_{cond}_{idx}")
        os.rename(latest, case_dir)

        success = 1 if read_rrt_goal_success(os.path.join(case_dir, "rrt_path.csv")) else 0
        nodes = read_nodes_count(os.path.join(case_dir, "rrt_nodes.csv"))
        delta_goal = float(out_cfg.get("delta_goal", "1.0"))
        best_cost = read_best_cost_from_nodes(os.path.join(case_dir, "rrt_nodes.csv"), gx, gy, delta_goal)
        last_err, rms_err = read_nmpc_errors(os.path.join(case_dir, "nmpc_exec.csv"))

        metrics.append(
            RunMetric(
                road_condition=cond,
                mu_scale=mu,
                success=success,
                nodes=nodes,
                best_cost=best_cost if math.isfinite(best_cost) else 1e100,
                nmpc_final_pos_err=last_err,
                nmpc_rms_pos_err=rms_err,
            )
        )

    # Write summary CSV
    sum_csv = os.path.join(run_dir, "scenario_summary.csv")
    with open(sum_csv, "w", newline="") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(["road_condition", "mu_scale", "success", "nodes", "best_cost", "nmpc_final_pos_err", "nmpc_rms_pos_err"])
        for m in metrics:
            w.writerow([m.road_condition, m.mu_scale, m.success, m.nodes, m.best_cost, m.nmpc_final_pos_err, m.nmpc_rms_pos_err])
    print(f"\nWrote: {sum_csv}")
    print(f"Wrote: {map_png}")

    # Simple summary plot
    try:
        import matplotlib.pyplot as plt
        import numpy as np

        fig, ax = plt.subplots(1, 3, figsize=(13, 4))
        conds = [m.road_condition for m in metrics]
        mu_s = [m.mu_scale for m in metrics]
        succ_s = [m.success for m in metrics]
        cost_s = [m.best_cost for m in metrics]
        err_s = [m.nmpc_rms_pos_err for m in metrics]

        ax[0].bar(conds, succ_s, color=["seagreen" if s else "crimson" for s in succ_s])
        ax[0].set_title("Success by road condition")
        ax[0].set_ylim(0, 1.05)
        ax[0].grid(True, axis="y", alpha=0.3)

        ax[1].plot(mu_s, cost_s, "o-", color="dodgerblue")
        ax[1].set_title("Planner best_cost vs mu_scale")
        ax[1].set_xlabel("mu_scale")
        ax[1].set_ylabel("best_cost")
        ax[1].grid(True, alpha=0.3)

        ax[2].plot(mu_s, err_s, "o-", color="darkorange")
        ax[2].set_title("NMPC RMS pos err vs mu_scale")
        ax[2].set_xlabel("mu_scale")
        ax[2].set_ylabel("RMS pos err [m]")
        ax[2].grid(True, alpha=0.3)

        fig.tight_layout()
        out_png = os.path.join(run_dir, "scenario_summary.png")
        fig.savefig(out_png, dpi=200)
        plt.close(fig)
        print(f"Wrote: {out_png}")
    except Exception as e:
        print(f"WARN: could not generate summary plot (matplotlib/numpy missing?): {e}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

