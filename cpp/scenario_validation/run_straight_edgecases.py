#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import glob
import itertools
import math
import os
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# local imports
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)
from roadgen import RoadSpec, centerline, rasterize_corridor, write_bg_png, write_pgm  # type: ignore


def now_stamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def parse_xosc_params(xosc_path: str) -> Dict[str, str]:
    tree = ET.parse(xosc_path)
    root = tree.getroot()
    params: Dict[str, str] = {}
    for pd in root.findall(".//ParameterDeclaration"):
        name = pd.attrib.get("name")
        value = pd.attrib.get("value")
        if name and value is not None:
            params[name] = value
    return params


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


def split_list(s: str) -> List[str]:
    out: List[str] = []
    for p in (s or "").split(","):
        t = p.strip()
        if t:
            out.append(t)
    return out


def flist(params: Dict[str, str], k: str, default: str) -> List[float]:
    vals = split_list(params.get(k, default))
    return [float(v) for v in vals]


def slist(params: Dict[str, str], k: str, default: str) -> List[str]:
    return split_list(params.get(k, default))


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


def vehicle_radius(vehicle_type: str) -> float:
    vt = vehicle_type.lower()
    if vt == "truck":
        return 2.2
    if vt == "bus":
        return 2.0
    return 1.2  # car


def blocker_radii(blocker_type: str, lane_width_m: float) -> Tuple[float, int]:
    bt = blocker_type.lower()
    if bt == "truck":
        return 1.8, 3
    if bt == "barrier":
        return 1.0, 4
    return 1.2, 3


def write_obstacles_csv(path: str, circles: List[Tuple[float, float, float]]) -> None:
    with open(path, "w") as f:
        for x, y, r in circles:
            f.write(f"{x},{y},{r}\n")


def read_state_at_time(nmpc_csv: str, t_event: float) -> Optional[Tuple[float, float, float, float, float, float]]:
    if not os.path.exists(nmpc_csv):
        return None
    best: Optional[Tuple[float, float, float, float, float, float]] = None
    with open(nmpc_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t = float(row["t"])
            except Exception:
                continue
            if t >= t_event:
                best = (float(row["x"]), float(row["y"]), float(row["psi"]), float(row["vx"]), float(row["vy"]), float(row["r"]))
                break
    return best


def read_rrt_success(case_dir: str) -> bool:
    return os.path.exists(os.path.join(case_dir, "rrt_path.csv"))


def read_nodes_count(nodes_csv: str) -> int:
    n = 0
    if not os.path.exists(nodes_csv):
        return 0
    with open(nodes_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for _ in r:
            n += 1
    return n


def read_nmpc_rms_err(nmpc_csv: str) -> float:
    if not os.path.exists(nmpc_csv):
        return float("inf")
    errs: List[float] = []
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
            errs.append(math.hypot(x - xr, y - yr))
    if not errs:
        return float("inf")
    return math.sqrt(sum(e * e for e in errs) / len(errs))


def read_min_clearance(nmpc_csv: str, circles: List[Tuple[float, float, float]]) -> float:
    if not os.path.exists(nmpc_csv):
        return float("inf")
    mn = float("inf")
    with open(nmpc_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                x = float(row["x"])
                y = float(row["y"])
            except Exception:
                continue
            for (ox, oy, rr) in circles:
                mn = min(mn, math.hypot(x - ox, y - oy) - rr)
    return mn


@dataclass
class CaseMetric:
    road_condition: str
    mu_scale: float
    ego_vx: float
    block_time_s: float
    block_dist_m: float
    blocker_type: str
    oncoming_vx: float
    oncoming_init_dist: float
    oncoming_type: str
    phase1_success: int
    phase2_success: int
    phase2_nodes: int
    nmpc_rms_err: float
    min_clearance_m: float
    case_dir: str


def newest_run_dir(root: str) -> Optional[str]:
    runs = sorted(glob.glob(os.path.join(root, "run_*")), key=os.path.getmtime, reverse=True)
    return runs[0] if runs else None


def run_ru_racer(bin_path: str, cfg_path: str, output_root: str) -> Optional[str]:
    os.makedirs(output_root, exist_ok=True)
    subprocess.run([bin_path, "--config", cfg_path], check=False)
    return newest_run_dir(output_root)


def main() -> int:
    ap = argparse.ArgumentParser(description="Edge-case straight scenario sweep: sudden blocker + oncoming vehicle.")
    ap.add_argument("--scenario", required=True, help="Path to .xosc (parameter-only)")
    ap.add_argument("--vehicle-cfg", required=True, help="Base vehicle+planner config (key=value)")
    ap.add_argument("--bin", default="cpp/build/ru_racer", help="Path to ru_racer binary")
    ap.add_argument("--out-root", default="cpp/results/scenario_validation_edgecases", help="Output root for this sweep")
    ap.add_argument("--max-cases", type=int, default=0, help="Optional cap on number of cases (0 = no cap)")
    args = ap.parse_args()

    scenario_path = os.path.abspath(args.scenario)
    vehicle_cfg = os.path.abspath(args.vehicle_cfg)
    bin_path = os.path.abspath(args.bin)
    out_root = os.path.abspath(args.out_root)

    params = parse_xosc_params(scenario_path)
    scenario_name = params.get("scenario_name", os.path.splitext(os.path.basename(scenario_path))[0])

    # Road geometry: 2 lanes => road_width = num_lanes * lane_width
    lane_width = f(params, "lane_width_m", 3.6)
    num_lanes = max(2, i(params, "num_lanes", 2))
    road_width = lane_width * num_lanes
    road_length = f(params, "road_length_m", 140.0)
    ppm = max(5, i(params, "ppm", 10))
    ego_lane_sign = 1.0 if params.get("ego_lane_sign", "+1").strip().startswith("+") else -1.0

    spec = RoadSpec(shape="tangent", width_m=road_width, length_m=road_length)
    cl = centerline(spec)
    data, W, H, xmin, xmax, ymin, ymax = rasterize_corridor(cl, road_width, ppm)

    run_dir = os.path.join(out_root, f"run_{now_stamp()}_{scenario_name}")
    os.makedirs(run_dir, exist_ok=True)

    map_pgm = os.path.join(run_dir, "road_map.pgm")
    map_png = os.path.join(run_dir, "road_map.png")
    write_pgm(map_pgm, W, H, data)
    write_bg_png(map_png, W, H, data)

    origin_x_px = 1.0 - xmin * ppm
    origin_y_px = 1.0 + ymax * ppm

    # Sweeps
    ego_speed_list = flist(params, "ego_speed_list", "10")
    block_time_list = flist(params, "block_time_s_list", "4")
    block_dist_list = flist(params, "block_dist_m_list", "10")
    blocker_type_list = slist(params, "blocker_type_list", "barrier")
    oncoming_speed_list = flist(params, "oncoming_speed_list", "10")
    oncoming_init_dist_list = flist(params, "oncoming_init_dist_list", "60")
    oncoming_type_list = slist(params, "oncoming_type_list", "car")
    road_conditions = slist(params, "road_conditions", "dry,wet")

    goal_x = f(params, "goal_x", road_length - 10.0)

    base_vehicle = read_kv_cfg(vehicle_cfg)
    base_vehicle.pop("mode", None)

    # Base map settings
    base_vehicle["map_file"] = map_pgm
    base_vehicle["map_use_affine"] = "false"
    base_vehicle["map_pixels_per_meter"] = str(ppm)
    base_vehicle["map_origin_x_px"] = f"{origin_x_px:.6f}"
    base_vehicle["map_origin_y_px"] = f"{origin_y_px:.6f}"
    base_vehicle["map_free_threshold"] = "0"
    base_vehicle["map_invert"] = "false"
    base_vehicle["map_vx_min"] = "0.0"

    # Bounds aligned to map
    base_vehicle["xmin_x"] = f"{xmin:.6f}"
    base_vehicle["xmax_x"] = f"{xmax:.6f}"
    base_vehicle["xmin_y"] = f"{ymin:.6f}"
    base_vehicle["xmax_y"] = f"{ymax:.6f}"

    # Start pose in ego lane, goal in ego lane
    start_y = ego_lane_sign * (lane_width * 0.5)
    goal_y = start_y

    metrics: List[CaseMetric] = []

    combos = list(
        itertools.product(
            road_conditions,
            ego_speed_list,
            block_time_list,
            block_dist_list,
            blocker_type_list,
            oncoming_speed_list,
            oncoming_init_dist_list,
            oncoming_type_list,
        )
    )
    if args.max_cases and args.max_cases > 0:
        combos = combos[: args.max_cases]

    for case_idx, (cond, ego_vx, t_block, d_block, blocker_type, v_on, d_on0, on_type) in enumerate(combos):
        mu = ROAD_MU.get(cond.lower(), 1.0)

        case_name = f"case_{case_idx:04d}_{cond}_ego{ego_vx:g}_tb{t_block:g}_db{d_block:g}_on{v_on:g}_{on_type}_{blocker_type}"
        case_dir = os.path.join(run_dir, case_name)
        os.makedirs(case_dir, exist_ok=True)

        # ---------- Phase 1: no obstacles, run to get ego state at t_block ----------
        cfg1 = dict(base_vehicle)
        cfg1["output_root"] = os.path.join(case_dir, "tmp_phase1")
        cfg1["seed"] = str(1000 + case_idx)
        cfg1["friction_mu_scale_f0"] = str(mu)
        cfg1["friction_mu_scale_r0"] = str(mu)
        cfg1["map_vx_max"] = str(max(6.0, ego_vx * 3.0))
        cfg1.pop("obstacles_csv", None)

        cfg1["start_x"] = "0.0"
        cfg1["start_y"] = str(start_y)
        cfg1["start_psi"] = "0.0"
        cfg1["start_vx"] = str(ego_vx)
        cfg1["start_vy"] = "0.0"
        cfg1["start_r"] = "0.0"
        cfg1["goal_x"] = str(goal_x)
        cfg1["goal_y"] = str(goal_y)
        cfg1["goal_psi"] = "0.0"
        cfg1["goal_vx"] = str(ego_vx)
        cfg1["goal_vy"] = "0.0"
        cfg1["goal_r"] = "0.0"

        cfg1_path = os.path.join(case_dir, "phase1.cfg")
        write_kv_cfg(cfg1_path, cfg1)
        r1 = run_ru_racer(bin_path, cfg1_path, cfg1["output_root"])
        phase1_ok = 0
        x_event = None
        if r1:
            # move phase1 run folder to stable location
            phase1_dir = os.path.join(case_dir, "phase1")
            if os.path.exists(phase1_dir):
                shutil.rmtree(phase1_dir)
            os.rename(r1, phase1_dir)
            phase1_ok = 1 if read_rrt_success(phase1_dir) else 0
            x_event = read_state_at_time(os.path.join(phase1_dir, "nmpc_exec.csv"), t_block)

        if x_event is None:
            # Can't proceed; log failure
            metrics.append(
                CaseMetric(
                    road_condition=cond,
                    mu_scale=mu,
                    ego_vx=ego_vx,
                    block_time_s=t_block,
                    block_dist_m=d_block,
                    blocker_type=blocker_type,
                    oncoming_vx=v_on,
                    oncoming_init_dist=d_on0,
                    oncoming_type=on_type,
                    phase1_success=phase1_ok,
                    phase2_success=0,
                    phase2_nodes=0,
                    nmpc_rms_err=float("inf"),
                    min_clearance_m=float("inf"),
                    case_dir=case_dir,
                )
            )
            continue

        ex, ey, epsi, evx, evy, er = x_event

        # ---------- Phase 2: sudden blocker + oncoming vehicle as obstacles ----------
        # Blocker is placed in ego lane at distance d_block ahead of ego at event time.
        x_block = ex + d_block
        y_block = start_y
        br, bcount = blocker_radii(blocker_type, lane_width)
        circles: List[Tuple[float, float, float]] = []
        # spread circles across lane width to "fill" the lane
        for j in range(bcount):
            t = (j / max(1, bcount - 1)) - 0.5
            circles.append((x_block, y_block + t * (lane_width * 0.8), br))

        # Oncoming vehicle predicted position at event time
        # Oncoming starts ahead at x=d_on0 and moves toward ego (negative x direction).
        x_on = d_on0 - v_on * t_block
        y_on = -start_y
        circles.append((x_on, y_on, vehicle_radius(on_type)))

        obs_csv = os.path.join(case_dir, "obstacles_phase2.csv")
        write_obstacles_csv(obs_csv, circles)

        cfg2 = dict(base_vehicle)
        cfg2["output_root"] = os.path.join(case_dir, "tmp_phase2")
        cfg2["seed"] = str(2000 + case_idx)
        cfg2["friction_mu_scale_f0"] = str(mu)
        cfg2["friction_mu_scale_r0"] = str(mu)
        cfg2["map_vx_max"] = str(max(6.0, ego_vx * 3.0))
        cfg2["obstacles_csv"] = obs_csv

        cfg2["start_x"] = str(ex)
        cfg2["start_y"] = str(ey)
        cfg2["start_psi"] = str(epsi)
        cfg2["start_vx"] = str(max(0.5, evx))
        cfg2["start_vy"] = str(evy)
        cfg2["start_r"] = str(er)
        cfg2["goal_x"] = str(goal_x)
        cfg2["goal_y"] = str(goal_y)
        cfg2["goal_psi"] = "0.0"
        cfg2["goal_vx"] = str(ego_vx)
        cfg2["goal_vy"] = "0.0"
        cfg2["goal_r"] = "0.0"

        cfg2_path = os.path.join(case_dir, "phase2.cfg")
        write_kv_cfg(cfg2_path, cfg2)
        r2 = run_ru_racer(bin_path, cfg2_path, cfg2["output_root"])
        phase2_ok = 0
        nodes2 = 0
        rms_err = float("inf")
        clr = float("inf")
        if r2:
            phase2_dir = os.path.join(case_dir, "phase2")
            if os.path.exists(phase2_dir):
                shutil.rmtree(phase2_dir)
            os.rename(r2, phase2_dir)
            phase2_ok = 1 if read_rrt_success(phase2_dir) else 0
            nodes2 = read_nodes_count(os.path.join(phase2_dir, "rrt_nodes.csv"))
            rms_err = read_nmpc_rms_err(os.path.join(phase2_dir, "nmpc_exec.csv"))
            clr = read_min_clearance(os.path.join(phase2_dir, "nmpc_exec.csv"), circles)

        metrics.append(
            CaseMetric(
                road_condition=cond,
                mu_scale=mu,
                ego_vx=ego_vx,
                block_time_s=t_block,
                block_dist_m=d_block,
                blocker_type=blocker_type,
                oncoming_vx=v_on,
                oncoming_init_dist=d_on0,
                oncoming_type=on_type,
                phase1_success=phase1_ok,
                phase2_success=phase2_ok,
                phase2_nodes=nodes2,
                nmpc_rms_err=rms_err,
                min_clearance_m=clr,
                case_dir=case_dir,
            )
        )

        print(
            f"[{case_idx+1}/{len(combos)}] cond={cond} mu={mu} ego_vx={ego_vx} t_block={t_block} d_block={d_block} on={v_on}@{d_on0} type={on_type} "
            f"phase2_success={phase2_ok}"
        )

    # Write summary CSV
    sum_csv = os.path.join(run_dir, "edgecase_summary.csv")
    with open(sum_csv, "w", newline="") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(
            [
                "road_condition",
                "mu_scale",
                "ego_vx",
                "block_time_s",
                "block_dist_m",
                "blocker_type",
                "oncoming_vx",
                "oncoming_init_dist",
                "oncoming_type",
                "phase1_success",
                "phase2_success",
                "phase2_nodes",
                "nmpc_rms_err",
                "min_clearance_m",
                "case_dir",
            ]
        )
        for m in metrics:
            w.writerow(
                [
                    m.road_condition,
                    m.mu_scale,
                    m.ego_vx,
                    m.block_time_s,
                    m.block_dist_m,
                    m.blocker_type,
                    m.oncoming_vx,
                    m.oncoming_init_dist,
                    m.oncoming_type,
                    m.phase1_success,
                    m.phase2_success,
                    m.phase2_nodes,
                    m.nmpc_rms_err,
                    m.min_clearance_m,
                    m.case_dir,
                ]
            )

    print(f"\nWrote: {sum_csv}")
    print(f"Wrote: {map_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

