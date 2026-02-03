#!/usr/bin/env python3

import argparse
import csv
import glob
import os
from typing import Dict, List, Optional, Tuple


def read_csv(path: str) -> Dict[str, List[float]]:
    cols: Dict[str, List[float]] = {}
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            for k, v in row.items():
                if k not in cols:
                    cols[k] = []
                if k in ("scenario",):
                    # keep as NaN-ish placeholder
                    continue
                try:
                    cols[k].append(float(v))
                except Exception:
                    pass
    return cols


def read_kv_cfg(path: str) -> Dict[str, str]:
    kv: Dict[str, str] = {}
    if not path or not os.path.exists(path):
        return kv
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#") or "=" not in s:
                continue
            k, v = s.split("=", 1)
            kv[k.strip()] = v.strip()
    return kv


def find_config_used(start_dir: str) -> Optional[str]:
    d = os.path.abspath(start_dir)
    for _ in range(5):
        cand = os.path.join(d, "config_used.cfg")
        if os.path.exists(cand):
            return cand
        d2 = os.path.dirname(d)
        if d2 == d:
            break
        d = d2
    return None


def getf(kv: Dict[str, str], k: str, default: float) -> float:
    try:
        return float(kv.get(k, str(default)))
    except Exception:
        return default


def choose_pose_indices(n: int, poses: int) -> List[int]:
    if n <= 0:
        return []
    poses = max(1, int(poses))
    if poses == 1:
        return [n - 1]
    idxs = []
    for i in range(poses):
        a = i / float(poses - 1)
        idx = int(round(a * (n - 1)))
        idxs.append(idx)
    # unique, ordered
    out = []
    seen = set()
    for i in idxs:
        if i not in seen:
            out.append(i)
            seen.add(i)
    return out


def rot2(theta: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    import math

    c = math.cos(theta)
    s = math.sin(theta)
    return ((c, -s), (s, c))


def apply_R(R, v: Tuple[float, float]) -> Tuple[float, float]:
    return (R[0][0] * v[0] + R[0][1] * v[1], R[1][0] * v[0] + R[1][1] * v[1])


def add2(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])


def plot_car_xy(ax, x: float, y: float, psi: float, delta: float, lf: float, lr: float, width: float, wheel_len: float, color, alpha: float):
    """
    Draws a simple car footprint + wheels similar in spirit to matlab/Plotcar.m,
    but in world coordinates. State assumed to be at CG with lf/lr to axles.
    """
    import math
    from matplotlib.patches import Polygon

    R = rot2(psi)
    half_w = 0.5 * width

    # Body corners in body frame: front/rear, left/right
    p_fl = (lf, +half_w)
    p_fr = (lf, -half_w)
    p_rr = (-lr, -half_w)
    p_rl = (-lr, +half_w)

    def to_world(p):
        return add2((x, y), apply_R(R, p))

    body = [to_world(p_fl), to_world(p_fr), to_world(p_rr), to_world(p_rl)]
    ax.add_patch(Polygon(body, closed=True, fill=False, edgecolor=color, linewidth=1.2, alpha=alpha))

    # Wheel centers (approx at axles)
    c_fl = to_world((lf, +half_w))
    c_fr = to_world((lf, -half_w))
    c_rl = to_world((-lr, +half_w))
    c_rr = to_world((-lr, -half_w))

    # Wheel directions
    Rf = rot2(psi + delta)
    Rr = rot2(psi)

    def wheel_segment(c, Rw):
        d = apply_R(Rw, (1.0, 0.0))
        p0 = (c[0] - 0.5 * wheel_len * d[0], c[1] - 0.5 * wheel_len * d[1])
        p1 = (c[0] + 0.5 * wheel_len * d[0], c[1] + 0.5 * wheel_len * d[1])
        return p0, p1

    for c, Rw in ((c_fl, Rf), (c_fr, Rf), (c_rl, Rr), (c_rr, Rr)):
        p0, p1 = wheel_segment(c, Rw)
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=color, linewidth=2.5, alpha=alpha)

    # Heading arrow (short)
    ah = apply_R(R, (0.6 * (lf + lr), 0.0))
    ax.plot([x, x + ah[0]], [y, y + ah[1]], color=color, linewidth=1.0, alpha=alpha)


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot open-loop validation logs from cpp/results/run_*/validation/*.csv")
    ap.add_argument("--dir", required=True, help="validation directory containing per-scenario CSVs")
    ap.add_argument("--out", required=True, help="output png")
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print("ERROR: matplotlib is required (pip install matplotlib)")
        print(f"Reason: {e}")
        return 2

    paths = sorted(glob.glob(os.path.join(args.dir, "*.csv")))
    paths = [p for p in paths if not p.endswith("open_loop_summary.csv")]
    if not paths:
        print(f"No CSVs found in: {args.dir}")
        return 2

    # Stable per-scenario colors reused across all subplots
    tab = plt.get_cmap("tab10")
    scenario_names = [os.path.splitext(os.path.basename(p))[0] for p in paths]
    color_of = {name: tab(i % 10) for i, name in enumerate(scenario_names)}

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    ax_xy = ax[0][0]
    ax_vx = ax[0][1]
    ax_u = ax[1][0]
    ax_mu = ax[1][1]
    ax_r = ax_vx.twinx()

    # Read vehicle geometry for Plotcar-like overlays
    cfg_path = find_config_used(args.dir)
    kv = read_kv_cfg(cfg_path) if cfg_path else {}
    lf = getf(kv, "vehicle_lf", 1.2)
    lr = getf(kv, "vehicle_lr", 1.6)
    car_w = getf(kv, "viz_car_width_m", 1.9)
    wheel_len = getf(kv, "viz_wheel_length_m", 0.6)
    poses_per = int(getf(kv, "viz_car_poses_per_traj", 5))

    for p in paths:
        name = os.path.splitext(os.path.basename(p))[0]
        c = color_of.get(name, "C0")
        d = read_csv(p)
        t = d.get("t", [])
        x = d.get("x", [])
        y = d.get("y", [])
        psi = d.get("psi", [])
        vx = d.get("vx", [])
        vy = d.get("vy", [])
        r = d.get("r", [])
        u_delta = d.get("u_delta", [])
        u_sfx = d.get("u_sfx", [])
        # Four-wheel validation logs use wheel torques instead of sfx
        tau_fl = d.get("tau_fl", [])
        tau_fr = d.get("tau_fr", [])
        tau_rl = d.get("tau_rl", [])
        tau_rr = d.get("tau_rr", [])
        muf = d.get("mu_scale_f", [])
        mur = d.get("mu_scale_r", [])

        if x and y:
            line = ax_xy.plot(x, y, linewidth=2.0, color=c, label=name)[0]
            # Plotcar-like pose overlays (a few per trajectory)
            if psi and u_delta and len(psi) == len(x) and len(u_delta) == len(x):
                idxs = choose_pose_indices(len(x), poses_per)
                for i in idxs:
                    plot_car_xy(
                        ax_xy,
                        x[i],
                        y[i],
                        psi[i],
                        u_delta[i],
                        lf=lf,
                        lr=lr,
                        width=car_w,
                        wheel_len=wheel_len,
                        color=line.get_color(),
                        alpha=0.45,
                    )
        if t and vx:
            ax_vx.plot(t, vx, linewidth=2.0, color=c, linestyle="-", label=f"{name}: vx")
        if t and vy:
            ax_vx.plot(t, vy, linewidth=2.0, color=c, linestyle="--", label=f"{name}: vy")
        if t and u_delta:
            ax_u.plot(t, u_delta, linewidth=2.0, color=c, linestyle="-", label=f"{name}: delta")
            if u_sfx:
                ax_u.plot(t, u_sfx, linewidth=2.0, color=c, linestyle="--", label=f"{name}: sfx")
            elif tau_fl or tau_fr or tau_rl or tau_rr:
                # Plot total torque (kNm) instead of per-wheel torques for readability
                n = len(t)
                def get_i(v, i):
                    return v[i] if v and i < len(v) else 0.0
                tau_tot = [(get_i(tau_fl, i) + get_i(tau_fr, i) + get_i(tau_rl, i) + get_i(tau_rr, i)) / 1000.0 for i in range(n)]
                ax_u.plot(t, tau_tot, linewidth=2.0, color=c, linestyle="--", label=f"{name}: tau_total [kNm]")
        if t and muf and mur:
            ax_mu.plot(t, muf, linewidth=2.0, color=c, linestyle="-", label=f"{name}: mu_f")
            ax_mu.plot(t, mur, linewidth=2.0, color=c, linestyle="--", label=f"{name}: mu_r")
        if t and r:
            ax_r.plot(t, r, linewidth=1.5, color=c, linestyle=":", alpha=0.9, label=f"{name}: r (yaw rate)")

    ax_xy.set_title("XY path")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis("equal")
    ax_xy.legend(loc="best", fontsize=8)

    ax_vx.set_title("Speed (vx, vy) and yaw-rate over time")
    ax_vx.set_xlabel("t [s]")
    ax_vx.set_ylabel("vx, vy [m/s]")
    ax_vx.grid(True, alpha=0.3)
    ax_vx.legend(loc="best", fontsize=8)

    ax_r.set_ylabel("r = dot(psi) [rad/s]")
    ax_r.grid(False)
    # Combine legends from both axes
    h1, l1 = ax_vx.get_legend_handles_labels()
    h2, l2 = ax_r.get_legend_handles_labels()
    ax_vx.legend(h1 + h2, l1 + l2, loc="best", fontsize=8)

    ax_u.set_title("Open-loop commands")
    ax_u.set_xlabel("t [s]")
    ax_u.set_ylabel("delta [rad], sfx [ ] / tau_total [kNm]")
    ax_u.grid(True, alpha=0.3)
    ax_u.legend(loc="best", fontsize=8)

    ax_mu.set_title("Friction scales (online-updatable)")
    ax_mu.set_xlabel("t [s]")
    ax_mu.set_ylabel("mu scale")
    ax_mu.grid(True, alpha=0.3)
    ax_mu.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=200)
    print(f"Wrote: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

