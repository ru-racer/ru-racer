#!/usr/bin/env python3

import argparse
import csv
import os
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class Node:
    node_id: int
    parent: int
    cum_cost: float
    x: float
    y: float


def read_nodes_csv(path: str) -> List[Node]:
    nodes: Dict[int, Node] = {}
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            # Writer currently stores numeric fields via to_string => some end up like "1.000000"
            node_id = int(float(row["node_id"]))
            parent = int(float(row["parent"]))
            cum_cost = float(row["cum_cost"])
            x = float(row["x"])
            y = float(row["y"])
            nodes[node_id] = Node(node_id=node_id, parent=parent, cum_cost=cum_cost, x=x, y=y)
    # return ordered by id
    return [nodes[i] for i in sorted(nodes.keys())]


def read_path_ids(path_csv: str) -> Optional[List[int]]:
    if not os.path.exists(path_csv):
        return None
    ids: List[int] = []
    with open(path_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            ids.append(int(float(row["node_id"])))
    return ids


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


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot explored RRT* tree from rrt_nodes.csv")
    ap.add_argument("--nodes", required=True, help="Path to rrt_nodes.csv")
    ap.add_argument("--path", default=None, help="Optional path to rrt_path.csv to overlay best path")
    ap.add_argument("--traj", default=None, help="Optional trajectory CSV to overlay (e.g., rrt_ref_traj.csv or nmpc_exec.csv).")
    ap.add_argument(
        "--traj-cols",
        default="x,y",
        help="Which columns to use for trajectory XY (default: x,y). For nmpc_exec.csv use x,y; for refs use x,y.",
    )
    ap.add_argument("--traj-label", default="trajectory", help="Legend label for --traj overlay.")
    ap.add_argument("--out", default=None, help="Output image path (png/pdf). If omitted, show interactive plot.")
    ap.add_argument("--stride", type=int, default=1, help="Plot every N-th edge (for speed on big trees).")
    ap.add_argument("--max-edges", type=int, default=0, help="Limit number of edges drawn (0 = no limit).")
    ap.add_argument("--color-by", choices=["none", "cost"], default="cost", help="Edge color scheme.")
    ap.add_argument("--bg", default=None, help="Optional background image (e.g., track.jpg) to plot on top of.")
    ap.add_argument(
        "--bg-transform",
        choices=["none", "track"],
        default="none",
        help="How to transform world (x,y) to background pixel coords. 'track' matches MATLAB Implot2d mapping.",
    )
    ap.add_argument("--config", default=None, help="Optional config_used.cfg (to plot start/goal). If omitted, auto-detect next to --nodes.")
    ap.add_argument("--show-start-goal", action="store_true", help="Plot start/goal markers from config_used.cfg if available.")
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        import numpy as np
    except Exception as e:
        print("ERROR: matplotlib is required. Install with: pip install matplotlib numpy")
        print(f"Reason: {e}")
        return 2

    nodes = read_nodes_csv(args.nodes)
    by_id = {n.node_id: n for n in nodes}

    def to_plot_xy(x_m: float, y_m: float):
        if args.bg and args.bg_transform == "track":
            # Matches `matlab/RRTBike.m` Implot2d:
            #   plot(687+360*y, 1125+390*x)
            px = 687.0 + 360.0 * y_m
            py = 1125.0 + 390.0 * x_m
            return px, py
        return x_m, y_m

    segs = []
    colors = []
    edges = 0
    for n in nodes[1:]:
        if n.node_id % max(1, args.stride) != 0:
            continue
        if n.parent < 0 or n.parent not in by_id:
            continue
        p = by_id[n.parent]
        p0 = to_plot_xy(p.x, p.y)
        p1 = to_plot_xy(n.x, n.y)
        segs.append([p0, p1])
        if args.color_by == "cost":
            colors.append(n.cum_cost)
        edges += 1
        if args.max_edges and edges >= args.max_edges:
            break

    fig, ax = plt.subplots(figsize=(8, 6))
    if args.bg:
        img = plt.imread(args.bg)
        ax.imshow(img, origin="upper")
    if segs:
        if args.color_by == "cost":
            lc = LineCollection(segs, cmap="viridis", linewidths=0.4, alpha=0.7)
            lc.set_array(np.array(colors, dtype=float))
            ax.add_collection(lc)
            cbar = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
            cbar.set_label("cumulative cost")
        else:
            lc = LineCollection(segs, colors=(0.0, 0.6, 0.0, 0.5), linewidths=0.4)
            ax.add_collection(lc)

    xs = []
    ys = []
    for n in nodes:
        xx, yy = to_plot_xy(n.x, n.y)
        xs.append(xx)
        ys.append(yy)
    ax.plot(xs, ys, ".", markersize=1.0, color=(0.0, 0.0, 0.0, 0.25), label="nodes")

    # Overlay best path if provided
    path_csv = args.path
    if path_csv is None:
        guess = os.path.join(os.path.dirname(args.nodes), "rrt_path.csv")
        if os.path.exists(guess):
            path_csv = guess
    path_ids = read_path_ids(path_csv) if path_csv else None
    if path_ids:
        px = []
        py = []
        for i in path_ids:
            if i not in by_id:
                continue
            xx, yy = to_plot_xy(by_id[i].x, by_id[i].y)
            px.append(xx)
            py.append(yy)
        ax.plot(px, py, "-", linewidth=2.0, color="dodgerblue", label="best path")

    # Overlay an arbitrary trajectory CSV (e.g., rrt_ref_traj.csv or nmpc_exec.csv)
    traj_csv = args.traj
    if traj_csv is None:
        # prefer executed NMPC if present, else reference
        guess_exec = os.path.join(os.path.dirname(args.nodes), "nmpc_exec.csv")
        guess_ref = os.path.join(os.path.dirname(args.nodes), "rrt_ref_traj.csv")
        if os.path.exists(guess_exec):
            traj_csv = guess_exec
            args.traj_label = "nmpc exec"
        elif os.path.exists(guess_ref):
            traj_csv = guess_ref
            args.traj_label = "rrt ref"

    if traj_csv and os.path.exists(traj_csv):
        cx, cy = [c.strip() for c in args.traj_cols.split(",")]
        tx, ty = [], []
        with open(traj_csv, "r", newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                if cx not in row or cy not in row:
                    continue
                xx = float(row[cx])
                yy = float(row[cy])
                px, py = to_plot_xy(xx, yy)
                tx.append(px)
                ty.append(py)
        if tx:
            ax.plot(tx, ty, "-", linewidth=2.0, color="red", label=args.traj_label)

    # Start/goal markers (from config)
    if args.show_start_goal:
        cfg_path = args.config
        if cfg_path is None:
            guess_cfg = os.path.join(os.path.dirname(args.nodes), "config_used.cfg")
            if os.path.exists(guess_cfg):
                cfg_path = guess_cfg
        kv = read_kv_cfg(cfg_path) if cfg_path else {}
        if "start_x" in kv and "start_y" in kv:
            sx, sy = float(kv["start_x"]), float(kv["start_y"])
            px, py = to_plot_xy(sx, sy)
            ax.plot([px], [py], marker="o", markersize=10, color="lime", markeredgecolor="black", label="start")
        else:
            # Fallback: node 0
            n0 = by_id.get(0)
            if n0 is not None:
                px, py = to_plot_xy(n0.x, n0.y)
                ax.plot([px], [py], marker="o", markersize=10, color="lime", markeredgecolor="black", label="start")

        if "goal_x" in kv and "goal_y" in kv:
            gx, gy = float(kv["goal_x"]), float(kv["goal_y"])
            px, py = to_plot_xy(gx, gy)
            ax.plot([px], [py], marker="*", markersize=14, color="yellow", markeredgecolor="black", label="goal")

    if args.bg and args.bg_transform == "track":
        ax.set_title("Explored tree on track image")
        ax.set_xlabel("image x [px]")
        ax.set_ylabel("image y [px]")
        ax.grid(False)
    else:
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("RRT* explored tree")
        ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.legend(loc="best")

    if not args.bg:
        ax.autoscale()
    if args.out:
        fig.tight_layout()
        fig.savefig(args.out, dpi=200)
        print(f"Wrote: {args.out}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

