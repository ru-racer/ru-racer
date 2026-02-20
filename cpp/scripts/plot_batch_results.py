#!/usr/bin/env python3

import argparse
import csv
import glob
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class BatchRow:
    i: int
    seed: int
    gx: float
    gy: float
    success: int
    best_cost: float
    nodes: int


def read_batch_summary(path: str) -> List[BatchRow]:
    rows: List[BatchRow] = []
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(
                BatchRow(
                    i=int(float(row["i"])),
                    seed=int(float(row["seed"])),
                    gx=float(row["goal_x"]),
                    gy=float(row["goal_y"]),
                    success=int(float(row["success"])),
                    best_cost=float(row["best_cost"]),
                    nodes=int(float(row["nodes"])),
                )
            )
    return rows


def read_xy_from_csv(path: str, xcol: str = "x", ycol: str = "y") -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            if xcol not in row or ycol not in row:
                continue
            xs.append(float(row[xcol]))
            ys.append(float(row[ycol]))
    return xs, ys


def iter_edges(nodes_csv: str, stride: int, max_edges: int) -> Tuple[List[Tuple[float, float, float, float]], int]:
    """
    Returns list of (x0,y0,x1,y1) edges, and total nodes seen.
    Reads entire nodes file into arrays, then emits edges with parent>=0.
    """
    xs: List[float] = []
    ys: List[float] = []
    parents: List[int] = []
    with open(nodes_csv, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            parents.append(int(float(row["parent"])))

    edges: List[Tuple[float, float, float, float]] = []
    n = len(xs)
    s = max(1, int(stride))
    for idx in range(1, n, s):
        p = parents[idx]
        if p < 0 or p >= n:
            continue
        edges.append((xs[p], ys[p], xs[idx], ys[idx]))
        if max_edges and len(edges) >= max_edges:
            break
    return edges, n


def main() -> int:
    ap = argparse.ArgumentParser(description="Visualize batch_setpoints runs (success analytics, NMPC overlays, combined RRT tree).")
    ap.add_argument("--run", required=True, help="Path to run folder (contains batch_summary.csv and goal_### subfolders).")
    ap.add_argument("--stride", type=int, default=20, help="Edge draw stride per goal for combined tree.")
    ap.add_argument("--max-edges-per-goal", type=int, default=4000, help="Cap edges per goal in combined tree (0=no cap).")
    ap.add_argument("--out-analytics", default=None, help="Output PNG for analytics (default: <run>/batch_analytics.png).")
    ap.add_argument("--out-trajs", default=None, help="Output PNG for NMPC trajectory overlay (default: <run>/batch_nmpc_trajs.png).")
    ap.add_argument("--out-tree", default=None, help="Output PNG for combined tree (default: <run>/batch_rrt_tree_combined.png).")
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        import numpy as np
    except Exception as e:
        print("ERROR: matplotlib is required (pip install matplotlib numpy)")
        print(f"Reason: {e}")
        return 2

    run_dir = os.path.abspath(args.run)
    batch_csv = os.path.join(run_dir, "batch_summary.csv")
    if not os.path.exists(batch_csv):
        print(f"ERROR: missing batch_summary.csv at: {batch_csv}")
        return 2

    out_analytics = args.out_analytics or os.path.join(run_dir, "batch_analytics.png")
    out_trajs = args.out_trajs or os.path.join(run_dir, "batch_nmpc_trajs.png")
    out_tree = args.out_tree or os.path.join(run_dir, "batch_rrt_tree_combined.png")

    rows = read_batch_summary(batch_csv)
    total = len(rows)
    succ = sum(r.success for r in rows)
    rate = (succ / total) if total else 0.0

    # ---------- Analytics figure ----------
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    ax_map = ax[0][0]
    ax_hist_nodes = ax[0][1]
    ax_hist_cost = ax[1][0]
    ax_scatter = ax[1][1]

    gx_s = [r.gx for r in rows if r.success == 1]
    gy_s = [r.gy for r in rows if r.success == 1]
    gx_f = [r.gx for r in rows if r.success == 0]
    gy_f = [r.gy for r in rows if r.success == 0]

    ax_map.plot(gx_f, gy_f, "x", color="crimson", label="fail")
    ax_map.plot(gx_s, gy_s, "o", color="seagreen", label="success", markersize=6)
    ax_map.plot([0.0], [0.0], marker="*", color="gold", markeredgecolor="black", markersize=12, label="start")
    ax_map.set_title(f"Goal outcomes (success {succ}/{total} = {rate:.1%})")
    ax_map.set_xlabel("goal x [m]")
    ax_map.set_ylabel("goal y [m]")
    ax_map.grid(True, alpha=0.3)
    ax_map.axis("equal")
    ax_map.legend(loc="best")

    nodes_all = [r.nodes for r in rows]
    nodes_s = [r.nodes for r in rows if r.success == 1]
    ax_hist_nodes.hist(nodes_all, bins=20, color="lightgray", edgecolor="black", label="all")
    if nodes_s:
        ax_hist_nodes.hist(nodes_s, bins=20, color="seagreen", alpha=0.7, label="success")
    ax_hist_nodes.set_title("Nodes explored")
    ax_hist_nodes.set_xlabel("nodes")
    ax_hist_nodes.set_ylabel("count")
    ax_hist_nodes.grid(True, alpha=0.25)
    ax_hist_nodes.legend(loc="best")

    cost_s = [r.best_cost for r in rows if r.success == 1 and np.isfinite(r.best_cost)]
    if cost_s:
        ax_hist_cost.hist(cost_s, bins=15, color="dodgerblue", edgecolor="black")
    ax_hist_cost.set_title("Best cost (successes)")
    ax_hist_cost.set_xlabel("best_cost")
    ax_hist_cost.set_ylabel("count")
    ax_hist_cost.grid(True, alpha=0.25)

    # scatter: best_cost vs nodes for successes
    if cost_s and nodes_s:
        ax_scatter.scatter(nodes_s, cost_s, c="dodgerblue", alpha=0.8)
    ax_scatter.set_title("Success tradeoff: nodes vs best_cost")
    ax_scatter.set_xlabel("nodes")
    ax_scatter.set_ylabel("best_cost")
    ax_scatter.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_analytics, dpi=200)
    print(f"Wrote: {out_analytics}")

    # ---------- NMPC trajectory overlay ----------
    fig2, ax2 = plt.subplots(figsize=(10, 8))
    cmap = plt.get_cmap("tab20")
    nmpc_paths = sorted(glob.glob(os.path.join(run_dir, "goal_*", "nmpc_exec.csv")))
    for idx, p in enumerate(nmpc_paths):
        goal_id = os.path.basename(os.path.dirname(p))
        xs, ys = read_xy_from_csv(p, "x", "y")
        if not xs:
            continue
        c = cmap(idx % 20)
        ax2.plot(xs, ys, "-", linewidth=2.0, alpha=0.85, color=c, label=goal_id)
        ax2.plot([xs[0]], [ys[0]], marker="o", color=c, markersize=4)
        ax2.plot([xs[-1]], [ys[-1]], marker="x", color=c, markersize=6)

    ax2.plot([0.0], [0.0], marker="*", color="gold", markeredgecolor="black", markersize=12, label="start")
    ax2.set_title(f"NMPC executed trajectories (successes only): {len(nmpc_paths)}/{total}")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    ax2.grid(True, alpha=0.3)
    ax2.axis("equal")
    ax2.legend(loc="best", fontsize=8)
    fig2.tight_layout()
    fig2.savefig(out_trajs, dpi=200)
    print(f"Wrote: {out_trajs}")

    # ---------- Combined RRT tree ----------
    nodes_files = sorted(glob.glob(os.path.join(run_dir, "goal_*", "rrt_nodes.csv")))
    segs: List[List[Tuple[float, float]]] = []
    for nf in nodes_files:
        edges, n = iter_edges(nf, args.stride, args.max_edges_per_goal)
        for (x0, y0, x1, y1) in edges:
            segs.append([(x0, y0), (x1, y1)])

    fig3, ax3 = plt.subplots(figsize=(10, 8))
    if segs:
        lc = LineCollection(segs, colors=(0.0, 0.0, 0.0, 0.08), linewidths=0.5)
        ax3.add_collection(lc)
    # overlay goal markers
    ax3.plot(gx_f, gy_f, "x", color="crimson", label="fail goals")
    ax3.plot(gx_s, gy_s, "o", color="seagreen", label="success goals", markersize=5)
    ax3.plot([0.0], [0.0], marker="*", color="gold", markeredgecolor="black", markersize=12, label="start")
    ax3.set_title(f"Combined RRT trees (downsampled): stride={args.stride}, max_edges/goal={args.max_edges_per_goal}")
    ax3.set_xlabel("x [m]")
    ax3.set_ylabel("y [m]")
    ax3.grid(True, alpha=0.2)
    ax3.axis("equal")
    ax3.legend(loc="best", fontsize=8)
    ax3.autoscale()
    fig3.tight_layout()
    fig3.savefig(out_tree, dpi=200)
    print(f"Wrote: {out_tree}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

