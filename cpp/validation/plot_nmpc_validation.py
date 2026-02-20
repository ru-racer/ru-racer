#!/usr/bin/env python3

import argparse
import glob
import os
from typing import Dict, List, Tuple


def read_csv(path: str) -> Dict[str, List[float]]:
    import csv

    cols: Dict[str, List[float]] = {}
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            for k, v in row.items():
                if k not in cols:
                    cols[k] = []
                try:
                    cols[k].append(float(v))
                except Exception:
                    pass
    return cols


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot NMPC validation outputs from validation/nmpc/*.csv")
    ap.add_argument("--dir", required=True, help="Directory containing *_ref.csv and *_exec.csv")
    ap.add_argument("--out", required=True, help="Output PNG path")
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print("ERROR: matplotlib is required")
        print(e)
        return 2

    ref_files = sorted(glob.glob(os.path.join(args.dir, "*_ref.csv")))
    if not ref_files:
        print(f"No *_ref.csv found in: {args.dir}")
        return 2

    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    ax_xy = ax[0][0]
    ax_err = ax[0][1]
    ax_u = ax[1][0]
    ax_v = ax[1][1]

    for rf in ref_files:
        name = os.path.basename(rf).replace("_ref.csv", "")
        ef = os.path.join(args.dir, name + "_exec.csv")
        if not os.path.exists(ef):
            continue

        ref = read_csv(rf)
        exe = read_csv(ef)
        xr, yr = ref.get("x", []), ref.get("y", [])
        xe, ye = exe.get("x", []), exe.get("y", [])
        t = exe.get("t", [])
        xref_e, yref_e = exe.get("x_ref", []), exe.get("y_ref", [])
        vx, vxr = exe.get("vx", []), exe.get("vx_ref", [])
        ud, us = exe.get("u_delta", []), exe.get("u_sfx", [])

        ax_xy.plot(xr, yr, "--", linewidth=2.0, label=f"{name} ref")
        ax_xy.plot(xe, ye, "-", linewidth=2.0, label=f"{name} exec")

        if t and xref_e and yref_e and xe and ye:
            import math

            err = [math.hypot(xe[i] - xref_e[i], ye[i] - yref_e[i]) for i in range(min(len(xe), len(xref_e)))]
            ax_err.plot(t[: len(err)], err, linewidth=2.0, label=name)

        if t and ud and us:
            ax_u.plot(t[: len(ud)], ud, linewidth=2.0, label=f"{name}: delta")
            ax_u.plot(t[: len(us)], us, linewidth=2.0, linestyle="--", label=f"{name}: sfx")

        if t and vx and vxr:
            ax_v.plot(t[: len(vx)], vx, linewidth=2.0, label=f"{name}: vx")
            ax_v.plot(t[: len(vxr)], vxr, linewidth=2.0, linestyle="--", label=f"{name}: vx_ref")

    ax_xy.set_title("XY: reference vs executed")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis("equal")
    ax_xy.legend(loc="best", fontsize=8)

    ax_err.set_title("Position error over time")
    ax_err.set_xlabel("t [s]")
    ax_err.set_ylabel("||pos - pos_ref|| [m]")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="best", fontsize=8)

    ax_u.set_title("NMPC controls")
    ax_u.set_xlabel("t [s]")
    ax_u.set_ylabel("delta [rad], sfx [ ]")
    ax_u.grid(True, alpha=0.3)
    ax_u.legend(loc="best", fontsize=8)

    ax_v.set_title("Speed tracking")
    ax_v.set_xlabel("t [s]")
    ax_v.set_ylabel("vx [m/s]")
    ax_v.grid(True, alpha=0.3)
    ax_v.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=200)
    print(f"Wrote: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

