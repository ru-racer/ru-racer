#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


Point = Tuple[float, float]


@dataclass
class RoadSpec:
    shape: str  # tangent|horizontal_curve|s_curve|hairpin|roundabout
    width_m: float = 8.0
    length_m: float = 120.0  # used by tangent
    radius_m: float = 30.0   # used by curves
    angle_deg: float = 90.0  # used by horizontal_curve / hairpin
    roundabout_radius_m: float = 25.0
    s_curve_radius_m: float = 35.0
    s_curve_angle_deg: float = 45.0


def _polyline_arc(center: Point, r: float, a0: float, a1: float, n: int) -> List[Point]:
    cx, cy = center
    pts: List[Point] = []
    for i in range(n):
        t = i / max(1, n - 1)
        a = a0 + t * (a1 - a0)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def centerline(spec: RoadSpec) -> List[Point]:
    """
    Returns a centerline polyline in world meters.
    Convention: start near (0,0) with heading +x.
    """
    s = spec.shape.lower()
    if s == "tangent":
        n = max(50, int(spec.length_m * 2))
        return [(t * spec.length_m, 0.0) for t in [i / (n - 1) for i in range(n)]]

    if s == "horizontal_curve":
        # Arc to the left by angle_deg, starting heading +x at (0,0).
        ang = math.radians(spec.angle_deg)
        r = spec.radius_m
        # center located at (0, r) for left turn with start point at (0,0) and tangent along +x
        c = (0.0, r)
        # start angle -pi/2, end angle -pi/2 + ang
        return _polyline_arc(c, r, -math.pi / 2, -math.pi / 2 + ang, n=200)

    if s == "hairpin":
        ang = math.radians(max(150.0, spec.angle_deg))
        r = max(8.0, spec.radius_m)
        c = (0.0, r)
        return _polyline_arc(c, r, -math.pi / 2, -math.pi / 2 + ang, n=260)

    if s == "s_curve":
        r = spec.s_curve_radius_m
        ang = math.radians(spec.s_curve_angle_deg)
        # two opposite arcs connected at a midpoint
        c1 = (0.0, r)
        arc1 = _polyline_arc(c1, r, -math.pi / 2, -math.pi / 2 + ang, n=140)
        # end pose of arc1
        x_end, y_end = arc1[-1]
        # second arc center to the right of tangent; approximate by shifting
        # We keep continuity by building arc around local frame.
        # Compute heading at end of arc1: +x rotated by ang
        hdg = ang
        # center of second arc: from end point, offset by -r in normal direction (right turn)
        nx = -math.sin(hdg)
        ny = math.cos(hdg)
        c2 = (x_end - r * nx, y_end - r * ny)
        arc2 = _polyline_arc(c2, r, hdg + math.pi / 2, hdg + math.pi / 2 - ang, n=140)
        return arc1 + arc2[1:]

    if s == "roundabout":
        r = spec.roundabout_radius_m
        # full circle centered at (r, 0) so start at (0,0)
        c = (r, 0.0)
        pts = _polyline_arc(c, r, math.pi, math.pi + 2.0 * math.pi, n=400)
        return pts

    raise ValueError(f"Unsupported road_shape={spec.shape}")


def bounds_for_polyline(cl: List[Point], margin_m: float) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in cl]
    ys = [p[1] for p in cl]
    return (min(xs) - margin_m, max(xs) + margin_m, min(ys) - margin_m, max(ys) + margin_m)


def rasterize_corridor(cl: List[Point], width_m: float, ppm: int) -> Tuple[List[int], int, int, float, float, float, float]:
    """
    Builds a PGM occupancy (free=255) corridor around centerline.
    Returns (data_u8, W, H, xmin,xmax,ymin,ymax) where y is standard world up.
    The map is designed for OccupancyGrid legacy transform:
      px = ceil(x*ppm + origin_x_px) - 1
      py = ceil(origin_y_px - y*ppm) - 1
    """
    import numpy as np

    margin = max(5.0, width_m * 1.5)
    xmin, xmax, ymin, ymax = bounds_for_polyline(cl, margin_m=margin)

    W = int(math.ceil((xmax - xmin) * ppm)) + 3
    H = int(math.ceil((ymax - ymin) * ppm)) + 3

    img = np.zeros((H, W), dtype=np.uint8)  # 0 = obstacle, 255 = free

    # Build a dense set of samples along centerline, then mark pixels within radius.
    half = 0.5 * width_m
    rad_px = max(1, int(math.ceil(half * ppm)))

    def world_to_px(x: float, y: float) -> Tuple[int, int]:
        # map top-left is (xmin,ymax)
        u = int(round((x - xmin) * ppm))
        v = int(round((ymax - y) * ppm))
        return u, v

    # Dense sampling along segments
    for i in range(1, len(cl)):
        x0, y0 = cl[i - 1]
        x1, y1 = cl[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(seg * ppm / 2))
        for k in range(n):
            t = k / (n - 1)
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            u, v = world_to_px(x, y)
            u0 = max(0, u - rad_px)
            u1 = min(W - 1, u + rad_px)
            v0 = max(0, v - rad_px)
            v1 = min(H - 1, v + rad_px)
            # filled disk
            for vv in range(v0, v1 + 1):
                for uu in range(u0, u1 + 1):
                    if (uu - u) * (uu - u) + (vv - v) * (vv - v) <= rad_px * rad_px:
                        img[vv, uu] = 255

    return img.reshape(-1).tolist(), W, H, xmin, xmax, ymin, ymax


def write_pgm(path: str, W: int, H: int, data_u8: List[int]) -> None:
    with open(path, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode("ascii"))
        f.write(bytes(data_u8))


def write_bg_png(path: str, W: int, H: int, data_u8: List[int]) -> None:
    import numpy as np
    import matplotlib.pyplot as plt

    img = np.array(data_u8, dtype=np.uint8).reshape((H, W))
    plt.figure(figsize=(8, 5))
    plt.imshow(img, cmap="gray", origin="upper")
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()

