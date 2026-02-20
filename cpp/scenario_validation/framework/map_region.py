"""
Map region builder: generates or loads map pieces for scenarios.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .scenario_def import MapRegion

# Local roadgen
THIS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)
from roadgen import RoadSpec, centerline, rasterize_corridor, write_bg_png, write_pgm  # type: ignore


@dataclass
class MapOutput:
    """Output of map region build."""

    map_pgm_path: str
    map_png_path: Optional[str] = None
    ppm: int = 10
    origin_x_px: float = 0.0
    origin_y_px: float = 0.0
    xmin: float = 0.0
    xmax: float = 0.0
    ymin: float = 0.0
    ymax: float = 0.0
    centerline_pts: Optional[List[Tuple[float, float]]] = None


def build_map_region(
    region: MapRegion,
    output_dir: str,
    scenario_name: str = "scenario",
) -> MapOutput:
    """
    Build map from MapRegion definition.

    For type=generated: uses roadgen to create PGM.
    For type=file: copies/links existing map (future).
    """
    if region.type == "file" and region.map_file:
        return _build_from_file(region, output_dir, scenario_name)
    return _build_generated(region, output_dir, scenario_name)


def _build_generated(
    region: MapRegion,
    output_dir: str,
    scenario_name: str,
) -> MapOutput:
    spec = RoadSpec(
        shape=region.road_shape,
        width_m=region.road_width_m,
        length_m=region.road_length_m,
        radius_m=region.road_radius_m,
        angle_deg=region.road_angle_deg,
        roundabout_radius_m=region.roundabout_radius_m,
        s_curve_radius_m=region.s_curve_radius_m,
        s_curve_angle_deg=region.s_curve_angle_deg,
    )
    ppm = max(5, region.ppm)
    cl = centerline(spec)
    data, W, H, xmin, xmax, ymin, ymax = rasterize_corridor(cl, spec.width_m, ppm)

    os.makedirs(output_dir, exist_ok=True)
    map_pgm = os.path.join(output_dir, "road_map.pgm")
    map_png = os.path.join(output_dir, "road_map.png")
    write_pgm(map_pgm, W, H, data)
    write_bg_png(map_png, W, H, data)

    # OccupancyGrid legacy transform
    origin_x_px = 1.0 - xmin * ppm
    origin_y_px = 1.0 + ymax * ppm

    return MapOutput(
        map_pgm_path=map_pgm,
        map_png_path=map_png,
        ppm=ppm,
        origin_x_px=origin_x_px,
        origin_y_px=origin_y_px,
        xmin=xmin,
        xmax=xmax,
        ymin=ymin,
        ymax=ymax,
        centerline_pts=cl,
    )


def _build_from_file(
    region: MapRegion,
    output_dir: str,
    scenario_name: str,
) -> MapOutput:
    """Use existing map file. For now, minimal implementation."""
    if not region.map_file or not os.path.exists(region.map_file):
        raise FileNotFoundError(f"Map file not found: {region.map_file}")
    # Could copy/link; for now just return path
    return MapOutput(
        map_pgm_path=os.path.abspath(region.map_file),
        map_png_path=None,
        ppm=int(region.map_pixels_per_meter or 10),
        origin_x_px=float(region.map_origin_x_px or 0),
        origin_y_px=float(region.map_origin_y_px or 0),
        xmin=-100.0,
        xmax=100.0,
        ymin=-100.0,
        ymax=100.0,
    )
