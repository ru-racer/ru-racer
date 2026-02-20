"""
Scenario definition schema for validation.

Supports declarative YAML/JSON scenario files with:
- Map region (geometry, obstacles, road conditions)
- Initial and goal vehicle state
- Expected outcome (metrics thresholds, pass criteria)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class VehicleState:
    """Vehicle state (x, y, psi, vx, vy, r)."""

    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    r: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "psi": self.psi,
            "vx": self.vx,
            "vy": self.vy,
            "r": self.r,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> VehicleState:
        return cls(
            x=float(d.get("x", 0)),
            y=float(d.get("y", 0)),
            psi=float(d.get("psi", 0)),
            vx=float(d.get("vx", 0)),
            vy=float(d.get("vy", 0)),
            r=float(d.get("r", 0)),
        )


@dataclass
class MapRegion:
    """
    Map piece where the scenario occurs.

    type: "generated" (roadgen) or "file" (PGM path)
    For generated: road_shape, dimensions, obstacles, road_conditions
    For file: path to PGM, transform params
    """

    type: str = "generated"
    # Generated road params (roadgen.py)
    road_shape: str = "tangent"
    road_length_m: float = 120.0
    road_width_m: float = 8.0
    road_radius_m: float = 30.0
    road_angle_deg: float = 90.0
    roundabout_radius_m: float = 25.0
    s_curve_radius_m: float = 35.0
    s_curve_angle_deg: float = 45.0
    ppm: int = 10
    # Obstacles: path to CSV or list of [x, y, r]
    obstacles: List[Any] = field(default_factory=list)
    obstacles_csv: Optional[str] = None
    # Road conditions (friction scaling)
    road_conditions: List[str] = field(default_factory=lambda: ["dry"])
    # File-based map
    map_file: Optional[str] = None
    map_pixels_per_meter: Optional[float] = None
    map_origin_x_px: Optional[float] = None
    map_origin_y_px: Optional[float] = None
    map_use_affine: bool = False

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> MapRegion:
        obs = d.get("obstacles", [])
        if isinstance(obs, str):
            obs = []
        return cls(
            type=str(d.get("type", "generated")),
            road_shape=str(d.get("road_shape", "tangent")),
            road_length_m=float(d.get("road_length_m", 120.0)),
            road_width_m=float(d.get("road_width_m", 8.0)),
            road_radius_m=float(d.get("road_radius_m", 30.0)),
            road_angle_deg=float(d.get("road_angle_deg", 90.0)),
            roundabout_radius_m=float(d.get("roundabout_radius_m", 25.0)),
            s_curve_radius_m=float(d.get("s_curve_radius_m", 35.0)),
            s_curve_angle_deg=float(d.get("s_curve_angle_deg", 45.0)),
            ppm=int(d.get("ppm", 10)),
            obstacles=list(obs) if isinstance(obs, list) else [],
            obstacles_csv=d.get("obstacles_csv"),
            road_conditions=d.get("road_conditions", ["dry"]),
            map_file=d.get("map_file"),
            map_pixels_per_meter=d.get("map_pixels_per_meter"),
            map_origin_x_px=d.get("map_origin_x_px"),
            map_origin_y_px=d.get("map_origin_y_px"),
            map_use_affine=bool(d.get("map_use_affine", False)),
        )


@dataclass
class ExpectedOutcome:
    """Pass/fail criteria and metric thresholds."""

    planner_success: bool = True
    controller_reaches_goal: bool = True
    metrics: Dict[str, float] = field(default_factory=dict)
    # Common keys: planner_nodes_max, planner_best_cost_max,
    # nmpc_rms_pos_err_max, nmpc_final_pos_err_max

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> ExpectedOutcome:
        return cls(
            planner_success=bool(d.get("planner_success", True)),
            controller_reaches_goal=bool(d.get("controller_reaches_goal", True)),
            metrics=dict(d.get("metrics", {})),
        )


@dataclass
class ScenarioDef:
    """Full scenario definition."""

    id: str
    name: str
    description: str = ""
    map_region: MapRegion = field(default_factory=MapRegion)
    initial_state: VehicleState = field(default_factory=VehicleState)
    goal_state: VehicleState = field(default_factory=VehicleState)
    expected_outcome: ExpectedOutcome = field(default_factory=ExpectedOutcome)
    # Optional: override vehicle/planner config for this scenario
    config_overrides: Dict[str, str] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> ScenarioDef:
        return cls(
            id=str(d.get("id", "unnamed")),
            name=str(d.get("name", d.get("id", "unnamed"))),
            description=str(d.get("description", "")),
            map_region=MapRegion.from_dict(d.get("map_region", {})),
            initial_state=VehicleState.from_dict(d.get("initial_state", {})),
            goal_state=VehicleState.from_dict(d.get("goal_state", {})),
            expected_outcome=ExpectedOutcome.from_dict(d.get("expected_outcome", {})),
            config_overrides=dict(d.get("config_overrides", {})),
        )

    @classmethod
    def from_yaml(cls, path: str) -> ScenarioDef:
        """Load scenario from YAML file."""
        try:
            import yaml
        except ImportError:
            raise ImportError("PyYAML required for YAML: pip install pyyaml")
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    @classmethod
    def from_json(cls, path: str) -> ScenarioDef:
        """Load scenario from JSON file."""
        import json
        with open(path, "r") as f:
            data = json.load(f)
        return cls.from_dict(data)
