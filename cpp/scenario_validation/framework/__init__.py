"""
Scenario validation framework for ru-racer.

Provides:
- ScenarioDef: declarative scenario schema (map, initial/goal state, expected outcome)
- MapRegion: map piece abstraction (generated road or file-based)
- Metrics: extraction from planner/controller outputs and pass/fail evaluation
- ScenarioRunner: orchestrates planner + controller execution
"""

from .scenario_def import (
    ScenarioDef,
    MapRegion,
    VehicleState,
    ExpectedOutcome,
)
from .metrics import Metrics, extract_metrics, evaluate_pass_fail
from .runner import ScenarioRunner

__all__ = [
    "ScenarioDef",
    "MapRegion",
    "VehicleState",
    "ExpectedOutcome",
    "Metrics",
    "extract_metrics",
    "evaluate_pass_fail",
    "ScenarioRunner",
]
