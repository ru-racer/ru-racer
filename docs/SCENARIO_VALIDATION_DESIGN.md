# Scenario Validation Framework — Design & Recommendations

This document describes the recommended code structure for scenario validation in ru-racer, enabling systematic validation of planners and controllers against defined scenarios with explicit expected outcomes and metrics.

---

## 1. Overview

The framework supports:

1. **Scenario definition** — Declarative description of what to test
2. **Map piece** — The portion of the map where the scenario occurs (geometry, obstacles, road conditions)
3. **Expected outcome** — Pass/fail criteria and target metrics
4. **Planner** — Finds feasible paths (Spars-RRT, RRT*, etc.)
5. **Controller** — Follows paths based on vehicle dynamics, road geometry, and objects

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     Scenario Validation Framework                         │
├─────────────────────────────────────────────────────────────────────────┤
│  Scenario Definition (YAML/JSON)                                         │
│  ├── id, name, description                                              │
│  ├── map_region (geometry, obstacles, road conditions)                   │
│  ├── initial_state, goal_state                                          │
│  └── expected_outcome (metrics, thresholds, pass_criteria)              │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Map Region Builder                                                      │
│  ├── roadgen.py (centerline, corridor)                                   │
│  ├── OccupancyGrid (PGM/BMP)                                             │
│  └── obstacles (circles, polygons)                                       │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Execution Pipeline                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────────┐  │
│  │   Planner   │───▶│  Reference  │───▶│  Controller (NMPC)          │  │
│  │ Spars-RRT   │    │  Trajectory │    │  Vehicle dynamics + geometry │  │
│  │ RRT*        │    │  rrt_ref_   │    │  Road + objects constraints  │  │
│  └─────────────┘    │  traj.csv   │    └─────────────────────────────┘  │
│                     └─────────────┘                    │                 │
│                                                         ▼                 │
│                                              nmpc_exec.csv, rrt_*.csv    │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Metrics Evaluator                                                       │
│  ├── success (path found, goal reached)                                  │
│  ├── planner: nodes, best_cost, time                                     │
│  ├── controller: pos_err, heading_err, jerk, constraint violations        │
│  └── pass/fail vs expected_outcome thresholds                           │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Recommended Directory Structure

```
cpp/
├── scenario_validation/
│   ├── framework/                    # NEW: Core validation framework
│   │   ├── __init__.py
│   │   ├── scenario_def.py          # Scenario schema (dataclasses)
│   │   ├── map_region.py            # Map piece abstraction
│   │   ├── metrics.py               # Metrics extraction & pass/fail
│   │   ├── runner.py                # Orchestrates planner + controller run
│   │   └── report.py                # Summary reports, plots
│   │
│   ├── scenarios/                    # Scenario definitions (YAML)
│   │   ├── straight_dry.yaml
│   │   ├── curve_wet.yaml
│   │   ├── obstacle_avoidance.yaml
│   │   └── edgecase_blocker.yaml
│   │
│   ├── run_openscenario_validation.py   # Existing: OpenSCENARIO runner
│   ├── run_straight_edgecases.py        # Existing: edge-case sweep
│   ├── run_scenario_suite.py             # NEW: Run full scenario suite
│   └── roadgen.py                       # Existing: road geometry
│
├── scenarios/                        # Existing: .cfg configs
├── validation/                       # Existing: open-loop, NMPC validation
└── ...
```

---

## 4. Scenario Definition Schema

Each scenario is a declarative file (YAML) with:

```yaml
# scenario_def example
id: straight_dry_001
name: "Straight road, dry conditions"
description: "Basic setpoint tracking on straight road"

map_region:
  type: generated              # generated | file
  road_shape: tangent
  road_length_m: 120.0
  road_width_m: 8.0
  road_conditions: [dry]
  obstacles: []                 # or path to CSV, or inline circles

initial_state:
  x: 0.0
  y: 0.0
  psi: 0.0
  vx: 12.0
  vy: 0.0
  r: 0.0

goal_state:
  x: 110.0
  y: 0.0
  psi: 0.0
  vx: 15.0

expected_outcome:
  planner_success: true
  controller_reaches_goal: true
  metrics:
    planner_nodes_max: 50000
    planner_best_cost_max: 200.0
    nmpc_rms_pos_err_max: 0.5
    nmpc_final_pos_err_max: 0.3
```

---

## 5. Metrics to Validate

| Category | Metric | Source | Use |
|----------|--------|--------|-----|
| **Planner** | `success` | rrt_path.csv exists | Path found |
| | `nodes` | rrt_nodes.csv | Tree size |
| | `best_cost` | rrt_nodes.csv | Path quality |
| | `plan_time_s` | (future) | Computation time |
| **Controller** | `nmpc_final_pos_err` | nmpc_exec.csv | Goal accuracy |
| | `nmpc_rms_pos_err` | nmpc_exec.csv | Tracking quality |
| | `max_steer_rate` | nmpc_exec.csv | Actuator limits |
| | `constraint_violations` | (future) | Safety |
| **Vehicle** | `max_lateral_accel` | Derived | Dynamics limits |
| | `collision` | OccupancyGrid check | Obstacle avoidance |

---

## 6. Integration with Existing Code

- **Planner**: `ru_racer` binary with `mode=plan` (default) — already produces `rrt_path.csv`, `rrt_nodes.csv`, `rrt_ref_traj.csv`
- **Controller**: NMPC runs automatically when `run_nmpc=true` — produces `nmpc_exec.csv`
- **Map**: `roadgen.py` + `OccupancyGrid` — PGM from centerline, obstacles from CSV
- **Config**: Key-value `.cfg` — framework merges scenario + vehicle config before run

---

## 7. Extensibility

- **New planners**: Add algorithm in C++ and expose via `algorithm=` config; framework runs same scenario with different configs
- **New controllers**: Add mode or binary; framework compares metrics across controller configs
- **New metrics**: Extend `metrics.py` to parse new CSV columns or derived values
- **New map types**: Extend `map_region.py` for HD maps, OpenDRIVE, etc.

---

## 8. Usage

```bash
# Run single scenario
python cpp/scenario_validation/run_scenario_suite.py \
  --scenario cpp/scenario_validation/scenarios/straight_dry.yaml \
  --vehicle-cfg cpp/data/fullsize_car.cfg

# Run full suite
python cpp/scenario_validation/run_scenario_suite.py \
  --suite cpp/scenario_validation/scenarios/ \
  --vehicle-cfg cpp/data/fullsize_car.cfg \
  --output cpp/results/scenario_suite/
```

---

## 9. Next Steps

1. Implement `framework/` modules (scenario_def, map_region, metrics, runner)
2. Add 2–3 example YAML scenarios
3. Add `run_scenario_suite.py` entry point
4. Optionally: CI integration to run suite on each commit
