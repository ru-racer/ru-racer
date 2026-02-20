# Scenario Validation Framework

Systematic validation of planners and controllers against defined scenarios with explicit expected outcomes and metrics.

## Quick Start

1. Build the C++ binary:
   ```bash
   cmake -S cpp -B cpp/build && cmake --build cpp/build -j
   ```

2. Install Python deps (optional, for YAML scenarios):
   ```bash
   pip install pyyaml
   ```

3. Run a single scenario:
   ```bash
   python cpp/scenario_validation/run_scenario_suite.py \
     --scenario cpp/scenario_validation/scenarios/straight_dry.yaml \
     --vehicle-cfg cpp/scenarios/fullsize_setpoint_common.cfg
   ```

4. Run full suite:
   ```bash
   python cpp/scenario_validation/run_scenario_suite.py \
     --suite cpp/scenario_validation/scenarios/ \
     --vehicle-cfg cpp/scenarios/fullsize_setpoint_common.cfg
   ```

## Structure

- **`framework/`** — Core validation logic
  - `scenario_def.py` — Scenario schema (map, state, expected outcome)
  - `map_region.py` — Map piece builder (roadgen integration)
  - `metrics.py` — Metrics extraction and pass/fail evaluation
  - `runner.py` — Orchestrates planner + controller execution
  - `report.py` — Summary CSV and console output

- **`scenarios/`** — YAML scenario definitions
  - `straight_dry.yaml` — Straight road, dry
  - `curve_wet.yaml` — Horizontal curve, wet

- **`run_scenario_suite.py`** — Entry point

## Scenario Schema

See `docs/SCENARIO_VALIDATION_DESIGN.md` for full design and schema.
