## Scenario-based validation (OpenSCENARIO-style)

This folder provides a **scenario-based validation** workflow:
- Choose a **vehicle config** (small / full-size).
- Choose a **scenario** defined in a minimal **OpenSCENARIO (`.xosc`) parameter style**.
- Automatically generate a **synthetic road occupancy map** (PGM) for common road alignments.
- Run **planner + NMPC** on multiple **road friction conditions** and summarize success/metrics.

### What “OpenSCENARIO-style” means here

We do **not** implement the full OpenSCENARIO specification.
Instead, we use the standard OpenSCENARIO XML container and read only:
- `ParameterDeclarations/ParameterDeclaration` (name + value)

This keeps scenarios easy to edit while staying compatible with `.xosc` files.

### Quick start

```bash
cd "/Users/mojyx/Documents/GA/AV Research/ru-racer"
python3 cpp/scenario_validation/run_openscenario_validation.py \
  --scenario cpp/scenario_validation/scenarios/straight.xosc \
  --vehicle-cfg cpp/data/fullsize_car.cfg
```

Outputs:
- A run folder under `cpp/results/scenario_validation/run_*/`
- Per-road-condition runs (each has `rrt_nodes.csv`, `rrt_ref_traj.csv`, `nmpc_exec.csv`, and visuals)
- `scenario_summary.csv` + summary plots

### Road shapes supported
- `tangent` (straight)
- `horizontal_curve` (single arc)
- `s_curve` (reverse curves)
- `hairpin` (tight U-turn arc)
- `roundabout` (circular loop)

### Road conditions supported
Configured in the `.xosc` parameter `road_conditions` (comma-separated).
Defaults map to a friction scale (applied via `friction_mu_scale_{f,r}0`):
- `dry` ≈ 1.0
- `wet` ≈ 0.7
- `snow` ≈ 0.4
- `ice` ≈ 0.2
- `gravel` ≈ 0.5

