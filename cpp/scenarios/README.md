## Open-space planner + NMPC scenarios (full-size car)

These scenarios run the full pipeline:
- **Planner**: RRT / RRT* / Spars-RRT / Spars-RRT*
- **Tracker**: NMPC following `rrt_ref_traj.csv`

They are designed for **open space** (no map, no obstacles unless specified).

### Build

```bash
cd "/Users/mojyx/Documents/GA/AV Research/ru-racer" && \
clang++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Icpp/include cpp/src/*.cpp -o cpp/build/ru_racer
```

### Run one scenario

```bash
./cpp/build/ru_racer --config cpp/scenarios/fullsize_setpoint_straight.cfg
```

Outputs go under `cpp/results/run_*/` and include:
- `rrt_nodes.csv`, `rrt_path.csv`
- `rrt_ref_traj.csv` (reference)
- `nmpc_exec.csv` (executed)
- `rrt_tree.png`, `rrt_speed.png` (auto-generated)

