# C++ Planning + Tracking Pipeline (`ru_racer_cpp`)

This folder adds a **self-contained C++ baseline** (no external dependencies) that:

- Plans a **time-cost RRT\*** trajectory using the 6‑state dynamic model from `Bike.m`
- Tracks the planned trajectory with a lightweight **NMPC-style** shooting optimizer
- Saves **CSV** outputs suitable for future research reuse

## Build

From the repo root:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
```

## Run

```bash
./cpp/build/ru_racer --config cpp/data/default.cfg
```

Results are written to `cpp/results/run_YYYYMMDD_HHMMSS/`.

## Inputs

Configuration is a simple `key=value` file (see `cpp/data/default.cfg`).

- **Obstacle circles (optional)**: `obstacles_csv=...` with rows `x,y,r`
- **Occupancy grid (optional)**: `map_pgm=...` using **PGM** (P2 or P5)
  - World→pixel mapping parameters are configurable:
    - `map_pixels_per_meter`
    - `map_origin_x_px`
    - `map_origin_y_px`
    - `map_free_threshold`

## Outputs (CSV)

- `rrt_nodes.csv`: full tree (node states, parents, costs, inputs)
- `rrt_path.csv`: best path (node sequence)
- `rrt_ref_traj.csv`: dt-resolution reference trajectory derived from the path
- `nmpc_exec.csv`: executed trajectory + control + reference at each timestep (if `run_nmpc=true`)

## Visualize the explored tree

After a run, plot the explored RRT* tree (and best path if `rrt_path.csv` exists):

```bash
python3 cpp/scripts/plot_rrt_tree.py \
  --nodes cpp/results/<run_folder>/rrt_nodes.csv \
  --out cpp/results/<run_folder>/rrt_tree.png
```

Tips:
- Use `--stride 5` or `--max-edges 20000` for faster plots on large trees.

### Plot on the real track image (`track.jpg`)

This uses the same world→pixel mapping as the MATLAB `Implot2d()` overlay:

```bash
python3 cpp/scripts/plot_rrt_tree.py \
  --nodes cpp/results/<run_folder>/rrt_nodes.csv \
  --bg track.jpg \
  --bg-transform track \
  --out cpp/results/<run_folder>/rrt_tree_on_track.png
```
