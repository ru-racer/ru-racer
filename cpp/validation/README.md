## Validation & testing (open-loop)

This folder contains **open-loop** validation scenarios to sanity-check the vehicle model in **open space**.

### Run

Build:

```bash
cd "/Users/mojyx/Documents/GA/AV Research/ru-racer" && \
clang++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Icpp/include cpp/src/*.cpp -o cpp/build/ru_racer
```

Run validation (full-size config):

```bash
./cpp/build/ru_racer --config cpp/validation/fullsize_open_space.cfg
```

Outputs are written to:
- `cpp/results/run_*/validation/*.csv`
- `cpp/results/run_*/validation/open_loop_summary.png` (if `python3 + matplotlib` are available)

### Scenarios

Configured by `validate_scenarios` (comma-separated):
- `accel_straight`
- `brake_straight`
- `turn_constant_steer`
- `turn_sine_steer`
- `mu_drop_sine_steer` (demonstrates online friction updates during a run)

