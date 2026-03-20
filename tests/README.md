# Tests

## Unit tests

Run the standard unit-test suite (45 tests across all modules):

```sh
cargo test --all-features
```

Feature-gated subsets:

```sh
cargo test                              # default features (damage + propulsion)
cargo test --no-default-features        # core only
cargo test --features "damage,propulsion"
```

## JSBSim comparison fixture

Compares avian_fdm's J3Cub against JSBSim's reference J3Cub model over a
60-second flight. Both simulators use identical initial conditions:

| Parameter | Value |
|-----------|-------|
| Altitude | 300 m AGL |
| True airspeed | 27 m/s |
| Attitude | Level flight |
| Throttle | 50 % → 75 % (linear ramp over 12.5 s) |

### Running the comparison (no JSBSim required)

The JSBSim reference data is committed as a CSV file, so the comparison
test runs without Python or JSBSim installed:

```sh
cargo test --features presets -- jsbsim_j3cub_comparison --nocapture
```

This runs avian_fdm headlessly for 60 s (≈10 s wall time), then prints a
side-by-side table comparing altitude, TAS, and AoA against the reference.

### Verifying the reference data (requires JSBSim)

A second test re-runs the Python JSBSim script and checks that its output
still matches the committed CSV. This catches staleness if the JSBSim
model or initial conditions change.

```sh
# One-time setup: create a venv and install JSBSim
python3 -m venv .venv
.venv/bin/pip install jsbsim

# Run the freshness check
JSBSIM_DATA_PATH=../jsbsim cargo test --features presets \
    -- jsbsim_regenerate_reference --nocapture
```

### Regenerating the reference CSV

If you change the initial conditions, throttle schedule, or JSBSim model,
regenerate the reference file:

```sh
JSBSIM_DATA_PATH=../jsbsim .venv/bin/python3 tests/run_jsbsim.py \
    2>/dev/null | grep -E '^[0-9t]' > tests/fixtures/jsbsim_j3cub_60s.csv
```

Commit the updated CSV alongside your changes.

### Files

| File | Purpose |
|------|---------|
| `jsbsim_comparison.rs` | Rust integration test — loads CSV, runs avian_fdm, compares |
| `run_jsbsim.py` | Python script — runs JSBSim J3Cub, outputs CSV to stdout |
| `fixtures/jsbsim_j3cub_60s.csv` | Committed JSBSim reference data (120 samples) |

### CI

The GitHub Actions workflow runs both tests on pushes to `main`:

```yaml
# .github/workflows/ci.yml — jsbsim-validate job
JSBSIM_DATA_PATH=./jsbsim cargo test --features presets -- jsbsim --nocapture
```

The job has `continue-on-error: true` until precision tolerances are met.
