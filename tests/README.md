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

- Altitude: 300 m AGL
- True airspeed: 27 m/s
- Attitude: level flight

Two scenarios are covered.

**Powered flight** - throttle ramps from 50% to 75% over 12.5 s, then
holds. Exercises the full propulsion and aero pipeline.

**Glide (engine off)** - throttle stays at 0, engine is never started in
JSBSim. On the avian_fdm side the engine zone is disabled via
`Failure { remaining: 0.0 }`, which exercises the damage model path.
The J3Cub descends from 300 m to about 179 m over 60 s.

### Running the comparisons (no JSBSim required)

The JSBSim reference data is committed as CSV files, so both tests run
without Python or JSBSim installed:

```sh
# Powered flight
cargo test --features presets -- jsbsim_j3cub_comparison --nocapture

# Glide (engine off)
cargo test --features presets -- jsbsim_j3cub_glide_comparison --nocapture

# Both at once
cargo test --test jsbsim_comparison --features presets -- --nocapture
```

Each test runs avian_fdm headlessly for 60 s (about 10 s wall time) and
prints a side-by-side table comparing altitude, TAS, and AoA against the
reference.

### Verifying the reference data (requires JSBSim)

Two freshness-check tests re-run the Python JSBSim scripts and assert
that output still matches the committed CSVs:

```sh
# One-time setup
python3 -m venv .venv
.venv/bin/pip install jsbsim

# Powered flight freshness check
JSBSIM_DATA_PATH=../jsbsim cargo test --features presets \
    -- jsbsim_regenerate_reference --nocapture

# Glide freshness check
JSBSIM_DATA_PATH=../jsbsim cargo test --features presets \
    -- jsbsim_regenerate_glide_reference --nocapture
```

### Regenerating the reference CSVs

If you change initial conditions, the throttle schedule, or the JSBSim
model, regenerate the affected fixture and commit it:

```sh
# Powered flight
JSBSIM_DATA_PATH=../jsbsim .venv/bin/python3 tests/run_jsbsim.py \
    2>/dev/null | grep -E '^[0-9t]' > tests/fixtures/jsbsim_j3cub_60s.csv

# Glide (engine off)
JSBSIM_DATA_PATH=../jsbsim .venv/bin/python3 tests/run_jsbsim_glide.py \
    2>/dev/null | grep -E '^[0-9t]' > tests/fixtures/jsbsim_j3cub_glide_60s.csv
```

### Files

- `jsbsim_comparison.rs` - Rust integration tests; loads CSVs, runs avian_fdm, compares
- `run_jsbsim.py` - JSBSim powered-flight script; outputs CSV to stdout
- `run_jsbsim_glide.py` - JSBSim glide (engine off) script; outputs CSV to stdout
- `fixtures/jsbsim_j3cub_60s.csv` - committed powered-flight reference (120 samples)
- `fixtures/jsbsim_j3cub_glide_60s.csv` - committed glide reference (120 samples)

### CI

The GitHub Actions workflow runs all four tests on pushes to `main`:

```yaml
# .github/workflows/ci.yml -- jsbsim-validate job
JSBSIM_DATA_PATH=./jsbsim cargo test --features presets -- jsbsim --nocapture
```

The job has `continue-on-error: true` until precision tolerances are met.
