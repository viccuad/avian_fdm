# avian_fdm_j3cub_jsbsim

A [Piper J-3 Cub](https://en.wikipedia.org/wiki/Piper_J-3_Cub) aircraft preset for
[avian_fdm](https://crates.io/crates/avian_fdm), used as a reference aircraft
and validation fixture against JSBSim.

## Where the values come from

All aerodynamic coefficients are transcribed from the `J3Cub.xml` JSBSim model
from [github.com/wlbragg/J3Cub](https://github.com/wlbragg/J3Cub)
(USA-35B airfoil, Du Y stability derivatives). The following conversions were applied
throughout: ft² -> m², lb -> kg, slug·ft² -> kg·m², inches -> metres.

Coefficients not directly available in the XML (aileron roll authority, tail-derived
pitch/yaw stability) were back-calculated from JSBSim's stability derivatives using
standard formulas — the derivations are documented in `src/presets/j3cub.rs`.

Extraction of the preset and creation of the Colliders was aided by LLM.

## Validation

Pre-recorded JSBSim reference trajectories (60 s powered flight, 60 s glide) are committed
as CSV fixtures. The test suite runs `avian_fdm` under the same initial conditions and checks
altitude, airspeed, and angle-of-attack against the reference data.

To regenerate the fixtures you need JSBSim and the
[wlbragg/J3Cub](https://github.com/wlbragg/J3Cub) aircraft repo:

```sh
# Install JSBSim (one-time)
python3 -m venv .venv && .venv/bin/pip install jsbsim

# Powered flight
JSBSIM_DATA_PATH=../jsbsim .venv/bin/python3 tests/run_jsbsim.py \
    2>/dev/null | grep -E '^[0-9t]' > tests/fixtures/jsbsim_j3cub_60s.csv

# Glide (engine off)
JSBSIM_DATA_PATH=../jsbsim .venv/bin/python3 tests/run_jsbsim_glide.py \
    2>/dev/null | grep -E '^[0-9t]' > tests/fixtures/jsbsim_j3cub_glide_60s.csv
```

## Usage

```toml
[dependencies]
avian_fdm              = "0.1"
avian_fdm_j3cub_jsbsim = "0.1"
avian3d                = "0.6"
bevy                   = "0.18"
```

```rust,ignore
use avian_fdm::prelude::*;
use avian_fdm_j3cub_jsbsim::presets::j3cub;

fn spawn(mut commands: Commands) {
    let root = j3cub::spawn(
        &mut commands,
        Transform::from_xyz(0.0, 300.0, 0.0),
    );
    commands.entity(root).insert(LinearVelocity(Vec3::new(27.0, 0.0, 0.0)));
}
```

See the `examples/` directory for runnable demos.

## License

GPL-3.0-only — inherited from the JSBSim J3Cub aerodynamic data in
[github.com/wlbragg/J3Cub](github.com/wlbragg/J3Cub).
