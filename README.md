[![License: LGPL-3.0+](https://img.shields.io/badge/license-LGPL--3+-blue.svg)](LICENSE)
[![ci](https://github.com/viccuad/avian_fdm/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/viccuad/avian_fdm/actions/workflows/ci.yml)

# avian_fdm

**Avian FDM** is 6-DoF Flight Dynamics Model plugin for
[Bevy](https://bevyengine.org/) + [Avian](https://crates.io/crates/avian3d).

---

## Example

Small recording of the provided J-3 Cub visual example.
First a controlled turn to starboard, then a harsh climb into a high AoA stall,
departing, diving, and recovering.

https://github.com/user-attachments/assets/529451a4-5d66-4297-af42-f22bafc50001

## Design

`avian_fdm` turns an Avian rigid-body hierarchy into a physically plausible
aircraft. You build your aircraft by assembling `AeroZone` entities around
an `AircraftCoreBundle` root.

Each physics step, the `avian_fdm` plugin evaluates aerodynamic and propulsive
forces for every `AeroZone` and accumulates them into Avian's
`ConstantForce`/`ConstantTorque` on the root body. Avian's integrator then
advances the state forward, `avian_fdm` never writes to position, velocity, or
orientation directly.

Mass, centre of gravity, and the full inertia tensor are computed automatically
by Avian from `ColliderDensity` on each child collider. In addition, several
emergent behaviors appear when computing the AeroZones together.

## Features

- ISA atmosphere (0-20 km) with density, pressure, temperature, viscosity
- Per-zone lift, drag, and side-force from tabulated coefficients (1D or 2D)
  - Reynolds-number-dependent coefficient lookup
  - Post-stall aerodynamics via Viterna-Corrigan extrapolation
- Pitch, roll, and yaw damping (per-zone or whole-aircraft LOD fallback)
- Induced drag (Oswald span efficiency)
- Zone-based damage/failure model with graceful degradation
- Automatic mass, CG, and inertia tensor from collider geometry
- Debug gizmo overlays for forces, moments, zones, and colliders
- Supports both avian3d f32 and f64 backends

### Emergent behavior

The zone-based architecture produces physically correct behaviors without
explicit global coefficients. Forces and moment arms are computed at each
zone's position, and Avian recomputes mass/CG/inertia from colliders. 
The following appear naturally: Stall, wing drop, snap roll, Spin autorotation,
Dutch roll, phugoid, short-period, spiral mode, Adverse yaw, Control authority,
Damage effects, and others.

See the lib.rs documentation for the complete list of emergent behaviors.

## Quick start

Add dependencies to `Cargo.toml`:

```toml
[dependencies]
avian_fdm              = "0.1"
avian_fdm_j3cub_jsbsim = "0.1"
avian3d                = "0.6"
bevy                   = "0.18"
```

Spawn the reference J-3 Cub aircraft:

```rust,ignore
use avian_fdm::prelude::*;
use avian_fdm_j3cub_jsbsim::presets::j3cub;
use avian3d::prelude::{LinearVelocity, PhysicsPlugins};
use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin::default())
        .add_systems(Startup, spawn)
        .run();
}

fn spawn(mut commands: Commands) {
    let root = j3cub::spawn(
        &mut commands,
        Transform::from_xyz(0.0, 300.0, 0.0),
    );
    commands.entity(root).insert(LinearVelocity(Vec3::new(27.0, 0.0, 0.0)));
}
```


## Feature flags

| Feature | Default | Description |
|---|---|---|
| `f32` | yes | Avian3d f32 physics backend (standard Bevy precision) |
| `f64` | -- | Avian3d f64 physics backend (mutually exclusive with f32) |
| `debug-plugin` | yes | Bevy Gizmo overlays for forces, moments, and zones |

Exactly one of `f32` or `f64` must be enabled. They are mutually exclusive.
For f64, use `default-features = false, features = ["f64"]`.

The JSBSim-derived J-3 Cub reference aircraft is in the separate
`avian_fdm_j3cub_jsbsim` crate (GPL-3.0-only, due to JSBSim data provenance).

## Version compatibility

| avian_fdm | Bevy | avian3d |
|---|---|---|
| 0.1 | 0.18 | 0.6 |

## License

`avian_fdm` is licensed under [LGPL-3.0-or-later](LICENSE).

`avian_fdm_j3cub_jsbsim` is licensed under GPL-3.0-only (JSBSim-derived data).
