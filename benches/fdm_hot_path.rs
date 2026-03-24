/// Criterion benchmarks for the `avian_fdm` hot path.
///
/// Run with:  `cargo bench`
/// View HTML:  `open target/criterion/report/index.html`
///
/// Benchmarks target the tight inner loop that executes every physics frame
/// per zone. Even at 200 Hz physics with 100 aircraft × 15 zones = 300 000
/// zone evaluations per second, each benchmark should complete in < 500 ns.
use criterion::{criterion_group, criterion_main, Criterion};
use std::hint::black_box;

use avian_fdm::components::aero_coeff::AeroCoeff;
use avian_fdm::atmosphere::{atmosphere_at, isa_density, isa_temperature};

// ── Shared test data ──────────────────────────────────────────────────────────

/// Alpha breakpoints (14 points, matches J3Cub USA-35B table).
const ALPHA_BP: [f64; 14] = [
    -1.5700, -0.3491, -0.2443, -0.1745, -0.0873,
     0.0000,  0.0873,  0.1309,  0.1745,  0.2182,
     0.2618,  0.3054,  0.3491,  1.5700,
];

/// Reynolds breakpoints.
const RE_BP: [f64; 2] = [1_668_183.0, 3_707_224.0];

/// Whole-aircraft J3Cub CL data (row-major, 14×2).
const CL_DATA: [f64; 28] = [
     0.0000,  0.0000,
    -0.0085, -0.5085,
    -0.5085, -0.8136,
    -0.5085, -0.5085,
     0.1017,  0.1017,
     0.5339,  0.5339,
     1.2204,  1.2204,
     1.4746,  1.4746,
     1.5000,  1.6272,
     1.6201,  1.7797,
     1.5645,  1.8306,
     1.4272,  1.6272,
     1.3138,  1.4238,
     0.0000,  0.0000,
];

fn make_table2d() -> AeroCoeff {
    AeroCoeff::Table2D {
        rows: ALPHA_BP.to_vec(),
        cols: RE_BP.to_vec(),
        data: CL_DATA.to_vec(),
    }
}

fn make_table1d() -> AeroCoeff {
    AeroCoeff::Table1D {
        breakpoints: ALPHA_BP.to_vec(),
        values: CL_DATA.iter().step_by(2).copied().collect(), // Re=1.7M column
    }
}

// ── Benchmarks ────────────────────────────────────────────────────────────────

/// Single bilinear lookup on the 14×2 J3Cub CL table.
/// This is the per-zone coefficient evaluation in `compute_zone_forces`.
fn bench_table2d_lookup(c: &mut Criterion) {
    let coeff = make_table2d();
    c.bench_function("table2d_lookup_14x2", |b| {
        b.iter(|| {
            coeff.evaluate(black_box(0.087), black_box(2_000_000.0))
        })
    });
}

/// Single linear lookup on the 14-point 1-D table.
fn bench_table1d_lookup(c: &mut Criterion) {
    let coeff = make_table1d();
    c.bench_function("table1d_lookup_14pt", |b| {
        b.iter(|| {
            coeff.evaluate(black_box(0.087), black_box(0.0))
        })
    });
}

/// Scalar coefficient evaluation — baseline for comparison.
fn bench_scalar_lookup(c: &mut Criterion) {
    let coeff = AeroCoeff::Scalar(0.53);
    c.bench_function("scalar_lookup", |b| {
        b.iter(|| {
            coeff.evaluate(black_box(0.087), black_box(2_000_000.0))
        })
    });
}

/// ISA atmosphere evaluation at a single altitude.
/// Called once per aircraft per physics frame in `update_atmosphere`.
fn bench_atmosphere_at(c: &mut Criterion) {
    c.bench_function("atmosphere_at_3000m", |b| {
        b.iter(|| atmosphere_at(black_box(3_000.0)))
    });
}

/// ISA temperature + density only (simpler path used in fast-path variants).
fn bench_isa_temperature_density(c: &mut Criterion) {
    c.bench_function("isa_temp_density_3000m", |b| {
        b.iter(|| {
            let t = isa_temperature(black_box(3_000.0));
            let rho = isa_density(black_box(3_000.0));
            black_box((t, rho))
        })
    });
}

/// Aggregate 15 zone CL lookups — simulates `compute_zone_forces` for one aircraft.
/// Uses the full 14×2 J3Cub CL table scaled by different area fractions.
fn bench_aggregate_zones_15(c: &mut Criterion) {
    let fractions: [f64; 6] = [0.175, 0.175, 0.150, 0.175, 0.175, 0.150];
    let tables: Vec<AeroCoeff> = fractions
        .iter()
        .flat_map(|&f| {
            // wing + wing (6 tables × 2 per side would be 12; pad to 15 with small zones)
            [
                AeroCoeff::Table2D {
                    rows: ALPHA_BP.to_vec(),
                    cols: RE_BP.to_vec(),
                    data: CL_DATA.iter().map(|&v| v * f).collect(),
                },
                AeroCoeff::Scalar(0.464),  // aileron
                AeroCoeff::Scalar(-0.485), // elevator
            ]
        })
        .take(15)
        .collect();

    let alpha = 0.087_f64;
    let re = 2_000_000.0_f64;

    c.bench_function("aggregate_zones_15", |b| {
        b.iter(|| {
            let sum: f64 = tables.iter().map(|t| t.evaluate(black_box(alpha), black_box(re))).sum();
            black_box(sum)
        })
    });
}

/// 100 aircraft × 15 zones — simulates a busy multiplayer/AI scenario.
fn bench_aggregate_zones_15x100(c: &mut Criterion) {
    let fractions: [f64; 6] = [0.175, 0.175, 0.150, 0.175, 0.175, 0.150];
    let tables: Vec<AeroCoeff> = fractions
        .iter()
        .flat_map(|&f| {
            [
                AeroCoeff::Table2D {
                    rows: ALPHA_BP.to_vec(),
                    cols: RE_BP.to_vec(),
                    data: CL_DATA.iter().map(|&v| v * f).collect(),
                },
                AeroCoeff::Scalar(0.464),
                AeroCoeff::Scalar(-0.485),
            ]
        })
        .take(15)
        .collect();

    // 100 aircraft at slightly different states
    let states: Vec<(f64, f64)> = (0..100)
        .map(|i| (0.05 + i as f64 * 0.001, 1_500_000.0 + i as f64 * 5_000.0))
        .collect();

    c.bench_function("aggregate_zones_15x100_aircraft", |b| {
        b.iter(|| {
            let mut total = 0.0_f64;
            for (alpha, re) in &states {
                for t in &tables {
                    total += t.evaluate(black_box(*alpha), black_box(*re));
                }
            }
            black_box(total)
        })
    });
}

criterion_group!(
    benches,
    bench_table2d_lookup,
    bench_table1d_lookup,
    bench_scalar_lookup,
    bench_atmosphere_at,
    bench_isa_temperature_density,
    bench_aggregate_zones_15,
    bench_aggregate_zones_15x100,
);
criterion_main!(benches);
