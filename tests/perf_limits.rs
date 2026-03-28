/// Performance regression guards for the `avian_fdm` hot path.
///
/// These tests catch catastrophic regressions (e.g. an O(n^2) lookup) in
/// ordinary `cargo test` runs. Limits are ~40x the expected times on an
/// AMD Ryzen 9 7950X so that slow CI runners pass without false positives.
///
/// Run with timings printed:
///   cargo test --features "presets,propulsion" --test perf_limits -- --nocapture
use std::hint::black_box;
use std::time::Instant;

use avian_fdm::components::aero_coeff::AeroCoeff;

// ── Shared test data ──────────────────────────────────────────────────────────

const ALPHA_BP: [f64; 14] = [
    -1.5700, -0.3491, -0.2443, -0.1745, -0.0873, 0.0000, 0.0873, 0.1309, 0.1745, 0.2182, 0.2618,
    0.3054, 0.3491, 1.5700,
];
const RE_BP: [f64; 2] = [1_668_183.0, 3_707_224.0];
const CL_DATA: [f64; 28] = [
    0.0000, 0.0000, -0.0085, -0.5085, -0.5085, -0.8136, -0.5085, -0.5085, 0.1017, 0.1017,
    0.5339, 0.5339, 1.2204, 1.2204, 1.4746, 1.4746, 1.5000, 1.6272, 1.6201, 1.7797, 1.5645,
    1.8306, 1.4272, 1.6272, 1.3138, 1.4238, 0.0000, 0.0000,
];

/// Returns the median elapsed time in nanoseconds over `n` calls to `f`.
fn median_ns<F: Fn()>(n: usize, f: F) -> u64 {
    let mut samples: Vec<u64> = (0..n)
        .map(|_| {
            let t = Instant::now();
            f();
            t.elapsed().as_nanos() as u64
        })
        .collect();
    samples.sort_unstable();
    samples[n / 2]
}

/// Prints the timing and asserts it is below `limit_ns`.
fn check(name: &str, ns: u64, limit_ns: u64) {
    if ns < 1_000 {
        println!("{name:<40} {ns:>6} ns   (limit {limit_ns} ns)");
    } else {
        println!(
            "{name:<40} {ns:>6} ns = {:.2} us  (limit {} ns)",
            ns as f64 / 1_000.0,
            limit_ns
        );
    }
    assert!(ns < limit_ns, "{name}: {ns} ns >= {limit_ns} ns limit");
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[test]
fn scalar_lookup_under_100ns() {
    let coeff = AeroCoeff::Scalar(0.53);
    let ns = median_ns(1_000, || {
        black_box(coeff.evaluate(black_box(0.087), black_box(2_000_000.0)));
    });
    check("scalar_lookup", ns, 100);
}

#[test]
fn table1d_lookup_under_300ns() {
    let coeff = AeroCoeff::Table1D {
        breakpoints: ALPHA_BP.to_vec(),
        values: CL_DATA.iter().step_by(2).copied().collect(),
    };
    let ns = median_ns(1_000, || {
        black_box(coeff.evaluate(black_box(0.087), black_box(0.0)));
    });
    check("table1d_lookup_14pt", ns, 300);
}

#[test]
fn table2d_lookup_under_500ns() {
    let coeff = AeroCoeff::Table2D {
        rows: ALPHA_BP.to_vec(),
        cols: RE_BP.to_vec(),
        data: CL_DATA.to_vec(),
    };
    let ns = median_ns(1_000, || {
        black_box(coeff.evaluate(black_box(0.087), black_box(2_000_000.0)));
    });
    check("table2d_lookup_14x2", ns, 500);
}

#[test]
fn aggregate_zones_15_under_5000ns() {
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

    let ns = median_ns(1_000, || {
        let sum: f64 = tables
            .iter()
            .map(|t| t.evaluate(black_box(0.087), black_box(2_000_000.0)))
            .sum();
        black_box(sum);
    });
    check("aggregate_zones_15", ns, 5_000);
}

#[test]
fn aggregate_zones_15x100_under_500000ns() {
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
    let states: Vec<(f64, f64)> = (0..100)
        .map(|i| (0.05 + i as f64 * 0.001, 1_500_000.0 + i as f64 * 5_000.0))
        .collect();

    let ns = median_ns(100, || {
        let mut total = 0.0_f64;
        for (alpha, re) in &states {
            for t in &tables {
                total += t.evaluate(black_box(*alpha), black_box(*re));
            }
        }
        black_box(total);
    });
    check("aggregate_zones_15x100_aircraft", ns, 500_000);
}

