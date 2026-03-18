// Criterion benchmarks for the FDM hot path.
// Run with: cargo bench
//
// Benchmarks (to be implemented in the benchmarks todo):
//   bench_table2d_lookup          — single bilinear interpolation on a 12×2 table
//   bench_aggregate_zones_15      — aggregate_zones on 1 aircraft × 15 zones
//   bench_aggregate_zones_15x100  — aggregate_zones on 100 aircraft × 15 zones
//   bench_aerodynamics_pipeline   — full force/moment pipeline for 1 aircraft
//   bench_atmosphere              — ISA evaluation at one altitude

use criterion::{criterion_group, criterion_main};

// TODO(benchmarks): implement benchmarks
criterion_group!(benches,);
criterion_main!(benches);
