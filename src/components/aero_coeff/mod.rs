//! [`AeroCoeff`], aerodynamic coefficient storage and lookup.
//!
//! Aerodynamic coefficients are dimensionless numbers that describe how much
//! lift, drag, or other force an airfoil produces at a given flight condition.
//! They vary with angle of attack (the angle between the wing chord and the
//! oncoming air) and with Reynolds number (a dimensionless ratio of inertial
//! to viscous forces that determines whether airflow is smooth or turbulent).
//!
//! An aerodynamic coefficient (e.g: CL, CD) can be stored as a constant, a 1-D
//! table over angle of attack, or a 2-D table over angle of attack and Reynolds
//! number.
//!
//! ## Stability derivatives
//!
//! Real aerodynamic coefficients are nonlinear functions of many variables.
//! The *stability derivative method* approximates them as a Taylor expansion
//! around a trim condition.
//! See: stability derivatives, aerodynamic Taylor series.

//! **Lift coefficient = baseline value + lift slope ×
//! angle-of-attack + pitch-rate correction term.
//!
//! ```text
//! CL(alpha, Re) ≈ CL₀ + CL_alpha * alpha + CL_q * (q & c̄/2V) + …
//! ```
//! For a high-fidelity simulation, pre-computed tables of CL vs alpha
//! (at several Re valyues) are more accurate than linear approximations,
//! especially near stall (the flight regime where the wing abruptly loses lift).
//!
//! ## Table storage layout
//!
//! `Table2D::data` is a **flat, row-major `Vec<S>`** of length
//! `rows.len() * cols.len()`. Element at row index `i`, column index `j`
//! is accessed as `data[i * cols.len() + j]`. This layout uses a single
//! heap allocation and maximises cache locality during bilinear
//! interpolation.

mod interpolation;
mod types;
mod validation;
mod viterna;

pub use types::AeroCoeff;
