//! [`AeroCoeff`]: storage and evaluation of aerodynamic coefficients.
//!
//! Aerodynamic coefficients are dimensionless numbers. They tell how much
//! lift, drag, or other force an airfoil makes at a given flight condition.
//! They change with the angle of attack (the angle between the wing chord and
//! the oncoming air). They also change with the Reynolds number. The Reynolds
//! number is a dimensionless ratio of inertial forces to viscous forces. It
//! tells if the airflow is smooth or turbulent.
//!
//! An aerodynamic coefficient (for example, CL or CD) has one of three forms:
//! a constant, a 1-D table over the angle of attack, or a 2-D table over the
//! angle of attack and the Reynolds number.
//!
//! ## Stability derivatives
//!
//! Real aerodynamic coefficients are nonlinear functions of many variables.
//! The *stability derivative method* gives an approximation of them: a Taylor
//! expansion around a trim condition. Related topics: stability derivatives,
//! aerodynamic Taylor series.
//!
//! **Lift coefficient = base value + lift slope × angle of attack +
//! pitch-rate correction term:**
//!
//! ```text
//! CL(alpha, Re) ≈ CL_0 + CL_alpha * alpha + CL_q * (q * c_bar / 2V) + …
//! ```
//!
//! For a high-detail simulation, tables of CL against alpha (at several Re
//! values) are more accurate than linear approximations. This is especially
//! true near the stall, where the wing suddenly loses lift.
//!
//! ## Table storage layout
//!
//! `Table2D::data` is a **flat, row-major `Vec<f32>`**. Its length is
//! `rows.len() * cols.len()`. The element at row index `i` and column index
//! `j` is `data[i * cols.len() + j]`. This layout uses one heap allocation.
//! It also gives good cache locality during bilinear interpolation.

mod interpolation;
mod types;
mod validation;
mod viterna;

pub use types::AeroCoeff;
