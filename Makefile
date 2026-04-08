.PHONY: check
check:
	# f32 (default) workspace-wide, then f64 core-only (mutually exclusive backends).
	cargo check --workspace
	cargo check -p avian_fdm --no-default-features --features "f64"

.PHONY: clippy
clippy:
	cargo clippy --workspace
	cargo clippy -p avian_fdm --no-default-features --features "f64"

.PHONY: test
test:
	# f32 (default) workspace-wide, then f64 core-only.
	cargo test --workspace
	cargo test -p avian_fdm --no-default-features --features "f64"
	
.PHONY: fmt
fmt:
	cargo fmt

.PHONY: doc
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc -p avian_fdm --no-deps
	RUSTDOCFLAGS="-D warnings" cargo doc -p avian_fdm_j3cub_jsbsim --no-deps

.PHONY: build
build:
	cargo build --workspace --release

.PHONY: jsbsim
jsbsim:
	cargo test -p avian_fdm_j3cub_jsbsim -- jsbsim --nocapture

.PHONY: bench
bench:
	cargo test -p avian_fdm --test perf_limits -- --nocapture

.PHONY: ci
ci: check clippy test doc build
